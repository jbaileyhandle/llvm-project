//===- RegisterTracker.h - Register pressure tracking -----------*- C++ -*-===//
//
// Tracks register pressure during incremental schedule construction.
// Register defs/uses are extracted from LLVM MachineInstrs once at
// construction. Pressure is updated incrementally via Schedule/Unschedule
// as instructions are added/removed from the schedule.
//
// Only tracks virtual registers. Physical registers ($vcc, $exec, etc.)
// are excluded because:
//   - They contribute a fixed overhead to occupancy regardless of
//     scheduling (e.g., VCC always adds 2 SGPRs).
//   - They are not in SSA form, which breaks our use-counting model.
//   - This matches OptSched's GCN wrapper, which also tracks only
//     virtual registers.
// The fixed physical register overhead should be added when converting
// peak pressure to occupancy, but is not relevant during search.
//
// This is an approximate model used during scheduling search. The winning
// schedule should be verified with LLVM's accurate RegPressureTracker
// before committing.
//
// Supports both copying (for beam search) and undo (for BnB) via the
// saved peak pressure stack.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_REGISTERTRACKER_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_REGISTERTRACKER_H

#include "ScheduleGraph.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/CodeGen/Register.h"
#include <vector>

namespace llvm {

class MachineRegisterInfo;
class TargetRegisterInfo;

namespace hierarchical_scheduler {

/// Register type classification for pressure tracking. Occupancy on
/// AMDGPU is determined independently per type — peak VGPRs limit VGPR
/// occupancy, peak SGPRs limit SGPR occupancy, and actual occupancy is
/// the minimum.
enum class RegType {
  kSGPR,
  kVGPR,
  kAGPR,
  kNumTypes
};

/// Tracks register pressure during incremental schedule construction.
class RegisterTracker {
public:
  /// Initialize from a set of ScheduleNodes. Extracts register defs/uses
  /// from each scheduling-unit node's MachineInstr (skipping subgraph
  /// proxies and nodes without MachineInstrs). Classifies virtual
  /// registers by type (SGPR, VGPR, AGPR), computes weights (number
  /// of 32-bit physical registers each virtual register occupies),
  /// and counts total uses per register.
  RegisterTracker(ArrayRef<ScheduleNode *> nodes,
                  const MachineRegisterInfo &mri,
                  const TargetRegisterInfo &tri);

  /// Update pressure after scheduling an instruction.
  /// For each def: register becomes live, add weight to pressure.
  /// For each use: decrement remaining uses. If zero, register dies,
  /// subtract weight from pressure.
  /// Saves peak pressure for Unschedule() to restore.
  void Schedule(const ScheduleNode *node);

  /// Reverse the last Schedule() call.
  void Unschedule(const ScheduleNode *node);

  /// Current pressure for a given register type.
  int GetCurrentRegisterPressure(RegType type) const {
    return pressure_[static_cast<int>(type)];
  }

  /// Peak pressure seen so far for a given register type.
  int GetPeakRegisterPressure(RegType type) const {
    return peak_pressure_[static_cast<int>(type)];
  }

  /// Human-readable description of the register effects of scheduling a
  /// node. Lists defs (registers becoming live) and uses (with remaining
  /// use count, and "DIES" if the register dies at this step).
  /// Does NOT modify state — this is a read-only query.
  std::string DescribeRegOps(const ScheduleNode *node) const;

private:
  static constexpr int kNumRegTypes = static_cast<int>(RegType::kNumTypes);

  /// Per-register tracking state, extracted once at construction.
  struct RegState {
    RegType type;
    int weight;          // Number of 32-bit physical regs this occupies.
    int remaining_uses;  // Decremented on schedule, incremented on unschedule.
  };

  /// Per-instruction register operands. Stored as register IDs — look up
  /// RegState in reg_states_ for type/weight/use info.
  struct InstrRegOps {
    SmallVector<Register> defs;
    SmallVector<Register> uses;
  };

  /// Map from LLVM virtual register ID to our tracking state.
  DenseMap<Register, RegState> reg_states_;

  /// Map from ScheduleNode to its register operands.
  DenseMap<const ScheduleNode *, InstrRegOps> instr_reg_ops_;

  /// Set of currently live registers. Used to avoid double-counting when
  /// a register is redefined (e.g., sub-register writes to %114.sub0 then
  /// %114.sub1 — the register is already live after the first write).
  DenseSet<Register> live_regs_;

  /// Current pressure per type (sum of weights of live registers).
  int pressure_[kNumRegTypes] = {};

  /// Peak pressure per type (running max of pressure_).
  int peak_pressure_[kNumRegTypes] = {};

  /// Saved peak pressure from before each Schedule() call. Peak is a
  /// running max that can't be decremented, so we save/restore it
  /// explicitly during Unschedule().
  ///
  /// Alternative: only push when the peak actually changes, and track a
  /// "did this step change the peak?" flag per step. This would save a
  /// few copies but adds branching logic. Not worth it unless profiling
  /// shows this is a bottleneck — each push is only kNumRegTypes ints.
  std::vector<std::array<int, kNumRegTypes>> saved_peak_stack_;

  // --- Construction helpers (called from constructor) ---

  /// Extract register defs/uses from each node's MachineInstr into
  /// instr_reg_ops_. Skips boundary nodes and physical registers.
  /// Reports fatal error for subgraph proxies (not yet supported).
  void ExtractRegOps(ArrayRef<ScheduleNode *> nodes,
                     const MachineRegisterInfo &mri,
                     const TargetRegisterInfo &tri);

  /// Build reg_states_ from instr_reg_ops_. Classifies each register,
  /// computes weight, and counts total uses.
  void BuildRegStates(const MachineRegisterInfo &mri,
                      const TargetRegisterInfo &tri);

  /// Classify a virtual register as SGPR, VGPR, or AGPR based on its
  /// register class.
  static RegType ClassifyRegister(Register reg,
                                  const MachineRegisterInfo &mri,
                                  const TargetRegisterInfo &tri);

  /// Compute the weight (number of 32-bit physical registers) for a
  /// virtual register based on its register class size.
  static int ComputeWeight(Register reg,
                           const MachineRegisterInfo &mri,
                           const TargetRegisterInfo &tri);
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_REGISTERTRACKER_H
