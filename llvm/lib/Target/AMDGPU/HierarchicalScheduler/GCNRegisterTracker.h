//===- GCNRegisterTracker.h - GCN register pressure tracking ----*- C++ -*-===//
//
// Tracks register pressure during incremental forward schedule construction,
// using LLVM's AMDGPU-native GCNRegPressure for accurate sub-register and
// tuple handling.
//
// Replaces RegisterTracker for new code. Fixes these issues from the old
// tracker:
//   - Sub-register double-counting: uses GCNRegPressure::inc() with lane
//     masks, so partial writes to the same register are tracked correctly.
//   - Dead defs: filtered during extraction (isDead() operands skipped).
//   - No tuple awareness: GCNRegPressure tracks tuple weights natively.
//   - Imprecise undo: delta-based undo records reverse each step exactly.
//
// Pressure model toggle:
//   kAMDGPU: add defs, take peak, remove dying uses.
//            Matches GCNDownwardRPTracker. Conservative: peak includes
//            both old live regs and new defs simultaneously.
//   kUsesFirst: remove dying uses, add defs, take peak.
//               Matches OptSched. Optimistic: assumes register allocator
//               can reuse a dying input's register for the output.
//   Currently fixed to kAMDGPU.
//
// Kill detection: whole-register granularity. A register dies when its
// last use (of any lane) is scheduled. Can overestimate pressure when
// sub-register uses finish at different times. Designed so upgrading to
// per-lane kill detection only changes remaining_uses_ and the kill
// logic in Schedule/Unschedule.
//
// Only tracks virtual registers. Physical registers contribute fixed
// overhead regardless of scheduling.
//
// Group nodes (subgraphs) are not yet supported. Fatal error if
// encountered during extraction. Future work: compute aggregate
// register effects at subgraph boundaries.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_GCNREGISTERTRACKER_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_GCNREGISTERTRACKER_H

#include "GCNRegPressure.h"
#include "ScheduleGraph.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/MC/LaneBitmask.h"
#include <string>
#include <vector>

namespace llvm {

class GCNSubtarget;
class LiveIntervals;
class MachineRegisterInfo;
class TargetRegisterInfo;

namespace hierarchical_scheduler {

/// Controls the order in which defs and dying uses are processed,
/// which affects where the pressure peak is recorded.
enum class PressureModel {
  /// Defs first, peak, then dying uses removed. Matches AMDGPU's own
  /// GCNDownwardRPTracker. More conservative: the peak includes both
  /// the instruction's new defs and everything already live.
  kAMDGPU,

  /// Dying uses removed first, then defs, then peak. Matches OptSched.
  /// More optimistic: dying inputs are freed before new defs are counted,
  /// so the peak is lower when an instruction's last-use inputs can have
  /// their physical registers reused for outputs.
  kUsesFirst,
};

class GCNRegisterTracker {
public:
  using LiveRegSet = GCNRPTracker::LiveRegSet;

  /// A register + lane mask pair. Used for pre-extracted defs and uses.
  struct RegMask {
    unsigned reg;
    LaneBitmask mask;
  };

  /// Pre-extracted register info for one node. Computed once at
  /// construction. Defs are deduplicated per register (sub-register
  /// defs of the same register have their masks ORed). Dead defs
  /// (isDead()) are excluded. Uses are similarly deduplicated.
  struct NodeRegInfo {
    SmallVector<RegMask, 4> defs;
    SmallVector<RegMask, 4> uses;
  };

  /// Construct from graph nodes. For leaf nodes with a MachineInstr,
  /// extracts defs/uses with lane masks. For entry/exit nodes (no MI),
  /// reads from ScheduleNode::RegDefs()/RegUses() with full lane masks.
  ///
  /// LiveIntervals is needed for accurate use-mask extraction on
  /// multi-lane registers.
  ///
  /// Reports fatal error if a group node (subgraph) is encountered.
  GCNRegisterTracker(ArrayRef<ScheduleNode *> nodes,
                     const MachineRegisterInfo &mri,
                     const TargetRegisterInfo &tri,
                     const LiveIntervals &lis);

  /// Update pressure after scheduling a node. Order depends on the
  /// pressure model (see PressureModel). Pushes an undo record.
  void Schedule(const ScheduleNode *node);

  /// Reverse the last Schedule() call.
  void Unschedule(const ScheduleNode *node);

  const GCNRegPressure &GetCurrentPressure() const { return cur_pressure_; }
  const GCNRegPressure &GetPeakPressure() const { return max_pressure_; }
  const LiveRegSet &GetLiveRegs() const { return live_regs_; }

  unsigned GetOccupancy(const GCNSubtarget &st) const {
    return max_pressure_.getOccupancy(st);
  }

  /// Human-readable description of a node's register effects.
  std::string DescribeRegOps(const ScheduleNode *node) const;

  /// Human-readable current/peak pressure summary.
  std::string DescribePressure() const;

private:
  static constexpr PressureModel kModel = PressureModel::kAMDGPU;

  // --- Undo record ---

  /// Saved state for one Schedule() call. Only records what changed,
  /// not the full tracker state.
  ///
  /// For each def: the register's lane mask BEFORE we added the def
  /// lanes. To undo: call inc(reg, current_mask, saved_prev_mask) to
  /// reverse the pressure change, then restore the live_regs_ entry.
  ///
  /// For each kill: the register's lane mask BEFORE it was erased from
  /// live_regs_. We know a register was killed during Unschedule by
  /// checking remaining_uses_[reg] == 0, but the lane mask was erased
  /// from live_regs_ and can't be recovered without saving it.
  ///
  /// For non-killing uses: nothing saved. remaining_uses_ is trivially
  /// reversed by incrementing. live_regs_ and cur_pressure_ are
  /// unchanged for non-killing uses.
  struct ScheduleStep {
    GCNRegPressure saved_max;
    SmallVector<std::pair<unsigned, LaneBitmask>, 4> def_prev_masks;
    SmallVector<std::pair<unsigned, LaneBitmask>, 4> kill_masks;
  };

  // --- Core state ---

  DenseMap<const ScheduleNode *, NodeRegInfo> node_reg_info_;
  DenseMap<unsigned, int> remaining_uses_;
  LiveRegSet live_regs_;
  GCNRegPressure cur_pressure_;
  GCNRegPressure max_pressure_;
  std::vector<ScheduleStep> undo_stack_;

  const MachineRegisterInfo *mri_;

  // --- Construction helpers ---

  void ExtractNodeRegInfo(ArrayRef<ScheduleNode *> nodes,
                          const MachineRegisterInfo &mri,
                          const TargetRegisterInfo &tri,
                          const LiveIntervals &lis);

  void InitRemainingUses();

  /// Warn if any node is a function call, since the callee's
  /// register usage is invisible to the scheduler.
  static void CheckForFunctionCalls(ArrayRef<ScheduleNode *> nodes);

  // --- Extraction helpers (per-node-type) ---

  /// Dedup helper: find reg in entries and OR in mask, or append.
  static void AddRegMask(SmallVectorImpl<RegMask> &entries,
                         unsigned reg, LaneBitmask mask);

  /// Extract from a leaf node with a MachineInstr.
  static void ExtractFromMachineInstr(const ScheduleNode *node,
                                      NodeRegInfo &info,
                                      const MachineRegisterInfo &mri,
                                      const LiveIntervals &lis);

  /// Extract from an entry/exit/test node (no MachineInstr).
  static void ExtractFromNodeRegLists(const ScheduleNode *node,
                                      NodeRegInfo &info,
                                      const MachineRegisterInfo &mri);

  /// Placeholder: fatal error for group nodes (not yet supported).
  static void ExtractFromGroupNode(const ScheduleNode *node);

  // --- Schedule helpers ---

  void ProcessDefs(const NodeRegInfo &info, ScheduleStep &step);
  void ProcessUses(const NodeRegInfo &info, ScheduleStep &step);
  void UndoDefs(const ScheduleStep &step);
  void UndoUses(const NodeRegInfo &info, const ScheduleStep &step);

  // --- Lane mask helpers ---
  // Reimplemented locally because GCNRegPressure.cpp defines these as
  // static (file-local) functions, not accessible from other files.

  static LaneBitmask GetDefMask(const MachineOperand &mo,
                                const MachineRegisterInfo &mri);

  static LaneBitmask GetUseMask(const MachineOperand &mo,
                                const MachineRegisterInfo &mri,
                                const LiveIntervals &lis);
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_GCNREGISTERTRACKER_H
