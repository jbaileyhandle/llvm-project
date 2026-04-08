//===- RegisterTracker.cpp - Register pressure tracking -------------------===//
//
// Implementation of approximate register pressure tracking for scheduling.
//
// Current limitations:
//   - Only tracks virtual registers. Physical registers are excluded
//     because they are not SSA and their pressure contribution is mostly
//     fixed (e.g., VCC always costs 2 SGPRs regardless of scheduling).
//     Even LLVM's own RegPressureTracker has degraded physical register
//     tracking on GPU targets — LiveRange data may not be computed, and
//     the tracker falls back to safe defaults. Our virtual-only model is
//     a comparable approximation. If this proves too inaccurate, we can
//     add physical register tracking later.
//   - Group nodes (subgraphs) are not yet supported. We report a fatal
//     error if any are encountered. Handling group nodes will require
//     computing aggregate register effects at subgraph boundaries.
//
//===----------------------------------------------------------------------===//

#include "RegisterTracker.h"
#include "SIRegisterInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

RegisterTracker::RegisterTracker(ArrayRef<ScheduleNode *> nodes,
                                 const MachineRegisterInfo &mri,
                                 const TargetRegisterInfo &tri) {
  ExtractRegOps(nodes, mri, tri);
  BuildRegStates(mri, tri);
}

void RegisterTracker::ExtractRegOps(ArrayRef<ScheduleNode *> nodes,
                                    const MachineRegisterInfo &mri,
                                    const TargetRegisterInfo &tri) {
  for (ScheduleNode *node : nodes) {
    if (!node->IsLeaf()) {
      std::string msg = "RegisterTracker does not yet support group nodes "
                        "(subgraphs). Node: " + node->ToString();
      report_fatal_error(StringRef(msg));
    }

    // Read defs/uses from the node (populated during graph construction
    // by ScheduleNode::ExtractRegInfo or by CreateEntryAndExitNodes).
    if (node->RegDefs().empty() && node->RegUses().empty()) {
      continue;
    }

    InstrRegOps ops;
    for (const RegWithLaneMask &rm : node->RegDefs()) {
      ops.defs.push_back(rm.reg);
    }
    for (const RegWithLaneMask &rm : node->RegUses()) {
      ops.uses.push_back(rm.reg);
    }
    instr_reg_ops_[node] = std::move(ops);
  }
}

void RegisterTracker::BuildRegStates(const MachineRegisterInfo &mri,
                                     const TargetRegisterInfo &tri) {
  for (const auto &[node, ops] : instr_reg_ops_) {
    // Ensure all defined registers have an entry.
    for (Register reg : ops.defs) {
      if (!reg_states_.count(reg)) {
        reg_states_[reg] = RegState{
            ClassifyRegister(reg, mri, tri),
            ComputeWeight(reg, mri, tri),
            /*remaining_uses=*/0};
      }
    }

    // Count uses for each register.
    for (Register reg : ops.uses) {
      if (!reg_states_.count(reg)) {
        reg_states_[reg] = RegState{
            ClassifyRegister(reg, mri, tri),
            ComputeWeight(reg, mri, tri),
            /*remaining_uses=*/0};
      }
      reg_states_[reg].remaining_uses++;
    }
  }
}

void RegisterTracker::Schedule(const ScheduleNode *node) {
  // Save peak pressure before this step (for Unschedule to restore).
  std::array<int, kNumRegTypes> saved_peak;
  for (int i = 0; i < kNumRegTypes; ++i) {
    saved_peak[i] = peak_pressure_[i];
  }
  saved_peak_stack_.push_back(saved_peak);

  auto ops_it = instr_reg_ops_.find(node);
  if (ops_it == instr_reg_ops_.end()) {
    // Node has no register info (boundary node or test node).
    return;
  }

  const InstrRegOps &ops = ops_it->second;

  // Process uses first: the hardware reads inputs before writing outputs.
  // A register whose last use is consumed here dies before the defs
  // become live, reducing peak pressure at this point.
  for (Register reg : ops.uses) {
    RegState &state = reg_states_[reg];
    state.remaining_uses--;
    if (state.remaining_uses == 0) {
      // Last use — register dies. Remove from live set and subtract weight.
      live_regs_.erase(reg);
      pressure_[static_cast<int>(state.type)] -= state.weight;
    }
  }

  // Process defs: register becomes live only if not already in the live
  // set. This handles sub-register redefs (e.g., %114.sub0 then %114.sub1
  // — the second write should not add weight again because %114 is
  // already live from the first write).
  for (Register reg : ops.defs) {
    if (live_regs_.insert(reg).second) {
      // Newly live — add weight.
      RegState &state = reg_states_[reg];
      pressure_[static_cast<int>(state.type)] += state.weight;
    }
  }

  // Update peak pressure after both uses and defs are processed.
  for (int i = 0; i < kNumRegTypes; ++i) {
    if (pressure_[i] > peak_pressure_[i]) {
      peak_pressure_[i] = pressure_[i];
    }
  }
}

void RegisterTracker::Unschedule(const ScheduleNode *node) {
  auto ops_it = instr_reg_ops_.find(node);
  if (ops_it != instr_reg_ops_.end()) {
    const InstrRegOps &ops = ops_it->second;

    // Reverse operations in opposite order from Schedule.
    // Reverse defs: if this was the instruction that made the register
    // live (it wasn't already live before), remove it and subtract weight.
    for (Register reg : ops.defs) {
      // Check if this register would still be live without this def.
      // It's still live if it has remaining uses > 0 (someone else
      // defined it earlier and it hasn't died yet).
      // For simplicity, we remove from live set and subtract weight,
      // but only if remaining_uses == 0 (meaning no prior def made it
      // live — this was the originating def).
      // TODO: This is imprecise for sub-register redefs. Revisit.
      if (reg_states_[reg].remaining_uses == 0 && live_regs_.erase(reg)) {
        RegState &state = reg_states_[reg];
        pressure_[static_cast<int>(state.type)] -= state.weight;
      }
    }

    // Reverse uses: re-increment remaining use counts. If a register
    // was dead (remaining_uses was 0), it becomes live again.
    for (Register reg : ops.uses) {
      RegState &state = reg_states_[reg];
      if (state.remaining_uses == 0) {
        live_regs_.insert(reg);
        pressure_[static_cast<int>(state.type)] += state.weight;
      }
      state.remaining_uses++;
    }
  }

  // Restore saved peak pressure.
  assert(!saved_peak_stack_.empty() && "Unschedule without matching Schedule");
  std::array<int, kNumRegTypes> saved_peak = saved_peak_stack_.back();
  saved_peak_stack_.pop_back();
  for (int i = 0; i < kNumRegTypes; ++i) {
    peak_pressure_[i] = saved_peak[i];
  }
}

static const char *RegTypeName(RegType type) {
  switch (type) {
  case RegType::kSGPR: return "SGPR";
  case RegType::kVGPR: return "VGPR";
  case RegType::kAGPR: return "AGPR";
  default: return "???";
  }
}

std::string RegisterTracker::DescribeRegOps(const ScheduleNode *node) const {
  auto ops_it = instr_reg_ops_.find(node);
  if (ops_it == instr_reg_ops_.end()) {
    return "(no register ops)";
  }

  const InstrRegOps &ops = ops_it->second;
  std::string result;

  if (!ops.defs.empty()) {
    result += "defs: ";
    for (Register reg : ops.defs) {
      auto state_it = reg_states_.find(reg);
      if (state_it != reg_states_.end()) {
        const RegState &state = state_it->second;
        result += "%" + std::to_string(Register::virtReg2Index(reg)) +
                  "(" + RegTypeName(state.type) +
                  " w" + std::to_string(state.weight) + ") ";
      }
    }
  }

  if (!ops.uses.empty()) {
    result += "uses: ";
    for (Register reg : ops.uses) {
      auto state_it = reg_states_.find(reg);
      if (state_it != reg_states_.end()) {
        const RegState &state = state_it->second;
        result += "%" + std::to_string(Register::virtReg2Index(reg)) +
                  "(" + RegTypeName(state.type) +
                  " w" + std::to_string(state.weight) +
                  " rem=" + std::to_string(state.remaining_uses);
        if (state.remaining_uses == 0) {
          result += " DIES";
        }
        result += ") ";
      }
    }
  }

  return result;
}

RegType RegisterTracker::ClassifyRegister(Register reg,
                                          const MachineRegisterInfo &mri,
                                          const TargetRegisterInfo &tri) {
  assert(reg.isVirtual() && "Only virtual registers should be classified");
  const TargetRegisterClass *rc = mri.getRegClass(reg);
  const SIRegisterInfo &si_tri = static_cast<const SIRegisterInfo &>(tri);

  if (si_tri.isSGPRClass(rc)) {
    return RegType::kSGPR;
  }
  if (si_tri.isAGPRClass(rc)) {
    return RegType::kAGPR;
  }
  // Default to VGPR for other register classes on AMDGPU.
  return RegType::kVGPR;
}

int RegisterTracker::ComputeWeight(Register reg,
                                   const MachineRegisterInfo &mri,
                                   const TargetRegisterInfo &tri) {
  assert(reg.isVirtual() && "Only virtual registers should be weighted");
  const TargetRegisterClass *rc = mri.getRegClass(reg);
  // Weight = number of 32-bit physical registers this virtual register
  // occupies. A VGPR_32 has weight 1, a VREG_64 has weight 2, an
  // SGPR_128 has weight 4, etc.
  int size_in_bits = tri.getRegSizeInBits(*rc);
  return size_in_bits / 32;
}
