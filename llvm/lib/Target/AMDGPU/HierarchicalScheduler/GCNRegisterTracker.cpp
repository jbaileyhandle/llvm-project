//===- GCNRegisterTracker.cpp - GCN register pressure tracking ------------===//
//
// Implementation of GCNRegPressure-based register pressure tracking.
//
// See GCNRegisterTracker.h for design rationale and known limitations.
//
//===----------------------------------------------------------------------===//

#include "GCNRegisterTracker.h"
#include "SIRegisterInfo.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

// ============================================================================
// Lane mask helpers
// ============================================================================
// Local reimplementations of the static helpers in GCNRegPressure.cpp
// (getDefRegMask, getUsedRegMask). Same logic, same cases.

LaneBitmask GCNRegisterTracker::GetDefMask(const MachineOperand &mo,
                                           const MachineRegisterInfo &mri) {
  assert(mo.isDef() && mo.isReg() && mo.getReg().isVirtual());
  // Don't rely on read-undef flag — it may not be set correctly for
  // tentative schedules. See comment in GCNRegPressure.cpp.
  if (mo.getSubReg() == 0) {
    return mri.getMaxLaneMaskForVReg(mo.getReg());
  }
  return mri.getTargetRegisterInfo()->getSubRegIndexLaneMask(mo.getSubReg());
}

LaneBitmask GCNRegisterTracker::GetUseMask(const MachineOperand &mo,
                                           const MachineRegisterInfo &mri,
                                           const LiveIntervals &lis) {
  assert(mo.isUse() && mo.isReg() && mo.getReg().isVirtual());

  if (auto sub_reg = mo.getSubReg()) {
    return mri.getTargetRegisterInfo()->getSubRegIndexLaneMask(sub_reg);
  }

  auto max_mask = mri.getMaxLaneMaskForVReg(mo.getReg());
  if (SIRegisterInfo::getNumCoveredRegs(max_mask) > 1) {
    return max_mask;
  }

  // Single-lane register: query LiveIntervals for the actual live
  // mask at this instruction. Subreg defs can be reordered but all
  // must dominate uses, so the live lane mask is schedule-invariant.
  auto si = lis.getInstructionIndex(*mo.getParent()).getBaseIndex();
  return getLiveLaneMask(mo.getReg(), si, lis, mri);
}

// ============================================================================
// Extraction helpers
// ============================================================================

void GCNRegisterTracker::AddRegMask(SmallVectorImpl<RegMask> &entries,
                                    unsigned reg, LaneBitmask mask) {
  auto it = llvm::find_if(
      entries, [reg](const RegMask &rm) { return rm.reg == reg; });
  if (it != entries.end()) {
    it->mask |= mask;
  } else {
    entries.push_back({reg, mask});
  }
}

void GCNRegisterTracker::ExtractFromMachineInstr(
    const ScheduleNode *node,
    NodeRegInfo &info,
    const MachineRegisterInfo &mri,
    const LiveIntervals &lis) {
  const MachineInstr &mi = *node->GetSUnit()->getInstr();

  // Defs: skip non-virtual and dead.
  for (const MachineOperand &mo : mi.all_defs()) {
    if (!mo.getReg().isVirtual() || mo.isDead()) {
      continue;
    }
    AddRegMask(info.defs, mo.getReg().id(), GetDefMask(mo, mri));
  }

  // Uses: skip non-virtual. Also skip operands that are syntactically
  // uses but don't actually read the register (undef flag — appears
  // as an implicit use to keep the live range alive for the register
  // allocator, but no hardware read occurs).
  for (const MachineOperand &mo : mi.operands()) {
    if (!mo.isReg() || !mo.getReg().isVirtual()) {
      continue;
    }
    if (!mo.isUse() || !mo.readsReg()) {
      continue;
    }
    AddRegMask(info.uses, mo.getReg().id(), GetUseMask(mo, mri, lis));
  }
}

void GCNRegisterTracker::ExtractFromNodeRegLists(
    const ScheduleNode *node,
    NodeRegInfo &info,
    const MachineRegisterInfo &mri) {
  for (const RegWithLaneMask &rm : node->RegDefs()) {
    if (!rm.reg.isVirtual()) {
      continue;
    }
    AddRegMask(info.defs, rm.reg.id(), rm.mask);
  }
  for (const RegWithLaneMask &rm : node->RegUses()) {
    if (!rm.reg.isVirtual()) {
      continue;
    }
    AddRegMask(info.uses, rm.reg.id(), rm.mask);
  }
}

void GCNRegisterTracker::ExtractFromGroupNode(const ScheduleNode *node) {
  std::string msg = "GCNRegisterTracker does not yet support group nodes "
                    "(subgraphs). Node: " + node->ToString();
  report_fatal_error(StringRef(msg));
}

// ============================================================================
// Construction
// ============================================================================

GCNRegisterTracker::GCNRegisterTracker(ArrayRef<ScheduleNode *> nodes,
                                       const MachineRegisterInfo &mri,
                                       const TargetRegisterInfo &tri,
                                       const LiveIntervals &lis)
    : mri_(&mri) {
  ExtractNodeRegInfo(nodes, mri, tri, lis);
  InitRemainingUses();
}

void GCNRegisterTracker::ExtractNodeRegInfo(ArrayRef<ScheduleNode *> nodes,
                                            const MachineRegisterInfo &mri,
                                            const TargetRegisterInfo &tri,
                                            const LiveIntervals &lis) {
  for (ScheduleNode *node : nodes) {
    if (!node->IsLeaf()) {
      ExtractFromGroupNode(node);
      continue;
    }

    NodeRegInfo info;

    SUnit *su = node->GetSUnit();
    if (su && su->getInstr()) {
      ExtractFromMachineInstr(node, info, mri, lis);
    } else {
      ExtractFromNodeRegLists(node, info, mri);
    }

    if (!info.defs.empty() || !info.uses.empty()) {
      node_reg_info_[node] = std::move(info);
    }
  }
}

void GCNRegisterTracker::InitRemainingUses() {
  for (const auto &[node, info] : node_reg_info_) {
    for (const RegMask &use : info.uses) {
      remaining_uses_[use.reg]++;
    }
  }
}

// ============================================================================
// Schedule / Unschedule
// ============================================================================

void GCNRegisterTracker::ProcessDefs(const NodeRegInfo &info,
                                     ScheduleStep &step) {
  for (const RegMask &def : info.defs) {
    auto &live_mask = live_regs_[def.reg];
    LaneBitmask prev_mask = live_mask;
    step.def_prev_masks.push_back({def.reg, prev_mask});
    live_mask |= def.mask;
    cur_pressure_.inc(def.reg, prev_mask, live_mask, *mri_);
  }
}

void GCNRegisterTracker::ProcessUses(const NodeRegInfo &info,
                                     ScheduleStep &step) {
  for (const RegMask &use : info.uses) {
    int &uses_left = remaining_uses_[use.reg];
    uses_left--;
    if (uses_left == 0) {
      // Last use — register dies.
      auto it = live_regs_.find(use.reg);
      if (it == live_regs_.end()) {
        report_fatal_error("GCNRegisterTracker: register %" +
                           Twine(Register::virtReg2Index(use.reg)) +
                           " killed but not in live set");
      }
      LaneBitmask prev_mask = it->second;
      step.kill_masks.push_back({use.reg, prev_mask});
      cur_pressure_.inc(use.reg, prev_mask, LaneBitmask::getNone(), *mri_);
      live_regs_.erase(it);
    }
  }
}

void GCNRegisterTracker::Schedule(const ScheduleNode *node) {
  ScheduleStep step;
  step.saved_max = max_pressure_;

  auto it = node_reg_info_.find(node);
  if (it != node_reg_info_.end()) {
    const NodeRegInfo &info = it->second;

    if constexpr (kModel == PressureModel::kAMDGPU) {
      // Defs first, peak, then dying uses.
      ProcessDefs(info, step);
      max_pressure_ = max(max_pressure_, cur_pressure_);
      ProcessUses(info, step);
    } else {
      // Dying uses first, then defs, then peak.
      ProcessUses(info, step);
      ProcessDefs(info, step);
      max_pressure_ = max(max_pressure_, cur_pressure_);
    }
  }

  undo_stack_.push_back(std::move(step));
}

void GCNRegisterTracker::UndoDefs(const ScheduleStep &step) {
  // Reverse in LIFO order.
  for (int i = static_cast<int>(step.def_prev_masks.size()) - 1; i >= 0; --i) {
    auto [reg, prev_mask] = step.def_prev_masks[i];
    auto it = live_regs_.find(reg);
    LaneBitmask cur_mask = (it != live_regs_.end())
                               ? it->second
                               : LaneBitmask::getNone();
    // inc() is reversible: inc(reg, A, B) then inc(reg, B, A) is a no-op.
    cur_pressure_.inc(reg, cur_mask, prev_mask, *mri_);
    if (prev_mask.none()) {
      if (it != live_regs_.end()) {
        live_regs_.erase(it);
      }
    } else {
      live_regs_[reg] = prev_mask;
    }
  }
}

void GCNRegisterTracker::UndoUses(const NodeRegInfo &info,
                                  const ScheduleStep &step) {
  // Restore killed registers first, in reverse order.
  for (int i = static_cast<int>(step.kill_masks.size()) - 1; i >= 0; --i) {
    auto [reg, prev_mask] = step.kill_masks[i];
    // The register should not be in the live set — we erased it
    // during ProcessUses. If it is, something is out of sync.
    auto it = live_regs_.find(reg);
    if (it != live_regs_.end()) {
      report_fatal_error("GCNRegisterTracker: restoring killed register %" +
                         Twine(Register::virtReg2Index(reg)) +
                         " but it is already in live set");
    }
    cur_pressure_.inc(reg, LaneBitmask::getNone(), prev_mask, *mri_);
    live_regs_[reg] = prev_mask;
    remaining_uses_[reg]++;
  }

  // Re-increment remaining_uses for non-killing uses.
  for (const RegMask &use : info.uses) {
    bool was_killed = false;
    for (const auto &[killed_reg, mask] : step.kill_masks) {
      if (killed_reg == use.reg) {
        was_killed = true;
        break;
      }
    }
    if (!was_killed) {
      remaining_uses_[use.reg]++;
    }
  }
}

void GCNRegisterTracker::Unschedule(const ScheduleNode *node) {
  if (undo_stack_.empty()) {
    report_fatal_error("GCNRegisterTracker: Unschedule without matching "
                       "Schedule");
  }
  ScheduleStep step = std::move(undo_stack_.back());
  undo_stack_.pop_back();

  auto it = node_reg_info_.find(node);
  if (it != node_reg_info_.end()) {
    const NodeRegInfo &info = it->second;

    if constexpr (kModel == PressureModel::kAMDGPU) {
      // Schedule was: defs, peak, uses. Undo in reverse: uses, defs.
      UndoUses(info, step);
      UndoDefs(step);
    } else {
      // Schedule was: uses, defs, peak. Undo in reverse: defs, uses.
      UndoDefs(step);
      UndoUses(info, step);
    }
  }

  max_pressure_ = step.saved_max;
}

// ============================================================================
// Diagnostics
// ============================================================================

std::string
GCNRegisterTracker::DescribeRegOps(const ScheduleNode *node) const {
  auto it = node_reg_info_.find(node);
  if (it == node_reg_info_.end()) {
    return "(no register ops)";
  }

  const NodeRegInfo &info = it->second;
  std::string result;

  if (!info.defs.empty()) {
    result += "defs: ";
    for (const RegMask &def : info.defs) {
      result += "%" + std::to_string(Register::virtReg2Index(def.reg));
      int covered = SIRegisterInfo::getNumCoveredRegs(def.mask);
      result += "(w" + std::to_string(covered) +
                " mask=0x" + Twine::utohexstr(def.mask.getAsInteger()).str() +
                ") ";
    }
  }

  if (!info.uses.empty()) {
    result += "uses: ";
    for (const RegMask &use : info.uses) {
      result += "%" + std::to_string(Register::virtReg2Index(use.reg));
      int covered = SIRegisterInfo::getNumCoveredRegs(use.mask);
      auto uses_it = remaining_uses_.find(use.reg);
      int remaining = (uses_it != remaining_uses_.end()) ? uses_it->second : 0;
      result += "(w" + std::to_string(covered) +
                " mask=0x" + Twine::utohexstr(use.mask.getAsInteger()).str() +
                " rem=" + std::to_string(remaining);
      if (remaining == 0) {
        result += " DEAD";
      }
      result += ") ";
    }
  }

  return result;
}

std::string GCNRegisterTracker::DescribePressure() const {
  std::string result;
  result += "cur: SGPR=" + std::to_string(cur_pressure_.getSGPRNum()) +
            " VGPR=" + std::to_string(cur_pressure_.getVGPRNum(false));
  result += "  peak: SGPR=" + std::to_string(max_pressure_.getSGPRNum()) +
            " VGPR=" + std::to_string(max_pressure_.getVGPRNum(false));
  return result;
}
