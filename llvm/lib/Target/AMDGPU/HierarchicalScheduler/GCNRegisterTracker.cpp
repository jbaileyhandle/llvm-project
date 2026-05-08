//===- GCNRegisterTracker.cpp - GCN register pressure tracking ------------===//
//
// Implementation of GCNRegPressure-based register pressure tracking.
//
// See GCNRegisterTracker.h for design rationale and known limitations.
//
//===----------------------------------------------------------------------===//

#include "GCNRegisterTracker.h"
#include "SIMachineFunctionInfo.h"
#include "SIRegisterInfo.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
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
    NodeRegInfo &info) {
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

// ============================================================================
// Construction
// ============================================================================

GCNRegisterTracker::GCNRegisterTracker(const ScheduleGraph &graph,
                                       const MachineFunction &mf,
                                       const LiveIntervals &lis)
    : mf_(&mf),
      st_(&mf.getSubtarget<GCNSubtarget>()),
      mfi_(mf.getInfo<SIMachineFunctionInfo>()),
      mri_(&mf.getRegInfo()),
      continuous_score_tables_(
          &GetOrComputeContinuousOccupancyScoreTables(*st_)) {
  CheckForFunctionCalls(mf);
  ExtractNodeRegInfo(graph, mf.getRegInfo(),
                     *mf.getSubtarget().getRegisterInfo(), lis);
  InitRemainingUses();
}

void GCNRegisterTracker::CheckForFunctionCalls(
    const MachineFunction &mf) {
  static DenseSet<const MachineFunction *> warned_functions;
  if (warned_functions.count(&mf)) {
    return;
  }
  if (mf.getFrameInfo().hasCalls()) {
    warned_functions.insert(&mf);
    llvm::outs() << "GCNRegisterTracker WARNING: function "
                 << mf.getName().str()
                 << " contains function calls. The callee's register "
                 << "usage is not visible to the scheduler, so "
                 << "register pressure and occupancy estimates may "
                 << "be optimistic.\n";
  }
}

void GCNRegisterTracker::ExtractNodeRegInfo(const ScheduleGraph &graph,
                                            const MachineRegisterInfo &mri,
                                            const TargetRegisterInfo &tri,
                                            const LiveIntervals &lis) {
  // One slot per node, indexed by topo_index. Any node whose
  // extraction produces no defs or uses leaves its slot
  // default-constructed (empty defs/uses) — ProcessDefs /
  // ProcessUses treat that as zero iterations, the semantically
  // correct no-op.
  node_reg_info_by_topo_index_.assign(graph.Size(), NodeRegInfo{});
  for (const ScheduleNode &node : graph.Nodes()) {
    if (!node.IsSchedulingUnit()) {
      // Subgraph proxies are synthetic and have no register
      // effect. Leave their slot in node_reg_info_by_topo_index_
      // default-constructed (empty NodeRegInfo) — that's the
      // semantically correct zero-effect entry. Trackers never
      // see proxies at runtime anyway: ScheduleConstructor's
      // dispatch filters them, and the Schedule/Unschedule
      // safety-net guards below catch any dispatch bug that
      // would let one through.
      continue;
    }

    NodeRegInfo info;

    SUnit *su = node.GetSUnit();
    if (su && su->getInstr()) {
      ExtractFromMachineInstr(&node, info, mri, lis);
    } else {
      ExtractFromNodeRegLists(&node, info);
    }

    if (!info.defs.empty() || !info.uses.empty()) {
      node_reg_info_by_topo_index_[node.GetTopoIndex()] = std::move(info);
    }
  }
}

void GCNRegisterTracker::InitRemainingUses() {
  for (const NodeRegInfo &info : node_reg_info_by_topo_index_) {
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
  if (test_mode_) {
    TestSchedule(node);
    return;
  }
  // Subgraph proxies have no register effect (synthetic node);
  // skip the pressure-update / undo-step path for them. We still
  // push a pressure_history_ entry at the end (the unchanged
  // cur_pressure_) so the vector's length tracks every Schedule
  // call, real or proxy.
  if (node->IsSchedulingUnit()) {
    ScheduleStep step;
    step.saved_max = max_pressure_;

    const NodeRegInfo &info =
        node_reg_info_by_topo_index_[node->GetTopoIndex()];
    // Empty info (no defs and no uses) is valid — ProcessDefs /
    // ProcessUses loop zero times.
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

    undo_stack_.push_back(std::move(step));
  }

  // After any cur_pressure_ update (real path) or no-op (proxy
  // path), record the post-step pressure. "Pressure after step k"
  // semantics; lets PressureHistoryTracker derive postfix peaks
  // via suffix-max.
  pressure_history_.push_back(cur_pressure_);
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
  if (test_mode_) {
    TestUnschedule(node);
    return;
  }
  // Symmetric with Schedule: proxies were no-ops, so undo is also
  // a no-op for cur_pressure_. Pop pressure_history_ unconditionally
  // first — every Schedule call (proxy or real) pushed an entry, so
  // every Unschedule pops one.
  pressure_history_.pop_back();
  if (!node->IsSchedulingUnit()) {
    return;
  }
  if (undo_stack_.empty()) {
    report_fatal_error("GCNRegisterTracker: Unschedule without matching "
                       "Schedule");
  }
  ScheduleStep step = std::move(undo_stack_.back());
  undo_stack_.pop_back();

  const NodeRegInfo &info =
      node_reg_info_by_topo_index_[node->GetTopoIndex()];
  // Empty info (no defs and no uses) is valid — UndoDefs / UndoUses
  // loop zero times. Their undo data structures in `step` are also
  // empty in that case, so the loops correctly do nothing.
  if constexpr (kModel == PressureModel::kAMDGPU) {
    // Schedule was: defs, peak, uses. Undo in reverse: uses, defs.
    UndoUses(info, step);
    UndoDefs(step);
  } else {
    // Schedule was: uses, defs, peak. Undo in reverse: defs, uses.
    UndoDefs(step);
    UndoUses(info, step);
  }

  max_pressure_ = step.saved_max;
}

// ============================================================================
// Test mode
// ============================================================================

void GCNRegisterTracker::EnableTestModeForTest(
    const std::vector<int> &per_node_vgpr_deltas) {
  if (!undo_stack_.empty() || !test_vgpr_deltas_.empty()) {
    report_fatal_error("GCNRegisterTracker::EnableTestModeForTest must be "
                       "called once, before any Schedule()");
  }
  test_mode_ = true;
  test_vgpr_deltas_ = per_node_vgpr_deltas;
  // cur_pressure_ / max_pressure_ already start at zero from
  // construction; pressure_history_ already empty. Nothing else
  // to reset.
}

void GCNRegisterTracker::TestSchedule(const ScheduleNode *node) {
  // Apply the per-node VGPR delta to cur_pressure_ via the scalar
  // GCNRegPressure constructor. The rest of the production state
  // tracking (max_pressure_, pressure_history_, undo_stack_)
  // updates the same way as the production path so consumers
  // (GetMetricScore, GetContinuousOccupancyScore, etc.) read
  // synthetic values transparently.
  ScheduleStep step;
  step.saved_max = max_pressure_;
  int new_vgpr = static_cast<int>(cur_pressure_.getVGPRNum(false)) +
                 test_vgpr_deltas_[node->GetTopoIndex()];
  if (new_vgpr < 0) {
    report_fatal_error("GCNRegisterTracker test mode: synthetic VGPR went "
                       "negative; check delta values");
  }
  cur_pressure_ = GCNRegPressure(static_cast<unsigned>(new_vgpr));
  max_pressure_ = max(max_pressure_, cur_pressure_);
  undo_stack_.push_back(std::move(step));
  pressure_history_.push_back(cur_pressure_);
}

void GCNRegisterTracker::TestUnschedule(const ScheduleNode *node) {
  pressure_history_.pop_back();
  if (undo_stack_.empty()) {
    report_fatal_error("GCNRegisterTracker test mode: TestUnschedule without "
                       "matching TestSchedule");
  }
  ScheduleStep step = std::move(undo_stack_.back());
  undo_stack_.pop_back();
  max_pressure_ = step.saved_max;
  int new_vgpr = static_cast<int>(cur_pressure_.getVGPRNum(false)) -
                 test_vgpr_deltas_[node->GetTopoIndex()];
  if (new_vgpr < 0) {
    report_fatal_error("GCNRegisterTracker test mode: VGPR went negative on "
                       "TestUnschedule (delta sign error?)");
  }
  cur_pressure_ = GCNRegPressure(static_cast<unsigned>(new_vgpr));
}

// ============================================================================
// SGPR ceiling table (file-local, built from the classifier)
// ============================================================================

namespace {

// Walk getOccupancyWithNumSGPRs from 0 upward, recording the last n
// at each occupancy level before it drops. Unreachable occupancies
// and the bottom (flat) bracket both stay at kNoSGPRCliff.
SmallVector<unsigned, 12> BuildSGPRMaxTable(const GCNSubtarget &st) {
  SmallVector<unsigned, 12> table;
  unsigned max_waves = st.getMaxWavesPerEU();
  table.assign(max_waves + 1, GCNRegisterTracker::kNoSGPRCliff);

  // Cap is just comfortably past any known SGPR cliff. gfx9's
  // classifier plateaus at 7 past 100 SGPRs; 128 is ample headroom.
  constexpr unsigned kScanCap = 128;

  unsigned prev_occ = st.getOccupancyWithNumSGPRs(0);
  for (unsigned n = 1; n <= kScanCap; ++n) {
    unsigned cur = st.getOccupancyWithNumSGPRs(n);
    if (cur < prev_occ) {
      if (prev_occ < table.size()) {
        table[prev_occ] = n - 1;
      }
      prev_occ = cur;
      if (cur == 0) {
        break;
      }
    }
  }
  // prev_occ now holds the bottom bracket. Leave its entry as
  // kNoSGPRCliff — the bracket is unbounded above our scan cap.
  return table;
}

} // namespace

unsigned GCNRegisterTracker::GetMaxNumSGPRsForOcc(const GCNSubtarget &st,
                                                  unsigned occ) {
  static const SmallVector<unsigned, 12> kTable = BuildSGPRMaxTable(st);
  return (occ < kTable.size()) ? kTable[occ] : kNoSGPRCliff;
}

// ============================================================================
// Continuous occupancy score helpers (file-local)
// ============================================================================
//
// Smooth version of integer occupancy: integer occupancy plus a
// fractional bonus for how far below the current bracket's upper
// edge the register count sits. See GCNRegisterTracker.h for the
// formula. Uncapped by design: the early-exit check
// (RegisterOnlyOccupancyIsAtOrAboveFunctionOccupancyTarget) makes
// the top-bracket plateau unreachable during search, so a cap
// would only add branches without changing any decision.
namespace {

// Continuous occupancy score for a single register class. The two
// callbacks mirror the GCNSubtarget methods of the same names,
// generic so the body is shared between VGPR and SGPR.
template <typename GetOccupancyWithNumRegsFunc, typename GetMaxNumRegsFunc>
int ComputeContinuousOccupancyScoreForRegClass(
    unsigned num_regs, int max_waves,
    GetOccupancyWithNumRegsFunc get_occupancy_with_num_regs_func,
    GetMaxNumRegsFunc get_max_num_regs_func) {
  constexpr int M = GCNRegisterTracker::kOccScoreMultiplier;

  int occ = static_cast<int>(get_occupancy_with_num_regs_func(num_regs));
  if (occ == 0) {
    // Spilling — below the occ=1 bracket. Score 0 so any
    // non-spilling schedule strictly beats it.
    return 0;
  }

  // Bracket for this occupancy: (floor, ceil]. ceil is the max reg
  // count still at this occ; floor is the max reg count at the next
  // higher occ (or 0 if this is the top bracket, so the bracket
  // spans 1..ceil).
  int ceil = static_cast<int>(get_max_num_regs_func(occ));
  int floor = (occ == max_waves)
                  ? 0
                  : static_cast<int>(get_max_num_regs_func(occ + 1));
  int width = ceil - floor;

  // Degenerate bracket: some targets have adjacent occupancies that
  // share a register ceiling (insufficient granules to distinguish
  // them). No within-bracket differentiation possible; just return
  // the integer occupancy score.
  if (width == 0) {
    return M * occ;
  }

  int within = M * (ceil - static_cast<int>(num_regs)) / width;
  return M * occ + within;
}

} // namespace

// ============================================================================
// Occupancy
// ============================================================================

int GCNRegisterTracker::ComputeContinuousOccupancyScore(
    const GCNSubtarget &st, unsigned num_vgpr, unsigned num_sgpr) {
  int max_waves = static_cast<int>(st.getMaxWavesPerEU());

  int vgpr_score = ComputeContinuousOccupancyScoreForRegClass(
      num_vgpr, max_waves,
      [&](unsigned n) { return st.getOccupancyWithNumVGPRs(n); },
      [&](unsigned occ) { return st.getMaxNumVGPRs(occ); });

  int sgpr_score = ComputeContinuousOccupancyScoreForRegClass(
      num_sgpr, max_waves,
      [&](unsigned n) { return st.getOccupancyWithNumSGPRs(n); },
      [&](unsigned occ) { return GetMaxNumSGPRsForOcc(st, occ); });

  return std::min(vgpr_score, sgpr_score);
}

int GCNRegisterTracker::GetContinuousOccupancyScore() const {
  // Hot path: one member-pointer load + two array indexes + one min.
  // No per-call arithmetic — the per-pressure-value scores are
  // precomputed once per subtarget and cached (see
  // GetOrComputeContinuousOccupancyScoreTables).
  return std::min(continuous_score_tables_->vgpr_score_by_count[
                      max_pressure_.getVGPRNum(st_->hasGFX90AInsts())],
                  continuous_score_tables_->sgpr_score_by_count[
                      max_pressure_.getSGPRNum()]);
}

const GCNRegisterTracker::ContinuousOccupancyScoreTables &
GCNRegisterTracker::GetOrComputeContinuousOccupancyScoreTables(
    const GCNSubtarget &st) {
  // Process-wide cache, lazily populated. One entry per unique
  // subtarget pointer; in typical compilations there's only one.
  // Construction-time call only — never on the hot path.
  static DenseMap<const GCNSubtarget *, ContinuousOccupancyScoreTables>
      cache;
  auto it = cache.find(&st);
  if (it != cache.end()) {
    return it->second;
  }

  ContinuousOccupancyScoreTables tables;
  for (size_t v = 0; v < kContinuousScoreVGPRTableSize; ++v) {
    // Score for this VGPR count alone (SGPR set to 0 = SGPR
    // dimension contributes its max possible score, so the min
    // returns the VGPR-side value).
    tables.vgpr_score_by_count[v] =
        ComputeContinuousOccupancyScore(st, /*num_vgpr=*/v,
                                        /*num_sgpr=*/0);
  }
  for (size_t s = 0; s < kContinuousScoreSGPRTableSize; ++s) {
    tables.sgpr_score_by_count[s] =
        ComputeContinuousOccupancyScore(st, /*num_vgpr=*/0,
                                        /*num_sgpr=*/s);
  }
  return cache.insert({&st, tables}).first->second;
}

unsigned GCNRegisterTracker::GetRegisterOnlyOccupancy() const {
  return max_pressure_.getOccupancy(*st_);
}

int GCNRegisterTracker::GetMetricScore(ScheduleMetric metric) const {
  switch (metric) {
  case ScheduleMetric::kMaximizeRegisterOccupancy:
    return static_cast<int>(GetRegisterOnlyOccupancy());
  case ScheduleMetric::kMaximizeContinuousRegisterOccupancyScore:
    return GetContinuousOccupancyScore();
  case ScheduleMetric::kMinimizeRegisterOccupancy:
    return -static_cast<int>(GetRegisterOnlyOccupancy());
  case ScheduleMetric::kMinimizeContinuousRegisterOccupancyScore:
    return -GetContinuousOccupancyScore();
  case ScheduleMetric::kMinimizeScheduleLength:
    report_fatal_error(
        "GCNRegisterTracker::GetMetricScore: kMinimizeScheduleLength is "
        "length-side, not pressure-side; length lives on "
        "ScheduleLengthTracker");
  }
  llvm_unreachable("Unknown ScheduleMetric");
}

unsigned GCNRegisterTracker::GetConfiguredMachineFunctionOccupancyLimit() const {
  return mfi_->getOccupancy();
}

int GCNRegisterTracker::ComputeAllFactorsOccupancy(
    const GCNSubtarget &st, const MachineFunction &mf,
    unsigned num_sgprs, unsigned num_vgprs) {
  const SIMachineFunctionInfo *mfi = mf.getInfo<SIMachineFunctionInfo>();
  unsigned occ = st.computeOccupancy(mf.getFunction(), mfi->getLDSSize(),
                                     num_sgprs, num_vgprs);
  return static_cast<int>(std::min(occ, mfi->getMaxWavesPerEU()));
}

int GCNRegisterTracker::ComputeNonRegisterOccupancy(
    const GCNSubtarget &st, const MachineFunction &mf) {
  return ComputeAllFactorsOccupancy(st, mf, /*num_sgprs=*/0,
                                    /*num_vgprs=*/0);
}

int GCNRegisterTracker::GetAllFactorsRegionOnlyOccupancy() const {
  return ComputeAllFactorsOccupancy(
      *st_, *mf_,
      max_pressure_.getSGPRNum(),
      max_pressure_.getVGPRNum(st_->hasGFX90AInsts()));
}

// ============================================================================
// Diagnostics
// ============================================================================

std::string
GCNRegisterTracker::DescribeRegOps(const ScheduleNode *node) const {
  const NodeRegInfo &info =
      node_reg_info_by_topo_index_[node->GetTopoIndex()];
  if (info.defs.empty() && info.uses.empty()) {
    return "(no register ops)";
  }
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
