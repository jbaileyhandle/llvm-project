//===- GCNRegisterTracker.cpp - GCN register pressure tracking ------------===//
//
// Implementation of GCNRegPressure-based register pressure tracking.
//
// See GCNRegisterTracker.h for design rationale and known limitations.
//
//===----------------------------------------------------------------------===//

#include "GCNRegisterTracker.h"
#include "NodeRegInfo.h"
#include "SIMachineFunctionInfo.h"
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
// Construction
// ============================================================================

GCNRegisterTracker GCNRegisterTracker::NoHistoryClone() const {
  return GCNRegisterTracker(*this, NoHistoryCloneTag{});
}

// Init lists below follow the class's member-declaration order to
// keep clang's -Wreorder-ctor quiet:
//   node_reg_info_table_, remaining_uses_, live_regs_, cur_pressure_,
//   ... (max_pressure_, occ_area_, undo_stack_, pressure_history_
//   default), ..., track_pressure_history_, mf_, st_, mfi_, mri_,
//   continuous_score_tables_.
GCNRegisterTracker::GCNRegisterTracker(const GCNRegisterTracker &source,
                                       NoHistoryCloneTag)
    : node_reg_info_table_(source.node_reg_info_table_),
      remaining_uses_(source.remaining_uses_),
      live_regs_(source.live_regs_),
      cur_pressure_(source.cur_pressure_),
      // max_pressure_, undo_stack_, pressure_history_ intentionally
      // default-constructed (no copy from source) — these are
      // history, not live state. BFS-DP probes via Schedule /
      // Unschedule on cloned PartitionNodes, but uses Schedule's
      // returned edge_peak directly rather than reading max_pressure_,
      // and the undo_stack_ is per-clone (Schedule pushes, Unschedule
      // pops, balanced within each clone's lifetime).
      track_pressure_history_(false),
      // occ_area_ resets to 0 (default member init) — the clone
      // accumulates fresh; BFS-DP reads it as a per-edge delta.
      // test_mode_ and test_vgpr_deltas_ ARE live state (they define
      // what Schedule(node) does), so propagate to the clone. Without
      // this, a clone'd tracker silently falls back to RegDefs/RegUses
      // -driven pressure, which is wrong (and identically zero) for
      // synthetic test graphs with no per-node register info.
      test_mode_(source.test_mode_),
      test_vgpr_deltas_(source.test_vgpr_deltas_),
      mf_(source.mf_),
      st_(source.st_),
      mfi_(source.mfi_),
      mri_(source.mri_),
      continuous_score_tables_(source.continuous_score_tables_) {}

GCNRegisterTracker::GCNRegisterTracker(const ScheduleGraph &graph,
                                       const MachineFunction &mf,
                                       bool track_pressure_history)
    : node_reg_info_table_(&graph.GetNodeRegInfoTable()),
      track_pressure_history_(track_pressure_history),
      mf_(&mf),
      st_(&mf.getSubtarget<GCNSubtarget>()),
      mfi_(mf.getInfo<SIMachineFunctionInfo>()),
      mri_(&mf.getRegInfo()),
      continuous_score_tables_(
          &GetOrComputeContinuousOccupancyScoreTables(*st_)) {
  CheckForFunctionCalls(mf);
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

void GCNRegisterTracker::InitRemainingUses() {
  // Walk every entry in the bound table once; for each use, bump
  // its register's count. Subgraph proxies and other no-register
  // nodes have empty uses vectors so they contribute nothing here.
  for (int i = 0; i < node_reg_info_table_->Size(); ++i) {
    for (const RegMask &use :
         node_reg_info_table_->GetForGraphLocalId(i).uses) {
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

GCNRegPressure GCNRegisterTracker::Schedule(const ScheduleNode *node) {
  if (test_mode_) {
    TestSchedule(node);
    // Test-mode pressure has no transient bump (cur_pressure_ is
    // the delta-driven snapshot post-step).
    return cur_pressure_;
  }
  // Subgraph proxies have no register effect (synthetic node);
  // skip the pressure-update / undo-step path for them. The
  // returned edge peak is just the unchanged cur_pressure_.
  GCNRegPressure edge_peak;
  if (node->IsSchedulingUnit()) {
    ScheduleStep step;
    step.saved_max = max_pressure_;

    const NodeRegInfo &info =
        node_reg_info_table_->GetForNode(node);
    // Empty info (no defs and no uses) is valid — ProcessDefs /
    // ProcessUses loop zero times.
    if constexpr (kModel == PressureModel::kAMDGPU) {
      // Defs first, peak, then dying uses.
      ProcessDefs(info, step);
      // cur_pressure_ now holds the transient peak: old live + new
      // defs, before any uses die. This is the edge_peak we report.
      edge_peak = cur_pressure_;
      max_pressure_ = max(max_pressure_, cur_pressure_);
      ProcessUses(info, step);
    } else {
      // Dying uses first, then defs, then peak. In this model
      // the peak coincides with the post-Schedule cur_pressure_.
      ProcessUses(info, step);
      ProcessDefs(info, step);
      edge_peak = cur_pressure_;
      max_pressure_ = max(max_pressure_, cur_pressure_);
    }

    // Occupancy area under the curve: add this step's continuous
    // occupancy score (of cur_pressure_ after the step). Cheap table
    // lookup; consumers read occ_area_ only for the area-tiebreak
    // metric. Recorded on the step so Unschedule can subtract it.
    step.area_contribution = ContinuousScoreForPressure(cur_pressure_);
    occ_area_ += step.area_contribution;
    undo_stack_.push_back(std::move(step));
  } else {
    edge_peak = cur_pressure_;
  }

  // After any cur_pressure_ update (real path) or no-op (proxy
  // path), record the post-step pressure. "Pressure after step k"
  // semantics; lets PressureHistoryTracker derive postfix peaks
  // via suffix-max. Gated on opt-in: most callers don't read
  // GetPressureHistory and the push/pop is pure overhead for them.
  if (track_pressure_history_) {
    pressure_history_.push_back(cur_pressure_);
  }

  return edge_peak;
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
  // a no-op for cur_pressure_. Pop pressure_history_ first — every
  // Schedule call (proxy or real) pushed an entry under the opt-in,
  // so every Unschedule pops one under the same opt-in.
  if (track_pressure_history_) {
    pressure_history_.pop_back();
  }
  if (!node->IsSchedulingUnit()) {
    return;
  }
  if (undo_stack_.empty()) {
    report_fatal_error("GCNRegisterTracker: Unschedule without matching "
                       "Schedule");
  }
  ScheduleStep step = std::move(undo_stack_.back());
  undo_stack_.pop_back();
  occ_area_ -= step.area_contribution;

  const NodeRegInfo &info =
      node_reg_info_table_->GetForNode(node);
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
  // tracking (max_pressure_, occ_area_, pressure_history_,
  // undo_stack_) updates the same way as the production path so
  // consumers (GetMetricScore, GetContinuousOccupancyScore,
  // GetContinuousOccupancyArea, etc.) read synthetic values
  // transparently.
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
  step.area_contribution = ContinuousScoreForPressure(cur_pressure_);
  occ_area_ += step.area_contribution;
  undo_stack_.push_back(std::move(step));
  if (track_pressure_history_) {
    pressure_history_.push_back(cur_pressure_);
  }
}

void GCNRegisterTracker::TestUnschedule(const ScheduleNode *node) {
  if (track_pressure_history_) {
    pressure_history_.pop_back();
  }
  if (undo_stack_.empty()) {
    report_fatal_error("GCNRegisterTracker test mode: TestUnschedule without "
                       "matching TestSchedule");
  }
  ScheduleStep step = std::move(undo_stack_.back());
  undo_stack_.pop_back();
  occ_area_ -= step.area_contribution;
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

int GCNRegisterTracker::ContinuousScoreForPressure(
    const GCNRegPressure &rp) const {
  // Hot path: one member-pointer load + two array indexes + one min.
  // No per-call arithmetic — the per-pressure-value scores are
  // precomputed once per subtarget and cached (see
  // GetOrComputeContinuousOccupancyScoreTables).
  return std::min(
      continuous_score_tables_->vgpr_score_by_count[
          rp.getVGPRNum(st_->hasGFX90AInsts())],
      continuous_score_tables_->sgpr_score_by_count[rp.getSGPRNum()]);
}

int GCNRegisterTracker::GetContinuousOccupancyScore() const {
  return ContinuousScoreForPressure(max_pressure_);
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
  case ScheduleMetric::kMaximizeContinuousOccupancyThenArea:
    // Primary peak score; the area tiebreak is read separately.
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
  case ScheduleMetric::kMaximizeScheduleLength:
    report_fatal_error(
        "GCNRegisterTracker::GetMetricScore: kMaximizeScheduleLength is "
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
      node_reg_info_table_->GetForNode(node);
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

int GCNRegisterTracker::GetDefCount(const ScheduleNode *node) const {
  if (!node->IsSchedulingUnit()) {
    return 0;
  }
  const NodeRegInfo &info =
      node_reg_info_table_->GetForNode(node);
  return static_cast<int>(info.defs.size());
}

int GCNRegisterTracker::CountLastUses(const ScheduleNode *node) const {
  if (!node->IsSchedulingUnit()) {
    return 0;
  }
  const NodeRegInfo &info =
      node_reg_info_table_->GetForNode(node);
  int kills = 0;
  for (const RegMask &use : info.uses) {
    auto it = remaining_uses_.find(use.reg);
    if (it != remaining_uses_.end() && it->second == 1) {
      ++kills;
    }
  }
  return kills;
}

int GCNRegisterTracker::GetNetDefMinusLastUse(
    const ScheduleNode *node) const {
  return GetDefCount(node) - CountLastUses(node);
}

std::string GCNRegisterTracker::DescribePressure() const {
  std::string result;
  result += "cur: SGPR=" + std::to_string(cur_pressure_.getSGPRNum()) +
            " VGPR=" + std::to_string(cur_pressure_.getVGPRNum(false));
  result += "  peak: SGPR=" + std::to_string(max_pressure_.getSGPRNum()) +
            " VGPR=" + std::to_string(max_pressure_.getVGPRNum(false));
  return result;
}
