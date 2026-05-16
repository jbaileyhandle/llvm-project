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
#include "NodeRegInfo.h"
#include "ScheduleGraph.h"
#include "ScheduleMetric.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/MC/LaneBitmask.h"
#include <array>
#include <string>
#include <vector>

namespace llvm {

class GCNSubtarget;
class LiveIntervals;
class MachineFunction;
class MachineRegisterInfo;
class SIMachineFunctionInfo;
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

  /// Construct from a graph and MachineFunction. The per-node
  /// register-operand info is read from `graph.GetNodeRegInfoTable()`
  /// — the table must already be installed by the graph factory
  /// (production: BuildFromSUnits installs via
  /// NodeRegInfoTable::BuildForGraph; tests: factories install an
  /// empty table sized for graph.Size()). Tracker keeps a const
  /// pointer to the graph's table; the graph must outlive the
  /// tracker.
  ///
  /// MF provides MachineRegisterInfo for pressure increments.
  ///
  /// Warns if the MachineFunction contains non-inlined function
  /// calls.
  GCNRegisterTracker(const ScheduleGraph &graph,
                     const MachineFunction &mf);

  /// Update pressure after scheduling a node. Order depends on the
  /// pressure model (see PressureModel). Pushes an undo record.
  void Schedule(const ScheduleNode *node);

  /// Reverse the last Schedule() call.
  void Unschedule(const ScheduleNode *node);

  const GCNRegPressure &GetCurrentPressure() const { return cur_pressure_; }
  const GCNRegPressure &GetPeakPressure() const { return max_pressure_; }
  const LiveRegSet &GetLiveRegs() const { return live_regs_; }

  /// Per-step record of cur_pressure_ after each Schedule call,
  /// indexed by the scheduling step (the schedule order). Includes
  /// entries for both real instructions and subgraph proxies; proxy
  /// steps duplicate the previous value (proxies don't change
  /// pressure). Pop-on-Unschedule keeps the vector aligned with the
  /// current schedule prefix.
  ///
  /// Used by the pressure history-domination machinery
  /// (PressureHistoryTracker) to derive postfix peaks via suffix-
  /// max once a complete schedule has been built. Suffix-max over
  /// duplicate-valued proxy entries reduces to suffix-max over
  /// real-instruction pressures, so proxy entries are absorbed
  /// without affecting peaks.
  ArrayRef<GCNRegPressure> GetPressureHistory() const {
    return pressure_history_;
  }

  /// Occupancy based on peak register pressure only (SGPR and VGPR
  /// limits). Does NOT account for LDS or launch bounds.
  unsigned GetRegisterOnlyOccupancy() const;

  /// Occupancy for this region, computed from scratch using
  /// GCNSubtarget::computeOccupancy() with this region's peak
  /// register pressure, the kernel's LDS usage, and the launch
  /// bounds attribute. Does NOT incorporate any occupancy limit
  /// currently configured on the MachineFunction.
  int GetAllFactorsRegionOnlyOccupancy() const;

  /// Static version of the compose used by
  /// GetAllFactorsRegionOnlyOccupancy, parameterized by register
  /// pressure. Register args of 0 skip the register-pressure clamping
  /// inside GCNSubtarget::computeOccupancy.
  static int ComputeAllFactorsOccupancy(
      const GCNSubtarget &st, const MachineFunction &mf,
      unsigned num_sgprs, unsigned num_vgprs);

  /// Occupancy ignoring register pressure: arch max ∩ LDS ∩ launch
  /// bounds. Useful as an independent cross-check against
  /// GetConfiguredMachineFunctionOccupancyLimit — after
  /// resetInitialOccupancy they should match, and a mismatch means
  /// something lowered the MFI value without a corresponding reset.
  /// Thin wrapper over ComputeAllFactorsOccupancy(st, mf, 0, 0).
  static int ComputeNonRegisterOccupancy(
      const GCNSubtarget &st, const MachineFunction &mf);

  /// The occupancy currently configured on the MachineFunction
  /// (MFI.getOccupancy()). This is the ceiling the region's occupancy
  /// is clamped against — the structural ceiling (hardware max, LDS,
  /// launch bounds) only if nothing has lowered it; otherwise it also
  /// reflects earlier register-pressure-driven reductions from prior
  /// passes or from this scheduler processing other regions of the
  /// same function. Improving this region's register pressure cannot
  /// raise occupancy above this value.
  unsigned GetConfiguredMachineFunctionOccupancyLimit() const;

  /// Continuous occupancy score based on peak SGPR/VGPR pressure.
  ///
  /// Unlike the integer occupancy getters above, this score varies
  /// smoothly as register pressure changes — two schedules at the
  /// same integer occupancy are distinguished by how far each is
  /// from the next-higher bracket. Higher is better.
  ///
  /// Register-only: LDS and launch bounds are not considered. Parallel
  /// to GetRegisterOnlyOccupancy(); compose with
  /// GetConfiguredMachineFunctionOccupancyLimit() / an early-exit
  /// check to handle non-register ceilings.
  ///
  /// For a single dimension (VGPR or SGPR):
  ///   score = M * occ + M * (bracket_ceil - num_reg) / bracket_width
  /// where M is kOccScoreMultiplier, `occ` is the integer occupancy
  /// currently achieved by this dimension, and the bracket is the
  /// register range in which the next integer occupancy step lies.
  /// Final score is min(vgpr_score, sgpr_score), capped at
  /// M * maxWavesPerEU. The bracket steps come from the subtarget,
  /// so the score is portable across gfx targets.
  int GetContinuousOccupancyScore() const;

  /// Pure-functional version of GetContinuousOccupancyScore for
  /// arbitrary register counts. Same formula, but takes the counts
  /// explicitly instead of reading from this region's peak pressure —
  /// useful for shakedown tests that sweep made-up values.
  static int ComputeContinuousOccupancyScore(const GCNSubtarget &st,
                                             unsigned num_vgpr,
                                             unsigned num_sgpr);

  /// Number of register defs the node would introduce (newly live
  /// registers). Read from the pre-extracted, deduplicated,
  /// dead-def-filtered NodeRegInfo.defs — dead defs are excluded
  /// because they don't add live-range pressure across the schedule.
  /// Returns 0 for proxies and nodes without defs. Used by ranking
  /// heuristics.
  int GetDefCount(const ScheduleNode *node) const;

  /// Number of node's uses that would be a "last use" (kill) if
  /// the node were scheduled now — uses where remaining_uses_[reg]
  /// == 1 (this node is the only remaining consumer). Dynamic:
  /// depends on which other nodes are still unscheduled. Used by
  /// ranking heuristics.
  ///
  /// Sub-register precision: lane masks are not considered. A use
  /// is a kill iff this node is the last unscheduled instruction
  /// reading the register at all, regardless of lane. Conservative
  /// — undercounts kills when other unscheduled instructions touch
  /// disjoint lanes of the same vreg (each is a per-lane kill,
  /// but our coarse check requires register-level sole-remaining
  /// status). Sufficient for ranking; not used for pressure
  /// accounting.
  int CountLastUses(const ScheduleNode *node) const;

  /// Net register-pressure delta if the node were scheduled now:
  /// GetDefCount(node) - CountLastUses(node). Negative = scheduling
  /// the node decreases pressure (frees more than it creates).
  /// Positive = increases. Zero = wash. Convenience wrapper around
  /// the two helpers above so policy ranking lambdas can read one
  /// number.
  int GetNetDefMinusLastUse(const ScheduleNode *node) const;

  /// Read-only access to the pre-extracted def/use record for `node`.
  /// Computed once at construction (deduplicated, dead-def-filtered).
  /// Subgraph proxies have empty defs and empty uses — callers that
  /// care about real-instruction-only behavior should branch on
  /// node->IsSchedulingUnit(). Used by the IlpTracker to identify
  /// the def-vreg set (producers to open) and the use-vreg set
  /// (potential first-consumers to close) without re-walking the
  /// MachineInstr operands.
  const NodeRegInfo &GetNodeRegInfo(const ScheduleNode *node) const {
    return node_reg_info_table_->GetForNode(node);
  }

  /// Test-only: switch this tracker to a delta-based synthetic
  /// pressure path. `per_node_vgpr_deltas` is indexed by node
  /// topo index; each Schedule(node) does
  ///   test_current_vgpr_ += deltas[node->GetTopoIndex()];
  ///   cur_pressure_ = GCNRegPressure(test_current_vgpr_);
  ///   max_pressure_ = max(max_pressure_, cur_pressure_);
  ///   pressure_history_.push_back(cur_pressure_);
  /// Unschedule reverses (pop history, restore saved max, subtract
  /// delta). RegDefs/RegUses on the synthetic nodes are ignored —
  /// only the deltas drive pressure.
  ///
  /// Production GetMetricScore / GetContinuousOccupancyScore /
  /// pressure_history_ all read max_pressure_ / cur_pressure_, so
  /// they automatically see the synthetic VGPR counts and report
  /// scores derived from them. No production-path dispatch needed.
  ///
  /// Use only in shakedowns. The deltas can be negative; test
  /// authors are responsible for keeping the running VGPR count
  /// non-negative.
  void EnableTestModeForTest(const std::vector<int> &per_node_vgpr_deltas);

  /// Test-only: true iff EnableTestModeForTest() has been called.
  bool IsInTestModeForTest() const { return test_mode_; }

  /// Test-only: directly set max_pressure_. Used to seed best_'s
  /// peak in shakedowns so working can beat it on the first
  /// IsBetterThan, exercising best-update during test-mode DFS.
  void SetMaxPressureForTest(GCNRegPressure new_max) {
    max_pressure_ = new_max;
  }

  /// Pressure-side score for `metric`, normalized so higher is
  /// better regardless of the metric's natural direction. Lets
  /// callers (e.g., PressureHistoryTracker's dominance check)
  /// stay direction-agnostic — they just compare ints with >.
  ///
  /// Dispatch:
  ///   kMaximizeRegisterOccupancy            → +GetRegisterOnlyOccupancy()
  ///   kMaximizeContinuousRegisterOccupancyScore
  ///                                         → +GetContinuousOccupancyScore()
  ///   kMinimizeRegisterOccupancy            → -GetRegisterOnlyOccupancy()
  ///   kMinimizeContinuousRegisterOccupancyScore
  ///                                         → -GetContinuousOccupancyScore()
  ///
  /// kMinimizeScheduleLength is length-side, not pressure-side;
  /// length lives on ScheduleLengthTracker, so this method
  /// fatal-errors on it.
  ///
  /// Note: returned values are absolute scores, NOT differences.
  /// The negation for minimize variants flips the ordering for
  /// dominance-style comparisons (`a > b` means "a is better than
  /// b" in either direction), but the magnitude no longer matches
  /// the raw register count or score. If you need the raw value,
  /// call the per-metric getters directly.
  int GetMetricScore(ScheduleMetric metric) const;

  /// Score points per integer occupancy step (the `M` in the formula
  /// above). One full occupancy level is worth this many points.
  static constexpr int kOccScoreMultiplier = 1000;

  /// Stand-in ceiling used for SGPR brackets that have no finite
  /// upper cliff (the bottom bracket, where the classifier plateaus,
  /// and unreachable occupancy levels). Chosen well beyond any
  /// realistic SGPR count so within-bracket interpolation stays
  /// positive across the full range we'd ever see.
  static constexpr unsigned kNoSGPRCliff = 255;

  /// Precomputed continuous-occupancy-score lookup tables for one
  /// subtarget — populated once per unique subtarget and cached.
  /// One entry per possible raw register count for each dimension,
  /// so per-call score lookup is just two array indexes + a min
  /// with no per-call arithmetic.
  ///
  /// Sizes are conservative upper bounds for AMDGPU targets we care
  /// about (gfx906 has 256 architectural VGPRs and ~102 per-wave
  /// SGPRs). The +1 above the architectural max covers the
  /// "one-above-cliff" probe used by shakedown sweeps (which test
  /// values just past the top cliff to verify spill-bracket
  /// behavior, score=0).
  static constexpr size_t kContinuousScoreVGPRTableSize = 258;
  static constexpr size_t kContinuousScoreSGPRTableSize = 129;
  struct ContinuousOccupancyScoreTables {
    std::array<int, kContinuousScoreVGPRTableSize> vgpr_score_by_count;
    std::array<int, kContinuousScoreSGPRTableSize> sgpr_score_by_count;
  };

  /// Rolled-own replacement for GCNSubtarget::getMaxNumSGPRs, built by
  /// scanning GCNSubtarget::getOccupancyWithNumSGPRs (the hardcoded
  /// hardware classifier). Returns the max SGPR count whose classifier
  /// value equals `occ`, or kNoSGPRCliff for the bottom bracket and
  /// for unreachable occupancies (so the bottom bracket still
  /// interpolates smoothly from its floor up to the stand-in ceil).
  ///
  /// Needed because LLVM's getMaxNumSGPRs applies granule alignment
  /// and trap-handler reservations, so it can disagree with the
  /// classifier on which bracket a given SGPR count falls in. This
  /// helper agrees with the classifier by construction.
  static unsigned GetMaxNumSGPRsForOcc(const GCNSubtarget &st, unsigned occ);

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

  /// Pointer to the graph's per-node register def/use table. The
  /// graph owns the table (installed by its factory); the tracker
  /// just reads from it. Borrowed pointer — graph must outlive
  /// tracker. Schedule/Unschedule index in via
  /// node_reg_info_table_->GetForNode(node).
  const NodeRegInfoTable *node_reg_info_table_;

  // TODO(perf): live_regs_ is a DenseMap<unsigned, LaneBitmask>
  // keyed by vreg index. Virtual registers are dense across the
  // function but sparse per region, so we can't trivially switch
  // to a vector. An llvm::SparseSet<LiveReg> would give O(1)
  // actual (not amortized) ops and cache-friendly iteration with
  // a one-time per-tracker sparse array allocation. Evaluate if
  // profiling identifies this as a hot spot.
  DenseMap<unsigned, int> remaining_uses_;
  LiveRegSet live_regs_;
  GCNRegPressure cur_pressure_;
  GCNRegPressure max_pressure_;
  std::vector<ScheduleStep> undo_stack_;

  /// See GetPressureHistory.
  std::vector<GCNRegPressure> pressure_history_;

  // --- Test-only state (see EnableTestModeForTest) ---
  bool test_mode_ = false;
  /// Per-node VGPR delta in test mode, indexed by topo index.
  /// Schedule applies, Unschedule reverses. cur_pressure_'s
  /// VGPR32 component holds the running synthetic pressure.
  std::vector<int> test_vgpr_deltas_;

  /// Test-mode counterparts of Schedule/Unschedule. Called from
  /// the public Schedule/Unschedule when test_mode_ is true. Apply
  /// (resp. reverse) the per-node VGPR delta to cur_pressure_,
  /// update max_pressure_/pressure_history_/undo_stack_ exactly as
  /// production would. RegDefs/RegUses on the synthetic nodes are
  /// ignored.
  void TestSchedule(const ScheduleNode *node);
  void TestUnschedule(const ScheduleNode *node);

  const MachineFunction *mf_;
  const GCNSubtarget *st_;
  const SIMachineFunctionInfo *mfi_;
  const MachineRegisterInfo *mri_;

  // --- Construction helpers ---

  /// Walk the bound table once and seed remaining_uses_ with the
  /// total user count for each register. Called by the constructor
  /// after node_reg_info_table_ is set.
  void InitRemainingUses();

  /// Pointer into the per-subtarget cache, set in the constructor.
  /// Never reassigned after construction.
  const ContinuousOccupancyScoreTables *continuous_score_tables_;

public:
  /// Return the continuous-occupancy-score lookup tables for `st`.
  /// Lazily populates a process-wide cache on first call for a given
  /// subtarget pointer. Each tracker instance stores the returned
  /// pointer in continuous_score_tables_ so per-call lookup is just
  /// one member load + array indexes (no DenseMap lookup, no per-call
  /// arithmetic). Public so shakedowns can index the tables directly
  /// for formula-vs-lookup verification.
  static const ContinuousOccupancyScoreTables &
  GetOrComputeContinuousOccupancyScoreTables(const GCNSubtarget &st);
  /// Warn if the MachineFunction contains any call instructions,
  /// since the callee's register usage is invisible to the scheduler.
  /// Call instructions are scheduling boundaries and don't appear
  /// in the DAG as SUnits, so this checks the MachineFunction
  /// directly. One warning per function.
  static void CheckForFunctionCalls(const MachineFunction &mf);

private:

  // --- Schedule helpers ---

  void ProcessDefs(const NodeRegInfo &info, ScheduleStep &step);
  void ProcessUses(const NodeRegInfo &info, ScheduleStep &step);
  void UndoDefs(const ScheduleStep &step);
  void UndoUses(const NodeRegInfo &info, const ScheduleStep &step);
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_GCNREGISTERTRACKER_H
