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
#include "SIMachineFunctionInfo.h"
#include "ScheduleGraph.h"
#include "Score.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/MC/LaneBitmask.h"
#include <array>
#include <optional>
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
  /// `track_pressure_history` opts the tracker into recording a
  /// per-Schedule pressure snapshot in `pressure_history_`. Default
  /// off because the only production consumer is the pressure
  /// history-domination machinery — most callers don't read it,
  /// and the per-Schedule push/pop is pure overhead for them.
  /// PressureHistoryTracker callers (and shakedowns that read
  /// GetPressureHistory) must pass true.
  ///
  /// Warns if the MachineFunction contains non-inlined function
  /// calls.
  GCNRegisterTracker(const ScheduleGraph &graph,
                     const MachineFunction &mf,
                     bool track_pressure_history = false);

  /// Update pressure after scheduling a node. Order depends on the
  /// pressure model (see PressureModel). Pushes an undo record.
  ///
  /// Returns the edge peak: the per-Schedule transient pressure
  /// peak, captured at the moment all defs have landed but before
  /// any dying uses have killed (kAMDGPU) — or after defs have
  /// landed in kUsesFirst, where it coincides with cur_pressure_
  /// at end-of-Schedule. For proxies and test-mode steps, the
  /// returned value is just the post-Schedule cur_pressure_ (no
  /// transient bump). BFS-DP reads this to compute
  /// edge_peak(P, x) directly.
  GCNRegPressure Schedule(const ScheduleNode *node);

  /// Reverse the last Schedule() call.
  void Unschedule(const ScheduleNode *node);

  /// Return a fresh tracker with cur_pressure_, live_regs_, and
  /// remaining_uses_ copied from `this`. Does NOT copy
  /// undo_stack_, pressure_history_, max_pressure_, or occ_area_:
  /// the clone starts with the vectors empty, max_pressure_ default-
  /// constructed (zero), and occ_area_ reset to 0, intended as a
  /// "frozen" starting state that future Schedule() calls can
  /// extend. (BFS-DP reads occ_area_ as a per-edge delta, so the
  /// reset is what it wants.) test_mode_ / test_vgpr_deltas_ are NOT
  /// copied — clones come up in production mode.
  ///
  /// node_reg_info_table_ and the MF/subtarget pointers are
  /// pointer-copied (shared, not duplicated). The clone borrows
  /// the original's graph; the graph must outlive both.
  ///
  /// track_pressure_history_ on the clone is forced to false —
  /// the clone has no historical context for the parent's history,
  /// and BFS-DP (the planned consumer) doesn't use the history
  /// machinery. Construct a fresh tracker directly if you need
  /// history on a derived state.
  ///
  /// Intended for BFS-DP: each LatticeNode owns a tracker snapshot
  /// captured via NoHistoryClone() from its parent's post-Schedule
  /// state. The light footprint (no undo stack, no history vector,
  /// no max_pressure_ tracking on the clone) keeps per-LatticeNode
  /// memory bounded by live_regs_ + remaining_uses_ + cur_pressure_.
  GCNRegisterTracker NoHistoryClone() const;

  const GCNRegPressure &GetCurrentPressure() const { return cur_pressure_; }
  const GCNRegPressure &GetPeakPressure() const { return max_pressure_; }
  const LiveRegSet &GetLiveRegs() const { return live_regs_; }

  /// Running "occupancy area under the curve": the sum over
  /// scheduling steps of the continuous occupancy score of
  /// cur_pressure_ at each step. Higher = pressure kept low
  /// throughout, not just at the peak. Always accumulated (cheap
  /// table lookup per step); consumers read it only for the area-
  /// tiebreak metric. int64_t to avoid overflow on long schedules.
  int64_t GetContinuousOccupancyArea() const { return occ_area_; }

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
  ///
  /// Fatal error if the tracker was constructed with
  /// `track_pressure_history = false` (the default) — calling this
  /// method is then a caller bug. Construct with the flag true to
  /// opt in.
  ArrayRef<GCNRegPressure> GetPressureHistory() const {
    if (!track_pressure_history_) {
      report_fatal_error(
          "GCNRegisterTracker::GetPressureHistory called on a "
          "tracker constructed with track_pressure_history=false. "
          "Construct with track_pressure_history=true to enable.");
    }
    return pressure_history_;
  }

  /// Occupancy based on peak register pressure only (SGPR and VGPR
  /// limits). Does NOT account for LDS or launch bounds.
  unsigned GetRegisterOnlyOccupancy() const;

  /// Effective occupancy: max(register-only, MFI->getMinWavesPerEU()).
  /// What the kernel will actually run at. The kernel cannot launch
  /// below the structural floor (set by the function's launch
  /// attributes) regardless of register pressure, so when
  /// register-only would drop below the floor the kernel still runs
  /// at the floor and the excess pressure manifests as spills. Use
  /// this rather than GetRegisterOnlyOccupancy in search-pruning
  /// gates that compare against the function's target -- a raw
  /// register-only gate over-prunes in the spill regime (where
  /// every path has reg-only < floor).
  unsigned GetLaunchFloorClampedRegisterOnlyOccupancy() const;

  /// True when current VGPR pressure (cur_pressure_'s VGPR count)
  /// exceeds the VGPR cap permitted at the occupancy floor
  /// (getMaxNumVGPRs(GetLaunchOccupancyFloor())). Per-step view; useful
  /// for per-step area accumulators.
  bool IsCurVGPRInSpillRegime() const;

  /// SGPR-side parallel of IsCurVGPRInSpillRegime.
  bool IsCurSGPRInSpillRegime() const;

  /// OR of IsCurVGPRInSpillRegime and IsCurSGPRInSpillRegime --
  /// "is this step's pressure in spill regime on either track."
  bool IsCurInSpillRegime() const;

  /// True when peak VGPR pressure (max_pressure_'s VGPR count)
  /// exceeds the VGPR cap permitted at the occupancy floor.
  /// Cumulative view; the schedule has spilled at some point if
  /// this is true. Used by the length-pass no-spill-regression gate
  /// and end-of-schedule diagnostics.
  bool IsPeakVGPRInSpillRegime() const;

  /// SGPR-side parallel of IsPeakVGPRInSpillRegime.
  bool IsPeakSGPRInSpillRegime() const;

  /// OR of IsPeakVGPRInSpillRegime and IsPeakSGPRInSpillRegime --
  /// "this schedule has spilled (on either track) at some point."
  bool IsPeakInSpillRegime() const;

  /// Static spill-regime predicates on an arbitrary GCNRegPressure. Same test
  /// as the instance methods above (peak/cur exceeds the register budget at the
  /// occupancy floor), but on a caller-supplied pressure so code holding a
  /// recorded peak -- e.g. RegionInfo::GetOriginalPeakPressure() -- can ask the
  /// question without constructing a tracker over a graph. `launch_floor` is the
  /// occupancy floor whose register budget is the spill threshold
  /// (getMaxNum{V,S}GPRs(launch_floor)); the instance methods pass their own
  /// pressure and GetLaunchOccupancyFloor() and delegate here.
  static bool IsVGPRInSpillRegime(const GCNSubtarget &st, unsigned launch_floor,
                                  const GCNRegPressure &pressure);
  static bool IsSGPRInSpillRegime(const GCNSubtarget &st, unsigned launch_floor,
                                  const GCNRegPressure &pressure);
  static bool IsInSpillRegime(const GCNSubtarget &st, unsigned launch_floor,
                              const GCNRegPressure &pressure) {
    return IsVGPRInSpillRegime(st, launch_floor, pressure) ||
           IsSGPRInSpillRegime(st, launch_floor, pressure);
  }

  /// Peak VGPR count seen so far (max_pressure_'s VGPR count, with the
  /// gfx90a-aware accounting baked in). Convenience accessor that hides
  /// the st_->hasGFX90AInsts() call sites need to make through
  /// GCNRegPressure::getVGPRNum.
  unsigned GetPeakVGPRNum() const;

  /// SGPR-side parallel of GetPeakVGPRNum.
  unsigned GetPeakSGPRNum() const;

  // ----------------------------------------------------------------
  // Current-pressure helpers, relative to the per-track limit
  // implied by the function's currently-configured occupancy target
  // (MFI->getOccupancy()). All read cur_pressure_ rather than
  // max_pressure_ -- suitable for per-step area accumulators and
  // gates that reason about the live step. Limits come from
  // GCNSubtarget::getMaxNum{VGPR,SGPR}s(MFI->getOccupancy(), ...).
  // Separate VGPR / SGPR so callers needn't fold the two tracks
  // together (typical metrics in practice care about VGPR; SGPR
  // helpers are here for the rare SGPR-bound case).
  // ----------------------------------------------------------------

  /// True when current VGPR count is at or below the VGPR cap for
  /// the target occupancy (the schedule could sustain the target
  /// at this step).
  bool IsCurVGPRCountAtOrBelowTargetLimit() const;

  /// Strict counterpart: true when current VGPR count exceeds the
  /// VGPR cap for the target occupancy (this step alone would drop
  /// achievable occupancy below target, or spill if target is
  /// already at the structural floor).
  bool IsCurVGPRCountAboveTargetLimit() const;

  /// max(0, target_VGPR_limit - cur_VGPR). The count of VGPRs by
  /// which we're under the target's cap (effectively, the headroom
  /// before the next pressure increment would push us over). 0 if
  /// we're at or above the limit. Suitable for per-step
  /// "below-limit area" measures.
  unsigned GetCurVGPRCountBelowTargetLimit() const;

  /// max(0, cur_VGPR - target_VGPR_limit). The count of VGPRs by
  /// which we're over the target's cap. 0 if at or below the limit.
  /// Suitable for per-step "above-limit area" measures.
  unsigned GetCurVGPRCountAboveTargetLimit() const;

  /// SGPR-side parallels of the four VGPR helpers above. Same
  /// semantics, swapping VGPR for SGPR and using
  /// getMaxNumSGPRs(target, /*Addressable=*/true) as the cap.
  bool IsCurSGPRCountAtOrBelowTargetLimit() const;
  bool IsCurSGPRCountAboveTargetLimit() const;
  unsigned GetCurSGPRCountBelowTargetLimit() const;
  unsigned GetCurSGPRCountAboveTargetLimit() const;

  // ----------------------------------------------------------------
  // Current-pressure helpers, relative to the per-track spill cap --
  // the per-track cap at the occupancy floor (GetLaunchOccupancyFloor()).
  // Cur counts above this cap are the per-step magnitude of actual
  // spilling: any VGPR/SGPR beyond the cap will be spilled by the
  // allocator. Bool-predicate parallels exist as
  // IsCurVGPRInSpillRegime / IsCurSGPRInSpillRegime (above); these
  // helpers add the magnitude variants used by the per-step
  // VGPR-spill-area accumulator and by direct count-magnitude
  // callers.
  // ----------------------------------------------------------------

  /// max(0, getMaxNumVGPRs(GetLaunchOccupancyFloor()) - cur_VGPR). The
  /// count of VGPRs by which the current step is below the spill
  /// cap; 0 if at or above. Suitable for per-step "below-spill-cap
  /// area" measures.
  unsigned GetCurVGPRCountBelowSpillCap() const;

  /// max(0, cur_VGPR - getMaxNumVGPRs(GetLaunchOccupancyFloor())). The
  /// count of VGPRs by which the current step is over the spill cap;
  /// 0 if at or below. Backs the per-step VGPR spill-area
  /// accumulator (see GetVGPRSpillArea) and is suitable for callers
  /// that want the live "how-much-spill-this-step" count directly.
  unsigned GetCurVGPRCountAboveSpillCap() const;

  /// SGPR-side parallels of the two VGPR spill-cap helpers above.
  /// Same semantics, swapping VGPR for SGPR and using
  /// getMaxNumSGPRs(GetLaunchOccupancyFloor(), /*Addressable=*/true) as
  /// the cap.
  unsigned GetCurSGPRCountBelowSpillCap() const;
  unsigned GetCurSGPRCountAboveSpillCap() const;

  /// Running "VGPR spill area under the curve": the sum over
  /// scheduling steps of GetCurVGPRCountAboveSpillCap at each step.
  /// Higher = more cumulative spilling across the schedule. Always
  /// accumulated (cheap subtract per step); consumers read it only
  /// when they need a spill-area-based metric (e.g., breaking ties
  /// between two schedules that both peak in the spill regime).
  /// int64_t to avoid overflow on long schedules. Reset to 0 in
  /// NoHistoryClone, like occ_area_.
  int64_t GetVGPRSpillArea() const { return vgpr_spill_area_; }

  /// max(0, peak_VGPR - getMaxNumVGPRs(GetLaunchOccupancyFloor())): the
  /// maximum number of VGPRs above the spill cap at any point in the
  /// schedule -- the worst-case count of registers that must be spilled
  /// at once, a proxy for scratch size. Peak-style (reads max_pressure_
  /// like GetPeakVGPRNum), so no accumulator: it just subtracts the cap
  /// from the peak. Backs ScoreDimension::kVgprSpillPeak.
  unsigned GetPeakVGPRCountAboveSpillCap() const;

  /// Occupancy for this region, computed from scratch using
  /// GCNSubtarget::computeOccupancy() with this region's peak
  /// register pressure, the kernel's LDS usage, and the launch
  /// bounds attribute. Does NOT incorporate any occupancy limit
  /// currently configured on the MachineFunction.
  int GetAllFactorsRegionOnlyOccupancy() const;

  /// max(GetAllFactorsRegionOnlyOccupancy(), GetLaunchOccupancyFloor()).
  /// The occupancy the kernel actually launches at for this region's
  /// schedule -- equal to the raw all-factors value when no spilling
  /// is required; equal to the launch floor when the raw value is
  /// below it (i.e., the schedule is in the spill regime, where the
  /// launch attribute keeps the kernel runnable at the cost of spill
  /// loads/stores). Region-only, like its raw counterpart; parallels
  /// GetLaunchFloorClampedRegisterOnlyOccupancy on the all-factors
  /// view that callers use to track per-region launch occupancy.
  int GetLaunchFloorClampedAllFactorsRegionOnlyOccupancy() const;

  /// Static version of the compose used by
  /// GetAllFactorsRegionOnlyOccupancy, parameterized by register
  /// pressure. Register args of 0 skip the register-pressure clamping
  /// inside GCNSubtarget::computeOccupancy.
  static int ComputeAllFactorsOccupancy(
      const GCNSubtarget &st, const MachineFunction &mf,
      unsigned num_sgprs, unsigned num_vgprs);

  /// Occupancy ignoring register pressure: arch max ∩ LDS ∩ launch
  /// bounds. Useful as an independent cross-check against
  /// GetConfiguredMachineFunctionOccupancyTarget — after
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
  unsigned GetConfiguredMachineFunctionOccupancyTarget() const;

  /// The launch-time occupancy floor (MFI->getMinWavesPerEU()) --
  /// the minimum waves/SIMD the kernel can be launched at given its
  /// attributes. The kernel is guaranteed to run at this occupancy
  /// even if register pressure would otherwise demand lower (spill
  /// regime). Honors test_occupancy_floor_override_ (set via
  /// SetOccupancyFloorForTest) so shakedowns can drive consumers at
  /// synthetic floors without mutating MFI; in production the
  /// override is always nullopt and the raw MFI value is returned.
  unsigned GetLaunchOccupancyFloor() const;

  /// Continuous occupancy score based on peak SGPR/VGPR pressure.
  ///
  /// Unlike the integer occupancy getters above, this score varies
  /// smoothly as register pressure changes — two schedules at the
  /// same integer occupancy are distinguished by how far each is
  /// from the next-higher bracket. Higher is better.
  ///
  /// Register-only: LDS and launch bounds are not considered. Parallel
  /// to GetRegisterOnlyOccupancy(); compose with
  /// GetConfiguredMachineFunctionOccupancyTarget() / an early-exit
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

  /// Test-only: directly set cur_pressure_. Mirrors
  /// SetMaxPressureForTest for the live-pressure side; used by
  /// shakedowns that exercise the current-pressure helpers (the
  /// IsCur* / GetCur* family) at controlled boundary points
  /// (well-below, at-limit, above-limit, spill regime).
  void SetCurPressureForTest(GCNRegPressure new_cur) {
    cur_pressure_ = new_cur;
  }

  /// Test-only: directly set vgpr_spill_area_. Lets shakedowns
  /// stage SCs with known accumulated spill area without driving
  /// pressure deltas through the full schedule path. Used by the
  /// DfsMinimizeLengthBoundedSpillSignalsPolicy shakedown to seed
  /// both the input baseline's spill area and the working SC's
  /// spill area at controlled boundary points (below, at, above
  /// the input ceiling).
  void SetVGPRSpillAreaForTest(int64_t new_spill_area) {
    vgpr_spill_area_ = new_spill_area;
  }

  /// Test-only: override what the helpers see for MFI->getOccupancy()
  /// (the target occupancy). Lets shakedowns drive the per-track
  /// helpers' limit calculations independent of the test MF's actual
  /// state -- e.g., sweep target = 10 / 8 / 4 against the same
  /// tracker. nullopt resets to the live MFI value.
  void SetTargetOccupancyForTest(std::optional<unsigned> t) {
    test_target_occupancy_override_ = t;
  }

  /// Test-only: override what GetLaunchFloorClampedRegisterOnlyOccupancy and the
  /// IsCur/IsPeak spill predicates see for the occupancy floor (the
  /// MFI->getMinWavesPerEU() value -- the minimum occupancy the
  /// kernel can be launched at given its attributes). Lets shakedowns
  /// drive the spill checks at arbitrary floors without needing a
  /// kernel-attribute setup. nullopt resets to the live MFI value.
  void SetOccupancyFloorForTest(std::optional<unsigned> f) {
    test_occupancy_floor_override_ = f;
  }

  /// Test-only: clear both overrides at once.
  void ClearTargetAndFloorOverridesForTest() {
    test_target_occupancy_override_.reset();
    test_occupancy_floor_override_.reset();
  }

  /// Pressure-side score for a single-slot `recipe`, polarity
  /// applied so the result is a higher-is-better int. Fatal-errors
  /// when:
  ///   - the recipe has more than one populated slot (this function
  ///     would silently drop tiebreaks otherwise), or
  ///   - the slot dim is not pressure-side (kScheduleLength /
  ///     kIlpScore live on their own trackers).
  int GetScalarScore(const ScoreRecipe &recipe) const;

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

    /// This step's contribution to occ_area_, saved so Unschedule
    /// can subtract it back. 0 for proxies (no pressure change).
    int area_contribution = 0;

    /// This step's contribution to vgpr_spill_area_ (the count of
    /// VGPRs above the spill cap at this step), saved so Unschedule
    /// can subtract it back. 0 for proxies and for steps whose
    /// cur_pressure_ stayed at or below the spill cap.
    int vgpr_spill_area_contribution = 0;
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

  /// Running occupancy area under the curve (see
  /// GetContinuousOccupancyArea). Accumulated in Schedule and
  /// subtracted back in Unschedule.
  int64_t occ_area_ = 0;

  /// Running VGPR spill area under the curve (see GetVGPRSpillArea).
  /// Parallel to occ_area_: accumulated in Schedule (production and
  /// test-mode) by adding GetCurVGPRCountAboveSpillCap after the
  /// step's pressure update, and subtracted back in Unschedule via
  /// the saved per-step contribution on ScheduleStep.
  int64_t vgpr_spill_area_ = 0;

  std::vector<ScheduleStep> undo_stack_;

  /// See GetPressureHistory. Populated only when
  /// track_pressure_history_ is true (Schedule push / Unschedule
  /// pop are gated on the flag).
  std::vector<GCNRegPressure> pressure_history_;

  /// Opt-in flag for pressure_history_. Default false. See ctor.
  bool track_pressure_history_;

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

  /// Test-only overrides for the MFI values the effective-occupancy /
  /// spill-regime / per-track-limit helpers consult. Set via
  /// SetTargetOccupancyForTest / SetOccupancyFloorForTest, cleared
  /// by ClearTargetAndFloorOverridesForTest. nullopt means "use the
  /// live MFI value."
  std::optional<unsigned> test_target_occupancy_override_;
  std::optional<unsigned> test_occupancy_floor_override_;

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

  /// Tag type for the NoHistoryClone-private constructor below.
  /// Distinguishes the clone path from the public ctor without
  /// adding a public-facing overload.
  struct NoHistoryCloneTag {};

  /// Private constructor used by NoHistoryClone. Member-init-lists
  /// only the fields the clone keeps (live_regs_, remaining_uses_,
  /// cur_pressure_, and pointer-typed bindings). Leaves
  /// max_pressure_, undo_stack_, pressure_history_, and test-mode
  /// state default-constructed — avoiding the heap-allocating
  /// copies the default copy constructor would do for the vector
  /// fields that the clone won't use.
  GCNRegisterTracker(const GCNRegisterTracker &source,
                     NoHistoryCloneTag);

  /// Cheap continuous occupancy score for an arbitrary pressure via
  /// the precomputed tables (one min over two array reads). Backs both
  /// GetContinuousOccupancyScore (peak) and the per-step area
  /// accumulation (cur pressure).
  int ContinuousScoreForPressure(const GCNRegPressure &rp) const;

  // --- Schedule helpers ---

  void ProcessDefs(const NodeRegInfo &info, ScheduleStep &step);
  void ProcessUses(const NodeRegInfo &info, ScheduleStep &step);
  void UndoDefs(const ScheduleStep &step);
  void UndoUses(const NodeRegInfo &info, const ScheduleStep &step);
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_GCNREGISTERTRACKER_H
