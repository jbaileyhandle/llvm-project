//===- DfsSearch.h - Depth-first search over schedules -------*- C++ -*-===//
//
// Generic DFS over complete schedules for a single region. The Policy
// class specifies the objective metric, ready-list ordering, and
// bounding; see SearchPolicies.h for concrete policies.
//
// Header-only template. A .cpp is not needed unless explicit
// instantiations are required.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_DFSSEARCH_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_DFSSEARCH_H

#include "LengthHistoryTracker.h"
#include "PressureHistoryTracker.h"
#include "ScheduleConstructor.h"
#include "ScheduleGraph.h"
#include "SearchResult.h"
#include "SubgraphFormation.h"
#include "llvm/ADT/SmallVector.h"
#include <chrono>
#include <limits>
#include <optional>

namespace llvm {
class GCNSubtarget;
class MachineFunction;
class LiveIntervals;

namespace hierarchical_scheduler {

// Policy contract (the static methods below — Policy classes don't
// inherit, they just have these by name):
//   static constexpr ScheduleMetric kMetric;
//   static void FilterAndSortReadyList(
//       const ScheduleConstructor &working,
//       SmallVectorImpl<const ScheduleNode *> &out);
//   static bool ShouldBoundSearch(
//       const ScheduleConstructor &schedule_constructor,
//       const ScheduleConstructor &best_schedule_constructor,
//       LengthHistoryTracker &length_history,
//       PressureHistoryTracker &pressure_history);
//   static bool ShouldEndSearch(
//       const ScheduleConstructor &schedule_constructor,
//       const ScheduleConstructor &best_schedule_constructor);
//   static SubgraphFormationPolicy MakeFormationPolicy();
//
// MakeFormationPolicy and FilterAndSortReadyList are provided with
// defaults by SearchPolicyBase (see SearchPolicies.h); concrete
// policies override as needed.
//
// FilterAndSortReadyList produces a per-Recurse snapshot in the
// policy's iteration order. DfsSearch iterates that snapshot via
// ScheduleConstructor::Schedule(node) — the underlying ready list is
// maintained in topo_index ascending order by ScheduleConstructor for
// O(log K) insert/lookup, but the iteration order is whatever the
// policy says.
//
// ShouldBoundSearch is a LOCAL prune: returning true abandons the
// current subtree but lets the search continue elsewhere. Called on
// partial schedules. ShouldEndSearch is a GLOBAL terminate:
// returning true stops the entire search and Run() returns whatever
// best is currently held. Called on completed schedules, after the
// IsBetterThan/update step.
template <typename Policy>
class DfsSearch {
 public:
  // form_subgraphs gates whether Policy::MakeFormationPolicy() is
  // invoked at construction. Default true (run formation when the
  // policy declares one). Pass false to suppress formation per-call
  // even when the policy supports it — useful for diagnostics that
  // want to compare with-vs-without on the same region.
  DfsSearch(ScheduleGraph &graph, const GCNSubtarget &st,
            const MachineFunction &mf, const LiveIntervals &lis,
            bool form_subgraphs = true)
      // MaybeFormSubgraphs runs formation as a side effect (when
      // form_subgraphs is true and Policy declares a non-empty
      // formation policy) and returns the same graph reference. C++
      // initializer-list ordering is by member declaration order:
      // working_schedule_constructor_ is initialized first (its
      // initializer below runs the side-effecting helper), then
      // best_schedule_constructor_ reads graph.GetInputScheduleConstructor()
      // — which is the PRE-formation input snapshot captured in
      // BuildFromSUnits Phase 4 and never re-derived, so it remains
      // a valid (unconstrained) baseline for the search to beat.
      : mf_(&mf),
        lis_(&lis),
        working_schedule_constructor_(
            MaybeFormSubgraphs(graph, form_subgraphs), st, mf),
        best_schedule_constructor_(graph.GetInputScheduleConstructor()),
        // length_history_ binds to working_'s trackers. Constructed
        // unconditionally; queried only when Policy::
        // kUseLengthHistoryPruning is true (the `if constexpr` in
        // Recurse dead-strips the consult/insert otherwise).
        // Pressure-score dim is gated by Policy::
        // kRefineOccupancyAtSameLength; ILP dim is gated by
        // Policy::kRefineIlpAtSameLength. When neither gate is
        // set, dominance reduces to length-only.
        length_history_(
            &working_schedule_constructor_.GetScheduledSetTracker(),
            &working_schedule_constructor_.GetLengthTracker(),
            &working_schedule_constructor_.GetPressureTracker(),
            &working_schedule_constructor_.GetIlpTracker(),
            Policy::kRefineOccupancyAtSameLength,
            Policy::kRefineIlpAtSameLength,
            Policy::kLengthMaxMode),
        // pressure_history_ binds to working_'s scheduled-set
        // tracker for partition keys, and to working_'s pressure
        // tracker for the no-arg score read. The metric matches
        // Policy::kMetric so the tracker reads values consistent
        // with what the search optimizes.
        pressure_history_(
            &working_schedule_constructor_.GetScheduledSetTracker(),
            &working_schedule_constructor_.GetPressureTracker(),
            Policy::kMetric) {
    // Stamp the stopwatch's lifetime_start at construction. (This
    // first Start() also seeds current_run_start, which Run()
    // overwrites on every entry.) timing_.lifetime_start is
    // unaffected by ResetForReuse, so it spans every Run() of
    // any outer loop driving this DfsSearch — and so does the
    // timeout check, which derives elapsed against it.
    timing_.Start();
    // Populate working's max-schedule-cycle table from the
    // current effective bound (best holds the input schedule
    // here, so the seed reflects the input-derived bound).
    RecomputeWorkingMaxScheduleCycles();
    // best_schedule_constructor_ is copy-constructed from the
    // graph's input schedule (built by BuildFromSUnits as Phase 4).
    // That gives us a complete valid schedule matching the region's
    // current MF order before any DFS step runs, so DFS guarantees
    // non-regression against the input MF order — no separate
    // outer-level comparison needed.
    //
    // The input schedule was built on the PRE-formation graph, but
    // it's still a valid baseline post-formation: the metrics
    // compared by IsBetterThan (current_cycle for length, peak
    // GCNRegPressure for occupancy) are functions of the real-
    // instruction schedule order plus per-node register info,
    // neither of which formation touches. Formation only constrains
    // which orders DFS will explore; the input baseline is the
    // unconstrained reference, and the post-formation search keeps
    // it iff no constrained schedule beats it.
    //
    // Note: best's ready list uses the default (topo_index) comparator
    // since input_schedule_constructor_ was built with the default.
    // That's fine — best is read-only after construction from the
    // DFS's perspective; we never iterate its ready list for order.
  }

  // Runs DFS and returns a SearchResult. Calls timing_.Start() at
  // entry, which resets current_run_start so per-Run telemetry
  // (GetCurrentRunElapsedMs) measures from this point.
  // timing_.lifetime_start was set at construction and is not
  // touched here.
  //
  // SearchResult::schedule is always populated for DFS — best is
  // seeded with the input order, so worst case it carries a copy
  // of the input (see SearchResult.h). It is therefore never
  // nullopt here; the optional shape exists for searches that can
  // genuinely produce nothing (BFS-DP).
  SearchResult Run() {
    timing_.Start();
    Recurse();
    return SearchResult{best_schedule_constructor_, GetTerminationCause()};
  }

  // Wall-clock elapsed (milliseconds) for the most recent Run().
  // Read right after Run() returns; calling later includes idle
  // time after the run ended (this stopwatch has no captured
  // end — see DualRunAndLifetimeStopwatch).
  int64_t GetCurrentRunElapsedMs() const {
    return timing_.CurrentRunElapsedMs();
  }

  // Wall-clock elapsed (milliseconds) since this DfsSearch was
  // constructed. Live; reflects "now" each time it's called.
  int64_t GetRegionElapsedMs() const {
    return timing_.LifetimeElapsedMs();
  }

  // Reset state so this DfsSearch can be Run() again — drives
  // working back to empty via UnscheduleAll, clears the history
  // trackers, re-seeds best from the graph's input baseline,
  // clears should_end_search_, and stores target_length for the
  // next Run()'s bound.
  //
  // Region-scope fields (region_start_time_, region_deadline_,
  // region_timed_out_) are intentionally NOT touched; they persist
  // across this DfsSearch's full lifetime so the same time budget
  // covers all Run() calls. If the region deadline has already
  // passed, the next Recurse will observe that immediately and
  // exit.
  //
  // Designed for outer loops that walk a target value over
  // multiple Run() calls (see ScheduleRegionForLengthPass).
  void ResetForReuse(int requested_target_length) {
    working_schedule_constructor_.Reset();
    length_history_.Reset();
    pressure_history_.Reset();
    best_schedule_constructor_ =
        working_schedule_constructor_.GetGraph().GetInputScheduleConstructor();
    should_end_search_ = false;
    requested_target_length_ = requested_target_length;
    RecomputeWorkingMaxScheduleCycles();
  }

  // True iff the region-level deadline (set in the ctor as
  // Policy::kTimeoutMsPerRegion past construction time) has
  // been reached at some point during this DfsSearch's lifetime,
  // and a Recurse aborted as a result. Sticky once set. The
  // single-bool shape is correct for this concept: once the
  // region deadline fires, the search exits and we don't start
  // another Run(), so a per-run vs lifetime distinction wouldn't
  // add information. A future per-run deadline (separate concept,
  // separate field) would get a DualRunAndLifetimeFlag because
  // its per-run and lifetime values can genuinely diverge.
  bool RegionTimedOut() const { return region_timed_out_; }

  // How the most recent Run ended. The three cases:
  //   kFullyExplored:   recursion drained naturally — every reachable
  //                     schedule was visited. Returned best is
  //                     provably optimal under the policy.
  //   kPolicySatisfied: Policy::ShouldEndSearch returned true on a
  //                     complete schedule (for the occupancy policy,
  //                     the current best hit the function occupancy
  //                     target). The policy decided the current best
  //                     is good enough; could have kept going but
  //                     saw no point.
  //   kTimedOut:        the region budget
  //                     (Policy::kTimeoutMsPerRegion) was exhausted
  //                     before search completed and before the
  //                     policy declared success. Returned best is
  //                     the best found so far; no optimality claim,
  //                     and the search would have kept looking for
  //                     something better.
  //
  // Per single Run() the two end-paths (timeout vs. policy) are
  // mutually exclusive: timeout returns up the stack immediately,
  // and policy fires only at a completed schedule, after which the
  // unwind doesn't re-enter Recurse and therefore doesn't re-check
  // the timeout. So we can derive the cause from the existing
  // flags without an extra "policy fired" bit. (The order of the
  // checks below codifies "if policy fired, classify as
  // policy-satisfied even if a stale region_timed_out_ from a
  // hypothetical earlier Run also happens to be set" — relevant
  // only if a future caller starts running multiple Run()s per
  // region; the occupancy pass runs Run() exactly once.)
  SearchTerminationCause GetTerminationCause() const {
    if (should_end_search_ && !region_timed_out_) {
      return SearchTerminationCause::kPolicySatisfied;
    }
    if (region_timed_out_) {
      return SearchTerminationCause::kTimedOut;
    }
    return SearchTerminationCause::kFullyExplored;
  }

  // Read-only access to the working constructor's
  // schedule-call counter. `.current_run` measures search effort
  // for the current Run() in isolation; `.lifetime` is the
  // cumulative across all Run()s. Compare to the region's node
  // count: equal means a single linear pass with no backtracking;
  // N * K means roughly K average orderings explored per node.
  const DualRunAndLifetimeCounter &ScheduleCallCount() const {
    return working_schedule_constructor_.ScheduleCallCount();
  }

  // Read-only access to the length history tracker. Useful for
  // shakedowns that want to inspect prune_count, total_entries, etc.
  // after Run() returns.
  const LengthHistoryTracker &GetLengthHistoryTracker() const {
    return length_history_;
  }

  // Diagnostic counters — total count of complete schedules
  // visited (every time Recurse hit working.IsDone()) and total
  // count of best updates (every time IsBetterThan returned true).
  // Useful for understanding search behavior:
  //   complete_schedules_count == 1 → search found one path,
  //     never backtracked to a different completion.
  //   complete_schedules_count >> 1, best_updates_count == 1 →
  //     search found many completions but only the first beat
  //     the seeded baseline (alternatives didn't improve on
  //     IsBetterThan's metric).
  //   best_updates_count > 1 → tiebreak / strictly-better updates
  //     fired more than once during search.
  // Lifetime values; never reset (no per-Run distinction yet —
  // add if needed).
  int CompleteSchedulesCount() const {
    return complete_schedules_count_;
  }
  int BestUpdatesCount() const { return best_updates_count_; }
  // Number of times a candidate beat `best` per our tracker but was
  // rejected by the per-candidate LLVM-tracker verification because
  // its ground-truth occupancy was below the kernel target. Should
  // ideally be 0 — non-zero means our pressure tracker is
  // under-counting somewhere relative to LLVM's.
  int LlvmTrackerRejectionsCount() const {
    return llvm_tracker_rejections_count_;
  }

  // Read-only access to the pressure history tracker. Useful for
  // shakedowns and per-region stat reporting.
  const PressureHistoryTracker &GetPressureHistoryTracker() const {
    return pressure_history_;
  }

  // Test-only: enable delta-based synthetic pressure on both the
  // working and best schedule constructors' pressure trackers,
  // and seed best's max_pressure_ to a deliberately-bad value so
  // working can beat it on the first IsBetterThan and the search
  // exercises real best-update behavior. Must be called before
  // Run(). After this, Schedule(node) applies
  // per_node_vgpr_deltas[node->GetTopoIndex()] to cur_pressure_'s
  // VGPR32 component instead of going through GCNRegPressure /
  // MRI machinery; see GCNRegisterTracker::EnableTestModeForTest
  // for the full contract.
  //
  // best_initial_vgpr_peak default is 255 — saturates past
  // gfx906's integer-occupancy cliffs so any synthetic working
  // peak under that beats it. Tests can lower it if they want a
  // tighter initial bound.
  void EnableTestModeForTest(const std::vector<int> &per_node_vgpr_deltas,
                             unsigned best_initial_vgpr_peak = 255) {
    working_schedule_constructor_.GetPressureTrackerForTest()
        .EnableTestModeForTest(per_node_vgpr_deltas);
    // best_ doesn't get Schedule()d directly by DFS; it's only
    // overwritten via copy assignment from working_ on
    // IsBetterThan, which carries working's test-mode state along
    // with everything else. So we only need to seed best_'s
    // max_pressure_ here, deliberately worse than any synthetic
    // working completion will reach.
    best_schedule_constructor_.GetPressureTrackerForTest()
        .SetMaxPressureForTest(GCNRegPressure(best_initial_vgpr_peak));
  }

 private:
  // Helper called from the member initializer list. Runs subgraph
  // formation as a side effect (when form_subgraphs is true and
  // Policy declares a non-empty formation policy), then returns the
  // same graph reference passed in.
  //
  // The return is a sequencing tool, not a value carrier. Putting
  // the call in the initializer expression for
  // working_schedule_constructor_ guarantees the side effect
  // completes BEFORE working_schedule_constructor_'s ctor reads
  // the graph — which is what we need for working to see the
  // post-formation graph state. Running formation in the ctor body
  // would be too late (member init runs first).
  //
  // FormSubgraphs early-returns on an empty pipeline, so the
  // SearchPolicyBase default (no formation) costs one bool check
  // and one function call per construction.
  static ScheduleGraph &MaybeFormSubgraphs(ScheduleGraph &graph,
                                           bool form_subgraphs) {
    if (form_subgraphs) {
      FormSubgraphs(graph, Policy::MakeFormationPolicy());
    }
    return graph;
  }

  // Re-derive working's per-node max-schedule-cycle table from
  // the current iteration target and best-schedule length. The
  // effective bound is min(requested_target_length_,
  // best_length - improvement_offset): smaller of the explicit
  // iteration target the outer driver set and the
  // strict-improvement (or same-length-allowed) bound implied by
  // the best schedule found so far. Called whenever either input
  // changes — at construction, at each ResetForReuse, and after
  // every best-improvement event in Recurse.
  //
  // The improvement_offset is policy-controlled via two flags
  // that both express "allow same-length completions":
  //   - kRefineOccupancyAtSameLength: refine occupancy within
  //     same length.
  //   - kRefineIlpAtSameLength: refine ILP within same length.
  // Either being true relaxes the bound from best.length - 1 to
  // best.length so same-length completions are produced and
  // IsBetterThan picks among them.
  //   neither set: offset = 1 → bound = best.length - 1, only
  //     strictly shorter completions are produced.
  //   either set:  offset = 0 → bound = best.length, same-length
  //     completions allowed.
  void RecomputeWorkingMaxScheduleCycles() {
    int best_length = best_schedule_constructor_.GetScheduleLength();
    constexpr bool kAllowSameLength =
        Policy::kRefineOccupancyAtSameLength ||
        Policy::kRefineIlpAtSameLength;
    constexpr int kImprovementOffset = kAllowSameLength ? 0 : 1;
    working_schedule_constructor_.SetMaxAcceptableScheduleLength(
        std::min(requested_target_length_,
                 best_length - kImprovementOffset));
  }

  // If the region elapsed time has reached the policy's per-region
  // budget, set region_timed_out_ (for telemetry) and
  // should_end_search_ (for propagation through the recursion),
  // and return true so the caller can unwind. Reuses
  // should_end_search_ for propagation: each recursive frame
  // already checks it after every child returns and unwinds when
  // set, so the search exits cleanly and Run() returns the seeded
  // baseline (or anything better DFS managed to find before the
  // wall hit). steady_clock::now() on Linux is vDSO-backed (~20ns),
  // so calling on every Recurse() entry is cheap.
  bool EndSearchIfTimedOut() {
    // nullopt = timeout disabled (e.g., shakedown oracles where any
    // early exit would yield a suboptimal reference answer).
    if (!Policy::kTimeoutMsPerRegion.has_value() ||
        timing_.LifetimeElapsedMs() < *Policy::kTimeoutMsPerRegion) {
      return false;
    }
    region_timed_out_ = true;
    should_end_search_ = true;
    return true;
  }

  void Recurse() {
    if (EndSearchIfTimedOut()) {
      return;
    }

    if (working_schedule_constructor_.IsDone()) {
      ++complete_schedules_count_;
      if (working_schedule_constructor_.IsBetterThan(
              best_schedule_constructor_, Policy::kMetric)) {
        // Cross-check the candidate's register pressure against
        // LLVM's GCNUpwardRPTracker before committing to it. If
        // LLVM's tracker says the candidate's ground-truth
        // occupancy is below the kernel target, we treat the
        // candidate as if it didn't beat best — our tracker
        // under-estimated pressure, and accepting the schedule
        // would silently drop occupancy. Log the divergence and
        // keep searching. No fatal error.
        auto verification =
            working_schedule_constructor_.VerifyPressureWithLlvmTracker(
                *mf_, *lis_);
        if (verification.target_met) {
          ++best_updates_count_;
          best_schedule_constructor_ = working_schedule_constructor_;
          // best.length may have shrunk; re-derive working's
          // max-schedule-cycle table so the new (tighter) bound
          // takes effect on subsequent steps.
          RecomputeWorkingMaxScheduleCycles();
        } else {
          ++llvm_tracker_rejections_count_;
          llvm::outs()
              << "\t\t\t\tLLVM verifier rejected candidate: target="
              << verification.target_occupancy
              << " ours_occ=" << verification.ours_occupancy
              << " llvm_occ=" << verification.llvm_occupancy
              << " ours_vgpr="
              << verification.ours_peak.getVGPRNum(
                     mf_->getSubtarget<GCNSubtarget>().hasGFX90AInsts())
              << " llvm_vgpr="
              << verification.llvm_peak.getVGPRNum(
                     mf_->getSubtarget<GCNSubtarget>().hasGFX90AInsts())
              << " ours_sgpr=" << verification.ours_peak.getSGPRNum()
              << " llvm_sgpr=" << verification.llvm_peak.getSGPRNum()
              << "\n";
        }
      }
      if (Policy::ShouldEndSearch(working_schedule_constructor_,
                                  best_schedule_constructor_)) {
        should_end_search_ = true;
      }
      return;
    }

    // ShouldBoundSearch covers all prune decisions for this policy,
    // including history-based domination when the policy opts in.
    // It MAY MUTATE length_history_ and/or pressure_history_ (record
    // current state, push fast-forward replay hints onto DfsSearch's
    // replay queue) as part of its check — see the policy's
    // ShouldBoundSearch contract and the trackers' class comments.
    if (Policy::ShouldBoundSearch(working_schedule_constructor_,
                                  best_schedule_constructor_,
                                  length_history_,
                                  pressure_history_)) {
      return;
    }

    // Iterate the policy's per-Recurse snapshot in priority order.
    // The snapshot is computed once at this Recurse entry — inner
    // Recurses produce their own snapshots, so they can't perturb
    // ours. Schedule(node) does an O(log K) lookup into the
    // underlying topo-sorted ready list to find the snapshot's
    // chosen node.
    SmallVector<const ScheduleNode *, 16> ordered_ready;
    Policy::FilterAndSortReadyList(working_schedule_constructor_,
                                    ordered_ready);
    for (const ScheduleNode *node : ordered_ready) {
      working_schedule_constructor_.Schedule(node);
      Recurse();
      working_schedule_constructor_.Unschedule();
      if (should_end_search_) {
        return;
      }
    }
  }

  // Captured at construction so the per-candidate LLVM-tracker
  // verification step can build a GCNUpwardRPTracker against the
  // same MF/LIS the schedule belongs to. Declared before
  // working_schedule_constructor_ so init-list order matches the
  // declaration order (mf_/lis_ have no member-init dependencies
  // but are referenced first in the ctor's init list for clarity).
  const MachineFunction *mf_;
  const LiveIntervals *lis_;

  // Mutable search state; Schedule/Unschedule walk every branch.
  ScheduleConstructor working_schedule_constructor_;

  // Best complete schedule seen. Seeded from the graph's input
  // schedule; replaced whenever working_schedule_constructor_ is
  // IsDone and beats it by Policy::kMetric.
  ScheduleConstructor best_schedule_constructor_;

  // Diagnostic counters — see accessors above.
  int complete_schedules_count_ = 0;
  int best_updates_count_ = 0;
  // Count of times a candidate would have updated best (passed
  // IsBetterThan) but was rejected by LLVM's tracker because its
  // ground-truth occupancy didn't meet the kernel target. Reflects
  // disagreement between our tracker and LLVM's — a non-zero count
  // means our tracker under-counted pressure on that schedule.
  int llvm_tracker_rejections_count_ = 0;

  // Set true by Recurse when Policy::ShouldEndSearch fires, OR by
  // EndSearchIfTimedOut when the region deadline has been reached.
  // Each recursive frame propagates the flag back up by checking
  // it after each child Recurse() returns.
  bool should_end_search_ = false;

  // Region-scope. Sticky flag set when EndSearchIfTimedOut first
  // observes that the region deadline has passed. Read by
  // RegionTimedOut() for telemetry. Region-scope here means it
  // persists across all Run() calls during this DfsSearch's
  // lifetime, matching the deadline that controls it.
  bool region_timed_out_ = false;

  // Stopwatch with both region-scope (lifetime_start, set in ctor
  // and untouched by ResetForReuse) and run-scope
  // (current_run_start, refreshed at the top of every Run())
  // origins. Read via GetCurrentRunElapsedMs and
  // GetRegionElapsedMs. EndSearchIfTimedOut also reads its
  // lifetime elapsed for the timeout check.
  DualRunAndLifetimeStopwatch timing_;

  // The longest schedule the search will accept on this Run().
  // Passed through to Policy::ShouldBoundSearch each visit; the
  // policy combines it with best.length-1 to get a single
  // max-acceptable bound. Default INT_MAX means "no extra
  // constraint," reducing the bound to the existing best-length
  // check. Outer loops that walk a target value rewrite this
  // before each Run().
  int requested_target_length_ = std::numeric_limits<int>::max();

  // History-based-domination table for length pruning. Bound to
  // working_schedule_constructor_'s ScheduledSetTracker and
  // ScheduleLengthTracker. Active only when
  // Policy::kUseLengthHistoryPruning is true; for other policies
  // the construction overhead is small but nonzero
  LengthHistoryTracker length_history_;

  // History-based-domination table for pressure pruning. Bound to
  // working's ScheduledSetTracker for partition keys, and to
  // working/best GCNRegisterTracker for score reads. Active only
  // when Policy::kUsePressureHistoryPruning is true; otherwise
  // constructed but never queried.
  PressureHistoryTracker pressure_history_;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_DFSSEARCH_H
