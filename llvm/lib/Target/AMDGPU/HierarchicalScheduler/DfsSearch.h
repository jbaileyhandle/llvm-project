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
#include "SubgraphFormation.h"
#include "llvm/ADT/SmallVector.h"
#include <chrono>
#include <limits>

namespace llvm {
class GCNSubtarget;
class MachineFunction;
class LiveIntervals;

namespace hierarchical_scheduler {

// Policy contract (the static methods below — Policy classes don't
// inherit, they just have these by name):
//   static constexpr ScheduleMetric kMetric;
//   static bool ReadyCompare(const ScheduleNode *a,
//                            const ScheduleNode *b);
//   static bool ShouldBoundSearch(
//       const ScheduleConstructor &schedule_constructor,
//       const ScheduleConstructor &best_schedule_constructor,
//       LengthHistoryTracker &length_history,
//       PressureHistoryTracker &pressure_history,
//       int target_length);
//   static bool ShouldEndSearch(
//       const ScheduleConstructor &schedule_constructor,
//       const ScheduleConstructor &best_schedule_constructor);
//   static SubgraphFormationPolicy MakeFormationPolicy();
//
// MakeFormationPolicy is provided with a default-empty implementation
// by SearchPolicyBase (see SearchPolicies.h); concrete policies that
// want subgraph formation override it. An empty SubgraphFormationPolicy
// makes the formation step a one-call no-op (FormSubgraphs early-
// returns on empty pipeline).
//
// ReadyCompare must be a strict total order; it drives the sort order
// of ScheduleConstructor's ready list. DfsSearch iterates the ready
// list by index, relying on the round-trip stability guarantee
// documented on ScheduleConstructor.
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
      : working_schedule_constructor_(
            MaybeFormSubgraphs(graph, form_subgraphs),
            st, mf, lis, &Policy::ReadyCompare),
        best_schedule_constructor_(graph.GetInputScheduleConstructor()),
        // length_history_ binds to working_'s trackers. Constructed
        // unconditionally; queried only when Policy::
        // kUseLengthHistoryPruning is true (the `if constexpr` in
        // Recurse dead-strips the consult/insert otherwise).
        length_history_(
            &working_schedule_constructor_.GetScheduledSetTracker(),
            &working_schedule_constructor_.GetLengthTracker()),
        // pressure_history_ binds to working_'s scheduled-set
        // tracker for partition keys, and to working_'s pressure
        // tracker for the no-arg score read. The metric matches
        // Policy::kMetric so the tracker reads values consistent
        // with what the search optimizes.
        pressure_history_(
            &working_schedule_constructor_.GetScheduledSetTracker(),
            &working_schedule_constructor_.GetPressureTracker(),
            Policy::kMetric),
        // Region-scope timing: start time captured at construction
        // so per-region telemetry can report elapsed time across
        // all Run() calls. Deadline is a fixed offset from the
        // start; EndSearchIfTimedOut compares against this and is
        // the only time-based enforcement. Both unaffected by
        // ResetForReuse, so they span every iteration of any outer
        // loop driving this DfsSearch.
        region_start_time_(std::chrono::steady_clock::now()),
        region_deadline_(region_start_time_ +
                         std::chrono::seconds(
                             Policy::kTimeoutSecondsPerRegion)) {
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

  // Runs DFS, returns a copy of the best schedule found. Captures
  // run_start_time_ so telemetry can compute elapsed time for
  // this Run().
  ScheduleConstructor Run() {
    run_start_time_ = std::chrono::steady_clock::now();
    Recurse();
    return best_schedule_constructor_;
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
  // multiple Run() calls (see ScheduleRegionForMinimumLength).
  void ResetForReuse(int target_length) {
    working_schedule_constructor_.Reset();
    length_history_.Reset();
    pressure_history_.Reset();
    best_schedule_constructor_ =
        working_schedule_constructor_.GetGraph().GetInputScheduleConstructor();
    should_end_search_ = false;
    target_length_ = target_length;
  }

  // True iff the region-level deadline (set in the ctor as
  // Policy::kTimeoutSecondsPerRegion past construction time) has
  // been reached at some point during this DfsSearch's lifetime,
  // and a Recurse aborted as a result. Sticky once set. The
  // single-bool shape is correct for this concept: once the
  // region deadline fires, the search exits and we don't start
  // another Run(), so a per-run vs lifetime distinction wouldn't
  // add information. A future per-run deadline (separate concept,
  // separate field) would get a DualRunAndLifetimeFlag because
  // its per-run and lifetime values can genuinely diverge.
  bool RegionTimedOut() const { return region_timed_out_; }

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

  // If the region deadline has been reached, set region_timed_out_
  // (for telemetry) and should_end_search_ (for propagation through
  // the recursion), and return true so the caller can unwind.
  // Reuses should_end_search_ for propagation: each recursive frame
  // already checks it after every child returns and unwinds when
  // set, so the search exits cleanly and Run() returns the seeded
  // baseline (or anything better DFS managed to find before the
  // wall hit). steady_clock::now() on Linux is vDSO-backed (~20ns),
  // so calling on every Recurse() entry is cheap.
  bool EndSearchIfTimedOut() {
    if (std::chrono::steady_clock::now() < region_deadline_) {
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
      if (working_schedule_constructor_.IsBetterThan(
              best_schedule_constructor_, Policy::kMetric)) {
        best_schedule_constructor_ = working_schedule_constructor_;
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
                                  pressure_history_,
                                  target_length_)) {
      return;
    }

    // Iterate the live ready list by index. Schedule+Unschedule is a
    // round-trip: after Unschedule the ready list has identical
    // contents AND index layout (strict-total-order comparator + sort
    // maintenance), so ++i points to the next sibling.
    int ready_size =
        static_cast<int>(working_schedule_constructor_.GetReadyList().size());
    for (int i = 0; i < ready_size; ++i) {
      working_schedule_constructor_.ScheduleByIndex(i);
      Recurse();
      working_schedule_constructor_.Unschedule();
      if (should_end_search_) {
        return;
      }
    }
  }

  // Mutable search state; Schedule/Unschedule walk every branch.
  ScheduleConstructor working_schedule_constructor_;

  // Best complete schedule seen. Seeded from the graph's input
  // schedule; replaced whenever working_schedule_constructor_ is
  // IsDone and beats it by Policy::kMetric.
  ScheduleConstructor best_schedule_constructor_;

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

  // Region-scope. Wall-clock instant when this DfsSearch was
  // constructed — i.e., when work on this region began.
  // Initialized once in the ctor; not modified afterward.
  std::chrono::steady_clock::time_point region_start_time_;

  // Region-scope. Wall-clock instant past which the search must
  // terminate. Initialized once in the ctor as
  // region_start_time_ + Policy::kTimeoutSecondsPerRegion.
  // EndSearchIfTimedOut compares against this. Not reset by
  // ResetForReuse, so all Run() calls within this DfsSearch's
  // lifetime share the same cap.
  std::chrono::steady_clock::time_point region_deadline_;

  // Run-scope. Wall-clock instant captured at the top of the
  // current Run() invocation. Reset on every Run() entry.
  std::chrono::steady_clock::time_point run_start_time_;

  // The longest schedule the search will accept on this Run().
  // Passed through to Policy::ShouldBoundSearch each visit; the
  // policy combines it with best.length-1 to get a single
  // max-acceptable bound. Default INT_MAX means "no extra
  // constraint," reducing the bound to the existing best-length
  // check. Outer loops that walk a target value rewrite this
  // before each Run().
  int target_length_ = std::numeric_limits<int>::max();

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
