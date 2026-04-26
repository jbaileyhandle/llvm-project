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

#include "ScheduleConstructor.h"
#include "ScheduleGraph.h"
#include "SubgraphFormation.h"
#include "llvm/ADT/SmallVector.h"

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
//       const ScheduleConstructor &best_schedule_constructor);
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
        best_schedule_constructor_(graph.GetInputScheduleConstructor()) {
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

  // Runs DFS, returns a copy of the best schedule found.
  ScheduleConstructor Run() {
    Recurse();
    return best_schedule_constructor_;
  }

  // Number of Schedule/ScheduleByIndex calls made on the working
  // constructor during the most recent Run() — a direct measure of
  // search effort. Compare to the region's node count: equal means
  // a single linear pass with no backtracking; N * K means roughly
  // K average orderings explored per node. Does not distinguish
  // pruned vs full subtrees; it just counts actual scheduling
  // operations performed.
  int64_t GetScheduleCallCount() const {
    return working_schedule_constructor_.GetScheduleCallCount();
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

  void Recurse() {
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

    if (Policy::ShouldBoundSearch(working_schedule_constructor_,
                                  best_schedule_constructor_)) {
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

  // Set true by Recurse when Policy::ShouldEndSearch fires. Each
  // recursive frame propagates the flag back up by checking it
  // after each child Recurse() returns.
  bool should_end_search_ = false;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_DFSSEARCH_H
