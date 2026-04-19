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
#include "llvm/ADT/SmallVector.h"

namespace llvm {
class GCNSubtarget;
class MachineFunction;
class LiveIntervals;

namespace hierarchical_scheduler {

// Policy contract:
//   static constexpr ScheduleMetric kMetric;
//   static SmallVector<const ScheduleNode*, 16> PruneAndSortReadyList(
//       const ScheduleConstructor &schedule_constructor);
//   static bool ShouldBoundSearch(
//       const ScheduleConstructor &schedule_constructor,
//       const ScheduleConstructor &best_schedule_constructor);
template <typename Policy>
class DfsSearch {
 public:
  DfsSearch(const ScheduleGraph &graph, const GCNSubtarget &st,
            const MachineFunction &mf, const LiveIntervals &lis)
      : working_schedule_constructor_(graph, st, mf, lis),
        best_schedule_constructor_(graph.GetInputScheduleConstructor()) {
    // best_schedule_constructor_ is copy-constructed from the
    // graph's input schedule (built by BuildFromSUnits as Phase 4).
    // That gives us a complete valid schedule matching the region's
    // current MF order before any DFS step runs, so DFS guarantees
    // non-regression against the input MF order — no separate
    // outer-level comparison needed.
  }

  // Runs DFS, returns a copy of the best schedule found.
  ScheduleConstructor Run() {
    Recurse();
    return best_schedule_constructor_;
  }

 private:
  void Recurse() {
    if (working_schedule_constructor_.IsDone()) {
      if (working_schedule_constructor_.IsBetterThan(
              best_schedule_constructor_, Policy::kMetric)) {
        best_schedule_constructor_ = working_schedule_constructor_;
      }
      return;
    }

    if (Policy::ShouldBoundSearch(working_schedule_constructor_,
                                  best_schedule_constructor_)) {
      return;
    }

    SmallVector<const ScheduleNode *, 16> ordered_ready =
        Policy::PruneAndSortReadyList(working_schedule_constructor_);
    for (const ScheduleNode *node : ordered_ready) {
      working_schedule_constructor_.Schedule(node);
      Recurse();
      working_schedule_constructor_.Unschedule();
    }
  }

  // Mutable search state; Schedule/Unschedule walk every branch.
  ScheduleConstructor working_schedule_constructor_;

  // Best complete schedule seen. Seeded with topo order; replaced
  // whenever working_schedule_constructor_ is IsDone and beats it by
  // Policy::kMetric.
  ScheduleConstructor best_schedule_constructor_;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_DFSSEARCH_H
