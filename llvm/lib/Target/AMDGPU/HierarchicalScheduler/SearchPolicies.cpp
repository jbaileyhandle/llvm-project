//===- SearchPolicies.cpp - Per-pass search policies ---------------------===//

#include "SearchPolicies.h"
#include "ScheduleGraph.h"
#include <algorithm>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

SmallVector<const ScheduleNode *, 16>
DfsMaximizeOccupancyPolicy::PruneAndSortReadyList(
    const ScheduleConstructor &schedule_constructor) {
  SmallVector<const ScheduleNode *, 16> ordered_ready;
  schedule_constructor.GetReadyListSnapshot(ordered_ready);
  std::sort(ordered_ready.begin(), ordered_ready.end(),
            [](const ScheduleNode *a, const ScheduleNode *b) {
              return a->GetId() < b->GetId();
            });
  return ordered_ready;
}

bool DfsMaximizeOccupancyPolicy::ShouldBoundSearch(
    const ScheduleConstructor &schedule_constructor,
    const ScheduleConstructor &best_schedule_constructor) {
  // Continuous-score bound: working's current continuous score is
  // the BEST it'll have at completion (peak pressure only grows
  // as more nodes are scheduled, so score only drops). If working's
  // current score is already <= best's, no completion of working
  // can strictly beat best. Bound.
  //
  // SOUNDNESS NOTE: this relies on the metric being maximize-
  // direction with monotonically-non-improving partial values.
  // True for kMaximizeContinuousRegisterOccupancyScore. A future
  // minimize-direction policy must NOT copy this bound verbatim —
  // partial values for minimize-occupancy IMPROVE as pressure
  // grows, so this bound would over-prune.
  if (schedule_constructor.GetPressureTracker()
          .GetContinuousOccupancyScore() <=
      best_schedule_constructor.GetPressureTracker()
          .GetContinuousOccupancyScore()) {
    return true;
  }

  return false;
}

bool DfsMaximizeOccupancyPolicy::ShouldEndSearch(
    const ScheduleConstructor & /*schedule_constructor*/,
    const ScheduleConstructor &best_schedule_constructor) {
  return best_schedule_constructor.IsAtOrAboveFunctionOccupancyCeiling();
}
