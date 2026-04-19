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
    const ScheduleConstructor & /*schedule_constructor*/,
    const ScheduleConstructor & /*best_schedule_constructor*/) {
  return false;
}

bool DfsMaximizeOccupancyPolicy::ShouldEndSearch(
    const ScheduleConstructor & /*schedule_constructor*/,
    const ScheduleConstructor &best_schedule_constructor) {
  return best_schedule_constructor.IsAtOrAboveFunctionOccupancyCeiling();
}
