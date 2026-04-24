//===- SearchPolicies.cpp - Per-pass search policies ---------------------===//

#include "SearchPolicies.h"
#include "ScheduleGraph.h"
#include <algorithm>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

bool DfsMinimizeLengthPolicy::ShouldBoundSearch(
    const ScheduleConstructor &schedule_constructor,
    const ScheduleConstructor &best_schedule_constructor) {
  int best_length =
      best_schedule_constructor.GetLengthTracker().GetCurrentCycle();
  int working_lb =
      schedule_constructor.GetLengthTracker().GetLengthLowerBound();
  if (working_lb >= best_length) {
    return true;
  }
  if (!schedule_constructor.IsAtOrAboveFunctionOccupancyCeiling()) {
    return true;
  }
  return false;
}

bool DfsMinimizeLengthPolicy::ShouldEndSearch(
    const ScheduleConstructor & /*schedule_constructor*/,
    const ScheduleConstructor &best_schedule_constructor) {
  int best_length =
      best_schedule_constructor.GetLengthTracker().GetCurrentCycle();
  int floor = best_schedule_constructor.GetGraph().GetGraphLengthFloor();
  return best_length <= floor;
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
