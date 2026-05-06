//===- SearchPolicies.cpp - Per-pass search policies ---------------------===//

#include "SearchPolicies.h"
#include "ScheduleGraph.h"
#include <algorithm>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

bool DfsMinimizeLengthPolicy::ShouldBoundSearch(
    const ScheduleConstructor &schedule_constructor,
    const ScheduleConstructor &best_schedule_constructor,
    LengthHistoryTracker &length_history,
    PressureHistoryTracker & /*pressure_history*/,
    int target_length) {
  int best_length =
      best_schedule_constructor.GetLengthTracker().GetCurrentCycle();
  int working_lb =
      schedule_constructor.GetLengthTracker().GetLengthLowerBound();
  // Max acceptable length combines the caller-supplied target with
  // the strict-improvement constraint. When target_length =
  // INT_MAX (caller has no extra constraint to apply), this
  // reduces to best_length - 1 and the bound matches the prior
  // `working_lb >= best_length` check.
  int max_acceptable = std::min(target_length, best_length - 1);
  if (working_lb > max_acceptable) {
    return true;
  }
  if (!schedule_constructor.IsAtOrAboveFunctionOccupancyCeiling()) {
    return true;
  }
  if constexpr (kUseLengthHistoryPruning) {
    // Mutating: records the current prefix in length_history when
    // it is NOT dominated. See LengthHistoryTracker class comment.
    if (length_history.IsDominatedElseInsert()) {
      return true;
    }
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
    const ScheduleConstructor &best_schedule_constructor,
    LengthHistoryTracker & /*length_history*/,
    PressureHistoryTracker &pressure_history,
    int /*target_length*/) {
  // Score-bound: working's GetMetricScore(kMetric) is non-
  // increasing as more nodes are scheduled (peak pressure grows
  // monotonically, kMetric is max-direction so its score only
  // drops). If working's score is already <= best's, no
  // completion of working can strictly beat best.
  if (schedule_constructor.GetPressureTracker().GetMetricScore(kMetric) <=
      best_schedule_constructor.GetPressureTracker().GetMetricScore(kMetric)) {
    return true;
  }

  if constexpr (kUsePressureHistoryPruning) {
    // Mutating: records the current state on miss, may also
    // enqueue a fast-forward hint onto DfsSearch's replay queue.
    // See PressureHistoryTracker.
    if (pressure_history.IsDominatedElseRecord()) {
      return true;
    }
  }

  return false;
}

bool DfsMaximizeOccupancyPolicy::ShouldEndSearch(
    const ScheduleConstructor & /*schedule_constructor*/,
    const ScheduleConstructor &best_schedule_constructor) {
  return best_schedule_constructor.IsAtOrAboveFunctionOccupancyCeiling();
}
