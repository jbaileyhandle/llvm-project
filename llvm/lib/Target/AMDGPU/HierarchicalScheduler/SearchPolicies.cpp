//===- SearchPolicies.cpp - Per-pass search policies ---------------------===//

#include "SearchPolicies.h"
#include "ScheduleGraph.h"
#include <algorithm>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

bool DfsMinimizeLengthPolicy::ShouldBoundSearch(
    const ScheduleConstructor &schedule_constructor,
    const ScheduleConstructor & /*best_schedule_constructor*/,
    LengthHistoryTracker &length_history,
    PressureHistoryTracker & /*pressure_history*/) {
  // Max acceptable schedule length is read straight from working's
  // length tracker, which DfsSearch keeps in sync with
  // min(requested_target_length, best.length - 1) at every event
  // that changes either input. The aggregate-LB check below is the
  // coarse-grained complement to the per-node deadline check
  // implied by the tracker's GetMaxScheduleCycle table.
  const auto &length_tracker = schedule_constructor.GetLengthTracker();
  int max_acceptable = length_tracker.GetMaxAcceptableScheduleLength();
  int working_lb = length_tracker.GetLengthLowerBound();
  if (working_lb > max_acceptable) {
    return true;
  }
  if (!schedule_constructor.RegisterOnlyOccupancyIsAtOrAboveFunctionOccupancyTarget()) {
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
    PressureHistoryTracker &pressure_history) {
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
  return best_schedule_constructor.RegisterOnlyOccupancyIsAtOrAboveFunctionOccupancyTarget();
}
