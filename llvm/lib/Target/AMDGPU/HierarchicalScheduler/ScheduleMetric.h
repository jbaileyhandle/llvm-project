//===- ScheduleMetric.h - Schedule comparison criterion -------*- C++ -*-===//
//
// The criterion by which two ScheduleConstructor states are compared.
// Lives in its own small header so consumers that need only the enum
// (e.g., GCNRegisterTracker's score-dispatch helper) can include it
// without pulling in ScheduleConstructor.h.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULEMETRIC_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULEMETRIC_H

//========================================================================================
// jbaile
//========================================================================================

namespace llvm {
namespace hierarchical_scheduler {

/// Criterion by which two ScheduleConstructor states are compared.
/// Used with ScheduleConstructor::IsBetterThan. Names are explicit
/// about direction (kMaximize*, kMinimize*) so the call site doesn't
/// have to remember which way each metric is "better".
enum class ScheduleMetric {
  /// Integer register occupancy (GetRegisterOnlyOccupancy).
  /// Coarse — schedules in the same occupancy bracket tie.
  kMaximizeRegisterOccupancy,

  /// Continuous register occupancy score (GetContinuousOccupancyScore).
  /// Smooth within brackets — useful when search needs to see
  /// progress toward the next higher bracket.
  kMaximizeContinuousRegisterOccupancyScore,

  /// Current schedule length in cycles.
  kMinimizeScheduleLength,

  /// Current schedule length in cycles, taken in the opposite
  /// direction: higher is better. Drives a "maximize length"
  /// search — useful as a control / worst-legal-schedule baseline
  /// for comparing against the length-min objective. Still subject
  /// to the occupancy floor (the policy that uses this metric does
  /// not relax the function-wide register-occupancy constraint).
  kMaximizeScheduleLength,

  /// Length-min with ILP score as the same-length tiebreaker.
  /// Length asc primary; among same-length, higher locked-in
  /// IlpTracker::GetIlpScore wins; among same-length-same-ILP,
  /// higher continuous register occupancy wins (tertiary tiebreak
  /// for stability / pressure-headroom preference). Used by
  /// DfsMinimizeLengthRefineIlpPolicy — that policy opts into the
  /// bound relaxation that produces same-length completions for
  /// IsBetterThan to choose among.
  kMinimizeScheduleLengthRefineIlp,

  /// Inverted register occupancy: lower GetRegisterOnlyOccupancy is
  /// "better." TEST-ONLY — used to drive a search toward worse
  /// register occupancy so we can verify search infrastructure
  /// (DFS, etc.) actually explores and selects against the input
  /// baseline. Not a useful production metric.
  kMinimizeRegisterOccupancy,

  /// Inverted continuous register occupancy score: lower
  /// GetContinuousOccupancyScore is "better." TEST-ONLY, parallel
  /// to kMinimizeRegisterOccupancy but uses the smooth score, so
  /// schedules that differ in within-bracket pressure (not just
  /// integer occupancy) are distinguishable. Useful for verifying
  /// DFS picks WORSE schedules even when no integer-occupancy
  /// cliff is crossed.
  kMinimizeContinuousRegisterOccupancyScore,
};

} // namespace hierarchical_scheduler
} // namespace llvm

//========================================================================================

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULEMETRIC_H
