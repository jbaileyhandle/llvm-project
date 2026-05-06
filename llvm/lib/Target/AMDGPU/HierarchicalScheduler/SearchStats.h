//===- SearchStats.h - Dual run-and-lifetime stat helpers ---*- C++ -*-===//
//
// Tiny helpers for stats that need to be tracked at two
// granularities simultaneously:
//   - current_run: cleared at the start of each search Run()
//     (via the wrapping object's Reset / ResetForReuse /
//     similar). Useful for per-iteration telemetry.
//   - lifetime: never cleared; reflects every event since the
//     wrapping object's construction. Useful for region-aggregate
//     telemetry across multiple Run() calls (one DfsSearch
//     covers one region).
//
// Both fields are updated at every event site. There is no
// snapshot / derive scheme — both values are always current at
// any read point.
//
// `DualRunAndLifetimeCounter` is for integer counters (e.g.,
// prune counts, schedule-call counts).
// `DualRunAndLifetimeFlag` is for booleans that go true once and
// stay true within their scope (sticky); ResetCurrentRun clears
// only the per-run flag.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHSTATS_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHSTATS_H

#include <cstdint>

namespace llvm {
namespace hierarchical_scheduler {

/// Pair of integer counters at two scopes: one cleared on
/// ResetCurrentRun, one persisting across Resets. Increment()
/// bumps both. Fields are public so callers can read directly
/// without going through passthrough accessors.
struct DualRunAndLifetimeCounter {
  int64_t current_run = 0;
  int64_t lifetime = 0;

  void Increment() {
    ++current_run;
    ++lifetime;
  }

  void ResetCurrentRun() { current_run = 0; }
};

/// Pair of sticky-true booleans at two scopes: one cleared on
/// ResetCurrentRun, one persisting across Resets. Set() drives
/// both true; once true a flag stays true within its scope (no
/// Clear path other than ResetCurrentRun on the per-run side).
struct DualRunAndLifetimeFlag {
  bool current_run = false;
  bool lifetime = false;

  void Set() {
    current_run = true;
    lifetime = true;
  }

  void ResetCurrentRun() { current_run = false; }
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHSTATS_H
