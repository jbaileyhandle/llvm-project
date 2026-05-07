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

#include <chrono>
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

/// Stopwatch tracking start times at two scopes. Start() updates
/// current_run_start unconditionally, and sets lifetime_start
/// only the first time it's called (so subsequent Start() calls
/// reset the per-run measurement without disturbing the
/// lifetime origin). Both elapsed accessors compute against
/// std::chrono::steady_clock::now() — read them right after
/// the work you want to measure ends; reading later includes
/// idle time. There is no Stop / no captured end value, by
/// design: callers are expected to ask once at the relevant
/// moment.
struct DualRunAndLifetimeStopwatch {
  std::chrono::steady_clock::time_point lifetime_start;
  std::chrono::steady_clock::time_point current_run_start;

  void Start() {
    auto now = std::chrono::steady_clock::now();
    if (lifetime_start.time_since_epoch().count() == 0) {
      lifetime_start = now;
    }
    current_run_start = now;
  }

  int64_t CurrentRunElapsedMs() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - current_run_start)
        .count();
  }

  int64_t LifetimeElapsedMs() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - lifetime_start)
        .count();
  }
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHSTATS_H
