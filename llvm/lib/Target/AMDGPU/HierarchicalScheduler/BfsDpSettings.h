//===- BfsDpSettings.h - Configuration for the BFS-DP search ----*- C++ -*-===//
//
// Settings bundle for the BFS-DP partition search (PartitionDag /
// BfsDpSearch). A plain value struct passed by value to their
// constructors: the search has no policy-varying *behavior* to
// inject, only scalar configuration, so this is a settings struct
// rather than a template policy. A default-constructed
// BfsDpSettings{} is the neutral configuration; ForOccupancyPass()
// is the preset the occupancy-maximization pass uses.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_BFSDPSETTINGS_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_BFSDPSETTINGS_H

#include "Score.h"
#include <cstdint>
#include <optional>

namespace llvm {
namespace hierarchical_scheduler {

/// Configuration for a BFS-DP partition search, passed by value to
/// the PartitionDag and BfsDpSearch constructors. The default
/// member initializers below are the neutral configuration —
/// BfsDpSettings{} — which shakedowns use (sometimes overriding
/// `metric`). ForOccupancyPass() is the production preset.
struct BfsDpSettings {
  /// Per-edge score function. Must be a MAX-direction metric
  /// (higher = better); PartitionDag's ctor fatal-errors on any
  /// other value. See PartitionDag's ctor for what each supported
  /// metric means for the search and its pruning.
  ScheduleMetric metric = ScheduleMetric::kMaximizeRegisterOccupancy;

  /// Per-region wall-clock budget for PartitionDag::Build(), in
  /// milliseconds. nullopt (the default) means no timeout — Build
  /// runs the score-bound-pruned search to exhaustion. Shakedowns
  /// omit it so the search runs to completion: the BFS-DP-vs-DFS
  /// comparison needs the exhaustive result, and a timeout could
  /// abort Build with no schedule at all.
  std::optional<int64_t> timeout_ms = std::nullopt;

  /// Preset for the occupancy-maximization pass: the integer-
  /// occupancy metric with a 10s per-region budget. The budget
  /// matches the production DFS occupancy pass's default
  /// (DfsSearch's ctor `timeout_ms` default of 10000) so the
  /// BfsDpForOccupancy misched.txt toggle compares the two search
  /// strategies under the same wall-clock budget.
  static constexpr BfsDpSettings ForOccupancyPass() {
    return BfsDpSettings{ScheduleMetric::kMaximizeRegisterOccupancy,
                         /*timeout_ms=*/10000};
  }
};

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_BFSDPSETTINGS_H
