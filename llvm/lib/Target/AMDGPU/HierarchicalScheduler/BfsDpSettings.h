//===- BfsDpSettings.h - Configuration for the BFS-DP search ----*- C++ -*-===//
//
// Settings bundle for the BFS-DP partition search (PartitionDag /
// BfsDpSearch). A plain value struct passed by value to their
// constructors: the search has no policy-varying *behavior* to
// inject, only scalar configuration, so this is a settings struct
// rather than a template policy. A default-constructed
// BfsDpSettings{} is the neutral configuration (no timeout); the
// occupancy pass sets timeout_ms from occupancy.search.timeout.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_BFSDPSETTINGS_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_BFSDPSETTINGS_H

#include "HierarchicalConfigEnums.h"     // OccupancyPolicy
#include "Score.h"
#include "llvm/Support/ErrorHandling.h" // llvm_unreachable
#include <cstdint>
#include <optional>

namespace llvm {
namespace hierarchical_scheduler {

/// Configuration for a BFS-DP partition search, passed by value to
/// the PartitionDag and BfsDpSearch constructors. The default
/// member initializers below are the neutral configuration —
/// BfsDpSettings{} — which shakedowns use (sometimes overriding
/// `metric`).
struct BfsDpSettings {
  /// Per-edge score recipe. Must satisfy IsRegisterPeakMetricOnly()
  /// (single-slot register-peak); PartitionDag's ctor fatal-errors
  /// otherwise. See PartitionDag's ctor for what each supported
  /// recipe means for the search and its pruning.
  ScoreRecipe recipe = score_recipes::kMaximizeRegisterOccupancy;

  /// Per-region wall-clock budget for PartitionDag::Build(), in
  /// milliseconds. nullopt (the default) means no timeout — Build
  /// runs the score-bound-pruned search to exhaustion. Shakedowns
  /// omit it so the search runs to completion: the BFS-DP-vs-DFS
  /// comparison needs the exhaustive result, and a timeout could
  /// abort Build with no schedule at all.
  std::optional<int64_t> timeout_ms = std::nullopt;

  /// BFS-DP settings for the occupancy pass: maximize register occupancy under
  /// `policy`'s metric (continuous score vs integer level), with `timeout_ms`
  /// (nullopt = no timeout). Flat BFS-DP and decompose-outer share this; the
  /// refine-spill-area policies are DFS-only, so they can't reach here.
  static BfsDpSettings ForOccupancy(OccupancyPolicy policy,
                                    std::optional<int64_t> timeout_ms) {
    BfsDpSettings settings;
    settings.timeout_ms = timeout_ms;
    switch (policy) {
    case OccupancyPolicy::kContinuousOccupancy:
      settings.recipe =
          score_recipes::kMaximizeContinuousRegisterOccupancyScore;
      break;
    case OccupancyPolicy::kIntegerOccupancy:
      settings.recipe = score_recipes::kMaximizeRegisterOccupancy;
      break;
    case OccupancyPolicy::kIntegerOccupancyRefineSpillArea:
    case OccupancyPolicy::kContinuousOccupancyRefineSpillArea:
      llvm_unreachable("refine-spill-area policies require search=dfs");
    }
    return settings;
  }
};

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_BFSDPSETTINGS_H
