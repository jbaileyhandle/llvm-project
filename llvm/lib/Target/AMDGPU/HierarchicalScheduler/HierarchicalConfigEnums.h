//===- HierarchicalConfigEnums.h - HS config axis enums ---------*- C++ -*-===//
//
// The plain enum classes for the HierarchicalScheduler's config axes: the
// search algorithm, the occupancy metric, and the length objective. Split out
// from HierarchicalConfig.h so low-level headers (e.g. BfsDpSettings.h) can name
// an OccupancyPolicy without pulling in HierarchicalConfig.h's heavy transitive
// includes (SubgraphFormation.h -> ScheduleGraph.h, DominatorTree.h, ...), which
// are needed only by the config *structs*, not these enums. HierarchicalConfig.h
// includes this and adds the structs.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_HIERARCHICALCONFIGENUMS_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_HIERARCHICALCONFIGENUMS_H

namespace llvm {
namespace hierarchical_scheduler {

/// `search` axis (occupancy pass only): the base scheduling algorithm.
///   kDfs      - depth-first occupancy search.
///   kBfsDp    - BFS / dynamic-programming partition search; may bail
///               (timeout / score-bound) and return no schedule.
///   kBfsDpDfs - BFS-DP with a DFS fallback, so a schedule is always
///               produced.
enum class Search { kDfs, kBfsDp, kBfsDpDfs };

/// Occupancy-pass primary metric. Applies to both flat and decompose-outer
/// search; inner per-subgraph searches in decompose are always continuous.
///   kContinuousOccupancy                 - continuous register-occupancy score.
///   kIntegerOccupancy                    - integer register-occupancy level.
///   kIntegerOccupancyRefineSpillArea     - integer occupancy with VGPR spill
///                                          area as a same-occupancy tiebreak.
///   kContinuousOccupancyRefineSpillArea  - continuous occupancy score with
///                                          VGPR spill area as a same-score
///                                          tiebreak. Finer primary than the
///                                          integer variant.
/// Validated at Build():
///   kIntegerOccupancyRefineSpillArea and kContinuousOccupancyRefineSpillArea
///   both require search=dfs (BFS-DP is not equipped for area today).
///   max_occ_above_input requires search=dfs (BFS-DP ignores the cap).
enum class OccupancyPolicy {
  kContinuousOccupancy,
  kIntegerOccupancy,
  kIntegerOccupancyRefineSpillArea,
  kContinuousOccupancyRefineSpillArea,
};

/// `policy` axis (length pass only): the length pass's objective.
///   kMin                - minimize schedule length (base).
///   kMinRefineIlp       - minimize, then refine ILP among same-length
///                         completions.
///   kMinRefineOccupancy - minimize, then refine occupancy likewise.
///   kMax                - maximize length (control / worst-legal
///                         baseline).
enum class LengthPolicy {
  kMin,
  kMinRefineIlp,
  kMinRefineOccupancy,
  // length-min with a hard upper bound on VGPR spill area equal to
  // the input baseline's accumulated spill area. Use after the
  // occupancy pass has nailed spill: this prevents length-min from
  // making spill worse while chasing shorter schedules.
  kMinBoundedSpillSignals,
  kMax,
};

/// `min_adjusted_length.region_weighting` axis: how the min-adjusted-
/// length pass weights each region inside BOTH sums of its score
/// (issue floor and wave lifetime) — i.e., its estimate of how many
/// times a wave executes the region.
///   kNone      - every region weight 1 (trip-count-blind).
///   kLoopDepth - weight = loop_weight_base ^ loop_depth(region): a
///                tunable, auditable stand-in for unknown trip counts,
///                so hot inner-loop regions outvote cold straight-line
///                code in the tier decision. (A future `mbfi` value —
///                MachineBlockFrequencyInfo static frequencies — is
///                the upgrade path if PGO arrives or branchy kernels
///                are shown to mispick tiers; deliberately not
///                implemented while its ~31^depth guess would be both
///                untunable and harder to audit than this one.)
enum class RegionWeighting { kNone, kLoopDepth };

/// `min_adjusted_length.tie_break` axis: which occupancy tier wins when
/// two tiers tie on the min-adjusted-length score. Ties are common —
/// issue-bound regions score issue_slots at every tier — so this choice
/// carries real weight.
///   kHighestOccupancy - more waves dampen a latency misestimate
///                       (insurance against model error).
///   kLowestOccupancy  - fewer waves means a bigger per-wave register
///                       budget, which lowers downstream register-
///                       allocator pressure the score cannot see
///                       (global live ranges spill without any region
///                       exceeding its per-region budget).
enum class TieBreak { kHighestOccupancy, kLowestOccupancy };

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_HIERARCHICALCONFIGENUMS_H
