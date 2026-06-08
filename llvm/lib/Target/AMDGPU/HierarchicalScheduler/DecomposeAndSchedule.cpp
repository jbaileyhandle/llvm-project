//===- DecomposeAndSchedule.cpp - Subgraph-scheduling driver --------------===//
//
// Implementation. See DecomposeAndSchedule.h for the contract.
//
//===----------------------------------------------------------------------===//

#include "DecomposeAndSchedule.h"

#include "BfsDpSearch.h"
#include "BfsDpSettings.h"
#include "DfsSearch.h"
#include "ScheduleGraph.h"
#include "Score.h"
#include "ScheduleSubgraph.h"
#include "SearchPolicies.h"
#include "SubgraphFormation.h"
#include "SubgraphInfo.h"
#include "llvm/Analysis/MachineInstrSchedulerConfig.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {
namespace hierarchical_scheduler {

SearchResult DecomposeAndSchedule(
    ScheduleGraph &graph,
    const GCNSubtarget &st,
    const MachineFunction &mf,
    const DecomposeAndScheduleOptions &opts) {
  // Step 1: form subgraphs and install them per opts.mode (proxies for
  // kSerialized, flat for kInterleaved).
  FormSubgraphs(graph, opts.formation, opts.mode);

  // Step 2: schedule each formed subgraph in isolation. The functor
  // decides whether to run a leaf search or recurse.
  for (const std::unique_ptr<SubgraphInfo> &info : graph.GetSubgraphInfos()) {
    ScheduleSubgraph(*info, graph, st, mf, opts.inner_search);
  }

  // Step 3: lock the chosen interiors into `graph` with order-edge
  // chains.
  graph.AddSubgraphOrderEdges();

  // Step 4: run the outer search over the proxied + chained graph.
  return opts.outer_search(graph);
}

namespace {

// 5s wall-clock budget shared by BfsDpWithDfsFallback across both
// stages, both algorithms. Inner is per-subgraph (cumulative cost);
// the same budget caps the outer DFS fallback when BFS-DP outer
// times out. Namespace-scope so the factory's lambdas can read it
// without listing it in their capture lists.
constexpr int64_t kBfsDpWithDfsFallbackTimeoutMs = 5000;

// Integer-metric variant of DfsMaximizeOccupancyPolicy. Used as the
// outer DFS fallback in BfsDpWithDfsFallback so the DFS metric
// matches the outer BFS-DP's integer occupancy metric. Everything
// else (FilterAndSortReadyList, ShouldBoundSearch, ShouldEndSearch,
// MakeFormationPolicy, pressure-history pruning) inherits unchanged
// — only the metric flips. PressureHistoryTracker is constructed
// with this metric, so its pruning reads integer-occupancy scores;
// monotonicity (peak pressure grows, occupancy drops) holds for
// either score, so the prune is sound under either.
class DfsMaximizeIntegerOccupancyPolicy : public DfsMaximizeOccupancyPolicy {
 public:
  static constexpr ScoreRecipe kScoreRecipe =
      score_recipes::kMaximizeRegisterOccupancy;
};

}  // namespace

DecomposeAndScheduleOptions DecomposeAndScheduleOptions::BfsDpWithDfsFallback(
    const GCNSubtarget &st,
    const MachineFunction &mf,
    const LiveIntervals &lis,
    int seed_occupancy,
    const FormationConfig &subgraph_formation,
    OuterSearch outer) {
  DecomposeAndScheduleOptions opts;
  // Formation: realized from the caller-supplied config. Decompose
  // requires a real formation (validated upstream), so the strategy is
  // dom-tree or min-cut, never none.
  opts.formation = SubgraphFormationPolicy::FromStrategy(
      subgraph_formation.strategy, subgraph_formation.min_cut);
  opts.mode = subgraph_formation.mode;

  // Inner: continuous occupancy score, no seed. DFS fallback uses
  // the same continuous-metric policy (DfsMaximizeOccupancyPolicy).
  // Both run on the already-extracted subgraph: we're inside a subgraph
  // extracted by ScheduleSubgraph, and nested formation isn't wired yet.
  opts.inner_search = [&st, &mf, &lis](ScheduleGraph &sub) -> SearchResult {
    BfsDpSettings settings;
    settings.recipe = score_recipes::kMaximizeContinuousRegisterOccupancyScore;
    settings.timeout_ms = kBfsDpWithDfsFallbackTimeoutMs;
    BfsDpSearch bfs(&sub, &st, &mf, settings);
    SearchResult result = bfs.Run();
    // Fraction of the subgraph's layers BFS-DP reached before it
    // finished or bailed — reported on the row whether or not BFS-DP
    // won, so a DFS-fallback row still shows how far BFS-DP got.
    std::optional<float> bfs_pct =
        (100.0f * bfs.GetLevelsExplored()) / sub.Size();
    if (result.schedule.has_value()) {
      result.winner = "bfs";
      result.bfs_pct = bfs_pct;
      return result;
    }
    // BFS-DP returned no schedule. The inner search is unseeded
    // (initial best INT_MIN), so the score-bound prune never empties the
    // frontier — an empty result here means the timeout fired, so DFS
    // rescues unconditionally. Same budget and metric.
    DfsSearch<DfsMaximizeOccupancyPolicy> dfs(
        sub, st, mf, lis,
        /*timeout_ms=*/kBfsDpWithDfsFallbackTimeoutMs);
    SearchResult dfs_result = dfs.Run();
    dfs_result.winner = "dfs";
    dfs_result.bfs_pct = bfs_pct;
    // Keep the BFS-DP throughput on the row even though DFS won.
    dfs_result.bfs_ms = result.bfs_ms;
    dfs_result.bfs_steps = result.bfs_steps;
    return dfs_result;
  };

  // Outer: selected by `outer`. kDfs runs DFS over the proxied graph (its
  // ShouldEndSearch honors the function occupancy target, so an occupancy
  // cap restrains it — BFS-DP would maximize past it). Otherwise BFS-DP:
  // kBfsDpInteger maximizes the integer register-occupancy level seeded with
  // the region's floor (seed_occupancy) so the score-bound prune drops any
  // path that can't strictly beat it; kBfsDpContinuous maximizes the
  // continuous score seeded with the input order's continuous score (the
  // integer floor is a different scale). Each BFS-DP path's DFS fallback uses
  // the policy matching its metric so the fallback's "better" judgement
  // agrees with the BFS-DP it replaces.
  opts.outer_search = [&st, &mf, &lis, seed_occupancy, outer](
                          ScheduleGraph &g) -> SearchResult {
    if (outer == OuterSearch::kDfs) {
      // No BFS-DP: DFS directly, so the occupancy-target "enough" applies.
      // This is a primary search, so it takes the default DFS budget (not
      // the shorter budget the BFS-DP-fallback path shares), matching the
      // non-decompose DFS path.
      DfsSearch<DfsMaximizeOccupancyPolicy> dfs(g, st, mf, lis);
      SearchResult result = dfs.Run();
      result.winner = "dfs";
      return result;
    }
    const bool outer_continuous = (outer == OuterSearch::kBfsDpContinuous);
    BfsDpSettings settings;
    settings.recipe =
        outer_continuous
            ? score_recipes::kMaximizeContinuousRegisterOccupancyScore
            : score_recipes::kMaximizeRegisterOccupancy;
    settings.timeout_ms = kBfsDpWithDfsFallbackTimeoutMs;
    BfsDpSearch bfs(&g, &st, &mf, settings);
    if (outer_continuous) {
      // Score the input order under the continuous metric and use it as
      // the prune floor (SetInitialBestScore extracts the metric score).
      bfs.SetInitialBestScore(g.GetInputScheduleConstructor());
    } else {
      bfs.SetInitialBestScore(seed_occupancy);
    }
    SearchResult result = bfs.Run();
    // Fraction of the proxied graph's layers BFS-DP reached; kept on
    // the row even when DFS rescues, to show how far the outer BFS-DP
    // got before bailing.
    std::optional<float> bfs_pct =
        (100.0f * bfs.GetLevelsExplored()) / g.Size();
    if (result.schedule.has_value()) {
      result.winner = "bfs";
      result.bfs_pct = bfs_pct;
      return result;
    }
    result.bfs_pct = bfs_pct;
    // Only DFS-rescue a timeout. A fully-explored empty result proves
    // (the score-bound prune is sound) that nothing beats the seed, so
    // DFS over the same objective can't either — keep the input order.
    if (result.termination_cause != SearchTerminationCause::kTimedOut) {
      result.winner = "input";
      return result;
    }
    // BFS-DP timed out — fall back to DFS over the proxied + chained
    // graph, with the policy matching the chosen outer metric.
    SearchResult dfs_result;
    if (outer_continuous) {
      DfsSearch<DfsMaximizeOccupancyPolicy> dfs(
          g, st, mf, lis, /*timeout_ms=*/kBfsDpWithDfsFallbackTimeoutMs);
      dfs_result = dfs.Run();
    } else {
      DfsSearch<DfsMaximizeIntegerOccupancyPolicy> dfs(
          g, st, mf, lis, /*timeout_ms=*/kBfsDpWithDfsFallbackTimeoutMs);
      dfs_result = dfs.Run();
    }
    dfs_result.winner = "dfs";
    dfs_result.bfs_pct = bfs_pct;
    // Keep the outer BFS-DP throughput on the row even though DFS won.
    dfs_result.bfs_ms = result.bfs_ms;
    dfs_result.bfs_steps = result.bfs_steps;
    return dfs_result;
  };

  return opts;
}

SearchResult RecursiveDecomposeAndSchedule(
    ScheduleGraph &graph, const GCNSubtarget &st, const MachineFunction &mf,
    const LiveIntervals &lis, int seed_occupancy,
    const FormationConfig &subgraph_formation, OuterSearch outer) {
  // Per-level options: continuous leaf inner search + the composable outer
  // search + the (max_parts-capped) mincut formation. BfsDpWithDfsFallback
  // builds all three; we reuse its inner as the leaf search and override it
  // below for the non-leaf case.
  DecomposeAndScheduleOptions opts =
      DecomposeAndScheduleOptions::BfsDpWithDfsFallback(
          st, mf, lis, seed_occupancy, subgraph_formation, outer);

  // Leaf: a (sub)graph with at most target_subgraph_size scheduling units is
  // exactly what mincut would refuse to split (k < 2), so schedule it
  // directly with the continuous leaf search rather than decomposing.
  const int leaf_size = subgraph_formation.min_cut.target_subgraph_size;
  if (graph.NumSchedulingUnits() <= leaf_size) {
    return opts.inner_search(graph);
  }

  // Non-leaf: recurse on each subgraph instead of leaf-searching it. Each
  // subgraph is strictly smaller than `graph` (mincut yields k >= 2 parts),
  // so the recursion terminates at the leaf size. ScheduleSubgraph runs this
  // inner search on each extracted subgraph before the level's outer search,
  // so the schedule is built bottom-up. Deeper levels order subgraph
  // interiors, so they always use kBfsDpContinuous — `outer` (integer / DFS)
  // is an outermost-only, region-level concept (see header).
  opts.inner_search = [&st, &mf, &lis, seed_occupancy,
                       subgraph_formation](ScheduleGraph &sub) -> SearchResult {
    return RecursiveDecomposeAndSchedule(sub, st, mf, lis, seed_occupancy,
                                         subgraph_formation,
                                         OuterSearch::kBfsDpContinuous);
  };
  return DecomposeAndSchedule(graph, st, mf, opts);
}

}  // namespace hierarchical_scheduler
}  // namespace llvm
