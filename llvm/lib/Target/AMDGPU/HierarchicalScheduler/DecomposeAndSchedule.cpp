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
#include "ScheduleMetric.h"
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
  if (opts.mode != SubgraphScheduleMode::kSerialized) {
    report_fatal_error(
        "DecomposeAndSchedule: only kSerialized mode is currently "
        "supported");
  }

  // Step 1: form subgraphs (and insert their proxies).
  FormSubgraphs(graph, opts.formation);

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
  static constexpr ScheduleMetric kMetric =
      ScheduleMetric::kMaximizeRegisterOccupancy;
};

}  // namespace

DecomposeAndScheduleOptions DecomposeAndScheduleOptions::BfsDpWithDfsFallback(
    const GCNSubtarget &st,
    const MachineFunction &mf,
    const LiveIntervals &lis,
    int seed_occupancy) {
  DecomposeAndScheduleOptions opts;
  // Formation: min-cut (dagP) when the MinCutFormation option is set,
  // otherwise the single-splitter top-down pipeline (matching
  // DfsMaximizeOccupancyPolicy::MakeFormationPolicy()), which brackets
  // consumer-side pressure peaks within each subgraph rather than letting
  // unrelated work spread peaks across the region.
  opts.formation =
      MachineInstrSchedulerConfig::GetConfig().HasSchedulingOption(
          MachineInstrSchedulerConfig::SchedulerOption::MinCutFormation)
          ? SubgraphFormationPolicy::MinCut()
          : SubgraphFormationPolicy::TopDownSingleSplitterOnly();
  opts.mode = SubgraphScheduleMode::kSerialized;

  // Inner: continuous occupancy score, no seed. DFS fallback uses
  // the same continuous-metric policy (DfsMaximizeOccupancyPolicy).
  // Both pass form_subgraphs=false: we're already inside a subgraph
  // extracted by ScheduleSubgraph, and nested formation isn't wired
  // yet (see §3.4 of AMDGPUSubgraphSchedulingDesign.md).
  opts.inner_search = [&st, &mf, &lis](ScheduleGraph &sub) -> SearchResult {
    BfsDpSettings settings;
    settings.metric = ScheduleMetric::kMaximizeContinuousRegisterOccupancyScore;
    settings.timeout_ms = kBfsDpWithDfsFallbackTimeoutMs;
    BfsDpSearch bfs(&sub, &st, &mf, settings);
    SearchResult result = bfs.Run();
    if (result.schedule.has_value()) {
      return result;
    }
    // BFS-DP returned no schedule — fall back to DFS at the same
    // budget and metric.
    DfsSearch<DfsMaximizeOccupancyPolicy> dfs(
        sub, st, mf, lis, /*form_subgraphs=*/false,
        /*timeout_ms=*/kBfsDpWithDfsFallbackTimeoutMs);
    return dfs.Run();
  };

  // Outer: integer occupancy score, seeded with the function-wide
  // occupancy floor. BFS-DP's score-bound prune drops any partition
  // path that can't strictly beat the floor. DFS fallback uses the
  // integer-metric variant so the search's "better" judgement
  // matches the BFS-DP it's replacing.
  opts.outer_search = [&st, &mf, &lis, seed_occupancy](
                          ScheduleGraph &g) -> SearchResult {
    BfsDpSettings settings;
    settings.metric = ScheduleMetric::kMaximizeRegisterOccupancy;
    settings.timeout_ms = kBfsDpWithDfsFallbackTimeoutMs;
    BfsDpSearch bfs(&g, &st, &mf, settings);
    bfs.SetInitialBestScore(seed_occupancy);
    SearchResult result = bfs.Run();
    if (result.schedule.has_value()) {
      return result;
    }
    // BFS-DP returned no schedule — fall back to DFS. See inner's
    // comment for the form_subgraphs=false rationale; here the graph
    // is the proxied + chained one DecomposeAndSchedule has already
    // formed, so re-forming would double-form.
    DfsSearch<DfsMaximizeIntegerOccupancyPolicy> dfs(
        g, st, mf, lis, /*form_subgraphs=*/false,
        /*timeout_ms=*/kBfsDpWithDfsFallbackTimeoutMs);
    return dfs.Run();
  };

  return opts;
}

}  // namespace hierarchical_scheduler
}  // namespace llvm
