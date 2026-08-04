//===- DecomposeAndSchedule.cpp - Subgraph-scheduling driver --------------===//
//
// Implementation. See DecomposeAndSchedule.h for the contract.
//
//===----------------------------------------------------------------------===//

#include "DecomposeAndSchedule.h"

#include "BfsDpSearch.h"
#include "BfsDpSettings.h"
#include "DfsOccupancyDispatch.h"
#include "DfsSearch.h"
#include "OccupancySearchDispatch.h"
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

DecomposeAndScheduleOptions DecomposeAndScheduleOptions::Make(
    const GCNSubtarget &st,
    const MachineFunction &mf,
    const LiveIntervals &lis,
    int seed_occupancy,
    const FormationConfig &subgraph_formation,
    OccupancyPolicy outer_policy,
    Search outer_search,
    std::optional<int64_t> outer_timeout_us,
    std::optional<int64_t> outer_fallback_us,
    Search inner_search,
    std::optional<int64_t> inner_timeout_us,
    std::optional<int64_t> inner_fallback_us) {
  DecomposeAndScheduleOptions opts;
  // Formation: realized from the caller-supplied config. Decompose
  // requires a real formation (validated upstream), so the strategy is
  // dom-tree or min-cut, never none.
  opts.formation = SubgraphFormationPolicy::FromStrategy(
      subgraph_formation.strategy, subgraph_formation.min_cut);
  opts.mode = subgraph_formation.mode;

  // Inner (within-subgraph "make"): objective is always continuous occupancy
  // (hardcoded); strategy and budgets come from the inner config. Unseeded (no
  // prune floor) and silent (decompose logs at a higher level), so all three
  // callbacks are no-ops.
  opts.inner_search = [&st, &mf, &lis, inner_search, inner_timeout_us,
                       inner_fallback_us](ScheduleGraph &sub) -> SearchResult {
    return RunOccupancySearch(
        /*kind=*/inner_search,
        /*policy=*/OccupancyPolicy::kContinuousOccupancy, sub, st, /*mf=*/mf,
        /*lis=*/lis,
        // Per-instruction budget (subgraph_size * rate, halved for decompose)
        // overrides the flat inner timeout when set. The fallback is the
        // BFS-DP DFS-rescue budget, never used under a per-instruction budget
        // (dfs-only), so it stays flat.
        /*primary_timeout_us=*/
        HierarchicalConfig::Get().occupancy.EffectiveTimeout(sub.Size(),
                                                             inner_timeout_us),
        /*fallback_timeout_us=*/inner_fallback_us,
        /*seed_bfs=*/[](BfsDpSearch &) {},
        /*bfs_after_run=*/[](BfsDpSearch &, SearchResult &) {},
        /*dfs_after_run=*/[](auto &, SearchResult &) {});
  };

  // Outer (among-subgraphs): strategy/objective/budgets from the outer config.
  // outer_search is kDfs or kBfsDpDfs (kBfsDp is rejected upstream). BFS-DP
  // seeds the continuous input score or the integer target occupancy; silent.
  opts.outer_search = [&st, &mf, &lis, seed_occupancy, outer_policy,
                       outer_search, outer_timeout_us,
                       outer_fallback_us](ScheduleGraph &g) -> SearchResult {
    const bool outer_continuous =
        outer_policy == OccupancyPolicy::kContinuousOccupancy;
    return RunOccupancySearch(
        /*kind=*/outer_search, /*policy=*/outer_policy, g, st, /*mf=*/mf,
        /*lis=*/lis,
        // Per-instruction budget (region_size * rate, halved for decompose)
        // overrides the flat outer timeout when set; fallback stays flat.
        /*primary_timeout_us=*/
        HierarchicalConfig::Get().occupancy.EffectiveTimeout(g.Size(),
                                                             outer_timeout_us),
        /*fallback_timeout_us=*/outer_fallback_us,
        /*seed_bfs=*/
        [&](BfsDpSearch &bfs) {
          if (outer_continuous) {
            bfs.SetInitialBestScore(g.GetInputScheduleConstructor());
          } else {
            bfs.SetInitialBestScore(seed_occupancy);
          }
        },
        /*bfs_after_run=*/[](BfsDpSearch &, SearchResult &) {},
        /*dfs_after_run=*/[](auto &, SearchResult &) {});
  };

  return opts;
}

SearchResult RecursiveDecomposeAndSchedule(
    ScheduleGraph &graph, const GCNSubtarget &st, const MachineFunction &mf,
    const LiveIntervals &lis, int seed_occupancy,
    const FormationConfig &subgraph_formation, OccupancyPolicy outer_policy,
    Search outer_search, std::optional<int64_t> outer_timeout_us,
    std::optional<int64_t> outer_fallback_us, Search inner_search,
    std::optional<int64_t> inner_timeout_us,
    std::optional<int64_t> inner_fallback_us) {
  // Per-level options: the inner (leaf) search + the composable outer search +
  // the (max_parts-capped) mincut formation. Make builds all three; we reuse
  // its inner as the leaf search and override it below for the non-leaf case.
  DecomposeAndScheduleOptions opts = DecomposeAndScheduleOptions::Make(
      st, mf, lis, seed_occupancy, subgraph_formation, outer_policy,
      outer_search, outer_timeout_us, outer_fallback_us, inner_search,
      inner_timeout_us, inner_fallback_us);

  // Leaf: a (sub)graph with at most target_subgraph_size scheduling units is
  // exactly what mincut would refuse to split (k < 2), so schedule it
  // directly with the inner search rather than decomposing.
  const int leaf_size = subgraph_formation.min_cut.target_subgraph_size;
  if (graph.NumSchedulingUnits() <= leaf_size) {
    return opts.inner_search(graph);
  }

  // Non-leaf: recurse on each subgraph. A deeper level's "outer" search is
  // really inner work (it orders a subgraph's interior), so it uses the INNER
  // config: strategy inner_search, budgets inner_timeout_us/inner_fallback_us,
  // objective the hardcoded continuous occupancy. Only the outermost level uses
  // (outer_policy, outer_search, outer_timeout). ScheduleSubgraph runs this
  // inner search on each extracted subgraph before the level's outer search, so
  // the schedule is built bottom-up.
  opts.inner_search = [&st, &mf, &lis, seed_occupancy, subgraph_formation,
                       inner_search, inner_timeout_us,
                       inner_fallback_us](ScheduleGraph &sub) -> SearchResult {
    return RecursiveDecomposeAndSchedule(
        sub, st, mf, lis, seed_occupancy, subgraph_formation,
        /*outer_policy=*/OccupancyPolicy::kContinuousOccupancy,
        /*outer_search=*/inner_search,
        /*outer_timeout_us=*/inner_timeout_us,
        /*outer_fallback_us=*/inner_fallback_us, inner_search, inner_timeout_us,
        inner_fallback_us);
  };
  return DecomposeAndSchedule(graph, st, mf, opts);
}

}  // namespace hierarchical_scheduler
}  // namespace llvm
