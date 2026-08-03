//===- OccupancySearchDispatch.h - Search enum -> occupancy search ---*- C++ -*-===//
//
// One place that maps a runtime `Search` (kDfs / kBfsDp / kBfsDpDfs) to the
// occupancy search it names, and runs it. Every occupancy search site --
// the flat (plain) path, the decompose OUTER (among-subgraphs) search, and the
// decompose INNER (within-subgraph "make") search -- goes through here, so the
// meaning of each `Search` value (and the "BFS-DP, fall back to DFS on timeout"
// skeleton) lives in exactly one spot instead of being re-expressed at each
// site. The DFS half delegates to RunOccupancyDfs (the policy->DfsSearch<T>
// bridge); this adds the BFS-DP half and the fallback control flow.
//
// The three sites differ only in parameters, supplied by the caller:
//   - seed_bfs(BfsDpSearch&): configures BFS-DP's prune floor (plain seeds the
//     region's original occupancy; outer seeds the input order or a target
//     occupancy; inner is unseeded -> pass a no-op).
//   - bfs_after_run(BfsDpSearch&, SearchResult&) / dfs_after_run(search, result):
//     site-specific logging that needs the live typed search object (the flat
//     path prints rates / post-schedule info; decompose passes no-ops).
// bfs_after_run is a template callback for the same reason RunOccupancyDfs's is:
// the BfsDpSearch it receives must be touched while alive.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_OCCUPANCYSEARCHDISPATCH_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_OCCUPANCYSEARCHDISPATCH_H

#include "BfsDpSearch.h"                 // BfsDpSearch, BfsDpSettings
#include "DfsOccupancyDispatch.h"        // RunOccupancyDfs
#include "HierarchicalConfigEnums.h"     // OccupancyPolicy, Search
#include "SearchResult.h"               // SearchResult
#include "SearchTerminationCause.h"     // SearchTerminationCause
#include "llvm/Support/ErrorHandling.h" // llvm_unreachable
#include <cstdint>
#include <optional>

namespace llvm {

class GCNSubtarget;
class MachineFunction;
class LiveIntervals;

namespace hierarchical_scheduler {

class ScheduleGraph;

/// Run the occupancy search named by `kind` over `graph`, returning its result.
///   kDfs        : DFS only (RunOccupancyDfs), budget `primary_timeout_ms`.
///   kBfsDp      : BFS-DP only, budget `primary_timeout_ms`.
///   kBfsDpDfs   : BFS-DP (budget `primary_timeout_ms`); if it produced no
///                 schedule because it timed out, DFS rescues (budget
///                 `fallback_timeout_ms`), carrying BFS-DP's stats onto the DFS
///                 result row.
/// `primary_timeout_ms` is the budget for whichever search runs first (DFS for
/// kDfs, BFS-DP otherwise); `fallback_timeout_ms` is the rescue DFS's budget,
/// used only by kBfsDpDfs. `policy` drives both halves
/// (BfsDpSettings::ForOccupancy for BFS-DP, RunOccupancyDfs for DFS). `seed_bfs`
/// configures BFS-DP's initial best score; `bfs_after_run` / `dfs_after_run` are
/// the per-site logging hooks.
template <typename SeedFn, typename BfsAfterRunFn, typename DfsAfterRunFn>
SearchResult RunOccupancySearch(Search kind, OccupancyPolicy policy,
                                ScheduleGraph &graph, const GCNSubtarget &st,
                                const MachineFunction &mf,
                                const LiveIntervals &lis,
                                std::optional<int64_t> primary_timeout_ms,
                                std::optional<int64_t> fallback_timeout_ms,
                                SeedFn seed_bfs, BfsAfterRunFn bfs_after_run,
                                DfsAfterRunFn dfs_after_run) {
  auto run_dfs = [&](std::optional<int64_t> timeout_ms) -> SearchResult {
    return RunOccupancyDfs(policy, graph, st, mf, lis, timeout_ms,
                           dfs_after_run);
  };

  switch (kind) {
  case Search::kDfs: {
    return run_dfs(primary_timeout_ms);
  }
  case Search::kBfsDp:
  case Search::kBfsDpDfs: {
    BfsDpSearch bfs(&graph, &st, &mf,
                    BfsDpSettings::ForOccupancy(policy, primary_timeout_ms));
    seed_bfs(bfs);
    SearchResult bfs_result = bfs.Run();
    // Fraction of the graph's layers BFS-DP reached; kept on the row whether or
    // not BFS-DP won, so a DFS-rescue row still shows how far BFS-DP got.
    if (graph.Size() > 0) {
      bfs_result.bfs_pct = (100.0f * bfs.GetLevelsExplored()) / graph.Size();
    }
    bfs_after_run(bfs, bfs_result);
    if (bfs_result.schedule.has_value()) {
      bfs_result.winner = "bfs";
      return bfs_result;
    }
    if (kind == Search::kBfsDp) {
      // BFS-DP only: no DFS rescue. Empty schedule keeps the input order.
      bfs_result.winner = "input";
      return bfs_result;
    }
    // kBfsDpDfs: only a timeout warrants a DFS rescue. A fully-explored empty
    // result proves (the score-bound prune is sound) that nothing beats the
    // seed, so DFS over the same objective can't either -- keep the input order.
    if (bfs_result.termination_cause != SearchTerminationCause::kTimedOut) {
      bfs_result.winner = "input";
      return bfs_result;
    }
    SearchResult dfs_result = run_dfs(fallback_timeout_ms);
    // Keep BFS-DP's throughput on the row even though DFS won.
    dfs_result.bfs_pct = bfs_result.bfs_pct;
    dfs_result.bfs_ms = bfs_result.bfs_ms;
    dfs_result.bfs_steps = bfs_result.bfs_steps;
    return dfs_result;
  }
  }
  llvm_unreachable("unhandled Search");
}

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_OCCUPANCYSEARCHDISPATCH_H
