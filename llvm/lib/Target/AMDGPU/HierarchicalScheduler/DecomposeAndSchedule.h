//===- DecomposeAndSchedule.h - Drive form-schedule-lock pipeline -*- C++ -*-===//
//
// DecomposeAndSchedule is the driver that ties the subgraph-scheduling
// pipeline together: it forms subgraphs, schedules each in isolation
// via the caller-supplied inner search, locks the chosen interiors
// into the graph as kSubgraphOrderEdge chains, and runs the outer
// search over the proxied + chained graph.
//
// One call, one pass over `graph`:
//   1. FormSubgraphs(graph, opts.formation)
//   2. for each formed SubgraphInfo:
//        ScheduleSubgraph(*info, graph, st, mf, opts.inner_search)
//   3. graph.AddSubgraphOrderEdges()
//   4. return opts.outer_search(graph)
//
// The flat (non-recursive) form. Recursion is the caller's
// responsibility: an `inner_search` lambda that itself calls
// DecomposeAndSchedule on the extracted subgraph with whatever inner
// options it wants. See §3.4 of AMDGPUSubgraphSchedulingDesign.md.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_DECOMPOSEANDSCHEDULE_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_DECOMPOSEANDSCHEDULE_H

#include "SearchResult.h"
#include "SubgraphFormation.h"
#include <functional>

namespace llvm {

class GCNSubtarget;
class LiveIntervals;
class MachineFunction;

namespace hierarchical_scheduler {

class ScheduleGraph;

/// Options for one DecomposeAndSchedule invocation. Grouped as
/// formation config (`formation`, `mode`) followed by the two search
/// callables in pipeline order (`inner_search` runs in step 2,
/// `outer_search` in step 4). The two searches share the same
/// `std::function<SearchResult(ScheduleGraph &)>` signature on
/// purpose — they run at different stages of the pipeline.
///
/// Searches are stored as owning `std::function` rather than
/// non-owning `function_ref` so factory methods (see
/// BfsDpWithDfsFallback below) can build closures that outlive the
/// factory call. Captures should still be by reference (st, mf, lis,
/// settings) — those are expected to outlive the options anyway, and
/// reference captures keep the closure small enough for std::function's
/// small-buffer optimization.
struct DecomposeAndScheduleOptions {
  /// Formation policy: which passes run, splitter/size thresholds.
  /// Consumed by FormSubgraphs in step 1.
  SubgraphFormationPolicy formation;

  /// Subgraph scheduling mode (kSerialized or kInterleaved, defined in
  /// SubgraphFormation.h). Passed to FormSubgraphs in step 1, which
  /// installs the subgraphs accordingly.
  SubgraphScheduleMode mode;

  /// The step-2 inner search — handed straight to ScheduleSubgraph
  /// for each formed SubgraphInfo. For a flat (non-recursive) use
  /// this is a leaf search, typically BFS-DP on the extracted
  /// subgraph. For a recursive use it is a lambda that calls
  /// DecomposeAndSchedule on the extracted subgraph; the lambda's
  /// captures hold whatever inner options that call needs.
  std::function<SearchResult(ScheduleGraph &)> inner_search;

  /// The step-4 outer search — runs over the proxied + chained graph
  /// after all subgraph interiors are locked. The chain leaves no
  /// choice inside subgraphs; the outer search is choosing among
  /// orderings of subgraph proxies and non-subgraph nodes. Both
  /// BFS-DP (PartitionDag handles proxies via GetInputOrderIndex)
  /// and DFS are valid choices here.
  std::function<SearchResult(ScheduleGraph &)> outer_search;

  /// Standard preset: BFS-DP first, DFS fallback if BFS-DP times out
  /// or returns no result. Both stages get a 5s wall-clock budget;
  /// they differ in metric, and each stage's DFS fallback uses the
  /// same metric as its BFS-DP:
  ///   - inner_search: BFS-DP and DFS fallback both run on the
  ///                   continuous occupancy score. No seed.
  ///   - outer_search: BFS-DP and DFS fallback both run on the
  ///                   integer occupancy score; BFS-DP is seeded
  ///                   with `seed_occupancy`.
  ///
  /// `seed_occupancy` is the function-wide register-only occupancy
  /// floor — typically `RegionInfo::GetOriginalRegisterOnlyOccupancy()`
  /// or the function's occupancy target. The outer BFS-DP's score-
  /// bound prune uses it so any partition path that can't strictly
  /// beat the floor is dropped. Inner BFS-DP doesn't apply this seed
  /// (the inner metric is continuous, which uses a different scale
  /// from the integer occupancy value); it explores exhaustively
  /// within the 5s budget.
  ///
  /// Both DFS fallbacks run after formation has already happened (inner
  /// on a leaf subgraph extracted by ScheduleSubgraph; outer on the
  /// proxied graph).
  ///
  /// `subgraph_formation` selects the formation strategy, install mode,
  /// and min-cut settings; the factory realizes it into
  /// `opts.formation`/`opts.mode` via
  /// SubgraphFormationPolicy::FromStrategy. Decompose needs a real
  /// strategy (kDomTree or kMinCut) — passing kNone forms no subgraphs
  /// and makes the pipeline a no-op.
  ///
  /// The returned options own their closures; `st`, `mf`, and `lis`
  /// are captured by reference and must outlive the options.
  /// `outer_continuous` switches the outer search's objective from the
  /// integer register-occupancy level (default) to the continuous
  /// register-occupancy score. The integer outer is seeded with
  /// `seed_occupancy`; the continuous outer is instead seeded with the
  /// input order's continuous score (the integer seed is a different
  /// scale). The inner search is always continuous.
  static DecomposeAndScheduleOptions BfsDpWithDfsFallback(
      const GCNSubtarget &st,
      const MachineFunction &mf,
      const LiveIntervals &lis,
      int seed_occupancy,
      const FormationConfig &subgraph_formation,
      bool outer_continuous = false);
};

/// Form subgraphs in `graph`, schedule each in isolation, lock the
/// chosen interiors with order-edge chains, then run the outer
/// search. Returns the outer search's `SearchResult`. See the file
/// header for the pipeline; see AMDGPUSubgraphSchedulingDesign.md
/// for the design.
SearchResult DecomposeAndSchedule(
    ScheduleGraph &graph,
    const GCNSubtarget &st,
    const MachineFunction &mf,
    const DecomposeAndScheduleOptions &opts);

/// Recursive multi-level decompose. Schedules `graph` by repeatedly
/// decomposing (mincut, capped at `subgraph_formation.min_cut.max_parts`
/// parts per level) and recursing, until a subgraph has at most
/// `min_cut.target_subgraph_size` scheduling units — a leaf, scheduled
/// directly by the continuous BfsDp+Dfs search. Each non-leaf level's inner
/// search recurses; its outer search (integer level, or continuous score
/// when `outer_continuous`) orders that level's subgraphs. Because
/// DecomposeAndSchedule schedules each subgraph's interior before the level's
/// outer runs, the schedule is built bottom-up. Leaves are always continuous;
/// the install mode and outer metric compose exactly as in the flat form.
SearchResult RecursiveDecomposeAndSchedule(
    ScheduleGraph &graph,
    const GCNSubtarget &st,
    const MachineFunction &mf,
    const LiveIntervals &lis,
    int seed_occupancy,
    const FormationConfig &subgraph_formation,
    bool outer_continuous);

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_DECOMPOSEANDSCHEDULE_H
