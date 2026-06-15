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

#include "HierarchicalConfig.h" // OccupancyPolicy, Search
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
/// non-owning `function_ref` so the `Make` factory below can build
/// closures that outlive the factory call. Captures should still be by reference (st, mf, lis,
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

  /// Build the options for one DecomposeAndSchedule run.
  ///
  /// `inner_search` is fixed: continuous occupancy, BFS-DP first with a
  /// 5s budget, DFS fallback (continuous policy) if BFS-DP times out.
  /// The continuous metric uses a different scale from the integer
  /// floor, so inner BFS-DP runs unseeded and explores exhaustively
  /// within the budget.
  ///
  /// `outer_search` is selected by `outer_policy` and `outer_search`.
  /// See HierarchicalConfig.h for the available values and the
  /// validation rules; see the .cpp for the per-(policy, search)
  /// dispatch. When BFS-DP is the chosen outer algorithm, the BFS-DP
  /// score-bound prune is seeded with `seed_occupancy` (integer outer)
  /// or the input order's continuous score (continuous outer); when
  /// DFS is the chosen outer algorithm, no seed applies and DFS runs
  /// directly.
  ///
  /// `seed_occupancy` is the function-wide register-only occupancy
  /// floor — typically `RegionInfo::GetOriginalRegisterOnlyOccupancy()`
  /// or the function's occupancy target. Only the integer-outer BFS-DP
  /// uses it; the others ignore it.
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
  static DecomposeAndScheduleOptions Make(
      const GCNSubtarget &st,
      const MachineFunction &mf,
      const LiveIntervals &lis,
      int seed_occupancy,
      const FormationConfig &subgraph_formation,
      OccupancyPolicy outer_policy = OccupancyPolicy::kIntegerOccupancy,
      Search outer_search = Search::kBfsDpDfs);
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
/// directly by the continuous BFS-DP+DFS search. Each non-leaf level's
/// inner search recurses. `outer_policy` and `outer_search` apply to
/// the OUTERMOST level only. Deeper recursion always uses
/// (kContinuousOccupancy, kBfsDpDfs); the other modes are region-level
/// constructs that don't apply below the top. Because
/// DecomposeAndSchedule schedules each subgraph's interior before the
/// level's outer runs, the schedule is built bottom-up. Leaves are
/// always continuous.
SearchResult RecursiveDecomposeAndSchedule(
    ScheduleGraph &graph,
    const GCNSubtarget &st,
    const MachineFunction &mf,
    const LiveIntervals &lis,
    int seed_occupancy,
    const FormationConfig &subgraph_formation,
    OccupancyPolicy outer_policy,
    Search outer_search);

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_DECOMPOSEANDSCHEDULE_H
