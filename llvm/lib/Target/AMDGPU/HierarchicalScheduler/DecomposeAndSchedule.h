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
#include "llvm/ADT/STLFunctionalExtras.h"

namespace llvm {

class GCNSubtarget;
class MachineFunction;

namespace hierarchical_scheduler {

class ScheduleGraph;

/// How a subgraph's members relate to the surrounding schedule.
///
/// kSerialized (the only supported value today): scope push/pop in
/// ScheduleConstructor forces the subgraph to be scheduled
/// contiguously — once entered, nothing else is scheduled until the
/// subgraph is drained. Combined with the order-edge chain installed
/// by AddSubgraphOrderEdges, the result is "members appear in
/// schedule_result.order, contiguously."
///
/// kInterleaved (future, §8 of AMDGPUSubgraphSchedulingDesign.md):
/// pre-scheduled subgraphs may interleave their members with other
/// work; only the internal order is fixed. Requires dropping scope
/// push/pop and relaxing proxy edges; not yet wired.
enum class SubgraphScheduleMode {
  kSerialized,
};

/// Options for one DecomposeAndSchedule invocation. Grouped as
/// formation config (`formation`, `mode`) followed by the two search
/// callables in pipeline order (`inner_search` runs in step 2,
/// `outer_search` in step 4). The two searches share the same
/// `function_ref<SearchResult(ScheduleGraph &)>` signature on
/// purpose — they run at different stages of the pipeline.
struct DecomposeAndScheduleOptions {
  /// Formation policy: which passes run, splitter/size thresholds.
  /// Consumed by FormSubgraphs in step 1.
  SubgraphFormationPolicy formation;

  /// Subgraph scheduling mode. Must be kSerialized today; the driver
  /// report_fatal_errors on any other value.
  SubgraphScheduleMode mode;

  /// The step-2 inner search — handed straight to ScheduleSubgraph
  /// for each formed SubgraphInfo. For a flat (non-recursive) use
  /// this is a leaf search, typically BFS-DP on the extracted
  /// subgraph. For a recursive use it is a lambda that calls
  /// DecomposeAndSchedule on the extracted subgraph; the lambda's
  /// captures hold whatever inner options that call needs.
  function_ref<SearchResult(ScheduleGraph &)> inner_search;

  /// The step-4 outer search — runs over the proxied + chained graph
  /// after all subgraph interiors are locked. The caller wraps
  /// whichever search algorithm it wants. BFS-DP only runs on
  /// proxy-free graphs and is NOT appropriate here; this should be
  /// DFS or branch-and-bound.
  function_ref<SearchResult(ScheduleGraph &)> outer_search;
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

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_DECOMPOSEANDSCHEDULE_H
