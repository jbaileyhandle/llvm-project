//===- ScheduleSubgraph.h - Schedule one subgraph in isolation --*- C++ -*-===//
//
// ScheduleSubgraph extracts a SubgraphInfo's members into a standalone
// ScheduleGraph, runs a caller-supplied search on it, and records the
// recovered schedule -- translated back to parent-graph nodes -- on
// the SubgraphInfo. It is the per-subgraph step of the decomposition:
// each subgraph is scheduled on its own before the full graph is
// scheduled with the in-subgraph orders locked.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULESUBGRAPH_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULESUBGRAPH_H

#include "SearchResult.h"
#include "SubgraphInfo.h"
#include "llvm/ADT/STLFunctionalExtras.h"

namespace llvm {

class GCNSubtarget;
class MachineFunction;

namespace hierarchical_scheduler {

class ScheduleGraph;

/// Schedule `info`'s subgraph in isolation, record the chosen
/// schedule on `info` (SubgraphInfo::schedule_result), and return it.
///
/// Extracts `info.members` into a standalone ScheduleGraph
/// (ScheduleGraph::BuildFromNodeSubset over `parent_graph`, the graph
/// the members belong to), runs `schedule` on that graph, and
/// translates the recovered order back to `parent_graph` member
/// nodes.
///
/// `schedule` is the search strategy: given the extracted graph it
/// returns a SearchResult. Its lambda captures whatever the strategy
/// needs (settings, ...), so ScheduleSubgraph stays agnostic to which
/// strategy runs.
///
/// If `schedule` returns a SearchResult with no schedule — BFS-DP
/// does when nothing beats its seed, or on timeout — the subgraph's
/// input order is recorded instead, so schedule_result is always
/// populated on return. A caller wanting other empty-result handling
/// does it inside `schedule`, or keys off the recorded
/// termination_cause.
const SubgraphScheduleResult &ScheduleSubgraph(
    SubgraphInfo &info, const ScheduleGraph &parent_graph,
    const GCNSubtarget &st, const MachineFunction &mf,
    function_ref<SearchResult(const ScheduleGraph &)> schedule);

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULESUBGRAPH_H
