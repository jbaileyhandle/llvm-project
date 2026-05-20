//===- ScheduleSubgraph.cpp - Schedule one subgraph in isolation ---------===//
//
// Implementation. See ScheduleSubgraph.h for the contract.
//
//===----------------------------------------------------------------------===//

#include "ScheduleSubgraph.h"
#include "GCNRegisterTracker.h"
#include "ScheduleConstructor.h"
#include "ScheduleGraph.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include <memory>

namespace llvm {
namespace hierarchical_scheduler {

const SubgraphScheduleResult &ScheduleSubgraph(
    SubgraphInfo &info, const ScheduleGraph &parent_graph,
    const GCNSubtarget &st, const MachineFunction &mf,
    function_ref<SearchResult(ScheduleGraph &)> schedule) {
  // Extract the subgraph's members into a standalone graph.
  DenseMap<const ScheduleNode *, ScheduleNode *>
      subgraph_node_to_parent_member;
  std::unique_ptr<ScheduleGraph> subgraph_graph =
      ScheduleGraph::BuildFromNodeSubset(info.members, parent_graph, st, mf,
                                         subgraph_node_to_parent_member);

  // Run the caller's search on the extracted graph.
  SearchResult result = schedule(*subgraph_graph);

  // The schedule to record: the search's, or — when it produced none
  // — the subgraph's input order, which BuildFromNodeSubset seeded
  // into the extracted graph's input ScheduleConstructor.
  const ScheduleConstructor &recorded_schedule =
      result.schedule ? *result.schedule
                      : subgraph_graph->GetInputScheduleConstructor();

  // Translate the recorded order (over subgraph-graph nodes) back to
  // parent member nodes. The synthetic entry/exit are absent from the
  // map and drop out.
  SmallVector<ScheduleNode *, 32> order;
  for (const ScheduleNode *node : recorded_schedule.GetScheduleOrder()) {
    if (ScheduleNode *member =
            subgraph_node_to_parent_member.lookup(node)) {
      order.push_back(member);
    }
  }

  // Store it. The order is in parent-graph terms and the metadata is
  // by value, so it all outlives subgraph_graph — which is freed when
  // this function returns.
  info.schedule_result = SubgraphScheduleResult{
      std::move(order),
      recorded_schedule.GetPressureTracker().GetPeakPressure(),
      result.termination_cause};
  return *info.schedule_result;
}

}  // namespace hierarchical_scheduler
}  // namespace llvm
