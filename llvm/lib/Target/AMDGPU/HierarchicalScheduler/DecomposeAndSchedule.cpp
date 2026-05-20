//===- DecomposeAndSchedule.cpp - Subgraph-scheduling driver --------------===//
//
// Implementation. See DecomposeAndSchedule.h for the contract.
//
//===----------------------------------------------------------------------===//

#include "DecomposeAndSchedule.h"

#include "ScheduleGraph.h"
#include "ScheduleSubgraph.h"
#include "SubgraphFormation.h"
#include "SubgraphInfo.h"
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
  for (SubgraphInfo *info : graph.GetSubgraphInfos()) {
    ScheduleSubgraph(*info, graph, st, mf, opts.inner_search);
  }

  // Step 3: lock the chosen interiors into `graph` with order-edge
  // chains.
  graph.AddSubgraphOrderEdges();

  // Step 4: run the outer search over the proxied + chained graph.
  return opts.outer_search(graph);
}

}  // namespace hierarchical_scheduler
}  // namespace llvm
