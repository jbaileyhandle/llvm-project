//===- BfsDpSearch.cpp - BFS-DP scheduler integration shell --------------===//
//
// Skeleton implementation. See BfsDpSearch.h for the contract.
//
//===----------------------------------------------------------------------===//

#include "BfsDpSearch.h"

namespace llvm {
namespace hierarchical_scheduler {

BfsDpSearch::BfsDpSearch(const ScheduleGraph *graph, const GCNSubtarget *st,
                         const MachineFunction *mf,
                         ScheduleMetric metric)
    : dag_(graph, st, mf, metric) {}

SearchResult BfsDpSearch::Run() {
  bool reached_sink = dag_.Build();
  std::optional<ScheduleConstructor> schedule;
  if (reached_sink) {
    // Copy the dag's reconstructed constructor into the result.
    // One deep copy per region that found a schedule — not a hot
    // path. The dag (and its reconstructed constructor) outlive
    // this call inside `dag_`, so a copy rather than a move keeps
    // the dag queryable afterward (GetDagForTest).
    schedule = dag_.GetScheduleConstructor();
  }
  // BFS-DP has no timeout or ShouldEndSearch hook yet — Build
  // always runs the (score-bound-pruned) partition dag to
  // exhaustion.
  return SearchResult{std::move(schedule),
                      SearchTerminationCause::kFullyExplored};
}

}  // namespace hierarchical_scheduler
}  // namespace llvm
