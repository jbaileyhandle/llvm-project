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
                         BfsDpSettings settings)
    : dag_(graph, st, mf, settings) {}

SearchResult BfsDpSearch::Run() {
  bool built = dag_.Build();
  std::optional<ScheduleConstructor> schedule;
  if (built) {
    // Copy the dag's reconstructed constructor into the result.
    // One deep copy per region that found a schedule — not a hot
    // path. The dag (and its reconstructed constructor) outlive
    // this call inside `dag_`, so a copy rather than a move keeps
    // the dag queryable afterward (GetDagForTest).
    schedule = dag_.GetScheduleConstructor();
  }
  // Build runs the (score-bound-pruned) partition dag to
  // exhaustion unless the settings.timeout_ms budget fired.
  // kTimedOut when it did — BFS-DP has no complete schedule to
  // salvage mid-search, so `schedule` is empty there too.
  // Otherwise kFullyExplored, whether or not a schedule was found
  // (an empty result there means the prune beat every path).
  SearchTerminationCause cause =
      dag_.TimedOut() ? SearchTerminationCause::kTimedOut
                      : SearchTerminationCause::kFullyExplored;
  return SearchResult{std::move(schedule), cause};
}

}  // namespace hierarchical_scheduler
}  // namespace llvm
