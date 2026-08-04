//===- BfsDpSearch.cpp - BFS-DP scheduler integration shell --------------===//
//
// Skeleton implementation. See BfsDpSearch.h for the contract.
//
//===----------------------------------------------------------------------===//

#include "BfsDpSearch.h"

#include <chrono>

namespace llvm {
namespace hierarchical_scheduler {

BfsDpSearch::BfsDpSearch(const ScheduleGraph *graph, const GCNSubtarget *st,
                         const MachineFunction *mf,
                         BfsDpSettings settings)
    : dag_(graph, st, mf, settings) {}

SearchResult BfsDpSearch::Run() {
  auto start = std::chrono::steady_clock::now();
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
  // exhaustion unless the settings.timeout_us budget fired.
  // kTimedOut when it did — BFS-DP has no complete schedule to
  // salvage mid-search, so `schedule` is empty there too.
  // Otherwise kFullyExplored, whether or not a schedule was found
  // (an empty result there means the prune beat every path).
  SearchTerminationCause cause =
      dag_.TimedOut() ? SearchTerminationCause::kTimedOut
                      : SearchTerminationCause::kFullyExplored;
  SearchResult result{std::move(schedule), cause};
  // Throughput telemetry: wall-clock and the dag's Schedule-probe
  // count for this Build (Unschedule not counted; one Build per dag).
  result.bfs_ms = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start)
          .count());
  result.bfs_steps = dag_.GetScheduleCallCount();
  return result;
}

}  // namespace hierarchical_scheduler
}  // namespace llvm
