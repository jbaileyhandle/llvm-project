//===- SearchResult.h - Common schedule-search result type -----*- C++ -*-===//
//
// The result type returned by a schedule search, shared by
// every search strategy (DfsSearch, BfsDpSearch, and any future
// ones). Keeping it strategy-agnostic means callers handle a search
// result uniformly regardless of which strategy produced it.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHRESULT_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHRESULT_H

#include "ScheduleConstructor.h"
#include "SearchTerminationCause.h"
#include <optional>

namespace llvm {
namespace hierarchical_scheduler {

/// The outcome of running a region schedule search.
struct SearchResult {
  /// The schedule the search produced, if any:
  ///   - DFS: always present. `best` is seeded with the input
  ///     order, so worst case `schedule` equals the input —
  ///     applying that is a harmless no-op.
  ///   - BFS-DP: present when the search reached a complete
  ///     schedule; empty when it produced none — e.g. the
  ///     score-bound prune eliminated every path to the sink, or
  ///     (once a BFS-DP timeout exists) the budget ran out before
  ///     completion.
  /// On empty, the caller keeps the region's input order.
  ///
  /// has_value() means "the search produced a schedule to apply,"
  /// NOT "the schedule improves on the input" — see the DFS note
  /// above. Callers that need "did the order actually change"
  /// compare against the input order themselves.
  std::optional<ScheduleConstructor> schedule;

  /// How the search ended. Always meaningful, even when `schedule`
  /// is empty.
  SearchTerminationCause termination_cause;

  /// Which backend produced the applied schedule: "bfs", "dfs",
  /// "input" (none beat the input), or "" if unset. For a bfsdp+dfs
  /// wrapper this is "bfs" when BFS-DP delivered, "dfs" when the
  /// fallback ran.
  std::string winner;

  /// BFS-DP depth reached as a percent of nodes when it bailed before
  /// delivering (so DFS took over); unset when BFS delivered or wasn't run.
  std::optional<float> bfs_pct;

  /// Wall-clock milliseconds each backend ran, where it ran. A bfsdp+dfs
  /// wrapper sets both on a fallback (bfs_ms = time BFS-DP ran before
  /// bailing, dfs_ms = the rescue DFS); a pure run sets only its own.
  /// Each search stamps its own field in Run(); the wrapper carries
  /// bfs_ms across the fallback the same way it carries bfs_pct.
  std::optional<int> bfs_ms;
  std::optional<int> dfs_ms;

  /// Schedule-step count each backend performed (BFS-DP VisitSuccessor
  /// probes / DFS Schedule calls), paired with the *_ms above so a
  /// step rate (steps/ms) is derivable. Same set-in-Run() and
  /// carry-across-fallback discipline as the *_ms fields.
  std::optional<int> bfs_steps;
  std::optional<int> dfs_steps;
};

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHRESULT_H