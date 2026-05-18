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
#include <optional>

namespace llvm {
namespace hierarchical_scheduler {

/// How a completed schedule search ended. Mutually exclusive.
enum class SearchTerminationCause {
  /// The search visited every reachable schedule (DFS: recursion
  /// drained; BFS-DP: the partition dag was built to the sink). The
  /// result is provably optimal under the search's metric.
  kFullyExplored,
  /// The per-region wall-clock budget was exhausted before the
  /// search completed. The result is the best found so far; no
  /// optimality claim.
  kTimedOut,
  /// The search's policy declared the current best good enough and
  /// stopped early (DfsSearch's Policy::ShouldEndSearch hook — for
  /// the occupancy policy, the best hit the function occupancy
  /// target). Searches with no such hook (BFS-DP) never produce
  /// this value.
  kPolicySatisfied,
};

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
};

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHRESULT_H