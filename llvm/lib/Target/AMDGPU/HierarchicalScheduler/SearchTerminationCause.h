//===- SearchTerminationCause.h - How a schedule search ended ---*- C++ -*-===//
//
// SearchTerminationCause: how a completed schedule search ended.
// Split out of SearchResult.h so headers that need only the cause
// enum -- not the SearchResult struct, which pulls in
// ScheduleConstructor and the whole graph -- can include just this.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHTERMINATIONCAUSE_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHTERMINATIONCAUSE_H

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

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHTERMINATIONCAUSE_H
