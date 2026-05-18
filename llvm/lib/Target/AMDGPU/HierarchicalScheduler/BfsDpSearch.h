//===- BfsDpSearch.h - BFS-DP scheduler integration shell -------*- C++ -*-===//
//
// Thin integration shell over PartitionDag. Owns the dag, drives Build,
// and is the point where the rest of the HierarchicalScheduler
// infrastructure plugs in (formation, telemetry, applying the recovered
// schedule to the region). The actual search lives in PartitionDag.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_BFSDPSEARCH_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_BFSDPSEARCH_H

#include "BfsDpSettings.h"
#include "PartitionDag.h"
#include "SearchResult.h"

namespace llvm {

class GCNSubtarget;
class MachineFunction;

namespace hierarchical_scheduler {

class ScheduleGraph;

class BfsDpSearch {
 public:
  /// `graph`, `st`, and `mf` must outlive this object. `settings`
  /// is passed straight through to PartitionDag; see BfsDpSettings
  /// and PartitionDag's ctor for the metric / timeout semantics and
  /// the defaults.
  BfsDpSearch(const ScheduleGraph *graph, const GCNSubtarget *st,
              const MachineFunction *mf, BfsDpSettings settings = {});

  /// Build the partition dag and return the result. SearchResult::
  /// schedule carries the recovered schedule when the dag reached
  /// the sink; it is empty when the score-bound prune (see
  /// SetInitialBestScore) eliminated every path, and also when the
  /// settings.timeout_ms budget fired before the BFS drained.
  /// termination_cause is kTimedOut in that timeout case and
  /// kFullyExplored otherwise — BFS-DP has no ShouldEndSearch /
  /// policy hook, so kPolicySatisfied never occurs.
  SearchResult Run();

  /// Test-only pass-through to PartitionDag::EnableTestModeForTest.
  /// Must be called before Run(); enables synthetic-pressure mode
  /// for shakedowns by injecting per-topo-index VGPR deltas instead
  /// of reading per-node register info.
  void EnableTestModeForTest(const std::vector<int> &per_node_vgpr_deltas) {
    dag_.EnableTestModeForTest(per_node_vgpr_deltas);
  }

  /// Pass-through to PartitionDag::SetInitialBestScore. See that
  /// method for the pruning semantics.
  void SetInitialBestScore(int score) {
    dag_.SetInitialBestScore(score);
  }

  /// Pass-through to PartitionDag::SetInitialBestScore's
  /// ScheduleConstructor overload.
  void SetInitialBestScore(const ScheduleConstructor &init) {
    dag_.SetInitialBestScore(init);
  }

  /// Number of completed BFS layers when Run() returned. The
  /// partition dag expands one layer per graph node, so for an
  /// N-node graph the all-scheduled sink sits at depth N. After a
  /// kFullyExplored result this equals N; after a kTimedOut result
  /// it is the layer the search reached before the budget fired, so
  /// dividing by N gives the (approximate) fraction of the search's
  /// depth that was explored.
  int GetLevelsExplored() const { return dag_.GetCurrentLevel(); }

  /// Test-only: access the dag for reading the sink's PathBottleneck
  /// and dag stats after Run().
  const PartitionDag &GetDagForTest() const { return dag_; }

 private:
  PartitionDag dag_;
};

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_BFSDPSEARCH_H
