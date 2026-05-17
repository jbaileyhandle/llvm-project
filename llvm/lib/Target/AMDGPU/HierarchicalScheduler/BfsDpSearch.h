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

#include "PartitionDag.h"

namespace llvm {

class GCNSubtarget;
class MachineFunction;

namespace hierarchical_scheduler {

class ScheduleGraph;

class BfsDpSearch {
 public:
  /// `graph`, `st`, and `mf` must outlive this object.
  BfsDpSearch(const ScheduleGraph *graph, const GCNSubtarget *st,
              const MachineFunction *mf);

  /// Build the partition dag and apply the recovered schedule.
  /// Integration with the rest of the scheduler (formation,
  /// telemetry) is added incrementally; this shell currently just
  /// delegates to PartitionDag::Build().
  void Run();

  /// Test-only pass-through to PartitionDag::EnableTestModeForTest.
  /// Must be called before Run(); enables synthetic-pressure mode
  /// for shakedowns by injecting per-topo-index VGPR deltas instead
  /// of reading per-node register info.
  void EnableTestModeForTest(const std::vector<int> &per_node_vgpr_deltas) {
    dag_.EnableTestModeForTest(per_node_vgpr_deltas);
  }

  /// Test-only: access the dag for reading the recovered schedule
  /// and the sink's PathBottleneck after Run().
  const PartitionDag &GetDagForTest() const { return dag_; }

 private:
  PartitionDag dag_;
};

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_BFSDPSEARCH_H
