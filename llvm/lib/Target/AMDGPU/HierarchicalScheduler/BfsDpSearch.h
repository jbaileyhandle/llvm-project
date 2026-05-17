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

namespace hierarchical_scheduler {

class ScheduleGraph;

class BfsDpSearch {
 public:
  /// `graph` and `st` must outlive this object.
  BfsDpSearch(const ScheduleGraph *graph, const GCNSubtarget *st);

  /// Build the partition dag and apply the recovered schedule.
  /// Integration with the rest of the scheduler (formation,
  /// telemetry) is added incrementally; this shell currently just
  /// delegates to PartitionDag::Build().
  void Run();

 private:
  PartitionDag dag_;
};

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_BFSDPSEARCH_H
