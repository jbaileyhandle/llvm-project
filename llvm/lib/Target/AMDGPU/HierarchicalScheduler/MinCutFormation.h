//===- MinCutFormation.h - dagP min-cut subgraph formation ---------------===//
//
// Subgraph formation by acyclic min-cut of the region's data-dependency
// DAG, computed in-process by the linked dagP partitioner. The MinCut()
// SubgraphFormationPolicy uses this; see MinCutFormation.cpp.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MINCUTFORMATION_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MINCUTFORMATION_H

#include <memory>
#include <vector>

namespace llvm {
namespace hierarchical_scheduler {

class ScheduleGraph;
struct SubgraphInfo;

/// Tunables for min-cut formation. Carried by the MinCut()
/// SubgraphFormationPolicy and passed to BuildSubgraphInfosByMinCut.
struct MinCutSettings {
  /// Number of parts requested from dagP is k = ceil(N / target_subgraph_size),
  /// where N is the count of real instruction nodes in the region.
  int target_subgraph_size = 24;
  /// dagP imbalance ratio (--ratio): the cap on any single part's weight is
  /// imbalance_ratio * (total_weight / k), i.e. how far a part may exceed the
  /// even share. 1.0 forces even sizes; higher lets dagP trade balance for a
  /// smaller data-edge cut. 1.5 is a middle ground.
  float imbalance_ratio = 1.5f;
  /// dagP RNG seed. A fixed nonzero value gives reproducible partitions
  /// (dagP treats seed 0 as "seed from time()").
  int seed = 1;
};

/// Partition the region's data-dependency DAG by acyclic min-cut (dagP, called
/// in-process) using `settings`, and return the resulting SubgraphInfos. This
/// is the min-cut analogue of BuildSubgraphInfos: FormSubgraphs calls it for
/// the MinCut() policy and owns the shared tail — the DumpSubgraphDag hook,
/// InsertSubgraphProxies, and the re-derive.
///
/// Returns an empty vector for regions too small to yield two or more
/// subgraphs, or with no strong edges among real nodes.
std::vector<std::unique_ptr<SubgraphInfo>>
BuildSubgraphInfosByMinCut(ScheduleGraph &graph, const MinCutSettings &settings);

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MINCUTFORMATION_H
