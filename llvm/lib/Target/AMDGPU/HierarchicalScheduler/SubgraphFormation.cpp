//===- SubgraphFormation.cpp - Subgraph formation for ScheduleGraph -------===//
//
// Implementation of SubgraphFormation.h. See
// AMDGPUSubgraphFormationDesign.md for the design.
//
//===----------------------------------------------------------------------===//

#include "SubgraphFormation.h"

namespace llvm {
namespace hierarchical_scheduler {

bool IsSubgraphSplitter(const ScheduleNode *node, int latency_threshold) {
  for (const ScheduleEdge &edge : node->Successors()) {
    // Only latency-contributing edges count: weak hints (Cluster /
    // Weak) and the strong-but-zero-latency proxy wiring
    // (kSubgraphOrderEdge) are skipped — neither produces the
    // multi-cycle bubble that defines a splitter.
    if (!edge.IsLatencyEdge()) {
      continue;
    }
    if (edge.latency_ > latency_threshold) {
      return true;
    }
  }
  return false;
}

} // namespace hierarchical_scheduler
} // namespace llvm
