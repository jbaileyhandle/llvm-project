//===- SubgraphFormation.h - Subgraph formation for ScheduleGraph -*- C++ -*-=//
//
// Subgraph formation: given a freshly built ScheduleGraph (after
// BuildFromSUnits + ValidateAndComputeTopologicalOrder +
// ComputeTransitiveReduction + ComputeDominatorTree), produce a list
// of SubgraphInfos ready to be handed to
// ScheduleGraph::InsertSubgraphProxies. Uses the dominator tree plus
// a latency-based "splitter" notion to decide where to cut the graph
// into subgraphs.
//
// See AMDGPUSubgraphFormationDesign.md for the full design — this
// header tracks the §4–§7 surface (predicate, tree, passes,
// pipelines, driver).
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SUBGRAPHFORMATION_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SUBGRAPHFORMATION_H

#include "ScheduleGraph.h"

namespace llvm {
namespace hierarchical_scheduler {

/// True if `node` has at least one outgoing latency-contributing
/// edge whose latency exceeds `latency_threshold`. Such a node
/// forces a multi-cycle bubble after it, so it acts as a natural
/// cut point ("splitter") for subgraph formation.
///
/// Default threshold of 32 cycles (used by the FormSubgraphs
/// driver) catches memory loads on AMDGPU gfx906 (latencies ~80)
/// and excludes ALU (latencies 1–5). Entry/exit sentinels never
/// fire — entry's edges to roots have latency 0 set by
/// CreateEntryAndExitNodes; exit has no outgoing edges.
///
/// Free function rather than a ScheduleNode method: the predicate
/// is policy (the threshold, the choice of "outgoing latency edges"
/// vs e.g. a direct mayLoad() check) and shouldn't bake into the
/// node type.
bool IsSubgraphSplitter(const ScheduleNode *node, int latency_threshold);

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SUBGRAPHFORMATION_H
