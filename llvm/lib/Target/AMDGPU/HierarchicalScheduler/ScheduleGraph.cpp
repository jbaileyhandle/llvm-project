//===- ScheduleGraph.cpp - Hierarchical scheduling graph ------------------===//
//
// Implementation of ScheduleGraph and ScheduleNode.
//
//===----------------------------------------------------------------------===//

#include "ScheduleGraph.h"
#include "llvm/ADT/DenseMap.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

/// Map an SDep edge to our ScheduleEdge::Kind. Preserves the full
/// distinction between SDep::Kind and SDep::OrderKind so no information
/// is lost.
///
/// SDep::Kind has four values: Data, Anti, Output, Order.
/// When Kind == Order, SDep::OrderKind further distinguishes:
///   Barrier, MayAliasMem, MustAliasMem, Artificial, Weak, Cluster.
/// We map each combination to a unique ScheduleEdge::Kind.
ScheduleEdge::Kind MapSDepToEdgeKind(const SDep &dep) {
  switch (dep.getKind()) {
  case SDep::Data:
    return ScheduleEdge::kData;
  case SDep::Anti:
    return ScheduleEdge::kAnti;
  case SDep::Output:
    return ScheduleEdge::kOutput;
  case SDep::Order:
    // Order edges have a sub-kind (OrderKind). SDep doesn't expose
    // OrderKind directly, so we use the query methods. We check Cluster
    // before Weak because isWeak() returns true for both Weak and Cluster
    // (it checks OrdKind >= Weak, and Cluster comes after Weak in the
    // enum). Every OrderKind has an explicit check; if none match, we hit
    // llvm_unreachable.
    if (dep.isCluster()) {
      return ScheduleEdge::kCluster;
    }
    if (dep.isWeak()) {
      return ScheduleEdge::kWeak;
    }
    if (dep.isBarrier()) {
      return ScheduleEdge::kBarrier;
    }
    if (dep.isMustAlias()) {
      return ScheduleEdge::kMustAliasMem;
    }
    if (dep.isArtificial()) {
      return ScheduleEdge::kArtificial;
    }
    if (dep.isNormalMemory()) {
      return ScheduleEdge::kMayAliasMem;
    }
    llvm_unreachable("Unknown SDep::OrderKind");
  }
  llvm_unreachable("Unknown SDep::Kind");
}

} // anonymous namespace

// --- ScheduleNode ---

int ScheduleNode::LeafSize() const {
  if (IsLeaf()) {
    return 1;
  }
  return GetSubgraph()->LeafSize();
}

// --- ScheduleGraph ---

int ScheduleGraph::LeafSize() const {
  int count = 0;
  for (const ScheduleNode &node : nodes_) {
    count += node.LeafSize();
  }
  return count;
}

ScheduleGraph
ScheduleGraph::BuildFromSUnits(MutableArrayRef<SUnit> sunits,
                               SUnit &entry_su, SUnit &exit_su) {
  ScheduleGraph graph;

  // Reserve space for all SUnits plus the two boundary nodes. This ensures
  // the vector does not reallocate when we add edges in phase 2, keeping
  // node pointers stable.
  graph.nodes_.reserve(sunits.size() + 2);

  // Phase 1: Create a leaf node for each SUnit, plus boundary nodes.
  // We build a map from SUnit pointer to ScheduleNode pointer so we can
  // resolve edge targets in phase 2.
  DenseMap<const SUnit *, ScheduleNode *> sunit_to_node;

  for (SUnit &su : sunits) {
    graph.nodes_.emplace_back(&su);
    sunit_to_node[&su] = &graph.nodes_.back();
  }

  // Add boundary nodes at the end.
  graph.nodes_.emplace_back(&entry_su);
  sunit_to_node[&entry_su] = &graph.nodes_.back();

  graph.nodes_.emplace_back(&exit_su);
  sunit_to_node[&exit_su] = &graph.nodes_.back();

  // Phase 2: Add edges. For each node, translate its SUnit's successor
  // edges into ScheduleEdges. AddSucc automatically creates the matching
  // predecessor edge on the target, so we only process Succs to avoid
  // duplicates.
  for (ScheduleNode &node : graph.nodes_) {
    SUnit *su = node.GetSUnit();
    for (const SDep &sdep : su->Succs) {
      const SUnit *succ_su = sdep.getSUnit();

      auto it = sunit_to_node.find(succ_su);
      assert(it != sunit_to_node.end() &&
             "SUnit successor not found in graph — broken DAG?");

      ScheduleEdge::Kind kind = MapSDepToEdgeKind(sdep);
      int latency = static_cast<int>(sdep.getLatency());

      node.AddSucc(ScheduleEdge(it->second, kind, latency));
    }
  }

  return graph;
}
