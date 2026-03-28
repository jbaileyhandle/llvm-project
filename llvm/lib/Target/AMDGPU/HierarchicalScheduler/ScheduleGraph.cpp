//===- ScheduleGraph.cpp - Hierarchical scheduling graph ------------------===//
//
// Implementation of ScheduleGraph and ScheduleNode.
//
//===----------------------------------------------------------------------===//

#include "ScheduleGraph.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <queue>

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

// Returns a unique ID and increments the counter. Used by both ScheduleNode
// and ScheduleGraph constructors.
int64_t GetAndIncrementScheduleId() {
  static int64_t next_id = 0;
  return next_id++;
}

} // anonymous namespace

// --- ScheduleNode ---

ScheduleNode::ScheduleNode(SUnit *su)
    : id_(GetAndIncrementScheduleId()), content_(su) {}

ScheduleNode::ScheduleNode(SUnit *su, std::string debug_name)
    : id_(GetAndIncrementScheduleId()), content_(su),
      debug_name_(std::move(debug_name)) {}

ScheduleNode::ScheduleNode(std::unique_ptr<ScheduleGraph> subgraph)
    : id_(GetAndIncrementScheduleId()), content_(std::move(subgraph)) {}

std::string ScheduleNode::ToString() const {
  std::string result = "[" + std::to_string(id_);

  if (!IsLeaf()) {
    result += ":" + GetSubgraph()->ToString() + "]";
    return result;
  }

  if (!debug_name_.empty()) {
    result += ":" + debug_name_ + "]";
    return result;
  }

  SUnit *su = std::get<SUnit *>(content_);
  if (su && su->getInstr()) {
    std::string mi_str;
    raw_string_ostream os(mi_str);
    // IsStandalone=true: print without context (self-contained).
    // SkipOpers=false: include all operands.
    // SkipDebugLoc=true: omit source location noise.
    // AddNewLine=false: no trailing newline.
    su->getInstr()->print(os, /*IsStandalone=*/true, /*SkipOpers=*/false,
                          /*SkipDebugLoc=*/true, /*AddNewLine=*/false);
    // Trim leading whitespace from MachineInstr::print output.
    size_t start = mi_str.find_first_not_of(" \t\n");
    if (start != std::string::npos) {
      mi_str = mi_str.substr(start);
    }
    result += "] " + mi_str;
    return result;
  }

  result += "]";
  return result;
}

int ScheduleNode::LeafSize() const {
  if (IsLeaf()) {
    return 1;
  }
  return GetSubgraph()->LeafSize();
}

// --- ScheduleGraph ---

ScheduleGraph::ScheduleGraph() : id_(GetAndIncrementScheduleId()) {}

std::string ScheduleGraph::ToString() const {
  return "graph[" + std::to_string(id_) + "](" +
         std::to_string(Size()) + " nodes)";
}

int ScheduleGraph::LeafSize() const {
  int count = 0;
  for (const ScheduleNode &node : nodes_) {
    count += node.LeafSize();
  }
  return count;
}

// Kahn's algorithm: iteratively remove nodes with no unmet predecessors.
// Produces a forward topological order (sources first, sinks last).
// If not all nodes are placed, the graph contains a cycle.
void ScheduleGraph::ComputeTopologicalOrder(bool include_weak_edges) {
  topo_order_.clear();
  topo_order_.reserve(nodes_.size());

  // Count the number of relevant predecessors for each node. We use a
  // separate counter rather than modifying the graph.
  DenseMap<ScheduleNode *, int> in_degree;
  for (ScheduleNode &node : nodes_) {
    int count = 0;
    for (const ScheduleEdge &pred : node.Preds()) {
      if (include_weak_edges || pred.IsStrongEdge()) {
        ++count;
      }
    }
    in_degree[&node] = count;
  }

  // Seed the queue with all nodes that have no relevant predecessors.
  std::queue<ScheduleNode *> ready;
  for (ScheduleNode &node : nodes_) {
    if (in_degree[&node] == 0) {
      ready.push(&node);
    }
  }

  // Process: remove a ready node, decrement its successors' in-degrees,
  // and add any that reach zero to the queue.
  while (!ready.empty()) {
    ScheduleNode *node = ready.front();
    ready.pop();

    node->SetTopoIndex(static_cast<int>(topo_order_.size()));
    topo_order_.push_back(node);

    for (const ScheduleEdge &succ : node->Succs()) {
      if (!include_weak_edges && succ.IsWeakEdge()) {
        continue;
      }
      int &deg = in_degree[succ.node_];
      --deg;
      if (deg == 0) {
        ready.push(succ.node_);
      }
    }
  }

  if (static_cast<int>(topo_order_.size()) != Size()) {
    std::string msg = "Cycle detected in " + ToString();
    report_fatal_error(llvm::StringRef(msg));
  }
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
  graph.nodes_.emplace_back(&entry_su, "EntrySU");
  sunit_to_node[&entry_su] = &graph.nodes_.back();

  graph.nodes_.emplace_back(&exit_su, "ExitSU");
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

ScheduleGraph ScheduleGraph::BuildTestDAG() {
  ScheduleGraph graph;

  // 7 nodes: A(0), C(1), D(2), E(3), F(4), G(5), H(6)
  // Nodes are leaves wrapping nullptr since we have no real SUnits.
  graph.nodes_.reserve(7);
  graph.nodes_.emplace_back(nullptr, "A");
  graph.nodes_.emplace_back(nullptr, "C");
  graph.nodes_.emplace_back(nullptr, "D");
  graph.nodes_.emplace_back(nullptr, "E");
  graph.nodes_.emplace_back(nullptr, "F");
  graph.nodes_.emplace_back(nullptr, "G");
  graph.nodes_.emplace_back(nullptr, "H");

  ScheduleNode &a = graph.nodes_[0];
  ScheduleNode &c = graph.nodes_[1];
  ScheduleNode &d = graph.nodes_[2];
  ScheduleNode &e = graph.nodes_[3];
  ScheduleNode &f = graph.nodes_[4];
  ScheduleNode &g = graph.nodes_[5];
  ScheduleNode &h = graph.nodes_[6];

  // Edges (all kData with zero latency):
  a.AddSucc(ScheduleEdge(&h, ScheduleEdge::kData));
  a.AddSucc(ScheduleEdge(&c, ScheduleEdge::kData));
  a.AddSucc(ScheduleEdge(&d, ScheduleEdge::kData));
  c.AddSucc(ScheduleEdge(&d, ScheduleEdge::kData));
  c.AddSucc(ScheduleEdge(&e, ScheduleEdge::kData));
  d.AddSucc(ScheduleEdge(&f, ScheduleEdge::kData));
  e.AddSucc(ScheduleEdge(&f, ScheduleEdge::kData));
  h.AddSucc(ScheduleEdge(&g, ScheduleEdge::kData));
  f.AddSucc(ScheduleEdge(&g, ScheduleEdge::kData));

  return graph;
}

ScheduleGraph ScheduleGraph::BuildTestDAGWithCycle() {
  ScheduleGraph graph;

  // 3 nodes with a cycle: A → B → C → B
  graph.nodes_.reserve(3);
  graph.nodes_.emplace_back(nullptr, "A");
  graph.nodes_.emplace_back(nullptr, "B");
  graph.nodes_.emplace_back(nullptr, "C");

  ScheduleNode &a = graph.nodes_[0];
  ScheduleNode &b = graph.nodes_[1];
  ScheduleNode &c = graph.nodes_[2];

  a.AddSucc(ScheduleEdge(&b, ScheduleEdge::kData));
  b.AddSucc(ScheduleEdge(&c, ScheduleEdge::kData));
  c.AddSucc(ScheduleEdge(&b, ScheduleEdge::kData));  // cycle: C → B

  return graph;
}
