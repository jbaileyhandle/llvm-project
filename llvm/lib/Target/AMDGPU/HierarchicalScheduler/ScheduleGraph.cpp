//===- ScheduleGraph.cpp - Hierarchical scheduling graph ------------------===//
//
// Implementation of ScheduleGraph and ScheduleNode.
//
//===----------------------------------------------------------------------===//

#include "ScheduleGraph.h"
#include "DominatorTree.h"
#include "GCNRegPressure.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
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
    : id_(GetAndIncrementScheduleId()), content_(su) {
  ExtractRegInfo();
}

ScheduleNode::ScheduleNode(SUnit *su, std::string debug_name)
    : id_(GetAndIncrementScheduleId()), content_(su),
      debug_name_(std::move(debug_name)) {
  ExtractRegInfo();
}

ScheduleNode::ScheduleNode(std::unique_ptr<ScheduleGraph> subgraph)
    : id_(GetAndIncrementScheduleId()), content_(std::move(subgraph)) {}

void ScheduleNode::ExtractRegInfo() {
  if (!IsLeaf()) {
    return;
  }
  SUnit *su = GetSUnit();
  if (!su || !su->getInstr()) {
    return;
  }
  for (const MachineOperand &mo : su->getInstr()->operands()) {
    if (!mo.isReg() || !mo.getReg().isVirtual()) {
      continue;
    }
    // Full lane mask as default — the GCNRegisterTracker computes
    // accurate lane masks directly from the MachineInstr operands,
    // so these are only used by the old RegisterTracker.
    if (mo.isDef()) {
      reg_defs_.push_back({mo.getReg(), LaneBitmask::getAll()});
    } else if (mo.isUse() && !mo.isUndef()) {
      reg_uses_.push_back({mo.getReg(), LaneBitmask::getAll()});
    }
  }
}

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
ScheduleGraph::~ScheduleGraph() = default;

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

  // Sort each node's successor list by ascending topo index. Required by
  // the transitive reduction algorithm (process closest successors first).
  for (ScheduleNode &node : nodes_) {
    node.SortSuccsByTopoIndex();
  }

  topo_sorted_ = true;
}

void ScheduleGraph::ComputeTransitiveReduction() {
  if (!topo_sorted_) {
    std::string msg = "ComputeTransitiveReduction called on " + ToString() +
                      " before ComputeTopologicalOrder";
    report_fatal_error(llvm::StringRef(msg));
  }

  int num_nodes = Size();

  // One BitVector per node, each of size num_nodes. reachable[i] tracks
  // which nodes are reachable from the node at topo index i (including
  // itself).
  std::vector<BitVector> reachable(num_nodes, BitVector(num_nodes, false));
  for (int topo_idx = 0; topo_idx < num_nodes; ++topo_idx) {
    reachable[topo_idx].set(topo_idx);
  }

  auto reduced = std::make_unique<ReducedGraph>(num_nodes);

  // Process nodes in reverse topo order (sinks first, sources last).
  // For each node, iterate its successors in ascending topo order (closest
  // first, guaranteed by SortSuccsByTopoIndex in ComputeTopologicalOrder).
  //
  // If a successor is already in our reachable set, it means we can reach
  // it through a closer successor we already processed — the direct edge
  // is redundant. Otherwise, the edge is essential: we keep it and merge
  // the successor's reachable set into ours.
  for (int curr_topo_idx = num_nodes - 1; curr_topo_idx >= 0;
       --curr_topo_idx) {
    ScheduleNode *curr_node = topo_order_[curr_topo_idx];

    for (const ScheduleEdge &succ_edge : curr_node->Succs()) {
      int succ_topo_idx = succ_edge.node_->GetTopoIndex();

      if (reachable[curr_topo_idx].test(succ_topo_idx)) {
        // Already reachable via a closer path — edge is redundant.
        continue;
      }

      // Essential edge — keep it in the reduced graph.
      reduced->succs[curr_topo_idx].push_back(succ_topo_idx);
      reduced->preds[succ_topo_idx].push_back(curr_topo_idx);

      // Merge the successor's reachable set into ours.
      reachable[curr_topo_idx] |= reachable[succ_topo_idx];
    }
  }

  reduced_graph_ = std::move(reduced);
}

void ScheduleGraph::ComputeDominatorTree() {
  if (!IsReduced()) {
    std::string msg = "ComputeDominatorTree called on " + ToString() +
                      " before ComputeTransitiveReduction";
    report_fatal_error(llvm::StringRef(msg));
  }

  dom_tree_ = std::make_unique<DominatorTree>(
      DominatorTree::Build(*reduced_graph_));
}

std::string ScheduleGraph::DominatorTreeToString() const {
  if (!HasDominatorTree()) {
    return "DominatorTree not computed\n";
  }
  return dom_tree_->ToString(*this);
}

ScheduleGraph
ScheduleGraph::BuildFromSUnits(MutableArrayRef<SUnit> sunits,
                               const LiveIntervals &lis,
                               const MachineRegisterInfo &mri,
                               SlotIndex region_begin_idx,
                               SlotIndex region_end_idx) {
  ScheduleGraph graph;

  // Reserve space for all SUnits plus our own entry and exit nodes.
  graph.nodes_.reserve(sunits.size() + 2);

  // Phase 1: Create a leaf node for each SUnit, skipping LLVM's boundary
  // nodes. Register defs/uses are extracted automatically by the
  // ScheduleNode constructor.
  DenseMap<const SUnit *, ScheduleNode *> sunit_to_node;

  for (SUnit &su : sunits) {
    if (su.isBoundaryNode()) {
      continue;
    }
    graph.nodes_.emplace_back(&su);
    sunit_to_node[&su] = &graph.nodes_.back();
  }

  // Phase 2: Add edges between real instruction nodes. Skip edges to/from
  // LLVM's boundary nodes.
  for (ScheduleNode &node : graph.nodes_) {
    SUnit *su = node.GetSUnit();
    if (!su) {
      continue;
    }
    for (const SDep &sdep : su->Succs) {
      const SUnit *succ_su = sdep.getSUnit();
      if (succ_su->isBoundaryNode()) {
        continue;
      }

      auto it = sunit_to_node.find(succ_su);
      if (it == sunit_to_node.end()) {
        continue;
      }

      ScheduleEdge::Kind kind = MapSDepToEdgeKind(sdep);
      int latency = static_cast<int>(sdep.getLatency());
      node.AddSucc(ScheduleEdge(it->second, kind, latency));
    }
  }

  // Phase 3: Create entry/exit nodes with edges and live register info.
  graph.CreateEntryAndExitNodes(lis, mri, region_begin_idx, region_end_idx);

  return graph;
}

void ScheduleGraph::CreateEntryAndExitNodes(const LiveIntervals &lis,
                                            const MachineRegisterInfo &mri,
                                            SlotIndex region_begin_idx,
                                            SlotIndex region_end_idx) {
  nodes_.emplace_back(static_cast<SUnit *>(nullptr), "Entry");
  ScheduleNode &entry_node = nodes_[nodes_.size() - 1];

  nodes_.emplace_back(static_cast<SUnit *>(nullptr), "Exit");
  ScheduleNode &exit_node = nodes_[nodes_.size() - 1];

  // Wire entry to all root nodes, exit from all leaf nodes.
  for (ScheduleNode &node : nodes_) {
    if (&node == &entry_node || &node == &exit_node) {
      continue;
    }
    if (node.NumPreds() == 0) {
      entry_node.AddSucc(ScheduleEdge(&node, ScheduleEdge::kArtificial));
    }
    if (node.NumSuccs() == 0) {
      // TODO: LLVM's buildSchedGraph adds an artificial edge from
      // high-latency leaf instructions to ExitSU with latency =
      // SU->Latency - 1 (ScheduleDAGInstrs.cpp, line 877). This
      // ensures cross-region heuristics account for results that
      // won't be ready for many cycles (e.g., a VMEM load with
      // latency 80 at the end of a region). We currently use
      // latency 0, which is fine for within-region schedule length
      // but may underestimate costs for cross-region analysis.
      node.AddSucc(ScheduleEdge(&exit_node, ScheduleEdge::kArtificial));
    }
  }

  // Populate register defs/uses from LiveIntervals with per-lane
  // accuracy. getLiveLaneMask queries sub-range liveness to determine
  // exactly which lanes are live at the given slot index, rather than
  // treating the whole register as live whenever any lane is.
  for (int i = 0, num_virt_regs = mri.getNumVirtRegs(); i < num_virt_regs;
       ++i) {
    Register reg = Register::index2VirtReg(i);
    if (!lis.hasInterval(reg)) {
      continue;
    }

    LaneBitmask live_in_mask =
        getLiveLaneMask(reg, region_begin_idx, lis, mri);
    if (live_in_mask.any()) {
      entry_node.AddRegDef(reg, live_in_mask);
    }

    LaneBitmask live_out_mask =
        getLiveLaneMask(reg, region_end_idx, lis, mri);
    if (live_out_mask.any()) {
      exit_node.AddRegUse(reg, live_out_mask);
    }
  }
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
