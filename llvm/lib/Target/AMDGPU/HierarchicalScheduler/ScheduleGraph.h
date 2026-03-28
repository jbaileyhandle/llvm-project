//===- ScheduleGraph.h - Hierarchical scheduling graph ----------*- C++ -*-===//
//
// Defines ScheduleNode and ScheduleGraph, which together form a recursive
// graph structure for hierarchical scheduling.
//
// A ScheduleNode is either:
//   - A leaf: wraps a single SUnit (instruction)
//   - A group: owns a subgraph (ScheduleGraph) of ScheduleNodes
//
// A ScheduleGraph is a collection of ScheduleNodes with edges between them.
// The same types are used at every level of the hierarchy, enabling uniform
// traversal and algorithm application regardless of depth.
//
// Example hierarchy:
//
//   Level 2:  ScheduleGraph { GroupA, GroupB }
//                 |              |
//                 v              v
//   Level 1:  ScheduleGraph    ScheduleGraph
//             { X, Y, Z }     { W, V }
//               |  |  |         |  |
//               v  v  v         v  v
//   Level 0:  leaf leaf leaf  leaf leaf
//             (SUnit)         (SUnit)
//
// IMPORTANT: All nodes must be added to a graph before any edges are added.
// ScheduleEdge stores raw pointers into the nodes_ vector. If a node is
// added after edges exist, the vector may reallocate to a new buffer,
// invalidating all existing edge pointers. BuildFromSUnits enforces this
// by reserving the vector upfront and adding all nodes before any edges.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULEGRAPH_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULEGRAPH_H

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/ScheduleDAG.h"
#include <cassert>
#include <cstdint>
#include <memory>
#include <string>
#include <variant>
#include <vector>

namespace llvm {
namespace hierarchical_scheduler {

class ScheduleGraph;
class ScheduleNode;
struct ReducedGraph;

/// An edge between two ScheduleNodes. Stores the full edge kind (mirroring
/// LLVM's SDep::Kind and SDep::OrderKind) so no information is lost during
/// translation. Classification helpers (IsDataEdge, IsStrongEdge, etc.)
/// provide the coarser categories needed by scheduling algorithms.
struct ScheduleEdge {
  enum Kind {
    // Register dependencies (from SDep::Kind).
    kData,         // True data dependence (RAW). One node produces a value
                   // that the other consumes. Carries latency.
    kAnti,         // Anti-dependence (WAR). One node reads a register before
                   // the other writes it. Pre-RA: physical registers only.
    kOutput,       // Output dependence (WAW). Both nodes write the same
                   // register.

    // Strong ordering constraints (from SDep::Order + SDep::OrderKind).
    // These block readiness: a node cannot be scheduled until all strong
    // predecessors have been scheduled.
    kBarrier,      // Scheduling barrier (calls, inline asm, unmodeled
                   // side effects). Everything above must come before,
                   // everything below must come after.
    kMayAliasMem,  // Memory operations that might alias. Conservative
                   // ordering when alias analysis cannot prove independence.
    kMustAliasMem, // Memory operations that definitely alias. Strict
                   // ordering to preserve memory semantics.
    kArtificial,   // Arbitrary strong edge, typically added by DAG mutations
                   // (e.g., to prevent reordering within a cluster).

    // Weak ordering constraints. These are scheduling hints that do NOT
    // block readiness. The scheduler may violate them if doing so produces
    // a better schedule.
    kCluster,      // Clustering hint: prefer scheduling these nodes
                   // adjacently so the hardware can merge memory operations.
    kWeak,         // Arbitrary weak hint.
  };

  ScheduleNode *node_; // The connected node (predecessor or successor).
  Kind kind_;
  int latency_; // Latency in cycles. Only meaningful for kData edges.

  ScheduleEdge(ScheduleNode *node, Kind kind, int latency = 0)
      : node_(node), kind_(kind), latency_(latency) {}

  // Classification helpers. Member functions defined in the class body are
  // implicitly inline per the C++ standard, so these have zero call overhead.

  /// True for register data dependencies (RAW). These carry latency and
  /// create live ranges.
  bool IsDataEdge() const { return kind_ == kData; }

  /// True for all edges that block readiness. Includes register dependencies
  /// (Data, Anti, Output) and strong ordering constraints (Barrier,
  /// MayAliasMem, MustAliasMem, Artificial).
  bool IsStrongEdge() const { return kind_ <= kArtificial; }

  /// True for edges that are hints only and do not block readiness.
  bool IsWeakEdge() const { return kind_ >= kCluster; }
};

/// A node in a ScheduleGraph. Either a leaf (wrapping a single SUnit) or a
/// group (owning a subgraph of ScheduleNodes). Uses std::variant to make
/// the leaf/group distinction type-safe — it is impossible to have both
/// an SUnit and a subgraph, or neither.
///
/// Note: assert() is compiled out in release builds (NDEBUG is defined),
/// so the GetSUnit/GetSubgraph checks have zero cost in production. They
/// exist only to catch misuse during development and debug builds.
class ScheduleNode {
public:
  /// Create a leaf node wrapping a single SUnit.
  explicit ScheduleNode(SUnit *su);

  /// Create a leaf node with a debug name (for test DAGs without real SUnits).
  ScheduleNode(SUnit *su, std::string debug_name);

  /// Create a group node owning a subgraph.
  explicit ScheduleNode(std::unique_ptr<ScheduleGraph> subgraph);

  bool IsLeaf() const {
    return std::holds_alternative<SUnit *>(content_);
  }

  /// Access the wrapped SUnit. Only valid for leaf nodes.
  SUnit *GetSUnit() const {
    assert(IsLeaf() && "GetSUnit called on group node");
    return std::get<SUnit *>(content_);
  }

  /// Access the owned subgraph. Only valid for group nodes.
  ScheduleGraph *GetSubgraph() const {
    assert(!IsLeaf() && "GetSubgraph called on leaf node");
    return std::get<std::unique_ptr<ScheduleGraph>>(content_).get();
  }

  /// Add a successor edge from this node to the target node. Automatically
  /// adds the matching predecessor edge on the target, ensuring edges are
  /// always symmetric. This works because C++ access control is per-class,
  /// not per-instance — a ScheduleNode method can access private members
  /// of any other ScheduleNode.
  void AddSucc(ScheduleEdge edge) {
    succs_.push_back(edge);
    edge.node_->preds_.push_back(
        ScheduleEdge(this, edge.kind_, edge.latency_));
  }

  /// Read-only edge access.
  ArrayRef<ScheduleEdge> Succs() const { return succs_; }
  ArrayRef<ScheduleEdge> Preds() const { return preds_; }

  int NumPreds() const { return static_cast<int>(preds_.size()); }
  int NumSuccs() const { return static_cast<int>(succs_.size()); }

  /// Sort successor edges by ascending topo index of the destination node.
  /// Called by ComputeTopologicalOrder after topo indices are assigned.
  /// Required by the transitive reduction algorithm (process closest
  /// successors first).
  void SortSuccsByTopoIndex() {
    llvm::sort(succs_, [](const ScheduleEdge &a, const ScheduleEdge &b) {
      return a.node_->GetTopoIndex() < b.node_->GetTopoIndex();
    });
  }

  /// For leaf nodes, returns 1. For group nodes, recursively counts the
  /// total number of leaf nodes across all subgraphs.
  int LeafSize() const;

  /// Topological index. Set by ScheduleGraph::ComputeTopologicalOrder().
  /// -1 means not yet computed.
  int GetTopoIndex() const { return topo_index_; }
  void SetTopoIndex(int index) { topo_index_ = index; }

  /// Universally unique ID across all ScheduleNode and ScheduleGraph
  /// instances. Assigned from a shared counter at construction time.
  int64_t GetId() const { return id_; }

  /// Human-readable description of this node. Format:
  ///   Leaf with instruction:  "[3] S_LOAD_DWORD ..."
  ///   Leaf with debug name:   "[7:A]"
  ///   Leaf with null SUnit:   "[7]"
  ///   Group node:             "[12:graph(5)]"  (5 = subgraph size)
  std::string ToString() const;

private:
  int64_t id_;
  std::variant<SUnit *, std::unique_ptr<ScheduleGraph>> content_;
  SmallVector<ScheduleEdge> succs_;
  SmallVector<ScheduleEdge> preds_;
  int topo_index_ = -1;
  std::string debug_name_;
};

/// A graph of ScheduleNodes. Used at every level of the hierarchy:
///   - Leaf level: each node wraps an SUnit (instruction)
///   - Quotient levels: each node is a group containing a subgraph
///
/// Construction is two-phase: add all nodes first, then add edges. This
/// ensures node pointers (stored in edges) remain stable.
class ScheduleGraph {
public:
  ScheduleGraph();

  int64_t GetId() const { return id_; }

  /// Build a leaf-level graph by copying the dependency structure from an
  /// existing SUnit DAG. Each SUnit becomes a leaf ScheduleNode, and SDep
  /// edges are translated to ScheduleEdges. Boundary nodes (EntrySU,
  /// ExitSU) are included as leaf nodes — they anchor live-in/live-out
  /// dependencies and may be useful for analysis, though they should not
  /// be scheduled.
  static ScheduleGraph BuildFromSUnits(MutableArrayRef<SUnit> sunits,
                                       SUnit &entry_su, SUnit &exit_su);

  /// Build a synthetic test DAG with known structure for testing algorithms
  /// like topological sort, transitive reduction, and dominator trees.
  /// Does not depend on LLVM SUnits — nodes are leaves wrapping nullptr.
  ///
  /// 7 nodes (A-H, no B), 9 edges (all kData):
  ///
  ///              A
  ///            / | \
  ///           /  |  \
  ///          /   |   \
  ///         v    |    v
  ///        H     |     C
  ///        |     |   /  \
  ///        |     v  v    v
  ///        |      D      E
  ///        |       \    /
  ///        |        v  v
  ///        |         F
  ///         \       /
  ///          \     /
  ///           v   v
  ///             G
  ///
  static ScheduleGraph BuildTestDAG();

  /// Build a synthetic test DAG that contains a cycle, for testing that
  /// ComputeTopologicalOrder correctly detects it and calls
  /// report_fatal_error.
  ///
  /// Structure (3 nodes, 3 edges — cycle between B and C):
  ///
  ///     A
  ///     |
  ///     v
  ///     B <---.
  ///     |     |
  ///     v     |
  ///     C ----'
  ///
  /// Edges: A→B, B→C, C→B
  static ScheduleGraph BuildTestDAGWithCycle();

  /// Human-readable identifier for this graph. Format: "graph[ID]"
  std::string ToString() const;

  /// Access the node list.
  MutableArrayRef<ScheduleNode> Nodes() { return nodes_; }
  ArrayRef<ScheduleNode> Nodes() const { return nodes_; }
  int Size() const { return static_cast<int>(nodes_.size()); }

  /// Total number of leaf nodes across all levels of the hierarchy.
  int LeafSize() const;

  /// Compute topological order using Kahn's algorithm (iterative BFS-based).
  /// Populates topo_order_ and sets topo_index_ on each node. Also serves
  /// as a cycle check: asserts if the graph contains a cycle (not all nodes
  /// are reachable).
  ///
  /// If include_weak_edges is false (the default), only strong edges
  /// constrain the ordering. If true, weak edges (Cluster, Weak) also
  /// count as predecessors that must be scheduled first.
  void ComputeTopologicalOrder(bool include_weak_edges = false);

  /// Access the computed topological order. Only valid after
  /// ComputeTopologicalOrder() has been called.
  ArrayRef<ScheduleNode *> TopoOrder() const { return topo_order_; }

  /// Whether ComputeTopologicalOrder() has been called.
  bool IsTopoSorted() const { return topo_sorted_; }

  /// Compute the transitive reduction of this graph. Returns a lightweight
  /// ReducedGraph (adjacency lists indexed by topo index). Requires topo
  /// sort to have been computed first.
  ///
  /// Uses a single-pass algorithm: iterate nodes in reverse topo order,
  /// process successors in ascending topo order (closest first). If a
  /// successor is already reachable via a previously processed closer
  /// path, the direct edge is redundant and discarded.
  ///
  /// Each edge is visited once. For each kept edge, we OR two BitVectors
  /// of size V (V/64 word operations). For sparse DAGs with E ~ kV edges
  /// (k = average degree), this gives ~kV^2/64 total word operations.
  /// Temporary space: one BitVector per node = V^2/8 bytes total.
  ReducedGraph ComputeTransitiveReduction() const;

private:
  int64_t id_;
  std::vector<ScheduleNode> nodes_;
  std::vector<ScheduleNode *> topo_order_;
  bool topo_sorted_ = false;
};

/// Lightweight adjacency-list representation produced by transitive
/// reduction. Indexed by topological index for O(1) access. Does not
/// own nodes — references back to the original ScheduleGraph's nodes
/// via node_map.
///
/// Storage is O(V + E_reduced). The outer std::vector is sized to the
/// number of nodes; each inner SmallVector holds only the edges that
/// survived the reduction (SmallVector because most nodes have few
/// successors, so inline storage avoids per-node heap allocation).
struct ReducedGraph {
  int size;                                // number of nodes
  std::vector<SmallVector<int>> succs;     // succs[topo_idx] = successor topo indices
  std::vector<SmallVector<int>> preds;     // preds[topo_idx] = predecessor topo indices

  /// Construct with a given number of nodes. Allocates empty adjacency
  /// lists of the specified size.
  explicit ReducedGraph(int num_nodes)
      : size(num_nodes), succs(num_nodes), preds(num_nodes) {}
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULEGRAPH_H
