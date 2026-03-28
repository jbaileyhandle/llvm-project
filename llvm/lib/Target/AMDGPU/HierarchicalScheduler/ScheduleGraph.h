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
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/ScheduleDAG.h"
#include <cassert>
#include <memory>
#include <variant>
#include <vector>

namespace llvm {
namespace hierarchical_scheduler {

class ScheduleGraph;
class ScheduleNode;

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
/// exist only to catch misuse during development and debug builds. Our
/// CMakeLists.txt uses -UNDEBUG to keep asserts enabled for this library
/// even in release builds.
class ScheduleNode {
public:
  /// Create a leaf node wrapping a single SUnit.
  explicit ScheduleNode(SUnit *su) : content_(su) {}

  /// Create a group node owning a subgraph.
  explicit ScheduleNode(std::unique_ptr<ScheduleGraph> subgraph)
      : content_(std::move(subgraph)) {}

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

  /// For leaf nodes, returns 1. For group nodes, recursively counts the
  /// total number of leaf nodes across all subgraphs.
  int LeafSize() const;

private:
  std::variant<SUnit *, std::unique_ptr<ScheduleGraph>> content_;
  SmallVector<ScheduleEdge> succs_;
  SmallVector<ScheduleEdge> preds_;
};

/// A graph of ScheduleNodes. Used at every level of the hierarchy:
///   - Leaf level: each node wraps an SUnit (instruction)
///   - Quotient levels: each node is a group containing a subgraph
///
/// Construction is two-phase: add all nodes first, then add edges. This
/// ensures node pointers (stored in edges) remain stable.
class ScheduleGraph {
public:
  ScheduleGraph() = default;

  /// Build a leaf-level graph by copying the dependency structure from an
  /// existing SUnit DAG. Each SUnit becomes a leaf ScheduleNode, and SDep
  /// edges are translated to ScheduleEdges. Boundary nodes (EntrySU,
  /// ExitSU) are included as leaf nodes — they anchor live-in/live-out
  /// dependencies and may be useful for analysis, though they should not
  /// be scheduled.
  static ScheduleGraph BuildFromSUnits(MutableArrayRef<SUnit> sunits,
                                       SUnit &entry_su, SUnit &exit_su);

  /// Access the node list.
  MutableArrayRef<ScheduleNode> Nodes() { return nodes_; }
  ArrayRef<ScheduleNode> Nodes() const { return nodes_; }
  int Size() const { return static_cast<int>(nodes_.size()); }

  /// Total number of leaf nodes across all levels of the hierarchy.
  int LeafSize() const;

private:
  std::vector<ScheduleNode> nodes_;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULEGRAPH_H
