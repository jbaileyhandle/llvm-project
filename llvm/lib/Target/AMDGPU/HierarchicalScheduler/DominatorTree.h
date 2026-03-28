//===- DominatorTree.h - Dominator tree for ScheduleGraphs ------*- C++ -*-===//
//
// Dominator tree built from a ReducedGraph. A node D dominates a node N if
// every path from the root to N must pass through D. The immediate dominator
// (idom) of N is the closest strict dominator of N — the last dominator
// before N on any path from the root.
//
// Uses the Cooper-Harvey-Kennedy (CHK) algorithm, which iteratively refines
// idom[] using the intersect (LCA) function. Converges quickly in practice
// (typically 2-3 iterations for scheduling DAGs).
//
// Handles multi-root DAGs (multiple nodes with no predecessors) by adding
// an implicit virtual root that dominates all real roots. The virtual root
// is at index -1 and does not appear in the ReducedGraph or ScheduleGraph.
//
// All queries are by topological index. Use the original ScheduleGraph's
// TopoOrder() to map topo indices back to ScheduleNode pointers.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_DOMINATORTREE_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_DOMINATORTREE_H

#include "ScheduleGraph.h"
#include <string>
#include <vector>

namespace llvm {
namespace hierarchical_scheduler {

class DominatorTree {
public:
  /// Build a dominator tree from a reduced graph using the CHK algorithm.
  /// If the graph has multiple root nodes (no predecessors), they are all
  /// dominated by an implicit virtual root at index -1.
  static DominatorTree Build(const ReducedGraph &graph);

  /// Get the immediate dominator of a node (by topo index).
  /// Returns -1 for nodes dominated directly by the virtual root
  /// (including single-root DAGs where the root's idom is -1).
  int GetIDom(int topo_idx) const { return idom_[topo_idx]; }

  /// Does node at topo index |dominator| dominate node at topo index |node|?
  /// A node dominates itself. The virtual root (-1) dominates everything.
  bool Dominates(int dominator, int node) const;

  /// Number of nodes in the tree (not counting the virtual root).
  int Size() const { return static_cast<int>(idom_.size()); }

  /// Human-readable dump of the dominator tree. Requires the original
  /// ScheduleGraph to map topo indices to node names.
  std::string ToString(const ScheduleGraph &graph) const;

private:
  // idom_[topo_idx] = topo index of the immediate dominator.
  // -1 means the node is dominated by the virtual root (i.e., it is a
  // root node, or the single entry node of the DAG).
  std::vector<int> idom_;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_DOMINATORTREE_H
