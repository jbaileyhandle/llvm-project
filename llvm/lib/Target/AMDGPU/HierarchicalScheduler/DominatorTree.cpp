//===- DominatorTree.cpp - Dominator tree for ScheduleGraphs --------------===//
//
// Implementation of dominator tree construction for DAGs.
//
// Based on the Cooper-Harvey-Kennedy (CHK) algorithm, but simplified: since
// our ReducedGraph is a DAG (no cycles), a single pass in topological order
// is sufficient — the iterative fixpoint loop from the original CHK paper
// (designed for general CFGs with back-edges) is not needed.
//
// Reference: Cooper, Harvey, Kennedy. "A Simple, Fast Dominance Algorithm."
// Software Practice and Experience, 2001.
//
//===----------------------------------------------------------------------===//

#include "DominatorTree.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

/// Find the nearest common dominator of two nodes by walking up the
/// dominator tree from both until they meet. Uses topo indices directly —
/// a lower topo index means the node is "higher" (closer to the root).
///
/// -1 represents the virtual root and dominates everything.
int Intersect(const std::vector<int> &idom, int finger_a, int finger_b) {
  while (finger_a != finger_b) {
    if (finger_a == -1 || finger_b == -1) {
      return -1;
    }
    while (finger_a > finger_b) {
      finger_a = idom[finger_a];
      if (finger_a == -1) {
        return -1;
      }
    }
    while (finger_b > finger_a) {
      finger_b = idom[finger_b];
      if (finger_b == -1) {
        return -1;
      }
    }
  }
  return finger_a;
}

} // anonymous namespace

DominatorTree DominatorTree::Build(const ReducedGraph &graph) {
  DominatorTree tree;
  int num_nodes = graph.size;
  tree.idom_by_topo_index_.assign(num_nodes, -1);

  // Single pass in topological order. Since this is a DAG, all predecessors
  // of node N have topo index < N and are already processed by the time we
  // reach N. No fixpoint iteration needed.
  for (int topo_idx = 0; topo_idx < num_nodes; ++topo_idx) {
    if (graph.predecessors_by_topo_index[topo_idx].empty()) {
      // Root node — dominated by the virtual root.
      tree.idom_by_topo_index_[topo_idx] = -1;
      continue;
    }

    // Start with the first predecessor as the candidate idom.
    int new_idom = graph.predecessors_by_topo_index[topo_idx][0];

    // Intersect with each remaining predecessor to find their nearest
    // common dominator.
    for (int pred_idx = 1;
         pred_idx < static_cast<int>(graph.predecessors_by_topo_index[topo_idx].size());
         ++pred_idx) {
      new_idom = Intersect(tree.idom_by_topo_index_, new_idom, graph.predecessors_by_topo_index[topo_idx][pred_idx]);
    }

    tree.idom_by_topo_index_[topo_idx] = new_idom;
  }

  return tree;
}

bool DominatorTree::Dominates(int dominator, int node) const {
  // The virtual root dominates everything.
  if (dominator == -1) {
    return true;
  }

  // Walk up from node until we find dominator or reach the virtual root.
  int current = node;
  while (current != -1) {
    if (current == dominator) {
      return true;
    }
    current = idom_by_topo_index_[current];
  }
  return false;
}

std::string DominatorTree::ToString(const ScheduleGraph &graph) const {
  std::string result;
  ArrayRef<ScheduleNode *> topo_order = graph.GetTopoOrder();

  for (int topo_idx = 0; topo_idx < Size(); ++topo_idx) {
    result += "  idom(" + topo_order[topo_idx]->ToString() + ") = ";
    if (idom_by_topo_index_[topo_idx] == -1) {
      result += "ROOT";
    } else {
      result += topo_order[idom_by_topo_index_[topo_idx]]->ToString();
    }
    result += "\n";
  }
  return result;
}
