//===- SubgraphFormation.cpp - Subgraph formation for ScheduleGraph -------===//
//
// Implementation of SubgraphFormation.h. See
// AMDGPUSubgraphFormationDesign.md for the design.
//
//===----------------------------------------------------------------------===//

#include "SubgraphFormation.h"
#include "llvm/Support/ErrorHandling.h"

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

namespace {

// Pass 1 of BuildFromDominatorTree: identity, parent wiring, splitter
// caching. Reads idom[i] -> parent_topo_idx, sets the parent pointer
// on every node, and remembers the unique root. Allocates `nodes_`
// to graph.Size() entries up front so the parent pointers it stores
// stay stable for the tree's lifetime (no later resize).
void WireIdentityAndParents(
    SubgraphFormationTree &tree,
    std::vector<SubgraphFormationTreeNode> &nodes,
    SubgraphFormationTreeNode *&root,
    const ScheduleGraph &graph,
    const DominatorTree &dom,
    llvm::function_ref<bool(const ScheduleNode *)> is_splitter) {
  int n = graph.Size();
  nodes.resize(n);
  ArrayRef<ScheduleNode *> topo = graph.GetTopoOrder();
  for (int i = 0; i < n; ++i) {
    SubgraphFormationTreeNode &node = nodes[i];
    node.topo_idx = i;
    node.schedule_node = topo[i];
    node.is_splitter = is_splitter(topo[i]);
    int idom = dom.GetIDomByTopoIndex(i);
    if (idom == -1) {
      // Single-source graphs (which is what BuildFromSUnits produces)
      // have exactly one root; remember it for Root().
      if (root != nullptr) {
        report_fatal_error("SubgraphFormationTree::BuildFromDominatorTree "
                           "saw multiple roots; expected single-source graph");
      }
      root = &node;
    } else {
      node.parent = &nodes[idom];
    }
  }
  if (root == nullptr) {
    report_fatal_error("SubgraphFormationTree::BuildFromDominatorTree "
                       "found no root");
  }
  (void)tree;
}

// Pass 2: children lists. With parent pointers in hand we can populate
// every children vector in one linear scan.
void PopulateChildrenLists(std::vector<SubgraphFormationTreeNode> &nodes) {
  for (SubgraphFormationTreeNode &node : nodes) {
    if (node.parent != nullptr) {
      node.parent->children.push_back(&node);
    }
  }
}

// Pass 3: subtree counts. Each node sums its children's counts and
// adds its own contribution. Post-order so children are ready when a
// parent is visited.
void ComputeSubtreeCounts(SubgraphFormationTreeNode *root) {
  PostOrderApply(root, [](SubgraphFormationTreeNode *node) {
    node->subtree_splitter_count = node->is_splitter ? 1 : 0;
    node->subtree_node_count = 1;
    for (auto *c : node->children) {
      node->subtree_splitter_count += c->subtree_splitter_count;
      node->subtree_node_count += c->subtree_node_count;
    }
  });
}

// Pass 4: DFS pre/post numbering. Two separate counters give the
// O(1) ancestor test its cleanest form (see IsInSubtreeOf).
void AssignDfsNumbers(SubgraphFormationTreeNode *root) {
  int pre_counter = 0;
  PreOrderApply(root, [&pre_counter](SubgraphFormationTreeNode *node) {
    node->dfs_pre = pre_counter++;
  });
  int post_counter = 0;
  PostOrderApply(root, [&post_counter](SubgraphFormationTreeNode *node) {
    node->dfs_post = post_counter++;
  });
}

} // namespace

SubgraphFormationTree SubgraphFormationTree::BuildFromDominatorTree(
    const ScheduleGraph &graph,
    const DominatorTree &dom,
    llvm::function_ref<bool(const ScheduleNode *)> is_splitter) {
  if (!graph.IsTopoSorted()) {
    report_fatal_error("SubgraphFormationTree::BuildFromDominatorTree "
                       "requires graph.IsTopoSorted()");
  }
  SubgraphFormationTree tree;
  WireIdentityAndParents(tree, tree.nodes_, tree.root_, graph, dom,
                         is_splitter);
  PopulateChildrenLists(tree.nodes_);
  ComputeSubtreeCounts(tree.root_);
  AssignDfsNumbers(tree.root_);
  return tree;
}

void SubgraphFormationTree::RecordEmission(SubgraphFormationTreeNode *node) {
  if (node->emitted) {
    return;
  }
  node->emitted = true;
  emit_points_.push_back(node);
  // Walk the parent chain, marking descendants_emitted on every
  // ancestor. We can stop early as soon as we hit an ancestor that
  // already has the flag — every ancestor above it must already be
  // marked too (invariant of how RecordEmission propagates).
  for (SubgraphFormationTreeNode *a = node->parent; a != nullptr;
       a = a->parent) {
    if (a->descendants_emitted) {
      break;
    }
    a->descendants_emitted = true;
  }
}

} // namespace hierarchical_scheduler
} // namespace llvm
