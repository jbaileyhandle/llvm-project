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

// --- Per-decision-rule passes (§5 of design doc) -------------------------

void BottomUpSingleSplitterPass(SubgraphFormationTree &tree) {
  PostOrderApply(tree.Root(), [&tree](SubgraphFormationTreeNode *n) {
    if (n->emitted || n->descendants_emitted) {
      return;
    }
    if (n->subtree_splitter_count == 1 && n->subtree_node_count > 1) {
      tree.RecordEmission(n);
    }
  });
}

void TopDownSingleSplitterPass(SubgraphFormationTree &tree) {
  PreOrderApplyWithSkip(tree.Root(), [&tree](SubgraphFormationTreeNode *n) {
    // Three-way dispatch — each behaviorally distinct:
    if (n->emitted) {
      // Subtree already wrapped by a prior pass. Nothing to do here
      // and no need to descend (members are wholly consumed).
      return true;
    }
    if (n->descendants_emitted) {
      // Some descendant emitted; emitting *here* would nest. But an
      // unrelated child branch could still satisfy the rule
      // independently — descend, and the emitted descendants get
      // caught by the `emitted` branch when visited.
      return false;
    }
    if (n->subtree_splitter_count == 1 && n->subtree_node_count > 1) {
      tree.RecordEmission(n);
      return true; // Subtree consumed; skip descent.
    }
    return false;
  });
}

void MultiSplitterRescuePass(SubgraphFormationTree &tree) {
  PostOrderApply(tree.Root(), [&tree](SubgraphFormationTreeNode *n) {
    if (n->emitted || n->descendants_emitted) {
      return;
    }
    // The node_count > splitter_count guard ensures the resulting
    // (multi-splitter) subgraph has bubble-filler material inside its
    // scope — without it, every member would be a splitter and there
    // would be nothing to overlap with their multi-cycle bubbles.
    if (n->subtree_splitter_count > 1 &&
        n->subtree_node_count > n->subtree_splitter_count) {
      tree.RecordEmission(n);
    }
  });
}

void LargeSplitterFreeRescuePass(SubgraphFormationTree &tree,
                                 int size_threshold) {
  PostOrderApply(tree.Root(),
                 [&tree, size_threshold](SubgraphFormationTreeNode *n) {
                   if (n->emitted || n->descendants_emitted) {
                     return;
                   }
                   if (n->subtree_splitter_count == 0 &&
                       n->subtree_node_count > size_threshold) {
                     tree.RecordEmission(n);
                   }
                 });
}

void SiblingRescuePass(SubgraphFormationTree &tree, int min_size) {
  PreOrderApplyWithSkip(tree.Root(),
                        [&tree, min_size](SubgraphFormationTreeNode *n) {
    if (n->emitted) {
      return true; // Subtree wholly consumed; skip descent.
    }
    // Cascading trigger: rescue fires at any non-emitted node when
    // any descendant of any child has been emitted (immediate-child
    // emit OR deeper). Captures the case where cohesion was
    // identified deep in one branch and we want sibling branches to
    // get the chance to be wrapped too.
    bool trigger = false;
    for (auto *c : n->children) {
      if (c->emitted || c->descendants_emitted) {
        trigger = true;
        break;
      }
    }
    if (trigger) {
      for (auto *c : n->children) {
        // Clean-subtree guard: rescuing a subtree with emissions
        // inside would create overlap (rescue would share members
        // with the inner subgraph).
        if (c->emitted || c->descendants_emitted) {
          continue;
        }
        // Min-size guard: a small rescue is mostly proxy overhead.
        if (c->subtree_node_count <= min_size) {
          continue;
        }
        tree.RecordEmission(c);
      }
    }
    return false; // Continue descending into non-emitted children.
  });
}

// --- Splitting + BuildSubgraphInfos (§4.4, §4.5) -------------------------

namespace {

// Walk an emit-point's subtree once, gathering members and locating
// the splitter. The single-splitter caller invariant (verified at
// call site) guarantees exactly one splitter exists.
void CollectSubtreeMembersAndFindSplitter(
    SubgraphFormationTreeNode *emit_point,
    SmallVectorImpl<ScheduleNode *> &members_out,
    ScheduleNode *&splitter_out) {
  splitter_out = nullptr;
  PostOrderApply(emit_point,
                 [&members_out, &splitter_out](SubgraphFormationTreeNode *n) {
                   members_out.push_back(n->schedule_node);
                   if (n->is_splitter) {
                     splitter_out = n->schedule_node;
                   }
                 });
}

// If `members` has at least 2 entries, package them into a new
// SubgraphInfo with name "subgraph_<id>", incrementing `next_id`.
// Singletons are dropped (proxy overhead would outweigh benefit).
void EmitIfLargeEnough(
    ArrayRef<ScheduleNode *> members,
    int &next_id,
    std::vector<std::unique_ptr<SubgraphInfo>> &out) {
  if (members.size() < 2) {
    return;
  }
  std::string name = "subgraph_" + std::to_string(next_id++);
  out.push_back(std::make_unique<SubgraphInfo>(members, name));
}

} // namespace

SplitterSplitResult SplitEmitPoint(SubgraphFormationTreeNode *emit_point,
                                   const ScheduleGraph &graph,
                                   SplitterPartitionPolicy policy) {
  SmallVector<ScheduleNode *, 16> members;
  ScheduleNode *splitter = nullptr;
  CollectSubtreeMembersAndFindSplitter(emit_point, members, splitter);
  if (splitter == nullptr) {
    report_fatal_error("SplitEmitPoint called on emit point without a "
                       "splitter — caller invariant violated");
  }

  SplitterSplitResult result;
  int splitter_idx = splitter->GetTopoIndex();
  for (ScheduleNode *m : members) {
    if (m == splitter) {
      // Splitter is standalone — not a member of any group.
      continue;
    }
    int member_idx = m->GetTopoIndex();
    bool reaches_splitter = graph.IsReachableInDag(member_idx, splitter_idx);
    if (reaches_splitter) {
      result.ancestors_of_splitter.push_back(m);
      continue;
    }
    if (policy == SplitterPartitionPolicy::kBundleDescendantsAndIndependents) {
      result.descendants_or_other.push_back(m);
    } else if (graph.IsReachableInDag(splitter_idx, member_idx)) {
      result.descendants_of_splitter.push_back(m);
    } else {
      result.independents.push_back(m);
    }
  }
  return result;
}

std::vector<std::unique_ptr<SubgraphInfo>> BuildSubgraphInfos(
    ArrayRef<SubgraphFormationTreeNode *> emit_points,
    const ScheduleGraph &graph,
    SplitterPartitionPolicy policy) {
  std::vector<std::unique_ptr<SubgraphInfo>> out;
  int next_id = 0;
  for (SubgraphFormationTreeNode *ep : emit_points) {
    if (ep->subtree_splitter_count == 1) {
      // Single-splitter — split and emit each non-empty group.
      SplitterSplitResult split = SplitEmitPoint(ep, graph, policy);
      EmitIfLargeEnough(split.ancestors_of_splitter, next_id, out);
      if (policy == SplitterPartitionPolicy::kBundleDescendantsAndIndependents) {
        EmitIfLargeEnough(split.descendants_or_other, next_id, out);
      } else {
        EmitIfLargeEnough(split.descendants_of_splitter, next_id, out);
        EmitIfLargeEnough(split.independents, next_id, out);
      }
    } else {
      // 0 or 2+ splitters — emit the whole subtree as one
      // SubgraphInfo. The multi-splitter rescue pass already
      // verified subtree_node_count > subtree_splitter_count, so
      // there's enough non-splitter material in scope to fill
      // bubbles.
      SmallVector<ScheduleNode *, 16> members;
      PostOrderApply(ep, [&members](SubgraphFormationTreeNode *n) {
        members.push_back(n->schedule_node);
      });
      EmitIfLargeEnough(members, next_id, out);
    }
  }
  return out;
}

// --- Pipelines (§6) ------------------------------------------------------
//
// Each entry in pipeline.passes is a std::function<void(tree &)> —
// a one-argument callable. BottomUpSingleSplitterPass etc. already
// match that signature, so they go in directly.
// LargeSplitterFreeRescuePass takes a second arg (size_threshold)
// so we wrap it in a lambda that binds the threshold; same pattern
// would apply to SiblingRescuePass(tree, min_size) if it were ever
// added to a default pipeline.

SubgraphFormationPolicy SubgraphFormationPolicy::BottomUpDefault() {
  SubgraphFormationPolicy p;
  int t = p.large_subtree_threshold;
  p.pipeline.passes = {
      BottomUpSingleSplitterPass,
      MultiSplitterRescuePass,
      [t](SubgraphFormationTree &tree) {
        LargeSplitterFreeRescuePass(tree, t);
      },
      // SiblingRescuePass deliberately omitted — see §5.5.
  };
  return p;
}

SubgraphFormationPolicy SubgraphFormationPolicy::TopDownAggressive() {
  SubgraphFormationPolicy p;
  int t = p.large_subtree_threshold;
  p.pipeline.passes = {
      TopDownSingleSplitterPass,    // <-- only difference vs BottomUpDefault
      MultiSplitterRescuePass,
      [t](SubgraphFormationTree &tree) {
        LargeSplitterFreeRescuePass(tree, t);
      },
      // SiblingRescuePass deliberately omitted — see §5.5.
  };
  return p;
}

SubgraphFormationPolicy SubgraphFormationPolicy::TopDownSingleSplitterOnly() {
  SubgraphFormationPolicy p;
  p.pipeline.passes = {
      TopDownSingleSplitterPass,
  };
  // Explicit (matches the default), so the partition choice is
  // visible at the factory: independents merge into the
  // descendants_or_other group rather than becoming their own
  // subgraph.
  p.splitter_partition =
      SplitterPartitionPolicy::kBundleDescendantsAndIndependents;
  return p;
}

// --- End-to-end driver (§7) ----------------------------------------------

void FormSubgraphs(ScheduleGraph &graph,
                   const SubgraphFormationPolicy &policy) {
  // Empty pipeline → nothing to do, and we skip the prereq
  // analyses (TR / dom) so callers that pass an empty policy as a
  // "no formation" sentinel pay nothing beyond this check.
  if (policy.pipeline.passes.empty()) {
    return;
  }

  // 1. Prerequisite analyses. Each ScheduleGraph::Compute* call is
  // cache-aware (early-returns if its result is already current),
  // so re-running on a freshly built graph that hasn't seen them
  // yet pays the full cost; re-running on a graph that already has
  // them costs O(1) per check.
  graph.ValidateAndComputeTopologicalOrder();
  graph.ComputeTransitiveReductionAndReachability();
  graph.ComputeDominatorTree();

  // 2. Formation tree.
  auto is_splitter = [&policy](const ScheduleNode *n) {
    return IsSubgraphSplitter(n, policy.latency_threshold);
  };
  SubgraphFormationTree tree =
      SubgraphFormationTree::BuildFromDominatorTree(
          graph, graph.GetDominatorTree(), is_splitter);

  // 3. Pipeline.
  for (auto &pass : policy.pipeline.passes) {
    pass(tree);
  }

  // 4. Materialize. Single-splitter emit points get partitioned
  // here; multi/zero-splitter emit as-is; singletons are dropped.
  std::vector<std::unique_ptr<SubgraphInfo>> infos =
      BuildSubgraphInfos(tree.EmitPoints(), graph,
                         policy.splitter_partition);

  // 5. Mutate. InsertSubgraphProxies takes the vector by value and
  // moves each unique_ptr into its start proxy node. No-op if
  // `infos` is empty (and in that case the graph isn't mutated, so
  // the re-derive below short-circuits).
  graph.InsertSubgraphProxies(std::move(infos));

  // 6. Re-derive topo + critical-paths so downstream consumers see
  // the post-mutation graph (proxies + artificial edges). Both
  // calls early-return if no mutation happened above.
  graph.ValidateAndComputeTopologicalOrder();
  graph.ComputeCriticalPaths();
}

} // namespace hierarchical_scheduler
} // namespace llvm
