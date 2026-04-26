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

#include "DominatorTree.h"
#include "ScheduleGraph.h"
#include "SubgraphInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include <functional>
#include <memory>
#include <vector>

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

/// One node in a SubgraphFormationTree. Pointer-based (parent and
/// children are stored by pointer) so traversal reads naturally,
/// and so that the per-node static counts and mutable emission
/// flags travel with the node rather than being held in parallel
/// arrays. See AMDGPUSubgraphFormationDesign.md §4.2.
struct SubgraphFormationTreeNode {
  /// Topo index of the underlying ScheduleNode in its ScheduleGraph.
  int topo_idx = -1;

  /// Backpointer to the ScheduleNode this tree node mirrors. Used by
  /// passes (to assemble member lists) and by splitting (to query
  /// reachability against the splitter).
  ScheduleNode *schedule_node = nullptr;

  /// Tree structure. `parent == nullptr` iff this is the root.
  SubgraphFormationTreeNode *parent = nullptr;
  SmallVector<SubgraphFormationTreeNode *, 4> children;

  // --- Static state. Populated by BuildFromDominatorTree, never
  //     changed afterward.

  /// IsSubgraphSplitter(schedule_node, threshold) — cached so passes
  /// don't re-walk successors per query.
  bool is_splitter = false;

  /// Number of splitters in the subtree rooted at this node,
  /// inclusive of self.
  int subtree_splitter_count = 0;

  /// Number of tree nodes in the subtree rooted at this node,
  /// inclusive of self.
  int subtree_node_count = 0;

  /// DFS pre/post numbers (separate counters), assigned during
  /// Build by one PreOrderApply pass and one PostOrderApply pass.
  /// Together they give an O(1) ancestor/descendant test:
  ///   IsInSubtreeOf(d, a) ==
  ///       a->dfs_pre <= d->dfs_pre && d->dfs_post <= a->dfs_post
  /// Useful for any "is X in Y's subtree?" question across passes
  /// — see SubgraphFormationTree::IsInSubtreeOf.
  int dfs_pre = -1;
  int dfs_post = -1;

  // --- Mutable state. Set by SubgraphFormationTree::RecordEmission
  //     and read by passes through their descendant-emit guards.

  /// True iff this node has been chosen as an emit point by some
  /// pass. The set of emit points after the pipeline finishes
  /// determines the final list of SubgraphInfos.
  bool emitted = false;

  /// True iff some descendant of this node has been emitted (any
  /// depth). Set as a side effect of RecordEmission walking the
  /// parent chain. Lets passes guard against producing nested
  /// (overlapping) subgraphs without re-walking the subtree.
  bool descendants_emitted = false;
};

/// A pointer-based tree built from a dominator tree (or, in the
/// future, an immediate post-dominator tree — see §3.4 of the
/// design doc), holding the per-node state subgraph formation
/// needs. Separate from DominatorTree so its mutable emission
/// state doesn't contaminate the dom tree's read-only role and so
/// a future post-dom backbone can be a parallel factory.
class SubgraphFormationTree {
 public:
  SubgraphFormationTree() = default;

  // Move-only. Per-node parent / children / root_ / emit_points_
  // pointers index into nodes_; copying the tree would copy the
  // vector to a new heap buffer while leaving every internal pointer
  // aimed at the old one (silent corruption). Move is fine because
  // std::vector move just transfers the existing buffer — its
  // address, and so every internal pointer, stays valid.
  //
  // Defaulting moves explicitly is required because user-declaring
  // the deleted copies would otherwise also suppress the implicit
  // move ops (which would break factory-by-value returns).
  SubgraphFormationTree(const SubgraphFormationTree &) = delete;
  SubgraphFormationTree &operator=(const SubgraphFormationTree &) = delete;
  SubgraphFormationTree(SubgraphFormationTree &&) = default;
  SubgraphFormationTree &operator=(SubgraphFormationTree &&) = default;

  /// Build a formation tree from `graph`'s dominator tree. The
  /// result has one tree node per ScheduleNode, indexed internally
  /// by topo index. `is_splitter` is invoked once per node and the
  /// answer cached on the node — passes shouldn't have to re-query
  /// the predicate.
  ///
  /// Requires graph.IsTopoSorted() and graph.HasDominatorTree().
  static SubgraphFormationTree BuildFromDominatorTree(
      const ScheduleGraph &graph,
      const DominatorTree &dom,
      llvm::function_ref<bool(const ScheduleNode *)> is_splitter);

  /// Root of the tree. Single-source graphs (which is what
  /// BuildFromSUnits always produces) have exactly one root.
  SubgraphFormationTreeNode *Root() { return root_; }
  const SubgraphFormationTreeNode *Root() const { return root_; }

  /// Lookup a tree node by its underlying ScheduleNode.
  SubgraphFormationTreeNode *GetNode(const ScheduleNode *n) {
    return GetNodeByTopoIndex(n->GetTopoIndex());
  }

  /// Lookup by topo index.
  SubgraphFormationTreeNode *GetNodeByTopoIndex(int topo_idx) {
    return &nodes_[topo_idx];
  }

  /// Mark `node` as an emit point and propagate the
  /// `descendants_emitted` flag up the parent chain. Idempotent
  /// (calling on an already-emitted node is a no-op). Every pass
  /// that wants to mark an emit point goes through this single
  /// mutator so the descendant-emit invariant holds across
  /// arbitrary pass compositions.
  void RecordEmission(SubgraphFormationTreeNode *node);

  /// All nodes that have been marked as emit points, in the order
  /// RecordEmission first fired on each. Maintained incrementally
  /// by RecordEmission (single push inside the same idempotency
  /// guard that sets `emitted`), so this is O(1) — no whole-tree
  /// scan. Each emit point becomes one or more SubgraphInfos in
  /// BuildSubgraphInfos (single-splitter emit points are split
  /// there; zero/multi-splitter emit as-is).
  ArrayRef<SubgraphFormationTreeNode *> EmitPoints() const {
    return emit_points_;
  }

  /// True iff `descendant` lies in `ancestor`'s subtree (inclusive
  /// — a node is in its own subtree). O(1) via the dfs_pre/dfs_post
  /// numbers populated at Build time.
  static bool IsInSubtreeOf(const SubgraphFormationTreeNode *descendant,
                            const SubgraphFormationTreeNode *ancestor) {
    return ancestor->dfs_pre <= descendant->dfs_pre &&
           descendant->dfs_post <= ancestor->dfs_post;
  }

 private:
  /// One entry per ScheduleNode, indexed by topo index. Sized once
  /// in Build and never resized, so node pointers (used as
  /// parent/children links) stay stable for the tree's lifetime.
  std::vector<SubgraphFormationTreeNode> nodes_;

  /// Backpointer to the unique root (single-source graphs only).
  SubgraphFormationTreeNode *root_ = nullptr;

  /// Emit points in first-emission order. Pushed by RecordEmission
  /// inside its idempotency guard, so each entry appears exactly
  /// once and the list stays in lockstep with the per-node
  /// `emitted` flag.
  std::vector<SubgraphFormationTreeNode *> emit_points_;
};

// --- Traversal helpers ----------------------------------------------------
//
// Three patterns capture every traversal the passes need: post-order
// (used by passes whose rule decides "what to do with this node after
// its descendants are processed"), plain pre-order (used by Build's
// dfs_pre numbering and by passes that visit every node top-down), and
// pre-order with a subtree-skip lambda (used by passes that decide
// "emit here? then skip descendants").
//
// Each pattern is provided in two forms:
//   - The default recursive form (3 lines, trivially correct, matches
//     the mental model of the traversal). Linux's 8 MB default stack
//     handles instruction-level dom-tree depths approaching 20k with
//     comfortable margin.
//   - An iterative *Iterative variant (heap-backed stack), kept for
//     environments with tighter per-thread stack budgets (Windows
//     1 MB default, threaded contexts) or for genuinely pathological
//     depths beyond what we currently expect. Not unused-and-deletable
//     — they exist as the ready substitute and as documentation that
//     the choice was deliberate.

/// Post-order — recursive. Default for "process children, then self."
template <typename Fn>
void PostOrderApply(SubgraphFormationTreeNode *n, Fn fn) {
  for (auto *c : n->children) {
    PostOrderApply(c, fn);
  }
  fn(n);
}

/// Post-order — iterative (heap-backed stack).
template <typename Fn>
void PostOrderApplyIterative(SubgraphFormationTreeNode *root, Fn fn) {
  // Stack frames hold (node, next_child_index_to_descend_into).
  SmallVector<std::pair<SubgraphFormationTreeNode *, int>, 32> stack;
  stack.push_back({root, 0});
  while (!stack.empty()) {
    auto &top = stack.back();
    SubgraphFormationTreeNode *n = top.first;
    int &idx = top.second;
    if (idx < static_cast<int>(n->children.size())) {
      SubgraphFormationTreeNode *child = n->children[idx];
      ++idx;
      stack.push_back({child, 0});
    } else {
      fn(n);
      stack.pop_back();
    }
  }
}

/// Pre-order — recursive. No skip semantic; visits every node.
template <typename Fn>
void PreOrderApply(SubgraphFormationTreeNode *n, Fn fn) {
  fn(n);
  for (auto *c : n->children) {
    PreOrderApply(c, fn);
  }
}

/// Pre-order — iterative.
template <typename Fn>
void PreOrderApplyIterative(SubgraphFormationTreeNode *root, Fn fn) {
  SmallVector<SubgraphFormationTreeNode *, 32> stack;
  stack.push_back(root);
  while (!stack.empty()) {
    SubgraphFormationTreeNode *n = stack.pop_back_val();
    fn(n);
    for (auto *c : n->children) {
      stack.push_back(c);
    }
  }
}

/// Pre-order with subtree-skip — recursive. The lambda returns true
/// to mean "I'm done with this subtree; skip its descendants."
template <typename Fn>
void PreOrderApplyWithSkip(SubgraphFormationTreeNode *n, Fn fn) {
  if (fn(n)) {
    return;
  }
  for (auto *c : n->children) {
    PreOrderApplyWithSkip(c, fn);
  }
}

/// Pre-order with subtree-skip — iterative. Same contract.
template <typename Fn>
void PreOrderApplyWithSkipIterative(SubgraphFormationTreeNode *root, Fn fn) {
  SmallVector<SubgraphFormationTreeNode *, 32> stack;
  stack.push_back(root);
  while (!stack.empty()) {
    SubgraphFormationTreeNode *n = stack.pop_back_val();
    if (fn(n)) {
      continue;
    }
    for (auto *c : n->children) {
      stack.push_back(c);
    }
  }
}

// --- Per-decision-rule passes --------------------------------------------
//
// Each pass is a free function that takes SubgraphFormationTree &
// and mutates it via RecordEmission. Reading any one pass tells you
// exactly what its rule does — no flags, no shared business logic.
// Every pass is guarded against double-emission and nesting (with
// one nuance for top-down passes — see TopDownSingleSplitterPass)
// so pipelines can chain them in any order without nesting bugs.
// See AMDGPUSubgraphFormationDesign.md §5.

/// Emit at the lowest dom-subtree on each branch where the subtree
/// has exactly one splitter and more than one node. "Lowest"
/// emerges from post-order + the descendant-emit guard: the deepest
/// qualifying node on each branch wins; once it emits, ancestors
/// see descendants_emitted == true and skip.
void BottomUpSingleSplitterPass(SubgraphFormationTree &tree);

/// Emit at the highest dom-subtree on each branch where the subtree
/// has exactly one splitter and more than one node. Pre-order with
/// subtree-skip: when a node satisfies the rule, emit it and skip
/// descent. The descendants_emitted guard is intentionally
/// non-blocking — see implementation for the three-way dispatch.
void TopDownSingleSplitterPass(SubgraphFormationTree &tree);

/// Bottom-up. Emit where the subtree has more than one splitter AND
/// more nodes than splitters (so there's non-splitter material
/// inside the resulting scope to fill bubbles).
void MultiSplitterRescuePass(SubgraphFormationTree &tree);

/// Bottom-up. Emit where the subtree has zero splitters and more
/// than `size_threshold` nodes. The threshold is policy — a guess
/// to be tuned with measurement; the design doc default is 24.
void LargeSplitterFreeRescuePass(SubgraphFormationTree &tree,
                                 int size_threshold);

/// NOT in default pipelines. Top-down, parent-centric. At each
/// non-emitted node, if any descendant of any child has been emitted
/// (cascading trigger), rescue clean siblings — children that are
/// themselves untouched and whose subtree has more than `min_size`
/// nodes.
///
/// Known unsoundness: when InsertSubgraphProxies wraps the rescued
/// subtree, the artificial end_proxy → ext_succ edges can reach into
/// sibling subtrees that the cascading-trigger sibling owns,
/// serializing what was previously parallel work and (in the worst
/// case) delaying critical-path nodes by unrelated bubble-filler.
/// See the §5.5 "Known unsoundness" note in
/// AMDGPUSubgraphFormationDesign.md. The pass exists as an opt-in
/// primitive for experimentation; default pipelines omit it.
void SiblingRescuePass(SubgraphFormationTree &tree, int min_size);

// --- Pipelines + BuildSubgraphInfos (§4.4, §4.5, §6 of design doc) -------

/// How to partition the non-splitter members of a single-splitter
/// emit point. See §4.4 of the design doc. Both policies produce
/// the same `ancestors_of_splitter` group (members that reach the
/// splitter in the DAG) and treat the splitter itself as a
/// standalone non-subgraph node; they differ only on how they
/// handle the rest.
enum class SplitterPartitionPolicy {
  /// Default. Everything that doesn't reach the splitter goes into
  /// a single "descendants_or_other" group. Simpler partition; the
  /// resulting bundle's proxy ext_predecessors include the splitter
  /// so the whole bundle gates on it.
  kBundleDescendantsAndIndependents,

  /// Alternate. Split the non-ancestor members into DAG-descendants
  /// of the splitter (those it reaches) and independents (neither
  /// direction). Independents become their own subgraph, free to
  /// schedule without waiting on the splitter.
  kSplitDescendantsAndIndependents,
};

/// Output of SplitEmitPoint. Ancestors / descendants / independents
/// are split according to SplitterPartitionPolicy; empty member
/// vectors are valid and get filtered downstream by
/// BuildSubgraphInfos's size-2 minimum.
struct SplitterSplitResult {
  /// Subtree members that reach the splitter in the DAG. Non-empty
  /// only when the splitter is not the dom-root of the emit point's
  /// subtree.
  SmallVector<ScheduleNode *> ancestors_of_splitter;

  /// Bundle of "everything not in ancestors_of_splitter."
  /// Only populated when policy == kBundleDescendantsAndIndependents.
  SmallVector<ScheduleNode *> descendants_or_other;

  /// Subtree members that the splitter reaches in the DAG.
  /// Only populated when policy == kSplitDescendantsAndIndependents.
  SmallVector<ScheduleNode *> descendants_of_splitter;

  /// Subtree members that neither reach nor are reached by the
  /// splitter. Only populated when policy ==
  /// kSplitDescendantsAndIndependents.
  SmallVector<ScheduleNode *> independents;
};

/// Partition the members of a single-splitter emit point. The
/// caller guarantees `emit_point->subtree_splitter_count == 1`;
/// the splitter is found by walking the emit_point subtree. The
/// splitter itself is left standalone (not a member of any group).
///
/// Reachability is read from the graph's cached matrix
/// (ScheduleGraph::IsReachableInDag), so the graph must have had
/// ComputeTransitiveReduction run since the last mutation.
SplitterSplitResult SplitEmitPoint(SubgraphFormationTreeNode *emit_point,
                                   const ScheduleGraph &graph,
                                   SplitterPartitionPolicy policy);

/// Materialize SubgraphInfos from the pipeline's emit points.
/// Per-emit-point dispatch by splitter count (§3.5):
///   - 0 splitters: emit the subtree's members as one SubgraphInfo.
///   - 1 splitter: split via SplitEmitPoint; emit each non-empty
///     resulting group.
///   - 2+ splitters: emit the subtree's members as one SubgraphInfo
///     as-is (no splitting — the multi-splitter rescue pass already
///     verified the nc > sc bubble-filler invariant).
/// In every case, a per-group ≥ 2 minimum filters out singletons
/// (one-member subgraphs add proxy overhead with no benefit).
std::vector<std::unique_ptr<SubgraphInfo>> BuildSubgraphInfos(
    ArrayRef<SubgraphFormationTreeNode *> emit_points,
    const ScheduleGraph &graph,
    SplitterPartitionPolicy policy);

/// An ordered list of formation passes. Each pass takes the shared
/// SubgraphFormationTree by reference and mutates it via
/// RecordEmission. Pipelines are configuration: different policy
/// choices (bottom-up vs top-down, etc.) become different pipeline
/// constants rather than scattered conditionals.
struct SubgraphFormationPipeline {
  std::vector<std::function<void(SubgraphFormationTree &)>> passes;
};

/// Top-level configuration object. Picks a splitter threshold,
/// rescue thresholds, partition policy, and an ordered pipeline
/// of passes. Two factory methods (BottomUpDefault,
/// TopDownAggressive) build the canonical configurations; callers
/// can mutate the returned struct to tweak.
struct SubgraphFormationPolicy {
  /// Latency threshold for IsSubgraphSplitter. Default 32 cycles
  /// catches memory loads on AMDGPU gfx906.
  int latency_threshold = 32;

  /// Size threshold for LargeSplitterFreeRescuePass.
  int large_subtree_threshold = 24;

  /// Min size for SiblingRescuePass (when used). Subtrees ≤ this
  /// are skipped — too small to be worth wrapping.
  int sibling_rescue_min_size = 1;

  /// Which partition policy BuildSubgraphInfos applies to
  /// single-splitter emit points.
  SplitterPartitionPolicy splitter_partition =
      SplitterPartitionPolicy::kBundleDescendantsAndIndependents;

  /// The pass sequence. Initialized by the factory methods below.
  SubgraphFormationPipeline pipeline;

  /// BottomUpDefault: emit at the lowest single-splitter subtrees,
  /// then multi-splitter rescue, then large-no-splitter rescue.
  /// SiblingRescuePass deliberately omitted (see §5.5).
  static SubgraphFormationPolicy BottomUpDefault();

  /// TopDownAggressive: same as BottomUpDefault but with the
  /// top-down single-splitter pass — emits at the HIGHEST
  /// single-splitter subtree on each branch instead of the lowest.
  /// Produces fewer, larger subgraphs.
  static SubgraphFormationPolicy TopDownAggressive();
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SUBGRAPHFORMATION_H
