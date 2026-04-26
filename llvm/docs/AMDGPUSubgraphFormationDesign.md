# HierarchicalScheduler: Subgraph Formation Design

## Purpose of this Document

The companion doc (`AMDGPUClusteringDesign.md`, Approach B)
specifies how a *given* set of subgraphs is inserted into the
ScheduleGraph and consumed at scheduling time. It does **not**
specify how to choose those subgraphs in the first place. This doc
fills that gap.

Concretely: given a freshly built ScheduleGraph (after
`BuildFromSUnits`), produce a list of `SubgraphInfo *` whose
`members` field is filled in, ready to be handed to
`InsertSubgraphProxies`. The algorithm uses the dominator tree plus
a latency-based "splitter" notion to decide
where to cut the graph into subgraphs.

Downstream assumption (from `AMDGPUClusteringDesign.md` §3.2): the
output is a **flat, two-level** set of subgraphs — members are raw
real ScheduleNodes and members of different subgraphs do not
overlap. Nested subgraphs are out of scope here; they remain future
work.

## Table of Contents

1. [Context and Prerequisites](#1-context-and-prerequisites)
2. [Terminology](#2-terminology)
3. [Architecture Overview](#3-architecture-overview)
4. [Components](#4-components)
5. [Per-Decision-Rule Passes](#5-per-decision-rule-passes)
6. [Pipelines](#6-pipelines)
7. [End-to-End Driver](#7-end-to-end-driver)
8. [Worked Example](#8-worked-example)
9. [Implementation Phases and Test Plan](#9-implementation-phases-and-test-plan)

---

## 1. Context and Prerequisites

### 1.1 What already exists

Each ScheduleGraph (after `BuildFromSUnits` +
`ValidateAndComputeTopologicalOrder` +
`ComputeTransitiveReduction` + `ComputeDominatorTree`) gives us:

- A topologically ordered list of nodes (`GetTopoOrder()`). Single-
  source / single-sink is enforced by
  `ValidateAndComputeTopologicalOrder`, so `GetTopoOrder()[0]` is
  always the unique source.
- A `ReducedGraph` with per-topo-index predecessor and successor
  lists, forming the transitive reduction.
- A `DominatorTree` exposing `GetIDomByTopoIndex(topo_idx)`. We
  also add a small convenience overload `GetIDom(const ScheduleNode *)`
  that does `GetIDomByTopoIndex(node->GetTopoIndex())` internally,
  so callers can pass a node pointer directly. We do not change
  the underlying storage; §4.2 builds a separate
  `SubgraphFormationTree` that reads `idom[]` and adds the per-node
  state formation needs.
- Per-node `IsLatencyEdge()` and `latency_` on every edge, used to
  define "subgraph splitter."

### 1.2 Reachability: where it actually lives today

Reachability is **not** computed as part of IDOM construction —
`DominatorTree::Build` only reads the reduced adjacency arrays; it
never materializes a reachability matrix. It is computed inside
`ScheduleGraph::ComputeTransitiveReduction`, where a local
`std::vector<BitVector> reachable(num_nodes, ...)` is built up to
decide which edges are redundant — and then **discarded** when the
function returns.

The plan in §4.3 is to keep this matrix instead of throwing it
away; splitting needs it on every member of every emit point that has a splitter.

### 1.3 What this doc adds

- `IsSubgraphSplitter(node, latency_threshold)` helper (§4.1).
- A `SubgraphFormationTree` data structure: pointer-based tree
  built from the dominator tree, with per-node static counts and
  mutable emission-tracking state (§4.2).
- A reachability cache on ScheduleGraph (§4.3).
- A small set of **passes**, one per decision rule, sharing the
  `SubgraphFormationTree` and a single `RecordEmission` mutator
  (§5).
- **Pipelines** that compose passes in a chosen order (§6).
- A splitting step that takes a single-splitter emit point and
  partitions its members, governed by a
  `SplitterPartitionPolicy` enum (§4.4).
- An end-to-end `FormSubgraphs` driver that ties these together
  and hands results to `InsertSubgraphProxies` (§7).

---

## 2. Terminology

- **Subgraph splitter** (or just **splitter**) — a node whose
  maximum outgoing latency-edge exceeds a threshold. Such a node
  forces a multi-cycle bubble after it, breaking what would
  otherwise be a clean subgraph. On AMDGPU gfx906 with the
  default threshold of 32 cycles, this is effectively "any memory
  load."
- **Emit point** — a dom-tree node whose entire subtree has been
  marked by some pass as a single subgraph candidate. The set of
  emit points after all passes have run determines the final
  list of `SubgraphInfo`s.
- **Single-splitter emit point** — emit point whose subtree
  contains exactly one splitter. Gets split downstream
  into ancestors-of-splitter vs descendants/independents.
- **Multi-splitter emit point** — emit point whose subtree
  contains two or more splitters. Emitted as one `SubgraphInfo`
  as-is (no splitting). Content inside the subtree fills splitter
  bubbles; the emission rule's `#nodes > #splitters` guard
  guarantees there's enough non-splitter material to do so.
- **Splitter-free emit point** — emit point whose subtree
  contains zero splitters. Emitted as one `SubgraphInfo` as-is.
- **Splitting** — operation applied to single-splitter emit
  points only: partitions members relative to the splitter and
  emits each non-trivial group as its own `SubgraphInfo`.
- **Valid subgraph** — any emitted `SubgraphInfo` with ≥ 2
  members. Smaller member lists are suppressed (one-member
  subgraphs add proxy overhead with no scheduling benefit).

---

## 3. Architecture Overview

### 3.1 Pipeline position

```
BuildFromSUnits
    │
    ▼
ValidateAndComputeTopologicalOrder
    │
    ▼
ComputeTransitiveReduction        ─── now also caches reachability
    │
    ▼
ComputeDominatorTree
    │
    ▼
SubgraphFormationTree::Build      ─── NEW: pointer-tree + count splitters/nodes
    │
    ▼
Pipeline of passes                ─── NEW: each pass marks emit points
    │
    ▼
BuildSubgraphInfos                ─── NEW: per emit point → SubgraphInfo(s)
    │                                  (single-splitter emit points are
    │                                   split here)
    ▼ (vector<unique_ptr<SubgraphInfo>>)
    │
InsertSubgraphProxies             ─── AMDGPUClusteringDesign.md §3.2
    │
    ▼
ValidateAndComputeTopologicalOrder  (re-run — graph mutated)
ComputeCriticalPathFromExit         (re-run)
    │
    ▼
Scheduling
```

Where exactly the formation step slots in (every pass that builds
a graph, or only specific passes) is deferred to integration time
— the right answer depends on which scheduling passes end up
wanting subgraphs. The formation function is pure with respect to
graph state, so it can be invoked or skipped at the pass level
independently.

### 3.2 Module layout

The formation code starts in two new files and may split further
as it grows:

- `HierarchicalScheduler/SubgraphFormation.h` — public API:
  `SubgraphFormationTree`, the pass functions, `FormationPipeline`,
  `SubgraphFormationPolicy`, `FormSubgraphs`.
- `HierarchicalScheduler/SubgraphFormation.cpp` — implementation.

Likely splits if the file grows: per-pass files
(`SubgraphFormationPasses/*.cpp`), or separating the tree type
from the pipeline machinery. Defer that until measured volume
justifies it.

Extended files:

- `HierarchicalScheduler/ScheduleGraph.{h,cpp}` — preserve the
  reachability matrix that `ComputeTransitiveReduction` already
  builds; expose `IsReachableInDag(from_topo_idx, to_topo_idx)`;
  clear in `InvalidateDerivedData`.

Not touched by this doc:

- `InsertSubgraphProxies` and `SubgraphInfo`: defined in
  `AMDGPUClusteringDesign.md`; subgraph formation produces inputs for
  them but does not reshape them.

### 3.3 Architectural principles

Three principles drive the structure of §4–§7.

**One pass per decision rule.** Each rule for "this tree node
should become an emit point" is its own pass function. Passes
share the underlying tree and a single mutator (`RecordEmission`),
nothing else. Reading any one pass tells you exactly what its
rule does; no flags or shared business logic to chase.

**Pointer-based shared tree.** The `SubgraphFormationTree` (§4.2)
is a pointer-based tree: each node is a struct, parent and
children are `Node *` fields, all per-node state — both static
counts and mutable emission flags — lives on the node. Walking
and mutating read naturally; no parallel-array indexing.

**Pipelines as configuration.** A `FormationPipeline` is just an
ordered list of passes (§6). The "what algorithm are we running"
question is one struct field, not scattered conditionals. Different
policy choices (bottom-up vs top-down for single-splitter subgraphs,
sibling rescue on/off, etc.) become different pipeline constants.

### 3.4 Tree backbone is pluggable (idom today, ipdom later)

The `SubgraphFormationTree` is initially built from the dominator
tree (`SubgraphFormationTree::BuildFromDominatorTree`). Nothing
about the per-node counts or pass machinery depends on that
choice — they're tree-shape-agnostic. A future
`BuildFromPostDominatorTree` factory would produce a parallel
formation tree built off the immediate post-dominator tree, and
all the existing passes would run on it unchanged. The naming
deliberately avoids "Dom" / "PostDom" so we don't commit either
way; the formation tree is just "the tree we're forming from."

Where this matters: the dom-tree perspective groups nodes that
share a common predecessor (good for "things downstream of a
load"); the post-dom tree groups nodes that share a common
successor (good for "things feeding into a store"). Different
backbones identify different cohesion patterns. Whether we want
one, the other, or both is deferred until the dom-based version
is measured.

### 3.5 Where splitting lives

Splitting is **not a pass** — it runs in
`BuildSubgraphInfos` after the pipeline finishes. Each emit point's
splitter count determines what happens:

- **0 splitters** (splitter-free): the subtree's members go into one
  `SubgraphInfo`.
- **1 splitter** (single-splitter): split into multiple
  `SubgraphInfo`s — ancestors-of-splitter, and one or two groups
  for the non-ancestor side per `SplitterPartitionPolicy`. The
  splitter itself is left standalone (not a member). See §4.4.
- **2+ splitters** (multi-splitter): the subtree's members go into
  one `SubgraphInfo` as-is. No splitting. The pass that emitted
  this point already required `#nodes > #splitters`, so there's
  bubble-filler material inside the scope.

This shape — passes mark emit points, post-processing turns each
emit point into one or more `SubgraphInfo`s — keeps the passes
themselves pure tree mutations. The "one emit point can become
multiple subgraphs" complication lives in one place.

---

## 4. Components

### 4.1 Subgraph splitter identification

A ScheduleNode is a subgraph splitter iff some outgoing latency-
contributing edge has latency exceeding a policy threshold. Default
threshold: **32 cycles**. On AMDGPU gfx906 this catches memory
loads (latencies typically ~80) and excludes ALU (latencies 1–5);
in practice no non-load instruction has outgoing latency above 32.

Entry and exit nodes are not splitters by construction — entry's
edges to roots have latency 0 (set by `CreateEntryAndExitNodes`)
and exit has no outgoing edges. No explicit guard needed in the
predicate.

```cpp
// Free function: the predicate is policy and shouldn't live on
// ScheduleNode. Default latency_threshold = 32.
bool IsSubgraphSplitter(const ScheduleNode *node, int latency_threshold) {
  for (const ScheduleEdge &edge : node->Successors()) {
    if (!edge.IsLatencyEdge()) {
      continue;  // skip weak / subgraph-order edges
    }
    if (edge.latency_ > latency_threshold) {
      return true;
    }
  }
  return false;
}
```

The threshold is a parameter to `FormSubgraphs` and threads through
to `IsSubgraphSplitter`. No enum-based SplitterPolicy yet — adding one
is cheap if we ever need a non-latency predicate (e.g., a direct
`mayLoad()` check).

Caveat: the latency proxy will also flag any non-load instruction
with outgoing latency > 32. On gfx906 this is effectively no real
instruction; if the formation pass is later compiled for a target
where that's not true, the predicate may need refinement.

### 4.2 SubgraphFormationTree

The formation tree is a pointer-based tree built from (currently)
the dominator tree. Each node owns its static counts and its
mutable emission state directly; passes navigate by pointer.

```cpp
struct SubgraphFormationTreeNode {
  // Identity
  int topo_idx;                       // index into ScheduleGraph's topo order
  ScheduleNode *schedule_node;        // backpointer for member assembly

  // Tree structure
  SubgraphFormationTreeNode *parent;  // null at the root
  SmallVector<SubgraphFormationTreeNode *> children;

  // Static — populated by Build, never changed afterward.
  bool is_splitter;                   // is_splitter(schedule_node) — cached
  int subtree_splitter_count;         // includes self if is_splitter
  int subtree_node_count;             // includes self

  // DFS pre/post numbers, assigned during Build. Enables O(1)
  // subtree-membership queries via SubgraphFormationTree::
  // IsInSubtreeOf — see below. Useful for any "is X dominated
  // by Y in this tree?" question; the planned safer
  // sibling-rescue (§5.5) is one prospective consumer.
  int dfs_pre;
  int dfs_post;

  // Mutable — set by SubgraphFormationTree::RecordEmission and
  // read by passes through the descendant-emit guard.
  bool emitted = false;
  bool descendants_emitted = false;
};

class SubgraphFormationTree {
 public:
  // Currently the only build path; a BuildFromPostDominatorTree
  // is anticipated as a parallel factory (see §3.4) and would
  // produce a tree usable by the same passes unchanged.
  static SubgraphFormationTree BuildFromDominatorTree(
      const ScheduleGraph &graph,
      const DominatorTree &dom,
      llvm::function_ref<bool(const ScheduleNode *)> is_splitter);

  SubgraphFormationTreeNode *Root();

  // Shared mutator. Marks node as an emit point, walks the parent
  // chain setting descendants_emitted = true on every ancestor.
  // Idempotent — calling on an already-emitted node is a no-op.
  // Every pass that wants to mark an emit point goes through this.
  void RecordEmission(SubgraphFormationTreeNode *node);

  // After all passes have run, return the set of nodes with
  // emitted == true. Each becomes one or more SubgraphInfos in
  // BuildSubgraphInfos (see §3.5 for the dispatch).
  std::vector<SubgraphFormationTreeNode *> CollectEmitPoints() const;

  // True iff `descendant` lies in `ancestor`'s subtree (inclusive
  // of ancestor itself). O(1) via the dfs_pre/dfs_post numbers
  // populated at Build time. See §4.2 Build details.
  static bool IsInSubtreeOf(const SubgraphFormationTreeNode *descendant,
                            const SubgraphFormationTreeNode *ancestor) {
    return ancestor->dfs_pre <= descendant->dfs_pre &&
           descendant->dfs_post <= ancestor->dfs_post;
  }

 private:
  // Reserved up front to graph.Size() so node pointers stay
  // stable for the tree's whole lifetime.
  std::vector<SubgraphFormationTreeNode> nodes_;
};
```

**Shared traversal helpers.** Three of the five passes (§5) want
post-order; the other two are top-down with subtree-skip
semantics. Two helper templates capture the patterns once, so
each pass — and Build's own count step — can express its body as
a one-line lambda.

The default helpers are recursive (3 lines each, trivially
correct, matches the mental model of the traversal). Recursion
fits the Linux 8 MB default stack with comfortable margin even
for instruction-level dom-tree depths approaching 20k. Iterative
variants are kept alongside, named `*Iterative`, for the cases
where recursion isn't safe (tighter stack budgets — Windows
1 MB default, threaded contexts with shrunk per-thread stacks —
or for true pathological depths beyond what we currently expect).

```cpp
// Post-order — recursive. Used by passes whose rule decides
// "what to do with each node after its descendants are processed."
template <typename Fn>
void PostOrderApply(SubgraphFormationTreeNode *n, Fn fn) {
  for (auto *c : n->children) PostOrderApply(c, fn);
  fn(n);
}

// Post-order — iterative (heap-backed stack). Use when the
// caller can't afford O(depth) system stack, e.g. a tighter
// per-thread stack budget.
template <typename Fn>
void PostOrderApplyIterative(SubgraphFormationTreeNode *root, Fn fn) {
  // Stack frames hold (node, next_child_index_to_descend_into).
  SmallVector<std::pair<SubgraphFormationTreeNode *, int>, 32> stack;
  stack.push_back({root, 0});
  while (!stack.empty()) {
    auto &[n, idx] = stack.back();
    if (idx < (int)n->children.size()) {
      stack.push_back({n->children[idx++], 0});
    } else {
      fn(n);
      stack.pop_back();
    }
  }
}

// Pre-order — recursive. No skip semantic; visits every node.
template <typename Fn>
void PreOrderApply(SubgraphFormationTreeNode *n, Fn fn) {
  fn(n);
  for (auto *c : n->children) PreOrderApply(c, fn);
}

// Pre-order — iterative.
template <typename Fn>
void PreOrderApplyIterative(SubgraphFormationTreeNode *root, Fn fn) {
  SmallVector<SubgraphFormationTreeNode *, 32> stack = {root};
  while (!stack.empty()) {
    auto *n = stack.pop_back_val();
    fn(n);
    for (auto *c : n->children) stack.push_back(c);
  }
}

// Top-down with subtree-skip — recursive. The lambda returns
// true to mean "I emitted at this node; skip its subtree."
template <typename Fn>
void PreOrderApplyWithSkip(SubgraphFormationTreeNode *n, Fn fn) {
  if (fn(n)) return;
  for (auto *c : n->children) PreOrderApplyWithSkip(c, fn);
}

// Top-down with subtree-skip — iterative. Same contract.
// Use under the same conditions as PostOrderApplyIterative.
template <typename Fn>
void PreOrderApplyWithSkipIterative(SubgraphFormationTreeNode *root, Fn fn) {
  SmallVector<SubgraphFormationTreeNode *, 32> stack = {root};
  while (!stack.empty()) {
    auto *n = stack.pop_back_val();
    if (fn(n)) continue;
    for (auto *c : n->children) stack.push_back(c);
  }
}
```

Default callers use the recursive forms. The iterative variants
are not unused-and-deletable — they exist as the ready
substitute if we ever need them, and as documentation that the
choice was deliberate.

**Build details.** Three passes, each using one of the shared
traversal helpers:

1. **Allocation + parent/child wiring.** One node per
   ScheduleGraph node, in `nodes_` reserved to `graph.Size()` so
   pointers stay stable. From the dominator tree, set
   `idom[topo_idx] → parent`; populate children lists from the
   resulting parent links.

2. **Subtree counts** via `PostOrderApply` from the root:
   ```cpp
   PostOrderApply(root, [&](SubgraphFormationTreeNode *n) {
     n->subtree_splitter_count = is_splitter(n->schedule_node) ? 1 : 0;
     n->subtree_node_count   = 1;
     for (auto *c : n->children) {
       n->subtree_splitter_count += c->subtree_splitter_count;
       n->subtree_node_count   += c->subtree_node_count;
     }
   });
   ```

3. **DFS pre/post numbering** — one `PreOrderApply` for `dfs_pre`,
   one `PostOrderApply` for `dfs_post`. Each uses its own
   counter:
   ```cpp
   int pre = 0;
   PreOrderApply(root, [&](SubgraphFormationTreeNode *n) {
     n->dfs_pre = pre++;
   });
   int post = 0;
   PostOrderApply(root, [&](SubgraphFormationTreeNode *n) {
     n->dfs_post = post++;
   });
   ```
   With both numbers populated, `IsInSubtreeOf(descendant,
   ancestor)` is the O(1) test
   `ancestor->dfs_pre <= descendant->dfs_pre && descendant->dfs_post <= ancestor->dfs_post`.
   (Two separate counters give that particular form. A shared
   single counter would yield `pre <= pre <= post`. Both work;
   two-counter is cleaner with the existing helpers.)

After Build, the tree is ready for any pass to consume.

**On the existing `DominatorTree` class.** This doc does not
change `DominatorTree`'s storage. The `BuildFromDominatorTree`
factory reads `idom[]` through the existing public API and builds
the formation tree as a separate structure. Keeping the formation
tree separate means: (a) its mutable emission state doesn't
contaminate the dom tree's read-only role, (b) a future
`BuildFromPostDominatorTree` produces a parallel formation tree
without entangling with the dom tree's representation.

### 4.3 Reachability cache

Splitting needs, for a splitter `C` and every other member
`X` of the emit point that contains `C`, "does `X` reach `C` in the DAG?"
(and, for the alternate partition policy, "does `C` reach `X`?").
These bits are exactly `reachable[X][C]` and `reachable[C][X]` —
already computed and then discarded inside
`ComputeTransitiveReduction`.

Instead of discarding, move it into a member:

```cpp
class ScheduleGraph {
  // ... existing members ...

  // Populated at the end of ComputeTransitiveReduction (instead of
  // being discarded). Cleared by InvalidateDerivedData. Outer size
  // == graph size; each inner BitVector has graph-size bits.
  std::vector<BitVector> reachability_by_topo_index_;

  bool IsReachableInDag(int from_topo_idx, int to_topo_idx) const {
    return reachability_by_topo_index_[from_topo_idx]
        .test(to_topo_idx);
  }
};
```

Storage cost: O(V²/8) bytes per graph. At V=1000 that's ~125 KB;
at V=2000, ~500 KB. One graph in flight per region; this is not
the allocation that pressures memory.

### 4.4 Splitting

Applies only to single-splitter emit points (those whose
`subtree_splitter_count == 1`). Multi-splitter and splitter-free emit
points skip splitting entirely (see §3.5 and §4.5 BuildSubgraphInfos).

For a single-splitter emit point with splitter `C`, partition the
subtree's other members based on their reachability relation to
`C`. Two policies, behind a `SplitterPartitionPolicy` enum:

```cpp
enum class SplitterPartitionPolicy {
  kBundleDescendantsAndIndependents,   // default
  kSplitDescendantsAndIndependents,    // alternate
};
```

Both policies produce the same first group (members that reach
`C` in the DAG) and treat `C` itself as a standalone non-subgraph
node. They differ only in how they handle the rest:

- **`kBundleDescendantsAndIndependents`** (default): everything
  not in the first group goes into a single second group. Simpler
  partition; the resulting bundle's proxy `ext_predecessors`
  include `C`, so the whole bundle gates on `C` being scheduled.
- **`kSplitDescendantsAndIndependents`** (alternate): split the
  rest into two — DAG-descendants of `C` (those `C` reaches) and
  independents (neither direction). Independents become their own
  subgraph, free to schedule without waiting for `C`.

Both policies are first-class; the alternate is not a "fix" for
the default. Bundling can produce larger, structurally cohesive
groups; splitting frees the independent set but produces an
"independents" subgraph whose members may have no scheduling
affinity. Pick one to start, measure, iterate. Both read the same
cached reachability matrix; the only difference is which bits are
queried.

```cpp
struct SplitterSplitResult {
  // Empty member vectors are valid — they get filtered by the
  // size-2 minimum in BuildSubgraphInfos.
  SmallVector<ScheduleNode *> ancestors_of_splitter;
  SmallVector<ScheduleNode *> descendants_or_other;    // policy A only
  SmallVector<ScheduleNode *> descendants_of_splitter;  // policy B only
  SmallVector<ScheduleNode *> independents;            // policy B only
};

// `emit_point` is a tree node whose subtree contains exactly one
// splitter. Walks the subtree, finds the splitter, and partitions the
// other members.
SplitterSplitResult SplitEmitPoint(
    SubgraphFormationTreeNode *emit_point,
    const ScheduleGraph &graph,
    SplitterPartitionPolicy policy) {
  ScheduleNode *splitter = nullptr;
  SmallVector<ScheduleNode *> members;
  PostOrderApply(emit_point, [&](SubgraphFormationTreeNode *n) {
    members.push_back(n->schedule_node);
    if (n->is_splitter) splitter = n->schedule_node;
  });
  // emit_point invariant guarantees splitter != nullptr.

  SplitterSplitResult r;
  int c_idx = splitter->GetTopoIndex();
  for (ScheduleNode *m : members) {
    if (m == splitter) continue;       // the splitter — standalone, not a subgraph
    int m_idx = m->GetTopoIndex();
    bool reaches_c = graph.IsReachableInDag(m_idx, c_idx);
    if (reaches_c) {
      r.ancestors_of_splitter.push_back(m);
      continue;
    }
    if (policy == SplitterPartitionPolicy::kBundleDescendantsAndIndependents) {
      r.descendants_or_other.push_back(m);
    } else if (graph.IsReachableInDag(c_idx, m_idx)) {
      r.descendants_of_splitter.push_back(m);
    } else {
      r.independents.push_back(m);
    }
  }
  return r;
}
```

(Note: `is_splitter` is a per-node `bool` on `SubgraphFormationTreeNode`,
populated at Build time from the same predicate that drives
`subtree_splitter_count`.)

**Edge cases.**

- If `C` is the root of the emit point's subtree, no other member
  reaches it (they're all DAG descendants of `C`). The
  `ancestors_of_splitter` group is empty; only the post-splitter
  side(s) become subgraph candidates.
- If `C` is a dom leaf, the post-splitter side may be empty. Only
  separate dom-branches in the subtree contribute.
- If the subtree has only `C` as a real node, all groups are
  empty and the result is nothing — just the standalone splitter.

### 4.5 BuildSubgraphInfos

For each emit point produced by the pipeline (§5–§6),
materialize one or more `SubgraphInfo`s. Three cases by splitter
count, dispatched here:

- **0 splitters** (splitter-free): emit the subtree's members as
  one `SubgraphInfo`.
- **1 splitter** (single-splitter): split via §4.4, emit
  each non-empty resulting group.
- **2+ splitters** (multi-splitter): emit the subtree's members
  as one `SubgraphInfo` as-is, no splitting.

In every case, the per-group singleton filter (≥ 2 members)
applies — a one-member "subgraph" adds proxy overhead with no
scheduling benefit.

```cpp
std::vector<std::unique_ptr<SubgraphInfo>> BuildSubgraphInfos(
    const std::vector<SubgraphFormationTreeNode *> &emit_points,
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
      // 0 or 2+ splitters — emit the whole subtree as one SubgraphInfo.
      SmallVector<ScheduleNode *> members;
      PostOrderApply(ep, [&](SubgraphFormationTreeNode *n) {
        members.push_back(n->schedule_node);
      });
      EmitIfLargeEnough(members, next_id, out);
    }
  }
  return out;
}

void EmitIfLargeEnough(
    SmallVectorImpl<ScheduleNode *> &members,
    int &next_id,
    std::vector<std::unique_ptr<SubgraphInfo>> &out) {
  if (members.size() < 2) return;
  std::string name = "subgraph_" + std::to_string(next_id++);
  out.push_back(std::make_unique<SubgraphInfo>(members, name));
}
```

`BuildSubgraphInfos` returns owned `SubgraphInfo`s as
`vector<unique_ptr<SubgraphInfo>>`. The driver hands this vector
to `InsertSubgraphProxies` (per `AMDGPUClusteringDesign.md` §3.2),
which takes the vector **by value** and moves each `unique_ptr`
into the corresponding start proxy node. Ownership transfers from
the driver into the graph at insertion time; after the move, the
driver's vector is empty.

---

## 5. Per-Decision-Rule Passes

Each pass is a free function that takes `SubgraphFormationTree &`
and mutates it via `RecordEmission`. Reading any one pass tells
you exactly what its rule does — no flags, no shared business
logic. **Every pass is guarded against double-emission and
nesting**: a node is never emitted if it's already emitted
itself, and (with one nuance for top-down passes — see §5.2) a
node isn't emitted if any of its descendants has been emitted.
The guards are uniform so that pipelines can chain passes in any
order without nesting bugs.

### 5.1 BottomUpSingleSplitterPass

Emit at the **lowest** dom-subtree on each branch where the
subtree has exactly one splitter and more than one node.

```cpp
void BottomUpSingleSplitterPass(SubgraphFormationTree &tree) {
  PostOrderApply(tree.Root(), [&](SubgraphFormationTreeNode *n) {
    if (n->emitted || n->descendants_emitted) return;
    if (n->subtree_splitter_count == 1 && n->subtree_node_count > 1) {
      tree.RecordEmission(n);
    }
  });
}
```

"Lowest" emerges from post-order + the descendant-emit guard:
the deepest qualifying node on each branch wins; once it emits,
ancestors see `descendants_emitted == true` and skip.

### 5.2 TopDownSingleSplitterPass

Emit at the **highest** dom-subtree on each branch where the
subtree has exactly one splitter and more than one node.

```cpp
void TopDownSingleSplitterPass(SubgraphFormationTree &tree) {
  PreOrderApplyWithSkip(tree.Root(), [&](SubgraphFormationTreeNode *n) {
    if (n->emitted) return true;              // wholly consumed; skip descent
    if (n->descendants_emitted) return false; // can't emit here (would nest),
                                              // but a separate clean child branch
                                              // could still qualify — keep descending
    if (n->subtree_splitter_count == 1 && n->subtree_node_count > 1) {
      tree.RecordEmission(n);
      return true;                            // skip descent — subtree consumed
    }
    return false;
  });
}
```

Three guard branches, each behaviorally distinct:

- `emitted`: this subtree is already a subgraph from a prior
  pass. Skip-descend (no work to do, members are wrapped).
- `descendants_emitted`: some descendant emitted; can't emit
  *here* without nesting, but an unrelated child branch could
  still satisfy the rule independently — so descend, and the
  emitted descendants get caught by the `emitted` branch when
  visited.
- Neither: evaluate the rule normally; on emit, skip descent.

### 5.3 MultiSplitterRescuePass

Bottom-up. Emit where the subtree has more than one splitter
AND there are more nodes than splitters (so there's
non-splitter material inside to fill bubbles).

```cpp
void MultiSplitterRescuePass(SubgraphFormationTree &tree) {
  PostOrderApply(tree.Root(), [&](SubgraphFormationTreeNode *n) {
    if (n->emitted || n->descendants_emitted) return;
    if (n->subtree_splitter_count > 1 &&
        n->subtree_node_count > n->subtree_splitter_count) {
      tree.RecordEmission(n);
    }
  });
}
```

The `nc > sc` guard ensures the resulting (multi-splitter)
subgraph has bubble-filler material inside its scope.

### 5.4 LargeSplitterFreeRescuePass

Bottom-up. Emit where the subtree has zero splitters and more
than `size_threshold` nodes. The threshold is policy (default 24)
— a guess to be tuned with measurement.

```cpp
void LargeSplitterFreeRescuePass(SubgraphFormationTree &tree, int size_threshold) {
  PostOrderApply(tree.Root(), [&](SubgraphFormationTreeNode *n) {
    if (n->emitted || n->descendants_emitted) return;
    if (n->subtree_splitter_count == 0 && n->subtree_node_count > size_threshold) {
      tree.RecordEmission(n);
    }
  });
}
```

### 5.5 SiblingRescuePass — defined but NOT in default pipelines

> **Warning**: this pass can delay critical-path execution by
> reaching into sibling subtrees. Read the "Known unsoundness"
> note below before enabling. Not included in
> `BottomUpDefault` or `TopDownAggressive`.

Top-down, parent-centric. At each non-emitted node, if any
descendant of any child has been emitted (cascading trigger),
rescue clean siblings — children that are themselves untouched
and large enough to be worth wrapping.

```cpp
void SiblingRescuePass(SubgraphFormationTree &tree, int min_size) {
  PreOrderApplyWithSkip(tree.Root(), [&](SubgraphFormationTreeNode *n) {
    if (n->emitted) return true;     // whole subtree consumed; skip descent

    bool trigger = false;
    for (auto *c : n->children) {
      if (c->emitted || c->descendants_emitted) { trigger = true; break; }
    }
    if (trigger) {
      for (auto *c : n->children) {
        if (c->emitted || c->descendants_emitted) continue;  // not clean
        if (c->subtree_node_count <= min_size) continue;     // too small
        tree.RecordEmission(c);
      }
    }
    return false;   // continue descending into non-emitted children
  });
}
```

Two guards on rescue candidates:

- **Clean-subtree** (`!emitted && !descendants_emitted`):
  rescuing a subtree that already has emissions inside would
  create overlap — the rescue subgraph and the inner subgraph
  would share members.
- **Min-size** (`subtree_node_count > min_size`): a 1-node
  rescue produces a near-trivial subgraph whose proxy overhead
  outweighs the scheduling benefit.

The cascading trigger (any descendant emit, not just immediate-
child emit) is intentional. It captures the case where cohesion
was identified deep in one branch and we want to give sibling
branches the chance to be wrapped too.

**Known unsoundness — why this isn't in the default pipelines.**
When `InsertSubgraphProxies` wraps the rescued subtree, it adds
an `end_proxy → ext_succ` artificial edge for every external
successor of any member. Each such artificial edge gates
`ext_succ` on every member of the rescued subgraph. If any
ext_successor lives **inside the dom-subtree of one of the
candidate's siblings** — including the sibling whose emission
triggered the rescue — we've reached into that sibling's
territory and added a "wait for the rescued members" constraint
on a node we don't own. In the case where the sibling's
subtree contains a splitter, that constraint is exactly the
"delay the splitter by an unrelated bubble-filler" scenario.
Even when no splitter is involved, the constraint serializes
work that was previously parallel.

The §8 worked example shows this concretely: rescuing `{P, P2}`
adds an artificial edge into `S` (which lives in sibling `S`'s
already-emitted subtree). `S` is the splitter, so its
downstream chain gets delayed by `P2`. Same story for `{Q, Q2}`.

**Safety check we'd want.** Don't rescue candidate `c` if any
of `c`'s ext_successors lives inside a sibling's dom-subtree.
This is one reachability/dom-membership query per ext_successor
per candidate, using the §4.3 reachability matrix and the
formation tree's parent links. Worth implementing if
measurement on real workloads shows missed cohesion that
sibling rescue would have caught — i.e., schedules where
dom-cohesive sibling subtrees end up split apart and we believe
wrapping them would have helped. Until then, the pass sits
here as an opt-in primitive for experimentation; the default
pipelines omit it.

---

## 6. Pipelines

A `SubgraphFormationPipeline` is just an ordered list of passes:

```cpp
struct SubgraphFormationPipeline {
  std::vector<std::function<void(SubgraphFormationTree &)>> passes;
};
```

The policy struct picks a pipeline:

```cpp
struct SubgraphFormationPolicy {
  int latency_threshold = 32;             // splitter cutoff
  int large_subtree_threshold = 24;       // for LargeSplitterFreeRescuePass
  int sibling_rescue_min_size = 1;        // for SiblingRescuePass
  SplitterPartitionPolicy splitter_partition =
      SplitterPartitionPolicy::kBundleDescendantsAndIndependents;
  SubgraphFormationPipeline pipeline;     // initialized by builder, see below

  static SubgraphFormationPolicy BottomUpDefault();
  static SubgraphFormationPolicy TopDownAggressive();
};
```

Each pass is independently bottom-up or top-down — that's a
per-pass property, not a pipeline property. Pipelines just
compose passes in some order. The descendant-emit guards make
the composition order-correct: any pass observes whatever any
prior pass did, regardless of walk direction.

### 6.1 BottomUpDefault

Lowest single-splitter, then multi-splitter rescue, then
large-no-splitter rescue. `SiblingRescuePass` is omitted — see
§5.5 for why.

```cpp
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
```

### 6.2 TopDownAggressive

Same as BottomUpDefault, except the single-splitter pass is the
top-down variant — emit at the **highest** single-splitter
subtree on each branch instead of the lowest. Produces fewer,
larger subgraphs. The other passes are unchanged.

```cpp
SubgraphFormationPolicy SubgraphFormationPolicy::TopDownAggressive() {
  SubgraphFormationPolicy p;
  int t = p.large_subtree_threshold;
  p.pipeline.passes = {
    TopDownSingleSplitterPass,        // <-- only difference vs BottomUpDefault
    MultiSplitterRescuePass,
    [t](SubgraphFormationTree &tree) {
      LargeSplitterFreeRescuePass(tree, t);
    },
    // SiblingRescuePass deliberately omitted — see §5.5.
  };
  return p;
}
```

### 6.3 Why this composes correctly

Every pass guards on `emitted` AND `descendants_emitted` (with
the top-down nuance from §5.2 for the latter). Once any pass
calls `RecordEmission`, every later pass sees the updated state
and respects it. The pipelines above work because of this
guarantee — not because of any direction-coupling between
passes.

---

## 7. End-to-End Driver

```cpp
// Top-level entry point for subgraph formation. Runs on a
// freshly built ScheduleGraph (after BuildFromSUnits). Mutates
// the graph via InsertSubgraphProxies at the end.
//
// Returns nothing: ownership of every emitted SubgraphInfo
// transfers into the graph (specifically into the start proxy
// node of each subgraph) when InsertSubgraphProxies takes the
// vector by value and moves each unique_ptr into its proxy.
void FormSubgraphs(ScheduleGraph &graph,
                   const SubgraphFormationPolicy &policy) {
  // 1. Prerequisites. All idempotent; safe to re-call.
  graph.ValidateAndComputeTopologicalOrder();
  graph.ComputeTransitiveReduction();    // also caches reachability
  graph.ComputeDominatorTree();

  // 2. Build the formation tree.
  auto is_splitter = [&](const ScheduleNode *n) {
    return IsSubgraphSplitter(n, policy.latency_threshold);
  };
  SubgraphFormationTree tree =
      SubgraphFormationTree::BuildFromDominatorTree(
          graph, graph.GetDominatorTree(), is_splitter);

  // 3. Run the configured pipeline.
  for (auto &pass : policy.pipeline.passes) {
    pass(tree);
  }

  // 4. Materialize SubgraphInfos from emit points. Single-
  //    splitter ones get split here; multi/zero-splitter emit
  //    as-is. Singletons are filtered.
  std::vector<std::unique_ptr<SubgraphInfo>> infos =
      BuildSubgraphInfos(tree.CollectEmitPoints(), graph,
                         policy.splitter_partition);

  // 5. Hand ownership to the graph (per
  //    AMDGPUClusteringDesign.md §3.2: takes vector by value
  //    and moves each unique_ptr into its start proxy node).
  graph.InsertSubgraphProxies(std::move(infos));

  // 6. Re-derive what downstream needs. The graph has been
  //    mutated; topo and critical-path are invalidated.
  graph.ValidateAndComputeTopologicalOrder();
  graph.ComputeCriticalPathFromExit();
}
```

---

## 8. Worked Example

10-node DAG designed so the single-splitter split produces
non-trivial groups in both partition policies. `S` is the
single splitter (latency 50 outgoing).

```
              A (entry)
             / \
            P   Q
           /|   |\
          P2 \ / Q2
              S (splitter, S→D latency 50)
             /|\
            D E F
            \|/
            ...     (D, E, F all → F's slot in dom tree)
```

Edges (latencies all 1 except `S→D=50`):
- `A→P`, `A→Q`
- `P→P2`, `P→S`
- `Q→Q2`, `Q→S`
- `S→D` (latency 50), `S→E`
- `D→F`, `E→F`
- `P2→Exit`, `Q2→Exit`, `F→Exit`

### Dominator tree

- `idom(A) = -1` (root).
- `idom(P) = A`, `idom(Q) = A`.
- `idom(P2) = P`, `idom(Q2) = Q`.
- `idom(S) = A` (P and Q both reach S, common dom = A).
- `idom(D) = S`, `idom(E) = S`, `idom(F) = S` (D, E both
  reach F, common dom = S).
- `idom(Exit) = A` (P2, Q2, F all reach Exit, common dom = A).

```
        A
       /|\\\
      P Q S Exit
      | |/|\
     P2 Q2 D E F
```

A's children = {P, Q, S, Exit}. S's children = {D, E, F}.

### SubgraphFormationTree after Build

| node | is_splitter | subtree_splitter_count | subtree_node_count |
|------|-------------|------------------------|--------------------|
| Exit, P2, Q2, D, E, F | false | 0 | 1 |
| P, Q | false | 0 | 2 |
| S    | true  | 1 | 4 |
| A    | false | 1 | 10 |

### Pipeline = BottomUpDefault

**BottomUpSingleSplitterPass** (post-order):
- Leaves (Exit, P2, Q2, D, E, F): `splitter_count == 0`, skip.
- P, Q: `splitter_count == 0`, skip.
- S: `splitter_count == 1`, `node_count == 4 > 1`, no
  descendants_emitted → **emit S**.
- A: `descendants_emitted == true`, skip.

After this pass: emit points = {S}.

**MultiSplitterRescuePass**: nothing has `splitter_count > 1`,
no emissions.

**LargeSplitterFreeRescuePass(24)**: max zero-splitter subtree
is 2 nodes (P, Q), nowhere near 24, no emissions.

(SiblingRescuePass is not in this pipeline.)

After all passes: emit points = {S}.

### BuildSubgraphInfos (BottomUpDefault, default
`SplitterPartitionPolicy::kBundleDescendantsAndIndependents`)

- **S** (`subtree_splitter_count == 1`, single-splitter case
  → split via §4.4):
  - Walk subtree `{S, D, E, F}`.
  - For D, E, F: do they reach S? No (S is dom-root of its
    own subtree; D, E, F are reached from S, not the other
    way). All → `descendants_or_other`.
  - `ancestors_of_splitter = {}` (empty, suppressed).
  - `descendants_or_other = {D, E, F}` (size 3) → emit
    `subgraph_0 = {D, E, F}`.

Final output: **one** `SubgraphInfo` = `{D, E, F}`.
Standalone nodes: A, P, P2, Q, Q2, S, Exit.

This is the BottomUp limitation flagged earlier: BottomUp emits
AT the splitter, so the emit point's subtree has no
ancestors-of-splitter content. Only the descendants group
survives. To get a non-trivial ancestors group, see
TopDownAggressive below.

### Pipeline = TopDownAggressive

**TopDownSingleSplitterPass** (pre-order with subtree skip):
- A: `splitter_count == 1`, `node_count == 10 > 1`, neither
  emitted nor descendants_emitted → **emit A**, skip descent.

After this pass: emit points = {A}.

Subsequent rescue passes (MultiSplitter, LargeSplitterFree)
all see A's whole subtree as `emitted`/`descendants_emitted` →
no emissions.

After all passes: emit points = {A}.

### BuildSubgraphInfos (TopDownAggressive, default policy)

- **A** (`subtree_splitter_count == 1` → split):
  - Walk subtree (everything: A, P, P2, Q, Q2, S, D, E, F, Exit).
  - For each member ≠ S: does it reach S in the DAG?
    - A → P → S, so A reaches S → `ancestors_of_splitter`.
    - P → S, P reaches → `ancestors_of_splitter`.
    - Q → S → `ancestors_of_splitter`.
    - P2: only edge is P2 → Exit. Doesn't reach S → `descendants_or_other`.
    - Q2: same → `descendants_or_other`.
    - D, E, F: reached from S, don't reach S → `descendants_or_other`.
    - Exit: reached from P2, Q2, F. Doesn't reach S → `descendants_or_other`.
  - `ancestors_of_splitter = {A, P, Q}` (size 3) → emit
    `subgraph_0 = {A, P, Q}`.
  - `descendants_or_other = {P2, Q2, D, E, F, Exit}` (size 6)
    → emit `subgraph_1 = {P2, Q2, D, E, F, Exit}`.

Final output: **two** `SubgraphInfo`s, each with ≥ 3 members.
Standalone: S.

### Same DAG, alternate `SplitterPartitionPolicy::kSplitDescendantsAndIndependents`

Same emit point (A). Re-partition the non-ancestor side:

- For each non-ancestor: does S reach it (DAG-descendant) or
  is it independent (neither)?
  - D, E, F: S reaches them (S → D → F, etc.) → `descendants_of_splitter`.
  - Exit: S reaches Exit (S → D → F → Exit) → `descendants_of_splitter`.
  - P2: S doesn't reach P2 (P2 is reached only from P, no edge from S) → `independents`.
  - Q2: same → `independents`.

- `ancestors_of_splitter = {A, P, Q}` (size 3) → emit `subgraph_0`.
- `descendants_of_splitter = {D, E, F, Exit}` (size 4) → emit `subgraph_1`.
- `independents = {P2, Q2}` (size 2) → emit `subgraph_2`.

**Three** `SubgraphInfo`s, each ≥ 2 members. The independents
subgraph's `ext_predecessors = {A}`, so it can schedule without
waiting on S — the parallelism the alternate policy is meant
to expose.

---

## 9. Implementation Phases and Test Plan

Each phase is independently testable. **Tests for the formation
machinery itself live as shakedowns in `Shakedowns.cpp`** — the
existing pattern (`RunInsertSubgraphProxiesShakedown`,
`RunSubgraphContiguityShakedown`, etc.). Synthetic
ScheduleGraphs are built directly in C++ via factory methods on
`ScheduleGraph` (the `BuildTestDAG` family), and tests check
the formation step's output against hand-computed expectations.
That gives us full control over DAG shape, which is impossible
if we tried to drive testing from hip kernels — the DAG depends
on codegen and we can't dictate its structure.

The sandbox `dfs_test`-style hip kernels are reserved for
**end-to-end smoke tests** in Phase 5: confirm the formation
pipeline runs to completion on a real region without crashing
or producing invalid assembly. They don't try to verify
specific subgraph contents — that's the shakedowns' job.

### Phase 1 — Splitter predicate + reachability cache

- Add `IsSubgraphSplitter(node, latency_threshold)` free function.
- Modify `ScheduleGraph::ComputeTransitiveReduction` to retain
  its reachability matrix in `reachability_by_topo_index_`;
  expose `IsReachableInDag`; clear in `InvalidateDerivedData`.
- Add the `DominatorTree::GetIDom(const ScheduleNode *)`
  convenience overload (§1.1).

Shakedown: on `BuildTestDAG` and a few new synthetic shapes,
verify the predicate fires on the right nodes (high-latency
outgoing edges) and reachability matches hand computation.
Verify the reachability cache is cleared after a graph
mutation.

### Phase 2 — SubgraphFormationTree + traversal helpers

- Add `SubgraphFormationTreeNode` (with `is_splitter`,
  subtree counts, dfs pre/post, mutable emission flags) and
  `SubgraphFormationTree` per §4.2.
- Implement `BuildFromDominatorTree`: parent/child wiring,
  subtree counts via `PostOrderApply`, dfs numbering via
  `PreOrderApply` + `PostOrderApply`.
- Implement `RecordEmission`, `CollectEmitPoints`,
  `IsInSubtreeOf`.
- Add the four traversal helpers (`PostOrderApply`,
  `PreOrderApply`, `PreOrderApplyWithSkip`, and their
  `*Iterative` variants).

Shakedown: on the §8 example DAG and small synthetic shapes,
verify tree structure, subtree counts, and dfs pre/post
numbers match hand computation. Verify `RecordEmission`
propagates `descendants_emitted` correctly along ancestor
chains. Verify `IsInSubtreeOf` answers correctly for a few
ancestor/descendant/sibling triples.

### Phase 3 — Per-decision-rule passes

- Add the five passes from §5: `BottomUpSingleSplitterPass`,
  `TopDownSingleSplitterPass`, `MultiSplitterRescuePass`,
  `LargeSplitterFreeRescuePass`, `SiblingRescuePass`.
- Note: `SiblingRescuePass` is implemented but NOT included in
  default pipelines (see §5.5). It exists as an opt-in
  primitive for experimentation.

Shakedown: per pass, on synthetic graphs designed to exercise
each rule's condition. Verify the right emit points are
marked. Cross-pass test: chain `BottomUpSingleSplitter` then
`TopDownSingleSplitter` and confirm the second is a no-op
(everything was already emitted).

### Phase 4 — Pipelines + BuildSubgraphInfos

- Add `SubgraphFormationPipeline`, `SubgraphFormationPolicy`,
  the two pipeline factory methods (`BottomUpDefault`,
  `TopDownAggressive`).
- Add `SplitterPartitionPolicy` enum.
- Add `SplitEmitPoint`, `BuildSubgraphInfos`. Wire the size-2
  minimum filter.

Shakedown: on the §8 example DAG, run both pipelines and
verify the emit points and resulting `SubgraphInfo`s match the
worked example (BottomUpDefault → 1 subgraph;
TopDownAggressive default → 2 subgraphs; TopDownAggressive
alternate → 3 subgraphs). Add a singleton-suppression case.

### Phase 5 — End-to-end driver

- Add `FormSubgraphs` (§7).
- Wire it into the production pass driver (the same hook the
  Phase 3 production-wiring item in the overall plan calls
  out).
- Re-derive topo / critical-path after `InsertSubgraphProxies`.

Smoke test: full end-to-end on a real AMDGPU region (via the
hip_stencil harness — `cd sandbox/hip_stencil && make`), with
`misched.txt` set to `HierarchicalScheduler`. Verify
(a) compile succeeds, (b) the graph ends up with at least one
proxy node, (c) produced assembly is valid. Schedule
correctness is downstream of formation — owned by whichever
search algorithm runs after the graph is shaped.

---

## Summary

`IsSubgraphSplitter` (default threshold 32, captures memory loads
on AMDGPU) marks splitters via outgoing latency. A
`SubgraphFormationTree` (pointer-based, one struct per dom-tree
node) holds per-node subtree counts, dfs pre/post numbers, and
mutable emission flags. Subgraph formation is structured as a
sequence of small passes, each implementing one decision rule
and updating the tree through a single `RecordEmission` mutator.
Pipelines compose passes in a chosen order: `BottomUpDefault`
emits at the lowest single-splitter subtrees plus
multi-splitter and large-no-splitter rescues; `TopDownAggressive`
emits at the highest single-splitter subtrees with the same
rescues. `SiblingRescuePass` is implemented but omitted from
default pipelines because it can delay critical-path execution
by reaching into sibling subtrees — a future safer version
would use `IsInSubtreeOf` to skip such cases. After passes,
each emit point becomes a `SubgraphInfo` (single-splitter emit
points are split via `SplitterPartitionPolicy`, multi-splitter
and splitter-free emit as-is), filtered by a singleton minimum,
and handed to `InsertSubgraphProxies`. Integration timing
(which scheduling passes call `FormSubgraphs`) is deferred to
when the relevant passes are wired up.
