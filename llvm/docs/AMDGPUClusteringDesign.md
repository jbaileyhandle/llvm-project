# HierarchicalScheduler: Subgraph Design Exploration

## Purpose of this Document

Two candidate architectures were under consideration for how the
HierarchicalScheduler should represent and schedule **subgraphs**
(sets of nodes that must be scheduled contiguously). Both are mapped
out below in enough pseudo-code that their complexity, leakage
surface, and implementation cost can be compared side by side.

**Outcome**: after working through both, Approach B was chosen. See
§4.1 for the comparison. Approach A is retained here for context;
its pseudo-code still uses the older "cluster / rep" vocabulary that
Approach B has since refined to "subgraph / proxy" — the underlying
concepts are the same, only the names differ.

The two approaches:

1. **Approach A (Two-Graph / Hierarchical)**: keep the original
   ScheduleGraph unmodified; build a separate "modulo" ScheduleGraph
   in which each subgraph is represented by a single synthetic proxy
   node and intra-subgraph edges are elided. Outer scheduling
   decisions are made on the modulo graph; when the outer scheduler
   picks a proxy, it descends into an inner scope that schedules the
   subgraph's real members on the original graph.

2. **Approach B (Single-Graph Hybrid)** — **chosen**: leave the
   original graph's nodes and edges in place. Insert synthetic
   subgraph-proxy nodes into the SAME graph and ADD artificial edges
   from every external predecessor of any member to the subgraph's
   proxy, plus artificial 0-latency edges from each proxy to each of
   its members. No rewriting of original edges. Readiness at each
   scope is gated by ordinary pred-count logic plus a scope-stack
   visibility rule — members are hidden from scopes other than
   their own.

## Table of Contents

1. [Background and Shared Prerequisites](#1-background-and-shared-prerequisites)
2. [Approach A: Two-Graph (Hierarchical)](#2-approach-a-two-graph-hierarchical)
3. [Approach B: Single-Graph Hybrid](#3-approach-b-single-graph-hybrid)
4. [Comparison and Open Questions](#4-comparison-and-open-questions)

---

## 1. Background and Shared Prerequisites

### 1.1 What subgraphs buy us

Grouping a set of related nodes into a subgraph and scheduling them
contiguously shrinks the outer search space: it no longer has to
interleave subgraph members with unrelated nodes. The cost is that
member-ordering decisions are pulled out of the global search into a
smaller nested search (potentially with caching).

### 1.2 Core design goals shared by both approaches

- **DFS-multiplication avoidance**: "schedule subgraph X next" must
  be ONE decision in the outer search, not N decisions that collapse
  to the same action (where N is the number of members that would
  be ready at the moment of entry).
- **Contiguity**: once a subgraph is entered, all its members must
  be scheduled before any non-member node or any *other* subgraph's
  member.
- **No caching assumption**: a subgraph's internal schedule is
  searched at scheduling time by default. Caching is a later
  optimization.
- **Nested subgraphs**: a subgraph may contain sub-subgraphs.
- **Unified ScheduleConstructor**: no separate
  `HierarchicalScheduleConstructor` wrapping a `ScheduleConstructor`.
  One class with scope-stack support handles both flat and nested
  cases.
- **Accurate length and register tracking**: trackers operate on
  real ScheduleNodes with real latencies and real register effects
  — no approximations at the emission layer.

### 1.3 Subgraph entry gating (shared machinery)

Both approaches need a well-defined "the subgraph is ready to enter"
criterion. The rule we use: **the subgraph is entered when every
external predecessor of any member is scheduled.** This does NOT
mean every member is simultaneously ready at that moment — only that
the outer scheduler is now allowed to commit to scheduling the
subgraph next. Members then become ready one-at-a-time according to
their own intra-subgraph dependencies, as scheduling progresses
inside the subgraph's scope.

Concretely:
- **Approach B** enforces this via an artificial edge from each
  external pred to the subgraph's proxy (§3.2). The proxy's
  pred_count drops to zero exactly when every external pred has
  been scheduled. Members' own pred_counts still include their
  intra-subgraph preds plus the artificial proxy→member edge, so
  members only become ready once BOTH the proxy is scheduled AND
  their intra-subgraph preds are done.
- **Approach A** embodies the same rule in its modulo-graph edge
  aggregation (§2.2): the modulo proxy node's preds are the union
  of external preds of its members.

### 1.4 Subgraph metadata (shared)

Vocabulary used throughout this document:

- **subgraph** (conceptual): a set of ScheduleNodes that must be
  scheduled contiguously. Synonymous with "cluster" in informal
  scheduling literature — we consistently use "subgraph" here.
- **SubgraphInfo** (the struct): the metadata record for one subgraph.
  Holds the member list, precomputed boundary (external preds/succs),
  and a backpointer to its proxy.
- **subgraph proxy** (the node): a ScheduleNode of subgraph-type that
  STANDS IN FOR the subgraph in the graph where the subgraph
  participates. Used as a single outer-scheduling decision point;
  does not contain the members. Shorthand: **proxy**.

```
struct SubgraphInfo {
  // Populated at construction: caller supplies `members` and
  // `debug_name`; constructor walks `members` once to fill in the
  // boundary (ext_preds, ext_succs). No graph side effects.
  SmallVector<ScheduleNode *, 32> members
  std::string debug_name
  SmallVector<ScheduleNode *, 16> ext_preds   // union over members
  SmallVector<ScheduleNode *, 16> ext_succs   // union over members

  // Populated later, when the proxy is added to the graph by
  // InsertSubgraphProxies.
  ScheduleNode *subgraph_proxy = nullptr

  SubgraphInfo(ArrayRef<ScheduleNode *> members, StringRef debug_name)
}
```

On the node side: every ScheduleNode has
`ScheduleNode *parent_subgraph_proxy` (null iff top level). Set by
`InsertSubgraphProxies` when members are assigned to a subgraph.
Used by the visibility rule at scheduling time.

### 1.5 Unified ScheduleConstructor with scope stack (shared)

Both approaches rely on ScheduleConstructor tracking a **stack of
scheduling scopes of arbitrary depth** — one per active nesting level.
Non-hierarchical scheduling is the stack-depth-1 case; any extra nesting
just pushes more scopes. There is no fixed "outer/inner" pair — every
scope is a parent to what pushes onto it and a child to whatever pushed
it. `scopes_.back()` is always the currently-active one.

The pushed unit is a named struct. Two fields for now; room for future
state (pass-specific bookkeeping, undo info, etc.) so we don't have
to fan out to parallel vectors later:

```
struct SubgraphScheduleScope {
  ScheduleNode *subgraph_proxy   // nullptr for the base scope (whole
                                 // graph); otherwise the proxy whose
                                 // subgraph this scope is scheduling
                                 // the members of.
  ReadyList ready                // nodes visible-in-this-scope that
                                 // are currently schedulable.
}
```

No scheduled-count, no completion flag. **"Scope done" is detected by
`ready.IsEmpty()`** (STL-style "no elements remain"). A subgraph scope
becomes done when its last member is scheduled; its ready list drains
to empty and we pop. The base scope becoming empty with no scope above
it on the stack means the whole region is done.

```
class ScheduleConstructor {
  std::vector<SubgraphScheduleScope> scopes_    // scopes_.back() = current
  GCNRegisterTracker reg_tracker_               // on real nodes only
  ScheduleLengthTracker len_tracker_            // on real nodes only

  Schedule(ScheduleNode *n)
  Unschedule()
  GetReadyListSnapshot(out)
  IsDone()    // scopes_.size() == 1 && scopes_[0].ready.IsEmpty()
}
```

Both approaches share this API. What differs is WHAT a "subgraph proxy"
looks like (separate graph vs same graph) and what's visible on the
ready list at each scope.

---

## 2. Approach A: Two-Graph (Hierarchical)

### 2.1 Structure

```
           outer scheduling view
          ┌───────────────────────┐
          │   ModuloScheduleGraph │        (one rep per cluster;
          │  (synthetic rep nodes │         non-cluster nodes have
          │   + stand-ins for     │         "stand-in" reps that
          │   non-cluster nodes)  │         alias to themselves)
          └─────────┬─────────────┘
                    │ picking rep R pushes frame,
                    │ delegates to inner view
                    │
          ┌─────────v─────────────┐
          │  Original             │
          │  ScheduleGraph        │        (pristine — unchanged)
          │                       │
          │   real MachineInstr-  │
          │   backed nodes live   │
          │   here                │
          └───────────────────────┘
```

Two graphs coexist. The **original graph** is never structurally mutated
by clustering — same nodes, same edges, same topo order, same critical
path. The **modulo graph** is built fresh from (original, cluster set)
and has one rep node per cluster plus one "stand-in" per non-cluster node
(so outer scheduling is always over modulo, uniformly). Intra-cluster
edges are elided in the modulo graph.

### 2.2 Modulo graph construction

```
def BuildModuloGraph(original: ScheduleGraph, clusters: List<Cluster>) -> ScheduleGraph:
  modulo = new ScheduleGraph
  original_to_modulo = DenseMap<ScheduleNode*, ScheduleNode*>()

  // 1. Create one stand-in per non-cluster node, one rep per cluster.
  for each node N in original.nodes_:
    if N not in any cluster:
      stand_in = modulo.EmplaceLeaf(debug_name=N.name, aliased_original=N)
      original_to_modulo[N] = stand_in
  for each cluster C in clusters:
    rep = modulo.EmplaceRep(debug_name=C.name, cluster=C)
    C.rep = rep
    for each member M of C:
      original_to_modulo[M] = rep

  // 2. Add aggregated edges.
  //    For each original edge A -> B with latency L:
  //      if A, B map to the same modulo node (both in same cluster) -> skip
  //      otherwise add (or merge with) a modulo edge mapped(A) -> mapped(B)
  //    Merge policy depends on who consumes the latency:
  //      - for outer topo / outer CP / outer length-LB:
  //          use MAX of L over all contributing original edges
  //          (conservative — we don't yet know the cluster-internal offset
  //          of A or B within their cluster)
  //      - register-side edges don't need a latency
  //    Recommended: the modulo stores both an edge kind and a latency;
  //    latency aggregation is upper-bound by default.
  for each node A in original.nodes_:
    for each edge A -> B (latency L, kind K) in original:
      a_m = original_to_modulo[A]
      b_m = original_to_modulo[B]
      if a_m == b_m: continue  // intra-cluster edge, elided
      if modulo has existing a_m -> b_m edge E':
        E'.latency = max(E'.latency, L)   // upper bound
        E'.kind = merge_kinds(E'.kind, K)
      else:
        modulo.AddEdge(a_m -> b_m, L, K)

  modulo.ComputeTopologicalOrder()
  modulo.ComputeCriticalPathFromExit()
  return modulo
```

### 2.3 Scheduling pseudo-code

```
def HierSC::Schedule(node: ScheduleNode*):
  // node may be in ANY of the active graphs (modulo, inner cluster, etc.)
  current = frames.back()

  if node.IsSubgraphProxy():            // subgraph proxy in the current frame's graph
    cluster = node.cluster()
    // Push new frame, start inner scheduling over cluster members.
    inner_frame = Frame{
      ready = { cluster roots (members with no intra-cluster preds) },
      active_scope = cluster,
      scheduled_count = 0
    }
    frames.push(inner_frame)
    return

  if node.IsStandIn():        // stand-in for a non-cluster real node
    real = node.aliased_original
    // Schedule the real node — this is what produces emitted output.
    reg_tracker.Schedule(real)
    len_tracker.Schedule(real)
    output_order.push_back(real)
    current.scheduled_count++

    // Update ready list: remove this node, release successors whose
    // preds are now satisfied (in the current frame's graph).
    current.ready.remove(node)
    for each succ edge S in current.graph:
      if all preds of S in current.graph are scheduled:
        current.ready.insert(S)

    // Check frame completion (outer frame complete means all scheduled):
    if current.scheduled_count == current.graph.Size():
      // Outer frame is done — we only ever pop it at end-of-search.
      return

  if node is a cluster member (inner frame scheduling):
    reg_tracker.Schedule(node)
    len_tracker.Schedule(node)
    output_order.push_back(node)
    current.scheduled_count++

    current.ready.remove(node)
    for each intra-cluster succ S of node:
      if all intra-cluster preds of S are scheduled:
        current.ready.insert(S)

    // Inner frame complete?
    if current.scheduled_count == current.active_scope.members.size():
      completed_cluster = current.active_scope
      frames.pop()
      // Now update the previously-pushed (outer) frame as if the rep
      // had just been "scheduled".
      outer = frames.back()
      outer.scheduled_count++
      outer.ready.remove(completed_cluster.rep)
      for each succ edge S of completed_cluster.rep in outer.graph:
        if all preds of S in outer.graph are scheduled:
          outer.ready.insert(S)
```

Unschedule is the mirror. Subtle bit: popping an inner frame during
Schedule must be pushable-backable in Unschedule. Cleanest is to record
a marker in an undo stack when the frame was pushed / popped and retrace
precisely.

### 2.4 Register and length tracker interaction

- Trackers live on the **original graph** only. Cluster reps and stand-
  ins in the modulo graph never call into them.
- `HierSC::Schedule(stand_in)` and `HierSC::Schedule(cluster_member)` both
  invoke trackers on the aliased real node; `HierSC::Schedule(rep)` does
  not (it only pushes a frame).
- Register pressure at outer scheduling points reflects real live-ins/
  outs of already-scheduled clusters / stand-ins. When an inner frame
  runs, pressure naturally tracks member-by-member as scheduling
  progresses inside the cluster.

### 2.5 Topo order and critical path

- **Original graph**: `topo_order_original`, `cp_from_exit_original` —
  computed once, stable. Used by the inner frame (when cluster is
  active) to drive bounds for the inner search.
- **Modulo graph**: `topo_order_modulo`, `cp_from_exit_modulo` — computed
  at modulo construction. Used by the outer frame for bounding.
- Neither is invalidated by scheduling decisions. Clustering is a
  pre-scheduling phase; graphs are frozen afterwards.

### 2.6 Nested clusters

Each level of nesting gets its own modulo graph:

```
Level 0 (outermost)    = modulo over original, with top-level subgraph proxys
Level 1 (mid)          = modulo over a top-level cluster's member set,
                         with any sub-subgraph proxys
Level 2 (innermost)    = member set of a sub-cluster (no further reps)
...
```

Each cluster carries its own member-level ScheduleGraph, constructed
lazily or eagerly. Scheduling a rep at level K pushes a frame whose
`active_scope` is the rep's cluster and whose `graph` is the level-(K+1)
member graph.

Downside: N nesting levels = N graphs to build and maintain.

### 2.7 Outer bounding

Outer DFS can bound using modulo-graph data without needing inner results:

- **Occupancy pass outer bound**: rep's pre-computed minimum-achievable
  peak pressure (computed by running the inner optimizer once per cluster
  at graph construction — optional; approximation is fine otherwise).
- **Length pass outer bound**: rep's critical-path length (lower bound,
  using edge-latency minima; or conservative upper bound for early
  pruning).

Whether outer bounding is worth the complexity is an open question —
inner bounding may already prune enough. Start without outer bounds;
add them if outer search is a hotspot.

### 2.8 Open questions and sharp edges (Approach A)

- **Modulo edge latency aggregation**: for a rep with multiple incoming
  edges from the same external node (each to different cluster members),
  we merge into one modulo edge. What latency do we use? Max for
  upper-bound on outer CP (safe); min for lower-bound on outer length
  (safe). These may want separate copies, or we compute bounds
  differently at query time.
- **Stand-in indirection**: outer scheduling goes through
  `modulo_node -> aliased_original` for every real operation. Extra
  indirection on the hot path. Mitigation: stand-ins just hold a raw
  pointer; one extra load.
- **Graph count grows with nesting depth**: each cluster level has its
  own graph. For 3 levels over 50 clusters you could have ~150 graphs
  in flight. Memory is small (KBs), but the bookkeeping surface grows.
- **Unschedule undo of frame push/pop** needs to interleave correctly
  with tracker undo. Probably straightforward but easy to get subtly
  wrong — needs its own shakedown.

---

## 3. Approach B: Single-Graph Hybrid

### 3.1 Structure

One ScheduleGraph contains BOTH real member nodes and synthetic
subgraph-proxy nodes. The subgraph-insertion pass does NOT rewrite
or remove any original edge. It only ADDS:

  1. One proxy node per subgraph.
  2. Artificial edges from every external predecessor (of any subgraph
     member) to the subgraph's proxy — gates proxy readiness on all
     subgraph live-ins being scheduled.
  3. Artificial 0-latency edges from each subgraph proxy to each of
     its direct members — gates each member's readiness on the proxy
     being "scheduled" (which in practice means "frame pushed").

All artificial edges use `kArtificial`, so `IsLatencyEdge()` filters
them out of length tracking and critical-path computation. They
contribute ONLY to pred-count readiness (scheduling-order gating),
not to timing estimates or emitted code.

```
  ┌────────────────────────────────────────────────────────┐
  │           ScheduleGraph (additions shown ·····)        │
  │                                                        │
  │   A ──────► B      (original, latency LAB)             │
  │   A ──────► C      (original, latency LAC)             │
  │   A ──────► D      (original, latency LAD)             │
  │   A ·······► X (proxy for {B,C,D})   (artificial)      │
  │                                                        │
  │   X ·······► B    (artificial, latency 0)              │
  │   X ·······► C    (artificial, latency 0)              │
  │   X ·······► D    (artificial, latency 0)              │
  │                                                        │
  │   B ──────► C      (original, intra-subgraph)          │
  │   D ──────► E      (original, member → external)       │
  │                                                        │
  │   A → B, A → C, A → D are NOT rewritten.               │
  │   B, C, D stay in the graph; they only become ready    │
  │   once proxy X is "scheduled" (because X is a pred of  │
  │   each via the X ····► member edges).                  │
  └────────────────────────────────────────────────────────┘
```

At the base scope, `X` is visible and becomes ready when all external
preds — here, `A` — are scheduled (via `A ····► X`). Members `B, C, D`
are NOT ready at the base scope because `X` is an unsatisfied pred of
each. Picking `X` "schedules" it (pushes a scope) and triggers normal
edge-release logic on `X`'s outgoing edges: for each member, decrement
its pred count; if all preds (real and artificial) are done, release
the member — the stack walk in Schedule directs it to the newly-
pushed scope's ready list, since its `parent_subgraph_proxy` matches.
No special push-time seeding code is needed; ordinary per-edge
release handles it.

### 3.2 Graph construction (subgraph-insertion pass, post-BuildFromSUnits)

**Scope limitation for this document**: construction pseudo-code here
assumes a **flat, two-level hierarchy only** — a base graph plus one
level of subgraphs, where every subgraph's members are raw real nodes
(no sub-subgraphs). Nested subgraphs are expected to work with the
scheduling-time mechanisms (stack of scopes, proxy→member edges,
`parent_subgraph_proxy` on nodes) essentially unchanged, but the
construction step has to build bottom-up and propagate
"external-to-the-whole-subtree" pred sets correctly at each level.
That extension is noted at the end of this section and not expanded
here — adding it is straightforward but bookkeeping-heavy and would
distract from the main story.

Input: a vector of `SubgraphInfo *` whose `members` and `debug_name`
fields are already populated (the SubgraphInfo constructor walks
`members` to also fill in `ext_preds` and `ext_succs`; see §1.4).

```
def InsertSubgraphProxies(graph: ScheduleGraph,
                          infos: ArrayRef<SubgraphInfo *>):
  // Assumption: no info's members overlap with another's; no nesting.
  for each SubgraphInfo *info in infos:
    proxy = graph.EmplaceSubgraphProxy(info)
    info.subgraph_proxy = proxy
    // Mark each member's parent proxy for scope-visibility lookups
    // during scheduling.
    for each member M in info.members:
      M.parent_subgraph_proxy = proxy
    // proxy.parent_subgraph_proxy stays null (top-level in the
    // two-level world).

    // 1. Artificial external-pred edges: for every pred P in
    //    info.ext_preds, add ONE artificial edge P -> proxy.
    //    (No dedup loop needed — ext_preds is already the unique
    //    set, computed by SubgraphInfo's constructor.)
    for each P in info.ext_preds:
      graph.AddEdge(P -> proxy, latency=0, kind=kArtificial)

    // 2. Artificial proxy -> member edges, one per member. These
    //    gate each member's pred-count readiness on the proxy being
    //    "scheduled" (frame pushed). Length 0, kind kArtificial —
    //    IsLatencyEdge() excludes them, so they don't affect length
    //    tracker cycles or critical-path computation.
    for each member M in info.members:
      graph.AddEdge(proxy -> M, latency=0, kind=kArtificial)

    // NOTE: original P -> M edges are NOT removed. They remain in
    // the graph and still propagate real latency for length tracking
    // and register liveness. A member's pred count now includes both
    // its real preds AND the artificial proxy -> M edge.
```

After `InsertSubgraphProxies` returns, the graph has been structurally
modified (new proxy nodes, new artificial edges). Per-graph derived
computations that depend on graph structure — topological order,
critical-path-from-exit, transitive reduction, dominator tree, etc. —
need to be computed (or recomputed) AFTER this step, by whoever drives
the graph through its pipeline. That's not the insertion pass's
job; this function just produces the mutated graph.

**Extension to nested subgraphs (future work, not implemented here)**:

- Build subgraphs bottom-up (post-order traversal of the subgraph-
  containment tree).
- A parent subgraph's `ext_preds` can be computed by unioning
  `ext_preds` of its direct child subgraphs (filtered to preds
  outside the parent's subtree) with the real preds of its
  non-subgraph members. No re-scanning of member-by-member needed
  because each child already cached its boundary.
- `proxy.parent_subgraph_proxy` is set by the parent subgraph's build
  step (when the parent iterates its direct members and finds the
  sub-subgraph proxy among them).
- Scheduling pseudo-code (§3.3) does not need to change — the
  visibility rule, stack walk, proxy→member edges, and pop-cascade
  all generalize naturally to deeper stacks.

### 3.3 Scheduling pseudo-code

**Parent subgraph proxy.** Every node (real or proxy) has exactly one
`parent_subgraph_proxy`: the proxy of the subgraph the node is a
direct member of, or `null` if it sits at the top level of the
hierarchy.

Under the two-level assumption of §3.2:
- Real non-subgraph nodes: `parent_subgraph_proxy = null`.
- Real subgraph members: `parent_subgraph_proxy = their subgraph's proxy`.
- Subgraph proxies: `parent_subgraph_proxy = null` (every subgraph is
  top-level, so no containing subgraph exists).

Under future nesting, a sub-subgraph's proxy would carry the containing
parent subgraph's proxy as its `parent_subgraph_proxy` (the proxy
itself sits inside its parent subgraph, even though it *represents* a
different subgraph). The visibility rule below is identical in both
cases.

**Visibility rule** at a scope: a node is visible iff
`node.parent_subgraph_proxy == scope.subgraph_proxy`. That's the whole
rule.

- Base scope (`subgraph_proxy = null`): non-subgraph real nodes and
  all top-level subgraph proxies are visible.
- Scope for subgraph whose proxy is `X` (`subgraph_proxy = X`): X's
  direct members are visible. Those are real member nodes under the
  two-level assumption; under future nesting they can also include
  proxies of sub-subgraphs of X.

**Finding the target scope during release**: when a scheduled node's
successor `S` needs releasing, walk the stack top-down looking for
the scope whose `subgraph_proxy` equals `S.parent_subgraph_proxy`.
Common case (same-scope successor — most real edges inside a
subgraph) hits on the first iteration, so release is effectively
O(1). Cross-scope releases (e.g., a member's external successor)
hit further down the stack. Stack depth is small (≤ 2 in the two-
level world, a few under future nesting), and there's no extra state
to maintain vs. a subgraph→scope map, so the stack walk wins on
simplicity.

Pseudo-code:

```
def HybridSC::Schedule(node: ScheduleNode*):
  current = scopes_.back()

  if node.IsSubgraphProxy():
    // Push a new scope for this subgraph. No special ready-list
    // seeding here — the common edge-release loop below handles it:
    // proxy has artificial 0-latency outgoing edges to each member
    // (added in §3.2), and releasing them via the stack walk drops
    // each newly-ready member into the scope we just pushed.
    current.ready.Remove(node)     // proxy consumed from current scope
    scopes_.push(SubgraphScheduleScope{subgraph_proxy=node, ready=empty})
    // Fall through to the common release loop. Don't call trackers —
    // proxy is synthetic.
  else:
    // Real node: trackers + emission.
    reg_tracker_.Schedule(node)
    len_tracker_.Schedule(node)
    output_order_.push_back(node)
    current.ready.Remove(node)

  // Common edge-release loop. Serves both branches above.
  // FindScopeOnStack always returns a valid scope by construction
  // (see subtleties below); callers don't need to null-check.
  for each outgoing edge node -> S (real or artificial):
    if all preds of S are scheduled:     // pred-count reached zero
      target_scope = FindScopeOnStack(S.parent_subgraph_proxy)
      target_scope.ready.Insert(S)

  // Pop-cascade: if the current scope is a subgraph scope whose
  // ready list has drained, the subgraph is fully scheduled. Pop.
  // Loop, because popping can reveal a now-drained parent scope too
  // under future nesting (trivially impossible under the two-level
  // assumption but cheap and forward-compatible).
  while scopes_.back().subgraph_proxy != null AND scopes_.back().ready.IsEmpty():
    scopes_.pop()

def FindScopeOnStack(target_proxy: ScheduleNode *) -> SubgraphScheduleScope *:
  // Walk from top of stack so intra-scope releases hit on iteration 1.
  for scope in reverse(scopes_):
    if scope.subgraph_proxy == target_proxy:
      return &scope
  // Fallthrough: not found. The proxy→member artificial edges make
  // this unreachable by construction — see subtleties. Assert here
  // rather than having every caller null-check.
  assert(false, "target subgraph proxy not on the scope stack")
```

Subtleties:

- Artificial `P → proxy` edges appear as outgoing edges of `P` and as
  preds of the proxy — they're in the pred-count bookkeeping that
  gates proxy readiness. `IsLatencyEdge()` skips them, so they don't
  distort length tracker cycles or CP.
- Artificial `proxy → member` edges are outgoing edges of the proxy.
  The push branch falls through to the common release loop, which
  decrements member pred counts via these edges; any member whose
  other (real) preds are already done is released into the just-
  pushed scope.
- "All preds of S are scheduled" is the raw pred-count-reaches-zero
  test. Every incoming edge (real or artificial) contributes equally
  to pred count.
- **Why `FindScopeOnStack` always finds a match by construction**: a
  member node S's pred_count includes the artificial
  `parent_proxy_of(S) → S` edge. That edge's source is S's
  `parent_subgraph_proxy`. So S's pred_count cannot drop to 0 until
  `parent_proxy_of(S)` is scheduled — which is the exact moment the
  parent subgraph's scope is pushed onto the stack. Corollary:
  whenever the release step runs for S (pred_count reached 0), S's
  parent scope is on the stack. True in both the two-level and
  nested cases.
- Pop-cascade is a simple loop; no recursion.

### 3.4 Register and length tracker interaction

- Trackers **skip proxy nodes**. Proxies are in the graph but have no
  real register or cycle effect. Enforcement has two parts:
  - **Schedule layer**: `HybridSC::Schedule(node)` only calls
    `reg_tracker_.Schedule(node)` / `len_tracker_.Schedule(node)` on
    the real-node branch. The proxy branch pushes a scope and falls
    through to the release loop without touching trackers. So the
    trackers never see a proxy at schedule time.
  - **Tracker construction / data layout**: any per-node state
    trackers build upfront (e.g., `node_reg_info_` in
    `GCNRegisterTracker`) must tolerate proxy entries. Simplest:
    skip proxies when extracting, leaving no entry; the fallback
    case of "no entry found" already means "no register effect",
    which is correct for proxies.
- Length tracker's readiness uses `IsLatencyEdge()`, which excludes
  both `P → proxy` and `proxy → member` artificial edges. So a
  member's `ready_cycle` is computed from its real preds' cycles
  only — the proxy doesn't funnel or inflate latency. This is what
  makes the §3.1 "ready-time tying" concern not actually a problem.

### 3.5 Topo order and critical path

- **Single topo order**, computed over the graph with proxies and
  artificial edges included. Proxies come AFTER their external preds
  (via `P → proxy`) and BEFORE their members (via `proxy → M`) in
  topo order.
- `cp_from_exit` is unaffected by subgraph insertion because
  `ComputeCriticalPathFromExit` iterates only `IsLatencyEdge()`
  edges. Artificial edges contribute no latency. Proxies' cp values
  come out as 0 (no latency-carrying outgoing edges from a proxy).
  Members' cp values depend only on their real outgoing edges —
  identical to the pre-clustering case.
- Any per-node algorithm that walks `graph.Nodes()` or
  `GetTopoOrder()` and wants to handle proxies differently needs
  `if (node->IsSubgraphProxy()) ...`. This is most of the audit
  surface called out in §3.8.

### 3.6 Nested subgraphs (future — not built here)

Intentionally brief; the design pseudocode in §3.2 and §3.3 is
scoped to the two-level case. When nesting is added:

- **Construction** (§3.2) changes: build subgraphs bottom-up and
  propagate each one's cached `ext_preds` upward so a parent's build
  can compute external-to-the-whole-subtree preds without re-scanning
  its descendants. `SubgraphInfo::ext_preds` is already there for
  this — its use in nested construction is why we cache it.
- **Proxy `parent_subgraph_proxy`**: a sub-subgraph's proxy is a
  direct member of its parent subgraph, so the proxy's
  `parent_subgraph_proxy` is set to the parent's proxy by the
  parent's build step.
- **Scheduling pseudo-code** (§3.3) does NOT change. Visibility,
  stack walk, `proxy → member` release, and pop-cascade all
  generalize; the "`FindScopeOnStack` always finds a match"
  invariant holds because the artificial edge from a node's parent
  subgraph proxy to the node is still what gates pred_count reaching
  zero.

### 3.7 Outer bounding

Same conceptual options as Approach A: bound outer-level search on
per-subgraph peak-pressure lower bound, length lower bound, etc.
Because everything lives in one graph with one topo and one CP,
bounds are computed on the same data structure — modest simplicity
advantage over Approach A (no separate modulo CP to compute).

Whether outer bounding is worth implementing is an open question
(same as Approach A): inner search may prune enough on its own.
Start without; add only if profiling shows outer search is a
hotspot.

### 3.8 Open questions and sharp edges (Approach B)

- **Audit surface for skip-proxy**: trackers, CP, topo iteration,
  output emission, anywhere that walks `graph.Nodes()` — each spot
  needs to either skip proxies or tolerate empty per-proxy state.
  5-8 touch points. Each small and mechanical, but spread across
  the codebase.
- **`graph.Size()` means something different**: it now counts
  proxies too. Any algorithm that assumes "Size() == real-
  instruction count" becomes wrong. Needs a clean `Size()` vs
  `LeafSize()` distinction (ScheduleNode already has `LeafSize` for
  the subgraph-content case — same concept).
- **Pred-count bookkeeping**: every node's "is this ready?" check
  counts all incoming edges uniformly, so adding artificial edges
  changes the count. Members have one extra pred (the
  `proxy → M` edge) that must be decremented when the proxy is
  scheduled. This is normal release logic, not a new bug class —
  but worth calling out that `pred_count == 0` is the readiness
  test, not "all real preds scheduled."
- **Mutation of the original graph**: the subgraph-insertion pass
  ADDS nodes and edges. The graph after `InsertSubgraphProxies` is
  not the same graph `BuildFromSUnits` produced. Any derived data
  computed against the pre-insertion graph (topo, CP, etc.) must be
  recomputed after — see the note at the end of §3.2.
- **Nested construction bookkeeping** (future work, §3.6): the
  `ext_preds` propagation adds meaningful boilerplate even though
  it's conceptually straightforward.

---

## 4. Comparison and Open Questions

### 4.1 Side-by-side summary

| Dimension                       | Approach A (Two-Graph)                   | Approach B (Hybrid Single-Graph)           |
|---------------------------------|------------------------------------------|--------------------------------------------|
| **Number of graphs**            | 1 original + 1 modulo (per nesting level) | 1 graph, mutated by adding proxy nodes + artificial edges |
| **Original graph mutated?**     | No                                       | Yes (proxies added, artificial edges added; no real edges touched) |
| **Proxy lives where?**          | Only in modulo graph                     | In the same graph as real nodes            |
| **Trackers see**                | Real nodes only (clean)                  | Real + proxy; skip proxies at schedule-time AND at tracker construction time (audit surface) |
| **Topo order**                  | Two separate orders, each over a graph of homogeneous node kind | One order over a graph of mixed kinds; walks need proxy-skip |
| **CP computation**              | Two separate (original + modulo)         | One; accurate for free because `IsLatencyEdge()` excludes artificial edges |
| **Proxy → member gating**       | Modulo proxy node → stand-in nodes       | Artificial `proxy → M` edges on the single graph; readiness via normal pred-count |
| **Nested subgraph cost**        | 1 extra graph per nesting level          | No extra graphs; bottom-up construction + `ext_preds` bookkeeping |
| **Stand-in indirection cost**   | Yes (outer nodes alias real nodes)       | No                                         |
| **Primary leakage risk**        | Two-graph synchronization bugs           | Forgot-to-skip-proxy bugs                  |
| **Fits unified ScheduleConstructor** | Yes                                 | Yes                                        |

### 4.2 Open design questions shared by both

- **Outer bounding: worth it?** We can start without and see. Inner
  bounding likely prunes most of the waste. If outer search blows up,
  add outer bounds. Same answer for both approaches.
- **Length pass: subgraph-internal re-search per context?** As
  noted earlier, within-subgraph optimal order for length *depends
  on* where the subgraph sits in the outer schedule (when it can
  start vs when its external successors need its outputs). Both
  approaches allow inner DFS to re-search each time; neither FORCES
  caching. This deferred concern is orthogonal to which proxy
  representation we pick.
- **Subgraph-definition pass itself**: what defines a subgraph?
  Heuristics (e.g., a def and all of its single-use memory ops) or
  structural (SILoadStoreOptimizer-style chains)? Independent of
  this document.

### 4.3 Not-yet-addressed in this document

- How subgraphs interact with the `SortRegionsByOriginalRegister...`
  step — presumably the per-region work is the same; subgraph
  insertion is per-region after region ordering is decided.
- How ACO / other search strategies interact with the scope stack.
  Probably the same — ACO's ants just traverse the scope-stacked
  ready lists the same way DFS does.
- Undo/redo semantics across frame push/pop in combination with the
  register tracker's delta undo. Non-trivial to get right; deserves
  its own shakedown whichever approach is chosen.

---

## Outcome

**Approach B (Single-Graph Hybrid) was chosen.** The deciding
considerations — derived data correctness falls out of
`IsLatencyEdge()` for free, fewer data structures to synchronize,
better scaling into nested subgraphs, simpler scheduling layer —
are laid out in §4.1 and the sharp-edge lists in §2.8 and §3.8.
Approach A is retained for reference.
