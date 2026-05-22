# HierarchicalScheduler: Subgraph Scheduling Design

## Purpose of this Document

`AMDGPUSubgraphFormationDesign.md` specifies how a region's graph is
*partitioned* into subgraphs. `AMDGPUClusteringDesign.md` (Approach B)
specifies how a chosen subgraph is *represented* in the ScheduleGraph
— a start/end proxy node pair plus a `SubgraphInfo`. Neither says how
the pieces are driven: when each subgraph is scheduled, in what order
relative to formation, or how a subgraph's chosen interior is held
fixed while the enclosing graph is scheduled.

This doc fills that gap. It covers:

- the recursive **process loop** that ties formation and scheduling
  together (§2–§4); and
- the **enforcement** mechanism that pins a subgraph's chosen
  schedule when the enclosing graph is scheduled (§5–§7).

It assumes the formation and clustering docs and describes the
machinery around them.

## Table of Contents

1. [Motivation](#1-motivation)
2. [The Process Loop](#2-the-process-loop)
3. [Formation/Scheduling Order and Recursion](#3-formationscheduling-order-and-recursion)
4. [Policy](#4-policy)
5. [Schedule Enforcement: The Requirement](#5-schedule-enforcement-the-requirement)
6. [Enforcement A: Order Edges (Selected)](#6-enforcement-a-order-edges-selected)
7. [Enforcement B: Stack-Based Cursor (Alternative)](#7-enforcement-b-stack-based-cursor-alternative)
8. [Interleaving Mode (Future)](#8-interleaving-mode-future)
9. [Status and Next Steps](#9-status-and-next-steps)

---

## 1. Motivation

The HierarchicalScheduler schedules each region with a near-exhaustive
search (BFS-DP, DFS, or branch-and-bound). Search cost grows
super-linearly in graph size: large regions are expensive and some
time out before the search completes (e.g. `hip_stencil` region[0],
~639 instructions).

Decomposition trades one large search for several small ones plus a
reduced top-level search:

1. Partition the region's graph into subgraphs.
2. Schedule each subgraph **in isolation** — a small graph, a cheap
   search.
3. **Lock** each subgraph's chosen interior order and schedule the
   whole graph, with the subgraph interiors fixed.

In step 3 the top-level search no longer orders all *N* instructions;
it orders the *K* subgraphs and the non-subgraph instructions, and
each subgraph's interior is a single forced sequence. The number of
free decisions drops sharply.

The design is **recursive**: a subgraph may itself be partitioned, so
"schedule a subgraph in isolation" is the same operation as "schedule
the region." The first implementation is flat (one level); the
structure is built so recursion is a drop-in (§3).

---

## 2. The Process Loop

The core routine, `DecomposeAndSchedule`, takes a ScheduleGraph and
an options bundle and returns a `SearchResult`:

```
SearchResult DecomposeAndSchedule(
        ScheduleGraph &graph,
        const GCNSubtarget &st,
        const MachineFunction &mf,
        const DecomposeAndScheduleOptions &opts):

    // 1. FORM — partition `graph`, insert start/end proxy pairs.
    //    May decline to subdivide (graph small enough, or a depth
    //    bound reached); then steps 2-3 see no subgraphs and this
    //    call behaves as a leaf.
    FormSubgraphs(graph, opts.formation)

    // 2. SCHEDULE SUBGRAPHS — each in isolation. opts.inner_search
    //    decides what to do with each: run a leaf search directly,
    //    or recurse via DecomposeAndSchedule with chosen inner options.
    for (SubgraphInfo *info : graph.GetSubgraphInfos()):
        ScheduleSubgraph(*info, graph, st, mf, opts.inner_search)

    // 3. LOCK — pin the chosen interiors into `graph`.
    AddSubgraphOrderEdges(graph)

    // 4. OUTER SEARCH over the proxied + chained graph.
    return opts.outer_search(graph)
```

**Step 1 — Form.** `FormSubgraphs` (see
`AMDGPUSubgraphFormationDesign.md`) partitions `graph` and calls
`InsertSubgraphProxies`, which replaces each member set with a
start/end proxy pair and records a `SubgraphInfo`. If formation
declines to subdivide, this is a no-op and the call bottoms out as a
leaf.

**Step 2 — Schedule subgraphs.** For each `SubgraphInfo`,
`ScheduleSubgraph` extracts the members into a standalone
ScheduleGraph (`ScheduleGraph::BuildFromNodeSubset`), runs the
caller-supplied `functor` on it, and records the recovered order plus
register metadata on `info.schedule_result`.

**Step 3 — Lock and schedule.** `AddSubgraphOrderEdges` bakes each
subgraph's chosen interior into `graph` (§6); `RunSearch` then runs
the top-level search, whose subgraph interiors are now fixed.

### 2.1 Recursion rides on the functor

`ScheduleSubgraph`'s `functor` parameter has type
`function_ref<SearchResult(ScheduleGraph &)>` — the seam through
which per-subgraph behavior is injected. A flat use sets a leaf
functor (BFS-DP or DFS directly). A recursive use sets a functor
that calls `DecomposeAndSchedule(sub, inner_opts)`, closing over the
inner options it wants. Either way the driver code is the same; the
driver itself takes no `depth` or provider.

`ScheduleSubgraph` and `BuildFromNodeSubset` were written to this
contract. The one change recursion will need: `ScheduleSubgraph` must
pass the extracted graph to the functor by **non-const** reference —
today's leaf searches do not mutate it, but a recursive
`DecomposeAndSchedule` runs `FormSubgraphs` on it.

### 2.2 Existing vs. new components

| Component | Status |
|---|---|
| `FormSubgraphs`, `InsertSubgraphProxies` | exists |
| `ScheduleGraph::BuildFromNodeSubset` | exists |
| `ScheduleSubgraph` | exists |
| Scope push/pop in `ScheduleConstructor` | exists |
| `ScheduleGraph::AddSubgraphOrderEdges` (§6) | exists |
| `DecomposeAndSchedule` driver, `DecomposeAndScheduleOptions` (§4) | next to implement |

---

## 3. Formation/Scheduling Order and Recursion

### 3.1 A depth-first walk of the subgraph tree

The subgraphs of a region form a tree: the region is the root, each
subgraph a child, recursively. `DecomposeAndSchedule` walks that tree
depth-first. Within one call, **formation precedes scheduling**:
step 1 forms, step 2 recurses into each child (forming and scheduling
its whole subtree), step 3 schedules this graph.

So formation is the **pre-order** action — a graph must be
partitioned before its children exist — and scheduling is the
**post-order** action — a parent's step-3 search needs its children's
locked orders. Along any root-to-leaf path this reads as "form down,
schedule up."

This is **not** a global two-phase split (all formation, then all
scheduling). It does not need to be: formation consumes **no** schedule
results — it is a pure function of graph structure (dominator tree +
latency splitters) and the formation policy. Two consequences:

- The per-subgraph loop in step 2 is data-independent — each
  iteration writes a distinct `SubgraphInfo` and reads `graph` only
  through `const` references. Subtrees are independent work and the
  loop is **parallelizable**, without holding the whole tree in
  memory the way a two-phase split would.
- If formation should ever react to schedule results, that belongs in
  an explicit outer refinement loop (form → schedule → re-form), not
  smeared into `DecomposeAndSchedule`.

### 3.2 The base case

`FormSubgraphs` declines to subdivide when the graph is small enough
or a depth bound is hit. Then step 2's loop is empty and step 3
schedules the graph directly. That is the recursion's base case —
there is no separate "leaf" code path.

### 3.3 Results flatten automatically

A subgraph's recorded order must end up expressed in nodes that
outlive the transient extracted graphs. This happens for free.

`ScheduleSubgraph` translates a recovered order back to parent-graph
member nodes: it keeps each node that `BuildFromNodeSubset`'s
extraction map resolves to a parent member and drops the rest. The map
has an entry for every extracted member node. When the extracted graph
is recursively sub-formed, the sub-subgraphs' members are a *subset*
of those member nodes — all still in the map — while the proxy nodes
sub-formation *adds* are not. So the existing translation flattens
automatically: members at any nesting depth survive; proxies at any
depth drop, by the same mechanism that already drops the extracted
graph's synthetic entry/exit sentinels.

No "proxy expansion" step is needed. Each recursion level translates
one step up and sheds the proxies it created; the root call returns a
flat sequence of atomic instruction nodes. (The root graph's own
proxies are dropped by the final consumer when the instruction order
is emitted.)

### 3.4 Worked example: two levels

Take a region graph **G** that forms one subgraph **A**, whose
interior in turn forms one sub-subgraph **A1**; **A1** is small
enough that formation declines to subdivide it. At each level the
caller supplies an `inner_search` that recurses into
`DecomposeAndSchedule` with that level's inner options:

```
DecomposeAndSchedule(G, outer_opts)
  1. FormSubgraphs(G)              → subgraph A; proxies inserted into G
  2. ScheduleSubgraph(A, G, …, outer_opts.inner_search)
       BuildFromNodeSubset(A.members, G)        → standalone graph G_A
       inner_search → DecomposeAndSchedule(G_A, level_1_opts)
         1. FormSubgraphs(G_A)     → sub-subgraph A1; proxies into G_A
         2. ScheduleSubgraph(A1, G_A, …, level_1_opts.inner_search)
              BuildFromNodeSubset(A1.members, G_A) → standalone G_A1
              inner_search → DecomposeAndSchedule(G_A1, level_2_opts)
                1. FormSubgraphs(G_A1) → declines (small) — leaf
                2. (no subgraphs)
                3. AddSubgraphOrderEdges(G_A1) (no-op)
                4. level_2_opts.outer_search(G_A1) → order over G_A1 nodes
              translate G_A1 order → G_A nodes; record A1.schedule_result
         3. AddSubgraphOrderEdges(G_A)
         4. level_1_opts.outer_search(G_A) → order over G_A nodes
       translate G_A order → G nodes; record A.schedule_result
  3. AddSubgraphOrderEdges(G)
  4. outer_opts.outer_search(G) → final order over G nodes
```

Formation fires on the way **down** (G, then G_A, then G_A1);
scheduling completes on the way **up** (G_A1's `outer_search` first,
then G_A's, then G's). `DecomposeAndSchedule(G_A1)` is the leaf —
`FormSubgraphs` declined, so step 2 has nothing to do and step 4 runs
`outer_search` directly. Each `ScheduleSubgraph` translates the order
it received into *its own* graph's node terms before recording it
(G_A1→G_A, then G_A→G), so `A.schedule_result.order` is already a
flat sequence of **G** nodes when step 3 of the outermost call
consumes it (§3.3).

---

## 4. Policy

Three things are configured per level of the tree:

- **Formation** — which passes run, splitter/size thresholds. Carried
  by `SubgraphFormationPolicy` (a runtime struct; exists).
- **Search algorithm** — BFS-DP, DFS, or branch-and-bound.
- **Search objective and tuning** — metric, seeding, time budget.

An **inner** (subgraph) search and the **outer** (region) search
differ deliberately. An inner search on a small graph may use a
*continuous* register-occupancy metric — integer occupancy ties every
schedule in the same bracket and gives the search no gradient, while
the continuous score distinguishes within-bracket pressure — and may
skip the input-order seed and run on a smaller budget. The outer
search uses the hardware-meaningful integer occupancy.

### 4.1 DecomposeAndScheduleOptions

The driver's configuration is one bundle:

```cpp
struct DecomposeAndScheduleOptions {
  SubgraphFormationPolicy                     formation;
  SubgraphScheduleMode                        mode;     // serialized | interleaved
  std::function<SearchResult(ScheduleGraph &)> inner_search;
  std::function<SearchResult(ScheduleGraph &)> outer_search;

  static DecomposeAndScheduleOptions BfsDpWithDfsFallback(
      const GCNSubtarget &, const MachineFunction &,
      const LiveIntervals &, int seed_occupancy);
};
```

Fields ordered by pipeline stage (formation/mode first, then the
two searches `DecomposeAndSchedule` invokes in step 2 and step 4).

`inner_search` is the per-subgraph behavior — a leaf search for flat
use, a closure around `DecomposeAndSchedule(sub, inner_opts)` for
recursive use. Per-level differences (outer vs. inner) are handled at
the closure boundary; the driver itself does not consult depth or a
provider. `outer_search` runs over the proxied + chained graph after
all subgraph interiors are locked.

Both searches are owning `std::function` rather than non-owning
`function_ref` so factory methods can build closures that outlive the
factory call. The closures still capture by reference — the captured
`st`, `mf`, `lis` outlive the options anyway, and reference captures
keep the closure inside std::function's small-buffer optimization.

`BfsDpWithDfsFallback` is the standard preset: BFS-DP first, DFS
fallback if BFS-DP returns no schedule. Both stages share a 5s budget;
inner uses the continuous occupancy score, outer uses the integer
occupancy score with `seed_occupancy` as the score-bound seed. DFS
fallbacks use a metric-matched policy (`DfsMaximizeOccupancyPolicy`
inner / `DfsMaximizeIntegerOccupancyPolicy` outer).

### 4.2 Compile-time policy is scoped to one place

`DfsSearch` is a template parameterized on a `SearchPolicy*` class
(see `SearchPolicies.h`): the search calls
`Policy::ShouldBoundSearch`, `Policy::FilterAndSortReadyList`, and
`Policy::ShouldEndSearch` from its inner `Recurse` loop, and
`if constexpr` gates on policy flags dead-strip pruning paths the
policy hasn't opted into. That earns its compile-time templating —
the calls are millions per region and the dead-stripping is real.

Everywhere else is runtime. The driver, the per-subgraph search
choice, `DecomposeAndScheduleOptions` — all runtime structs and
function objects (`std::function` for the options' searches,
`function_ref` for the lower-level `ScheduleSubgraph` parameter).
The seam is `ScheduleSubgraph`'s functor parameter
`function_ref<SearchResult(ScheduleGraph &)>`: above it, runtime;
factory methods like `DecomposeAndScheduleOptions::BfsDpWithDfsFallback`
build closures that internally pick `DfsSearch<…>` instantiations to
cross into the compile-time world. `LengthPolicyChoice` in
`ScheduleDAGHierarchicalScheduler.cpp` is the established precedent
for that runtime→compile-time switch.

So new knobs follow a simple placement rule:

- A new knob *called from `DfsSearch::Recurse`*: add it to a
  `SearchPolicy*` class.
- Anything else (new mode, new threshold, new budget, new flag the
  driver reads): a runtime field on the relevant struct.

### 4.3 BFS-DP and proxies

BFS-DP works on graphs with subgraph proxies: `PartitionDag` treats a
proxy as a single node, and `GetInputOrderIndex` collapses a proxy to
the smallest input-order index among its members (so the input-order
tiebreak still sorts correctly). It can therefore run as either the
inner search (extracted subgraphs, no proxies) or the outer search
(proxied + chained graph). The standard preset `BfsDpWithDfsFallback`
exercises both placements.

---

## 5. Schedule Enforcement: The Requirement

After step 2, `info.schedule_result.order` is the interior chosen for
the subgraph, as a sequence of `graph`'s member nodes. Step 3
schedules the whole proxied graph and must **honor** that interior —
otherwise the step-3 search re-derives it from scratch and the
isolated scheduling work is wasted.

There are two distinct requirements:

- **Order** — the subgraph's members appear in step 3's schedule in
  exactly `schedule_result.order`.
- **Contiguity** — the members are scheduled as one uninterrupted run
  (nothing else between them). Required in the default mode; dropped
  in interleaving mode (§8).

**Contiguity already exists.** `ScheduleConstructor` maintains a stack
of scheduling scopes: scheduling a start proxy pushes a scope,
scheduling an end proxy pops it, and the search only ever picks from
the top scope. So once the search enters a subgraph it cannot schedule
anything outside it until the subgraph is drained.

This document's enforcement question is therefore only about
**order**. §6 and §7 are two ways to enforce it.

---

## 6. Enforcement A: Order Edges (Selected)

After isolated scheduling, for each subgraph add artificial edges
along the chosen sequence:

```
order[0] → order[1] → order[2] → … → order[n-1]
```

— a Hamiltonian path through the members. The edges are
`kSubgraphOrderEdge`: **strong** (they constrain scheduling order) but
**not latency-contributing** (they do not inflate cycle counts and are
invisible to the length and frontier trackers).

### 6.1 How it enforces order

`ScheduleConstructor::ReleaseSuccessors` decrements each successor's
count of unscheduled strong predecessors and moves a successor to the
ready list when that count hits zero.

The order edge is one of `order[i+1]`'s strong predecessors, so
`order[i+1]` cannot be ready until `order[i]` is scheduled. And once
`order[i]` is scheduled, all of `order[i+1]`'s other predecessors are
already satisfied — its intra-subgraph predecessors lie within
`order[0..i]`, and its external predecessors were gated by the start
proxy. So `order[i+1]` becomes ready *exactly* when `order[i]` is
scheduled, and `order[i+2]` stays blocked behind it. At most one
member of the subgraph is ready at a time; the step-3 search has no
choice to make inside a subgraph.

### 6.2 What this costs — and does not

- **No `ScheduleConstructor` changes.** Scope push/pop already exists;
  `ReleaseSuccessors` already releases the next member when the order
  edge clears; the order edges are ordinary strong edges.
- **Backtracking is free.** `ReleaseSuccessors` / `UnreleaseSuccessors`
  already do/undo ordinary edges in lockstep with `Schedule` /
  `Unschedule`.
- **Locking is graph construction, not search logic.** A new
  `ScheduleGraph::AddSubgraphOrderEdges()`: for every `SubgraphInfo`
  with a populated `schedule_result`, add the chain, then re-derive
  the topological order and critical paths (the same tail as
  `InsertSubgraphProxies`).

The cost is ~O(members) extra edges per subgraph — the same order as
the proxy edges `InsertSubgraphProxies` already adds.

### 6.3 Correctness condition: convexity

The chain follows `schedule_result.order`, a topological order of the
subgraph's intra-member edges. It is cycle-free **iff** that order is
consistent with how the full graph orders those members — which holds
iff the subgraph is **convex**: no path between two members escapes
through a non-member.

Convexity is already guaranteed. In a non-convex subgraph some path
runs `member → … → member` by way of non-members; that path leaves the
member set through an *external successor* and re-enters through an
*external predecessor*. `InsertSubgraphProxies` wires
`end_proxy → (external successor)` and
`(external predecessor) → start_proxy`, so the escaping path closes a
cycle with the `start_proxy → … → end_proxy` path through the members.
`InsertSubgraphProxies` re-runs the topological sort, detects the
cycle, and `report_fatal_error`s. So any subgraph that successfully
receives proxies is convex, and its order chain is cycle-free.

### 6.4 Timing

`schedule_result.order` is known only *after* isolated scheduling, so
`AddSubgraphOrderEdges` runs as a step of `DecomposeAndSchedule` after step 2
— it is not part of formation-time `InsertSubgraphProxies`.

The chosen order is **compiled into the graph structure** and is
static for the lifetime of the step-3 search. §7.3 discusses when that
becomes a limitation.

---

## 7. Enforcement B: Stack-Based Cursor (Alternative)

This approach was designed and then **not** selected, in favor of §6.
It is recorded here because it is the cleaner substrate if scheduling
ever has to *choose* among multiple candidate subgraph schedules
(§7.3).

Instead of compiling the order into edges, keep `schedule_result.order`
as data and track a **cursor** — a per-subgraph `int` index meaning
"members of this subgraph scheduled so far." Drive the ready list from
the cursor.

### 7.1 Storage

The cursor lives on the scheduling scope. `SubgraphScheduleScope` gains
a `DenseMap<const ScheduleNode * /*start proxy*/, int>`. In the default
mode a subgraph scope holds one cursor; in interleaving mode (§8, no
subgraph scopes) the base scope holds all of them. Tying cursor
storage to the scope means a single decision — push a scope or not —
encodes the mode.

### 7.2 Mechanics

All in `ScheduleConstructor`, alongside the existing scope push/pop:

- **Start proxy scheduled** → push scope; cursor = 0; release
  `order[0]`.
- **Member scheduled** → `++cursor`; release `order[cursor]` if in
  range.
- **Member unscheduled** → un-release `order[cursor]` if in range;
  `--cursor`.
- **End proxy scheduled** → pop scope. **End proxy unscheduled** →
  re-push scope; cursor = `order.size()` (all members are scheduled at
  that point, so the value is determined — no stored history needed).
- **Start proxy unscheduled** → un-release `order[0]`, then pop.

A **guard** in `ReleaseSuccessors` / `UnreleaseSuccessors` keeps them
from touching the ready-list membership of locked-subgraph members —
the cursor owns it — while still maintaining those members'
predecessor counts so the end proxy (gated behind all members) still
fires. The guard is a symmetric `IsLockedSubgraphMember` check in both
functions.

The cursor is reversible search state: `++`/`--` mirror exactly, and
at the one point a scope is built from nothing (end-proxy unschedule)
the correct value is `order.size()`.

### 7.3 When the stack-based approach is preferable

Order edges compile **one** order into the graph. If a future step-3
search needs to **select** among several candidate schedules per
subgraph — e.g. isolated scheduling emits the top-K orders trading
length against register pressure, and the top-level search picks one
per subgraph based on surrounding context — order edges become
awkward: trying a different candidate means removing one edge set,
inserting another, and re-running the topological sort, *inside* the
search's do/undo loop. That is graph surgery on the hot path.

The cursor keeps the order as data. A subgraph can carry K candidate
sequences; "use candidate j" is the cursor reading sequence j — an
O(1) branch point, with no graph mutation and no topology re-derive.
Per-subgraph schedule selection becomes a first-class decision the
step-3 search can explore and backtrack over.

So the dividing line is *when* a subgraph's order is decided. Order
edges (§6) are correct and simplest when the order is fixed **before**
step 3. The stack-based cursor wins if the order must be chosen
**during** step 3.

### 7.4 Why §6 was chosen for now

In the current design each subgraph has exactly one chosen order,
fixed before step 3. Under that assumption order edges are strictly
simpler: no `ScheduleConstructor` changes, backtracking for free, and
interleaving mode (§8) for free — whereas the cursor needs new
hot-path logic, hand-written backtracking, and a "conjunction" upgrade
for interleaving (§8).

---

## 8. Interleaving Mode (Future)

The default mode requires a subgraph to be scheduled **contiguously**:
once entered, nothing else is scheduled until it is drained. A future
**interleaving mode** would let pre-scheduled subgraphs interleave their
members with each other and with non-subgraph nodes — only each
subgraph's *internal order* stays fixed.

**The mechanism is order edges with no proxies.** Form the subgraph and
lock its chosen order with the `AddSubgraphOrderEdges` chain
(`order[i] → order[i+1]`, §6), but do **not** insert proxies for it at
all. The members stay as ordinary nodes in the flat graph, keeping their
own real external edges; the chain forces the internal sequence while
nothing forces contiguity, so non-members fall between members freely.
That is the whole behavior — no scope push/pop (there is no proxy to push
a scope for), no edge rerouting.

This supersedes an earlier sketch (keep the proxies but relax their edges
and drop scopes). The proxies were doing two jobs: making the subgraph
*atomic/contiguous* — not wanted here — and serving as the attach point
for rerouted external edges — unnecessary, since the members already
carry their own. For interleaving they are pure overhead, so drop them
entirely rather than relax them.

**Soundness (acyclicity).** A chain edge `u → v` (u before v in the
isolated order) can only close a cycle through a path `v → … → u`. It
cannot stay inside the subgraph — the isolated order is a topological
order of the internal edges and the chain edges are forward — so it would
have to leave and re-enter: `v → external → … → external → u`. That means
some external chain is *both* a successor of member v and a predecessor of
member u, i.e. an external round-trip through the subgraph, which an
**acyclic quotient forbids**. Min-cut formation produces an acyclic
quotient by construction, so no chain edge can create a cycle.
`AddSubgraphOrderEdges` re-runs cycle detection after adding the chain, so
a violated assumption surfaces as a fatal graph cycle rather than
silently. (In the default mode the equivalent check rides inside
`InsertSubgraphProxies`, which interleaving skips; the order-edge cycle
check covers the same ground.)

**One structural prerequisite: ownership off the proxy.** A subgraph must
be able to exist without a proxy node. Today the start proxy *owns* its
`SubgraphInfo` (a `unique_ptr` in the node payload) and `subgraph_infos_`
holds raw pointers. Make `subgraph_infos_` itself the owner
(`vector<unique_ptr<SubgraphInfo>>`); both proxies then back-reference the
vector-owned info by raw pointer, and the owning payload alternative goes
away. The info's lifetime becomes graph-scoped and independent of any
proxy. This decouples ownership from proxies in the *default* path too,
and makes interleaving fall out: register the infos into the owning
vector (with the same `CheckNoNestedMembers` / `CheckMembersDisjoint`
checks), run `AddSubgraphOrderEdges`, and skip proxy emplacement.

**Enforcement is mode-independent (§6).** The order edges work unchanged:
a member's predecessor count ANDs its order edge with its real
data-dependency edges, so a member readies only when both its
locked-order predecessor and its real predecessors are scheduled —
exactly the gating interleaving needs. (The §7 cursor alternative would
have needed a "conjunction" upgrade — ready when cursor-reached **and**
real predecessor count zero — one more reason §6 was chosen.)

So: **interleaving is the default formation minus proxies — order edges on
the members, ownership on the `subgraph_infos_` vector.**

---

## 9. Status and Next Steps

**Implemented:**

- `FormSubgraphs` + `InsertSubgraphProxies` (formation; see
  `AMDGPUSubgraphFormationDesign.md`).
- `ScheduleGraph::BuildFromNodeSubset` — extract a member set into a
  standalone graph with a modeled register boundary.
- `ScheduleSubgraph` — schedule one subgraph in isolation and record
  the result on its `SubgraphInfo`.
- Scope push/pop in `ScheduleConstructor` (contiguity).
- `ScheduleGraph::AddSubgraphOrderEdges()` — enforcement approach A
  (§6), with shakedown.
- `DecomposeAndSchedule` driver and `DecomposeAndScheduleOptions`
  bundle (flat, no recursion yet), with end-to-end shakedowns for
  both hand-wired and factory-wired (`BfsDpWithDfsFallback`) options.

**Later:**

- Recursion (swap the leaf `inner_search` for one that recurses into
  `DecomposeAndSchedule`).
- Interleaving mode (§8).
