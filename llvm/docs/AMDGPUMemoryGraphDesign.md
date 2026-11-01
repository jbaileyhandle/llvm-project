# MemoryGraph: a memory-op view of a region for MLP search

2026-10-27. Draft design, nothing implemented yet. Pseudo-code is at the
level of a sketch, not final code.

## 0. Goal and reasoning

Maximize memory-level parallelism (MLP) without exceeding the register
pressure ceiling. The ceiling is known before any of this runs: it comes
from the occupancy tier being targeted.

**Why a separate graph.** The space of full schedules is far too large to
explore. The working theory is that a region's performance is mostly set
by how well its memory latency overlaps; the other instructions are short
enough that their order matters mainly through register pressure. So the
plan is to decide the memory part first, on a graph small enough to
reason about, and let the existing scheduler fill in everything else.

**The menu.** The MemoryGraph produces a list of candidate memory
orderings spanning the achievable MLP range, from the ideal down to fully
serialized, with several different candidates at each level. Candidates
are tried from the highest MLP down: each is imposed on the full graph,
the scheduler fills in the other instructions, and the first candidate
whose result fits under the ceiling wins. More overlap means more loaded
values live at once, so sliding down the menu is how register pressure
gets traded for MLP. The MemoryGraph does not model register pressure
itself; the full graph does that after the fill-in.

The steps:

1. compute the region's **ideal MLP** (unit latency);
2. **score** any valid ordering of the graph's nodes;
3. build the **menu** of candidate orderings;
4. **impose** a chosen candidate on the full graph.

Background on the MLP measures: AMDGPUMemoryLevelParallelismAnalysis.md
(section 2 measures a schedule, section 3 is the unit-latency stage
method).

## 1. Vocabulary

- **Original graph**: the region's ScheduleGraph.
- **Tracked load**: a load we count for MLP (section 2).
- **User of load L**: a node with a data edge (`ScheduleEdge::kData`) from
  L, meaning it reads a register L defines. Only *direct* users count.
- **Queue store**: a vector-memory store. It takes part in the vmcnt
  queue (section 2) but is not a load and is not counted in MLP.
- **Memory node**: a node of the MemoryGraph. The memory nodes are all
  tracked loads, all their users, and all queue stores. One instruction
  can play several roles, for example a load whose address comes from
  another load's data (pointer chasing).
- **Memory node index**: a memory node's position in the MemoryGraph's
  own arrays, 0 to n−1.
- **Wait queue**: the hardware counter a memory op is tracked by
  (section 2).
- **Strong edge**: an edge that enforces ordering. Cluster and weak hints
  (`IsWeakEdge()`) are not strong.
- **The walk**: the procedure in section 5.3 that replays an ordering
  and advances a clock only when a user has to wait for data.
- **Round**: one value of the walk's clock. Loads issued in the same
  round are in flight together.

## 2. Classifying memory ops and their wait queues

```
enum class WaitQueue { kVmem, kLgkm };
enum class MemClass  { kVmemLoad, kLdsLoad, kSmemLoad, kVmemStore };

struct MemoryGraphOptions {
  bool include_scalar = true;   // s_load_*, s_buffer_load_*
  bool include_lds    = true;   // ds_read_*
  bool include_stores = true;   // vector stores as queue stores
}

optional<MemClass> Classify(node, options):
  mi = node.instr
  if mi is null: return none
  if mi is a VMEM or FLAT-global load:      return kVmemLoad
  if mi is a VMEM or FLAT-global store:     return options.include_stores ? kVmemStore : none
  if mi is an SMEM load:                    return options.include_scalar ? kSmemLoad : none
  if mi is a DS read:                       return options.include_lds ? kLdsLoad : none
  return none        // true FLAT (may hit LDS): see open questions

WaitQueue QueueOf(mem_class):
  return mem_class in {kVmemLoad, kVmemStore} ? kVmem : kLgkm
```

How the counters behave on gfx906, from `SIInsertWaitcnts.cpp`
(`WaitEventMaskForInst`, `getVmemWaitEventType`, `counterOutOfOrder`)
and `AMDGPUBaseInfo.cpp` (bit widths):

- **vmcnt** counts every outstanding vector-memory op, loads *and*
  stores. gfx906 has no separate store counter (`hasVscnt()` is false).
  - A store increments vmcnt when it issues and decrements it when the
    write completes.
  - The counter is treated as decrementing in issue order. A wait is
    written "stall until at most N ops are outstanding". To make sure
    load B is done, N is set to the number of ops issued *after* B, so
    the wait also covers every op issued before B, stores included.
  - Nothing needs a store itself to finish. A store only delays a wait
    for a load issued after it, or a full drain (for example `vmcnt(0)`
    before a barrier or release fence). A store issued after the loads
    being waited on never blocks anything.
  - 6 bits on gfx9: at most 63 outstanding.
- **lgkmcnt** counts LDS, GDS, scalar memory and messages in one counter.
  - Scalar loads can return out of order. While any scalar load is
    pending, every lgkm wait is `lgkmcnt(0)`: drain the whole queue.
  - Mixing event types in the counter has the same effect.
  - **4 bits on gfx9: at most 15 outstanding.** Past that, the compiler
    waits for max−1 instead of the exact count (`getWaitCountMax`), so
    waits become conservative. This caps achievable LDS MLP in
    LDS-heavy regions and the model should apply the same cap.
- vmcnt and lgkmcnt are independent. A wait on one never covers ops in
  the other. Exception: a pending true-FLAT op (one that may hit LDS or
  global) forces both counters to 0 on gfx9
  (`hasFlatLgkmVMemCountInOrder()` is false).

## 3. Construction

```
class MemoryGraph {
  // One entry per memory node, indexed by memory node index.
  vector<ScheduleNode*>       original_node    // valid while the original graph lives
  vector<const MachineInstr*> original_instr   // stable across graph rebuilds
  vector<optional<MemClass>>  mem_class        // set for loads and queue stores
  vector<vector<int>>         loads_used       // loads this node directly reads
  vector<vector<Edge>>        successors       // Edge = {to, waits_for_data (0 or 1)}
  vector<vector<Edge>>        predecessors
  vector<int>                 topo_order
  int                         sink             // synthetic, index n
  DenseMap<const ScheduleNode*, int> memory_node_index   // original node -> index
}

MemoryGraph Build(original, options):
  original.ComputeTransitiveReductionAndReachability()   // strong edges only (section 9)

  // 1. Loads and queue stores.
  for node in original.topo_order:
    if c = Classify(node, options):
      AddMemoryNode(node).mem_class = c

  // 2. Direct users of loads.
  for each load L:
    for edge in original_node[L].successors:
      if edge.IsDataEdge() and edge.to is not the original exit node:
        U = memory_node_index.get(edge.to) or AddMemoryNode(edge.to)
        loads_used[U].push(L)

  // 3. Edges: one per reachable pair (transitive closure).
  for each pair of memory nodes (A, B), A != B:
    if original.IsReachableInDag(original_node[A], original_node[B]):
      AddEdge(A, B, waits_for_data = (IsLoad(A) and A in loads_used[B]) ? 1 : 0)

  // 4. Sink. Every load's data must return before the region is done.
  sink = AddSyntheticNode()
  for each memory node M:
    AddEdge(M, sink, waits_for_data = IsLoad(M) ? 1 : 0)

  topo_order = TopoSort()
```

**What an edge means.**

- **Reachable** → an edge with flag 0: B can't issue before A, but in the
  zero-issue-time model (section 5) it can issue at the same instant.
- **Direct data use** → flag 1: B must wait for A's data to come back.

Reachability alone does not mean waiting. If B is reachable from load A
only through ordering edges (for example A → store S → load B, kept in
order because the addresses might alias), B doesn't need A's data. Any
instruction that does need A's data, directly or not, sits after one of
A's direct users. Those users are memory nodes and carry the 1, so waits
still propagate along chains.

**"Load B depends on load A"** means B is reachable, through strong
edges, from one of A's direct users. Equivalently, some MemoryGraph path
from A to B contains a flag-1 edge.

**What the flag is for.** It is a dependence flag, not a latency. Ideal
MLP (section 4) uses it directly as a unit latency. Ordering evaluation
(section 5) takes each load's latency from the chosen latency mode and
uses the flag only to know that a wait is needed.

**Size.** Closure edges grow as n². That's fine at tens of memory nodes.
If reduction is ever needed, it must be weight-aware: a flag-1 edge may
only be dropped if some other path between the same nodes also contains
a flag-1 edge. Plain transitive reduction would drop A→B whenever any
other path exists and could silently remove a wait. Simplest rule: never
drop flag-1 edges.

## 4. Ideal MLP (always unit latency)

```
int IdealRounds():
  for node in topo_order:
    t[node] = max over predecessors p of (t[p] + waits_for_data(p -> node)),
              or 0 if node has no predecessors
  return t[sink]

double IdealMlp():
  return NumLoads() / IdealRounds()      // loads only; users and stores not counted
```

`IdealRounds()` is the number of loads on the longest chain of data
waits: the number of stages, D, in the MLP doc. Queue stores don't change
it, because under unit latency a store issued in a round finishes within
that round.

**It is an upper bound on what any ordering can score** in unit mode with
until-return intervals (section 5):

1. The walk's clock only advances when a user waits on a load issued in
   the current round, and then by exactly one. So every round has a load
   in flight, and busy time equals the total time T.
2. An ordering's MLP is therefore NumLoads ÷ T.
3. Every load issues no earlier than its longest flag path from the
   source, so T ≥ D.

The only unit-mode scores possible are NumLoads ÷ T for T = D … NumLoads.

**Two caveats.**

- **The bound is not always reachable.** "Loads as early as dependencies
  allow, users after" reaches T = D unless the lgkmcnt drain gets in the
  way: an ordering edge can force a scalar load S into the same round as,
  and before, a user U that waits on an earlier lgkm load. U's wait is
  then `lgkmcnt(0)`, which also waits for S and costs an extra round.
  `IdealRounds()` ignores this.
- **In unit mode the in-order queue cannot delay a return.** An op
  issued earlier always returns no later than a later op's issue + 1. So
  queue modeling only changes until-return scores when latencies differ
  (`kClass`, `kModel`). It still changes until-use scores in every mode,
  because it decides which loads a wait covers.

## 5. Scoring an ordering

### 5.1 Model

- The input is an ordering: every memory node, consistent with the edges,
  with the sink last.
- Issuing takes zero time, and any number of instructions can issue at
  once. Time advances only when a user must wait for data.
- Instructions that aren't memory nodes are ignored.

**Latency mode** (where each memory op's latency comes from):

| Mode | Latency |
|---|---|
| `kUnit` | 1 for every load and store |
| `kClass` | a nominal latency per MemClass, from measurement (VMEM ≫ LDS) |
| `kModel` | the op's modeled latency in the original graph |

`kClass` is the expected default. Exact latencies are unknown and vary
with cache hit rate, but the ratio between classes (VMEM roughly an order
of magnitude slower than LDS) is known and matters.

**Interval mode** (when a load stops counting as in flight):

| Mode | A load is in flight until… | Answers |
|---|---|---|
| `kUntilReturn` | its effective return (5.2) | how many loads overlap, if the latencies are right |
| `kUntilUse` | the first use that covers it (5.2) | how many loads the schedule leaves un-waited at once; the number that survives when real latency is longer than the estimate |

Under `kUntilUse` there is no "ideal" in the section 4 sense: a load whose
first use is far away keeps counting, so the score can exceed
NumLoads ÷ D. Its top is NumLoads (every first use at the end).

### 5.2 Wait queues

**Effective return time.** Within one queue, the counter reports op B
done only after every op issued before it:

  effective_return(B) = max(issue(B) + latency(B), effective_return(previous op in B's queue))

Waiting on B then automatically includes waiting on everything issued
before it in that queue, stores included.

**A use covers a load** if it waits on that load directly, or on a later
op in the same queue. While a scalar load is outstanding, any lgkm wait
covers every outstanding lgkm op.

### 5.3 Pseudo-code

```
struct OrderScore {
  map<MemClass, double> mlp     // per load class (5.4)
  double combined_mlp
  double length
}

OrderScore ScoreOrder(order, latency_mode, interval_mode):
  time = 0
  last_return = {kVmem: 0, kLgkm: 0}
  outstanding = {kVmem: [], kLgkm: []}     // ops issued and not yet covered, in issue order
  intervals = []                           // [start, end) and class, per load

  for node in order:
    // 1. Wait, if this node reads loaded data (a node can be both a user and a load).
    if loads_used[node] is not empty:
      wait_until = max over L in loads_used[node] of WaitTime(L)
      time = max(time, wait_until)
      for L in loads_used[node]:
        Cover(L, time)        // ends the covered loads' intervals in kUntilUse mode

    // 2. Issue, if this node is a load or a queue store.
    if mem_class[node]:
      q = QueueOf(mem_class[node])
      ret = max(time + Latency(node, latency_mode), last_return[q])
      last_return[q] = ret
      effective_return[node] = ret
      issue[node] = time
      outstanding[q].push(node)

  end_time = max(time, max of last_return)
  for each load L:
    end = (interval_mode == kUntilReturn) ? effective_return[L]
                                          : covered_at[L] if set, else end_time
    intervals.push({issue[L], end, mem_class[L]})

  return {mlp: SweepPerClass(intervals), combined_mlp: Sweep(intervals), length: end_time}

WaitTime(L):
  if QueueOf(L) == kLgkm and a scalar load is outstanding in kLgkm:
    return last_return[kLgkm]         // lgkmcnt(0): drain everything issued so far
  return effective_return[L]

Cover(L, t):
  // In-order queue: covering L also covers every op issued before it.
  if L is not in outstanding[QueueOf(L)]: return    // already covered by an earlier wait
  q = QueueOf(L)
  if q == kLgkm and a scalar load is outstanding:  pop all of outstanding[q]
  else:  pop outstanding[q] up to and including L
  for each popped op P:  covered_at[P] = t
```

The sweep is the one in the MLP doc (sections 2.1–2.3): average =
(total interval length) ÷ (time covered by at least one interval). Uses
at the same instant need no special handling, because a span of zero
length contributes nothing.

The `outstanding` queues are the push-on-issue / pop-on-use model.
Effective return times give the *time* a wait ends; the queues record
*which* loads that wait covers, for `kUntilUse`.

### 5.4 Per-class MLP

Report MLP separately for vector, LDS and scalar loads instead of one
combined number. "One vector load and three LDS loads in flight" is not
meaningfully 4: they are different resources, with different latencies
and different wait queues. For a class, the denominator is the time
during which at least one load *of that class* is in flight; the
combined number uses all loads and is reported alongside.

Vector latency is what dominates performance, so **vector MLP** is the
natural primary objective, with LDS and scalar MLP as secondary terms or
tiebreaks.

## 6. Building the menu

### 6.1 What the menu has to vary

Every ordering the walk can produce has the same shape: a sequence of
rounds, each one being "some users wait, then some loads and stores
issue". So two things describe an ordering for MLP purposes:

1. **How many loads are issued before each wait.** This is the overlap,
   and it is also what drives register pressure: every load in flight
   holds its destination registers.
2. **Which loads those are.** Two orderings with the same overlap can
   hold different values live, which matters once the full graph is
   filled in.

The menu therefore needs one knob that moves (1) smoothly from "all
loads before any wait" down to "one load per wait", and variation in (2)
at each knob setting.

### 6.2 Backbone: windowed list scheduling

A list scheduler over the MemoryGraph with a **window W**: the most loads
allowed in flight at once. W is the knob. W = ∞ gives the ideal ordering
(loads as early as dependencies allow); W = 1 gives full serialization.
Every integer in between is a rung of the menu.

```
Ordering WindowedSchedule(W, priority):
  state: clock, outstanding queues, ready list, in_flight = loads issued and not yet covered
  order = []
  while nodes remain:
    // Users whose data is already back cost nothing and free registers: take them first.
    if some ready user has every load it reads already returned (WaitTime <= clock):
      take it
    else if in_flight < W and some ready load or store exists:
      take the one `priority` ranks highest
    else if some ready user exists:
      take the ready user whose wait ends earliest     // the cheapest clock advance
    else:
      // Only loads are ready, the window is full, and no user can run: the in-flight
      // loads' users all need a load that hasn't issued. The window is soft here.
      take the highest-priority ready load
    append it to order and update the walk state
  return order + [sink]
```

Why it fits the goal:

- **W is the pressure slider.** W loads in flight means W destination
  tuples live, so each rung down the menu frees registers in a
  predictable way. The fill-in step still measures real pressure; W only
  orders the candidates.
- **Cheap.** One pass per candidate, so dozens of candidates per region
  cost nothing next to a scheduler run.
- **It never delays a user whose data is back.** Such a user costs no
  time under the model and only extends a live range if delayed.
- **Per-class windows** (W_vmem, W_lds) are a natural extension: vector
  loads are the expensive overlap and LDS loads are cheap in latency but
  still cost registers.

Variation at a fixed W comes from `priority`, which decides which ready
load to issue first:

- longest flag path to the sink first (critical loads early);
- program order;
- fewest destination registers first;
- randomized with a seed.

Different priorities put different loads in flight together, which is
the variation the fill-in step needs. Duplicate orderings are dropped.

Each candidate is scored with `ScoreOrder` under the chosen modes and the
menu is sorted by the primary objective (5.4). The W used to make a
candidate is recorded with it.

**A variant for until-use.** The "take returned users first" rule ends
each load's until-use interval as early as the model allows. If the goal
is to leave loads un-waited for longer than the estimated latency, add a
slack S: hold a user until clock ≥ its wait time + S. S = 0 is the rule
above.

### 6.3 Checking the backbone: exhaustive search on small regions

```
vector<Candidate> EnumerateOrders(latency_mode, interval_mode, max_orders):
  DFS over the ready list, recording ScoreOrder for every complete ordering,
  pruned by the first-use rule below; stop at max_orders.
```

Its job is validation: on regions small enough to enumerate, compare the
best exhaustive score at each number of rounds against what the windowed
scheduler found at the same W. That measures how much the heuristic
loses before anyone depends on it.

**First-use rule** (a pruning heuristic). For two loads in the same
queue, where A was issued before B: A's first use must not come after
B's first use. The reasoning: waiting on B also waits on A, so A's data
is ready by then, and moving A's first use up to just before B's costs
nothing *if the moved user's predecessors are all already in place*.
When they aren't, moving them too can change the queue order of other
loads, so the rule can discard orderings that are genuinely different.
It is kept because it collapses user permutations that only reorder
simultaneous uses. If the rule leaves no valid ordering at all (A forced
before B by an ordering chain while A's users need B's data), waive it
for that pair.

**Size.** With n independent loads in one queue, each with one user, the
rule leaves n! load orders × Cₙ interleavings of uses, where Cₙ is the
Catalan number ((2n)! / ((n+1)! · n!)): the number of ways to interleave
n issues and n uses, each sequence in a fixed order, so that every use
comes after its own issue. For n = 8 that is 40,320 × 1,430 ≈ 58
million. A cap is mandatory, and this is why the search is a check, not
the backbone.

### 6.4 If the backbone proves too coarse: dominance-pruned search

If exhaustive comparison shows the windowed scheduler missing good
candidates, the middle ground is a search over partial orderings with
dominance, in the pattern of the existing PartitionDag / history
trackers: the state is (set of scheduled memory nodes, clock, queue
contents), and among partial orderings with the same state only the best
is kept. Target one number of rounds T at a time, pruning prefixes that
can no longer finish in T rounds (longest remaining flag path from any
unissued load exceeds T − clock). Don't build this until the comparison
says it's needed.

### 6.5 From rounds to an ordering (for round-based generators)

If a generator produces round numbers instead of an ordering (section
6.4, section 6.6, or any sampler over round assignments), two rules make
the walk reproduce those rounds:

- **Every round must be closed by a wait**: for every round k before the
  last, some user in round k+1 directly uses a load issued in round k.
  Without this the walk's clock never reaches k+1 and the rounds after it
  collapse, so the ordering's real MLP is higher than the label says.
- **Within a round, users go before loads and stores** unless an edge
  forces otherwise, so no user waits on an op from its own round (this is
  what trips the lgkmcnt drain). Remaining ties break by topological
  index.

### 6.6 Alternative generator: assign rounds directly

The stage method in the MLP doc (its sections 3 and 4.2) suggests a
different generator. Instead of ordering ops one at a time, give each
load a round number and let section 6.5 turn the assignment into an
ordering. The number of rounds, T, is the knob.

Each load v has a legal range of rounds:

- **earliest(v)**: 1 if v waits on no other load; otherwise one more
  than the highest round among the loads it waits on. This is v's stage
  from the stage method.
- **latest(v)**: T minus the number of loads on the longest wait chain
  that starts at v (counting v), plus 1. Any later and the chain after v
  would not fit in T rounds.

Ordering edges (flag 0) narrow the range further: a load can't be in an
earlier round than an op it must follow. Each user goes in the round
after the latest round among the loads it reads, and a queue store goes
in any round at or after its predecessors'.

The menu then varies two things:

1. **T**, from `IdealRounds()` (every load as early as possible) up to
   `NumLoads()` (one load per round). Fewer rounds means more overlap.
2. **The placement within each range** at a fixed T. Loads with slack
   (earliest < latest) can be pushed later to spread the in-flight set
   out, or pulled earlier to bunch it. The MLP doc's example is a chain
   of four loads plus four independent ones: with T = 4, the independent
   loads can all sit in round 1, giving round sizes (5, 1, 1, 1), or one
   per round, giving (2, 2, 2, 2). Same T, same average MLP, very
   different peak pressure.

That last point is why this is not the backbone. T counts rounds, not
loads in flight, so a rung of this menu does not move register pressure
predictably the way a rung of W does (6.2). It is the natural generator
when the goal is to explore *shapes* at a fixed MLP, and it is the output
format of the dominance search in 6.4. Build it if the exhaustive
comparison (6.3) shows the windowed scheduler missing good candidates
that differ in shape rather than in W.

## 7. Imposing a candidate on the full graph

Only two things in a candidate affect its timeline:

- the issue order of loads and stores (it fixes the queues);
- for each user, which loads are issued before it and which after.

The relative order of users among themselves does not matter to MLP, and
fixing it would only take freedom away from the fill-in scheduler. So
impose exactly the two facts above:

```
void ApplyOrder(order, target_graph):
  // target_graph: a build of the same region from the SAME instruction order
  // as the graph the MemoryGraph was built from.
  previous_op = null
  for memory node m in order (without the sink):
    node = target_graph.NodeFor(original_instr[m])     // MachineInstr -> SUnit -> node
    if IsLoad(m) or IsStore(m):
      if previous_op: AddEdge(previous_op, node)        // issue order
      for each user u placed before m in order: AddEdge(NodeFor(u), node)
      previous_op = node
    else:                                               // a pure user
      if previous_op: AddEdge(previous_op, node)        // issued-before set
  // AddEdge = target_graph.AddEdge(from, to, kMemoryOrderEdge, latency 0)
```

- **Same instruction order.** `buildSchedGraph` directs may-alias memory
  edges from the earlier instruction to the later one *in the current
  order*. If a pass reorders the region between the MemoryGraph build and
  the target build, those edges flip and the added edges can close a
  cycle. Build both from the same order, or re-check reachability before
  each added edge.
- **Rebuilds, not copies.** ScheduleGraph can't be copied, and the
  scheduler rebuilds it for every pass (`WithRegionGraph`). `ScheduleNode*`
  and `SUnit*` are invalid after a rebuild, so the lookup goes through
  `original_instr`, which survives rebuilds.
- **Edge kind.** A strong edge that adds no latency. `kSubgraphOrderEdge`
  already has those semantics, but a dedicated kind keeps dumps and
  debugging clear.
- **Redundant edges** (a user→load edge already implied by the chain)
  are harmless; the existing transitive reduction removes them.

After the edges are added, the existing scheduler fills in the other
instructions, and the result's register pressure is checked against the
ceiling. If it fails, the next candidate down the menu is tried.

## 8. Standalone class, not a subclass of ScheduleGraph

| | Standalone | Subclass / reuse ScheduleGraph |
|---|---|---|
| Nodes and edges | Integer indices, plain vectors | `ScheduleNode` wraps an SUnit or a proxy; `ScheduleEdge` kinds and latencies mean something else |
| Construction | Small | `BuildFromSUnits` / `BuildFromNodeSubset` copy only *direct* edges, so a closure variant would be needed |
| Register state | None needed | A `NodeRegInfoTable` and a live-in/live-out boundary are required but meaningless for a subset |
| Search | Windowed list scheduler plus a small DFS (section 6) | The existing searches drive `ScheduleConstructor`, which always carries `GCNRegisterTracker` and optimizes length/pressure `Score`s, not MLP |
| Class design | Free | No virtual methods; factories return `unique_ptr<ScheduleGraph>`; non-copyable and non-movable |

Register pressure is judged on the full graph after `ApplyOrder`, with
the existing machinery, so the MemoryGraph doesn't need it.

## 9. Prerequisite: strong-edge-only reachability (done)

Done in commit `6a4a37ec6eff`.
`ScheduleGraph::ComputeTransitiveReductionAndReachability` now skips weak
edges (`kCluster` / `kWeak`, the ordering hints `BuildFromSUnits` copies
in), so reachability, the reduced graph, and the dominator tree built on
top of them reflect strong edges only. A shakedown adds weak edges to the
test DAG and checks that they change neither reachability nor the reduced
graph.

No effect on today's schedules is expected: cluster edges are added by
DAG mutations, which run only in `postprocessDAG()`, and `WithRegionGraph`
calls only `buildSchedGraph()`.

## 10. Open questions

1. **True FLAT ops** count against both counters and force full waits
   on gfx9. Classify as VMEM and accept the error, or model both queues?
2. **Hardware ordering.** The compiler treats vmcnt as decrementing in
   issue order for loads and stores together, and lgkm same-type ops in
   order except scalar reads. This matches the Vega ISA's data-dependency
   section as remembered; verify against the document.
3. **Weak edges from `buildSchedGraph()`.** Believed not to happen;
   unverified.
4. **`kClass` latency numbers** per class. The measured-latency feedback
   infrastructure (per-kernel vmem/smem/lds latencies fed into misched)
   is the natural source if it is still in place.
5. **Combining per-class MLP into one objective:** vector primary, LDS
   and scalar as tiebreaks, or a weighted sum?
6. **Region end in `kUntilUse` mode.** A load with no use in the region
   currently ends at the region's end time. Its value is consumed by a
   later region, so this undercounts its window.
7. **Deadlock rule in the windowed scheduler** (6.2, last branch): issue
   the highest-priority load, or the one whose issue unblocks the most
   users?
8. **Non-first users.** Their position never affects MLP but does affect
   live ranges. The backbone places them as early as possible; is that
   always what the fill-in step wants?

## 11. Plan, and cheap probes before building the menu

### 11.1 First steps (planned)

1. **The MemoryGraph itself** (section 3), with the two helpers:
   `IdealMlp()` (section 4) and `ScoreOrder()` (section 5).
2. **An MLP analysis pass**, modeled on `ScheduleDAGLengthAnalyzer`:
   runs after the last real scheduler, rebuilds each region's graph
   from the final instruction order, builds the MemoryGraph, and reports
   ideal MLP and the observed schedule's MLP (per class, both interval
   modes). Analysis only; it never reorders.
3. **A sweep** over the shortlist kernels, build-only with the analysis
   enabled, reporting ideal and observed MLP at the default occupancy
   and at occupancy − 1.

Step 3 is probe 11.2.1 below. What it measures: whether the register
ceiling is what limits MaxOccupancy's achieved MLP. If observed MLP barely
moves when the budget grows, the ceiling isn't binding and the menu
can't help; if it jumps, the ceiling is binding and the question becomes
how much of that MLP could fit under the original ceiling with better
choices.

### 11.2 Probes that estimate headroom without building the menu

Comparing ideal to observed MLP shows the gap to the *unconstrained*
ideal. It doesn't say how much of that gap can be closed under the
register ceiling. These probes get at that question cheaply.

1. **Relax the ceiling.** Pin occupancy one or two tiers lower and
   measure observed MLP, pressure and runtime. Interpretation as in
   11.1.
2. **Decompose the pressure peak.** In MaxOccupancy's final schedule,
   split the live registers at the peak into values defined by loads
   (issued, last use not yet reached) and everything else (addresses,
   accumulators, ALU temporaries, tile state). The register tracker in
   the analyzer already has the live set; the split is a per-register
   "defined by a load" lookup. The budget any memory ordering can use is
   roughly (ceiling − other state), so the achievable loads-in-flight
   cap is that divided by the average load width. Compare the cap to the
   observed in-flight count. It is an estimate, not a bound: more
   overlap also lengthens address and index live ranges.
3. **A manual menu with `sched_group_barrier`.**
   `__builtin_amdgcn_sched_group_barrier(mask, size, sync_id)` tells the
   scheduler to place groups of N instructions of a class (VMEM loads,
   VALU, …) in a given order. It works through a DAG mutation
   (`createIGroupLPDAGMutation`, already registered in the target
   machine), so the default MaxOccupancy pipeline honors it with no
   compiler changes. For two or three shortlist kernels, write a few
   hand-chosen memory orderings (all loads first; two batches;
   interleaved), compile normally, and compare pressure, occupancy,
   spills and runtime. This is the menu built by hand, judged by the
   real register allocator and real hardware. If no hand ordering beats
   MaxOccupancy under the same ceiling, the automated version won't
   either; if one does, it shows what kind of ordering wins. Forced
   groups ignore pressure, so some orderings will drop a tier or spill;
   that is a result. Only the first scheduling pass sees the mutation,
   so run with the hierarchical scheduler off.
4. **Existing search results as evidence.** The hierarchical
   scheduler's length pass already searches for minimum
   latency-exposed length under a register budget, which is close to
   maximum MLP under the ceiling. Where that search ran to completion
   and found nothing better than the plain pipeline, that is a direct
   statement about headroom on those regions, with two possible
   readings: limited search (timeouts, region size, latency-model
   error) or little real headroom. Probes 1–3 don't depend on the
   latency model, so they separate the two.

### 11.3 A mechanism worth checking alongside

Spill reloads are vector loads, so a reload's `s_waitcnt vmcnt(N)` waits
in issue order for every vector-memory op issued before it. A reload
placed mid-region would therefore drain the whole queue and collapse the
MLP of the surrounding loads, which would explain why even light
spilling costs far more than the reload's own latency. This is a
hypothesis. The `s_waitcnt` values before reloads in a spilling kernel's
ISA would confirm or kill it, and the MLP analysis pass (11.1 step 2)
would show it directly as a drop in observed MLP around the reload.
