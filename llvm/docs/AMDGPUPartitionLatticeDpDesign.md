# HierarchicalScheduler: Partition-Lattice DP for Min-Pressure Scheduling Design

## Purpose of this Document

This doc specifies a dynamic-programming-based scheduler for minimum
register pressure, intended as a drop-in alternative to
`DfsMaximizeOccupancyPolicy`. The state of the search is the **set
of already-scheduled instructions** (a "partition" in the existing
HierarchicalScheduler vocabulary). The algorithm processes the
lattice of valid partitions level by level (BFS order), computes
the optimal min-peak-pressure schedule by a minimax-path
recursion, and recovers the schedule via backtrace.

**Prior-art disclosure up front.** The algorithm is **not novel**.
It is described in:

> Christoph Kessler. *Scheduling Expression DAGs for Minimal
> Register Need.* Computer Languages 24(1):33–53, September 1998.

The Castañeda Lozano & Schulte 2019 combinatorial-scheduling
survey describes Kessler's "ncv" algorithm in language that
matches this proposal sentence by sentence (§A.1, classified in
Table 5 alongside Govindarajan 2003 IP, Malik 2008 CP, and
Shobaki et al. 2013 OptSched). Reported optimal scalability for
Kessler 1998: basic blocks up to ~51 instructions with register
need up to 13. Multi-objective extension (length + RP):
~25 instructions.

**AMDGPU-specific competitor.** Shobaki et al. CGO 2024,
*Instruction Scheduling for the GPU on the GPU*, co-authored with
Austin Kerbow at AMD, reports up to 74% execution-speed
improvement over AMD's production scheduler on rocPRIM using
GPU-parallel Ant Colony Optimization for register-pressure-aware
scheduling in LLVM. APRP cost function tied to occupancy bins.
Direct overlap with the niche this proposal would target.

So this doc proceeds with eyes open: the algorithm is established
(Kessler 1998), the AMDGPU+LLVM precise-scheduling space has a
recent occupant (Shobaki et al. 2024), and the contribution — if
any — is engineering: applying Kessler-DP within the
HierarchicalScheduler's subgraph-decomposition framework, with
empirical comparison against the production heuristic, OptSched,
and Shobaki's ACO. §10 takes this up explicitly.

The companion doc `AMDGPUHistoryDominationDesign.md` covers the
existing DFS+history-tracker scheme (Phase 4b shipped). The
current doc proposes a separate, parallel search infrastructure —
nothing in §11 of the History Domination retrospective is
contradicted; the BFS-DP avoids the postfix-tracking soundness
pitfalls by construction.

## Table of Contents

1. [Context and Prerequisites](#1-context-and-prerequisites)
2. [Terminology](#2-terminology)
3. [The Algorithm](#3-the-algorithm)
4. [Soundness](#4-soundness)
5. [Architecture and Components](#5-architecture-and-components)
6. [Lightweight Alternatives to Schedule-Constructor Copies](#6-lightweight-alternatives-to-schedule-constructor-copies)
7. [Concerns and Mitigations](#7-concerns-and-mitigations)
8. [Prior Art and Novelty Assessment](#8-prior-art-and-novelty-assessment)
9. [Open Questions](#9-open-questions)
10. [Implementation Phases](#10-implementation-phases)
11. [Citations](#11-citations)

---

## 1. Context and Prerequisites

### 1.1 Problem statement

Given a `ScheduleGraph` for a region (after `BuildFromSUnits`,
topological ordering, transitive reduction, and optionally
subgraph formation), produce a schedule that minimizes peak
register pressure under the metric used by
`DfsMaximizeOccupancyPolicy::kMetric` (continuous register
occupancy score). The scheduler must integrate with the rest of
the HierarchicalScheduler pipeline: same input types, same output
type (a `ScheduleConstructor` carrying the chosen schedule), same
fallback contract when a budget is exceeded.

The motivation is two-fold:

1. **Optimality without DFS pathologies.** The existing
   DFS-with-history scheme (`DfsSearch` + `PressureHistoryTracker`
   Phase 4b) does prefix-dominance pruning but cannot do
   total-bound (case-3) pruning soundly without postfix tracking,
   which §11 of `AMDGPUHistoryDominationDesign.md` documents as
   architecturally fragile in the DFS framing. BFS-DP processes
   each partition once, after all its predecessors are finalized,
   removing the interleaving issues that broke postfix soundness.

2. **A natural fit for subgraph decomposition.** Subgraph formation
   already breaks regions into smaller chunks. Per-subgraph
   partition counts are bounded; per-subgraph BFS-DP is feasible
   in regimes where Kessler 1998 alone would not be (he reported
   up to ~51 instructions; subgraphs in our pipeline are smaller).

### 1.2 What already exists

The relevant infrastructure is in
`llvm/lib/Target/AMDGPU/HierarchicalScheduler/`:

- `ScheduleGraph` and `ScheduleConstructor` — graph and partial-
  schedule abstractions. `Schedule(node)` and `Unschedule()` are
  LIFO and reversible; pressure and length trackers are
  notification-driven and stay consistent.
- `ScheduledSetTracker` — maintains an XOR signature, scheduled
  bitset, and frontier of unscheduled real nodes with at least
  one scheduled latency-bearing predecessor. `PartitionKey` and
  `PartitionKeyView` are the DenseMap-friendly identities of a
  partition; full `DenseMapInfo<PartitionKey>` specialization is
  in `ScheduledSetTracker.h`.
- `GCNRegisterTracker` — incremental forward register tracker.
  Supports `GetMetricScore(ScheduleMetric)` for the occupancy
  score and (via the test mode) synthetic delta-based pressure.
  Currently does **not** support a from-scratch
  `ComputeFromScratch(BitVector)` query; this would need adding
  (see §5.5).
- `DfsSearch<Policy>` — the existing depth-first enumerator. The
  BFS-DP scheduler is a sibling of this, not a refactor.
- Subgraph formation (`SubgraphFormation.cpp`,
  `SubgraphInfo.{h,cpp}`) — produces a flat two-level set of
  subgraphs, ready to be consumed by either DFS or BFS-DP.
- `PressureHistoryTracker` — the table behind DFS prefix-
  dominance pruning. Not used by BFS-DP, but the
  `PartitionKey`-as-key pattern carries over.

### 1.3 What this doc adds

- A new `BfsDpSearch` class, parallel to `DfsSearch`, that drives
  BFS-by-levels enumeration of the partition lattice with DP
  table updates and backtrace reconstruction.
- A `BfsDpMaximizeOccupancyPolicy` (or an equivalent
  configuration parameter), exposing the score metric and budget
  knobs.
- A `LatticeNode` per-partition record holding `f` (min peak
  pressure to reach this partition), `level`, and a backtrace
  pointer plus the transition instruction.
- An optional `ComputeFromScratch(BitVector)` extension to
  `GCNRegisterTracker` (independent utility; see §5.5 for when
  it is required).
- Outer-loop integration so existing length-target outer loops
  (e.g., `ScheduleRegionForMinimumLength`) can drive BFS-DP the
  same way they drive `DfsSearch::ResetForReuse(target_length)`,
  if the multi-objective extension is in scope.

### 1.4 Relationship to existing DFS-based search

`DfsSearch` and `BfsDpSearch` are **alternatives**, not layered.
Both are gated by policy choice; the production pipeline picks
one per (sub)region based on size and an optional fallback rule.
A typical configuration:

- BFS-DP for subgraphs whose partition count is bounded
  (estimated up front from the DAG width — see §7.1).
- DFS+history for everything else, including any subgraph where
  BFS-DP exceeds its memory budget mid-run.

This avoids the all-or-nothing collapse that BFS-DP would
otherwise have on its first overflow: budget exceeded → discard
partial table → fall back to the existing DFS result for that
(sub)region.

---

## 2. Terminology

- **Partition** — same as in `AMDGPUHistoryDominationDesign.md`:
  the bipartition of graph nodes induced by a partial schedule.
  Identified by `PartitionKey` (signature + scheduled bitset).
- **Lattice** — the DAG over partitions where the empty partition
  is the root, the full-set partition is the unique sink, and
  edges go from `P` to `P ∪ {x}` for each ready instruction `x`.
  Nodes are downsets of the data-flow DAG.
- **Ready set at P** — instructions `x ∉ P` whose DAG predecessors
  are all in `P`. Computable from `P` and the graph; matches the
  ready-list contract of `ScheduleConstructor`.
- **Cross-section pressure of P** — count of registers defined in
  `P` and used by some instruction in `Pᶜ`. Function of the
  partition only, independent of order; same notion as "boundary
  live set" in the History Domination doc, §2.
- **Edge peak** — for the lattice edge `P → Q = P ∪ {x}`, the
  maximum register pressure observed during scheduling of `x`
  given live set determined by `P`. May exceed both
  cross-section pressures of `P` and `Q` due to defs landing
  before last-uses kill (see §4.4). Function of `(P, x)`.
- **f(P)** — DP value: the minimum, over all valid schedules of
  the instructions in `P`, of the maximum register pressure
  observed during scheduling. The recursion is in §3.2.
- **Backtrace** — the chain of best-predecessor edges from
  `f(full set)` back to `f(empty)`, which yields the optimal
  schedule.
- **Cursor walk** — a traversal of the partition lattice in which
  a single `ScheduleConstructor` is rolled forward and backward
  by `Schedule`/`Unschedule` to visit each partition in turn.
  Avoids per-partition constructor copies. See §6.1.

---

## 3. The Algorithm

### 3.1 State and transitions

The DP state is a partition `P` (a `BitVector` over instructions,
or equivalently a `PartitionKey`). For each partition we store:

- `f(P)` — the minimum-over-schedules-of-`P` of the peak pressure
  observed.
- `back(P)` — pointer to the lattice predecessor (a partition
  `Q ⊂ P` with `|Q| = |P|−1`) on the optimal path to `P`.
- `instr(P)` — the instruction `x ∈ P` such that
  `Q = P ∖ {x}` and the optimal path goes `Q → P` via `x`.

Initialization:

- `f(∅) = pressure(∅)` — the live-in count, which the existing
  `GCNRegisterTracker` reports as the initial pressure before any
  Schedule call.
- `back(∅) = nullptr`, `instr(∅) = nullptr`.

Transitions: from a partition `P`, for each ready instruction
`x`, build `Q = P ∪ {x}`. Compute `edge_peak(P, x)` (see §4.4),
form a candidate value `max(f(P), edge_peak(P, x))`, and merge
into the existing entry for `Q` if it improves `f(Q)`.

### 3.2 The DP recursion

Per §4 (soundness), the recursion is a **minimax path** problem
on the partition lattice with edge weights = `edge_peak(P, x)`:

```
f(Q) = min over (P, x) with P ∪ {x} = Q of
            max(f(P), edge_peak(P, x))
```

If we restrict ourselves to per-node (rather than per-edge)
pressure — i.e., we record only the post-instruction
cross-section pressure and ignore transient bumps within an
instruction — the recursion simplifies to:

```
f(Q) = max(pressure(Q), min over predecessors P of f(Q) of f(P))
```

The simplified form **under-counts** the true peak by up to
`|last_uses(x)|` for the entering instruction (§4.4). For
correctness against LLVM's `RegisterPressureTracker` semantics we
use the per-edge form; the per-node form is documented for clarity
and may be used in shakedowns where the comparison schedule also
ignores transients.

### 3.3 Schedule reconstruction

Once `f(full set)` is finalized, the optimal schedule is
recovered by walking `back` backward:

```
node = full_set_partition
reverse_schedule = []
while back(node):
    reverse_schedule.push(instr(node))
    node = back(node)
return reverse(reverse_schedule)
```

The result is a sequence of instructions in scheduling order. To
hand it to the rest of the pipeline, we replay it through a
fresh `ScheduleConstructor`, calling `Schedule(node)` for each in
turn. This replay also runs the existing
`GCNRegisterTracker` and `ScheduleLengthTracker` updates, so the
returned constructor matches the contract of
`DfsSearch::Run()`'s return.

### 3.4 Worked example — diamond DAG

Consider the diamond DAG `A → B`, `A → C`, `B → D`, `C → D`,
where `A` defines `vA` (used by `B` and `C`), `B` defines `vB`
(used by `D`), `C` defines `vC` (used by `D`), and `D` defines
`vD` (live-out).

The partition lattice has six nodes:

```
    {}
    └── {A}
        ├── {A, B}
        │   └── {A, B, C}
        │       └── {A, B, C, D}
        └── {A, C}
            └── {A, B, C}   (same as above; merges)
```

Cross-section pressures (post-instruction):

- `pressure({}) = 0` (no live-in for this example).
- `pressure({A}) = 1` (vA live, used by B and C).
- `pressure({A, B}) = 2` (vA live for C; vB live for D).
- `pressure({A, C}) = 2` (vA live for B; vC live for D).
- `pressure({A, B, C}) = 2` (vB live for D; vC live for D).
- `pressure({A, B, C, D}) = 1` (vD live-out).

DP values, processed by level:

- `f({}) = 0`.
- `f({A}) = max(0, pressure({A})) = 1`. Back: `{}`.
- `f({A, B}) = max(1, 2) = 2`. Back: `{A}` via `B`.
- `f({A, C}) = max(1, 2) = 2`. Back: `{A}` via `C`.
- `f({A, B, C})`: two predecessors. From `{A, B}` via `C`:
  `max(2, 2) = 2`. From `{A, C}` via `B`: `max(2, 2) = 2`. Both
  candidates equal; `f = 2`. Back: either (say `{A, B}` via `C`).
- `f({A, B, C, D}) = max(2, 1) = 2`. Back: `{A, B, C}` via `D`.

Optimal peak pressure: `2`. Schedule recovered by backtrace
(reading instructions in reverse from `{A,B,C,D}` to `{}`):
`D, C, B, A` reversed = `A, B, C, D` (or `A, C, B, D`,
depending on the tie-break at `{A, B, C}`).

This matches the obvious answer for a diamond DAG and confirms
the recursion shape on a tractable case.

---

## 4. Soundness

### 4.1 Order-within-prefix doesn't matter

For any partition `P`, the future of the search depends only on:

- **Live registers crossing the cut.** A register `r` is live
  iff `def(r) ∈ P` (or `r` is live-in) and at least one user of
  `r` is in `Pᶜ`. Function of `P` alone.
- **Ready instructions.** `x ready` iff `x ∉ P` and
  predecessors of `x` are all in `P`. Function of `P` alone.
- **Per-register remaining-use counts.** For each live `r`,
  the count of users still in `Pᶜ`. Function of `P` alone.

Different orderings that yield the same `P` are interchangeable
for everything downstream. This is the same observation the
History Domination doc relies on for prefix-dominance soundness;
nothing new here.

### 4.2 Minimax path on the partition lattice

For any complete schedule `σ = x₁, x₂, …, xₙ`, define
`Pₖ(σ) = {x₁, …, xₖ}`. The peak pressure of `σ` is

```
peak(σ) = max over k of edge_peak(Pₖ₋₁, xₖ)
```

The set `{P₀, P₁, …, Pₙ}` is a chain in the partition lattice
from `∅` to the full set, and any chain in the lattice
corresponds to a valid schedule (since each step is a ready
instruction). So:

```
min over σ of peak(σ) = min over chains C from ∅ to full of
                            (max over edge in C of edge_peak)
```

This is the **minimax path** problem on the lattice (a.k.a.
bottleneck shortest path; widest path under sign flip). On a
DAG, the standard exact algorithm is topological-order DP, which
is what §3 implements.

The optimal-substructure argument: for any partition `Q`, the
optimal cost to reach `Q` from `∅` is

```
f(Q) = min over (P, x) with P ∪ {x} = Q of
            max(f(P), edge_peak(P, x))
```

because (a) any chain to `Q` ends in some edge `(P, x) → Q`,
(b) the cost of that chain is the max of its prefix's cost and
the entering edge, and (c) `f(P)` is already the optimal
prefix-cost by induction. Standard Bellman-style argument; see
e.g. CLRS exercises on bottleneck paths or the directed-pathwidth
literature (Tamaki 2011 §3, where the same recursion underlies
the parameterized algorithm).

### 4.3 Multi-dimensional pressure

AMDGPU has SGPR, VGPR, and (on some chips) AGPR. `pressure(P)`
is a vector. Vector `min` is not well-defined for paths; the DP
needs a totally ordered cost.

The clean fix is to use the same scalar metric the existing
DFS uses: `Policy::kMetric` evaluated by
`GCNRegisterTracker::GetMetricScore(metric)`. This produces a
single integer (the continuous occupancy score) that BFS-DP
minimizes. Tie-breaking can be lexicographic on a secondary
metric.

What does **not** work: separate per-class DPs. Each class would
optimize over a different Pareto frontier and the chosen
schedules would be incompatible.

### 4.4 Transient pressure during instruction scheduling

When instruction `x` is scheduled with prefix `P`, three pressure
values are in play:

1. `pressure(P)` — boundary pressure just before `x`.
2. `pressure(P ∪ {x})` — boundary pressure just after `x`.
3. **Transient peak during `x`** — defs of `x` can land before
   `x`'s last-uses kill, momentarily holding both sets live.
   Worst case: `pressure(P) + |defs(x)|` =
   `pressure(P ∪ {x}) + |last_uses(x)|`.

LLVM's `RegisterPressureTracker::advance` does record this
transient — see `bumpUpwardPressure` /
`bumpDownwardPressure` in `llvm/lib/CodeGen/RegisterPressure.cpp`.
The "max pressure" the tracker reports for a region's schedule
can therefore exceed both adjacent boundary pressures.

Implication for the DP: the simplified form
`f(Q) = max(pressure(Q), min_P f(P))` **under-counts** the true
peak by up to `|last_uses(x)|`. The per-edge form
`f(Q) = min_(P,x) max(f(P), edge_peak(P, x))` is correct.
`edge_peak(P, x)` is purely a function of `(P, x)` (the live set
at `P` and `x`'s def/use pattern), so the DP fits without losing
optimality.

### 4.5 Why this avoids the postfix-tracking pitfalls

§11 of `AMDGPUHistoryDominationDesign.md` documents why
postfix tracking + forward DFS proved architecturally fragile:
case-2 pruning interleaves with postfix recording, and IsDone-
only walks can leave `stored_postfix` under-bounded for
subtrees whose paths got pruned mid-DFS.

BFS-DP avoids this entirely. Each partition is processed exactly
once, after **all** its lattice predecessors are finalized. There
is no mid-subtree pruning interleaved with table updates because
there are no subtrees in the BFS framing. Soundness reduces to
the textbook minimax-path argument; no case-by-case reasoning
about pruning interactions.

---

## 5. Architecture and Components

### 5.1 Component placement

```
ScheduleGraph                             (unchanged)
  └── (formation produces SubgraphInfos as before)

BfsDpSearch                               NEW
  ├── DenseMap<PartitionKey, LatticeNode> lattice_
  ├── working_schedule_constructor_       (cursor; see §6.1)
  ├── reuses ScheduledSetTracker /        (for PartitionKey lookups)
  │   PartitionKey machinery
  └── reuses GCNRegisterTracker for       (incremental, via cursor)
      score reads
```

Files to add:

- `BfsDpSearch.h` / `BfsDpSearch.cpp` — the search class.
- `BfsDpMaximizeOccupancyPolicy` in `SearchPolicies.{h,cpp}` —
  policy-style configuration parallel to
  `DfsMaximizeOccupancyPolicy`. Or, if BFS-DP doesn't admit
  meaningful policy variation, fold its knobs onto a single
  config struct.

Files to extend:

- `GCNRegisterTracker.{h,cpp}` — optional `ComputeFromScratch`
  query (§5.5). Used as an audit / alternative to the cursor.

`DfsSearch`, `PressureHistoryTracker`, `LengthHistoryTracker`,
and the existing policies stay untouched. BFS-DP is parallel
infrastructure.

### 5.2 LatticeNode

```cpp
struct LatticeNode {
  PartitionKey key;                  // signature + scheduled bitset
  GCNRegPressure f;                  // or MetricScore — see §4.3
  int level;                         // = key.scheduled_set.count()
  const LatticeNode *back = nullptr;
  const ScheduleNode *transition_instr = nullptr;
};
```

Storage: `DenseMap<PartitionKey, LatticeNode>`. The key is
already DenseMap-compatible via the
`DenseMapInfo<PartitionKey>` specialization in
`ScheduledSetTracker.h`.

Memory cost per node: `sizeof(PartitionKey)` (~24 bytes plus the
bitset, which is `(n+7)/8` bytes for an n-instruction region) +
`sizeof(GCNRegPressure)` (~16 bytes for SGPR/VGPR/AGPR ints) +
4 (level) + 8 (back) + 8 (instr) ≈ `40 + bitset` bytes per node.
For n = 100 instructions, ~50 bytes per node; for n = 700, ~130
bytes. See §7.4 for the memory budget calculus.

### 5.3 BfsDpSearch

Top-level shape mirrors `DfsSearch`:

```cpp
template <typename Policy>
class BfsDpSearch {
 public:
  BfsDpSearch(ScheduleGraph &graph, const GCNSubtarget &st,
              const MachineFunction &mf, const LiveIntervals &lis,
              bool form_subgraphs = true);

  ScheduleConstructor Run();
  void ResetForReuse(int target_length);  // optional; see §7.5

  bool RegionTimedOut() const;
  bool BudgetExceeded() const;
  int  PartitionCount() const;
  // ... read-only telemetry parallel to DfsSearch's.

 private:
  // BFS-by-levels driver (§5.3.1)
  void RunBfs();

  // Process one partition: enumerate ready instructions,
  // compute candidates, merge into lattice_.
  void ProcessPartition(LatticeNode *current);

  // Backtrace and replay (§3.3, §5.6).
  ScheduleConstructor BuildResult();

  // Fallback: if BudgetExceeded() during RunBfs, the caller
  // either runs DfsSearch on the same region or uses the
  // graph's input baseline.
  // ...
};
```

The BFS-by-levels driver (§5.3.1):

```
RunBfs():
  initialize lattice_[empty_key] = LatticeNode{f = pressure(empty), ...}
  current_level = [&lattice_[empty_key]]
  for L = 0 to N-1:
    next_level = []
    for P in current_level:
      ProcessPartition(P)            // fills next_level via lattice_ inserts
      if budget_exceeded(): return
    if next_level empty: break
    current_level = next_level
```

`ProcessPartition` iterates the ready set at `P` (computed from
the cursor at `P`'s state), and for each ready `x`:

```
Q_key = key for P.scheduled_set | {x}
candidate_f = max(P.f, edge_peak(P, x))   // §4.4
LatticeNode &Q = lattice_[Q_key]          // creates if absent
if Q is newly created:
  Q.level = L + 1
  Q.f = candidate_f
  Q.back = &P
  Q.transition_instr = x
  next_level.push_back(&Q)
elif candidate_f < Q.f:                   // strict improvement
  Q.f = candidate_f
  Q.back = &P
  Q.transition_instr = x
```

### 5.4 BfsDpMaximizeOccupancyPolicy

Parallel to `DfsMaximizeOccupancyPolicy` but with different
contract: BFS-DP doesn't have a `ShouldBoundSearch` /
`ShouldEndSearch` concept. The policy supplies:

- `kMetric`: the score metric (occupancy, etc.).
- `kBudgetMaxPartitions`: hard cap on lattice size before
  fallback. Mirrors `kMaxEntries` on the history trackers
  (10⁷ as a starting point — see §7.4).
- `kBudgetMaxSeconds`: wall-clock fallback timeout, like
  `kTimeoutSecondsPerRegion` on `SearchPolicyBase`.
- `kMaxSubgraphSizeForBfsDp`: the maximum subgraph size at which
  BFS-DP is preferred over DFS (BFS-DP only runs on subgraphs
  smaller than this; larger subgraphs use the existing DFS).
- `MakeFormationPolicy()`: same hook as DFS, allowing different
  formation knobs for BFS-DP if needed.

If only one BFS-DP variant ever ships, the policy struct can
collapse to a config bag. The Policy template form leaves room
for variants (e.g., a length-pressure multi-objective variant)
without refactoring later.

### 5.5 Score query for arbitrary partitions

`GCNRegisterTracker` is incremental — it only knows the pressure
implied by its `Schedule`/`Unschedule` history. BFS-DP needs
either:

1. A way to roll the tracker to an arbitrary partition `P`
   (Schedule a sequence of instructions whose set equals `P`).
   This is what the cursor walk does (§6.1).
2. A from-scratch query
   `GCNRegPressure ComputeFromScratch(const BitVector &scheduled_set)`
   that computes pressure(P) directly from the bitset and the
   graph's def/use structure, without simulation.

The cursor walk is sufficient for production. The from-scratch
query is useful as:

- A shakedown audit: assert
  `tracker.GetCurrentPressure() == ComputeFromScratch(P)` after
  rolling the cursor to P, verifying tracker consistency.
- A pure check used in tests where simulating a roll to `P` is
  inconvenient.

The implementation walks the graph's def/use info: for each
register `r`, `r` is live at `P` iff `def(r) ∈ P` (or `r` is
live-in) and `users(r) ⊄ P`. Approximately O(|registers| × |P|),
or O(|edges|) with appropriate indexing. ~50 lines on top of the
existing tracker.

### 5.6 Schedule reconstruction

After `f(full set)` is finalized:

```cpp
ScheduleConstructor BuildResult() {
  // 1. Backtrace the lattice from full -> empty.
  std::vector<const ScheduleNode *> reverse_schedule;
  for (auto *n = &lattice_[full_key]; n->back; n = n->back) {
    reverse_schedule.push_back(n->transition_instr);
  }
  std::reverse(reverse_schedule.begin(), reverse_schedule.end());

  // 2. Replay through a fresh ScheduleConstructor.
  ScheduleConstructor result(graph_, st_, mf_, lis_,
                             default_ready_compare);
  for (auto *node : reverse_schedule) {
    result.ScheduleByPointer(node);   // or equivalent
  }
  return result;
}
```

The resulting constructor matches the same shape as
`DfsSearch::Run()`'s return: a complete schedule with all the
trackers consistent. The rest of the HierarchicalScheduler
pipeline consumes it without modification.

---

## 6. Lightweight Alternatives to Schedule-Constructor Copies

The naive shape — store a `ScheduleConstructor` snapshot on each
`LatticeNode` to recover state on visit — costs O(#partitions ×
sizeof(ScheduleConstructor)) memory, where each constructor is
hundreds of bytes plus the trackers. For 10⁷ partitions, this
crosses tens of GB. Unworkable in production.

There are several cheaper paths.

### 6.1 Option A: Single shared cursor, depth-first walk

Maintain ONE `ScheduleConstructor` ("the cursor"). To process
partition `P`, roll the cursor's state to `P` via a sequence of
`Schedule`/`Unschedule` calls. Visiting partitions in an order
that maximizes prefix sharing (effectively a DFS traversal of
the lattice) bounds each step by O(|P_curr Δ P_target|) tracker
updates. With careful ordering, amortized close to O(1) per
partition.

The traversal is not a strict BFS — instead, we visit each
partition once, in level order overall, but we process all
children of a single partition before moving on. The cursor moves
forward through children (one Schedule each), and backtracks
(one Unschedule) after each child's subtree (which in BFS-DP is
just "after the child has been recorded into the lattice and any
edges into it computed").

This option uses only the existing infrastructure
(`ScheduleConstructor`, `GCNRegisterTracker`,
`ScheduledSetTracker`) — no new pressure code in the hot path.

### 6.2 Option B: Per-partition live-set bitvector

Store on each `LatticeNode` a small bitvector: the live registers
at `P`. Don't store the constructor.

To process `P`:

- Live set in the node.
- Ready set computed from `P.scheduled_set` and the DAG (O(n)).
- Delta to compute pressure of `Q = P ∪ {x}`: for each ready `x`,
  defs of `x` add to live if they have downstream users; uses of
  `x` kill if last use. O(|defs(x)| + |uses(x)|).

This is what `GCNRegisterTracker` does internally on Schedule,
just hoisted out. Memory cost: one bitvector of width
≈ |virtual registers in region|. Sparse representation can keep
this under a hundred bytes per partition for typical regions.

### 6.3 Option C: From-scratch pressure query

Use `GCNRegisterTracker::ComputeFromScratch(BitVector)` (§5.5)
to compute `pressure(P)` on demand. No per-partition state.

Most O(1)-memory; most CPU. For 10⁷ partitions × 10⁻⁵ s per
query, that's ~100 s of CPU. Tolerable for occasional use but
not for the inner loop.

### 6.4 Option D (recommended): Cursor walk + from-scratch as audit

Combine A + C:

- Production hot path uses Option A's cursor walk: a single
  `ScheduleConstructor`, rolled across the lattice, doing the
  pressure work the existing tracker is already optimized for.
- Shakedowns and assertions use Option C's `ComputeFromScratch`
  to audit the cursor's state at sampled partitions, ensuring no
  drift between incremental updates and the from-scratch
  ground truth.

This combination:

- Reuses existing incremental machinery (no new pressure code in
  the hot path).
- Adds zero per-partition memory beyond `LatticeNode { PartitionKey, f,
  level, back, instr }` — ~50 bytes plus the bitset.
- Keeps `ScheduleConstructor` doing the job it was designed for
  (incremental tracking).
- Schedule reconstruction at the end uses the same constructor
  rolled back to empty, then forward through the optimal path.

The user-visible part of the design is just §5; this section
is about the implementation strategy that backs §5 efficiently.

### 6.5 Memory budget

With Option D, per-partition cost is approximately

- `PartitionKey`: 4 (signature) + bitset of `(n + 7) / 8` bytes.
- `f`: 16 bytes (`GCNRegPressure`) or 4 bytes (scalar score).
- `level`: 4 bytes.
- `back`, `instr`: 16 bytes total.
- DenseMap overhead: ~8 bytes amortized.

For n = 100, ≈ 60 bytes/partition; 10⁷ partitions → 600 MB.
For n = 700, ≈ 130 bytes/partition; 10⁷ partitions → 1.3 GB.

The 10⁷ cap matches the existing `kMaxEntries` soft cap on the
history trackers. It may need lowering for BFS-DP since BFS-DP
holds the full lattice in memory simultaneously, while DFS holds
only a stack-shaped subset. A starting cap of 10⁶ partitions
(60–130 MB) is safer, with `kBudgetMaxPartitions` configurable.

---

## 7. Concerns and Mitigations

### 7.1 Exponential partition count

The number of valid prefixes (downsets of the data-flow DAG) is
bounded by `2ⁿ` and achieves `2ⁿ` for a wide antichain (e.g., n
pairwise-independent loads). AMDGPU regions, especially after
vectorization or loop unrolling, can have wide antichains.

Empirical evidence we already have:

- 641-instruction stencil region (per
  `project_current_state.md`): the existing
  `PressureHistoryTracker` records 4.7M prune events under the
  10M `kMaxEntries` cap. Distinct partitions encountered are a
  subset of this; the cap was not hit. So at least one realistic
  high-pressure AMDGPU region has its reachable downsets in the
  millions, not billions.
- BFS-DP visits exactly the same set of reachable downsets as
  DFS-with-history (modulo ordering). If DFS-with-history is
  staying within memory, BFS-DP should too at comparable scale —
  but BFS-DP's resident-memory pattern is different (full
  lattice vs DFS stack), so the cap may need adjustment.

Mitigations, layered:

1. **Apply per subgraph, not per region** (§7.2). Subgraphs are
   smaller; partition counts scale per subgraph.
2. **Hard cap on `lattice_.size()`** (§5.4 `kBudgetMaxPartitions`).
   On exceedance, BFS-DP discards its partial table and the
   region falls back to DFS+history.
3. **Branch-and-bound with DFS as upper-bound seed** (§7.3).
4. **Beam search at each level** — losing optimality but
   bounding cost. Probably not the first choice; the value of
   BFS-DP is provable optimality.

### 7.2 Subgraph integration as the primary mitigation

Subgraph formation already produces flat two-level subgraphs with
disjoint members. Per `AMDGPUSubgraphFormationDesign.md`, member
counts are typically much smaller than full regions.

The integration: BFS-DP runs **per subgraph**, only when
`subgraph.member_count() <= kMaxSubgraphSizeForBfsDp`. Larger
subgraphs continue to use DFS+history. The composition step
that interleaves subgraph schedules at the region level
(`AMDGPUClusteringDesign.md` Approach B) is unchanged — BFS-DP
only affects how each subgraph's internal schedule is chosen.

This is the same composition pattern Malik 2008 used to scale CP
to larger basic blocks (per Castañeda Lozano & Schulte §4.1):
decompose, solve subproblems, compose. The novelty in our case
is doing it for AMDGPU + LLVM + Kessler-style DP specifically.

### 7.3 Branch-and-bound with DFS as a seed

Run the existing DFS-with-history first to obtain a complete
schedule's peak pressure `K`. Then run BFS-DP with the prune

```
if candidate_f >= K, skip recording — Q can never beat K
```

Any path through the lattice with cost ≥ K is dominated by the
seed schedule, so prune. This can cut the lattice size
substantially when the seed is near-optimal, at the cost of
running DFS first.

Cost-benefit: doubles work on regions where DFS is already
optimal; saves work on regions where DFS misses optimum but
BFS-DP would have to explore many paths to confirm. Not a
default; an opt-in flag.

### 7.4 Memory cost

§6.5 already covered the per-partition budget. The total
budget calculation:

- Per-partition: ~60-130 bytes (varies with n).
- Cap: `kBudgetMaxPartitions`. Starting value: 10⁶.
- Total: ~60-130 MB for the lattice itself.

On top of that, the DenseMap has ~25-50% overhead at typical
load factors. So plan on 100-200 MB for a near-cap run on a
medium region; 1+ GB if we ever raise the cap to 10⁷.

This is a per-region allocation that's freed when the region's
search ends. As long as regions are processed serially (which
they are in `MachineScheduler`'s top-level driver), peak memory
is bounded by the largest single region's lattice.

### 7.5 Outer-loop integration (length passes)

`DfsSearch::ResetForReuse(target_length)` exists for outer loops
that walk a target value across multiple `Run()` calls (see
`ScheduleRegionForMinimumLength`). The length pass is a
secondary objective different from BFS-DP's primary objective
(minimum register pressure).

If we want a length-pressure multi-objective BFS-DP variant
later (Kessler 1998 §6 suggests this is feasible at smaller
scale), the DP state extends to `(partition, current_cycle)` and
the recursion becomes lexicographic. Out of scope here. For
now: BFS-DP runs once per (sub)region, single-objective. The
length pass keeps using DFS.

---

## 8. Prior Art and Novelty Assessment

This section is the result of the literature review documented
in `reference_register_pressure_dp_prior_art.md`. The bottom
line is restated up front in the Purpose section; this section
elaborates.

### 8.1 Kessler 1998 — the actual algorithm

Christoph Kessler. *Scheduling Expression DAGs for Minimal
Register Need.* Computer Languages 24(1):33–53, September 1998.

Algorithm: subset-DP over the partition lattice with a state
transition that merges nodes representing the same scheduled
set, keeping the entry with the lowest register pressure. The
correctness argument is the prefix-order-independence
observation in §4.1.

Worst-case complexity reported: O(n · 2^(2n)). Reported
practical scalability: optimal for basic blocks of up to ~51
instructions, register need up to 13. Multi-objective
extension (length + RP) scales to ~25 instructions.

The match between Kessler's algorithm and this design is
verbatim. There is no algorithmic novelty here. The Castañeda
Lozano & Schulte 2019 survey describes Kessler's algorithm in
language identical to §3 of this doc:

> "The technique explores a search tree where nodes correspond
> to sets of instructions that can be issued next ... A key
> improvement comes from the realization that nodes with the
> same sets of instructions can be merged in a dynamic
> programming fashion by simply selecting the one with lowest
> register pressure. This is possible since the particular
> order in which earlier instructions are scheduled to arrive
> at a certain search node does not need to be known to compute
> the optimal solution."

### 8.2 The Castañeda Lozano & Schulte survey

Roberto Castañeda Lozano and Christian Schulte. *Survey on
Combinatorial Register Allocation and Instruction Scheduling.*
arXiv:1409.7628v3 (2019). Published in ACM TOPLAS 2019.

The canonical literature map. Appendix A describes
register-pressure-aware approaches; Table 5 classifies them by
technique, scope, problem coverage, and largest size solved
optimally:

| Approach | Year | Technique | Scope | Largest |
|---|---|---|---|---|
| Kessler | 1998 | Enumeration + DP | local | ~51 |
| Govindarajan et al. (MRIS) | 2003 | IP + lineage heuristic | local | ~20 |
| Malik | 2008 | CP | local | ~100s |
| Shobaki et al. (OptSched) | 2013 | Enumeration / B&B | local | ~100s |
| Barany–Krall | 2013 | IP | global | — |

Worth reading end-to-end if anyone touches this work
seriously; it disambiguates terminology and gives a clean
genealogy.

### 8.3 Shobaki et al. CGO 2024 — direct AMDGPU competitor

Ghassan Shobaki et al. *Instruction Scheduling for the GPU on
the GPU.* CGO 2024. Co-authored with Austin Kerbow (AMD).

Algorithm: GPU-parallel Ant Colony Optimization (ACO) for the
RP-aware instruction scheduling problem, in LLVM, targeting
AMDGPU. APRP (adjusted peak register pressure) cost function
tied to occupancy bins — same target metric as our
`continuous_register_occupancy_score`.

Reported result: up to **74% execution-speed improvement** over
AMD's production scheduler on rocPRIM benchmarks. Cites Kessler
as [2].

This is the closest competitor for any AMDGPU-targeted
publication that comes out of this work. Their algorithmic
approach is different (ACO, not Kessler-DP), but their
problem statement, target compiler, target architecture, and
target metric overlap heavily with ours. Any paper we write has
to have a clear story for why our approach complements or
beats theirs.

### 8.4 Vertex separation / directed pathwidth

The graph-theory framing: minimum-RP scheduling on an SSA
data-flow DAG is equivalent to **vertex separation** of that
DAG (also called **directed pathwidth**). See:

- Bodlaender 1998, *A partial k-arboretum of graphs with bounded
  treewidth* — survey, including subset-DP exact algorithms.
- Kinnersley 1992, *The vertex separation number of a graph
  equals its path-width* — equivalence for the undirected case.
- Tamaki 2011, *A Polynomial Time Algorithm for Bounded
  Directed Pathwidth* (WG 2011 / LNCS 6986). O(m · n^(k+1))
  parameterized by k = directed pathwidth.

The compiler literature does not, as far as the lit review
found, make this equivalence explicit. The graph-theory side
hasn't applied its tools to compiler scheduling. If we make the
connection explicit and bring graph-theoretic results to bear
(e.g., width-parameterized algorithms, branch-decomposition
bounds), that's a framing contribution that hasn't been
written up.

### 8.5 OptSched B&B comparison

Shobaki et al. 2013 (OptSched) and the subsequent CGO 2024 ACO
work share the same problem but use B&B and ACO respectively.
The History Domination doc already cites OptSched's
hist_table for the inspiration behind Phase 4b's prefix-
dominance pruning.

Comparison points that matter for evaluation:

- **Optimality guarantee:** BFS-DP is exact (modulo budget); ACO
  is intelligent search (often near-optimal but not provably
  exact); OptSched B&B is exact under its time budget.
- **Scaling shape:** BFS-DP is bounded by partition count; B&B
  by branch count; ACO by ant population × iterations. Different
  regimes win on different inputs.
- **Production use:** OptSched is shipped as a plugin; Shobaki's
  GPU-ACO is research code. The HierarchicalScheduler is
  production-targeted.
- **Multi-objective handling:** OptSched and ACO both handle
  length+RP; Kessler's multi-objective extension handles it
  with worse scaling; our BFS-DP is single-objective (RP only)
  in scope as described here.

### 8.6 What remains worth doing

Given the prior-art picture, the contributions plausibly worth
publishing are:

1. **Kessler-DP scaled via subgraph decomposition.** Per-subgraph
   BFS-DP, hierarchically composed at the region level, applied
   to AMDGPU regions of 600+ instructions where Kessler 1998
   alone times out. Decomposition is itself well-known (Malik
   2008 used it for CP), so the contribution is the specific
   composition + Kessler-DP + AMDGPU empirics.

2. **Empirical comparison on AMDGPU.** Head-to-head against AMD's
   production scheduler, OptSched B&B, and Shobaki et al.'s
   ACO, on rocPRIM and other ROCm/HIP benchmarks.
   Characterize where each wins.

3. **The vertex-separation framing.** Bridge between compiler
   and graph-theory literatures. Cite Tamaki's parameterized
   algorithm, discuss whether width-parameterized variants are
   worth implementing, etc. Light-touch theoretical
   contribution.

None of these is "novel algorithm." All of them are
"engineering or framing contributions on top of a known
algorithm." That puts realistic publication targets at CGO,
CC, or TACO; not PLDI or ASPLOS.

The honest framing in any writeup: "We adapt Kessler 1998 to
AMDGPU via subgraph-decomposed composition, evaluate against
production and research baselines including Shobaki et al.
2024, and report ..."

---

## 9. Open Questions

These are decisions we need to make before implementation, not
things the algorithm leaves ambiguous.

1. **Drop-in scope.** Does BFS-DP replace DFS:
   - (a) only inside subgraphs of size ≤ threshold, with DFS for
     everything else?
   - (b) at the region level whenever feasible, falling back to
     DFS on budget overflow?
   - (c) unconditionally in some experimental-only mode?

   Recommendation: (a) for the first prototype. Lowest risk,
   clearest scope.

2. **Branch-and-bound with DFS seed.** Run DFS first to get an
   upper bound, then BFS-DP with B&B pruning? Doubles work on
   easy regions; saves work on hard ones. Default off,
   opt-in flag?

3. **Length objective.** Does BFS-DP need to handle length-RP
   multi-objective (Kessler 1998 §6's extension)? This would
   parallel the existing `ScheduleRegionForMinimumLength` outer
   loop. Out of scope for v1.

4. **Cursor walk vs from-scratch query.** §6 recommends the
   cursor walk (Option D). Are there specific workloads where
   the from-scratch query is preferred? (e.g., extremely wide
   subgraphs where cursor amortization breaks down?)

5. **Memory cap.** Start at `kBudgetMaxPartitions = 10⁶` to be
   conservative? Or 10⁷ to match the existing history-tracker
   cap? What's the fallback behavior in production — silent
   fallback to DFS, or visible diagnostic?

6. **Test corpus.** Beyond the 641-instruction stencil, what
   other regions should be in the shakedown set? The Shobaki
   2024 paper uses rocPRIM; matching their corpus would help
   for any future comparison.

---

## 10. Implementation Phases

Following the pattern of `AMDGPUHistoryDominationDesign.md`'s
Phase 4 split: each pair has one "logic + standalone tests"
phase followed by one "DFS / production wiring + integration
tests" phase.

### Phase 1 — LatticeNode and from-scratch pressure query

Standalone, no integration with the search yet.

- Add `LatticeNode` struct and `BfsDpSearch` skeleton with the
  `lattice_` DenseMap.
- Add `GCNRegisterTracker::ComputeFromScratch(BitVector)` (§5.5).
- Standalone shakedowns: empty partition pressure, full partition
  pressure, single-instruction partitions for hand-crafted DAGs.
  Cross-check against the cursor's incremental score after
  rolling.

### Phase 2 — Lattice enumeration and DP loop

The core BFS-DP loop in `RunBfs`, with the cursor walk (§6.1).

- Enumerate ready instructions at each partition.
- Compute candidate `f` values using the cursor's incremental
  pressure.
- Update `lattice_` entries with strict-improvement merges.
- Standalone shakedowns: hand-crafted small DAGs (chain, diamond,
  fan-out-fan-in) where the optimal `f` value is computable by
  hand. Verify the lattice's `f(full set)` matches.

### Phase 3 — Backtrace and replay

Schedule reconstruction (§3.3, §5.6).

- Walk `back` from full to empty, collect transitions, reverse,
  replay through a fresh `ScheduleConstructor`.
- Standalone shakedowns: on the small DAGs from Phase 2, verify
  the replayed schedule matches an optimal hand-computed
  schedule (by peak pressure, not by exact instruction order
  since ties exist).

### Phase 4 — Production integration

**Production behavior changes after this phase** if the policy
flag is enabled per (sub)region.

- `BfsDpMaximizeOccupancyPolicy` with the budget knobs from §5.4.
- Per-subgraph dispatch: if
  `subgraph.member_count() <= kMaxSubgraphSizeForBfsDp`, run
  BFS-DP; else run the existing DFS+history. Fall back to DFS on
  BFS-DP budget exceedance.
- Per-region telemetry parallel to DFS's
  (`schedule_call_count`-equivalent: `partition_count`,
  `budget_exceeded`).
- Comparison shakedown: on a pressure-meaningful region, verify
  BFS-DP and DFS agree on the optimal `f` value. Verify
  BFS-DP's `partition_count` is bounded.

### Phase 5 — Real-region exercise

- Run on the 641-instruction stencil. Expect: most subgraphs
  fall under the threshold and use BFS-DP; a few large ones use
  DFS. Total compile time and final occupancy compared to the
  current DFS-only baseline.
- Run on additional rocPRIM kernels if available.
- Telemetry: how many subgraphs hit the BFS-DP path, how often
  the budget is exceeded, how partition counts distribute.

### Phase 6 — (optional) Branch-and-bound seeding

If empirical results in Phase 5 suggest BFS-DP's lattice grows
larger than necessary on some workloads:

- Run DFS first to seed an upper bound; pass it to BFS-DP as a
  prune threshold (§7.3).
- Measure the partition-count reduction.

### Phase 7 — (optional, much later) Multi-objective variant

Length + RP, parallel to Kessler 1998 §6's extension. State
becomes `(partition, current_cycle)`; objective is
lexicographic. Out of scope for the initial work; documented
here so the architectural shape is clear.

---

## 11. Citations

- Christoph Kessler. *Scheduling Expression DAGs for Minimal
  Register Need.* Computer Languages 24(1):33–53, 1998.
  ([ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0096055198000022))

- Roberto Castañeda Lozano and Christian Schulte. *Survey on
  Combinatorial Register Allocation and Instruction Scheduling.*
  arXiv:1409.7628v3, 2019; ACM TOPLAS 2019.
  ([arXiv](https://arxiv.org/abs/1409.7628))

- Ghassan Shobaki, Pınar Muyan-Özçelik, Josh Hutton, Bruce Linck,
  Vladislav Malyshenko, Austin Kerbow, Ronaldo Ramirez-Ortega,
  Vahl Scott Gordon. *Instruction Scheduling for the GPU on
  the GPU.* CGO 2024.
  ([PDF](https://athena.ecs.csus.edu/~gordonvs/papers/cgo24-paper69.pdf))

- Ghassan Shobaki, Maxim Shawabkeh, Najm Eldeen Abu Rmaileh.
  *Preallocation Instruction Scheduling with Register Pressure
  Minimization Using a Combinatorial Optimization Approach.*
  ACM TACO 10(3), September 2013.

- R. Govindarajan, Hongbo Yang, José Nelson Amaral, Chihong Zhang,
  Guang R. Gao. *Minimum Register Instruction Sequencing to
  Reduce Register Spills in Out-of-Order Issue Superscalar
  Architectures.* IEEE Transactions on Computers 52(1), 2003.
  ([Author PDF](https://webdocs.cs.ualberta.ca/~amaral/papers/LRA-TC-2003.pdf))

- Hisao Tamaki. *A Polynomial Time Algorithm for Bounded
  Directed Pathwidth.* Workshop on Graph-Theoretic Concepts in
  Computer Science (WG 2011), LNCS 6986.

- Hans L. Bodlaender. *A partial k-arboretum of graphs with
  bounded treewidth.* Theoretical Computer Science 209(1-2),
  1998.

- Nancy G. Kinnersley. *The vertex separation number of a graph
  equals its path-width.* Information Processing Letters 42(6),
  1992.
