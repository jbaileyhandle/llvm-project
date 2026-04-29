# HierarchicalScheduler: History-Based Domination Design

## Purpose of this Document

This doc specifies a B&B pruning optimization for the
HierarchicalScheduler's `DfsSearch`. Two prefixes that scheduled the
same set of instructions face the same "remaining work" problem.
Whichever prefix has better progress-so-far dominates the other; the
dominated one's subtree can be pruned entirely.

The technique is "history-based domination" from Shobaki et al.'s
work on enumeration-based instruction scheduling (most thoroughly
documented in *Optimal Superblock Scheduling Using Enumeration*,
MICRO 2004; also see *A Parallel Branch-and-Bound Algorithm with
History-Based Domination*, PPoPP 2022). OptSched implements it in
`OptSched/lib/Scheduler/{enumerator,hist_table}.cpp` and reports
1-2 orders of magnitude speedup on enumeration-heavy regions.

This doc covers our planned implementation: the data structures,
how length and register-pressure history differ, how proxy nodes
from subgraph formation fit in, and the phasing.

## Table of Contents

1. [Context and Prerequisites](#1-context-and-prerequisites)
2. [Terminology](#2-terminology)
3. [Architecture Overview](#3-architecture-overview)
4. [Components](#4-components)
5. [Length History](#5-length-history)
6. [Pressure History](#6-pressure-history)
7. [Proxy Handling](#7-proxy-handling)
8. [DfsSearch Integration](#8-dfssearch-integration)
9. [Worked Example](#9-worked-example)
10. [Implementation Phases](#10-implementation-phases)

A note on the structure: §6 ("Pressure History") is the part of
this design that has evolved most. It now covers a score-based
metric (matching `kMaximizeContinuousRegisterOccupancyScore`),
implicit handling of fully-pruned subtrees via an `INT_MIN`
sentinel, and a fast-forward mechanism via a generic replay queue
on `DfsSearch`. A short comparison with OptSched's `hist_table`
sits inside §6 to make the deltas explicit.

---

## 1. Context and Prerequisites

### 1.1 The optimization at a glance

`DfsSearch<Policy>` enumerates schedules by depth-first search over
ready-list orderings. At each level, it picks an instruction from
the ready list, schedules it, and recurses. The same set of
scheduled instructions can be reached via N! different orderings —
without history-based domination, each one re-explores the same
remaining work.

History-based domination keeps a table indexed by "which set of
instructions has been scheduled so far." When the search reaches a
prefix at signature S, it consults the table:

- If a prior prefix at signature S exists with provably-better
  state (same scheduled set, same-or-better progress on every
  remaining-work dimension), prune the current subtree.
- Otherwise, insert the current prefix into the table for future
  prefixes to compare against.

### 1.2 Why it works

The remaining work after a partial schedule is determined by:
1. The set of unscheduled instructions (= complement of scheduled set).
2. The starting conditions for the postfix (current cycle, frontier
   readiness times, register-pressure baseline).

(1) is the same for any two prefixes with the same scheduled set.
(2) varies between prefixes — that's what the dominance check
compares.

If prefix A has same scheduled set as prefix B AND A's starting
conditions are no worse than B's on every dimension, then any
schedule reachable from B is also reachable from A at the same or
lower cost. So B's subtree can be pruned without losing optimality.

### 1.3 What already exists

- `DfsSearch<Policy>` in `DfsSearch.h` does the recursion.
- `ScheduleConstructor` tracks the partial-schedule state, including
  `current_cycle` (length tracker) and `peak_pressure` (forward
  register tracker).
- `ScheduleNode` has a unique topological index per graph.
- `ScheduleGraph::ComputeCriticalPathFromExit` provides the
  per-node `cp_from_exit`.
- The forward register tracker exists in `GCNRegisterTracker`.

What we need to add:
- A `ScheduledSetTracker`: signature, bitset, frontier tracking.
- Per-instruction pressure recording on the forward register tracker.
- Two history-table classes, one for each metric.
- Policy-driven opt-in via a `kUse*` flag pattern.

---

## 2. Terminology

- **Partition**: the bipartition of graph nodes induced by a
  partial schedule. Scheduled set on one side, unscheduled set on
  the other. Identified up to set-equality by a 64-bit XOR
  signature with bitset disambiguation.
- **Prefix**: the partial schedule itself — a specific ordered
  sequence of scheduled nodes plus their cycle assignments.
  Different prefixes can share a partition (same scheduled set,
  different ordering).
- **Postfix**: the remaining unscheduled nodes; the work the search
  still has to do.
- **Frontier**: unscheduled nodes with at least one scheduled
  predecessor. Their lower bounds on earliest issue cycle are
  prefix-dependent.
- **Boundary live set**: for register-pressure purposes, the set of
  virtual registers defined in the prefix and used in the postfix
  (= data edges crossing the partition cut). Function of the
  partition only, independent of any specific prefix.
- **Domination**: prefix A dominates prefix B iff they share a
  partition AND A's starting conditions are no worse than B's on
  every dimension that affects achievable postfix outcomes.
- **History table**: per-search hash table keyed by prefix
  signature. Holds entries describing previously-visited prefixes.
- **`ScheduledSetTracker`**: the per-`ScheduleConstructor` state
  that history trackers query — signature, scheduled bitset,
  frontier.

---

## 3. Architecture Overview

### 3.1 Component placement

```
ScheduleGraph
  └── (unchanged — proxy nodes from formation don't change history scheme)

ScheduleConstructor                          (extended)
  ├── ScheduleLengthTracker                  (existing)
  ├── GCNForwardRegisterTracker              (existing — extended to record per-
  │                                           instruction pressure)
  └── ScheduledSetTracker                       NEW
        — self-contained: per-node signatures, prefix XOR signature,
          scheduled bitset, frontier (counts + LBs).
        — updated in lockstep with other trackers in
          Schedule/Unschedule.

DfsSearch                                    (extended)
  ├── working_, best_ : ScheduleConstructor
  ├── LengthHistoryTracker length_history_      NEW (only used if
  │                                                  Policy::kUseLengthHistoryPruning)
  ├── PressureHistoryTracker pressure_history_  NEW (only used if
  │                                                  Policy::kUsePressureHistoryPruning)
  └── std::queue<const ScheduleNode*> replay_queue_   NEW (generic
                                                          replay
                                                          mechanism;
                                                          pressure
                                                          fast-forward
                                                          uses it)

SearchPolicyBase                             (extended)
  └── kUseLengthHistoryPruning, kUsePressureHistoryPruning
      bool flags, default false; concrete policies override true.

Policy::ShouldBoundSearch                    (extended signature)
  └── takes (working, best, length_history, pressure_history) and
      dispatches to history-pruning checks internally via
      `if constexpr` on the policy's pruning flags. There is a
      single prune-decision API; `Recurse` does one call. The
      length-history dispatch lands in Phase 2b; pressure-history
      dispatch lands in Phase 4b/4c.
```

### 3.2 Why this placement

**`ScheduledSetTracker` on `ScheduleConstructor`**: its mutable
state moves in lockstep with `Schedule`/`Unschedule` — that's
schedule-construction state. Lives alongside the existing trackers.
Self-contained: it owns the per-instruction signature array
(initialized once in its ctor from the graph), the running prefix
signature, the bitset, and the frontier. Nothing outside this class
needs to know about signatures.

**History tables on `DfsSearch`**: tables accumulate across many
prefixes during the search — that's search-lifetime state, not
schedule-lifetime. `ScheduleConstructor` is copyable (`best_` is
copy-constructed from `working_` whenever working beats it); having
the table inside it would copy the table along with the constructor.
The table belongs to the search.

**Forward register tracker extended, no separate backward
tracker**: postfix peak is derivable from per-instruction pressure
recorded by the forward tracker (suffix max). No need for a
separate backward tracker — see §6 for details.

### 3.3 Architectural principles

**Policy opt-in is uniform.** Same pattern as `MakeFormationPolicy`
in subgraph formation: each pass declares which trackers/histories
it wants via static flags on its policy class. Trackers are
constructed unconditionally on `DfsSearch` but only consulted when
the corresponding flag is true (gated by `if constexpr`, so the
inactive code is dead-stripped).

**Length and pressure histories are siblings, not derived from a
common base.** They share the `ScheduledSetTracker` but their dominance logic
is genuinely different: length needs a Pareto frontier of
multi-dimensional entries; pressure has one entry per partition
with two scalars. Forcing a common base would constrain both
without reducing duplication.

**`ScheduledSetTracker` is self-contained.** Per-instruction
signatures are an implementation detail. They're initialized inside
`ScheduledSetTracker`'s ctor from the graph. Nothing outside the
class sees them.

**`ScheduledSetTracker` handles proxies uniformly.** Proxy nodes from subgraph
formation participate in signatures, bitset, frontier just like
real instructions. See §7 for why.

---

## 4. Components

### 4.1 ScheduledSetTracker

```cpp
class ScheduledSetTracker {
public:
  // Initialize per-instruction signatures from the graph at
  // construction time. Uses a deterministic-seeded mt19937_64 so
  // signatures are reproducible across runs. Sized to graph.Size()
  // — which includes proxies.
  ScheduledSetTracker(const ScheduleGraph &graph);

  // Notification hooks called by ScheduleConstructor on every
  // Schedule/Unschedule. Updates prefix_signature_, scheduled_set_,
  // frontier_ in lockstep.
  void OnSchedule(const ScheduleNode *node);
  void OnUnschedule(const ScheduleNode *node);

  // Read-only accessors for trackers to query.
  int64_t GetPrefixSignature() const { return prefix_signature_; }
  const BitVector &GetScheduledSet() const { return scheduled_set_; }
  const DenseMap<int, FrontierEntry> &GetFrontier() const {
    return frontier_;
  }

private:
  std::vector<int64_t> per_node_signatures_;   // by topo_idx, immutable
  int64_t prefix_signature_ = 0;               // running XOR of scheduled
  BitVector scheduled_set_;                    // bit per topo_idx
  std::vector<int> scheduled_pred_count_by_topo_;  // for frontier maint.
  DenseMap<int, FrontierEntry> frontier_;      // topo_idx → frontier info
};

struct FrontierEntry {
  int lower_bound;   // earliest issue cycle this frontier node could fire
};
```

**Signature width.** 64 bits. Birthday-bound collision threshold
~2^32 entries, comfortable margin for our regions (up to ~20k
nodes).

**Frontier maintenance.**

When a node X is scheduled (forward tracker has just placed it at
some cycle `c`):
1. For each successor S of X:
   - If S's `scheduled_pred_count == 0` (S becomes a frontier node):
     add S to `frontier_` with `lower_bound = c + max(1, lat(X→S))`.
   - Else:
     update existing frontier entry's `lower_bound = max(current,
     c + max(1, lat(X→S)))`.
   - Increment S's `scheduled_pred_count`.
2. If X itself was in `frontier_` (it must have been): remove it
   from `frontier_`.

When a node X is unscheduled:
1. Add X back to `frontier_` with its `lower_bound` recomputed from
   remaining scheduled preds (or 0 if none).
2. For each successor S of X:
   - Decrement S's `scheduled_pred_count`.
   - If `scheduled_pred_count` is now 0: remove S from `frontier_`.
   - Else: recompute S's `lower_bound` from remaining scheduled preds.

The recompute on unschedule is O(in-degree of S) per affected
successor. Acceptable for typical low-degree DAGs; could be
optimized later via per-(S,P) contribution storage if profiling
shows it's hot.

### 4.2 LengthHistoryTracker

See §5 for the full design. Brief shape (matching the
committed code):

```cpp
struct FrontierLb {
  int node_topo_idx;
  int lower_bound;
};

class LengthHistoryTracker {
 public:
  static constexpr int kMaxEntries = 20'000;

  struct Entry {
    int end_cycle;
    SmallVector<FrontierLb, 16> frontier_lbs;
  };

  /// Bind to source-of-truth pointers at construction; per-visit
  /// API takes no arguments. Mirrors the ScheduledSetTracker
  /// pattern.
  LengthHistoryTracker(
      const ScheduledSetTracker *scheduled_set_tracker,
      const ScheduleLengthTracker *length_tracker);

  /// Combined check + insert. Returns true if the current prefix
  /// (read from the bound trackers) is dominated by some existing
  /// Pareto-frontier entry at this partition; if false, inserts
  /// the current prefix into the bucket (Pareto-trimming dominated
  /// entries) and returns false. One bucket lookup per call.
  bool IsDominatedElseInsert();

  /// Read-only check (no insert, no Pareto trim). Useful for
  /// shakedowns.
  bool IsDominated() const;

  int GetTotalEntries() const;
  int GetTotalPruneCount() const;
  SmallVector<FrontierLb, 16> GetFrontierLbsSnapshot() const;

  // Test-only:
  void InsertEntryForTest(const PartitionKey &key, Entry entry);
  ArrayRef<Entry> GetBucketForTest(const PartitionKey &key) const;

 private:
  DenseMap<PartitionKey, SmallVector<Entry, 2>> table_;
  int prune_count_ = 0;
  // bound trackers, etc.
};
```

Per partition, the bucket holds a Pareto frontier of incomparable
entries. The map key is the shared `PartitionKey` exposed by
`ScheduledSetTracker` (see §4.1); cross-partition signature
collisions are handled automatically by `DenseMap` open-addressing
+ `DenseMapInfo<PartitionKey>::isEqual` (which compares the full
bitset). No manual bucket-walk for collision disambiguation.

`kMaxEntries` is enforced internally; insertion past the cap
reports a fatal error (loud rather than silent degradation; LRU
eviction deferred — see §8.3).

### 4.3 PressureHistoryTracker

See §6 for the design rationale and pruning logic. Brief shape,
mirroring `LengthHistoryTracker`'s pattern (constructor binds
source-of-truth state pointers; per-visit API takes no arguments
beyond what's available on the bound state):

```cpp
class PressureHistoryTracker {
 public:
  /// One entry per partition. Public so tests can stage entries
  /// directly via InsertEntryForTest, parallel to LengthHistoryTracker.
  struct Entry {
    int best_prefix_score;            // higher = better (we MAXIMIZE)
    int best_postfix_score = INT_MIN; // sentinel: no completion below
    ScheduleNode *next_node_hint = nullptr;  // fast-forward replay
  };

  /// Bind to:
  ///   - working ScheduleConstructor's scheduled_set_tracker and
  ///     forward register tracker (sources of truth for partition
  ///     key + current_prefix_score),
  ///   - best ScheduleConstructor's forward register tracker (source
  ///     of truth for best_so_far_score used in the total-bound
  ///     prune),
  ///   - an enqueue_for_replay callback the tracker invokes when
  ///     case 4 of the prune logic fires (see §6.4). The callback
  ///     pushes the supplied node onto DfsSearch's replay queue
  ///     and returns true on success. It returns false (no-op)
  ///     when the queue is non-empty — see §6.6
  ///     "no-op-on-non-empty invariant".
  /// All pointers/callable non-null; must outlive the tracker.
  PressureHistoryTracker(
      const ScheduledSetTracker *scheduled_set_tracker,
      const GCNForwardRegisterTracker *working_register_tracker,
      const GCNForwardRegisterTracker *best_register_tracker,
      std::function<bool(ScheduleNode *)> enqueue_for_replay);

  /// Combined check + record. Returns true if pruning should fire
  /// (prefix dominance OR total-bound). On miss-prune, records the
  /// current prefix score in the entry. May call enqueue_for_replay_
  /// internally when case 4 (§6.4) fires, requesting a fast-forward
  /// step from DfsSearch. Reads current_prefix_score from
  /// working_register_tracker_ and best_so_far_score from
  /// best_register_tracker_ — no arguments. Mirrors
  /// LengthHistoryTracker::IsDominatedElseInsert.
  bool IsDominatedElseRecord();

  /// Called on completion (working_.IsDone()). Walks schedule_order
  /// internally, computes partition keys incrementally, and updates
  /// best_postfix_score and next_node_hint for every partition along
  /// the path.
  void RecordPostfixScoresFromCompletedSchedule();

  /// Test-only helpers (parallel to LengthHistoryTracker).
  void InsertEntryForTest(const PartitionKey &key, Entry entry);
  /// Returns a pointer to the entry for `key`, or nullptr if absent.
  /// Pointer invalidated by any subsequent table mutation.
  const Entry *GetEntryForTest(const PartitionKey &key) const;
  int GetTotalEntries() const { return table_.size(); }
  int GetTotalPruneCount() const { return prune_count_; }

 private:
  const ScheduledSetTracker *scheduled_set_tracker_;
  const GCNForwardRegisterTracker *working_register_tracker_;
  const GCNForwardRegisterTracker *best_register_tracker_;
  std::function<bool(ScheduleNode *)> enqueue_for_replay_;
  DenseMap<PartitionKey, Entry, PartitionKeyDenseMapInfo> table_;
  int prune_count_ = 0;
};
```

#### Why a `std::function` callback for replay enqueue, rather than a `DfsSearch *` handle?

The tracker needs some way to push hints onto the DfsSearch's
replay queue when fast-forward fires. Three options were
considered:

(a) **Hold a `std::queue<ScheduleNode *> *`** (raw queue pointer).
    Tightly couples the tracker to the queue's concrete type;
    if DfsSearch later changes the queue representation
    (SmallVector with head index, ring buffer, etc.) the
    tracker breaks. Also gives the tracker more access than it
    needs (could pop, clear).

(b) **Hold a `DfsSearch<Policy> *`** with a public `EnqueueReplay`
    method. Most natural OO shape — tracker holds a handle to
    its parent, calls a method. But `DfsSearch` is templated on
    `Policy`, and `PressureHistoryTracker` is currently not.
    Holding a `DfsSearch<Policy> *` forces the tracker to be
    templated on `Policy` too, which:
    - Ripples through the source layout (header-only or explicit
      instantiation list).
    - Breaks parallelism with `LengthHistoryTracker`, which is
      non-templated.
    - Forces all the shakedown tests to instantiate against a
      concrete policy.
    The alternative — a non-templated base class for `DfsSearch`
    exposing `EnqueueReplay` — adds an inheritance hierarchy
    just for one method call.

(c) **Hold a `std::function<void(ScheduleNode *)>`** (chosen).
    Dependency injection: the constructor receives a callback,
    DfsSearch passes a small lambda that pushes onto its queue.
    Tracker stays non-templated. Tracker doesn't see DfsSearch's
    API surface or the queue's concrete type. Tests can pass a
    recording lambda and assert which nodes were enqueued.

The cost of (c) over (b) is one type-erased indirect call per
enqueue. The captured state is one pointer (`this`), well within
`std::function`'s small-buffer optimization (no heap allocation).
Enqueues happen at most once per visit. The performance cost is
negligible against the partition-key computation that dominates
a visit.

The design choice is **(c) callback**, primarily for the
templating reason: keeping `PressureHistoryTracker` non-templated
parallels `LengthHistoryTracker` and keeps tests simple.

The shape differs from the original draft in four ways:

1. **`PartitionKey` + `DenseMapInfo` reuse.** The tracker uses the
   shared `PartitionKey` type and `DenseMapInfo` specialization
   that `ScheduledSetTracker` exposes — the same key
   `LengthHistoryTracker` uses. Cross-partition signature
   collisions are handled automatically by `DenseMap` probing.
2. **One Entry value per partition** (not a Pareto frontier of
   incomparable entries). Pressure is one-dimensional via the
   continuous occupancy score, so a single best-prefix-score /
   best-postfix-score pair captures everything we need at a
   partition (see §6.1 for the decoupling argument). The map
   value type is `Entry` directly, not `SmallVector<Entry>`.
   There is no "bucket" — `LengthHistoryTracker`'s bucket
   terminology applies because that map's value is the Pareto
   frontier (a vector); ours collapses to one record.
3. **No-argument per-visit API.** Like `LengthHistoryTracker`, the
   tracker binds its source-of-truth state pointers at construction
   and reads them on every call. The two pieces of data the prune
   logic needs — current_prefix_score and best_so_far_score —
   come from the working and best forward register trackers
   respectively. Call site is just
   `pressure_history_.IsDominatedElseRecord()`.
4. **Fast-forward via injected callback** (chosen for the
   templating reason above).

The metric is the continuous occupancy score from
`kMaximizeContinuousRegisterOccupancyScore` — higher is better.
"Best" means `max`, and total = `min(prefix_score, postfix_score)`
— the schedule's score is bottlenecked by the worse half. This
matches what `DfsMaximizeOccupancyPolicy::kMetric` already
optimizes; no translation between metrics.

`INT_MIN` is the postfix sentinel: an entry whose subtree was
explored without ever recording a completion below it. This
sentinel is load-bearing — §6.4 explains how it implicitly
handles the "subtree fully pruned" case without a separate
boolean flag.

### 4.4 Forward register tracker extension

Forward tracker records pressure-after-each-instruction in a
vector. On `Schedule`: push current pressure. On `Unschedule`:
pop. At completion, the vector has N values; suffix-max gives
postfix peaks for all partitions on the path.

```cpp
class GCNForwardRegisterTracker {
  // existing members...
  void Schedule(ScheduleNode *node) {
    // existing live-set update
    int pressure_after = LiveCount();
    pressure_history_.push_back(pressure_after);   // NEW
    peak_ = std::max(peak_, pressure_after);
  }

  void Unschedule(ScheduleNode *node) {
    pressure_history_.pop_back();                  // NEW
    // existing live-set restore + peak undo
  }

  ArrayRef<int> GetPressureHistory() const { return pressure_history_; }

private:
  SmallVector<int, 64> pressure_history_;          // NEW
};
```

Memory cost: O(N) ints per `ScheduleConstructor`. We have two
(`working_`, `best_`), so ~2*N ints total. For N=20k, ~160 KB. Fine.

The recorded pressure also makes proxy handling clean: proxies
have no reg defs/uses, so pressure-after-proxy equals
pressure-after-previous-real-instruction. Suffix-max naturally
absorbs these duplicate values without affecting peaks.

### 4.5 Why no separate backward register tracker

Earlier drafts considered a separate `GCNBackwardRegisterTracker`
that walks the completed schedule from end to start to compute
postfix peaks. Discarded in favor of recording forward pressures
because:

- The forward tracker already computes per-instruction pressure;
  storing it is a one-liner.
- Suffix-max over an int array is simpler and faster than running
  a second tracker.
- One tracker (not two) means no convention-mismatch risk between
  forward and backward pressure semantics.
- Proxies fall out automatically (zero-delta entries in the
  recorded array).

---

## 5. Length History

### 5.1 The problem

Two prefixes A and B with the same scheduled set can have:
- Different `end_cycle` (= cycle of last scheduled instruction).
  Example: same set {A, B, C} with different orders gives
  different end cycles when latencies and bubbles differ.
- Different frontier LBs. Example: scheduling pred P early gives
  successor S a lower LB; scheduling P late gives S a higher LB.

The postfix problem from each of A and B faces these as starting
constraints. A's postfix can do everything B's can do (or better)
iff A's `end_cycle` ≤ B's AND A's frontier LBs are pointwise ≤ B's.

So length dominance is multi-dimensional: one dimension for
`end_cycle`, plus one per frontier node.

### 5.2 Pareto frontier of entries

For a single partition, multiple prefixes may be incomparable on
this multi-dimensional space (each better on some dimensions,
worse on others). We need to keep all incomparable entries to
prune as much as possible, but we don't need to keep entries that
are dominated by another entry in the same bucket.

So: maintain a Pareto frontier per bucket. On insert:
1. Walk bucket. If any existing entry dominates the new one,
   prune the visit (don't add).
2. Otherwise, walk bucket and discard entries dominated by the new
   one.
3. Add the new entry.

Pareto frontier sizes typically stay small (handful of entries) for
low-dimensional cases, though worst case can grow. Memory cap with
fatal-error provides a backstop.

### 5.3 Soundness

If prefix A dominates prefix B (same partition, A's `end_cycle` ≤
B's, A's frontier LBs pointwise ≤ B's), then for every postfix
ordering Q, the schedule [A's ordering] ++ Q has length ≤ the
schedule [B's ordering] ++ Q has length. Specifically:
- Cycle assignments in Q from A's prefix: each Q-instruction's
  earliest cycle ≤ what it would be from B's (since frontier LBs
  bound the cascade of cycle assignments through the postfix).
- Total length = max scheduled cycle ≤ B's total.

So the best-completion of A ≤ best-completion of B. Pruning B
loses nothing.

(Note: this assumes A's prefix is a valid prefix that we'd actually
extend. Insert-on-visit gives us this — A was visited and is in
the table, so the search did consider extending it.)

### 5.4 Frontier-LB comparison: full frontier, per-edge skip

The dominance check compares A's and B's frontier LBs. Because
frontier LBs cascade — if a scheduled pred P is at the same cycle
in A and B, it contributes the same LB to its successor — we only
need to do work for preds that are scheduled at different cycles.

In practice this is "all preds that were scheduled above their
static LB" — preds at their static LB are at the same cycle in
any prefix that schedules them, contributing invariantly.

Initial implementation: just walk the full frontier and compare.
Optimize later if profiling shows frontier comparison is hot.

### 5.5 Insert timing

Insert-on-visit. Every time the search reaches a non-pruned prefix,
record it in the table. Two reasons:

- The history entry only stores prefix-level info (`end_cycle` and
  frontier LBs). Both are known at visit time; no need to wait.
- Insert-on-visit lets sibling/cousin subtrees compare against the
  current prefix even before our subtree completes.

OptSched does insert-on-backtrack to also record the optimal
suffix, which enables additional prunes. We can revisit this if
prefix-only domination doesn't extract enough speedup.

### 5.6 Pruning with respect to bound-pruned prefixes

If P1 visited partition X and was bound-pruned (without exploring
its subtree), P1 is still in the table. If P2 later visits X with
P2 dominated by P1, P2 is pruned by dominance against P1.

This is sound: P1's prefix-level info (end_cycle, frontier LBs)
fully describes what P1 would have explored. Any completion
reachable from P2 is also reachable from P1's prefix at no greater
cost. We've already chosen not to explore P1's subtree (because of
the bound prune); choosing not to explore P2's subtree is
consistent with that choice.

---

## 6. Pressure History

### 6.1 Why pressure is fundamentally different

For a partition X, the **boundary live set** is exactly determined:
it's the data edges crossing the partition cut. Two prefixes with
the same partition have the same boundary live set.

Once the partition is fixed:
- Prefix score depends on prefix order.
- Postfix score depends on postfix order.
- These are independent: prefix order doesn't constrain postfix
  order (the boundary fully encapsulates the prefix→postfix data
  flow).

This decoupling means we don't need a Pareto frontier per
partition — a single best-prefix-score and best-postfix-score per
partition captures everything the search can use.

### 6.2 Score-based metric (continuous occupancy)

Earlier drafts framed pressure history in terms of "peak pressure"
(lower = better, total = `max(prefix_peak, postfix_peak)`). The
implementation has converged on the **continuous occupancy score**
instead, because it matches what `DfsMaximizeOccupancyPolicy::kMetric`
already optimizes (`kMaximizeContinuousRegisterOccupancyScore`).
A single metric runs end-to-end through the policy, the
per-instruction recording on the forward register tracker, the
prune logic, and the cost/best comparisons in `DfsSearch`.

In score form:
- Higher score is better; "best" means `max`.
- Total score for a complete schedule = `min(prefix_score,
  postfix_score)`. A schedule is bottlenecked by whichever half
  has the worse score.
- Prefix dominance becomes "prior entry's prefix score `>=` ours"
  (instead of `<=` in the peak framing).
- Total-bound prune becomes "the best total reachable from this
  prefix caps at `<= best_so_far_score`".

The decoupling argument (§6.1) carries over: postfix score is
partition-determined regardless of prefix order, so prefix and
postfix can be optimized independently per partition.

### 6.3 Per-partition entry

```cpp
struct Entry {
  int best_prefix_score;            // higher = better
  int best_postfix_score = INT_MIN; // sentinel; see §6.4
  ScheduleNode *next_node_hint = nullptr;  // see §6.6
};
```

One value per partition (no Pareto frontier, no per-partition
SmallVector). Cross-partition signature collisions are handled by
`DenseMap` open-addressing transparently.

### 6.4 Pruning logic, fast-forward, and the INT_MIN sentinel

The combined `IsDominatedElseRecord()` call dispatches on
five cases. Let X be the partition the working prefix has reached,
and let `cur` = `current_prefix_score`.

1. **No prior entry at X.** Insert a new entry with
   `best_prefix_score = cur`, `best_postfix_score = INT_MIN`,
   `next_node_hint = nullptr`. Outcome: continue exploring
   normally (no prune, no replay).
2. **Prior `best_prefix_score >= cur`** (prefix dominance). A
   prior visit was no worse on the only prefix-dependent
   dimension. By the decoupling argument, anything our subtree
   could reach is reachable (at no worse score) from the prior
   visit. Outcome: prune.
3. **Prior `best_postfix_score != INT_MIN`** (real postfix
   completion was recorded below this partition) and
   `min(cur, prior.best_postfix_score) <= best_so_far_score`.
   The total reachable score from our prefix is upper-bounded
   by that `min`; if it can't beat the running best, no
   completion below us improves the result. Outcome: prune.
4. **Prior `best_postfix_score != INT_MIN`** and the total bound
   in case 3 does NOT fire AND prior has a non-null
   `next_node_hint`. We're not dominated, the recorded path
   below this partition can in principle beat best-so-far, and
   we know the next step on a path that previously reached a
   real completion. Update `best_prefix_score = max(prior, cur)`.
   Outcome: continue, AND request fast-forward by invoking
   `enqueue_for_replay_(prior.next_node_hint)` so DFS schedules
   the hint next (see §6.6). The callback may silently no-op
   (returning `false`) if a multi-element replay is already in
   flight — case 4 still completes successfully; the hint is
   just dropped because the queue's existing contents are
   already driving DFS.
5. **Otherwise** (we're not dominated, no useful postfix info to
   ceiling us out, no hint to follow). Update
   `best_prefix_score = max(prior, cur)`. Outcome: continue
   normally.

`IsDominatedElseRecord` returns plain `bool prune` —
`true` for cases 2 and 3, `false` otherwise. The fast-forward
side effect in case 4 happens via the `enqueue_for_replay_`
callback the tracker holds (see §4.3 for why the tracker holds
a callback rather than a `DfsSearch` handle). `Policy::ShouldBoundSearch`
keeps a uniform `bool`-returning prune-predicate signature
across all policies; pressure-tracker-specific signaling lives
inside the tracker.

**The INT_MIN sentinel is load-bearing.** It implicitly handles
the "subtree was fully explored, no completion found" case
without an explicit boolean.

Why this works in single-threaded DFS: when a future visitor B
arrives at X, any prior visitor A's exploration of X's subtree
is complete (A backtracked before B got to X). Two outcomes for A:
- A found at least one completion below X →
  `best_postfix_score` is set to a real value.
- A's subtree fully pruned (every path hit a prune condition,
  no `IsDone` ever fired) → `best_postfix_score` stays at
  INT_MIN.

In the fully-pruned case, A pruned at some depth `k` of suffix
`Q` because A's working score (`min(A_prefix,
postfix_running_score(Q, k))`) dropped to `<= best_so_far_A`.
For pure pressure, postfix_running_score is partition-determined
under a fixed boundary live set — it's the same function of `Q`
regardless of which prefix entered X. In a maximization search
`best_so_far` is monotone non-decreasing, so `best_so_far_B >=
best_so_far_A`. B's working score at the same depth = `min(
B_prefix, postfix_running_score(Q, k)) <= postfix_running_score(
Q, k) <= best_so_far_A <= best_so_far_B`. B prunes too on the
same path. The argument applies to every path in X's subtree.

The math falls out automatically through case 3:
`min(cur, INT_MIN) = INT_MIN <= best_so_far_score` for any
finite running best, so case 3 fires and B is pruned without
exploration. No "fully pruned" boolean needed.

### 6.5 Postfix recording at completion (backward walk)

When DFS reaches `IsDone()`, the working prefix is a complete
schedule. `RecordPostfixScoresFromCompletedSchedule()` walks
`schedule_order_` and updates the history table for every
partition along the path:

For each step `k` from 0 to N (representing the partition
reached after scheduling the first `k` nodes):
- Compute the partition X' incrementally (each step adds one
  node's contribution to the partition signature).
- Compute the postfix score for X' via suffix-extreme over the
  forward register tracker's `pressure_history_` (the
  per-instruction record added in Phase 3).
- Update `entry.best_postfix_score = max(prior, postfix_score)`.
- Set `entry.next_node_hint = schedule_order_[k]` — the node
  scheduled NEXT from X' along this completion's path.

The walk is O(N) per completion. Forward register tracker's
recorded pressure provides the data; one suffix scan gives all
postfix scores along the path.

### 6.6 Fast-forward via DfsSearch replay queue

Fast-forward (case 4 in §6.4) lets the search skip ready-list
iteration at partitions where we already have a known-good next
step.

Mechanism: a generic replay queue on `DfsSearch`:

```cpp
class DfsSearch {
 private:
  std::queue<const ScheduleNode *> replay_queue_;
 public:
  /// Generic interface: a client preloads a known-good path into
  /// the queue, and Recurse drives along it. Returns true on
  /// success; returns false (no-op) if the queue is non-empty —
  /// see "no-op-on-non-empty invariant" below.
  bool EnqueueReplay(ArrayRef<const ScheduleNode *> path);
};
```

#### No-op-on-non-empty invariant

`EnqueueReplay` only inserts when the queue is **already empty**.
If the queue is non-empty when the call arrives, `EnqueueReplay`
returns `false` and inserts nothing. The pressure tracker's
case-4 lambda follows the same policy.

This invariant is not just hygiene — it prevents a real
correctness bug:

- Suppose the (future) `SubgraphInfo` cache loads a multi-element
  known-good schedule into the queue when a `proxy_start` is
  scheduled: `EnqueueReplay({n1, n2, n3, ...})`.
- `Recurse` pops `n1`, schedules it, recurses.
- The recursive call's `IsDominatedElseRecord` reaches
  case 4 and calls `enqueue_for_replay_(nx)` — push `nx` onto
  the queue.
- Without the invariant, the queue becomes `{n2, n3, ..., nx}`.
- `Recurse` pops `n2` (not `nx`!) and schedules it. Eventually
  the queue drains, the cache replay finishes, and the tail
  `nx` (which is no longer the right next step at the
  current-partition X') gets popped and scheduled. Now we're
  scheduling a node out of context — possibly a node already
  scheduled, or one not in the current ready list. Bad.

With the invariant, the case-4 enqueue silently drops when a
multi-element replay is in flight. The cache client's known-good
path drains uninterrupted, and the pressure tracker resumes
fast-forwarding on its own once the queue is empty again.
Returning `false` rather than asserting is deliberate: the
collision between two queue clients is a normal interaction, not
a programming error.

Concretely, DfsSearch's constructor binds the tracker callback
to a closure that returns `bool` (whether the push happened):

```cpp
// In DfsSearch's constructor, the lambda for the tracker:
[this](ScheduleNode *n) -> bool {
  if (replay_queue_.empty()) {
    replay_queue_.push(n);
    return true;
  }
  // A multi-element replay is in flight; silently skip.
  return false;
}
```

(The lambda's `bool` return value lets the tracker count
fast-forward attempts that landed vs. were skipped, useful for
shakedown stats. Pruning behavior is independent of the return
value — the case 4 path doesn't depend on whether the enqueue
actually happened.)

`EnqueueReplay`'s body is similarly non-fatal:

```cpp
bool DfsSearch::EnqueueReplay(ArrayRef<const ScheduleNode *> path) {
  if (!replay_queue_.empty()) {
    return false;
  }
  for (const ScheduleNode *n : path) {
    replay_queue_.push(n);
  }
  return true;
}
```

Single-step pressure-tracker enqueues are inherently
empty-queue-safe in normal operation: `Recurse` pops before
recursing, so the recursive `IsDominatedElseRecord` sees
an empty queue. The lambda's empty check is a no-op in the
normal case; it only matters when a multi-element cache replay
is interleaved with normal exploration.

#### Recurse driving the queue

`Recurse()` checks the queue first; if non-empty, it pops and
schedules the front node, skipping the ready-list loop entirely:

```cpp
void Recurse() {
  if (working_.IsDone()) {
    OnCompletion(...);
    replay_queue_ = {};   // safety: drop any stale tail
    return;
  }
  if (Policy::ShouldBoundSearch(working_, best_,
                                length_history_, pressure_history_)) {
    replay_queue_ = {};
    return;
  }
  if (!replay_queue_.empty()) {
    const ScheduleNode *next = replay_queue_.front();
    replay_queue_.pop();
    working_.Schedule(next);
    Recurse();
    working_.Unschedule();
    return;
  }
  // Normal ready-list iteration ...
}
```

When the pressure tracker's case 4 fires in normal exploration,
the lambda enqueues exactly ONE node (the hint at this
partition). DFS pops it on the next `Recurse`, schedules it,
advances to the new partition X', possibly enqueues again from
the deeper `IsDominatedElseRecord`, and so on. The tracker
doesn't walk the chain itself — DFS chains the calls naturally
via recursion, one partition at a time. If a hint chain leads
to a prune (e.g., best_so_far has improved since the original
recording, and case 3 now fires at a deeper partition), the
queue is drained and DFS resumes normal exploration above the
prune point on the next backtrack.

The replay queue is a generic mechanism on `DfsSearch`, not a
pressure-tracker-specific construct. Its main intended future
client is subgraph schedule caching: when a `proxy_start` is
scheduled, the `SubgraphInfo` can pre-load the subgraph's
known-good schedule via `EnqueueReplay` and let `Recurse` walk
it without per-step ready-list iteration — and the
no-op-on-non-empty invariant is what makes that interaction safe
alongside the pressure tracker.

### 6.7 Soundness

**Prefix dominance** (case 2) is sound because total =
`min(prefix_score, postfix_score)` and an improvement on the
prefix half cannot make the total worse for any postfix
(postfix_score is partition-determined by the decoupling
argument).

**Total-bound prune** (case 3, finite postfix) is sound because
both `cur` and `prior.best_postfix_score` are valid upper bounds
on what the search can score from this prefix; if `min` of those
is `<= best_so_far_score`, no completion below this prefix can
improve on the running best.

**Total-bound prune** (case 3, INT_MIN postfix) is sound by the
monotone-best_so_far argument in §6.4.

**Fast-forward** (case 4) is sound because the hinted node is a
valid ready-list element at this partition (it was scheduled
there in the recorded completion), and following the hint to a
sequence of partitions where each entry's recorded postfix info
is consulted preserves the prune decisions DFS would otherwise
make at every step. The total-bound check still fires along the
hint chain; fast-forward only collapses the work of choosing
among ready-list alternatives at partitions whose hint is
non-null. A dropped enqueue (case 4 with a non-empty queue) is
also sound: case 4 returns "continue normally" with respect to
DFS's prune decision; the only effect of the dropped enqueue is
forgoing one fast-forward step. Correctness doesn't depend on
fast-forward firing.

### 6.8 Comparison with OptSched

OptSched's history-domination implementation
(`OptSched/lib/Scheduler/{enumerator,hist_table}.cpp` and
`OptSched/include/.../hist_table.h`) is the reference design
for the technique. Contrasting our pressure tracker against it
makes the deltas explicit.

**OptSched does:**
- Insert-on-backtrack — entries are inserted only after the
  subtree below them has been fully explored.
- Stores a suffix sequence per entry as
  `std::shared_ptr<std::vector<SchedInstruction *>> suffix_` on
  `HistEnumTreeNode`. The `shared_ptr` means multiple history
  entries can share the same suffix vector when their cached
  paths converge, but conceptually each entry has its own
  handle to "the known-good suffix from this partition."
- Two-stage dominance check (structural + cost), with cost-
  function-specific pruning rules.
- One combined history table for all metrics (length + spill +
  pressure together) — a single pass with a combined cost.
- `BinHashTable` with chained linked lists; chains accumulate
  (no Pareto trim on insert).
- `DoesDominate` iterates the chain backward and stops at the
  first match. Replay uses the dominator's `suffix_`.

**Our design differs:**
- **Insert-on-visit for prefix entries.** Simpler; works in
  single-threaded DFS where the prior visitor is always done by
  the time a future visitor arrives. The INT_MIN sentinel +
  monotone-`best_so_far` argument in §6.4 covers what OptSched
  uses insert-on-backtrack to express explicitly.
- **Two separate trackers** (length, pressure) instead of one
  combined cost table, because we run two separate single-
  objective passes (length DFS and occupancy DFS) rather than a
  single combined-cost pass.
- **`LengthHistoryTracker` keeps a Pareto frontier per partition**
  (pressure-trimmed on insert) — bounded chain length, simpler
  than OptSched's accumulated chains.
- **`PressureHistoryTracker` keeps one entry per partition.**
  Pressure is one-dimensional via the continuous occupancy
  score; the Pareto-frontier complication doesn't arise.
- **Fast-forward via "next node hint"** (one `ScheduleNode *`
  per partition) plus DFS-level recursive chaining via the
  replay queue, instead of OptSched's per-entry suffix vectors.
  Storage is O(1) per partition rather than O(suffix length).
  Equivalent pruning power because the hint plus follow-up
  entries collectively determine the same suffix; we just store
  it distributively across partitions instead of caching a full
  copy on each entry. (OptSched's `shared_ptr` deduplication
  reduces total storage when paths converge, but each entry
  still owns a handle.)

**Why our pure-length tracker has no postfix info or fast-forward:**
For pure length minimization, the optimal postfix from a
partition depends on the prefix's frontier_lbs (because cycle
assignments in the postfix cascade from those LBs). A single
best suffix recorded from one prefix's frontier is lossy when
applied to a different prefix's frontier in the same partition.
OptSched's combined cost masks this — their cached suffix is a
*heuristic* under combined cost, not a sound optimal-completion
replay. For our pure-length pass, sound pruning requires the
Pareto-frontier-on-prefix only; no postfix caching. Pressure,
by contrast, has partition-determined postfix peaks (decoupling
§6.1), so postfix caching IS sound and clean — and that's why
fast-forward lives only on `PressureHistoryTracker`.

### 6.9 Possible novelty (unverified)

The decoupling property — "given a partition, prefix and postfix
register pressure can be optimized independently" — is well-known
register-pressure scheduling theory; foundational to Sethi-Ullman
numbering for trees and standard in DAG scheduling literature.
History-based domination as a B&B technique comes from Shobaki
(MICRO 2004 onward) via OptSched.

The specific combination — applying this decoupling within
Shobaki's history-domination framework with two scalars per
partition (`best_prefix_score`, `best_postfix_score`) plus the
INT_MIN-sentinel-implicitly-handles-fully-pruned-subtree
observation, plus fast-forward via per-partition next-node hint
and a generic DfsSearch replay queue — is one we have not found
explicitly documented. OptSched's investigated code
(`CostHistEnumTreeNode` in `hist_table.cpp`) tracks aggregate
cost / partial cost / peak cost across the whole schedule, not
separated by prefix/postfix sides. This may be:

- (a) implicit in OptSched but missed in our investigation,
- (b) in another Shobaki paper we have not read,
- (c) considered too obvious to publish, or
- (d) genuinely a small refinement.

Worth a focused literature pass at some point before claiming
novelty. If it turns out to be a real refinement, worth a writeup.
See the corresponding TODO entry in section 10.8 of
`AMDGPUMachineSchedulerGuide.md`.

---

## 7. Proxy Handling

Subgraph formation introduces proxy nodes (start/end pairs per
formed subgraph). They appear in `schedule_order_` and participate
in scheduling decisions but don't issue cycles or have register
defs/uses.

### 7.1 Why include proxies in everything

Two prefixes with the same real-instruction set but different
proxy state are at **different search states**:

Example: subgraph S = {I2, I3}.
- A: [I1, P_start_S, I2, I3, P_end_S]
- B: [I1, P_start_S, I2, I3]

Both have real-instruction set {I1, I2, I3}, but A has popped the
subgraph scope (P_end fired) while B hasn't. A's frontier includes
external successors of S (now ungated); B's doesn't (still gated
by `P_end → ext_succ` artificial edges).

If we excluded proxies from the signature, A and B would compare
as "same partition" but their frontier shapes differ — comparison
logic would have to reconcile differently-sized frontier maps.
More complex than necessary.

If we include proxies, A and B have different signatures and are
treated as distinct nodes in the search. B will eventually
schedule P_end and reach A's state, at which point dominance fires
normally. We don't miss the prune; we just defer it by one DFS
step.

Include proxies in: signatures, scheduled bitset, frontier
tracking, pressure history recording. `ScheduledSetTracker` handles them
uniformly with no special-casing.

### 7.2 Special cases that fall out cleanly

- **End cycle for length history**: proxies don't advance cycle, so
  `current_cycle` stays at the last real instruction's cycle. The
  existing length tracker handles this; no change needed.
- **Boundary live count for pressure**: computed from data edges
  crossing the partition. Proxies are connected only by
  `kSubgraphOrderEdge` (artificial, non-latency), so they never
  contribute to the boundary live set. Whether proxies are in the
  scheduled set doesn't change boundary count.
- **Postfix peak from forward pressure history**: pressure-after-
  proxy equals pressure-after-previous-real-instruction. Suffix-max
  naturally handles these as duplicate values that don't shift any
  peak.
- **Frontier behavior**: P_start enters the frontier when its
  ext_preds are scheduled; gets removed when scheduled. Members
  enter the frontier when their gating P_start → member edge is
  satisfied. P_end enters the frontier when all members are
  scheduled. ext_succs enter when P_end is scheduled. All of this
  is the existing scope mechanism doing its job;
  `ScheduledSetTracker` observes via `Schedule`/`Unschedule`
  notifications.

---

## 8. DfsSearch Integration

### 8.1 Policy contract

Two new boolean flags on `SearchPolicyBase`, defaulting to false.
Concrete policies override to opt in:

```cpp
class SearchPolicyBase {
public:
  static SubgraphFormationPolicy MakeFormationPolicy() { return {}; }
  static constexpr bool kUseLengthHistoryPruning = false;
  static constexpr bool kUsePressureHistoryPruning = false;
};

class DfsMinimizeLengthPolicy : public SearchPolicyBase {
  // existing methods (kMetric, ReadyCompare, ShouldEndSearch,
  // MakeFormationPolicy).
  static constexpr bool kUseLengthHistoryPruning = true;

  // ShouldBoundSearch now takes the length history tracker by
  // non-const ref. Body internally gates the history-pruning
  // call with `if constexpr (kUseLengthHistoryPruning)`. Single
  // prune-decision API, mutating side effect (records the prefix)
  // documented in the function's contract.
  static bool ShouldBoundSearch(
      const ScheduleConstructor &working,
      const ScheduleConstructor &best,
      LengthHistoryTracker &length_history);
};

class DfsMaximizeOccupancyPolicy : public SearchPolicyBase {
  // existing methods
  static constexpr bool kUsePressureHistoryPruning = true;

  // Same shape as the length policy, but consults the
  // PressureHistoryTracker. (Phase 4b folds this in; for now
  // it just declares the flag.)
  static bool ShouldBoundSearch(
      const ScheduleConstructor &working,
      const ScheduleConstructor &best,
      LengthHistoryTracker &length_history,
      PressureHistoryTracker &pressure_history);
};
```

**Test policies override `ShouldBoundSearch` directly.** For
shakedown comparison tests (e.g., `TestLengthPolicyNoBoundsNoHistory`,
`TestLengthPolicyNoBoundsWithHistory`), test variants inherit
from the production policy and override `ShouldBoundSearch` with
their own body. They omit LB / occupancy bounds and choose
whether to call the history tracker explicitly. Code duplication
of the body is the cost; clarity and test-isolation is the win.

CRTP was considered to allow "inherit + toggle flag" without
rewriting the method body. Rejected: derived classes overriding
the flag don't change behavior of inherited static methods'
bodies (C++ name lookup binds inside the defining class). CRTP
fixes this but adds template machinery for one-off test
ergonomics. Direct override is simpler given how few test
variants we have.

### 8.2 DfsSearch shape

```cpp
template<typename Policy>
class DfsSearch {
public:
  DfsSearch(ScheduleGraph &graph, ...)
      : working_schedule_constructor_(...),
        best_schedule_constructor_(...),
        length_history_(
            &working_.GetScheduledSetTracker(),
            &working_.GetLengthTracker()),
        pressure_history_(
            &working_.GetScheduledSetTracker(),
            &working_.GetForwardRegisterTracker(),
            &best_.GetForwardRegisterTracker(),
            [this](const ScheduleNode *n) {
              return this->EnqueueReplay(n);
            }) {}

  // Generic single-node enqueue for clients (PressureHistoryTracker
  // for fast-forward; future SubgraphInfo for cached subgraph
  // schedules). Queue-management policy lives here, not in any
  // caller's lambda.
  bool EnqueueReplay(const ScheduleNode *node);

private:
  void Recurse() {
    if (working_.IsDone()) {
      // existing best-update logic
      if constexpr (Policy::kUsePressureHistoryPruning) {
        pressure_history_.RecordPostfixScoresFromCompletedSchedule();
      }
      replay_queue_ = {};   // safety: drop any stale tail
      return;
    }

    // ShouldBoundSearch is the single prune-decision API. The
    // policy folds history pruning inside its body via
    // `if constexpr (Policy::kUseLengthHistoryPruning)` (and the
    // pressure equivalent). The policy may MUTATE the history
    // tracker(s) as part of the check (record-on-miss-prune,
    // request fast-forward via callback). One return point.
    if (Policy::ShouldBoundSearch(working_, best_,
                                  length_history_,
                                  pressure_history_)) {
      replay_queue_ = {};
      return;
    }

    // Replay queue: when non-empty, schedule the front node and
    // recurse, skipping the ready-list loop. Loaded by
    // EnqueueReplay calls (e.g., from PressureHistoryTracker's
    // fast-forward).
    if (!replay_queue_.empty()) {
      const ScheduleNode *next = replay_queue_.front();
      replay_queue_.pop();
      working_.Schedule(next);
      Recurse();
      working_.Unschedule();
      return;
    }

    // Normal per-child loop.
    int ready_size = working_.GetReadyList().size();
    for (int i = 0; i < ready_size; ++i) {
      working_.ScheduleByIndex(i);
      Recurse();
      working_.Unschedule();
      if (should_end_search_) {
        replay_queue_ = {};
        return;
      }
    }
  }

  ScheduleConstructor working_, best_;
  LengthHistoryTracker length_history_;
  PressureHistoryTracker pressure_history_;
  std::queue<const ScheduleNode *> replay_queue_;
};
```

The Recurse body has exactly two return-via-prune sites: IsDone
(after best update) and ShouldBoundSearch. History pruning lives
inside the policy's ShouldBoundSearch (gated by `if constexpr`
on the pruning flags), not as a separate block. Phase 4c adds
postfix recording at IsDone and the replay queue.

### 8.3 Memory cap

Each `*HistoryTracker` enforces `max_entries`. On insert that
would exceed the cap, `report_fatal_error`. LRU eviction is a
future option; fatal-error makes resource exhaustion loud rather
than silent.

Initial default: 100k entries per tracker per search. Tunable via
constructor parameter. Production callers may set lower for
memory-constrained environments.

---

## 9. Worked Example

Consider a 4-instruction graph: A, B, C, D. Latencies all 1.
Edges: A→B, A→C, B→D, C→D.

Topological orders that DFS could explore:
- A, B, C, D (or A, C, B, D — same partition at depth 2).

After scheduling A then B at depth 2, scheduled set = {A, B}.
After scheduling A then C at depth 2, scheduled set = {A, C}.
These are different partitions.

After scheduling A, B, C at depth 3, scheduled set = {A, B, C}.
Same partition reached two ways: {A→B→C} and {A→C→B}.

### 9.1 Length history at depth 3

Suppose A→B path schedules A=0, B=1, C=2 (B and C in parallel
slots — C waits 1 cycle for A's latency 1):
- end_cycle = 2
- Frontier = {D}. LB(D) from B at cycle 1 + lat(B→D)=1 = 2. From C
  at cycle 2 + lat(C→D)=1 = 3. Combined LB(D) = max(2, 3) = 3.

Suppose A→C path schedules A=0, C=1, B=2 (similar):
- end_cycle = 2
- Frontier = {D}. LB(D) from C at cycle 1 + 1 = 2. From B at cycle
  2 + 1 = 3. Combined LB(D) = max(2, 3) = 3.

Same partition, same end_cycle, same frontier LBs. Either entry
dominates the other (they're equal); the second visit gets pruned.

### 9.2 Length history with frontier-LB difference

Add one more edge: A→D directly with latency 5. Now D is in the
frontier from depth 1 onward.

A→B→C path: A=0, B=1, C=2. Frontier(D) at depth 3:
- From A at cycle 0 + 5 = 5
- From B at cycle 1 + 1 = 2
- From C at cycle 2 + 1 = 3
- LB(D) = max(5, 2, 3) = 5.

A→C→B path: A=0, C=1, B=2. Same LB(D) = 5.

Same partition, same end_cycle, same frontier. Dominance prunes
the second visit.

### 9.3 Length history with bubble difference

Add edge B→C with latency 3. Now after B at cycle 1, C must wait
until cycle 4.

A→B→C: A=0, B=1, C=4. end_cycle = 4. Frontier(D):
- From A: 5, from B: 2, from C: 5. LB(D) = 5.

A→C→B: A=0, C=1, B=2 (C only depends on A; B can issue at 2).
end_cycle = 2. Frontier(D):
- From A: 5, from C: 2, from B: 3. LB(D) = 5.

Same partition. Different end_cycle (4 vs 2). Frontier LBs equal.
A→C→B dominates A→B→C: better end_cycle, same frontier LBs.

If A→B→C visits first, it gets recorded. When A→C→B visits, it
finds A→B→C in the table, but A→C→B isn't dominated (better
end_cycle). The new entry is added. A→B→C is now dominated by the
new entry — discarded. Bucket holds only A→C→B.

### 9.4 Pressure history

Suppose all values written by A, B, C, D are virtual registers.
After scheduling A and B (depth 2), the boundary live set is
{value_from_A_used_by_C, value_from_B_used_by_D} — assuming both
are alive. Boundary = 2.

A→B path: prefix peak might be 2 (A defines, holds, B defines).
A→C path: prefix peak might also be 2.

For partition {A}, two visits. Both have prefix_peak = 1
(only A scheduled).

For partition {A, B}: prefix_peak in this prefix might be 2.
For partition {A, C}: same.

Different partitions; each has its own entry.

For partition {A, B, C}, two visits possible:
- Via A→B→C: prefix_peak = max over scheduling steps.
- Via A→C→B: same.

If both prefixes have prefix_peak = 2, no dominance fires (equal).
Either entry is fine.

If A→B→C had prefix_peak = 3 (some unfortunate ordering of defs)
and A→C→B had prefix_peak = 2, then on the second visit the
entry's `best_prefix_peak` is updated from 3 to 2.

Now suppose a third visit arrives at {A, B, C} with prefix_peak =
4. Prefix-dominance fires: 4 > 2. Prune.

---

## 10. Implementation Phases

Reorganized so length history goes end-to-end before pressure work
begins. Tracker classes and their DFS wiring are split into
separate phases so each tracker's logic can be tested in isolation
before behavior changes in production.

End-to-end checkpoints: after Phase 2b, length DFS uses
history-based pruning in production. After Phase 4b, occupancy DFS
does too.

Each phase commits independently. Shakedowns added per phase.

### Phase 1 — ScheduledSetTracker (DONE)

- New `ScheduledSetTracker.{h,cpp}` in
  `lib/Target/AMDGPU/HierarchicalScheduler/`.
- `ScheduledSetTracker` member on `ScheduleConstructor`, hooked
  via `Schedule`/`Unschedule` notifications.
- Exposes shared `PartitionKey` (signature + bitset) and
  `DenseMapInfo<PartitionKey>` for downstream history trackers.

Shakedowns: round-trip, order-invariance, frontier counts.

### Phase 2a — LengthHistoryTracker class (DONE)

Tracker built and exercised in isolation.

- New `LengthHistoryTracker.{h,cpp}`. Per-partition Pareto frontier
  of `(end_cycle, frontier_lbs)` entries. Map keyed by
  `PartitionKey`.
- API: `IsDominatedElseInsert()`, `IsDominated() const` (combined
  check + insert; one bucket lookup per call). Pareto maintenance
  internal.

Shakedowns: insert / strict dominator / Pareto trim / incomparable
co-existence / hash collision via hand-crafted PartitionKeys / etc.

### Phase 2b — Length DFS wiring (LENGTH END-TO-END) (DONE)

**Production behavior changed after this phase.**

- `LengthHistoryTracker length_history_` member on `DfsSearch`.
- Folded history-pruning call into
  `DfsMinimizeLengthPolicy::ShouldBoundSearch` via
  `if constexpr (kUseLengthHistoryPruning)`. Single prune-decision
  API in `Recurse`.
- `DfsMinimizeLengthPolicy::kUseLengthHistoryPruning = true`.

Shakedowns: synthetic-DAG comparison
(`BuildHistoryPruneTestDAG`) demonstrates same best length,
history's prune count > 0, and history calls strictly less than
no-history calls.

### Phase 3 — Forward register tracker records per-step pressure (DONE)

Prerequisite for Phase 4. Doesn't affect length history.

- `pressure_history_` vector on `GCNRegisterTracker`. Push on
  `Schedule`, pop on `Unschedule`. Includes proxy entries
  (duplicate of previous-real-instruction pressure; suffix-max
  absorbs).
- `GetPressureHistory()` accessor.

Shakedowns: per-step pressure matches `cur_pressure_` after each
Schedule on a real region; round-trip empties the vector.

Phase 4 is split into six sub-phases, paired by sub-feature
(prefix / postfix / replay) so each pair has one
"tracker logic + standalone tests" phase followed by one
"DFS wiring + integration tests + production behavior change"
phase. The split honors test-isolation: when a standalone phase
is green and an integration phase fails, the regression is in the
wiring, not the tracker logic.

### Phase 4a — Prefix tracker logic

Tracker built and exercised in isolation; not yet wired into
DfsSearch.

- New `PressureHistoryTracker.{h,cpp}`. Entry struct (full final
  shape — `best_postfix_score = INT_MIN` and
  `next_node_hint = nullptr` are placeholders until 4c/4e).
  `DenseMap<PartitionKey, Entry>` storage (no Pareto frontier;
  one entry per partition).
- Constructor binds `ScheduledSetTracker*`, working/best
  `GCNRegisterTracker*`, and an enqueue callback (no-op until
  Phase 4f).
- `IsDominatedElseRecord` implements cases 1 + 2 (insert on miss;
  prefix dominance prune). Cases 3 + 4 + 5 stubbed.
- Stats baked in: `GetTotalPruneCount`, `GetTotalEntries`.
  Test-only `InsertEntryForTest`, `GetEntryForTest`.

Shakedowns: empty-table behavior, first insert + self-dominance,
strict dominator pruning, distinct partitions get distinct
entries, hash collision via hand-crafted PartitionKeys.

### Phase 4b — Prefix E2E (DFS wiring)

**Production behavior changes after this phase.**

- Add `kUsePressureHistoryPruning` flag on `SearchPolicyBase`
  (default false).
- `DfsMaximizeOccupancyPolicy::ShouldBoundSearch` updated
  signature (takes both length and pressure trackers); folds
  prefix-dominance call gated by
  `if constexpr (kUsePressureHistoryPruning)`. Set the flag
  `true` on the policy.
- `PressureHistoryTracker pressure_history_` member on
  `DfsSearch`, constructed with appropriate bound pointers and
  a callback that for now is a no-op (replay queue arrives in
  4f).
- Production stat reporting: surface `GetTotalPruneCount`
  alongside other per-region stats.

Integration shakedown: comparison test (parallel to length's)
on a pressure-meaningful synthetic DAG. Verifies same best
occupancy and reduced `schedule_call_count` under prefix-
dominance pruning.

(Resolves the synthetic-DAG-with-meaningful-pressure question
deferred from Phase 4a — addressed when integration testing
needs it.)

### Phase 4c — Postfix tracker logic

Tracker logic for case 3 (total-bound prune) added; standalone
shakedowns extended.

- Add `RecordPostfixScoresFromCompletedSchedule` (backward walk
  over `schedule_order_` with suffix-extreme over
  `GCNRegisterTracker::GetPressureHistory()`; per-partition
  `best_postfix_score = max(prior, this_completion)`).
- Update `IsDominatedElseRecord` with case 3 (total-bound:
  `min(cur, prior.best_postfix_score) <= best_so_far → prune`).
  INT_MIN sentinel auto-handles fully-pruned subtree case via
  `min(cur, INT_MIN) = INT_MIN`.
- Standalone shakedowns: total-bound prune with INT_MIN (auto
  fully-pruned), total-bound prune with real recorded postfix,
  postfix-recording correctness across a hand-staged completed
  schedule.

### Phase 4d — Postfix E2E (DFS wiring)

**Production behavior changes after this phase.**

- Hook `RecordPostfixScoresFromCompletedSchedule` into Recurse
  at IsDone (gated by `kUsePressureHistoryPruning`).
- No new policy flag; the existing prefix flag now also
  activates postfix recording + total-bound prune.

Integration shakedown: extended comparison test verifies
total-bound prune additionally fires (e.g., expected
schedule_call_count drops below 4b's baseline).

### Phase 4e — Replay tracker logic

Tracker logic for case 4 (fast-forward via callback) added.

- Set `next_node_hint` during the same backward walk as 4c's
  `RecordPostfixScoresFromCompletedSchedule` (the node scheduled
  next from each partition along the completed path).
- Add case 4 to `IsDominatedElseRecord`: when not pruned and
  `next_node_hint != nullptr`, invoke the enqueue callback.
- Standalone shakedown: case 4 callback fires with the recorded
  hint when conditions met. Use a recording-fixture callback in
  test; verify it captures the right node.

### Phase 4f — Replay E2E (DFS replay queue)

**Production behavior changes after this phase.**

- Add `std::queue<const ScheduleNode*> replay_queue_` member to
  `DfsSearch`.
- Add `EnqueueReplay(const ScheduleNode*)` method on `DfsSearch`
  with the queue-management policy (e.g., no-op when queue is
  non-empty).
- Update Recurse: drain the queue (schedule front, recurse,
  unschedule, return) before the ready-list loop. Clear queue on
  prune / IsDone / end-search returns.
- Replace the no-op enqueue callback in `pressure_history_`'s
  ctor with a thin lambda delegating to `EnqueueReplay`.

Integration shakedown: extended comparison test verifies the
replay path is exercised (some recorded hint actually drives DFS
through the replay queue) and `schedule_call_count` reduces
further vs the 4d baseline.

### Phase 5 (optional sidequest) — Tracker enablement policy flags

Pure cleanup; can be done at any time, including not at all.

- Add tracker-construction flags to `SearchPolicyBase` (default
  false, override true on policies that consume the tracker).
- Gate `DfsSearch`'s tracker construction on the flags. Avoids
  the small construction overhead for passes that don't use a
  given tracker.

---

## Summary

History-based domination prunes B&B subtrees by recognizing that
multiple prefixes can reach the same scheduled set, and the search
only needs to follow the best one. The `ScheduledSetTracker`
(signatures, bitset, frontier) lives on `ScheduleConstructor` as a
self-contained object and exposes the shared `PartitionKey`
(signature + bitset) and `DenseMapInfo<PartitionKey>` that the
history trackers use as their map key. The history tables live on
`DfsSearch` because they accumulate across the search.

Length and pressure history are siblings, not derived from a
common base, because their dominance logic differs fundamentally.
Length needs a Pareto frontier per partition over multi-dimensional
entries (`end_cycle` + frontier LBs). Pressure has one entry per
partition (`best_prefix_score`, `best_postfix_score`,
`next_node_hint`) enabled by the partition fully decoupling
prefix and postfix register pressure.

The metric on the pressure side is the continuous occupancy score
(higher = better) — the same metric `DfsMaximizeOccupancyPolicy`
already optimizes. Total = `min(prefix_score, postfix_score)`.

Postfix score is derived from the forward register tracker's
recorded per-step pressure (suffix-extreme), avoiding a separate
backward tracker.

Proxy nodes from subgraph formation are treated uniformly with
real instructions in the `ScheduledSetTracker`: they get their
own signatures, appear in the scheduled bitset, and participate
in frontier tracking. Their pressure-recording entries duplicate
the previous real instruction's pressure (proxies have no
defs/uses), which suffix-extreme absorbs.

Policy contract follows the `MakeFormationPolicy` pattern: two
new `kUseLengthHistoryPruning` / `kUsePressureHistoryPruning`
flags, defaulting false, overridden true on the policies that
want them. The flags are consumed inside the policy's
`ShouldBoundSearch` body via `if constexpr`, folding history
pruning into the single prune-decision API. Test policies override
`ShouldBoundSearch` directly when they want different behavior
(no CRTP needed).
