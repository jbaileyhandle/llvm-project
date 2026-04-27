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
  ├── LengthHistoryTracker length_history_   NEW (only used if
  │                                              Policy::kUseLengthHistory)
  └── PressureHistoryTracker pressure_history_  NEW (only used if
                                                Policy::kUsePressureHistory)

SearchPolicyBase                             (extended)
  └── kUseLengthHistory, kUsePressureHistory
      bool flags, default false; concrete policies override true.
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

See §5 for the full design. Brief shape:

```cpp
class LengthHistoryTracker {
public:
  LengthHistoryTracker(int max_entries);

  bool ShouldPrune(const ScheduledSetTracker &tracker, int end_cycle) const;
  void Insert(const ScheduledSetTracker &tracker, int end_cycle);

private:
  struct Entry {
    BitVector scheduled_set;      // exact-match disambiguation
    int end_cycle;
    SmallVector<std::pair<int,int>, 16> frontier_lbs;  // sorted by topo_idx
  };
  DenseMap<int64_t, SmallVector<Entry, 2>> table_;
  int total_entries_ = 0;
  int max_entries_;
};
```

Per partition, the bucket holds a Pareto frontier of incomparable
entries. Bucket can also have entries from XOR signature collisions
across genuinely different partitions; bitset disambiguates.

### 4.3 PressureHistoryTracker

See §6. Brief shape:

```cpp
class PressureHistoryTracker {
public:
  PressureHistoryTracker(int max_entries);

  bool ShouldPrune(const ScheduledSetTracker &tracker,
                   int current_prefix_peak,
                   int ceiling) const;
  void RecordPrefixPeak(const ScheduledSetTracker &tracker,
                        int current_prefix_peak);
  void RecordPostfixPeakForPartition(const ScheduledSetTracker &tracker,
                                     int postfix_peak);

private:
  struct Entry {
    BitVector scheduled_set;
    int best_prefix_peak;
    int best_postfix_peak;       // ∞ until first completion
  };
  DenseMap<int64_t, SmallVector<Entry, 1>> table_;
  int total_entries_ = 0;
  int max_entries_;
};
```

One entry per partition. Bucket can have multiple entries only on
XOR collisions across genuinely different partitions.

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
- Prefix peak depends on prefix order.
- Postfix peak depends on postfix order.
- These are independent: prefix order doesn't constrain postfix
  order or pressure (the boundary fully encapsulates the
  prefix→postfix data flow).

Total peak = max(prefix peak, postfix peak). The boundary live
count is dominated by both peaks (registers crossing the boundary
are live during the prefix's last instruction AND during the
postfix's first instruction), so it doesn't add to the prune
calculation.

This decoupling means we don't need a Pareto frontier per
partition — we just need the best prefix peak and the best postfix
peak observed across all visits to this partition.

### 6.2 Per-partition entry

```cpp
struct Entry {
  BitVector scheduled_set;
  int best_prefix_peak;        // min across visited prefix orderings
  int best_postfix_peak;       // min across completed postfix orderings; ∞ initially
};
```

One entry per partition. Bucket-level chains exist only for XOR
signature collisions across truly different partitions.

### 6.3 Pruning logic

**Prefix dominance**: if `current_prefix_peak > best_prefix_peak(X)`
then a prior prefix at this partition was strictly better on the
only dimension that matters. For any postfix order, our total =
max(current_prefix_peak, postfix_peak) ≥ max(best_prefix_peak,
postfix_peak) = what the better prior prefix achieves. Prune.

**Total-bound prune**: if `max(current_prefix_peak,
best_postfix_peak(X)) ≥ ceiling`, no completion of this prefix can
stay below ceiling. Prune.

When `best_postfix_peak` is unset (∞), total-bound degenerates to
`current_prefix_peak ≥ ceiling`, which is the existing pre-history
pressure check — no regression.

### 6.4 Updates

- **On visit (`RecordPrefixPeak`)**: `best_prefix_peak(X) =
  min(best_prefix_peak(X), current_prefix_peak)`.
- **On completion (`RecordPostfixPeakForPartition`)**: for each
  partition X on the path to the current completion,
  `best_postfix_peak(X) = min(best_postfix_peak(X),
  postfix_peak_for_X)`.

The postfix peak for each partition along the completion path is
extracted via suffix-max over the forward tracker's recorded
pressure history. One O(N) sweep at completion gives postfix peaks
for all N partitions on the path.

### 6.5 Soundness

Prefix dominance is sound because total = max(prefix_peak,
postfix_peak) and a strict improvement on prefix_peak can only
improve total (postfix_peak is partition-determined regardless of
prefix order, by the decoupling argument).

Total-bound is sound because both `current_prefix_peak` and
`best_postfix_peak` are valid bounds on what the search can
achieve from this prefix.

### 6.6 Possible novelty (unverified)

The decoupling property — "given a partition, prefix and postfix
register pressure can be optimized independently" — is well-known
register-pressure scheduling theory; foundational to Sethi-Ullman
numbering for trees and standard in DAG scheduling literature.
History-based domination as a B&B technique comes from Shobaki
(MICRO 2004 onward) via OptSched.

The specific combination — applying this decoupling within
Shobaki's history-domination framework with two scalars
(`best_prefix_peak`, `best_postfix_peak`) per partition — is one
we have not found explicitly documented. OptSched's investigated
code (`CostHistEnumTreeNode` in `hist_table.cpp`) tracks
aggregate cost / partial cost / peak cost across the whole
schedule, not separated by prefix/postfix sides. This may be:

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
  static constexpr bool kUseLengthHistory = false;
  static constexpr bool kUsePressureHistory = false;
};

class DfsMinimizeLengthPolicy : public SearchPolicyBase {
  // existing methods (kMetric, ReadyCompare, ShouldBoundSearch,
  // ShouldEndSearch, MakeFormationPolicy)
  static constexpr bool kUseLengthHistory = true;
};

class DfsMaximizeOccupancyPolicy : public SearchPolicyBase {
  // existing methods
  static constexpr bool kUsePressureHistory = true;
};
```

### 8.2 DfsSearch shape

```cpp
template<typename Policy>
class DfsSearch {
public:
  DfsSearch(ScheduleGraph &graph, ..., bool form_subgraphs = true,
            int max_history_entries = kDefaultMax)
      : working_schedule_constructor_(...),
        best_schedule_constructor_(...),
        length_history_(max_history_entries),       // unused if !kUseLengthHistory
        pressure_history_(max_history_entries) {}   // unused if !kUsePressureHistory

private:
  void Recurse() {
    if (working_.IsDone()) {
      // existing best-update logic
      if constexpr (Policy::kUsePressureHistory) {
        UpdatePostfixPeaksFromCompletedSchedule();
      }
      return;
    }

    if (Policy::ShouldBoundSearch(working_, best_)) return;

    // History prune checks (compile-time gated; dead code stripped
    // when flag is false).
    if constexpr (Policy::kUseLengthHistory) {
      const auto &tracker = working_.GetScheduledSetTracker();
      int end_cycle = working_.GetCurrentCycle();
      if (length_history_.ShouldPrune(tracker, end_cycle)) return;
      length_history_.Insert(tracker, end_cycle);
    }
    if constexpr (Policy::kUsePressureHistory) {
      const auto &tracker = working_.GetScheduledSetTracker();
      int prefix_peak = working_.GetForwardRegisterTracker().GetPeak();
      if (pressure_history_.ShouldPrune(tracker, prefix_peak, ceiling_))
        return;
      pressure_history_.RecordPrefixPeak(tracker, prefix_peak);
    }

    // Existing per-child loop — ScheduledSetTracker updates happen
    // inside working_.ScheduleByIndex / Unschedule.
    int ready_size = working_.GetReadyList().size();
    for (int i = 0; i < ready_size; ++i) {
      working_.ScheduleByIndex(i);
      Recurse();
      working_.Unschedule();
      if (should_end_search_) return;
    }
  }

  void UpdatePostfixPeaksFromCompletedSchedule() {
    ArrayRef<int> p = working_.GetForwardRegisterTracker().GetPressureHistory();
    SmallVector<int> postfix_peak(p.size() + 1, 0);
    for (int k = p.size() - 1; k >= 0; --k) {
      postfix_peak[k] = std::max(postfix_peak[k + 1], p[k]);
    }
    // For each partition along the path, update history.
    // (Implementation walks schedule_order_ in parallel with the
    // postfix_peak array, computing the partition signature
    // incrementally.)
  }

  ScheduleConstructor working_, best_;
  LengthHistoryTracker length_history_;
  PressureHistoryTracker pressure_history_;
  int ceiling_;   // function-wide occupancy ceiling
};
```

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

Each phase commits independently. Shakedowns added per phase.

### Phase 1 — ScheduledSetTracker

- New `ScheduledSetTracker.{h,cpp}` files in
  `lib/Target/AMDGPU/HierarchicalScheduler/`.
- Add `ScheduledSetTracker scheduled_set_tracker_` member to
  `ScheduleConstructor`. Construct in `ScheduleConstructor`'s ctor
  with the graph.
- Hook `scheduled_set_tracker_.OnSchedule(node)` and
  `OnUnschedule(node)` into `ScheduleConstructor::Schedule` /
  `Unschedule`.

Shakedowns:
- Round-trip property: after a sequence of Schedule/Unschedule
  calls that returns to the empty schedule, prefix_signature == 0,
  scheduled_set is empty, frontier is empty.
- Order-invariance: scheduling {A, B} in different orders yields
  the same prefix_signature and scheduled_set (XOR commutativity
  + bitset is set-equal).
- Frontier counts: synthetic graph with known structure, verify
  scheduled_pred_count and frontier membership at each step.

### Phase 2 — Forward register tracker records pressure

- Add `pressure_history_` vector to `GCNForwardRegisterTracker`.
- On `Schedule`: push current pressure. On `Unschedule`: pop.
- Expose `GetPressureHistory()`.

Shakedowns:
- Round-trip: after Schedule/Unschedule round trip,
  `pressure_history_` is empty.
- Suffix-max correctness: schedule a known-shape graph, check
  suffix-max values match hand-computed.

### Phase 3 (optional) — Tracker enablement policy flags

- Add `kUseLengthTracker`, `kUseForwardRegisterTracker` to
  `SearchPolicyBase` (default false, override true on existing
  policies).
- Conditionally construct/maintain trackers in `ScheduleConstructor`
  based on these flags.

Useful as a refactor before adding more trackers — sets up the
opt-in pattern uniformly. Skippable if we're willing to keep
trackers always-on for now.

### Phase 4 — LengthHistoryTracker

- New `LengthHistoryTracker.{h,cpp}`.
- Add `LengthHistoryTracker length_history_` to `DfsSearch`.
- Add `if constexpr (Policy::kUseLengthHistory)` block in `Recurse`
  for prune check + insert.
- Set `DfsMinimizeLengthPolicy::kUseLengthHistory = true`.

Shakedowns:
- Synthetic DAG where two orderings reach same partition with
  identical end_cycle / frontier LBs; verify second visit is
  pruned and table size stays at 1.
- Synthetic DAG where two orderings reach same partition with
  different metrics; verify Pareto frontier is maintained.
- End-to-end on hip_stencil and dfs_test: DFS produces same best
  schedule as without history, but with reduced
  `schedule_call_count`.

### Phase 5 — PressureHistoryTracker

- New `PressureHistoryTracker.{h,cpp}`.
- Add `PressureHistoryTracker pressure_history_` to `DfsSearch`.
- Add `if constexpr (Policy::kUsePressureHistory)` blocks:
  - In `Recurse`: prune check + `RecordPrefixPeak`.
  - At `IsDone`: suffix-max over forward tracker's pressure
    history; update postfix peaks for each partition on the path.
- Set `DfsMaximizeOccupancyPolicy::kUsePressureHistory = true`.

Shakedowns:
- Synthetic DAG where two orderings reach same partition with
  different prefix_peak; verify worse one is pruned.
- Verify total-bound prune fires when prefix + postfix exceeds
  ceiling.
- End-to-end on dfs_test (which exercises occupancy DFS):
  reduced `schedule_call_count`, same final occupancy.

---

## Summary

History-based domination prunes B&B subtrees by recognizing that
multiple prefixes can reach the same scheduled set, and the search
only needs to follow the best one. The `ScheduledSetTracker` (signatures,
bitset, frontier) lives on `ScheduleConstructor` as a self-
contained object. The history tables live on `DfsSearch` because
they accumulate across the search.

Length and pressure history are siblings, not derived from a
common base, because their dominance logic differs fundamentally.
Length needs a Pareto frontier per partition over multi-dimensional
entries (`end_cycle` + frontier LBs). Pressure has one entry per
partition with two scalars (`best_prefix_peak`,
`best_postfix_peak`), enabled by the partition fully decoupling
prefix and postfix register pressure.

Postfix peak is derived from the forward register tracker's
recorded per-instruction pressure (suffix-max), avoiding the need
for a separate backward tracker.

Proxy nodes from subgraph formation are treated uniformly with
real instructions in the `ScheduledSetTracker`. They get their own signatures,
appear in the scheduled bitset, and participate in frontier
tracking. Their pressure-recording entries are duplicates of the
previous real instruction's pressure (proxies have no defs/uses),
which is correctly handled by suffix-max.

Policy contract follows the `MakeFormationPolicy` pattern: two
new `kUseLengthHistory` / `kUsePressureHistory` flags, defaulting
false, overridden true on the policies that want them. `if
constexpr` gates the tracker code so unused histories cost nothing.
