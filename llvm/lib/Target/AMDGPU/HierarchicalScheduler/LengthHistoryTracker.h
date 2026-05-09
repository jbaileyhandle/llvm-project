//===- LengthHistoryTracker.h - Length history-based domination -*- C++ -*-===//
//
// History table for length-minimization B&B pruning. Memoizes
// previously-visited prefixes by partition; new prefixes that reach
// a partition with no-better state get pruned.
//
// A "partition" is the bipartition of the graph induced by which
// nodes are scheduled vs. unscheduled — identified by the
// `ScheduledSetTracker`'s (signature, scheduled_set) pair. Two
// prefixes share a partition iff they scheduled the same set of
// nodes (regardless of order).
//
// Per partition, multiple prefix orderings can produce different
// (end_cycle, frontier-LB) tuples. Length dominance is multi-
// dimensional: prefix A dominates prefix B iff A.end_cycle <=
// B.end_cycle AND A.frontier_lbs[i].lower_bound <=
// B.frontier_lbs[i].lower_bound for every frontier node i. Two
// prefixes that are incomparable on these dimensions both stay in
// the bucket — a Pareto frontier per partition.
//
// Why incomparable entries arise. Within a single partition,
// different prefix orderings produce different (end_cycle,
// frontier_lbs) tuples. Two prefixes can be genuinely incomparable:
// A has a lower end_cycle but a higher LB on some frontier node;
// B is the reverse. Each is potentially better for *some* postfix
// orderings (A's lower end_cycle helps when the postfix is short;
// B's lower frontier LB on node X helps when X's downstream cone
// dominates the postfix). Collapsing to a single "best" entry per
// partition would require a total order, which would discard
// genuinely-useful prefixes and lose pruning opportunities. The
// Pareto frontier keeps every non-dominated entry and prunes only
// what is provably dominated.
//
// Soundness: if A dominates B at the same partition, then for any
// postfix ordering Q, [A's prefix] ++ Q has length <= [B's prefix]
// ++ Q. So pruning B's subtree loses no optimal completion. See
// §5.3 of AMDGPUHistoryDominationDesign.md.
//
// Storage: DenseMap keyed by `PartitionKey` (signature + bitset).
// One bucket per partition holds the Pareto frontier of incomparable
// entries. Hash collisions across genuinely different partitions
// are handled by DenseMap's open-addressing probing, transparent to
// this class.
//
// Memory cap: soft-internal `kMaxEntries`. Insertion past the cap
// silently no-ops (no insert, no fatal error) and sets a flag that
// callers can read via `MemoryCapWasHit()` for telemetry. The
// dominance check against already-recorded entries is unaffected —
// pruning still fires for any prefix that an existing entry
// dominates; we just stop *recording* new prefixes once the cap is
// reached. The per-region wall-clock timeout in DfsSearch (see
// DfsSearch.h) is the real bound on runaway searches; the cap
// exists only as a memory backstop. LRU eviction was considered
// and deferred — see §8.3 of the design doc for the rationale.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_LENGTHHISTORYTRACKER_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_LENGTHHISTORYTRACKER_H

#include "GCNRegisterTracker.h"
#include "ScheduleLengthTracker.h"
#include "ScheduledSetTracker.h"
#include "SearchStats.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"

namespace llvm {
namespace hierarchical_scheduler {

/// One frontier-LB entry: a frontier node identified by its topo
/// index, paired with the prefix-derived lower bound on its
/// earliest issue cycle.
///
/// Note: `node_topo_idx` is not strictly required for the
/// dominance computation. Same-partition entries have parallel-
/// indexed frontier_lbs vectors (we sort by topo index at snapshot
/// time), so element-wise comparison can use just the LB. We carry
/// the topo index anyway for debug legibility — dumping a bucket
/// reads as "node 42 lb=7, node 53 lb=9, ..." instead of
/// "[7, 9, ...]". Memory cost is ~4 bytes per frontier element.
/// If memory pressure becomes an issue and the class is well-
/// tested, this field can be dropped to halve the per-element
/// footprint.
struct FrontierLb {
  int node_topo_idx;
  int lower_bound;
};

class LengthHistoryTracker {
 public:
  /// Total-entry cap across all partitions. Insertion past this
  /// limit silently no-ops (search continues, just without
  /// recording the new prefix); the per-region wall-clock timeout
  /// in DfsSearch is the real bound on runaway searches. Sized
  /// generously so the cap is just a memory backstop. Per-entry
  /// footprint is the PartitionKey's BitVector (~150 B for a
  /// 600-node region, scaling with N) plus the per-Entry
  /// end_cycle and inline-16 frontier_lbs SmallVector (~150 B).
  /// At 10M entries that's ~3 GB worst case. See class-level
  /// comment for the soft-cap-vs-eviction rationale.
  static constexpr int kMaxEntries = 10'000'000;

  /// One Pareto-frontier element. The (signature, scheduled_set)
  /// is encoded by the bucket's PartitionKey, not duplicated here.
  /// Public so tests can construct entries directly via
  /// InsertEntryForTest without going through the production
  /// schedule-the-DAG path.
  struct Entry {
    int end_cycle;
    /// Frontier LBs sorted by node_topo_idx. Two prefixes in the
    /// same partition have the same frontier nodes (same topo
    /// indices) but possibly different LBs, so element-wise
    /// comparison is a parallel walk.
    SmallVector<FrontierLb, 16> frontier_lbs;
    /// Continuous register-occupancy score at the time this entry
    /// was inserted. Always populated. Whether it participates in
    /// dominance is gated by `include_pressure_dim_` on the
    /// owning tracker — set true for the length-min-refine-
    /// occupancy policy, false otherwise. When the gate is off,
    /// this field is dead weight (~4 bytes/entry).
    int continuous_occupancy_score;
  };

  /// Construct over `scheduled_set_tracker`, `length_tracker`, and
  /// `pressure_tracker`. All pointers must be non-null and outlive
  /// this tracker.
  ///
  /// `include_pressure_dim` controls whether the partial schedule's
  /// continuous occupancy score (at insertion time) participates in
  /// dominance. False (default) reproduces the length-only behavior;
  /// true adds a reversed-direction dimension (higher score is
  /// better) so prior dominates only if its score >= current's.
  /// Used by DfsMinimizeLengthRefineOccupancyPolicy. The score
  /// field is always populated on Entry regardless — the gate is
  /// only on whether the field is consulted.
  ///
  /// The bitset-size >= 2 invariant required by PartitionKey's
  /// DenseMapInfo sentinels is enforced by ScheduledSetTracker's
  /// own constructor; we don't recheck it here.
  LengthHistoryTracker(const ScheduledSetTracker *scheduled_set_tracker,
                       const ScheduleLengthTracker *length_tracker,
                       const GCNRegisterTracker *pressure_tracker,
                       bool include_pressure_dim);

  /// True iff the bound trackers' current prefix is dominated by
  /// some existing entry in this partition's bucket. Pure read; no
  /// side effects. Useful primarily for tests.
  bool IsDominated() const;

  /// True iff the current prefix is dominated by some existing
  /// entry. If false, a new entry for the current prefix is
  /// inserted and any existing entries dominated by it are removed
  /// (Pareto trim). If inserting would exceed `kMaxEntries`, the
  /// insert is silently skipped and `memory_cap_hit_` is set
  /// (queryable via `MemoryCapWasHit()`); the search continues
  /// with whatever entries are already recorded. Returns false in
  /// that path — without recording the current prefix we can't
  /// claim it's dominated by anything in this bucket beyond what
  /// the explicit dominance check above already determined.
  bool IsDominatedElseInsert();

  /// Drop all recorded entries and clear the *_this_run counters
  /// and flags. Lifetime counters (prune count, memory-cap-hit
  /// flag) are intentionally preserved so they continue to
  /// accumulate across Run() boundaries. Useful for outer loops
  /// that re-use the tracker across multiple search iterations,
  /// where each iteration wants a fresh dominance table but the
  /// region-level totals should reflect all iterations together.
  void Reset();

  /// Total entries across all partitions. Useful for shakedowns
  /// and stat dumps.
  int GetTotalEntries() const { return total_entries_; }

  /// Read-only access to the prune counter. `.current_run` is the
  /// number of times IsDominatedElseInsert returned true during
  /// the current Run() (cleared by Reset). `.lifetime` is the
  /// total since construction (never cleared by Reset).
  const DualRunAndLifetimeCounter &PruneCount() const {
    return prune_count_;
  }

  /// Read-only access to the memory-cap-hit flag. `.current_run`
  /// is true iff IsDominatedElseInsert hit the `kMaxEntries` soft
  /// cap during the current Run() (cleared by Reset).
  /// `.lifetime` is true iff it ever hit during this tracker's
  /// lifetime (never cleared; sticky once set). Useful for
  /// telemetry flagging searches whose pruning effectiveness
  /// was clipped by the cap.
  const DualRunAndLifetimeFlag &MemoryCapHit() const {
    return memory_cap_hit_;
  }

  /// Snapshot the bound scheduled-set tracker's current frontier as
  /// a node_topo_idx-sorted vector of FrontierLb. Public so tests
  /// can verify the snapshot matches what's stored.
  SmallVector<FrontierLb, 16> GetFrontierLbsSnapshot() const;

  /// Test-only: directly push `entry` into `key`'s bucket without
  /// any dominance check or Pareto trim. Lets tests stage arbitrary
  /// starting states without reverse-engineering DAGs to produce
  /// the desired (signature, scheduled_set, end_cycle, frontier_lbs)
  /// tuples through the production schedule path. Production code
  /// should never call this — it can violate the Pareto-frontier
  /// invariant.
  void InsertEntryForTest(const PartitionKey &key, Entry entry);

  /// Test-only: returns the bucket for `key`, or empty if no bucket
  /// exists. The returned ArrayRef is invalidated by any subsequent
  /// mutation (Insert, InsertEntryForTest).
  ArrayRef<Entry> GetBucketForTest(const PartitionKey &key) const;

 private:
  /// Returns true iff `a` dominates `b` on every Pareto dimension.
  /// Length dimensions (end_cycle and each frontier LB) use
  /// smaller-is-better. When `include_pressure_dim` is true, also
  /// requires a's continuous_occupancy_score >= b's
  /// (higher-is-better — a reversed-direction dimension). Both
  /// Entries must be from the same partition: their frontier_lbs
  /// vectors have equal length and parallel node_topo_idx ordering.
  bool DoesDominate(const Entry &a, const Entry &b) const;

  const ScheduledSetTracker *scheduled_set_tracker_;
  const ScheduleLengthTracker *length_tracker_;
  const GCNRegisterTracker *pressure_tracker_;
  /// Gate for the pressure-score dimension on dominance. See
  /// constructor comment.
  bool include_pressure_dim_;
  DenseMap<PartitionKey, SmallVector<Entry, 2>> table_;
  int total_entries_ = 0;
  /// Incremented at every prune event. .current_run is cleared
  /// by Reset; .lifetime persists.
  DualRunAndLifetimeCounter prune_count_;
  /// Set true the first time IsDominatedElseInsert wants to
  /// insert but `total_entries_` is at `kMaxEntries`. Sticky —
  /// the .current_run flag is cleared by Reset; .lifetime stays
  /// set once tripped, even if subsequent Pareto trims drop
  /// `total_entries_` below the cap again. See `MemoryCapHit()`.
  DualRunAndLifetimeFlag memory_cap_hit_;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_LENGTHHISTORYTRACKER_H