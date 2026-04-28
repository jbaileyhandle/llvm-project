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
// Memory cap: hard-internal `kMaxEntries`. Insertion past the cap
// reports a fatal error rather than silently degrading. LRU
// eviction was considered and deferred — see §8.3 of the design
// doc for the rationale.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_LENGTHHISTORYTRACKER_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_LENGTHHISTORYTRACKER_H

#include "ScheduleLengthTracker.h"
#include "ScheduledSetTracker.h"
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
  /// limit reports a fatal error. See class-level comment for the
  /// fatal-error-vs-eviction rationale.
  static constexpr int kMaxEntries = 20'000;

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
  };

  /// Construct over `scheduled_set_tracker` and `length_tracker`.
  /// Both pointers must be non-null and outlive this tracker.
  ///
  /// The bitset-size >= 2 invariant required by PartitionKey's
  /// DenseMapInfo sentinels is enforced by ScheduledSetTracker's
  /// own constructor; we don't recheck it here.
  LengthHistoryTracker(const ScheduledSetTracker *scheduled_set_tracker,
                       const ScheduleLengthTracker *length_tracker);

  /// True iff the bound trackers' current prefix is dominated by
  /// some existing entry in this partition's bucket. Pure read; no
  /// side effects. Useful primarily for tests.
  bool IsDominated() const;

  /// True iff the current prefix is dominated by some existing
  /// entry. If false, a new entry for the current prefix is
  /// inserted and any existing entries dominated by it are removed
  /// (Pareto trim). Reports a fatal error if inserting would exceed
  /// `kMaxEntries`.
  bool IsDominatedElseInsert();

  /// Total entries across all partitions. Useful for shakedowns
  /// and stat dumps.
  int GetTotalEntries() const { return total_entries_; }

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
  /// Returns true iff `a` dominates `b` on every Pareto dimension
  /// (end_cycle and each frontier LB). Both Entries must be from
  /// the same partition: their frontier_lbs vectors have equal
  /// length and parallel node_topo_idx ordering.
  static bool DoesDominate(const Entry &a, const Entry &b);

  const ScheduledSetTracker *scheduled_set_tracker_;
  const ScheduleLengthTracker *length_tracker_;
  DenseMap<PartitionKey, SmallVector<Entry, 2>> table_;
  int total_entries_ = 0;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_LENGTHHISTORYTRACKER_H