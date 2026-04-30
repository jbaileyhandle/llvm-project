//===- PressureHistoryTracker.h - Pressure history-based domination *- C++ -*-===//
//
// History table for pressure-side B&B pruning in occupancy DFS.
// Memoizes previously-visited prefixes by partition; when a new
// prefix arrives at a partition where a prior prefix was no
// worse, the new prefix's subtree can be pruned.
//
// A "partition" is the bipartition of the graph induced by which
// nodes are scheduled vs. unscheduled — identified by
// `ScheduledSetTracker`'s (signature, scheduled_set) pair. Two
// prefixes share a partition iff they scheduled the same set of
// nodes (regardless of order).
//
// Per partition, only ONE entry is needed. Pressure is one-
// dimensional: a single best-prefix-score plus (eventually) a
// single best-postfix-score per partition captures everything
// the search can use, because postfix score is partition-
// determined regardless of prefix order — the boundary live set
// is exactly the data edges crossing the partition cut, so
// prefix and postfix peaks can be optimized independently.
//
// Metric-agnosticism. The tracker does not encode a specific
// metric; its `IsDominatedElseRecord(int current, int best)`
// overload just compares two ints. Convention: higher = better
// (we MAXIMIZE), regardless of which metric the caller is using
// — continuous occupancy score, integer occupancy, etc. all
// follow the same direction. Soundness requires the caller's
// metric here to be at least as fine as the metric the policy
// is optimizing — otherwise dominance can prune away paths that
// are strictly better under the policy's own metric.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_PRESSUREHISTORYTRACKER_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_PRESSUREHISTORYTRACKER_H

//========================================================================================
// jbaile
//========================================================================================

#include "GCNRegisterTracker.h"
#include "ScheduleGraph.h"
#include "ScheduleMetric.h"
#include "ScheduledSetTracker.h"
#include "llvm/ADT/DenseMap.h"
#include <climits>
#include <functional>

namespace llvm {
namespace hierarchical_scheduler {

class PressureHistoryTracker {
 public:
  /// One entry per partition. Public so tests can stage entries
  /// directly via InsertEntryForTest.
  struct Entry {
    /// Best (highest) prefix score for any prefix that has reached
    /// this partition. Higher = better. Set on first visit; updated
    /// to `max(prior, current)` when a non-dominating later visit
    /// arrives.
    int best_prefix_score = 0;

    /// Postfix-side placeholder. INT_MIN means "no completion has
    /// been recorded below this partition." The sentinel is meant
    /// to be load-bearing once postfix recording is wired in: the
    /// total-bound prune `min(current, prior_postfix) <= best_so_far`
    /// auto-handles the fully-pruned-subtree case via
    /// `min(current, INT_MIN) == INT_MIN`. Stays at INT_MIN until
    /// postfix recording is added.
    int best_postfix_score = INT_MIN;

    /// Hint for fast-forward replay — set during postfix recording
    /// to the node scheduled NEXT from this partition along the
    /// recorded completion's path. Stays nullptr until replay
    /// support is added.
    const ScheduleNode *next_node_hint = nullptr;
  };

  /// Bind:
  ///   - `scheduled_set_tracker` — source of truth for the
  ///     partition key. Non-null; must outlive the tracker.
  ///   - `working_register_tracker` / `best_register_tracker` —
  ///     sources of truth for current_prefix_score and
  ///     best_so_far_score that the no-arg overload of
  ///     IsDominatedElseRecord reads via the bound metric. May
  ///     be nullptr in test fixtures that only use the explicit-
  ///     scores overload; in that configuration the no-arg
  ///     overload fatal-errors. Otherwise must outlive the
  ///     tracker.
  ///   - `metric` — ScheduleMetric used by the no-arg overload
  ///     to read scores from the bound register trackers via
  ///     GCNRegisterTracker::GetMetricScore. GetMetricScore
  ///     normalizes minimize variants so higher = better
  ///     regardless of direction; length-side metrics
  ///     fatal-error there because GCNRegisterTracker has no
  ///     length data.
  ///   - `enqueue_for_replay` — invoked when fast-forward replay
  ///     fires (currently never; held so the constructor
  ///     signature is stable across later phases). Must be
  ///     non-empty.
  PressureHistoryTracker(
      const ScheduledSetTracker *scheduled_set_tracker,
      const GCNRegisterTracker *working_register_tracker,
      const GCNRegisterTracker *best_register_tracker,
      ScheduleMetric metric,
      std::function<bool(const ScheduleNode *)> enqueue_for_replay);

  /// Combined check + record. Reads the partition key from the
  /// bound scheduled-set tracker. Returns true if pruning should
  /// fire.
  ///
  /// Cases (given the partition this prefix has reached, and
  /// `current_prefix_score` from the caller):
  ///   - No prior entry at this partition: insert with
  ///     `best_prefix_score = current_prefix_score`; return false.
  ///   - Prior entry's `best_prefix_score >= current_prefix_score`:
  ///     the prior visit is no worse on the only prefix-dependent
  ///     dimension. By the partition's prefix/postfix decoupling,
  ///     anything our subtree could reach is reachable at no
  ///     worse score from the prior visit. Increment
  ///     `prune_count_`; return true.
  ///   - Prior entry exists and current is strictly better:
  ///     update `best_prefix_score = max(prior, current)`; return
  ///     false.
  ///
  /// Postfix-side and replay-side logic is not yet active —
  /// `best_postfix_score` stays at INT_MIN on insert,
  /// `next_node_hint` stays nullptr, and `enqueue_for_replay_`
  /// is never invoked.
  ///
  /// Metric-agnostic: this overload just compares the two ints
  /// (higher = better). The caller picks what those ints
  /// represent. `best_so_far_score` is currently unused; it's on
  /// the signature for forward-compatibility with the upcoming
  /// total-bound prune (`min(current, prior_postfix) <=
  /// best_so_far → prune`).
  bool IsDominatedElseRecord(int current_prefix_score,
                              int best_so_far_score);

  /// Production wrapper. Reads scores from the bound register
  /// trackers via GCNRegisterTracker::GetMetricScore(metric_),
  /// then delegates to the explicit-scores overload above.
  /// Fatal-errors if either bound register tracker is null.
  bool IsDominatedElseRecord();

  /// Total entries across all partitions.
  int GetTotalEntries() const { return static_cast<int>(table_.size()); }

  /// Total times IsDominatedElseRecord returned true. Cumulative
  /// across the tracker's lifetime.
  int GetTotalPruneCount() const { return prune_count_; }

  /// Test-only: directly insert (or overwrite) an entry for
  /// `key`. Bypasses the production schedule path so tests can
  /// stage arbitrary starting states without driving real DAG
  /// scheduling to produce specific score values.
  void InsertEntryForTest(const PartitionKey &key, Entry entry);

  /// Test-only: returns a pointer to the entry for `key`, or
  /// nullptr if absent. Pointer invalidated by any subsequent
  /// table mutation.
  const Entry *GetEntryForTest(const PartitionKey &key) const;

 private:
  const ScheduledSetTracker *scheduled_set_tracker_;
  const GCNRegisterTracker *working_register_tracker_;
  const GCNRegisterTracker *best_register_tracker_;
  ScheduleMetric metric_;
  std::function<bool(const ScheduleNode *)> enqueue_for_replay_;
  DenseMap<PartitionKey, Entry> table_;
  int prune_count_ = 0;
};

} // namespace hierarchical_scheduler
} // namespace llvm

//========================================================================================

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_PRESSUREHISTORYTRACKER_H
