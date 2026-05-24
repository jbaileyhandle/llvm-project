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
// Per partition, only ONE entry is needed: the lexicographic best
// (peak primary, occupancy area as the same-peak tiebreak) over all
// prefixes that have reached this partition. A new prefix whose
// (peak, area) is <= the prior visit's lexicographically is dominated
// and can be pruned. For pure-peak callers area is 0, so this reduces
// to scalar peak domination. Single-entry lexicographic domination is
// exact for the peak objective and for area among equal-peak prefixes;
// a higher-peak prior can prune a higher-area prefix (a tiebreak-only
// loss — we keep one entry, not a full (peak, area) Pareto frontier).
//
// Memory cap: soft-internal `kMaxEntries`. Insertion past the cap
// silently no-ops (no insert, no fatal error) and sets a flag
// readable via `MemoryCapWasHit()` for telemetry. Existing
// dominance checks are unaffected — pruning still fires for any
// prefix that an already-recorded entry dominates; we just stop
// *recording* new partitions once the cap is reached. The
// per-region wall-clock timeout in DfsSearch (see DfsSearch.h) is
// the real bound on runaway searches; the cap exists only as a
// memory backstop. The table grows monotonically here, so once 
// the cap is hit it stays hit for the rest of the tracker's lifetime.
//
// Metric-agnosticism. The tracker does not encode a specific
// metric; its `IsDominatedElseRecord(int current_prefix_score)`
// overload just compares ints. Convention: higher = better (we
// MAXIMIZE), regardless of which metric the caller is using —
// continuous occupancy score, integer occupancy, etc. all follow
// the same direction. Soundness requires the caller's metric
// here to be at least as fine as the metric the policy is
// optimizing — otherwise dominance can prune away paths that are
// strictly better under the policy's own metric.
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
#include "SearchStats.h"
#include "llvm/ADT/DenseMap.h"

namespace llvm {
namespace hierarchical_scheduler {

class PressureHistoryTracker {
 public:
  /// Total-entry cap across all partitions. Insertion past this
  /// limit silently no-ops (search continues, just without
  /// recording the new partition); the per-region wall-clock
  /// timeout in DfsSearch is the real bound on runaway searches.
  /// Sized generously so the cap is just a memory backstop. Per-
  /// entry footprint is dominated by the PartitionKey's BitVector
  /// (heap-allocated, sized by region node count) — call it ~150 B
  /// for a 600-node region, scaling roughly linearly with N. At
  /// 10M entries that's ~1.5 GB worst case. The pressure tracker
  /// stores at most one entry per partition (no Pareto frontier),
  /// so the same entry count covers more search states than the
  /// length tracker's matching cap.
  static constexpr int kMaxEntries = 10'000'000;

  /// One entry per partition. Public so tests can stage entries
  /// directly via InsertEntryForTest.
  struct Entry {
    /// Lexicographic (peak, area) best over all prefixes reaching
    /// this partition; higher = better. best_area is the same-peak
    /// tiebreak (0 for pure-peak callers). See class comment. Set on
    /// first visit; replaced by a lexicographically-greater visit.
    int best_prefix_score = 0;
    int64_t best_area = 0;
  };

  /// Bind:
  ///   - `scheduled_set_tracker` — source of truth for the
  ///     partition key. Non-null; must outlive the tracker.
  ///   - `working_register_tracker` — source of truth for
  ///     current_prefix_score that the no-arg overload of
  ///     IsDominatedElseRecord reads via the bound metric. May
  ///     be nullptr in test fixtures that only use the explicit-
  ///     score overload; in that configuration the no-arg
  ///     overload fatal-errors. Otherwise must outlive the
  ///     tracker.
  ///   - `metric` — ScheduleMetric used by the no-arg overload
  ///     to read the score from the bound working register
  ///     tracker via GCNRegisterTracker::GetMetricScore.
  ///     GetMetricScore normalizes minimize variants so higher
  ///     = better regardless of direction; length-side metrics
  ///     fatal-error there because GCNRegisterTracker has no
  ///     length data.
  PressureHistoryTracker(
      const ScheduledSetTracker *scheduled_set_tracker,
      const GCNRegisterTracker *working_register_tracker,
      ScheduleMetric metric);

  /// Combined check + record. Reads the partition key from the
  /// bound scheduled-set tracker. Returns true if pruning should
  /// fire.
  ///
  /// Cases (given the partition this prefix has reached, and
  /// `current_prefix_score` from the caller):
  ///   - No prior entry at this partition: insert with
  ///     `best_prefix_score = current_prefix_score`; return false.
  ///     If inserting would exceed `kMaxEntries`, the insert is
  ///     silently skipped, `memory_cap_hit_` is set, and the
  ///     return is still false.
  ///   - Prior entry's `best_prefix_score >= current_prefix_score`:
  ///     the prior visit is no worse on the only prefix-dependent
  ///     dimension. By the partition's prefix/postfix decoupling,
  ///     anything our subtree could reach is reachable at no
  ///     worse score from the prior visit. Increment
  ///     `prune_count_`; return true.
  ///   - Prior entry exists and current is strictly better:
  ///     update `best_prefix_score = max(prior, current)`; return
  ///     false. (No table growth, so no cap concern.)
  ///
  /// Metric-agnostic: this overload just compares ints (higher =
  /// better). The caller picks what those ints represent. Delegates
  /// to the (peak, area) overload below with current_area = 0, so
  /// domination is decided on peak alone.
  bool IsDominatedElseRecord(int current_prefix_score);

  /// Lexicographic (peak, area) variant: peak primary, area as the
  /// same-peak tiebreak. Prunes when a prior visit's (peak, area) is
  /// >= this one lexicographically; otherwise records this prefix as
  /// the partition's new lexicographic best. See class comment for
  /// the soundness tradeoff.
  bool IsDominatedElseRecord(int current_peak_score, int64_t current_area);

  /// Production wrapper. Reads peak via GetMetricScore(metric_) from
  /// the bound working register tracker, and area via
  /// GetContinuousOccupancyArea() only when metric_ is the area-
  /// tiebreak metric (0 otherwise, so other metrics prune peak-only),
  /// then delegates to the (peak, area) overload. Fatal-errors if the
  /// bound working register tracker is null.
  bool IsDominatedElseRecord();

  /// Drop all recorded entries and clear the *.current_run
  /// counters and flags. Lifetime values persist. Useful for
  /// outer loops that re-use the tracker across multiple search
  /// iterations (mirrors LengthHistoryTracker::Reset for API
  /// symmetry; the occupancy pass currently runs single-Run so
  /// doesn't exercise this, but provided so the trackers behave
  /// consistently).
  void Reset();

  /// Total entries across all partitions.
  int GetTotalEntries() const { return static_cast<int>(table_.size()); }

  /// Read-only access to the prune counter. `.current_run` is
  /// the number of times IsDominatedElseRecord returned true
  /// during the current Run() (cleared by Reset). `.lifetime`
  /// is the total since construction (never cleared by Reset).
  const DualRunAndLifetimeCounter &PruneCount() const {
    return prune_count_;
  }

  /// Read-only access to the memory-cap-hit flag. `.current_run`
  /// is true iff IsDominatedElseRecord hit the `kMaxEntries`
  /// soft cap during the current Run() (cleared by Reset).
  /// `.lifetime` is true iff it ever hit during this tracker's
  /// lifetime (never cleared; sticky once set). Useful for
  /// telemetry flagging searches whose pruning effectiveness was
  /// clipped by the cap.
  const DualRunAndLifetimeFlag &MemoryCapHit() const {
    return memory_cap_hit_;
  }

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
  ScheduleMetric metric_;
  DenseMap<PartitionKey, Entry> table_;
  /// Incremented at every prune event. .current_run is cleared
  /// by Reset; .lifetime persists.
  DualRunAndLifetimeCounter prune_count_;
  /// Set true the first time IsDominatedElseRecord wants to
  /// insert but `table_.size()` is at `kMaxEntries`. Sticky —
  /// the table only grows here (no Pareto trim), so once at cap
  /// we stay at cap; .lifetime never clears, .current_run
  /// clears on Reset. See `MemoryCapHit()`.
  DualRunAndLifetimeFlag memory_cap_hit_;
};

} // namespace hierarchical_scheduler
} // namespace llvm

//========================================================================================

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_PRESSUREHISTORYTRACKER_H
