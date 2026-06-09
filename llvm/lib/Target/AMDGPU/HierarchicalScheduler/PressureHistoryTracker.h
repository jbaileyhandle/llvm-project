//===- PressureHistoryTracker.h - Pressure history-based domination *- C++ -*-===//
//
// History table for pressure-side B&B pruning in occupancy DFS.
// Memoizes previously-visited prefixes by partition; when a new
// prefix arrives at a partition where a prior prefix Pareto-
// dominates it on the bound Score, the new prefix's subtree can be
// pruned.
//
// A "partition" is the bipartition of the graph induced by which
// nodes are scheduled vs. unscheduled -- identified by
// `ScheduledSetTracker`'s (signature, scheduled_set) pair. Two
// prefixes share a partition iff they scheduled the same set of
// nodes (regardless of order).
//
// Per partition: a Pareto frontier of incomparable Score entries
// (parallel to LengthHistoryTracker's design). Prefix A dominates
// prefix B iff A.score.Dominates(B.score) -- every slot of A's
// canonical Score is >= the corresponding slot of B's. Incomparable
// entries (each better on some slot) both stay in the bucket. A
// lex-collapse memo (one entry per partition, lex-best) is unsound
// for multi-slot Scores: when the suffix equalizes a higher-priority
// slot at completion, a lower-slot tiebreak that was the wrong-side
// of lex at the prefix can become decisive at completion, so a lex-
// collapse prune can throw away the prefix that produces the optimal
// completion. The Pareto frontier preserves every non-dominated
// entry and prunes only what is provably dominated on every slot.
//
// FUTURE: Pareto vs lex prune. Pareto is necessary here because the
// production occupancy recipe (peak + area) contains a ND-canonical
// slot (area Max grows over completion). For an all-NI-canonical
// recipe, lex-on-prefix dominance is sound AND prunes strictly more
// than Pareto: per-slot A.best_final = min(A.prefix, S_partition) >=
// min(B.prefix, S_partition) = B.best_final, so the lex decision on
// prefix carries to the lex decision on best completions. Today's
// single-slot peak recipes are degenerate NI cases where lex and
// Pareto coincide. To realize the optimization in multi-slot form,
// the ND area dim could be replaced with its NI complement (see
// Score.h's note on the (max_per_step * num_scheduled - accumulated)
// deficit transformation) so the entire recipe becomes NI, then
// IsDominatedElseRecord could switch to a lex check on the recorded
// entries. Not implementing now; the all-NI recipe doesn't exist yet.
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

#include "ScheduleGraph.h"
#include "Score.h"
#include "ScheduledSetTracker.h"
#include "SearchStats.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"

namespace llvm {
namespace hierarchical_scheduler {

class ScheduleConstructor;

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

  /// One entry on a partition's Pareto frontier. Public so tests
  /// can stage entries directly via InsertEntryForTest.
  struct Entry {
    /// One Score on this partition's Pareto frontier. Slot
    /// semantics come from whichever ScoreRecipe produced the
    /// Score (see ScheduleConstructor::GetScore). The bucket is
    /// the set of Scores reaching this partition that are not
    /// Pareto-dominated by any other entry in the bucket.
    /// Default-constructed Score has all slots zero, so a default
    /// Entry is a valid "empty" sentinel.
    Score best_score = Score::Make({});
  };

  /// One bucket per partition: the Pareto frontier of incomparable
  /// Score entries. Inline capacity 1 covers the common "single-
  /// entry frontier" case for peak-only metrics; multi-slot metrics
  /// with genuinely trade-off-y dims may push past it and heap-
  /// allocate (revisit if profiling identifies that as a hot spot).
  /// Type alias so the inline capacity is set in one place.
  using Bucket = SmallVector<Entry, 1>;

  /// True iff this tracker's prune is sound and well-defined for
  /// `recipe`. DfsSearch reads this to decide whether to construct a
  /// PHT for a given policy; the constructor re-checks it and
  /// fatal-errors on a mismatch (so any other caller that bypasses
  /// the DfsSearch gate still trips loudly).
  ///
  /// Sound shapes today: primary dim is one of the peak-style
  /// pressure metrics — kRegisterOcc or kContinuousOccScore. For
  /// these, prefix Pareto dominance on the full Score carries
  /// through to completion (peak dominance constrains the suffix's
  /// running peak, which in turn constrains any sum-style tiebreak
  /// like kContinuousOccArea). Other primaries — kScheduleLength
  /// (use LHT instead), kContinuousOccArea (sum-style alone is not
  /// dominance-preserving without a peak slot), kIlpScore (not
  /// analyzed) — are rejected.
  static constexpr bool IsApplicableToRecipe(const ScoreRecipe &recipe) {
    if (!recipe.slots[0]) {
      return false;
    }
    switch (recipe.slots[0]->dim) {
      case ScoreDimension::kRegisterOcc:
      case ScoreDimension::kContinuousOccScore:
        return true;
      case ScoreDimension::kContinuousOccArea:
      case ScoreDimension::kScheduleLength:
      case ScoreDimension::kIlpScore:
        return false;
    }
    return false;
  }

  /// PHT exposes two `IsDominatedElseRecord` shapes:
  ///   1. `IsDominatedElseRecord(const Score &current_score)` --
  ///      caller supplies the Score explicitly. Test fixtures stage
  ///      Scores directly via this path.
  ///   2. `IsDominatedElseRecord()` -- production path. Computes
  ///      `working_schedule_constructor->GetScore(recipe)` internally
  ///      using the bound SC and the recipe stored at construction.
  ///
  /// Bind:
  ///   - `scheduled_set_tracker` — source of truth for the partition
  ///     key. Non-null; must outlive the tracker.
  ///   - `working_schedule_constructor` — supplies the Score for the
  ///     production no-arg `IsDominatedElseRecord()` via
  ///     GetScore(recipe). May be nullptr in test fixtures that only
  ///     use the explicit-Score overload; in that configuration the
  ///     no-arg path fatal-errors. Otherwise must outlive the tracker.
  ///   - `recipe` — the ScoreRecipe used by the no-arg
  ///     `IsDominatedElseRecord()` to read the Score from the bound
  ///     working schedule constructor. Not validated here -- DfsSearch
  ///     constructs PHT for every policy (including length-primary
  ///     ones that never query PHT) so a recipe shape check at
  ///     construction time would over-reject.
  PressureHistoryTracker(
      const ScheduledSetTracker *scheduled_set_tracker,
      const ScheduleConstructor *working_schedule_constructor,
      const ScoreRecipe &recipe);

  /// Combined check + record. Reads the partition key from the
  /// bound scheduled-set tracker. Returns true if pruning should
  /// fire.
  ///
  /// Walks this partition's Pareto bucket:
  ///   - If any existing entry Pareto-dominates current_score
  ///     (entry.score.Dominates(current_score) — every slot >=),
  ///     increment prune_count_ and return true.
  ///   - Otherwise, insert current_score into the bucket and remove
  ///     any existing entries that current_score Pareto-dominates
  ///     (Pareto trim). Return false. If inserting would push
  ///     total_entries_ over kMaxEntries, the insert is silently
  ///     skipped and memory_cap_hit_ is set; the return is still
  ///     false (a not-recorded prefix isn't dominated by anything
  ///     in this partition beyond what the dominance walk just
  ///     determined).
  bool IsDominatedElseRecord(const Score &current_score);

  /// Production wrapper. Reads the current Score from the bound
  /// working schedule constructor via GetScore(metric_), then
  /// delegates to the Score overload. Fatal-errors if the bound
  /// working schedule constructor is null.
  bool IsDominatedElseRecord();

  /// Drop all recorded entries and clear the *.current_run
  /// counters and flags. Lifetime values persist. Useful for
  /// outer loops that re-use the tracker across multiple search
  /// iterations (mirrors LengthHistoryTracker::Reset for API
  /// symmetry; the occupancy pass currently runs single-Run so
  /// doesn't exercise this, but provided so the trackers behave
  /// consistently).
  void Reset();

  /// Total entries across all partitions (sum of bucket sizes).
  int GetTotalEntries() const { return total_entries_; }

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

  /// Test-only: directly append `entry` to `key`'s bucket without
  /// any dominance check or Pareto trim. Lets tests stage arbitrary
  /// starting states without reverse-engineering DAGs through the
  /// production schedule path. Production code should never call
  /// this -- it can violate the Pareto-frontier invariant.
  void InsertEntryForTest(const PartitionKey &key, Entry entry);

  /// Test-only: returns the bucket for `key`, or empty if no
  /// bucket exists. The returned ArrayRef is invalidated by any
  /// subsequent table mutation.
  ArrayRef<Entry> GetBucketForTest(const PartitionKey &key) const;

 private:
  const ScheduledSetTracker *scheduled_set_tracker_;
  const ScheduleConstructor *working_schedule_constructor_;
  ScoreRecipe recipe_;
  /// Per-partition Pareto frontier of incomparable Score entries.
  /// Bucket type alias keeps the inline capacity in one place
  /// (see the Bucket typedef above).
  DenseMap<PartitionKey, Bucket> table_;
  /// Sum of bucket sizes across all partitions. Maintained
  /// incrementally on every Insert / Pareto trim so GetTotalEntries
  /// stays O(1) instead of summing on demand.
  int total_entries_ = 0;
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
