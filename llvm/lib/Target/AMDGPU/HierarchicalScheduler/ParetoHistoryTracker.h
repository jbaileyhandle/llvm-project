//===- ParetoHistoryTracker.h - Pareto history-based domination -*- C++ -*-===//
//
// History table for DFS B&B pruning. Memoizes previously-visited
// prefixes by partition; new prefixes that reach a partition with
// no-better state get pruned.
//
// A "partition" is the bipartition of the graph induced by which
// nodes are scheduled vs. unscheduled — identified by the
// `ScheduledSetTracker`'s (signature, scheduled_set) pair. Two
// prefixes share a partition iff they scheduled the same set of
// nodes (regardless of order).
//
// The dominance dimensions are recipe-driven: the scalar dims are
// exactly the bound ScoreRecipe's slots (length for the length
// passes; intermix credit + length for the pipe-mix pass), and the
// recipe's dims additionally gate the vector-shaped sibling dims
// (frontier LBs and per-pipe staleness; open-producer ILP counts) —
// see Entry. E.g. for length minimization: prefix A dominates
// prefix B iff A.end_cycle <= B.end_cycle AND
// A.frontier_lbs[i].lower_bound <= B.frontier_lbs[i].lower_bound
// for every frontier node i. Two prefixes that are incomparable on
// the active dimensions both stay in the bucket — a Pareto frontier
// per partition.
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

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_PARETOHISTORYTRACKER_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_PARETOHISTORYTRACKER_H

#include "GCNRegisterTracker.h"
#include "HwPipeClass.h"
#include "IlpTracker.h"
#include "PipeStalenessTracker.h"
#include "ScheduleLengthTracker.h"
#include "ScheduledSetTracker.h"
#include "SearchStats.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include <array>

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

class ParetoHistoryTracker {
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
    /// Recipe-driven canonical Score, computed by walking the
    /// tracker's bound ScoreRecipe slot-by-slot and feeding each
    /// dim's value through the matching tracker (length -> length
    /// tracker, kContinuousOccScore/kContinuousOccArea/kVgprSpillArea
    /// -> pressure tracker, kIlpScore -> ILP tracker). Same shape
    /// as PressureHistoryTracker::Entry::best_score; dominance is
    /// `score.Dominates(other.score)`. Polarity in each recipe slot
    /// flips the canonical direction, so e.g. length-min and
    /// length-max share the same compare logic differing only in
    /// the recipe's slot-0 polarity.
    Score score = Score::Make({});
    /// Frontier LBs sorted by node_topo_idx. Two prefixes in the
    /// same partition have the same frontier nodes (same topo
    /// indices) but possibly different LBs, so element-wise
    /// comparison is a parallel walk. Direction follows
    /// `length_max_mode_`: length-min wants lower LBs, length-max
    /// wants higher LBs. Vector-shaped, so it stays a sibling
    /// field rather than collapsing into the scalar Score.
    /// Populated and compared only when the recipe has a
    /// kScheduleLength dim (`include_length_dims_`); empty
    /// otherwise.
    SmallVector<FrontierLb, 16> frontier_lbs;
    /// Per-pipe capped staleness: visible instructions issued since
    /// each pipe last issued, capped at that pipe's target spacing
    /// (spacing ceiling 16, so values fit uint8_t). Populated from
    /// PipeStalenessTracker::GetStalenessSnapshot() and compared
    /// only when the recipe has a kIntermixCredit dim
    /// (`include_staleness_dim_`); zero-filled otherwise. Dominance
    /// direction: staler is better — a prefix whose every pipe is
    /// at least as stale earns at least as much credit from any
    /// given suffix (capped staleness is the sufficient statistic
    /// for future credit; proof in PipeStalenessTracker.h), so A
    /// dominates B requires A.staleness[p] >= B.staleness[p] for
    /// every pipe p.
    std::array<uint8_t, kNumHwPipes> pipe_staleness{};
    /// Per-open-producer issue positions, sorted by reg ascending.
    /// Populated and compared only when `include_ilp_dim_` is on
    /// (recipe has a kIlpScore dim); empty otherwise. When the
    /// gate is on, A dominates B requires A.inst_count[R] <=
    /// B.inst_count[R] for every open producer R (so A has
    /// same-or-more future ILP cover for each). Same-partition
    /// entries share the same vreg set in the same reg-sorted
    /// order, so element-wise comparison is well-defined.
    /// Vector-shaped, doesn't fit into Score either.
    ///
    /// Inline capacity 0: refine-ILP is not the production
    /// default, so most entries would carry a dead inline buffer
    /// (128 B at capacity 16). Cost of 0 is one heap alloc per
    /// entry when the ILP dim is on.
    SmallVector<IlpTracker::OpenProducerInstCount, 0>
        open_producer_inst_counts;
  };

  /// One bucket per partition: the Pareto frontier of incomparable
  /// Entry values. Inline capacity 2 covers small frontiers without
  /// heap alloc. Type alias so the inline capacity lives in one
  /// place (matches PressureHistoryTracker::Bucket's intent).
  using Bucket = SmallVector<Entry, 2>;

  /// Construct over the bound trackers. Which searches run this
  /// tracker at all is a policy decision (SearchPolicies' kHistory
  /// constants, read by DfsSearch) — this constructor only checks
  /// wiring: `scheduled_set_tracker` (partition key source) must
  /// be non-null, and `length_tracker` must be non-null when the
  /// recipe has a kScheduleLength dim. The score-source trackers
  /// (`pressure_tracker`, `ilp_tracker`,
  /// `pipe_staleness_tracker`) may be null (some tests / non-
  /// production paths don't supply them); when null, the
  /// corresponding fields populate from a fallback (0 / empty).
  /// Production callers (DfsSearch) wire every tracker the recipe
  /// reads. All bound trackers must outlive this tracker.
  ///
  /// The scalar Pareto axes -- length, intermix credit, continuous
  /// occupancy score, VGPR spill area, ILP score -- are all
  /// carried by the Entry's `score` field, computed from `recipe`
  /// by walking the recipe's slots and feeding each dim's value
  /// through the matching tracker. The set of slots in the recipe
  /// is the set of dims that participate in scalar dominance;
  /// adding a new scalar dim is "extend the recipe + extend the
  /// per-dim dispatch," no new gate flag.
  ///
  /// Three dim-shaped axes don't fit into the scalar Score and
  /// stay as sibling Entry fields, each gated by a recipe dim:
  ///
  ///   * `frontier_lbs` (length-axis vector): per-frontier-node
  ///     lower bounds. Gated by `include_length_dims_` from
  ///     `recipe.HasDim(kScheduleLength)`. Direction follows
  ///     length_max_mode_ derived from `recipe.IsLengthMaxMode()`:
  ///     length-min wants lower LBs, length-max wants higher.
  ///
  ///   * `open_producer_inst_counts` (ILP-axis vector): per-open-
  ///     producer issue positions. Gated by `include_ilp_dim_`
  ///     derived from `recipe.HasDim(kIlpScore)`. When gated on,
  ///     a Pareto carry check `a.inst_count[R] <= b.inst_count[R]`
  ///     for every R must hold for `a` to dominate `b`.
  ///
  ///   * `pipe_staleness` (credit-axis vector): per-pipe capped
  ///     staleness. Gated by `include_staleness_dim_` derived from
  ///     `recipe.HasDim(kIntermixCredit)`. When gated on, `a`
  ///     dominates `b` requires `a.pipe_staleness[p] >=
  ///     b.pipe_staleness[p]` for every pipe p (staler earns at
  ///     least as much from any suffix — see Entry).
  ///
  /// Combining `length_max_mode = true` with `include_ilp_dim =
  /// true` is unsupported by design -- length-max policies do not
  /// participate in ILP refinement here, for simplicity. The
  /// constructor asserts against this combination so a stray
  /// future configuration trips loudly instead of silently
  /// producing meaningless dominance results.
  ///
  /// The bitset-size >= 2 invariant required by PartitionKey's
  /// DenseMapInfo sentinels is enforced by ScheduledSetTracker's
  /// own constructor; we don't recheck it here.
  ParetoHistoryTracker(const ScheduledSetTracker *scheduled_set_tracker,
                       const ScheduleLengthTracker *length_tracker,
                       const GCNRegisterTracker *pressure_tracker,
                       const IlpTracker *ilp_tracker,
                       const PipeStalenessTracker *pipe_staleness_tracker,
                       const ScoreRecipe &recipe);

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

  /// Test-only: ask "would `query` be dominated by some entry in
  /// `key`'s bucket?" using the caller-supplied query Entry rather
  /// than building one from the bound trackers. Lets tests stage
  /// query Entries with arbitrary score / open-producer fields,
  /// which is necessary to exercise the per-open-producer walk in
  /// DoesDominate when no real IlpTracker is wired in the fixture.
  /// Pure read; no insert; no prune-counter increment. Returns
  /// false if `key`'s bucket doesn't exist.
  bool IsDominatedByEntryForTest(const PartitionKey &key,
                                 const Entry &query) const;

 private:
  /// Returns true iff `a` dominates `b` on every Pareto dimension.
  /// Four layers:
  ///   1. Scalar Score (via Score::Dominates): all recipe slots
  ///      compared with polarity-applied "higher is better"
  ///      semantics. Slot 0's polarity handles length min/max
  ///      direction automatically; other slots take their
  ///      direction from the recipe too.
  ///   2. frontier_lbs (length-axis vector, gated by
  ///      include_length_dims_): parallel walk; direction follows
  ///      length_max_mode_ (min mode wants a's LBs no greater,
  ///      max mode wants no smaller).
  ///   3. open_producer_inst_counts (ILP-axis vector, gated by
  ///      include_ilp_dim_): parallel walk requiring
  ///      a.inst_count[R] <= b.inst_count[R] for every R.
  ///   4. pipe_staleness (credit-axis vector, gated by
  ///      include_staleness_dim_): parallel walk requiring
  ///      a.pipe_staleness[p] >= b.pipe_staleness[p] for every
  ///      pipe p.
  /// Both Entries must be from the same partition: their
  /// frontier_lbs and open_producer_inst_counts vectors have
  /// equal length and parallel orderings.
  bool DoesDominate(const Entry &a, const Entry &b) const;

  /// Snapshot the bound trackers' current state into an Entry.
  /// Used by IsDominated and IsDominatedElseInsert to construct
  /// the query Entry. Populates the scalar Score from the bound
  /// recipe + trackers, plus the two vector dims.
  Entry BuildQueryEntry() const;

  /// Per-dim dispatch from the bound trackers to the raw value
  /// for a given ScoreDimension. Mirrors
  /// ScheduleConstructor::GetScoreDimensionValue's switch but
  /// reads from the trackers this tracker was bound to. Returns 0
  /// when the matching tracker is null (test-only path; production
  /// callers wire every tracker the recipe reads).
  int64_t GetDimRawValue(ScoreDimension dim) const;

  const ScheduledSetTracker *scheduled_set_tracker_;
  const ScheduleLengthTracker *length_tracker_;
  const GCNRegisterTracker *pressure_tracker_;
  const IlpTracker *ilp_tracker_;
  const PipeStalenessTracker *pipe_staleness_tracker_;
  /// Recipe used to build the canonical scalar Score stored on
  /// each Entry (see Entry::score). Captured at construction so
  /// BuildQueryEntry can walk the slots without an extra pass-
  /// through.
  ScoreRecipe recipe_;
  /// Gate for the length-axis vector dim (frontier_lbs). Derived
  /// at construction from `recipe.HasDim(kScheduleLength)`.
  bool include_length_dims_;
  /// Gate for the ILP-axis vector dim (open_producer_inst_counts).
  /// Derived at construction from `recipe.HasDim(kIlpScore)`.
  bool include_ilp_dim_;
  /// Gate for the credit-axis vector dim (pipe_staleness). Derived
  /// at construction from `recipe.HasDim(kIntermixCredit)`.
  bool include_staleness_dim_;
  /// Flip the direction of `frontier_lbs` element comparison.
  /// Derived at construction from `recipe.IsLengthMaxMode()`. The
  /// length-scalar direction is already encoded in `Entry::score`'s
  /// slot-0 polarity, so it doesn't need this flag; only the
  /// vector frontier-LB check does.
  bool length_max_mode_;
  DenseMap<PartitionKey, Bucket> table_;
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

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_PARETOHISTORYTRACKER_H