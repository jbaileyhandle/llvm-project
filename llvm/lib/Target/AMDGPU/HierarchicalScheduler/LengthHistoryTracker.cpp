//===- LengthHistoryTracker.cpp - Length history-based domination ---------===//
//
// Implementation. See header for the contract.
//
//===----------------------------------------------------------------------===//

#include "LengthHistoryTracker.h"
#include "llvm/Support/ErrorHandling.h"
#include <algorithm>

namespace llvm {
namespace hierarchical_scheduler {

LengthHistoryTracker::LengthHistoryTracker(
    const ScheduledSetTracker *scheduled_set_tracker,
    const ScheduleLengthTracker *length_tracker,
    const GCNRegisterTracker *pressure_tracker,
    const IlpTracker *ilp_tracker,
    const ScoreRecipe &recipe)
    : scheduled_set_tracker_(scheduled_set_tracker),
      length_tracker_(length_tracker),
      pressure_tracker_(pressure_tracker),
      ilp_tracker_(ilp_tracker),
      recipe_(recipe),
      include_ilp_dim_(recipe.HasDim(ScoreDimension::kIlpScore)),
      length_max_mode_(recipe.IsLengthMaxMode()) {
  if (scheduled_set_tracker_ == nullptr) {
    report_fatal_error(
        "LengthHistoryTracker: scheduled_set_tracker must not be null");
  }
  if (length_tracker_ == nullptr) {
    report_fatal_error(
        "LengthHistoryTracker: length_tracker must not be null");
  }
  // Recipe shape: DfsSearch already gates construction on
  // IsApplicableToRecipe; the re-check here catches any other caller
  // that bypassed the gate. See the predicate's doc for what's
  // supported.
  if (!IsApplicableToRecipe(recipe)) {
    report_fatal_error(
        "LengthHistoryTracker: recipe shape not supported "
        "(see LengthHistoryTracker::IsApplicableToRecipe)");
  }
  // Length-max + ILP-dim is unsupported by design (see header).
  // Trip loudly on a stray configuration rather than silently
  // producing meaningless dominance results.
  if (length_max_mode_ && include_ilp_dim_) {
    report_fatal_error(
        "LengthHistoryTracker: length_max_mode=true with "
        "include_ilp_dim=true is unsupported — length-max policies "
        "do not participate in ILP refinement here");
  }
  // pressure_tracker_ and ilp_tracker_ may be null. When non-null
  // and the corresponding gate is true, IsDominated /
  // IsDominatedElseInsert populate the matching Entry fields from
  // the tracker and dominance consults them. When null, the
  // fields are populated with fallback values (typically 0 / empty)
  // — useful for tests that use InsertEntryForTest to stage scores
  // explicitly without driving the production query path.
  // Production callers that opt into a gate are expected to supply
  // the matching tracker; there's no runtime check for that
  // misconfiguration since the sole production caller (DfsSearch)
  // wires it consistently. Bitset-size >= 2 invariant is enforced
  // by ScheduledSetTracker's ctor — no recheck here.
}

// Per-dim dispatch from this tracker's bound trackers to the raw
// value for a recipe slot. Mirrors
// ScheduleConstructor::GetScoreDimensionValue's switch but pulls
// from the four trackers LHT was bound to (rather than from a
// ScheduleConstructor it doesn't have). Returns 0 when the matching
// tracker is null -- test paths use that to stage Score values
// directly via InsertEntryForTest without driving the production
// trackers; production callers (DfsSearch) wire all four trackers
// consistently so the null path never fires.
int64_t LengthHistoryTracker::GetDimRawValue(ScoreDimension dim) const {
  switch (dim) {
  case ScoreDimension::kScheduleLength:
    return length_tracker_->GetCurrentCycle();
  case ScoreDimension::kRegisterOcc:
    return pressure_tracker_ ? pressure_tracker_->GetRegisterOnlyOccupancy()
                             : 0;
  case ScoreDimension::kContinuousOccScore:
    return pressure_tracker_
               ? pressure_tracker_->GetContinuousOccupancyScore()
               : 0;
  case ScoreDimension::kContinuousOccArea:
    return pressure_tracker_
               ? pressure_tracker_->GetContinuousOccupancyArea()
               : 0;
  case ScoreDimension::kVgprSpillArea:
    return pressure_tracker_ ? pressure_tracker_->GetVGPRSpillArea() : 0;
  case ScoreDimension::kIlpScore:
    return ilp_tracker_ ? ilp_tracker_->GetIlpScore() : 0;
  }
  llvm_unreachable("LengthHistoryTracker::GetDimRawValue: unknown dim");
}

LengthHistoryTracker::Entry LengthHistoryTracker::BuildQueryEntry() const {
  // Walk recipe slots; for each populated slot, snapshot the raw
  // value from the matching tracker and pair it with the slot's
  // polarity. Score::Make applies polarity to produce the
  // canonical "higher is better" form, the same shape as
  // PressureHistoryTracker::Entry::best_score.
  std::array<std::optional<Score::SlotInput>, kMaxScoreSlots> inputs{};
  for (int i = 0; i < kMaxScoreSlots; ++i) {
    if (!recipe_.slots[i]) {
      continue;
    }
    inputs[i] = Score::SlotInput{GetDimRawValue(recipe_.slots[i]->dim),
                                 recipe_.slots[i]->pol};
  }

  // open_producer_inst_counts: vector ILP dim, populated only when
  // the gate is on (and a tracker is available). Otherwise leave
  // empty.
  SmallVector<IlpTracker::OpenProducerInstCount, 16>
      open_producer_inst_counts;
  if (include_ilp_dim_ && ilp_tracker_ != nullptr) {
    open_producer_inst_counts =
        ilp_tracker_->GetOpenProducerInstCountsSnapshot();
  }

  return Entry{Score::Make(inputs), GetFrontierLbsSnapshot(),
               std::move(open_producer_inst_counts)};
}

SmallVector<FrontierLb, 16>
LengthHistoryTracker::GetFrontierLbsSnapshot() const {
  SmallVector<FrontierLb, 16> result;
  const DenseMap<int, FrontierEntry> &frontier =
      scheduled_set_tracker_->GetFrontier();
  result.reserve(frontier.size());
  for (const auto &kv : frontier) {
    result.push_back({kv.first, kv.second.lower_bound});
  }
  // Sort by node_topo_idx so element-wise comparison across two
  // entries in the same partition aligns at corresponding frontier
  // nodes.
  std::sort(result.begin(), result.end(),
            [](const FrontierLb &a, const FrontierLb &b) {
              return a.node_topo_idx < b.node_topo_idx;
            });
  return result;
}

bool LengthHistoryTracker::DoesDominate(const Entry &a,
                                        const Entry &b) const {
  // Scalar Pareto dominance: full Score.Dominates() over every
  // recipe slot. Polarity in each slot encodes direction --
  // length-min (Min polarity on kScheduleLength) maps lower raw
  // cycles to higher canonical value, length-max (Max polarity)
  // the reverse, and tiebreak slots (kContinuousOccScore Max,
  // kVgprSpillArea Min, kIlpScore Max, ...) each carry their own
  // polarity. So this single check covers what used to be three
  // separate scalar comparisons (end_cycle, ilp_score,
  // continuous_occupancy_score).
  if (!a.score.Dominates(b.score)) {
    return false;
  }
  // Frontier-LB parallel walk (length-axis vector). Direction
  // follows length_max_mode_. Same-partition invariant (caller
  // restricts to one DenseMap bucket) guarantees same length and
  // same node_topo_idx ordering, so we can ignore node_topo_idx
  // and compare only the LBs.
  for (size_t i = 0; i < a.frontier_lbs.size(); ++i) {
    if (length_max_mode_) {
      if (a.frontier_lbs[i].lower_bound < b.frontier_lbs[i].lower_bound) {
        return false;
      }
    } else {
      if (a.frontier_lbs[i].lower_bound > b.frontier_lbs[i].lower_bound) {
        return false;
      }
    }
  }
  // ILP vector dim: per-open-producer issue positions. Gated on
  // include_ilp_dim_ (recipe.HasDim(kIlpScore)). When on, a
  // dominates b only if a.inst_count[R] <= b.inst_count[R] for
  // every R (same-partition entries share the same reg-sorted
  // order, so a parallel walk aligns them). This pairs with the
  // ilp_score slot already in a.score / b.score: together they
  // capture both same-or-more locked-in ILP and same-or-more
  // future ILP cover.
  //
  // NOTE -- conservative on saturated producers. Each open producer
  // R has a per-op desirable_spacing cap (see IlpTracker): the
  // contribution at close is min(actual_spacing, desirable_spacing[R]),
  // so once spacing meets the cap, further reductions in inst_count
  // don't earn more credit. Two entries that are both already past
  // saturation for R contribute identically, but this strict
  // inst_count compare still discriminates them and keeps both on
  // the frontier. That's a performance issue (extra incomparable
  // entries) rather than a soundness issue -- the check never
  // wrongly prunes, just under-prunes when both sides are saturated.
  // A saturation-aware variant (clamp inst_count at
  // current_issue_count - desirable_spacing[R] before comparing)
  // would collapse those into equals; not implemented now.
  if (include_ilp_dim_) {
    for (size_t i = 0; i < a.open_producer_inst_counts.size(); ++i) {
      if (a.open_producer_inst_counts[i].inst_count >
          b.open_producer_inst_counts[i].inst_count) {
        return false;
      }
    }
  }
  return true;
}

bool LengthHistoryTracker::IsDominated() const {
  // Lookup via view — no bitset copy.
  PartitionKeyView view = scheduled_set_tracker_->GetPartitionKeyView();
  auto it = table_.find_as(view);
  if (it == table_.end()) {
    return false;
  }
  Entry query = BuildQueryEntry();
  for (const Entry &existing : it->second) {
    if (DoesDominate(existing, query)) {
      return true;
    }
  }
  return false;
}

bool LengthHistoryTracker::IsDominatedElseInsert() {
  PartitionKeyView view = scheduled_set_tracker_->GetPartitionKeyView();
  Entry query = BuildQueryEntry();

  // Lookup via view — no bitset copy in the existing-bucket path.
  // We construct an owning PartitionKey only when we have to insert
  // into a previously-unseen partition (below).
  auto it = table_.find_as(view);

  if (it == table_.end()) {
    // First visit to this partition. Build the owning key, push the
    // query into a fresh bucket. No dominator can exist (empty
    // bucket → no entries to compare against).
    if (total_entries_ + 1 > kMaxEntries) {
      // Soft cap: stop recording, set the sticky flag, let the
      // search continue. Returning false is correct — the empty-
      // bucket arm has no existing entries to dominate the query,
      // so "not dominated" is the truthful answer regardless of
      // whether we recorded.
      memory_cap_hit_.Set();
      return false;
    }
    PartitionKey key = scheduled_set_tracker_->GetPartitionKey();
    table_[key].push_back(std::move(query));
    ++total_entries_;
    return false;
  }

  // Bucket exists. Mutate it directly via the iterator; no key copy
  // needed.
  Bucket &bucket = it->second;

  // Single-pass walk: look for an existing dominator while
  // collecting indices of entries the query dominates. If a
  // dominator is found, return immediately — the dominated indices
  // collected so far are irrelevant (we're not inserting).
  SmallVector<size_t, 4> dominated_indices;
  for (size_t i = 0; i < bucket.size(); ++i) {
    const Entry &existing = bucket[i];
    if (DoesDominate(existing, query)) {
      prune_count_.Increment();
      return true;
    }
    if (DoesDominate(query, existing)) {
      dominated_indices.push_back(i);
    }
  }

  // No dominator. Trim entries dominated by the query (swap-erase
  // from the back so earlier indices stay valid), then insert.
  for (size_t i = dominated_indices.size(); i > 0; --i) {
    size_t idx = dominated_indices[i - 1];
    bucket[idx] = std::move(bucket.back());
    bucket.pop_back();
  }
  total_entries_ -= static_cast<int>(dominated_indices.size());

  if (total_entries_ + 1 > kMaxEntries) {
    // Soft cap: stop recording, set the sticky flag, let the
    // search continue. We've already done the dominance check
    // against the existing bucket above and confirmed no
    // dominator exists, so "not dominated" remains the truthful
    // answer. We're just dropping the chance to dominate future
    // siblings — the bucket reflects fewer prefixes than it
    // could, so downstream dominance results may be weaker than
    // they would have been otherwise.
    memory_cap_hit_.Set();
    return false;
  }

  bucket.push_back(std::move(query));
  ++total_entries_;
  return false;
}

void LengthHistoryTracker::Reset() {
  table_.clear();
  total_entries_ = 0;
  prune_count_.ResetCurrentRun();
  memory_cap_hit_.ResetCurrentRun();
}

void LengthHistoryTracker::InsertEntryForTest(
    const PartitionKey &key, Entry entry) {
  if (total_entries_ + 1 > kMaxEntries) {
    report_fatal_error(
        "LengthHistoryTracker::InsertEntryForTest: would exceed "
        "kMaxEntries");
  }
  table_[key].push_back(std::move(entry));
  ++total_entries_;
}

ArrayRef<LengthHistoryTracker::Entry>
LengthHistoryTracker::GetBucketForTest(const PartitionKey &key) const {
  auto it = table_.find(key);
  if (it == table_.end()) {
    return {};
  }
  return it->second;
}

bool LengthHistoryTracker::IsDominatedByEntryForTest(
    const PartitionKey &key, const Entry &query) const {
  auto it = table_.find(key);
  if (it == table_.end()) {
    return false;
  }
  for (const Entry &existing : it->second) {
    if (DoesDominate(existing, query)) {
      return true;
    }
  }
  return false;
}

} // namespace hierarchical_scheduler
} // namespace llvm
