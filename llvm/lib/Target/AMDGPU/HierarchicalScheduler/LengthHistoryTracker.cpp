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
      include_pressure_dim_(recipe.HasDim(ScoreDimension::kContinuousOccScore)),
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

LengthHistoryTracker::Entry LengthHistoryTracker::BuildQueryEntry() const {
  // Skip the work when the corresponding gate is off — leave the
  // field at its default (0 / empty) since DoesDominate won't
  // consult it. When the gate is on but the tracker is null
  // (test paths only — production wires both consistently), fall
  // back to default too.
  int continuous_occupancy_score = 0;
  if (include_pressure_dim_ && pressure_tracker_ != nullptr) {
    continuous_occupancy_score =
        pressure_tracker_->GetContinuousOccupancyScore();
  }
  int ilp_score = 0;
  SmallVector<IlpTracker::OpenProducerInstCount, 16>
      open_producer_inst_counts;
  if (include_ilp_dim_ && ilp_tracker_ != nullptr) {
    ilp_score = ilp_tracker_->GetIlpScore();
    open_producer_inst_counts =
        ilp_tracker_->GetOpenProducerInstCountsSnapshot();
  }
  return Entry{length_tracker_->GetCurrentCycle(),
               GetFrontierLbsSnapshot(),
               continuous_occupancy_score, ilp_score,
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
  // Length axes flip direction based on length_max_mode_. In min
  // mode, lower end_cycle / lower frontier LB dominates (a beats b
  // iff a's value is no greater). In max mode, higher end_cycle /
  // higher frontier LB dominates (a beats b iff a's value is no
  // smaller). Soundness in max mode: propagating component-wise
  // no-smaller starting LBs through any postfix ordering yields
  // no-shorter completions — the symmetric argument to the min-mode
  // case. See the LengthHistoryTracker constructor comment.
  if (length_max_mode_) {
    if (a.end_cycle < b.end_cycle) {
      return false;
    }
  } else {
    if (a.end_cycle > b.end_cycle) {
      return false;
    }
  }
  // Parallel walk over frontier_lbs. Same-partition invariant
  // (caller restricts to one DenseMap bucket) guarantees same
  // length and same node_topo_idx ordering, so we can ignore
  // node_topo_idx and compare only the LBs.
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
  // ILP dimension (in priority order before pressure: matches
  // the IsBetterThan tiebreak hierarchy length → ILP → pressure).
  // Two required-no-worse Pareto checks:
  //   1. Per-open-producer: a.inst_count[R] <= b.inst_count[R]
  //      for every R (same-partition entries share the same
  //      reg-sorted order of open producers, so a parallel walk
  //      aligns them). Ensures a has same-or-more future ILP
  //      cover for every open producer.
  //   2. Locked-in: a.ilp_score >= b.ilp_score (reversed
  //      direction: higher is better).
  // Together (1) and (2) imply a's max completion ILP >=
  // b's max completion ILP — sound for refine-ILP dominance.
  // Gated on include_ilp_dim_ (false in policies that don't
  // refine ILP).
  if (include_ilp_dim_) {
    for (size_t i = 0; i < a.open_producer_inst_counts.size(); ++i) {
      if (a.open_producer_inst_counts[i].inst_count >
          b.open_producer_inst_counts[i].inst_count) {
        return false;
      }
    }
    if (a.ilp_score < b.ilp_score) {
      return false;
    }
  }
  // Pressure-score dimension. Reversed direction: higher score is
  // better, so a dominates iff a.score >= b.score. Pressure is
  // monotonically non-decreasing during search, so the recorded
  // score is an upper bound on any completion's score — making
  // this dominance sound when paired with the length dimensions
  // above. Gated on include_pressure_dim_ (false in the default
  // length-only policy).
  if (include_pressure_dim_) {
    if (a.continuous_occupancy_score < b.continuous_occupancy_score) {
      return false;
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
