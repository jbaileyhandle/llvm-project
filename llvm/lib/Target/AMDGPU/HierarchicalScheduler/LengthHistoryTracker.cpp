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
    bool include_pressure_dim)
    : scheduled_set_tracker_(scheduled_set_tracker),
      length_tracker_(length_tracker),
      pressure_tracker_(pressure_tracker),
      include_pressure_dim_(include_pressure_dim) {
  if (scheduled_set_tracker_ == nullptr) {
    report_fatal_error(
        "LengthHistoryTracker: scheduled_set_tracker must not be null");
  }
  if (length_tracker_ == nullptr) {
    report_fatal_error(
        "LengthHistoryTracker: length_tracker must not be null");
  }
  // pressure_tracker_ may be null. When non-null and
  // include_pressure_dim_ is true, IsDominated/IsDominatedElseInsert
  // populate Entry's continuous_occupancy_score from the tracker
  // and dominance consults it. When null, the score field is
  // populated with 0 — useful for tests that use
  // InsertEntryForTest to set scores explicitly without driving
  // the production query path. Production callers that opt into
  // include_pressure_dim are expected to supply a real tracker;
  // there's no runtime check for that misconfiguration since the
  // sole production caller (DfsSearch) wires it consistently.
  // Bitset-size >= 2 invariant is enforced by ScheduledSetTracker's
  // ctor — no recheck here.
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
  if (a.end_cycle > b.end_cycle) {
    return false;
  }
  // Parallel walk over frontier_lbs. Same-partition invariant
  // (caller restricts to one DenseMap bucket) guarantees same
  // length and same node_topo_idx ordering, so we can ignore
  // node_topo_idx and compare only the LBs.
  for (size_t i = 0; i < a.frontier_lbs.size(); ++i) {
    if (a.frontier_lbs[i].lower_bound > b.frontier_lbs[i].lower_bound) {
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
  // pressure_tracker_ may be null when include_pressure_dim_ is
  // false (tests don't always have one). The score field is dead
  // weight in that case — store 0 so the field is initialized.
  int score = pressure_tracker_ != nullptr
                  ? pressure_tracker_->GetContinuousOccupancyScore()
                  : 0;
  Entry query{length_tracker_->GetCurrentCycle(),
              GetFrontierLbsSnapshot(), score};
  for (const Entry &existing : it->second) {
    if (DoesDominate(existing, query)) {
      return true;
    }
  }
  return false;
}

bool LengthHistoryTracker::IsDominatedElseInsert() {
  PartitionKeyView view = scheduled_set_tracker_->GetPartitionKeyView();
  // See IsDominated above for the nullable-pressure-tracker rationale.
  int score = pressure_tracker_ != nullptr
                  ? pressure_tracker_->GetContinuousOccupancyScore()
                  : 0;
  Entry query{length_tracker_->GetCurrentCycle(),
              GetFrontierLbsSnapshot(), score};

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
  SmallVector<Entry, 2> &bucket = it->second;

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

} // namespace hierarchical_scheduler
} // namespace llvm
