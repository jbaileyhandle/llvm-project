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
    const ScheduleLengthTracker *length_tracker)
    : scheduled_set_tracker_(scheduled_set_tracker),
      length_tracker_(length_tracker) {
  if (scheduled_set_tracker_ == nullptr) {
    report_fatal_error(
        "LengthHistoryTracker: scheduled_set_tracker must not be null");
  }
  if (length_tracker_ == nullptr) {
    report_fatal_error(
        "LengthHistoryTracker: length_tracker must not be null");
  }
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

bool LengthHistoryTracker::DoesDominate(const Entry &a, const Entry &b) {
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
  return true;
}

bool LengthHistoryTracker::IsDominated() const {
  PartitionKey key = scheduled_set_tracker_->GetPartitionKey();
  auto it = table_.find(key);
  if (it == table_.end()) {
    return false;
  }
  Entry query{length_tracker_->GetCurrentCycle(), GetFrontierLbsSnapshot()};
  for (const Entry &existing : it->second) {
    if (DoesDominate(existing, query)) {
      return true;
    }
  }
  return false;
}

bool LengthHistoryTracker::IsDominatedElseInsert() {
  PartitionKey key = scheduled_set_tracker_->GetPartitionKey();
  Entry query{length_tracker_->GetCurrentCycle(), GetFrontierLbsSnapshot()};

  // operator[] default-constructs the bucket if absent. Safe to do
  // up front: if no dominator exists we'll be inserting anyway, and
  // the empty bucket costs nothing if we early-return.
  SmallVector<Entry, 2> &bucket = table_[key];

  // Single-pass walk: look for an existing dominator while
  // collecting indices of entries the query dominates. If a
  // dominator is found, return immediately — the dominated indices
  // collected so far are irrelevant (we're not inserting).
  SmallVector<size_t, 4> dominated_indices;
  for (size_t i = 0; i < bucket.size(); ++i) {
    const Entry &existing = bucket[i];
    if (DoesDominate(existing, query)) {
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
    report_fatal_error(
        "LengthHistoryTracker: insertion would exceed kMaxEntries (" +
        Twine(kMaxEntries) + "); search produced too many partitions "
        "for the current dominance pruning to control");
  }

  bucket.push_back(std::move(query));
  ++total_entries_;
  return false;
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
