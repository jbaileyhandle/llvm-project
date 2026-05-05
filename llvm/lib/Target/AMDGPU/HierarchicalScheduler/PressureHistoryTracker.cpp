//===- PressureHistoryTracker.cpp - Pressure history-based domination -----===//
//
// Implementation. See header for the contract.
//
//===----------------------------------------------------------------------===//

//========================================================================================
// jbaile
//========================================================================================

#include "PressureHistoryTracker.h"
#include "llvm/Support/ErrorHandling.h"
#include <algorithm>

namespace llvm {
namespace hierarchical_scheduler {

PressureHistoryTracker::PressureHistoryTracker(
    const ScheduledSetTracker *scheduled_set_tracker,
    const GCNRegisterTracker *working_register_tracker,
    ScheduleMetric metric)
    : scheduled_set_tracker_(scheduled_set_tracker),
      working_register_tracker_(working_register_tracker),
      metric_(metric) {
  if (scheduled_set_tracker_ == nullptr) {
    report_fatal_error(
        "PressureHistoryTracker: scheduled_set_tracker must not be null");
  }
  // working_register_tracker_ is permitted to be null — see header
  // for the test-fixture contract.
}

bool PressureHistoryTracker::IsDominatedElseRecord() {
  if (working_register_tracker_ == nullptr) {
    report_fatal_error(
        "PressureHistoryTracker::IsDominatedElseRecord(): bound working "
        "register tracker must be non-null for the no-arg overload; either "
        "bind it at construction or call the explicit-score overload");
  }
  return IsDominatedElseRecord(
      working_register_tracker_->GetMetricScore(metric_));
}

bool PressureHistoryTracker::IsDominatedElseRecord(
    int current_prefix_score) {
  // Heterogeneous lookup via PartitionKeyView avoids copying the
  // bitset on the existing-entry path. DenseMap dispatches on
  // signature for the hash bucket, then disambiguates with the
  // full bitset via DenseMapInfo<PartitionKey>::isEqual.
  PartitionKeyView view = scheduled_set_tracker_->GetPartitionKeyView();

  auto it = table_.find_as(view);
  if (it == table_.end()) {
    // First visit to this partition. Build the owning key and
    // insert a fresh entry. No prior entry, so no dominator
    // exists.
    PartitionKey key = scheduled_set_tracker_->GetPartitionKey();
    Entry fresh;
    fresh.best_prefix_score = current_prefix_score;
    table_[std::move(key)] = std::move(fresh);
    return false;
  }

  Entry &prior = it->second;
  if (prior.best_prefix_score >= current_prefix_score) {
    // Prior prefix is no worse on the only prefix-dependent
    // dimension. By the partition's prefix/postfix decoupling,
    // anything our subtree could reach is reachable at no worse
    // score from the prior prefix. Prune.
    ++prune_count_;
    return true;
  }

  // Current prefix is strictly better. Update best-prefix score.
  // (max() is defensive — current is known to be strictly greater
  // here, but the symmetry makes the intent obvious.)
  prior.best_prefix_score =
      std::max(prior.best_prefix_score, current_prefix_score);
  return false;
}

void PressureHistoryTracker::InsertEntryForTest(
    const PartitionKey &key, Entry entry) {
  table_[key] = std::move(entry);
}

const PressureHistoryTracker::Entry *
PressureHistoryTracker::GetEntryForTest(const PartitionKey &key) const {
  auto it = table_.find(key);
  if (it == table_.end()) {
    return nullptr;
  }
  return &it->second;
}

} // namespace hierarchical_scheduler
} // namespace llvm

//========================================================================================
