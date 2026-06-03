//===- PressureHistoryTracker.cpp - Pressure history-based domination -----===//
//
// Implementation. See header for the contract.
//
//===----------------------------------------------------------------------===//

//========================================================================================
// jbaile
//========================================================================================

#include "PressureHistoryTracker.h"
#include "ScheduleConstructor.h"
#include "llvm/Support/ErrorHandling.h"
#include <algorithm>

namespace llvm {
namespace hierarchical_scheduler {

PressureHistoryTracker::PressureHistoryTracker(
    const ScheduledSetTracker *scheduled_set_tracker,
    const ScheduleConstructor *working_schedule_constructor,
    ScheduleMetric metric)
    : scheduled_set_tracker_(scheduled_set_tracker),
      working_schedule_constructor_(working_schedule_constructor),
      metric_(metric) {
  if (scheduled_set_tracker_ == nullptr) {
    report_fatal_error(
        "PressureHistoryTracker: scheduled_set_tracker must not be null");
  }
  // working_schedule_constructor_ is permitted to be null -- see
  // header for the test-fixture contract.
}

bool PressureHistoryTracker::IsDominatedElseRecord() {
  if (working_schedule_constructor_ == nullptr) {
    report_fatal_error(
        "PressureHistoryTracker::IsDominatedElseRecord(): bound working "
        "schedule constructor must be non-null for the no-arg overload; "
        "either bind it at construction or call the explicit-Score overload");
  }
  return IsDominatedElseRecord(
      working_schedule_constructor_->GetScore(metric_));
}

bool PressureHistoryTracker::IsDominatedElseRecord(
    const Score &current_score) {
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
    if (static_cast<int>(table_.size()) >= kMaxEntries) {
      // Soft cap: stop recording, set the sticky flag, let the
      // search continue. No prior entry exists for this
      // partition, so "not dominated" is the truthful answer
      // regardless of whether we recorded.
      memory_cap_hit_.Set();
      return false;
    }
    PartitionKey key = scheduled_set_tracker_->GetPartitionKey();
    table_[std::move(key)] = Entry{current_score};
    return false;
  }

  Entry &prior = it->second;
  // Lexicographic Score domination. Prune when the prior prefix's
  // Score is >= this one lexicographically. By the partition's
  // prefix/postfix decoupling, the shared postfix adds the same per-
  // dim contribution to both, so the prior prefix dominates ours on
  // every completion.
  if (prior.best_score >= current_score) {
    prune_count_.Increment();
    return true;
  }

  // Not dominated => this prefix is lexicographically greater (total
  // order). Record it as the partition's new best.
  prior.best_score = current_score;
  return false;
}

void PressureHistoryTracker::Reset() {
  // table_.size() IS the entry count for this tracker (one entry
  // per partition), so clearing the map zeros GetTotalEntries()
  // implicitly — no separate counter to reset.
  table_.clear();
  prune_count_.ResetCurrentRun();
  memory_cap_hit_.ResetCurrentRun();
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
