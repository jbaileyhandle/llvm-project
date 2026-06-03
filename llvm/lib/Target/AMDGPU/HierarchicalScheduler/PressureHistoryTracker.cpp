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
  // bitset on the existing-bucket path. DenseMap dispatches on
  // signature for the hash bucket, then disambiguates with the
  // full bitset via DenseMapInfo<PartitionKey>::isEqual.
  PartitionKeyView view = scheduled_set_tracker_->GetPartitionKeyView();

  auto it = table_.find_as(view);
  Bucket *bucket = (it == table_.end()) ? nullptr : &it->second;

  // Pareto dominance walk: if any existing entry dominates
  // current_score on every slot, prune.
  if (bucket != nullptr) {
    for (const Entry &entry : *bucket) {
      if (entry.best_score.Dominates(current_score)) {
        prune_count_.Increment();
        return true;
      }
    }
  }

  // Not dominated. If inserting would push past the soft cap, skip
  // recording but don't claim dominance (we already verified no
  // recorded entry dominates current).
  if (total_entries_ >= kMaxEntries) {
    memory_cap_hit_.Set();
    return false;
  }

  if (bucket == nullptr) {
    // First visit to this partition. Build the owning key and seed
    // a one-element bucket.
    PartitionKey key = scheduled_set_tracker_->GetPartitionKey();
    Bucket fresh;
    fresh.push_back(Entry{current_score});
    table_[std::move(key)] = std::move(fresh);
    total_entries_++;
    return false;
  }

  // Pareto trim: drop existing entries that current_score now
  // dominates -- they're obsolete on the new frontier. (The walk
  // above already confirmed no existing entry dominates current,
  // so trimming can't remove a dominator of the new entry.) Then
  // push the new entry.
  int size_before = static_cast<int>(bucket->size());
  bucket->erase(std::remove_if(bucket->begin(), bucket->end(),
                               [&](const Entry &e) {
                                 return current_score.Dominates(
                                     e.best_score);
                               }),
                bucket->end());
  int removed = size_before - static_cast<int>(bucket->size());
  bucket->push_back(Entry{current_score});
  total_entries_ += 1 - removed;
  return false;
}

void PressureHistoryTracker::Reset() {
  table_.clear();
  total_entries_ = 0;
  prune_count_.ResetCurrentRun();
  memory_cap_hit_.ResetCurrentRun();
}

void PressureHistoryTracker::InsertEntryForTest(
    const PartitionKey &key, Entry entry) {
  table_[key].push_back(std::move(entry));
  total_entries_++;
}

ArrayRef<PressureHistoryTracker::Entry>
PressureHistoryTracker::GetBucketForTest(const PartitionKey &key) const {
  auto it = table_.find(key);
  if (it == table_.end()) {
    return {};
  }
  return it->second;
}

} // namespace hierarchical_scheduler
} // namespace llvm

//========================================================================================
