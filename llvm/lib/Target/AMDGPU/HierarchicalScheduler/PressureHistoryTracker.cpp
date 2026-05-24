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
  // The tracker always accumulates occupancy area, but only the
  // area-tiebreak metric should let it affect domination; pass 0 for
  // every other metric so their pruning stays peak-only.
  int64_t area =
      metric_ == ScheduleMetric::kMaximizeContinuousOccupancyThenArea
          ? working_register_tracker_->GetContinuousOccupancyArea()
          : 0;
  return IsDominatedElseRecord(
      working_register_tracker_->GetMetricScore(metric_), area);
}

bool PressureHistoryTracker::IsDominatedElseRecord(int current_prefix_score) {
  return IsDominatedElseRecord(current_prefix_score, /*current_area=*/0);
}

bool PressureHistoryTracker::IsDominatedElseRecord(int current_peak_score,
                                                   int64_t current_area) {
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
    Entry fresh;
    fresh.best_prefix_score = current_peak_score;
    fresh.best_area = current_area;
    table_[std::move(key)] = std::move(fresh);
    return false;
  }

  Entry &prior = it->second;
  // Lexicographic (peak, area) domination. Prune when the prior
  // prefix is no worse on peak, or ties peak and is no worse on area.
  // By the partition's prefix/postfix decoupling the shared postfix
  // adds the same peak and the same area to both, so the prior prefix
  // dominates ours on every completion. For pure-peak callers area is
  // 0 on both sides and this is the old best_prefix_score >= check.
  bool prior_dominates =
      prior.best_prefix_score > current_peak_score ||
      (prior.best_prefix_score == current_peak_score &&
       prior.best_area >= current_area);
  if (prior_dominates) {
    prune_count_.Increment();
    return true;
  }

  // Not dominated ⇒ this prefix is lexicographically greater (total
  // order). Record it as the partition's new best.
  prior.best_prefix_score = current_peak_score;
  prior.best_area = current_area;
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
