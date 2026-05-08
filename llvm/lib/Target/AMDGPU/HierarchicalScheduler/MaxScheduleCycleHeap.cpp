//===- MaxScheduleCycleHeap.cpp - Heap of unscheduled max cycles ----------===//

#include "MaxScheduleCycleHeap.h"

#include "ScheduleGraph.h"
#include "ScheduleLengthTracker.h"
#include "llvm/ADT/Twine.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

MaxScheduleCycleHeap::MaxScheduleCycleHeap(
    const ScheduleLengthTracker &tracker)
    : tracker_(&tracker) {}

void MaxScheduleCycleHeap::Insert(int topo_idx) {
  // No-op when no max acceptable schedule length has been set.
  // The tracker may invoke this from its Unschedule path before
  // any SetMaxAcceptableScheduleLength has fired (e.g., when the
  // tracker is used outside a target-aware DfsSearch); in that
  // case max_schedule_cycle_by_topo_index_ is empty and reading
  // it would be undefined behavior.
  if (!tracker_->HasMaxAcceptableScheduleLength()) {
    return;
  }
  // Insert must produce a fresh entry. A duplicate means
  // Schedule/Unschedule are out of sync — Insert has been called
  // without a matching prior Remove for this topo_idx — and any
  // downstream prune decision would be reading a stale heap.
  // Loud failure here pinpoints the violation rather than letting
  // it produce wrong-but-quiet results later.
  auto result = entries_.insert(
      {tracker_->max_schedule_cycle_by_topo_index_[topo_idx], topo_idx});
  if (!result.second) {
    report_fatal_error(
        "MaxScheduleCycleHeap::Insert: entry already present for topo_idx=" +
        Twine(topo_idx));
  }
}

void MaxScheduleCycleHeap::Insert(const ScheduleNode *node) {
  Insert(node->GetTopoIndex());
}

void MaxScheduleCycleHeap::Remove(int topo_idx) {
  // Mirror of Insert: no-op when no max acceptable schedule
  // length is set. (entries_ is empty in that state, so erase
  // would be a harmless no-op too, but the array read above is
  // not safe — keep both paths gated identically.)
  if (!tracker_->HasMaxAcceptableScheduleLength()) {
    return;
  }
  // Remove must find an entry to erase. A miss means
  // Schedule/Unschedule are out of sync — Remove has been called
  // without a matching prior Insert (or with a stale max_cycle
  // key from before SetMaxAcceptableScheduleLength rebuilt the
  // heap). Loud failure here pinpoints the violation rather than
  // letting it produce wrong-but-quiet results later.
  size_t erased = entries_.erase(
      {tracker_->max_schedule_cycle_by_topo_index_[topo_idx], topo_idx});
  if (erased == 0) {
    report_fatal_error(
        "MaxScheduleCycleHeap::Remove: no entry to remove for topo_idx=" +
        Twine(topo_idx));
  }
}

void MaxScheduleCycleHeap::Remove(const ScheduleNode *node) {
  Remove(node->GetTopoIndex());
}

void MaxScheduleCycleHeap::Clear() { entries_.clear(); }

void MaxScheduleCycleHeap::Rebuild() {
  Clear();
  // Caller (the tracker's SetMaxAcceptableScheduleLength) has just
  // populated max_schedule_cycle_by_topo_index_ at the new length,
  // so Insert below is safe to dereference the array. If somehow
  // called when no length is set, the per-Insert guard catches it
  // and the heap stays empty.
  //
  // Iterate by node, not by vector position: a node's topo_index
  // is assigned by Kahn's algorithm in ComputeTopologicalOrder
  // and does not match its position in nodes_. Using vector
  // position to index scheduled_cycle_by_topo_index_ would mix
  // up which nodes are skipped and which get inserted.
  const ScheduleGraph &graph = *tracker_->graph_;
  for (const ScheduleNode &node : graph.Nodes()) {
    if (!node.IsSchedulingUnit()) {
      // Subgraph proxies don't have real deadlines; they never go
      // through Schedule/Unschedule and so are not tracked here.
      continue;
    }
    int topo_idx = node.GetTopoIndex();
    if (tracker_->scheduled_cycle_by_topo_index_[topo_idx] >= 0) {
      // Already scheduled. Skip.
      continue;
    }
    Insert(topo_idx);
  }
}

bool MaxScheduleCycleHeap::IsCurrentCycleBeyondEarliestMaxCycle() const {
  if (entries_.empty()) {
    return false;
  }
  return entries_.begin()->max_cycle < tracker_->current_cycle_;
}

std::optional<MaxScheduleCycleHeap::Entry>
MaxScheduleCycleHeap::Peek() const {
  if (entries_.empty()) {
    return std::nullopt;
  }
  return *entries_.begin();
}

int MaxScheduleCycleHeap::Size() const {
  return static_cast<int>(entries_.size());
}
