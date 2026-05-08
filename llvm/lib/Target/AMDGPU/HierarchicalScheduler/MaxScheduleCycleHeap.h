//===- MaxScheduleCycleHeap.h - Heap of unscheduled max cycles ---*- C++ -*-===//
//
// Min-heap (backed by std::set) of unscheduled instructions keyed by
// their max schedule cycle. The "earliest deadline" — the smallest
// max_schedule_cycle among instructions still to schedule — is
// available in O(1) via the set's begin() iterator.
//
// Owned by ScheduleLengthTracker as a member. The heap reads its data
// (max_schedule_cycle_by_topo_index_, scheduled_cycle_by_topo_index_,
// graph nodes, current_cycle_) via friend access on the tracker, so
// callers don't need to plumb arrays or predicates through the heap's
// API.
//
// Maintenance contract (called by the tracker):
//   - Schedule(node)              -> Remove(node)
//   - Unschedule(node)            -> Insert(node)
//   - SetMaxAcceptableScheduleLength -> Rebuild() (cycle values
//     change, so the entire heap is replaced)
//
// Cycle values are stable between Rebuild calls; Insert/Remove rely
// on that stability to find the correct entry.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MAXSCHEDULECYCLEHEAP_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MAXSCHEDULECYCLEHEAP_H

#include <optional>
#include <set>
#include <tuple>

namespace llvm {
namespace hierarchical_scheduler {

class ScheduleLengthTracker;
class ScheduleNode;

class MaxScheduleCycleHeap {
public:
  // One entry per unscheduled instruction. Sorted ascending by
  // max_cycle, with topo_idx as a tiebreaker so two instructions
  // sharing a max_cycle both fit (std::set requires unique keys).
  // begin() exposes the earliest deadline. Public so callers can
  // inspect the heap top via Peek (used by shakedowns).
  struct Entry {
    int max_cycle;
    int topo_idx;
    bool operator<(const Entry &o) const {
      return std::tie(max_cycle, topo_idx) <
             std::tie(o.max_cycle, o.topo_idx);
    }
  };

  // Heap stores a back-pointer to the tracker for friend access to
  // its data. The constructor only stores the address — it does not
  // call any methods on tracker — so passing *this from the
  // tracker's initializer list (mid-construction) is safe.
  explicit MaxScheduleCycleHeap(const ScheduleLengthTracker &tracker);

  // Add an entry for the given node / topo index. The pair stored
  // is (max_schedule_cycle[topo_idx], topo_idx); the heap reads
  // max_schedule_cycle from the tracker.
  void Insert(int topo_idx);
  void Insert(const ScheduleNode *node);

  // Remove the entry for the given node / topo index. Caller must
  // guarantee max_schedule_cycle[topo_idx] holds the same value as
  // when Insert was called for this topo index — i.e., no
  // SetMaxAcceptableScheduleLength has fired in between (the
  // tracker's contract).
  void Remove(int topo_idx);
  void Remove(const ScheduleNode *node);

  // Empty the heap. Callers usually follow with Rebuild or with a
  // sequence of Insert calls.
  void Clear();

  // Empty + insert one entry per currently-unscheduled scheduling
  // unit, reading scheduled state and the max_schedule_cycle table
  // straight from the tracker. Called by the tracker after
  // SetMaxAcceptableScheduleLength has populated the table at new
  // values (the previous entries are stale because their max_cycle
  // keys are out of date).
  void Rebuild();

  // True iff the smallest stored max_schedule_cycle is below the
  // tracker's current_cycle — i.e., the search has advanced past a
  // deadline that no completion of this prefix can recover from
  // without exceeding max_acceptable_schedule_length. O(1) (reads
  // entries_.begin() and tracker_->current_cycle_).
  bool IsCurrentCycleBeyondEarliestMaxCycle() const;

  // Read-only inspection of the heap's top (smallest entry). Returns
  // nullopt when the heap is empty. Intended for shakedowns and
  // diagnostics that want to verify Insert/Remove/Rebuild produced
  // the expected ordering; production prune logic uses
  // IsCurrentCycleBeyondEarliestMaxCycle instead.
  std::optional<Entry> Peek() const;

  // Number of entries currently in the heap. Diagnostic; production
  // code should rely on IsCurrentCycleBeyondEarliestMaxCycle for
  // pruning.
  int Size() const;

private:
  const ScheduleLengthTracker *tracker_;
  std::set<Entry> entries_;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MAXSCHEDULECYCLEHEAP_H
