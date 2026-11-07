//===- WaitQueues.h - The vmcnt and lgkmcnt wait queues ---------*- C++ -*-===//
//
// The model of the hardware's wait counters that MemoryGraph::ScoreOrder uses
// to walk an order: which ops are still outstanding, the earliest a wait can
// pop each one, and when a wait has to be a drain. See
// llvm/docs/AMDGPUMemoryGraphDesign.md, sections 5.2 and 5.3.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_WAITQUEUES_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_WAITQUEUES_H

#include "MemoryGraph.h"
#include "llvm/ADT/ArrayRef.h"
#include <vector>

namespace llvm {
namespace hierarchical_scheduler {

/// The kind of an op in a wait queue, named after the compiler's WaitEventType
/// values for vmcnt and lgkmcnt (SIInsertWaitcnts.cpp). A wait on a queue that
/// holds a scalar op, or more than one kind, becomes a drain
/// (counterOutOfOrder).
enum class WaitEvent { kVmemAccess, kLdsAccess, kGdsAccess, kSmemAccess };

/// The kind an op has in the lgkmcnt queue, as updateEventWaitcntAfter assigns
/// it. The LDS side of a FLAT records the same LDS_ACCESS as an LDS op. In the
/// vmcnt queue every op is kVmemAccess: gfx906 has no vscnt (hasVscnt() is
/// false), so getVmemWaitEventType gives loads and stores the same event.
WaitEvent LgkmcntEvent(MemType type);

/// Whether an op is a FLAT that goes in both queues, which turns on cautious
/// mode (design doc section 5.2).
bool IsFlatInBothQueues(const MemoryNode &node);

/// One op in a wait queue. Push sets every field.
struct QueueEntry {
  const MemoryNode *node;

  /// The op's event in this queue.
  WaitEvent event;

  /// The earliest time a wait can pop this op. A queue pops ops in the order
  /// they were issued, so this is the later of the op's own
  /// raw_op_complete_time (its issue time plus its latency) and the
  /// earliest_possible_pop_time of the entry before it in the same queue.
  int earliest_possible_pop_time;

  /// True for a tracked read (TrackedReadTypes).
  bool counts_in_tracked_peak;
};

/// What the waits before one node do.
struct NodeWait {
  /// When the node issues, after its waits. Never earlier than the
  /// issue_wait_begin_time passed to WaitFor or DrainAll, and equal to it
  /// when no wait holds the node back.
  int issue_time = 0;

  /// The ops this node's waits popped from the queues. A FLAT that goes in
  /// both queues can be listed twice, once per queue.
  std::vector<const MemoryNode *> popped;
};

/// Which of the two wait queues a WaitQueue is. The type decides which ops go
/// in the queue and what event each one gets.
enum class WaitQueueType { kVmcnt, kLgkmcnt };

/// One wait queue, vmcnt or lgkmcnt: the ops issued into it that no wait has
/// popped yet, oldest first (design doc section 5.3).
class WaitQueue {
public:
  /// `tracked_read_types` picks the reads that count in the tracked peak. It
  /// changes no timing.
  explicit WaitQueue(
      WaitQueueType type,
      const TrackedReadTypes &tracked_read_types = TrackedReadTypes())
      : type_(type), tracked_read_types_(tracked_read_types) {}

  /// Puts an op at the back of the queue, with its event in this queue and
  /// whether it counts in the tracked peak. Fails hard if the op doesn't go in
  /// this queue. A queue pops ops in the order they were issued, so the op
  /// can't be popped before the entry already at the back. If the queue is
  /// empty, every earlier op was already popped, no later than now, so
  /// nothing holds this one back.
  void Push(const MemoryNode &node, int raw_op_complete_time);

  /// Pops `node` and every entry before it, and returns them, oldest first.
  /// `node` must be in the queue.
  std::vector<QueueEntry> Wait(const MemoryNode &node);

  /// Pops every entry and returns them, oldest first.
  std::vector<QueueEntry> Drain();

  bool Holds(const MemoryNode &node) const;

  /// Whether a wait on this queue has to be a drain, because its entries can
  /// be reported done out of order: one of them is a scalar op, or they
  /// aren't all the same kind (counterOutOfOrder; the ISA manual passages are
  /// quoted in design doc section 5.2). Only the lgkmcnt queue can hold
  /// either.
  bool HoldsOutOfOrderKinds() const;

  /// Whether a FLAT that goes in both queues is in this one. Cautious mode is
  /// on while one is in either queue.
  bool HoldsFlatInBothQueues() const;

  /// The entries not yet popped, oldest first.
  ArrayRef<QueueEntry> Entries() const { return entries_; }

  /// The most entries this queue ever held at once.
  int GetPeakSize() const { return peak_size_; }

  /// The most entries that count in the tracked peak this queue ever held at
  /// once.
  int GetPeakTrackedSize() const { return peak_tracked_size_; }

private:
  /// If `pushed` counts in the tracked peak, raises tracked_size_, and
  /// peak_tracked_size_ with it when that's a new high.
  void RaiseTrackedSize(const QueueEntry &pushed);

  /// Lowers tracked_size_ by one for each entry of `popped` that counts in
  /// the tracked peak.
  void LowerTrackedSize(ArrayRef<QueueEntry> popped);

  WaitQueueType type_;
  TrackedReadTypes tracked_read_types_;

  std::vector<QueueEntry> entries_;  // oldest first
  int peak_size_ = 0;

  // The entries now in the queue that count in the tracked peak, and the most
  // there have ever been.
  int tracked_size_ = 0;
  int peak_tracked_size_ = 0;
};

/// Both wait queues, and the rules that involve both (design doc section 5.3).
/// The walk talks only to this.
class WaitQueues {
public:
  /// `tracked_read_types` picks the reads that count in each queue's tracked
  /// peak. It changes no timing.
  explicit WaitQueues(
      const TrackedReadTypes &tracked_read_types = TrackedReadTypes())
      : vmcnt_(WaitQueueType::kVmcnt, tracked_read_types),
        lgkmcnt_(WaitQueueType::kLgkmcnt, tracked_read_types) {}

  /// Issues a memory op into the vmcnt queue, the lgkmcnt queue or both, as
  /// its MemOpInfo says.
  void Issue(const MemoryNode &node, int raw_op_complete_time);

  /// The waits before a node that reads the data of `loads_used`.
  /// `issue_wait_begin_time` is when the walk reaches the node: the issue time
  /// of the node before it. A load that's no longer in any queue was already
  /// popped by an earlier wait, so it needs none.
  NodeWait WaitFor(ArrayRef<const MemoryNode *> loads_used,
                   int issue_wait_begin_time);

  /// The wait before an s_barrier: both queues drain. `issue_wait_begin_time`
  /// is as for WaitFor.
  NodeWait DrainAll(int issue_wait_begin_time);

  /// Cautious mode is on while a FLAT that goes in both queues is still in
  /// either queue. While it's on every wait is a drain, so the FLAT can only
  /// leave a queue through a drain, and "still in a queue" means exactly
  /// "that queue hasn't drained since the FLAT" (design doc section 5.2).
  bool CautiousMode() const;

  /// Read-only views of the two queues, for their contents and peak sizes.
  /// Only WaitQueues pushes and pops.
  const WaitQueue &GetVmcntQueue() const { return vmcnt_; }
  const WaitQueue &GetLgkmcntQueue() const { return lgkmcnt_; }

private:
  /// Whether a wait on `queue` has to be a drain: cautious mode is on, or the
  /// queue's entries can be reported done out of order.
  static bool ShouldDrain(const WaitQueue &queue, bool cautious);

  /// Records entries a wait popped. The node can't issue before every one of
  /// them could be popped, so its issue time moves to the latest of their
  /// earliest_possible_pop_times, if that's later than it already is.
  static void AddPopped(const std::vector<QueueEntry> &popped,
                        NodeWait &wait);

  WaitQueue vmcnt_;
  WaitQueue lgkmcnt_;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_WAITQUEUES_H
