//===- MlpTracker.h - Memory-level parallelism tracking ---------*- C++ -*-===//
//
// Walks an order of a MemoryGraph's nodes through the wait queues, one node
// at a time, and scores it: the MLP of the tracked reads, the elapsed time,
// and the queues' peak sizes. See llvm/docs/AMDGPUMemoryGraphDesign.md,
// section 5.
//
// The walk keeps a clock. Issuing a node takes no time. The clock moves only
// when a node has to wait for data that isn't back yet (a barrier waits for
// every op in flight), and then it jumps to when the wait ends. A load or
// write starts at the clock's time when it issues.
//
// There's no Unschedule yet. A search that needs one can add it: each
// WalkStep lists the ops its waits popped, which is what Unschedule puts
// back.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MLPTRACKER_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MLPTRACKER_H

#include "MemoryGraph.h"
#include "WaitQueues.h"
#include "llvm/ADT/ArrayRef.h"
#include <vector>

namespace llvm::hierarchical_scheduler {

class MlpTracker {
public:
  /// The tracker keeps `graph`, so it must not be null and must outlive the
  /// tracker. The MLPs count only the reads `tracked_read_types` picks. The
  /// timing counts every op.
  MlpTracker(const MemoryGraph *graph, LatencySetting setting,
             const TrackedReadTypes &tracked_read_types);

  /// Walks one more node: its waits, then its issue. Doesn't check that the
  /// node's predecessors were scheduled before it.
  void Schedule(const MemoryNode *node);

  /// Where the clock is: the issue time of the last node scheduled, or 0
  /// before any. Once the sink is scheduled, this is the order's
  /// elapsed_time.
  int GetCurrentTime() const {
    return steps_.empty() ? 0 : steps_.back().issue_time;
  }

  /// One WalkStep per node scheduled so far, in order.
  ArrayRef<WalkStep> GetSteps() const { return steps_; }

  /// The score of the nodes scheduled. Fails hard unless the last one is the
  /// sink: until the sink's waits, some reads haven't been popped, so their
  /// use-based flights have no end.
  OrderScore GetScore() const;

private:
  const MemoryGraph *graph_;
  LatencySetting setting_;
  TrackedReadTypes tracked_read_types_;
  WaitQueues queues_;

  // One per node scheduled, in order. Each node's issue time and pops are
  // read from here, so nothing else keeps them.
  std::vector<WalkStep> steps_;
};

} // namespace llvm::hierarchical_scheduler

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_MLPTRACKER_H
