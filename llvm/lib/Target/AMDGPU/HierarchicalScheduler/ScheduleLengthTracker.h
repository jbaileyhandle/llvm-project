//===- ScheduleLengthTracker.h - Schedule length tracking -------*- C++ -*-===//
//
// Tracks schedule length and bubbles (stall cycles) during incremental
// forward schedule construction. Supports do/undo for search algorithms
// (beam search, branch-and-bound) and is fully copyable.
//
// The model assumes IssueWidth = 1: one instruction per cycle. This
// assumption is checked at construction time. The schedule length is
// determined entirely by the instruction ordering and data dependency
// latencies on edges. No resource contention is modeled. See
// Appendix M of AMDGPUMachineSchedulerGuide.md for the rationale.
//
// For each scheduled instruction:
//   ready_cycle = max(current_cycle,
//                     max(pred.scheduled_cycle + edge.latency)
//                         for all scheduled predecessors)
//   bubble_cycles += ready_cycle - current_cycle
//   scheduled_cycle[node] = ready_cycle
//   current_cycle = ready_cycle + 1
//
// Bounding support: the tracker can compute a lower bound on the
// final schedule length from the current partial schedule. This
// requires critical path data precomputed on the graph (longest
// latency-weighted path from each node to the exit). The lower
// bound is:
//   max(current_cycle + num_unscheduled,
//       max(scheduled_cycle[n] + critical_path_from[n])
//           for all scheduled nodes n)
// If this exceeds a known best, the branch can be pruned.
// Critical path precomputation is not part of this class — it
// belongs on the ScheduleGraph and would be passed in for queries.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULELENGTHTRACKER_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULELENGTHTRACKER_H

#include "ScheduleGraph.h"
#include "llvm/ADT/DenseMap.h"
#include <string>
#include <vector>

namespace llvm {

class GCNSubtarget;

namespace hierarchical_scheduler {

class ScheduleLengthTracker {
public:
  /// Construct from a graph and subtarget. On first construction for
  /// a given graph (identified by its unique ID), validates that:
  ///   - IssueWidth == 1 (from the subtarget's scheduling model)
  ///   - No node in the graph uses a reserved/unbuffered resource
  ///     (SUnit::hasReservedResource)
  /// Subsequent constructions with the same graph ID skip validation.
  /// Reports fatal error if validation fails.
  ScheduleLengthTracker(const ScheduleGraph &graph,
                        const GCNSubtarget &st);

  /// Schedule a node. Computes its ready cycle from already-scheduled
  /// predecessors' data dependency edges (kData), records the cycle,
  /// updates bubble count, and advances the cycle counter.
  void Schedule(const ScheduleNode *node);

  /// Reverse the last Schedule() call. The tracker remembers what
  /// was last scheduled — no argument needed.
  void Unschedule();

  /// Current schedule length — the cycle at which the next instruction
  /// would be placed. After scheduling all N instructions, this is the
  /// total number of cycles used.
  int GetCurrentCycle() const { return current_cycle_; }

  /// Total bubble (stall) cycles accumulated so far. A bubble occurs
  /// when an instruction's ready cycle is later than the current cycle,
  /// meaning the processor stalls waiting for a dependency.
  int GetTotalBubbles() const { return total_bubbles_; }

  /// The cycle at which a specific node was scheduled.
  /// Only valid for nodes that have been scheduled.
  int GetScheduledCycle(const ScheduleNode *node) const;

  /// Whether a node has been scheduled.
  bool IsScheduled(const ScheduleNode *node) const {
    return scheduled_cycle_.count(node) > 0;
  }

  /// Number of nodes scheduled so far.
  int GetNumScheduled() const {
    return static_cast<int>(scheduled_cycle_.size());
  }

  /// Human-readable summary.
  std::string Describe() const;

private:
  DenseMap<const ScheduleNode *, int> scheduled_cycle_;
  int current_cycle_ = 0;
  int total_bubbles_ = 0;

  /// Undo record for one Schedule() call.
  struct ScheduleStep {
    const ScheduleNode *node;
    int prev_cycle;
    int prev_bubbles;
  };

  std::vector<ScheduleStep> undo_stack_;

  /// Validates the graph and subtarget assumptions. Called once per
  /// graph (results cached internally by graph ID).
  static void ValidateGraph(const ScheduleGraph &graph,
                            const GCNSubtarget &st);
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULELENGTHTRACKER_H
