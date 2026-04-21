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
// Bounding support: the tracker computes a lower bound on the final
// schedule length from the current partial schedule:
//
//   LB = max(current_cycle + num_unscheduled,
//            max over scheduled n of
//                (scheduled_cycle[n] + cp_from_exit[n]))
//
// First term: each unscheduled node still needs at least one more
// cycle, so length >= current_cycle + (remaining nodes).
//
// Second term: for any scheduled node n, successors must issue
// respecting edge latencies, so the exit issues at cycle
// >= scheduled_cycle[n] + cp_from_exit[n]. Taking the max across
// all scheduled nodes gives the tightest such floor.
//
// The second-term max is maintained incrementally in
// max_scheduled_plus_cp_, so GetLengthLowerBound is O(1) (no sweep
// over scheduled nodes). Schedule bumps the max; Unschedule restores
// the prior value from the undo record.
//
// PRECONDITION: cp_from_exit must be computed on the graph
// (ScheduleGraph::ComputeCriticalPathFromExit) before the tracker is
// constructed. The tracker asserts this via report_fatal_error in
// the constructor.
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
    return scheduled_cycle_by_topo_index_[node->GetTopoIndex()] >= 0;
  }

  /// Number of nodes scheduled so far.
  int GetNumScheduled() const {
    // Each Schedule() call pushes one entry; each Unschedule() pops
    // one. So the stack's size is the count.
    return static_cast<int>(undo_stack_.size());
  }

  /// Lower bound on the final schedule length given the current
  /// partial schedule. See the class-level comment for the formula.
  /// O(1).
  int GetLengthLowerBound() const;

  /// Human-readable summary.
  std::string Describe() const;

private:
  const ScheduleGraph *graph_;
  /// Cycle at which each node was scheduled, indexed by topo_index.
  /// Sentinel -1 means "not scheduled." Sized to graph.Size() at
  /// construction. Topo indices are stable for the tracker's
  /// lifetime because the construction-time precondition (cp must
  /// be computed on the graph) requires ComputeTopologicalOrder
  /// first; any subsequent topo re-sort would invalidate cp and
  /// therefore the tracker's contract.
  std::vector<int> scheduled_cycle_by_topo_index_;
  int current_cycle_ = 0;
  int total_bubbles_ = 0;
  /// Second-term max maintained incrementally across Schedule /
  /// Unschedule. Sum `scheduled_cycle[n] + cp_from_exit[n]` over all
  /// currently-scheduled nodes, kept as a running max.
  int max_scheduled_plus_cp_ = 0;

  /// Undo record for one Schedule() call.
  struct ScheduleStep {
    const ScheduleNode *node;
    int prev_cycle;
    int prev_bubbles;
    int prev_max_scheduled_plus_cp;
  };

  std::vector<ScheduleStep> undo_stack_;

  /// Validates the graph and subtarget assumptions. Called once per
  /// graph (results cached internally by graph ID).
  static void ValidateGraph(const ScheduleGraph &graph,
                            const GCNSubtarget &st);

  /// Cycle at which `node` would be ready to issue, given the current
  /// partial schedule. Walks `node`'s latency-carrying predecessors
  /// and takes the latest pred_cycle + edge_latency.
  int ComputeReadyCycle(const ScheduleNode *node) const;

  /// Snapshot all mutable state onto undo_stack_ so the next
  /// Unschedule can revert.
  void PushUndoStep(const ScheduleNode *node);

  /// Advance the schedule by placing `node` at `ready_cycle`. Updates
  /// scheduled_cycle_, current_cycle_, total_bubbles_.
  void AdvanceSchedule(const ScheduleNode *node, int ready_cycle);

  /// Bump the running max for GetLengthLowerBound's second term
  /// using `node`'s just-computed ready_cycle and cp_from_exit.
  void UpdateLengthLowerBoundMax(const ScheduleNode *node, int ready_cycle);
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULELENGTHTRACKER_H
