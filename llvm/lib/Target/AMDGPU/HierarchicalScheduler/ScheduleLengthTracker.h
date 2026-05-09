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
//                (scheduled_cycle[n] + cp_from_exit[n] + 1))
//
// First term: each unscheduled node still needs at least one more
// cycle, so length >= current_cycle + (remaining nodes).
//
// Second term: every graph we process has a single latency-sink —
// the synthetic exit node in BuildFromSUnits graphs, or the unique
// terminal node in test DAGs — and that sink is the last instruction
// scheduled (every other node is a transitive predecessor of it).
// cp_from_exit[n] is the longest latency-weighted path from n to that
// sink, so sink.cycle >= scheduled_cycle[n] + cp_from_exit[n], and
// length = sink.cycle + 1 >= scheduled_cycle[n] + cp_from_exit[n] + 1.
// The trailing +1 is the cycle the sink itself occupies.
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

#include "MaxScheduleCycleHeap.h"
#include "ScheduleGraph.h"
#include "llvm/ADT/DenseMap.h"
#include <optional>
#include <string>
#include <vector>

namespace llvm {

class GCNSubtarget;

namespace hierarchical_scheduler {

class ScheduleLengthTracker {
  // The heap reads max_schedule_cycle_by_topo_index_,
  // scheduled_cycle_by_topo_index_, current_cycle_, and graph_
  // through this friend declaration so its API can stay narrow
  // (Insert/Remove by topo_idx with no array passing).
  friend class MaxScheduleCycleHeap;

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

  /// Reverse the last Schedule() call. The undo data is
  /// self-contained on undo_stack_; the `node` parameter exists
  /// only so the tracker can self-skip subgraph proxies (matching
  /// Schedule's early-return).
  void Unschedule(const ScheduleNode *node);

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

  /// Set the maximum schedule length the search will accept, and
  /// populate each node's max schedule cycle:
  ///   max_schedule_cycle[i] = max_acceptable_schedule_length - 1
  ///                                - cp_from_exit[i]
  ///
  /// Derivation: a schedule whose last instruction is at cycle
  /// L-1 has length exactly L (cycles 0..L-1 are used). Node i
  /// with cp_from_exit[i] forces a chain ending at cycle
  /// node_cycle + cp_from_exit[i], which must be <= L-1, so
  /// node_cycle <= L - 1 - cp_from_exit[i]. Taking L =
  /// max_acceptable_schedule_length yields the formula above.
  ///
  /// A node placed past its max schedule cycle pushes the chain
  /// past cycle L-1, growing the schedule beyond
  /// max_acceptable_schedule_length — i.e., infeasible at this target.
  /// Subsequent infrastructure consumes
  /// max_schedule_cycle_by_topo_index_ for a per-step pruning
  /// check that rejects such subtrees.
  ///
  /// max_acceptable_schedule_length is the same quantity the policy's
  /// ShouldBoundSearch derives for its LB-vs-best comparison:
  ///   min(iteration_target, best.length - 1)
  /// so callers should pass that value. DfsSearch maintains it,
  /// calling this setter at three points: at construction (with
  /// the input-derived seed), in ResetForReuse (with the new
  /// iteration's target), and on every best-improvement (when
  /// best.length - 1 drops below the previous setting).
  ///
  /// Idempotent: calling with the same value re-populates the
  /// vector to the same contents. Subsequent calls overwrite.
  ///
  /// Precondition: cp_from_exit must be computed on the graph
  /// (validated at tracker construction).
  void SetMaxAcceptableScheduleLength(int max_acceptable_schedule_length);

  /// True iff SetMaxAcceptableScheduleLength has been called.
  /// Both the stored max_acceptable_schedule_length_ and the
  /// derived per-node max_schedule_cycle_by_topo_index_ become
  /// readable when this returns true.
  bool HasMaxAcceptableScheduleLength() const {
    return max_acceptable_schedule_length_.has_value();
  }

  /// The max acceptable schedule length last passed to
  /// SetMaxAcceptableScheduleLength. Caller must ensure
  /// HasMaxAcceptableScheduleLength() — bare optional dereference,
  /// no guard.
  int GetMaxAcceptableScheduleLength() const {
    return *max_acceptable_schedule_length_;
  }

  /// Maximum schedule cycle (latest acceptable placement cycle)
  /// for a node, indexed by topo_index. Caller must ensure
  /// HasMaxAcceptableScheduleLength() — bare lookup, no guard.
  int GetMaxScheduleCycleByTopoIndex(int topo_idx) const {
    return max_schedule_cycle_by_topo_index_[topo_idx];
  }

  /// Convenience: same as above, but extracts the topo index from
  /// the node.
  int GetMaxScheduleCycle(const ScheduleNode *node) const {
    return GetMaxScheduleCycleByTopoIndex(node->GetTopoIndex());
  }

  /// Minimum schedule cycle (earliest possible placement cycle
  /// given the current partial schedule) for a node, indexed by
  /// topo_index. Initialized at construction to cp_from_entry
  /// (the static floor on an empty schedule); maintained
  /// incrementally as Schedule / Unschedule are called — see
  /// the data member's class-level comment below for the
  /// propagation rule.
  int GetMinScheduleCycleByTopoIndex(int topo_idx) const {
    return min_schedule_cycle_by_topo_index_[topo_idx];
  }

  int GetMinScheduleCycle(const ScheduleNode *node) const {
    return GetMinScheduleCycleByTopoIndex(node->GetTopoIndex());
  }

  /// Test-only mutable access to the underlying heap. Used by
  /// shakedowns that need to call Peek/Size to verify Insert/
  /// Remove/Rebuild produced the expected ordering. Production
  /// code uses IsCurrentCycleBeyondEarliestMaxScheduleCycle().
  const MaxScheduleCycleHeap &GetMaxScheduleCycleHeapForTest() const {
    return unscheduled_max_cycle_heap_;
  }

  /// True iff the search has advanced past the earliest max
  /// schedule cycle among unscheduled instructions — i.e., some
  /// unscheduled node's deadline has already been passed and no
  /// completion of this prefix can honor it without exceeding
  /// the configured max acceptable schedule length. The length
  /// policy's per-step prune check reads this.
  ///
  /// Returns false (no pruning) when no max acceptable schedule
  /// length has been set or no unscheduled instructions remain.
  ///
  /// Delegates to unscheduled_max_cycle_heap_, which maintains
  /// the unscheduled-by-max-schedule-cycle set across
  /// Schedule / Unschedule / SetMaxAcceptableScheduleLength so
  /// this query is O(1) (heap's begin()) at call time.
  bool IsCurrentCycleBeyondEarliestMaxScheduleCycle() const {
    return unscheduled_max_cycle_heap_
        .IsCurrentCycleBeyondEarliestMaxCycle();
  }

  /// True iff the most recent Schedule's forward propagation pushed
  /// some unscheduled node's min_schedule_cycle above its
  /// max_schedule_cycle — i.e., that node can no longer be placed
  /// in time, so no completion of this prefix can honor the
  /// configured max acceptable schedule length. Tighter complement
  /// to IsCurrentCycleBeyondEarliestMaxScheduleCycle: the heap-
  /// based check fires only after current_cycle has advanced past
  /// a deadline; this fires immediately as soon as propagation
  /// reveals that some node's earliest exceeds its latest, even
  /// before current_cycle catches up. The length policy's per-
  /// step prune check reads both.
  ///
  /// Returns false when no max acceptable schedule length has
  /// been set, or when no Schedule has fired yet, or when the
  /// most recent Schedule's propagation found no violation.
  bool IsAnyMinScheduleCycleBeyondMaxScheduleCycle() const {
    return min_exceeds_max_after_last_schedule_;
  }

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

  /// One entry in a Schedule's propagation undo trail: the topo
  /// index of an unscheduled node whose min_schedule_cycle was
  /// raised, and the value to restore to on Unschedule.
  struct PriorMinScheduleCycle {
    int topo_idx;
    int prior_min_schedule_cycle;
  };

  /// Undo record for one Schedule() call.
  struct ScheduleStep {
    const ScheduleNode *node;
    int prev_cycle;
    int prev_bubbles;
    int prev_max_scheduled_plus_cp;
    /// True iff min_exceeds_max_after_last_schedule_ was true
    /// before this Schedule. Restored on Unschedule so the bool
    /// always reflects the result of whatever Schedule is now
    /// most recent. Strictly speaking the search flow doesn't
    /// read the bool between an Unschedule and the next Schedule
    /// (which would reset it anyway), but undoing is cheap and
    /// makes the bool's semantics robust to future readers.
    bool prev_min_exceeds_max_after_last_schedule;
    /// One entry per unscheduled node whose min_schedule_cycle
    /// this Schedule's propagation raised. On Unschedule, each
    /// entry is restored. Empty when propagation found no nodes
    /// to update.
    std::vector<PriorMinScheduleCycle> prev_min_schedule_cycle_entries;
  };

  std::vector<ScheduleStep> undo_stack_;

  /// Per-node max schedule cycle (latest acceptable placement
  /// cycle), indexed by topo_index. Empty when SetMaxAcceptableScheduleLength
  /// has not been called. Populated and overwritten in place by
  /// SetMaxAcceptableScheduleLength as the effective target tightens (across
  /// outer-loop iterations and best-improvement events).
  ///
  /// Static for the duration between two SetMaxAcceptableScheduleLength calls:
  /// not touched by Schedule / Unschedule. The dependency on
  /// cp_from_exit (graph-static) and the caller-supplied
  /// max_acceptable_schedule_length (search-state) are both captured in
  /// SetMaxAcceptableScheduleLength's recompute.
  std::vector<int> max_schedule_cycle_by_topo_index_;

  /// Per-node min schedule cycle (earliest possible placement
  /// cycle given the current partial schedule), indexed by
  /// topo_index. Initialized at construction to cp_from_entry —
  /// the static floor that holds on an empty schedule. Maintained
  /// incrementally by Schedule / Unschedule:
  ///
  ///   On Schedule(N): forward transitive propagation through the
  ///   unscheduled forward cone of N. For each unscheduled
  ///   successor S reachable from N, recompute
  ///     new = max over predecessors P of S of
  ///             (effective_cycle_P
  ///                + max(edge.Latency(), P.IssueSlotsConsumed()))
  ///   where effective_cycle_P is scheduled_cycle[P] when P is
  ///   scheduled and min_schedule_cycle[P] otherwise. If new is
  ///   greater than the stored value, write it and continue
  ///   propagation through S's successors. Each prior value is
  ///   recorded into the matching ScheduleStep so Unschedule can
  ///   restore.
  ///
  ///   On Unschedule(N): walk the step's prev_min_schedule_cycle_by_topo_index
  ///   list and restore each entry to its prior value.
  ///
  /// Mirrors max_schedule_cycle_by_topo_index_: together they
  /// bracket each unscheduled node's feasible placement window.
  /// A node with min > max is infeasible at the current max
  /// acceptable schedule length.
  std::vector<int> min_schedule_cycle_by_topo_index_;

  /// The maximum schedule length the search will accept, as last
  /// passed to SetMaxAcceptableScheduleLength. Read by the length
  /// policy's aggregate lower-bound prune check (compares the
  /// tracker's GetLengthLowerBound against this value). Has no
  /// value before SetMaxAcceptableScheduleLength has been called;
  /// callers must guard reads with HasMaxAcceptableScheduleLength.
  std::optional<int> max_acceptable_schedule_length_;

  /// Min-heap of currently-unscheduled instructions keyed by
  /// max_schedule_cycle. Maintained across Schedule/Unschedule
  /// (entries removed/re-inserted) and SetMaxAcceptableScheduleLength
  /// (heap rebuilt because cycle values changed). Reads its
  /// inputs from this tracker via friend access. Construction
  /// passes *this; the heap's constructor only stores the
  /// pointer, so this is safe even though the tracker is still
  /// being initialized at that point.
  MaxScheduleCycleHeap unscheduled_max_cycle_heap_;

  /// Set inline during Schedule's forward propagation when some
  /// unscheduled node's raised min_schedule_cycle exceeds its
  /// max_schedule_cycle. Reset to false at the start of every
  /// Schedule and restored from the popped step's
  /// prev_min_exceeds_max_after_last_schedule on Unschedule.
  /// Read by the policy via
  /// IsAnyMinScheduleCycleBeyondMaxScheduleCycle.
  bool min_exceeds_max_after_last_schedule_ = false;

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

  /// Forward transitive propagation of min_schedule_cycle from a
  /// just-scheduled node. Drives a topo-ordered worklist of S's
  /// unscheduled forward cone, recomputing each node's min and
  /// raising the stored value where needed. Updates are recorded
  /// onto the back-of-stack ScheduleStep so Unschedule can
  /// restore. Sets min_exceeds_max_after_last_schedule_ if any
  /// raised value exceeds the corresponding max_schedule_cycle.
  void PropagateMinScheduleCycleForward(
      const ScheduleNode *just_scheduled);

  /// Recompute one unscheduled node's min_schedule_cycle from
  /// current state:
  ///   max over S's latency predecessors P of
  ///     (effective_cycle_P
  ///        + max(edge.Latency(), P.IssueSlotsConsumed()))
  /// where effective_cycle_P is scheduled_cycle[P] when P is
  /// scheduled and min_schedule_cycle[P] otherwise. Returns 0
  /// for nodes with no latency predecessors (entry nodes).
  int ComputeMinScheduleCycleForward(const ScheduleNode *S) const;

  /// Raise min_schedule_cycle[topo_idx] to `candidate_min` if
  /// strictly higher than the stored value. On a real raise:
  /// records the prior value into `step`, flips
  /// min_exceeds_max_after_last_schedule_ when the new value
  /// exceeds max_schedule_cycle[topo_idx] (and a max acceptable
  /// schedule length is set), and returns true. Returns false
  /// if no raise was needed.
  bool TryRaiseMinScheduleCycle(int topo_idx, int candidate_min,
                                ScheduleStep &step);
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULELENGTHTRACKER_H
