//===- ScheduleLengthTracker.cpp - Schedule length tracking ----------------===//
//
// Implementation of schedule length tracking for IssueWidth = 1.
//
// See ScheduleLengthTracker.h for design rationale.
//
//===----------------------------------------------------------------------===//

#include "ScheduleLengthTracker.h"
#include "GCNSubtarget.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/MC/MCSchedule.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

// ============================================================================
// Validation
// ============================================================================

void ScheduleLengthTracker::ValidateGraph(const ScheduleGraph &graph,
                                          const GCNSubtarget &st) {
  // Cache of graph IDs already validated. Static local — initialized
  // once, scoped to this function. Uses the graph's unique int64_t ID
  // (monotonically increasing, never reused) rather than pointers.
  static DenseSet<int64_t> validated_ids;

  if (validated_ids.count(graph.GetId())) {
    return;
  }

  // Check IssueWidth.
  unsigned issue_width = st.getSchedModel().IssueWidth;
  if (issue_width != 1) {
    report_fatal_error("ScheduleLengthTracker requires IssueWidth == 1, "
                       "but the scheduling model has IssueWidth = " +
                       Twine(issue_width));
  }

  // Check each node.
  for (const ScheduleNode &node : graph.Nodes()) {
    if (!node.IsSchedulingUnit()) {
      // Subgraph proxies carry no SUnit — nothing to check here.
      // Phase 1 will revisit whether any proxy-level invariants
      // belong in this validator; for now, skip.
      continue;
    }

    SUnit *su = node.GetSUnit();
    if (!su) {
      // Entry/exit/test nodes — no SUnit, nothing to check.
      continue;
    }

    if (su->hasReservedResource) {
      report_fatal_error(
          "ScheduleLengthTracker does not model resource contention, "
          "but node " +
          Twine(node.GetId()) +
          " uses a reserved (unbuffered) resource (e.g., HWXDL/MFMA)");
    }
  }

  validated_ids.insert(graph.GetId());
}

// ============================================================================
// Construction
// ============================================================================

ScheduleLengthTracker::ScheduleLengthTracker(const ScheduleGraph &graph,
                                             const GCNSubtarget &st)
    : graph_(&graph),
      scheduled_cycle_by_topo_index_(graph.Size(), /*sentinel=*/-1),
      unscheduled_max_cycle_heap_(*this) {
  ValidateGraph(graph, st);
  if (!graph.HasCriticalPathFromExit()) {
    report_fatal_error(
        "ScheduleLengthTracker requires ScheduleGraph::"
        "ComputeCriticalPathFromExit to have been called before "
        "construction (see class-level precondition).");
  }
}

// ============================================================================
// Schedule / Unschedule
// ============================================================================

void ScheduleLengthTracker::Schedule(const ScheduleNode *node) {
  // Subgraph proxies are synthetic and consume no cycles. Early
  // return so ScheduleConstructor can call trackers uniformly
  // without branching on node kind. Symmetric with Unschedule.
  if (!node->IsSchedulingUnit()) {
    return;
  }

  int ready_cycle = ComputeReadyCycle(node);
  PushUndoStep(node);
  AdvanceSchedule(node, ready_cycle);
  UpdateLengthLowerBoundMax(node, ready_cycle);
  unscheduled_max_cycle_heap_.Remove(node);
}

void ScheduleLengthTracker::Unschedule(const ScheduleNode *node) {
  // Symmetric with Schedule: proxies were no-ops, so undo is also
  // a no-op.
  if (!node->IsSchedulingUnit()) {
    return;
  }

  if (undo_stack_.empty()) {
    report_fatal_error("ScheduleLengthTracker: Unschedule without matching "
                       "Schedule");
  }

  ScheduleStep step = undo_stack_.back();
  undo_stack_.pop_back();

  // The undo stack records which node was just scheduled; the
  // caller-supplied node should match. A mismatch indicates the
  // tracker's mutators are being driven out of order — undoing
  // the wrong instruction would silently corrupt state.
  if (step.node != node) {
    report_fatal_error(
        "ScheduleLengthTracker::Unschedule called with node " +
        Twine(node->GetId()) +
        " but the most recent Schedule was for node " +
        Twine(step.node->GetId()));
  }

  scheduled_cycle_by_topo_index_[node->GetTopoIndex()] = -1;
  current_cycle_ = step.prev_cycle;
  total_bubbles_ = step.prev_bubbles;
  max_scheduled_plus_cp_ = step.prev_max_scheduled_plus_cp;
  unscheduled_max_cycle_heap_.Insert(node);
}

int ScheduleLengthTracker::ComputeReadyCycle(
    const ScheduleNode *node) const {
  // ready_cycle is the max over:
  //   (a) current_cycle_ — IssueWidth=1 global floor: no two
  //       instructions co-issue, so `node` (a scheduling unit)
  //       must come at least one cycle after the most-recently-
  //       scheduled instruction even when that instruction isn't
  //       a predecessor.
  //   (b) per scheduled predecessor P:
  //         P.cycle + max(edge.Latency(), P.IssueSlotsConsumed())
  //       The inner max combines the data-latency floor (from the
  //       edge) with the IssueWidth=1 issue-slot floor (P consumes
  //       a slot if it's a real instruction — proxies don't).
  //
  // The IsLatencyEdge filter excludes kSubgraphOrderEdge, which
  // is the only non-latency strong edge kind in the current
  // encoding. Both endpoints of the surviving edges are scheduling
  // units (verified by the proxy → proxy assert in
  // ScheduleGraph::AddEdge), so P.IssueSlotsConsumed() == 1 here;
  // the inner max simplifies to max(latency_, 1).
  int ready_cycle = current_cycle_;
  for (const ScheduleEdge &edge : node->Predecessors()) {
    if (!edge.IsLatencyEdge()) {
      continue;
    }
    int pred_cycle = GetScheduledCycle(edge.node_);
    int pred_ready = pred_cycle +
                     std::max(edge.Latency(),
                              edge.node_->IssueSlotsConsumed());
    if (pred_ready > ready_cycle) {
      ready_cycle = pred_ready;
    }
  }
  return ready_cycle;
}

void ScheduleLengthTracker::PushUndoStep(const ScheduleNode *node) {
  undo_stack_.push_back({node, current_cycle_, total_bubbles_,
                         max_scheduled_plus_cp_});
}

void ScheduleLengthTracker::AdvanceSchedule(const ScheduleNode *node,
                                            int ready_cycle) {
  int bubbles = ready_cycle - current_cycle_;
  total_bubbles_ += bubbles;
  scheduled_cycle_by_topo_index_[node->GetTopoIndex()] = ready_cycle;
  current_cycle_ = ready_cycle + 1;
}

void ScheduleLengthTracker::UpdateLengthLowerBoundMax(
    const ScheduleNode *node, int ready_cycle) {
  // The +1 translates "latency-sink's cycle" into "schedule length":
  // cp_from_exit[node] bounds the sink's cycle from node, and length
  // is one past the last issued cycle. See the formula in the
  // class-level comment.
  int contribution =
      ready_cycle + graph_->GetCriticalPathFromExit(node) + 1;
  if (contribution > max_scheduled_plus_cp_) {
    max_scheduled_plus_cp_ = contribution;
  }
}

int ScheduleLengthTracker::GetLengthLowerBound() const {
  // Constructor enforces cp_from_exit availability, so no per-call
  // check here. NumSchedulingUnits counts every scheduling-unit node
  // (including entry/exit sentinels, which the tracker treats as
  // consuming one cycle each); subgraph proxies are excluded, since
  // they are synthetic and don't advance the cycle counter.
  int num_unscheduled = graph_->NumSchedulingUnits() - GetNumScheduled();
  return std::max(current_cycle_ + num_unscheduled, max_scheduled_plus_cp_);
}

int ScheduleLengthTracker::GetScheduledCycle(
    const ScheduleNode *node) const {
  int cycle = scheduled_cycle_by_topo_index_[node->GetTopoIndex()];
  if (cycle < 0) {
    report_fatal_error("ScheduleLengthTracker: GetScheduledCycle called "
                       "for unscheduled node " + Twine(node->GetId()));
  }
  return cycle;
}

void ScheduleLengthTracker::SetMaxAcceptableScheduleLength(int max_acceptable_schedule_length) {
  // A schedule of max_acceptable_schedule_length cycles uses cycles
  // 0..max_acceptable_schedule_length-1. For node i with cp_from_exit[i],
  // the latency-weighted chain from i to the exit takes
  // cp_from_exit[i] cycles, so if i is placed at cycle c, the
  // chain ends at cycle c + cp_from_exit[i]. That last cycle
  // must fit in the schedule:
  //   c + cp_from_exit[i] <= max_acceptable_schedule_length - 1
  // Solving for c:
  //   c <= max_acceptable_schedule_length - 1 - cp_from_exit[i]
  // which is the max schedule cycle stored below.
  if (!graph_->HasCriticalPathFromExit()) {
    report_fatal_error(
        "ScheduleLengthTracker::SetMaxAcceptableScheduleLength called when graph "
        "cp_from_exit has been invalidated");
  }

  // Update overall max length
  max_acceptable_schedule_length_ = max_acceptable_schedule_length;

  // Update max_schedule_cycle per node
  const int n = graph_->Size();
  max_schedule_cycle_by_topo_index_.resize(n);
  for (int topo_idx = 0; topo_idx < n; ++topo_idx) {
    int cp_from_exit =
        graph_->GetCriticalPathFromExitByTopoIndex(topo_idx);
    max_schedule_cycle_by_topo_index_[topo_idx] =
        max_acceptable_schedule_length - 1 - cp_from_exit;
  }

  // The cycle values just changed, so any prior heap entries are
  // keyed at stale max_cycle values. Rebuild from scratch from
  // the new array.
  unscheduled_max_cycle_heap_.Rebuild();
}

// ============================================================================
// Diagnostics
// ============================================================================

std::string ScheduleLengthTracker::Describe() const {
  std::string result;
  result += "length=" + std::to_string(current_cycle_) +
            " bubbles=" + std::to_string(total_bubbles_) +
            " scheduled=" + std::to_string(GetNumScheduled());
  return result;
}
