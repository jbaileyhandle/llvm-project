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
      scheduled_cycle_by_topo_index_(graph.Size(), /*sentinel=*/-1) {
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
  if (!node->IsSchedulingUnit()) {
    report_fatal_error(
        "ScheduleLengthTracker: cannot schedule subgraph proxy " +
        Twine(node->GetId()) +
        ". Subgraph-proxy scheduling is not yet implemented.");
  }

  int ready_cycle = ComputeReadyCycle(node);
  PushUndoStep(node);
  AdvanceSchedule(node, ready_cycle);
  UpdateLengthLowerBoundMax(node, ready_cycle);
}

void ScheduleLengthTracker::Unschedule() {
  if (undo_stack_.empty()) {
    report_fatal_error("ScheduleLengthTracker: Unschedule without matching "
                       "Schedule");
  }

  ScheduleStep step = undo_stack_.back();
  undo_stack_.pop_back();

  scheduled_cycle_by_topo_index_[step.node->GetTopoIndex()] = -1;
  current_cycle_ = step.prev_cycle;
  total_bubbles_ = step.prev_bubbles;
  max_scheduled_plus_cp_ = step.prev_max_scheduled_plus_cp;
}

int ScheduleLengthTracker::ComputeReadyCycle(
    const ScheduleNode *node) const {
  // Only latency-carrying edges constrain readiness (see
  // ScheduleEdge::IsLatencyEdge for the single source of truth).
  int ready_cycle = current_cycle_;
  for (const ScheduleEdge &edge : node->Predecessors()) {
    if (!edge.IsLatencyEdge()) {
      continue;
    }
    int pred_cycle = GetScheduledCycle(edge.node_);
    int pred_ready = pred_cycle + edge.latency_;
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
