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
    if (!node.IsLeaf()) {
      // Group node: validate its subgraph recursively.
      ValidateGraph(*node.GetSubgraph(), st);
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
                                             const GCNSubtarget &st) {
  ValidateGraph(graph, st);
}

// ============================================================================
// Schedule / Unschedule
// ============================================================================

void ScheduleLengthTracker::Schedule(const ScheduleNode *node) {
  if (!node->IsLeaf()) {
    report_fatal_error("ScheduleLengthTracker: cannot schedule group node " +
                       Twine(node->GetId()) +
                       ". Group node scheduling is not yet implemented.");
  }

  // Compute ready cycle from already-scheduled predecessors. Only
  // latency-carrying edges constrain readiness (see
  // ScheduleEdge::IsLatencyEdge for the single source of truth).
  int ready_cycle = current_cycle_;
  for (const ScheduleEdge &edge : node->Preds()) {
    if (!edge.IsLatencyEdge()) {
      continue;
    }
    int pred_cycle = GetScheduledCycle(edge.node_);
    int pred_ready = pred_cycle + edge.latency_;
    if (pred_ready > ready_cycle) {
      ready_cycle = pred_ready;
    }
  }

  // Push undo record.
  undo_stack_.push_back({node, current_cycle_, total_bubbles_});

  // Update state.
  int bubbles = ready_cycle - current_cycle_;
  total_bubbles_ += bubbles;
  scheduled_cycle_[node] = ready_cycle;
  current_cycle_ = ready_cycle + 1;
}

void ScheduleLengthTracker::Unschedule() {
  if (undo_stack_.empty()) {
    report_fatal_error("ScheduleLengthTracker: Unschedule without matching "
                       "Schedule");
  }

  ScheduleStep step = undo_stack_.back();
  undo_stack_.pop_back();

  scheduled_cycle_.erase(step.node);
  current_cycle_ = step.prev_cycle;
  total_bubbles_ = step.prev_bubbles;
}

int ScheduleLengthTracker::GetScheduledCycle(
    const ScheduleNode *node) const {
  auto it = scheduled_cycle_.find(node);
  if (it == scheduled_cycle_.end()) {
    report_fatal_error("ScheduleLengthTracker: GetScheduledCycle called "
                       "for unscheduled node " + Twine(node->GetId()));
  }
  return it->second;
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
