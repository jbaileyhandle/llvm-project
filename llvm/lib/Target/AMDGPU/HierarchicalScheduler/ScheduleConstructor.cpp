//===- ScheduleConstructor.cpp - Combined scheduling interface ------------===//
//
// Implementation of the unified scheduling interface.
//
// See ScheduleConstructor.h for design rationale.
//
//===----------------------------------------------------------------------===//

#include "ScheduleConstructor.h"
#include "GCNSubtarget.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

// ============================================================================
// Construction
// ============================================================================

ScheduleConstructor::ScheduleConstructor(const ScheduleGraph &graph,
                                         const GCNSubtarget &st,
                                         const MachineFunction &mf,
                                         const LiveIntervals &lis)
    : graph_(&graph),
      pressure_tracker_(graph, mf, lis),
      length_tracker_(graph, st) {
  // Check for group nodes.
  for (const ScheduleNode &node : graph.Nodes()) {
    if (!node.IsLeaf()) {
      report_fatal_error("ScheduleConstructor does not yet support group "
                         "nodes. Node: " +
                         Twine(node.GetId()));
    }
  }

  InitReadyList();
}

// ============================================================================
// Ready list
// ============================================================================

int ScheduleConstructor::CountStrongPreds(const ScheduleNode *node) {
  int count = 0;
  for (const ScheduleEdge &edge : node->Preds()) {
    if (edge.IsStrongEdge()) {
      count++;
    }
  }
  return count;
}

void ScheduleConstructor::InitReadyList() {
  for (const ScheduleNode &node : graph_->Nodes()) {
    int strong_preds = CountStrongPreds(&node);
    remaining_strong_preds_[&node] = strong_preds;
    if (strong_preds == 0) {
      ready_list_.insert(&node);
    }
  }
}

void ScheduleConstructor::ReleaseSuccessors(const ScheduleNode *node) {
  for (const ScheduleEdge &edge : node->Succs()) {
    if (!edge.IsStrongEdge()) {
      continue;
    }
    const ScheduleNode *succ = edge.node_;
    int &remaining = remaining_strong_preds_[succ];
    remaining--;
    if (remaining == 0) {
      ready_list_.insert(succ);
    }
  }
}

void ScheduleConstructor::UnreleaseSuccessors(const ScheduleNode *node) {
  for (const ScheduleEdge &edge : node->Succs()) {
    if (!edge.IsStrongEdge()) {
      continue;
    }
    const ScheduleNode *succ = edge.node_;
    int &remaining = remaining_strong_preds_[succ];
    if (remaining == 0) {
      ready_list_.erase(succ);
    }
    remaining++;
  }
}

// ============================================================================
// Schedule / Unschedule
// ============================================================================

void ScheduleConstructor::Schedule(const ScheduleNode *node) {
  if (!ready_list_.count(node)) {
    report_fatal_error("ScheduleConstructor: scheduling node " +
                       Twine(node->GetId()) +
                       " which is not in the ready list");
  }

  // Update trackers.
  pressure_tracker_.Schedule(node);
  length_tracker_.Schedule(node);

  // Remove from ready list and add to schedule order.
  ready_list_.erase(node);
  schedule_order_.push_back(node);

  // Release successors.
  ReleaseSuccessors(node);
}

void ScheduleConstructor::GetReadyListSnapshot(
    SmallVectorImpl<const ScheduleNode *> &out) const {
  out.append(ready_list_.begin(), ready_list_.end());
}

void ScheduleConstructor::Unschedule() {
  if (schedule_order_.empty()) {
    report_fatal_error("ScheduleConstructor: Unschedule with empty "
                       "schedule order");
  }

  const ScheduleNode *node = schedule_order_.back();
  schedule_order_.pop_back();

  // Reverse successor release.
  UnreleaseSuccessors(node);

  // Add the node back to the ready list.
  ready_list_.insert(node);

  // Undo trackers (reverse order of Schedule).
  length_tracker_.Unschedule();
  pressure_tracker_.Unschedule(node);
}

// ============================================================================
// Comparison
// ============================================================================

bool ScheduleConstructor::IsBetterThan(const ScheduleConstructor &other,
                                       ScheduleMetric metric) const {
  switch (metric) {
  case ScheduleMetric::kMaximizeRegisterOccupancy:
    return pressure_tracker_.GetRegisterOnlyOccupancy() >
           other.pressure_tracker_.GetRegisterOnlyOccupancy();

  case ScheduleMetric::kMaximizeContinuousRegisterOccupancyScore:
    return pressure_tracker_.GetContinuousOccupancyScore() >
           other.pressure_tracker_.GetContinuousOccupancyScore();

  case ScheduleMetric::kMinimizeScheduleLength:
    return length_tracker_.GetCurrentCycle() <
           other.length_tracker_.GetCurrentCycle();

  case ScheduleMetric::kMinimizeRegisterOccupancy:
    return pressure_tracker_.GetRegisterOnlyOccupancy() <
           other.pressure_tracker_.GetRegisterOnlyOccupancy();

  case ScheduleMetric::kMinimizeContinuousRegisterOccupancyScore:
    return pressure_tracker_.GetContinuousOccupancyScore() <
           other.pressure_tracker_.GetContinuousOccupancyScore();
  }
  llvm_unreachable("Unknown ScheduleMetric");
}

bool ScheduleConstructor::IsAtOrAboveFunctionOccupancyCeiling() const {
  return pressure_tracker_.GetRegisterOnlyOccupancy() >=
         pressure_tracker_.GetConfiguredMachineFunctionOccupancyLimit();
}

// ============================================================================
// Diagnostics
// ============================================================================

std::string ScheduleConstructor::Describe() const {
  std::string result;
  result += "scheduled=" + std::to_string(GetNumScheduled()) +
            "/" + std::to_string(graph_->Size());
  result += " ready=" + std::to_string(ready_list_.size());
  result += " " + length_tracker_.Describe();
  result += " " + pressure_tracker_.DescribePressure();
  return result;
}
