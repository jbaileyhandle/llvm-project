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
#include <algorithm>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

// ============================================================================
// Construction
// ============================================================================

ScheduleConstructor::ScheduleConstructor(const ScheduleGraph &graph,
                                         const GCNSubtarget &st,
                                         const MachineFunction &mf,
                                         const LiveIntervals &lis,
                                         ReadyComparator ready_cmp)
    : graph_(&graph),
      pressure_tracker_(graph, mf, lis),
      length_tracker_(graph, st),
      ready_comparator_(ready_cmp) {
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
// Ready list maintenance (sorted SmallVector under ready_comparator_)
// ============================================================================

int ScheduleConstructor::GetReadyListIndexOf(const ScheduleNode *node) const {
  auto it = std::lower_bound(ready_list_.begin(), ready_list_.end(), node,
                             ready_comparator_);
  if (it == ready_list_.end() || *it != node) {
    return -1;
  }
  return static_cast<int>(it - ready_list_.begin());
}

void ScheduleConstructor::ReadyListInsert(const ScheduleNode *node) {
  auto it = std::lower_bound(ready_list_.begin(), ready_list_.end(), node,
                             ready_comparator_);
  // Duplicate check: since ready_comparator_ is a strict total order,
  // node can only already be present at the position lower_bound
  // returned. Cheap — reuses the same traversal.
  if (it != ready_list_.end() && *it == node) {
    report_fatal_error("ScheduleConstructor: ReadyListInsert on node " +
                       Twine(node->GetId()) +
                       " which is already in the ready list");
  }
  ready_list_.insert(it, node);
}

void ScheduleConstructor::ReadyListEraseAt(int index) {
  ready_list_.erase(ready_list_.begin() + index);
}

void ScheduleConstructor::ReadyListErase(const ScheduleNode *node) {
  int index = GetReadyListIndexOf(node);
  if (index < 0) {
    report_fatal_error("ScheduleConstructor: ReadyListErase on node " +
                       Twine(node->GetId()) +
                       " which is not in the ready list");
  }
  ReadyListEraseAt(index);
}

int ScheduleConstructor::CountStrongPredecessors(const ScheduleNode *node) {
  int count = 0;
  for (const ScheduleEdge &edge : node->Predecessors()) {
    if (edge.IsStrongEdge()) {
      count++;
    }
  }
  return count;
}

void ScheduleConstructor::InitReadyList() {
  remaining_strong_predecessors_by_topo_index_.assign(graph_->Size(), 0);
  for (const ScheduleNode &node : graph_->Nodes()) {
    int strong_predecessors = CountStrongPredecessors(&node);
    remaining_strong_predecessors_by_topo_index_[node.GetTopoIndex()] =
        strong_predecessors;
    if (strong_predecessors == 0) {
      ReadyListInsert(&node);
    }
  }
}

void ScheduleConstructor::ReleaseSuccessors(const ScheduleNode *node) {
  for (const ScheduleEdge &edge : node->Successors()) {
    if (!edge.IsStrongEdge()) {
      continue;
    }
    const ScheduleNode *succ = edge.node_;
    int &remaining =
        remaining_strong_predecessors_by_topo_index_[succ->GetTopoIndex()];
    remaining--;
    if (remaining == 0) {
      ReadyListInsert(succ);
    }
  }
}

void ScheduleConstructor::UnreleaseSuccessors(const ScheduleNode *node) {
  for (const ScheduleEdge &edge : node->Successors()) {
    if (!edge.IsStrongEdge()) {
      continue;
    }
    const ScheduleNode *succ = edge.node_;
    int &remaining =
        remaining_strong_predecessors_by_topo_index_[succ->GetTopoIndex()];
    if (remaining == 0) {
      ReadyListErase(succ);
    }
    remaining++;
  }
}

// ============================================================================
// Schedule / Unschedule
// ============================================================================

void ScheduleConstructor::Schedule(const ScheduleNode *node) {
  int index = GetReadyListIndexOf(node);
  if (index < 0) {
    report_fatal_error("ScheduleConstructor: scheduling node " +
                       Twine(node->GetId()) +
                       " which is not in the ready list");
  }
  ScheduleByIndex(index);
}

void ScheduleConstructor::ScheduleByIndex(int index) {
  if (index < 0 || index >= static_cast<int>(ready_list_.size())) {
    report_fatal_error(
        "ScheduleConstructor: ScheduleByIndex with out-of-range index " +
        Twine(index) + " (ready size " +
        Twine(static_cast<int>(ready_list_.size())) + ")");
  }
  const ScheduleNode *node = ready_list_[index];

  // Update trackers.
  pressure_tracker_.Schedule(node);
  length_tracker_.Schedule(node);

  // Remove from ready list and append to schedule order.
  ReadyListEraseAt(index);
  schedule_order_.push_back(node);

  // Release successors.
  ReleaseSuccessors(node);
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

  // Add the node back to the ready list at its sorted position.
  ReadyListInsert(node);

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
