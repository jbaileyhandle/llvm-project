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
  InitReadyList();
}

ScheduleConstructor::SubgraphScheduleScope &
ScheduleConstructor::FindScopeOnStack(const ScheduleNode *target_proxy) {
  // Top-down walk: same-scope releases (most successors are
  // intra-scope when DFS is inside a subgraph) hit on iteration 1.
  for (auto it = scopes_.rbegin(); it != scopes_.rend(); ++it) {
    if (it->subgraph_proxy == target_proxy) {
      return *it;
    }
  }
  report_fatal_error(
      "ScheduleConstructor::FindScopeOnStack: no scope on stack "
      "matches target_proxy. The artificial proxy→member edges and "
      "the base scope's subgraph_proxy=nullptr should make this "
      "unreachable.");
}

// ============================================================================
// Ready list maintenance (sorted SmallVector under ready_comparator_)
//
// Helpers take the target ready_list as a parameter so callers can
// route a release into any scope's ready list. Phase 0 always uses
// scopes_.back().ready (= scopes_[0].ready, the base scope), but the
// signatures are already shaped for the scope-stack world.
// ============================================================================

int ScheduleConstructor::GetReadyListIndexOf(
    const SmallVectorImpl<const ScheduleNode *> &ready_list,
    const ScheduleNode *node) const {
  auto it = std::lower_bound(ready_list.begin(), ready_list.end(), node,
                             ready_comparator_);
  if (it == ready_list.end() || *it != node) {
    return -1;
  }
  return static_cast<int>(it - ready_list.begin());
}

void ScheduleConstructor::ReadyListInsert(
    SmallVectorImpl<const ScheduleNode *> &ready_list,
    const ScheduleNode *node) {
  auto it = std::lower_bound(ready_list.begin(), ready_list.end(), node,
                             ready_comparator_);
  // Duplicate check: since ready_comparator_ is a strict total order,
  // node can only already be present at the position lower_bound
  // returned. Cheap — reuses the same traversal.
  if (it != ready_list.end() && *it == node) {
    report_fatal_error("ScheduleConstructor: ReadyListInsert on node " +
                       Twine(node->GetId()) +
                       " which is already in the ready list");
  }
  ready_list.insert(it, node);
}

void ScheduleConstructor::ReadyListEraseAt(
    SmallVectorImpl<const ScheduleNode *> &ready_list, int index) {
  ready_list.erase(ready_list.begin() + index);
}

void ScheduleConstructor::ReadyListErase(
    SmallVectorImpl<const ScheduleNode *> &ready_list,
    const ScheduleNode *node) {
  int index = GetReadyListIndexOf(ready_list, node);
  if (index < 0) {
    report_fatal_error("ScheduleConstructor: ReadyListErase on node " +
                       Twine(node->GetId()) +
                       " which is not in the ready list");
  }
  ReadyListEraseAt(ready_list, index);
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
  // Seed scopes_ with the base scope. Phase 0 never pushes past this.
  scopes_.push_back({/*subgraph_proxy=*/nullptr, {}});
  auto &base_ready = scopes_.back().ready;

  remaining_strong_predecessors_by_topo_index_.assign(graph_->Size(), 0);
  for (const ScheduleNode &node : graph_->Nodes()) {
    int strong_predecessors = CountStrongPredecessors(&node);
    remaining_strong_predecessors_by_topo_index_[node.GetTopoIndex()] =
        strong_predecessors;
    if (strong_predecessors == 0) {
      ReadyListInsert(base_ready, &node);
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
      ReadyListInsert(GetReadyListForNode(succ), succ);
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
      ReadyListErase(GetReadyListForNode(succ), succ);
    }
    remaining++;
  }
}

// ============================================================================
// Schedule / Unschedule
// ============================================================================

void ScheduleConstructor::Schedule(const ScheduleNode *node) {
  // DFS invariant: the picked node lives in the current scope. Look
  // it up in scopes_.back().ready rather than GetReadyListForNode(node),
  // to make that invariant explicit at the call site.
  int index = GetReadyListIndexOf(scopes_.back().ready, node);
  if (index < 0) {
    report_fatal_error("ScheduleConstructor: scheduling node " +
                       Twine(node->GetId()) +
                       " which is not in the ready list");
  }
  ScheduleByIndex(index);
}

void ScheduleConstructor::ScheduleByIndex(int index) {
  auto &ready_list = scopes_.back().ready;
  if (index < 0 || index >= static_cast<int>(ready_list.size())) {
    report_fatal_error(
        "ScheduleConstructor: ScheduleByIndex with out-of-range index " +
        Twine(index) + " (ready size " +
        Twine(static_cast<int>(ready_list.size())) + ")");
  }
  const ScheduleNode *node = ready_list[index];

  ++schedule_call_count_;

  // Update Trackers
  // Trackers self-skip for subgraph proxies (no register or cycle
  // effect — see each tracker's Schedule for the early-return).
  // We call them uniformly here.
  pressure_tracker_.Schedule(node);
  length_tracker_.Schedule(node);

  // Subgraph-proxy nodes push a new scope so members released
  // below land somewhere (FindScopeOnStack lookup on each
  // member's parent_subgraph_proxy will return this new scope).
  // ready_list above (captured by reference) refers to the
  // pre-push top-of-stack and stays valid for the EraseAt below.
  if (node->IsSubgraphProxy()) {
    scopes_.push_back(
        SubgraphScheduleScope{/*subgraph_proxy=*/node, /*ready=*/{}});
  }

  // Remove from the current (pre-push) scope's ready list and
  // append to schedule order. schedule_order_ holds both real
  // nodes and proxies (proxies are filtered at ApplyScheduleOrder
  // time).
  ReadyListEraseAt(ready_list, index);
  schedule_order_.push_back(node);

  // Release successors
  ReleaseSuccessors(node);
}

void ScheduleConstructor::Unschedule() {
  if (schedule_order_.empty()) {
    report_fatal_error("ScheduleConstructor: Unschedule with empty "
                       "schedule order");
  }

  const ScheduleNode *node = schedule_order_.back();
  schedule_order_.pop_back();

  // Reverse successor release. For a proxy: increments member
  // pred_counts back above 0 and removes those members from the
  // pushed scope. After this, the pushed scope's ready list
  // should be empty.
  UnreleaseSuccessors(node);

  // Pop the scope that this proxy pushed in Schedule (mirror of
  // the Schedule push). Asserts the scope was correctly drained
  // by un-release.
  if (node->IsSubgraphProxy()) {
    if (!scopes_.back().ready.empty()) {
      report_fatal_error(
          "ScheduleConstructor::Unschedule(proxy): pushed scope's "
          "ready list is non-empty after un-release; round-trip "
          "invariant violated");
    }
    scopes_.pop_back();
  }

  // Re-insert the node back into its home scope's ready list at
  // its sorted position. Routes via GetReadyListForNode rather
  // than scopes_.back().ready because the home scope may not be
  // the current top (e.g. for a member node un-released while
  // we're unwinding back through several scope levels).
  ReadyListInsert(GetReadyListForNode(node), node);

  // Undo trackers (reverse order of Schedule). Trackers self-skip
  // for subgraph proxies — see each tracker's Unschedule.
  length_tracker_.Unschedule(node);
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
  const auto &current_scope = scopes_.back();
  result += " scope=";
  if (current_scope.subgraph_proxy == nullptr) {
    result += "base";
  } else {
    result += "subgraph[" +
              std::to_string(current_scope.subgraph_proxy->GetTopoIndex()) +
              "]";
  }
  result += " ready=" + std::to_string(current_scope.ready.size());
  result += " " + length_tracker_.Describe();
  result += " " + pressure_tracker_.DescribePressure();
  return result;
}
