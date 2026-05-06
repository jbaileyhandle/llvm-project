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
      // scheduled_set_tracker_ depends on length_tracker_ for cycle
      // lookups; declared after it in the class so member init order
      // is correct.
      scheduled_set_tracker_(&graph, &length_tracker_),
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

  schedule_call_count_.Increment();

  // Trackers self-skip for subgraph proxies (no register or cycle
  // effect — see each tracker's Schedule for the early-return).
  // We call them uniformly here.
  // ScheduleSetTracker.Schedule must come AFTER length_tracker_'s
  // — its frontier-LB computation reads the just-scheduled node's
  // cycle from length_tracker_.
  pressure_tracker_.Schedule(node);
  length_tracker_.Schedule(node);
  scheduled_set_tracker_.Schedule(node);

  // Erase from current scope's ready list FIRST. ready_list must
  // not be touched after this point — the scope mutation below can
  // realloc scopes_ (push) or destroy the referenced scope (pop)
  // and dangle the reference either way.
  ReadyListEraseAt(ready_list, index);

  // Scope mutation. Start proxy pushes a new scope so members
  // released below land somewhere (each member's
  // parent_subgraph_proxy is this start, so GetReadyListForNode
  // routes them into the just-pushed scope). End proxy pops the
  // now-drained subgraph scope so ext_successors released below
  // land in the parent scope.
  if (node->IsSubgraphStartProxy()) {
    scopes_.push_back(
        SubgraphScheduleScope{/*subgraph_proxy=*/node, /*ready=*/{}});
  } else if (node->IsSubgraphEndProxy()) {
    scopes_.pop_back();
  }

  // Append to schedule order. schedule_order_ holds both real
  // nodes and proxies (proxies are filtered at ApplyScheduleOrder
  // time).
  schedule_order_.push_back(node);

  // Release successors into their home scopes (now correctly set
  // up by the scope mutation above).
  ReleaseSuccessors(node);
}

void ScheduleConstructor::Unschedule() {
  if (schedule_order_.empty()) {
    report_fatal_error("ScheduleConstructor: Unschedule with empty "
                       "schedule order");
  }

  const ScheduleNode *node = schedule_order_.back();
  schedule_order_.pop_back();

  // Inverse of ReleaseSuccessors. For each strong successor of
  // node whose pred_count reached 0 during the matching Schedule
  // (i.e., that successor is currently in some scope's ready
  // list), erase it from that ready list and bump the count back
  // above 0.
  UnreleaseSuccessors(node);

  // Mirror of ScheduleByIndex's scope mutation. Inverse op, in
  // reverse:
  //   Start proxy: Schedule pushed a new scope; Unschedule pops
  //                it. The pushed scope must be empty after
  //                un-release (any member still in it would
  //                indicate a round-trip bug — members are
  //                un-released in their own LIFO Unschedule
  //                calls, which come BEFORE the start proxy's).
  //   End proxy:   Schedule popped the subgraph scope; Unschedule
  //                pushes it back, empty. The end proxy is
  //                un-scheduled BEFORE any member (members were
  //                scheduled before end_proxy, so are un-scheduled
  //                after), so the just-pushed scope is correctly
  //                empty at this moment — members will reinsert
  //                themselves into it during their own subsequent
  //                Unschedule calls.
  if (node->IsSubgraphStartProxy()) {
    if (!scopes_.back().ready.empty()) {
      report_fatal_error(
          "ScheduleConstructor::Unschedule(start proxy): pushed "
          "scope's ready list is non-empty after un-release; "
          "round-trip invariant violated");
    }
    scopes_.pop_back();
  } else if (node->IsSubgraphEndProxy()) {
    // Push back the subgraph scope this end proxy popped at
    // Schedule time. The end proxy lives in this scope (its
    // parent_subgraph_proxy IS the start proxy that defines the
    // scope), so GetReadyListForNode(end_proxy) below routes it
    // into the just-pushed scope.
    scopes_.push_back(
        SubgraphScheduleScope{
            /*subgraph_proxy=*/node->GetParentSubgraphProxy(),
            /*ready=*/{}});
  }

  // Re-insert the node back into its home scope's ready list at
  // its sorted position. Routes via GetReadyListForNode rather
  // than scopes_.back().ready because the home scope may not be
  // the current top (e.g. for a member node un-released while
  // we're unwinding back through several scope levels).
  ReadyListInsert(GetReadyListForNode(node), node);

  // Undo trackers (reverse order of Schedule). Trackers self-skip
  // for subgraph proxies — see each tracker's Unschedule.
  scheduled_set_tracker_.Unschedule(node);
  length_tracker_.Unschedule(node);
  pressure_tracker_.Unschedule(node);
}

void ScheduleConstructor::UnscheduleAll() {
  // Repeated Unschedule rolls back trackers and scope state through
  // their normal undo path — same per-step bookkeeping the search
  // would do during a normal backtrack — so a search-iteration
  // teardown that ends in this state is indistinguishable from a
  // search that backtracked all the way to root.
  while (!schedule_order_.empty()) {
    Unschedule();
  }
}

void ScheduleConstructor::Reset() {
  UnscheduleAll();
  schedule_call_count_.ResetCurrentRun();
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
