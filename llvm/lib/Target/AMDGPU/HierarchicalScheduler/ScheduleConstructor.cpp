//===- ScheduleConstructor.cpp - Combined scheduling interface ------------===//
//
// Implementation of the unified scheduling interface.
//
// See ScheduleConstructor.h for design rationale.
//
//===----------------------------------------------------------------------===//

#include "ScheduleConstructor.h"
#include "GCNRegPressure.h"
#include "GCNSubtarget.h"
#include "ScheduleGraph.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
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
                                         std::optional<ScoreRecipe> recipe,
                                         ScheduleConstructorOptions options)
    : graph_(&graph),
      options_(options),
      pressure_tracker_(graph, mf, options.track_pressure_history),
      // Tracker gating rules (see ctor doc for the rationale and
      // caveats):
      //   length tracker enabled iff recipe absent OR recipe
      //   declares kScheduleLength.
      //   ILP tracker enabled iff recipe absent OR recipe is
      //   length-primary.
      //
      // The ILP rule is the load-bearing compromise: length-primary
      // policies' ready-list sort heuristic consults the ILP tracker
      // even when kIlpScore isn't a comparison slot. Tying ILP to
      // IsLengthPrimary() covers that without forcing kIlpScore into
      // recipes that don't compare on it -- at the cost of (a) a
      // wasted ILP tracker for DfsMaximizeLengthPolicy (length-
      // primary, doesn't use ILP for sort) and (b) zero coverage for
      // a hypothetical future occupancy-primary policy that wants
      // ILP in its sort. The current policy set fits this rule;
      // adding a policy that breaks it means revisiting the gating
      // (e.g., a per-policy "extra trackers" hook or a recipe
      // annotation distinguishing comparison dims from sort
      // dependencies).
      length_tracker_(
          (!recipe || recipe->HasDim(ScoreDimension::kScheduleLength))
              ? std::make_optional<ScheduleLengthTracker>(graph, st)
              : std::nullopt),
      ilp_tracker_(
          (!recipe || recipe->IsLengthPrimary())
              ? std::make_optional<IlpTracker>(graph, pressure_tracker_)
              : std::nullopt),
      scheduled_set_tracker_(
          &graph,
          length_tracker_ ? &*length_tracker_ : nullptr) {
  InitReadyList();
}

ScheduleConstructor ScheduleConstructor::NoHistoryClone() const {
  // The inner trackers' history-cleared clones are only defined for
  // GCNRegisterTracker today; length / ILP trackers maintain their
  // own undo stacks but we haven't written NoHistoryClone equivalents
  // for them. Catch the misconfiguration loudly rather than silently
  // producing a clone with stale length/ILP state.
  if (length_tracker_ || ilp_tracker_) {
    report_fatal_error(
        "ScheduleConstructor::NoHistoryClone called on a constructor "
        "whose recipe enables length or ILP tracking. Only the "
        "pressure-only configuration (BFS-DP path) is supported.");
  }
  return ScheduleConstructor(*this, NoHistoryCloneTag{});
}

// See the public NoHistoryClone for the length/ilp precondition;
// this private ctor assumes both options are off and constructs
// only the fields the clone needs.
ScheduleConstructor::ScheduleConstructor(const ScheduleConstructor &source,
                                         NoHistoryCloneTag)
    : graph_(source.graph_),
      options_(source.options_),
      // pressure_tracker_ via its own NoHistoryClone — copies live
      // state without dragging undo_stack_ / pressure_history_. C++17
      // guaranteed copy elision constructs the return value directly
      // in pressure_tracker_'s storage; no extra copy/move.
      pressure_tracker_(source.pressure_tracker_.NoHistoryClone()),
      // length_tracker_ / ilp_tracker_ left as default-constructed
      // empty optionals (precondition: source had both off).
      // scheduled_set_tracker_ copied by value: its DenseMap and
      // BitVector copy themselves; matches what BFS-DP needs
      // (scheduled_set + signature for PartitionKey; frontier is
      // empty in the no-length-tracker case so copy is cheap).
      scheduled_set_tracker_(source.scheduled_set_tracker_),
      // schedule_order_ intentionally not copied — empty by default.
      // schedule_call_count_ intentionally not copied — defaulted.
      scopes_(source.scopes_),
      remaining_strong_predecessors_by_topo_index_(
          source.remaining_strong_predecessors_by_topo_index_) {}

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
// Ready list maintenance (sorted SmallVector by topo_index ascending —
// the natural strict-total-order. Per-Recurse iteration priority is
// the search policy's responsibility; see Policy::FilterAndSortReadyList
// in SearchPolicies.h.)
//
// Helpers take the target ready_list as a parameter so callers can
// route a release into any scope's ready list. Phase 0 always uses
// scopes_.back().ready (= scopes_[0].ready, the base scope), but the
// signatures are already shaped for the scope-stack world.
// ============================================================================

namespace {
// Strict total order used to maintain the ready list. Fixed (not
// policy-tunable) — guarantees O(log N) insert/lookup via lower_bound
// and round-trip preservation of list layout across Schedule/Unschedule.
bool ReadyByTopoIndex(const ScheduleNode *a, const ScheduleNode *b) {
  return a->GetTopoIndex() < b->GetTopoIndex();
}
} // namespace

int ScheduleConstructor::GetReadyListIndexOf(
    const SmallVectorImpl<const ScheduleNode *> &ready_list,
    const ScheduleNode *node) const {
  auto it = std::lower_bound(ready_list.begin(), ready_list.end(), node,
                             ReadyByTopoIndex);
  if (it == ready_list.end() || *it != node) {
    return -1;
  }
  return static_cast<int>(it - ready_list.begin());
}

void ScheduleConstructor::ReadyListInsert(
    SmallVectorImpl<const ScheduleNode *> &ready_list,
    const ScheduleNode *node) {
  auto it = std::lower_bound(ready_list.begin(), ready_list.end(), node,
                             ReadyByTopoIndex);
  // Duplicate check: topo_index is strictly unique per node, so node
  // can only already be present at the position lower_bound returned.
  // Cheap — reuses the same traversal.
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

GCNRegPressure ScheduleConstructor::Schedule(const ScheduleNode *node) {
  // DFS invariant: the picked node lives in the current scope. Look
  // it up in scopes_.back().ready rather than GetReadyListForNode(node),
  // to make that invariant explicit at the call site.
  if (scopes_.empty()) {
    report_fatal_error("ScheduleConstructor::Schedule: no scope on stack");
  }
  int index = GetReadyListIndexOf(scopes_.back().ready, node);
  if (index < 0) {
    report_fatal_error("ScheduleConstructor: scheduling node " +
                       Twine(node->GetId()) +
                       " which is not in the ready list");
  }
  return ScheduleByIndex(index);
}

GCNRegPressure ScheduleConstructor::ScheduleByIndex(int index) {
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
  // We call them uniformly here, gating length / ILP on the
  // options' enable flags (their optionals are empty otherwise).
  // ScheduleSetTracker.Schedule must come AFTER length_tracker_'s
  // — its frontier-LB computation reads the just-scheduled node's
  // cycle from length_tracker_ when length tracking is on (and
  // skips that work entirely when off).
  GCNRegPressure edge_peak = pressure_tracker_.Schedule(node);
  if (length_tracker_) {
    length_tracker_->Schedule(node);
  }
  if (ilp_tracker_) {
    ilp_tracker_->Schedule(node);
  }
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
    if (scopes_.size() <= 1) {
      report_fatal_error("ScheduleConstructor: end proxy node " +
                         Twine(node->GetId()) + " pops the base scope");
    }
    scopes_.pop_back();
  }

  // Append to schedule order. schedule_order_ holds both real
  // nodes and proxies (proxies are filtered at ApplyScheduleOrder
  // time).
  schedule_order_.push_back(node);

  // Release successors into their home scopes (now correctly set
  // up by the scope mutation above).
  ReleaseSuccessors(node);

  return edge_peak;
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
  // length / ILP gated on the same flags as Schedule.
  scheduled_set_tracker_.Unschedule(node);
  if (ilp_tracker_) {
    ilp_tracker_->Unschedule(node);
  }
  if (length_tracker_) {
    length_tracker_->Unschedule(node);
  }
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

Score ScheduleConstructor::GetScore(const ScoreRecipe &recipe) const {
  std::array<std::optional<Score::SlotInput>, kMaxScoreSlots> inputs{};
  for (int i = 0; i < kMaxScoreSlots; ++i) {
    if (recipe.slots[i]) {
      inputs[i] = Score::SlotInput{
          GetScoreDimensionValue(recipe.slots[i]->dim),
          recipe.slots[i]->pol};
    }
  }
  return Score::Make(inputs);
}

int64_t
ScheduleConstructor::GetScoreDimensionValue(ScoreDimension dim) const {
  switch (dim) {
  case ScoreDimension::kRegisterOcc:
    return pressure_tracker_.GetRegisterOnlyOccupancy();
  case ScoreDimension::kContinuousOccScore:
    return pressure_tracker_.GetContinuousOccupancyScore();
  case ScoreDimension::kContinuousOccArea:
    return pressure_tracker_.GetContinuousOccupancyArea();
  case ScoreDimension::kVgprSpillArea:
    return pressure_tracker_.GetVGPRSpillArea();
  case ScoreDimension::kVgprSpillPeak:
    return pressure_tracker_.GetPeakVGPRCountAboveSpillCap();
  case ScoreDimension::kScheduleLength:
    if (!length_tracker_) {
      report_fatal_error(
          "ScheduleConstructor::GetScoreDimensionValue: "
          "kScheduleLength requires length tracking enabled");
    }
    return length_tracker_->GetCurrentCycle();
  case ScoreDimension::kIlpScore:
    if (!ilp_tracker_) {
      report_fatal_error(
          "ScheduleConstructor::GetScoreDimensionValue: kIlpScore "
          "requires ILP tracking enabled");
    }
    return ilp_tracker_->GetIlpScore();
  }
  llvm_unreachable("Unknown ScoreDimension");
}

bool ScheduleConstructor::RegisterOnlyOccupancyIsAtOrAboveFunctionOccupancyTarget() const {
  return pressure_tracker_.GetRegisterOnlyOccupancy() >=
         pressure_tracker_.GetConfiguredMachineFunctionOccupancyTarget();
}

bool ScheduleConstructor::RegisterOnlyOccupancyExceedsFunctionOccupancyTarget() const {
  return pressure_tracker_.GetRegisterOnlyOccupancy() >
         pressure_tracker_.GetConfiguredMachineFunctionOccupancyTarget();
}

bool ScheduleConstructor::LaunchFloorClampedRegisterOnlyOccupancyIsAtOrAboveFunctionOccupancyTarget() const {
  return pressure_tracker_.GetLaunchFloorClampedRegisterOnlyOccupancy() >=
         pressure_tracker_.GetConfiguredMachineFunctionOccupancyTarget();
}

ScheduleConstructor::LlvmTrackerVerification
ScheduleConstructor::VerifyPressureWithLlvmTracker(
    const MachineFunction &mf, const LiveIntervals &lis) const {
  LlvmTrackerVerification result;

  const GCNSubtarget &st = mf.getSubtarget<GCNSubtarget>();
  result.ours_peak = pressure_tracker_.GetPeakPressure();
  result.ours_occupancy =
      pressure_tracker_.GetAllFactorsRegionOnlyOccupancy();

  // Collect MachineInstrs in schedule order, skipping subgraph
  // proxies and entry/exit sentinels (no underlying MI).
  SmallVector<MachineInstr *, 32> mis;
  for (const ScheduleNode *node : GetScheduleOrder()) {
    if (!node->IsSchedulingUnit()) {
      continue;
    }
    SUnit *su = node->GetSUnit();
    if (su && su->getInstr()) {
      mis.push_back(su->getInstr());
    }
  }

  if (mis.empty()) {
    // Nothing to verify — treat as agreement with our tracker.
    result.llvm_peak = result.ours_peak;
    result.llvm_occupancy = result.ours_occupancy;
    result.tracker_confirmed = result.llvm_occupancy >= result.ours_occupancy;
    return result;
  }

  // GCNUpwardRPTracker.recede(MI) accepts arbitrary MIs (not just
  // BB-order), so we walk our schedule order backward.
  //
  // Captured states in forward terms:
  //   - reset(mis[N-1]): live set = "state after mis[N-1]"
  //   - recede(mis[i])  for i = N-1 .. 1: live set = "state after
  //                                       mis[i-1]"
  //   - recede(mis[0]):  live set = "state before mis[0]" = the
  //                      region's live-in pressure
  //
  // Peak is the max over every point in the region's execution,
  // including the live-in state at region entry. Our own forward
  // tracker captures live-ins in its peak too, so we include
  // recede(mis[0]) here for symmetry.
  const MachineRegisterInfo &mri = mf.getRegInfo();
  GCNUpwardRPTracker tracker(lis);
  tracker.reset(*mis.back());

  GCNRegPressure peak = llvm::getRegPressure(mri, tracker.getLiveRegs());

  for (int i = static_cast<int>(mis.size()) - 1; i >= 0; --i) {
    tracker.recede(*mis[i]);
    GCNRegPressure step = llvm::getRegPressure(mri, tracker.getLiveRegs());
    peak = max(peak, step);
  }

  result.llvm_peak = peak;
  result.llvm_occupancy = static_cast<int>(peak.getOccupancy(st));
  result.tracker_confirmed = result.llvm_occupancy >= result.ours_occupancy;
  return result;
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
  if (length_tracker_) {
    result += " " + length_tracker_->Describe();
  }
  result += " " + pressure_tracker_.DescribePressure();
  return result;
}
