//===- SearchPolicies.cpp - Per-pass search policies ---------------------===//

#include "SearchPolicies.h"
#include "ScheduleGraph.h"
#include "SubgraphInfo.h"
#include <algorithm>
#include <climits>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// Effective min_schedule_cycle for ranking. Scheduling-unit node:
// its own min from the length tracker. Subgraph proxy: min over
// the proxy's subgraph members' effective mins — treats the proxy
// as a composite. Recursive on members so a member that is itself
// a proxy (nested subgraphs, currently unsupported but future-
// proofed) is handled the same way. Returns 0 if the proxy has no
// members (degenerate; defensive). Note: not called for end
// proxies (those short-circuit in FilterAndSortReadyList).
int EffectiveMinScheduleCycle(
    const ScheduleNode *node,
    const ScheduleLengthTracker &length_tracker) {
  if (node->IsSchedulingUnit()) {
    return length_tracker.GetMinScheduleCycleByTopoIndex(
        node->GetTopoIndex());
  }
  SubgraphInfo *info = node->GetSubgraphInfo();
  int result = INT_MAX;
  for (ScheduleNode *member : info->members) {
    result = std::min(result,
                      EffectiveMinScheduleCycle(member, length_tracker));
  }
  return (result == INT_MAX) ? 0 : result;
}

// Effective max_schedule_cycle for ranking. Scheduling-unit node:
// its own max. Subgraph proxy: min (most deadline-pressured) over
// members' effective maxes. Returns max_acceptable - 1 if the
// proxy has no members.
int EffectiveMaxScheduleCycle(
    const ScheduleNode *node,
    const ScheduleLengthTracker &length_tracker) {
  if (node->IsSchedulingUnit()) {
    return length_tracker.GetMaxScheduleCycleByTopoIndex(
        node->GetTopoIndex());
  }
  SubgraphInfo *info = node->GetSubgraphInfo();
  int result = INT_MAX;
  for (ScheduleNode *member : info->members) {
    result = std::min(result,
                      EffectiveMaxScheduleCycle(member, length_tracker));
  }
  if (result == INT_MAX) {
    return length_tracker.GetMaxAcceptableScheduleLength() - 1;
  }
  return result;
}

// Effective node-input-order for ranking — corresponds to LLVM's
// SUnit::NodeNum, which is the order LLVM's pre-RA scheduler emits
// instructions in. That order is register-pressure-aware (LLVM's
// pre-RA scheduler considers RP), so using it as a tiebreaker
// after deadline pressure inherits LLVM's RP-awareness implicitly
// — same trick OptSched uses with its NID heuristic.
//
// Scheduling-unit node: its own SUnit::NodeNum. Subgraph proxy:
// min over members' effective node-nums (matches the other
// Effective* helpers — proxy is treated as the most "input-order
// urgent" of its members). Returns 0 for proxies with no members.
int EffectiveNodeNum(const ScheduleNode *node) {
  if (node->IsSchedulingUnit()) {
    const SUnit *su = node->GetSUnit();
    return su != nullptr ? static_cast<int>(su->NodeNum) : 0;
  }
  SubgraphInfo *info = node->GetSubgraphInfo();
  int result = INT_MAX;
  for (ScheduleNode *member : info->members) {
    result = std::min(result, EffectiveNodeNum(member));
  }
  return (result == INT_MAX) ? 0 : result;
}

// Effective net (defs - kills) for ranking. Scheduling-unit node:
// pressure_tracker's GetNetDefMinusLastUse. Subgraph proxy:
// neutral (0) for now — proper subgraph aggregate (defs going out
// minus uses freed) is non-trivial and deferred. On regions
// without subgraphs (formation finds 0), this doesn't matter.
int EffectiveNetDefMinusLastUse(
    const ScheduleNode *node,
    const GCNRegisterTracker &pressure_tracker) {
  if (node->IsSchedulingUnit()) {
    return pressure_tracker.GetNetDefMinusLastUse(node);
  }
  return 0;
}

} // namespace

void DfsMinimizeLengthPolicy::FilterAndSortReadyList(
    const ScheduleConstructor &working,
    SmallVectorImpl<const ScheduleNode *> &out) {
  out.clear();
  const ScheduleLengthTracker &length_tracker =
      working.GetLengthTracker();
  ArrayRef<const ScheduleNode *> ready = working.GetReadyList();

  auto by_topo = [](const ScheduleNode *a, const ScheduleNode *b) {
    return a->GetTopoIndex() < b->GetTopoIndex();
  };

  if (!length_tracker.HasMaxAcceptableScheduleLength()) {
    out.assign(ready.begin(), ready.end());
    std::sort(out.begin(), out.end(), by_topo);
    return;
  }

  // End-proxy short-circuit. An end proxy enters the ready list
  // only when all members of its subgraph are scheduled — at that
  // moment it is the sole entry in the subgraph scope's ready
  // list. Skip the per-node aggregation work entirely, return only
  // the end proxy. Asserts the alone-ness invariant: a violation
  // would mean an end proxy became ready while a sibling was still
  // waiting, which would indicate a scope/release-tracking bug
  // upstream.
  for (const ScheduleNode *node : ready) {
    if (node->IsSubgraphEndProxy()) {
      if (ready.size() != 1) {
        report_fatal_error(
            "DfsMinimizeLengthPolicy::FilterAndSortReadyList: end "
            "proxy node " +
            Twine(node->GetId()) + " is in a ready list of size " +
            Twine(static_cast<int>(ready.size())) +
            "; expected to be the sole entry");
      }
      out.push_back(node);
      return;
    }
  }

  int current_cycle = length_tracker.GetCurrentCycle();

  // Level 1: no-bubble candidates (effective min <= current_cycle),
  // sorted by effective max ascending (most deadline-pressured
  // first), then NID ascending (LLVM input order — pressure-aware
  // tiebreak from LLVM's pre-RA scheduler), then topo ascending.
  for (const ScheduleNode *node : ready) {
    if (EffectiveMinScheduleCycle(node, length_tracker) <= current_cycle) {
      out.push_back(node);
    }
  }
  if (!out.empty()) {
    const GCNRegisterTracker &pressure_tracker =
        working.GetPressureTracker();
    std::sort(
        out.begin(), out.end(),
        [&length_tracker, &pressure_tracker](const ScheduleNode *a,
                                              const ScheduleNode *b) {
          int a_max = EffectiveMaxScheduleCycle(a, length_tracker);
          int b_max = EffectiveMaxScheduleCycle(b, length_tracker);
          if (a_max != b_max) {
            return a_max < b_max;
          }
          // Pressure-aware secondary: among same-deadline candidates,
          // pick the one whose schedule-now would relieve more
          // pressure (more negative net = more relief). NID below
          // is unique per SUnit so this is the load-bearing
          // tiebreak; NID is then a deterministic backup.
          int a_net =
              EffectiveNetDefMinusLastUse(a, pressure_tracker);
          int b_net =
              EffectiveNetDefMinusLastUse(b, pressure_tracker);
          if (a_net != b_net) {
            return a_net < b_net;
          }
          int a_nid = EffectiveNodeNum(a);
          int b_nid = EffectiveNodeNum(b);
          if (a_nid != b_nid) {
            return a_nid < b_nid;
          }
          return a->GetTopoIndex() < b->GetTopoIndex();
        });
    return;
  }

  // Level 2: all candidates would bubble. Sort by effective min
  // ascending (smallest forced bubble), then effective max
  // ascending (deadline pressure within same-min), then net
  // def-kill ascending (pressure relief among same-deadline),
  // then NID, then topo.
  const GCNRegisterTracker &pressure_tracker =
      working.GetPressureTracker();
  out.assign(ready.begin(), ready.end());
  std::sort(
      out.begin(), out.end(),
      [&length_tracker, &pressure_tracker](const ScheduleNode *a,
                                            const ScheduleNode *b) {
        int a_min = EffectiveMinScheduleCycle(a, length_tracker);
        int b_min = EffectiveMinScheduleCycle(b, length_tracker);
        if (a_min != b_min) {
          return a_min < b_min;
        }
        int a_max = EffectiveMaxScheduleCycle(a, length_tracker);
        int b_max = EffectiveMaxScheduleCycle(b, length_tracker);
        if (a_max != b_max) {
          return a_max < b_max;
        }
        int a_net = EffectiveNetDefMinusLastUse(a, pressure_tracker);
        int b_net = EffectiveNetDefMinusLastUse(b, pressure_tracker);
        if (a_net != b_net) {
          return a_net < b_net;
        }
        int a_nid = EffectiveNodeNum(a);
        int b_nid = EffectiveNodeNum(b);
        if (a_nid != b_nid) {
          return a_nid < b_nid;
        }
        return a->GetTopoIndex() < b->GetTopoIndex();
      });
}

bool DfsMinimizeLengthPolicy::ShouldBoundSearch(
    const ScheduleConstructor &schedule_constructor,
    const ScheduleConstructor & /*best_schedule_constructor*/,
    LengthHistoryTracker &length_history,
    PressureHistoryTracker & /*pressure_history*/) {
  // Max acceptable schedule length is read straight from working's
  // length tracker, which DfsSearch keeps in sync with
  // min(requested_target_length, best.length - 1) at every event
  // that changes either input. The aggregate-LB check below is the
  // coarse-grained complement to the per-node deadline check
  // implied by the tracker's GetMaxScheduleCycle table.
  const auto &length_tracker = schedule_constructor.GetLengthTracker();

  int max_acceptable = length_tracker.GetMaxAcceptableScheduleLength();
  int working_lb = length_tracker.GetLengthLowerBound();
  if (working_lb > max_acceptable) {
    return true;
  }

  // Per-instruction max-schedule-cycle prune: tighter than the
  // aggregate LB above when the smallest unscheduled
  // max_schedule_cycle has been overrun, even though the LB
  // itself hasn't yet exceeded max_acceptable.
  if (length_tracker.IsCurrentCycleBeyondEarliestMaxScheduleCycle()) {
    return true;
  }

  // Min-vs-max prune: complementary to the check above. Each
  // catches cases the other can miss.
  //   Heap check (above) fires when current_cycle has crept
  //     past some unscheduled node's max — the node has been
  //     stranded by general advancement, not by a specific
  //     predecessor's contribution.
  //   This check fires when the most recent Schedule's forward
  //     propagation just pushed some node's min over its max
  //     via a long-latency path, even though current_cycle
  //     hasn't yet caught up.
  // Either way, the named node can no longer be placed in time,
  // so no completion of the working schedule can honor the
  // configured max acceptable schedule length.
  if (length_tracker.IsAnyMinScheduleCycleBeyondMaxScheduleCycle()) {
    return true;
  }

  if (!schedule_constructor.RegisterOnlyOccupancyIsAtOrAboveFunctionOccupancyTarget()) {
    return true;
  }

  if constexpr (kUseLengthHistoryPruning) {
    // Mutating: records the current prefix in length_history when
    // it is NOT dominated. See LengthHistoryTracker class comment.
    if (length_history.IsDominatedElseInsert()) {
      return true;
    }
  }
  return false;
}

bool DfsMinimizeLengthPolicy::ShouldEndSearch(
    const ScheduleConstructor & /*schedule_constructor*/,
    const ScheduleConstructor &best_schedule_constructor) {
  int best_length =
      best_schedule_constructor.GetLengthTracker().GetCurrentCycle();
  int floor = best_schedule_constructor.GetGraph().GetGraphLengthFloor();
  return best_length <= floor;
}

bool DfsMinimizeLengthRefineOccupancyPolicy::ShouldEndSearch(
    const ScheduleConstructor & /*schedule_constructor*/,
    const ScheduleConstructor &best_schedule_constructor) {
  int best_length =
      best_schedule_constructor.GetLengthTracker().GetCurrentCycle();
  int floor = best_schedule_constructor.GetGraph().GetGraphLengthFloor();
  if (best_length > floor) {
    return false;
  }
  // At length floor. End only if integer occupancy strictly
  // exceeds the function target — within-bracket continuous-
  // score refinement can't unlock anything more once we're
  // already above the target. When occupancy equals the target,
  // continue searching same-length completions for higher
  // continuous score (more headroom within the bracket).
  return best_schedule_constructor
      .RegisterOnlyOccupancyExceedsFunctionOccupancyTarget();
}

void DfsMaximizeOccupancyPolicy::FilterAndSortReadyList(
    const ScheduleConstructor &working,
    SmallVectorImpl<const ScheduleNode *> &out) {
  out.clear();
  ArrayRef<const ScheduleNode *> ready = working.GetReadyList();
  const GCNRegisterTracker &pressure_tracker =
      working.GetPressureTracker();

  // Filter: if any "pure reader" (real instruction with def_count
  // == 0) is in the ready list, keep ONLY pure readers — they
  // consume registers without producing new live ranges, so
  // picking them now strictly relieves pressure (or holds steady).
  // Subgraph proxies don't qualify (they're scope markers, not
  // real instructions); when a pure reader exists they're
  // deferred.
  bool any_pure_reader = false;
  for (const ScheduleNode *node : ready) {
    if (node->IsSchedulingUnit() &&
        pressure_tracker.GetDefCount(node) == 0) {
      any_pure_reader = true;
      break;
    }
  }
  if (any_pure_reader) {
    for (const ScheduleNode *node : ready) {
      if (node->IsSchedulingUnit() &&
          pressure_tracker.GetDefCount(node) == 0) {
        out.push_back(node);
      }
    }
  } else {
    out.assign(ready.begin(), ready.end());
  }

  // Sort: net (defs - kills) ascending — most pressure-relieving
  // first; pure readers naturally have net <= 0. NID tiebreak,
  // then topo. Proxies sort with net=0 (neutral; see helper).
  std::sort(
      out.begin(), out.end(),
      [&pressure_tracker](const ScheduleNode *a, const ScheduleNode *b) {
        int a_net = EffectiveNetDefMinusLastUse(a, pressure_tracker);
        int b_net = EffectiveNetDefMinusLastUse(b, pressure_tracker);
        if (a_net != b_net) {
          return a_net < b_net;
        }
        int a_nid = EffectiveNodeNum(a);
        int b_nid = EffectiveNodeNum(b);
        if (a_nid != b_nid) {
          return a_nid < b_nid;
        }
        return a->GetTopoIndex() < b->GetTopoIndex();
      });
}

bool DfsMaximizeOccupancyPolicy::ShouldBoundSearch(
    const ScheduleConstructor &schedule_constructor,
    const ScheduleConstructor &best_schedule_constructor,
    LengthHistoryTracker & /*length_history*/,
    PressureHistoryTracker &pressure_history) {
  // Score-bound: working's GetMetricScore(kMetric) is non-
  // increasing as more nodes are scheduled (peak pressure grows
  // monotonically, kMetric is max-direction so its score only
  // drops). If working's score is already <= best's, no
  // completion of working can strictly beat best.
  if (schedule_constructor.GetPressureTracker().GetMetricScore(kMetric) <=
      best_schedule_constructor.GetPressureTracker().GetMetricScore(kMetric)) {
    return true;
  }

  if constexpr (kUsePressureHistoryPruning) {
    // Mutating: records the current state on miss, may also
    // enqueue a fast-forward hint onto DfsSearch's replay queue.
    // See PressureHistoryTracker.
    if (pressure_history.IsDominatedElseRecord()) {
      return true;
    }
  }

  return false;
}

bool DfsMaximizeOccupancyPolicy::ShouldEndSearch(
    const ScheduleConstructor & /*schedule_constructor*/,
    const ScheduleConstructor &best_schedule_constructor) {
  return best_schedule_constructor.RegisterOnlyOccupancyIsAtOrAboveFunctionOccupancyTarget();
}
