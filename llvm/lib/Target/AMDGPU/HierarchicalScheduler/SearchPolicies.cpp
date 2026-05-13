//===- SearchPolicies.cpp - Per-pass search policies ---------------------===//

#include "SearchPolicies.h"
#include "IlpTracker.h"
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

// Slack threshold (in cycles) that splits ready candidates into
// "urgent" (must be picked soon to meet the deadline) and "relaxed"
// (plenty of room — let ILP guide the choice). Picked to be a few
// times the typical short-op latency on AMDGPU; fine-tuning is
// benchmark-driven. Must be > 0 so urgency is defined.
//
// Distinct from IlpTracker::desirable_spacing — slack is in CYCLES
// (deadline-derived); desirable_spacing is in INSTRUCTIONS (issue-
// count-derived). Different units, different roles.
//
// TODO: this should be DYNAMIC, not static. A candidate with
// slack=10 is effectively urgent when the ready list also has
// 15 other slack<=10 candidates competing for the same window —
// the deadline pressure is shared across all of them, and at
// IssueWidth=1 only one fires per cycle. The right cutoff
// depends on the slack distribution and the remaining issue
// capacity in the deadline window, not a single global number.
// See Appendix R.4 in AMDGPUMachineSchedulerGuide.md.
constexpr int kIlpRelaxedSlackThreshold = 8;

// All the per-node fields used to rank ready-list candidates in
// DfsMinimizeLengthPolicy::FilterAndSortReadyList. Built once per
// candidate via BuildSortKey; the four comparators below
// (UrgentLess, RelaxedLess, NoBubbleLess, ForcedBubbleLess) are
// pure permutations of these fields via std::tie — no further work.
//
// Field summary:
//   min_cycle, max_cycle  : earliest / latest cycle the node can
//     legally be scheduled at, given the current partial schedule.
//   ilp_cost              : ILP credit this node would forfeit by
//     scheduling now — sum over its uses of max(0,
//     desirable_spacing_R - spacing_R) for each open producer R
//     it would close as first consumer. Higher = closer to a
//     just-opened / heavy producer being cut short. See
//     IlpTracker::CloseCostForNode.
//   net_def_kill          : defs - first-use-kills, pressure proxy.
//   nid                   : LLVM input order (SUnit::NodeNum).
//   topo                  : final deterministic tiebreak.
//   is_urgent             : derived; true iff slack
//     max_cycle - current_cycle < kIlpRelaxedSlackThreshold.
struct SortKey {
  int min_cycle;
  int max_cycle;
  int ilp_cost;
  int net_def_kill;
  int nid;
  int topo;
  bool is_urgent;
};

SortKey BuildSortKey(const ScheduleNode *node, int current_cycle,
                     const ScheduleLengthTracker &length_tracker,
                     const GCNRegisterTracker &pressure_tracker,
                     const IlpTracker &ilp_tracker) {
  SortKey key;
  key.min_cycle = EffectiveMinScheduleCycle(node, length_tracker);
  key.max_cycle = EffectiveMaxScheduleCycle(node, length_tracker);
  key.ilp_cost = ilp_tracker.CloseCostForNode(node);
  key.net_def_kill = EffectiveNetDefMinusLastUse(node, pressure_tracker);
  key.nid = EffectiveNodeNum(node);
  key.topo = node->GetTopoIndex();
  key.is_urgent = key.max_cycle - current_cycle < kIlpRelaxedSlackThreshold;
  return key;
}

bool UrgentLess(const SortKey &a, const SortKey &b) {
  return std::tie(a.max_cycle, a.ilp_cost, a.net_def_kill, a.nid, a.topo) <
         std::tie(b.max_cycle, b.ilp_cost, b.net_def_kill, b.nid, b.topo);
}

bool RelaxedLess(const SortKey &a, const SortKey &b) {
  return std::tie(a.ilp_cost, a.max_cycle, a.net_def_kill, a.nid, a.topo) <
         std::tie(b.ilp_cost, b.max_cycle, b.net_def_kill, b.nid, b.topo);
}

// NoBubble: candidates that can fire at current_cycle (their
// min_cycle <= current_cycle). Urgent first, then relaxed; within
// tier, tier-specific less. Used when there's at least one
// candidate that won't force a stall.
bool NoBubbleLess(const SortKey &a, const SortKey &b) {
  if (a.is_urgent != b.is_urgent) {
    return a.is_urgent;
  }
  return a.is_urgent ? UrgentLess(a, b) : RelaxedLess(a, b);
}

// ForcedBubble: every candidate's min_cycle > current_cycle, so
// some stall is unavoidable. Min ascending outermost (smallest
// forced bubble), then within same-min the NoBubble ordering
// applies. Used as the fallback when no no-bubble candidate exists.
bool ForcedBubbleLess(const SortKey &a, const SortKey &b) {
  if (a.min_cycle != b.min_cycle) {
    return a.min_cycle < b.min_cycle;
  }
  return NoBubbleLess(a, b);
}

} // namespace

void DfsMinimizeLengthPolicy::FilterAndSortReadyList(
    const ScheduleConstructor &working,
    SmallVectorImpl<const ScheduleNode *> &out) {
  out.clear();
  const ScheduleLengthTracker &length_tracker =
      working.GetLengthTracker();
  ArrayRef<const ScheduleNode *> ready = working.GetReadyList();

  // Precondition: the search must have configured the bound by the
  // time we're ranking — both the urgency tier and the per-node
  // SortKey rely on MaxScheduleCycle being readable. DfsSearch
  // wires this in its constructor (RecomputeWorkingMaxScheduleCycles)
  // before any Recurse fires, so this guard catches caller-order
  // bugs rather than expected states.
  if (!length_tracker.HasMaxAcceptableScheduleLength()) {
    report_fatal_error(
        "DfsMinimizeLengthPolicy::FilterAndSortReadyList called "
        "before SetMaxAcceptableScheduleLength — bound not "
        "configured; sort would have no urgency basis");
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
  const GCNRegisterTracker &pressure_tracker =
      working.GetPressureTracker();
  const IlpTracker &ilp_tracker = working.GetIlpTracker();

  // Build a sort key for every ready candidate once. The
  // SortKey-aware comparators below (NoBubbleLess /
  // ForcedBubbleLess) are pure permutations of these fields —
  // no per-comparison Effective*-helper calls.
  SmallVector<std::pair<const ScheduleNode *, SortKey>, 64> keyed;
  keyed.reserve(ready.size());
  for (const ScheduleNode *node : ready) {
    keyed.push_back({node, BuildSortKey(node, current_cycle, length_tracker,
                                        pressure_tracker, ilp_tracker)});
  }

  // Partition: no-bubble candidates (min_cycle <= current_cycle)
  // first, forced-bubble candidates after. If any no-bubble
  // candidate exists, only that group goes into `out` (sorted by
  // NoBubbleLess) — we never offer a forced-bubble candidate when
  // a no-bubble one is available. Otherwise the full list goes
  // in, sorted by ForcedBubbleLess (which puts min-asc outermost).
  auto first_forced = std::partition(
      keyed.begin(), keyed.end(),
      [current_cycle](const auto &entry) {
        return entry.second.min_cycle <= current_cycle;
      });

  if (first_forced != keyed.begin()) {
    std::sort(keyed.begin(), first_forced,
              [](const auto &a, const auto &b) {
                return NoBubbleLess(a.second, b.second);
              });
    for (auto it = keyed.begin(); it != first_forced; ++it) {
      out.push_back(it->first);
    }
    return;
  }

  std::sort(keyed.begin(), keyed.end(),
            [](const auto &a, const auto &b) {
              return ForcedBubbleLess(a.second, b.second);
            });
  for (const auto &entry : keyed) {
    out.push_back(entry.first);
  }
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

bool DfsMinimizeLengthRefineIlpPolicy::ShouldEndSearch(
    const ScheduleConstructor & /*schedule_constructor*/,
    const ScheduleConstructor &best_schedule_constructor) {
  // No natural "no further ILP refinement possible" condition —
  // unlike occupancy refine, where exceeding the function target
  // makes further refinement worthless. ILP can in principle
  // keep improving until every producer is fully saturated, and
  // we have no cheap test for that. Always return false; the
  // per-region timeout (Policy::kTimeoutMsPerRegion) is the
  // only stopping condition for refine-ILP at length floor.
  // Above floor, we'd want to keep going for length improvement
  // anyway, so the answer is uniformly false.
  (void)best_schedule_constructor;
  return false;
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
