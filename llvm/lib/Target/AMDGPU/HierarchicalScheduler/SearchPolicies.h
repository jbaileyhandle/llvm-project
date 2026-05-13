//===- SearchPolicies.h - Per-pass search policies ------------*- C++ -*-===//
//
// Policies parameterize a search (DfsSearch, future AcoSearch, etc.)
// for a specific pass objective. A policy is search-strategy-coupled
// by design — bounding and ordering heuristics that are sound for DFS
// may not be sound for ACO or other strategies. Policy class names
// therefore telegraph the coupling (e.g., DfsMaximizeOccupancyPolicy).
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHPOLICIES_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHPOLICIES_H

#include "LengthHistoryTracker.h"
#include "PressureHistoryTracker.h"
#include "ScheduleConstructor.h"
#include "SubgraphFormation.h"
#include "llvm/ADT/SmallVector.h"

namespace llvm {
namespace hierarchical_scheduler {

class ScheduleNode;

// Common defaults for all search policies. Currently just provides a
// "no formation" default that derived policies can override when they
// want subgraph formation applied to the graph before search. Static
// methods are inherited (not virtual): if a derived class doesn't
// declare its own MakeFormationPolicy, lookup finds the base's
// version and DfsSearch sees an empty SubgraphFormationPolicy → the
// formation step is a one-call no-op (FormSubgraphs early-returns on
// an empty pipeline).
class SearchPolicyBase {
 public:
  // Default: no formation. Override in a derived policy to opt in.
  static SubgraphFormationPolicy MakeFormationPolicy() { return {}; }

  // Per-Recurse iteration shape and order. DfsSearch calls this
  // once at each Recurse() entry and iterates the resulting vector
  // via ScheduleConstructor::Schedule(node) — so a policy can:
  //   - filter the ready list down to a subset (omitted nodes are
  //     not scheduled in this Recurse iteration);
  //   - impose any priority order, including ones that depend on
  //     dynamic per-search state (min/max schedule cycles, register
  //     pressure, etc.).
  //
  // Default: copy the ready list as-is. The underlying list is
  // maintained in topo_index ascending order by ScheduleConstructor,
  // so a no-op default reproduces the previous static-order
  // behavior. Concrete policies override for filtering or dynamic
  // priority.
  static void FilterAndSortReadyList(
      const ScheduleConstructor &working,
      SmallVectorImpl<const ScheduleNode *> &out) {
    ArrayRef<const ScheduleNode *> ready = working.GetReadyList();
    out.assign(ready.begin(), ready.end());
  }

  // Length-min refine opt-ins. When EITHER is true, DfsSearch
  // relaxes the per-Recurse max_acceptable bound from
  // best.length - 1 to best.length so same-length completions
  // are produced and IsBetterThan picks among them under the
  // policy's metric. Two flags rather than one because the two
  // refinements are distinct policies — they could in principle
  // be combined, but for now each policy enables exactly one.
  //
  //   kRefineOccupancyAtSameLength: refine continuous register-
  //     occupancy within same length. Used by
  //     DfsMinimizeLengthRefineOccupancyPolicy.
  //   kRefineIlpAtSameLength: refine ILP within same length.
  //     Used by DfsMinimizeLengthRefineIlpPolicy. Also gates
  //     LengthHistoryTracker's ILP dim (per-open-producer
  //     inst_counts and locked-in ILP score participate in
  //     dominance) so the search isn't pruned prematurely on
  //     paths that could refine ILP.
  static constexpr bool kRefineOccupancyAtSameLength = false;
  static constexpr bool kRefineIlpAtSameLength = false;

  // History-based-domination pruning opt-in flags. Default false;
  // concrete policies override to true to enable the corresponding
  // history table in DfsSearch. The `if constexpr` gate in
  // DfsSearch::Recurse dead-strips the consult/insert code when the
  // flag is false, so policies that don't opt in pay nothing at
  // runtime in the hot recursion (the table is still constructed
  // but never queried).
  //
  // See AMDGPUHistoryDominationDesign.md §8.1.
  static constexpr bool kUseLengthHistoryPruning = false;
  static constexpr bool kUsePressureHistoryPruning = false;

  // Per-region wall-clock budget for the search, in milliseconds.
  // DfsSearch checks elapsed time against this on every Recurse()
  // entry and ends the search globally (returning the best schedule
  // found so far) once the budget is exhausted. The seeded baseline
  // (the input MF-order schedule) guarantees we always have at least
  // the input to fall back on, so a timeout never produces a
  // worse-than-input result. Concrete policies may override to give
  // one pass more budget than another (e.g. a longer length pass
  // once an occupancy ceiling has been pinned).
  static constexpr int kTimeoutMsPerRegion = 10;
};

// Policy for DFS when the objective is to minimize schedule length for
// a single region, SUBJECT TO not dropping the region's register-only
// occupancy below the function-wide ceiling set by an earlier
// occupancy pass.
class DfsMinimizeLengthPolicy : public SearchPolicyBase {
 public:
  static constexpr ScheduleMetric kMetric =
      ScheduleMetric::kMinimizeScheduleLength;

  // Override SearchPolicyBase: two-level filter+sort, modelled on
  // OptSched's cycle-by-cycle window-then-priority pattern.
  //
  //   Level 1 — no-bubble candidates: ready nodes whose effective
  //   min_schedule_cycle <= current_cycle. These can be scheduled
  //   at current_cycle with zero bubble. Sort ascending by
  //   effective max_schedule_cycle (most deadline-pressured first),
  //   tiebreak by topo_index.
  //
  //   Level 2 — bubble fallback: only if level 1 is empty (no ready
  //   node can issue at current_cycle without stalling). Take all
  //   ready nodes; sort ascending by effective min_schedule_cycle
  //   (smallest forced bubble first), then by effective
  //   max_schedule_cycle ascending (most deadline-pressured among
  //   same-min), then topo_index.
  //
  // Effective min/max for a scheduling-unit node is its own value
  // from the length tracker. For a subgraph proxy, it's the min
  // (smallest) of the corresponding values across the proxy's
  // subgraph members — so the proxy is treated as a composite of
  // its members.
  //
  // Falls back to topo-only when no max acceptable schedule length
  // is set (max_schedule_cycle would be undefined). DfsSearch
  // always sets one for this policy, so the fallback only matters
  // for the corner case of a search with no target seed.
  static void FilterAndSortReadyList(
      const ScheduleConstructor &working,
      SmallVectorImpl<const ScheduleNode *> &out);

  // Bound the current subtree if any of:
  //   (a) the working schedule's length lower bound exceeds the
  //       max acceptable schedule length stored on working's length
  //       tracker (read via GetMaxAcceptableScheduleLength) — i.e.,
  //       no completion of working can yield a schedule we'd accept.
  //       The max acceptable length is maintained by DfsSearch as
  //       min(requested_target_length, best.length - 1): we accept
  //       a completion if it's both at or below the iteration's
  //       target AND strictly better than current best; or
  //   (b) the working schedule's register-only occupancy has dropped
  //       below the function occupancy target — no completion can
  //       recover, and we must not degrade occupancy; or
  //   (c) (when kUseLengthHistoryPruning is true) some prior visit
  //       to this same scheduled-set partition recorded a state
  //       that dominates the current prefix on every Pareto
  //       dimension — see AMDGPUHistoryDominationDesign.md §5.
  //
  // SOUNDNESS:
  //   (a) LB is monotonically non-decreasing as nodes are scheduled
  //       (current_cycle + num_unscheduled never decreases, and the
  //       running max of scheduled_cycle + cp + 1 only grows), so
  //       once LB exceeds max acceptable length, every completion's
  //       length will too.
  //   (b) Peak register pressure is monotonically non-decreasing, so
  //       register-only occupancy is monotonically non-increasing.
  //       Once working drops below the function ceiling, no
  //       completion restores it.
  //   (c) See LengthHistoryTracker class comment.
  //
  // SIDE EFFECT: when (c) does NOT prune, the current prefix is
  // recorded in `length_history` for future-sibling comparison.
  // (IsDominatedElseInsert combines the check and record so we
  // don't pay two bucket lookups per visit.)
  //
  // `pressure_history` parameter is unused here — present only
  // because DfsSearch invokes ShouldBoundSearch with a uniform
  // signature across policies; this policy doesn't opt into
  // pressure-history pruning.
  //
  static bool ShouldBoundSearch(
      const ScheduleConstructor &schedule_constructor,
      const ScheduleConstructor &best_schedule_constructor,
      LengthHistoryTracker &length_history,
      PressureHistoryTracker &pressure_history);

  // End the search globally once best matches the graph-level length
  // floor (max(NumSchedulingUnits, cp_length + 1)) — no schedule can
  // be shorter,
  // so further exploration is pointless. The bound check in
  // ShouldBoundSearch handles incremental pruning; this is the cheap
  // "we hit the optimum" exit.
  static bool ShouldEndSearch(
      const ScheduleConstructor &schedule_constructor,
      const ScheduleConstructor &best_schedule_constructor);

  // Override SearchPolicyBase: length-min wants top-down
  // single-splitter formation with descendants + independents
  // bundled. The bracketing tightens the length-LB bound (subgraph
  // members can't drift apart to fill unrelated bubbles), reducing
  // branching during DFS.
  static SubgraphFormationPolicy MakeFormationPolicy() {
    return SubgraphFormationPolicy::TopDownSingleSplitterOnly();
  }

  // Override SearchPolicyBase: enable length history-domination
  // pruning. DFS records each visited prefix's (end_cycle,
  // frontier-LBs) in a per-partition Pareto frontier; on a later
  // visit to the same partition, if any recorded entry dominates
  // the current prefix on every dimension, the subtree is pruned.
  // See AMDGPUHistoryDominationDesign.md §5.
  static constexpr bool kUseLengthHistoryPruning = true;
};

// Length-min variant that, after finding a length-optimal schedule,
// continues exploring same-length completions to refine continuous
// register-occupancy score. Inherits everything from
// DfsMinimizeLengthPolicy except:
//   - kRefineOccupancyAtSameLength flipped to true (DfsSearch uses
//     this to relax the max_acceptable bound from best.length - 1
//     to best.length, allowing same-length completions to be
//     produced).
//   - ShouldEndSearch overridden to also require occupancy strictly
//     above the function target — when occupancy already exceeds
//     target, further pressure refinement won't unlock anything,
//     so we end at length floor as in the no-refine path; when
//     occupancy equals target, we keep searching to maximize
//     continuous score within the bracket.
//
// Subclassing gives us a distinct type that DfsSearch can
// instantiate alongside DfsMinimizeLengthPolicy in the same
// binary — the orchestrator picks which to use.
class DfsMinimizeLengthRefineOccupancyPolicy
    : public DfsMinimizeLengthPolicy {
 public:
  static constexpr bool kRefineOccupancyAtSameLength = true;

  static bool ShouldEndSearch(
      const ScheduleConstructor &schedule_constructor,
      const ScheduleConstructor &best_schedule_constructor);
};

// Length-min variant that, after finding a length-optimal schedule,
// continues exploring same-length completions to refine ILP score
// (defer first-uses to grow producer windows). Inherits everything
// from DfsMinimizeLengthPolicy except:
//   - kMetric: kMinimizeScheduleLengthRefineIlp — IsBetterThan
//     tiebreaks length asc, then ILP desc, then occupancy desc.
//   - kRefineIlpAtSameLength = true — relaxes the bound to allow
//     same-length completions through, AND gates the
//     LengthHistoryTracker's ILP dim (per-open-producer
//     inst_counts and locked-in ILP score participate in
//     dominance, so the search isn't pruned prematurely on
//     paths that could refine ILP).
//   - ShouldEndSearch never auto-stops at the length floor (no
//     natural "no further refinement possible" condition for
//     ILP — unlike occupancy refine, where exceeding the
//     function target stops the search). Relies on the per-
//     region timeout to bound total search.
class DfsMinimizeLengthRefineIlpPolicy
    : public DfsMinimizeLengthPolicy {
 public:
  static constexpr ScheduleMetric kMetric =
      ScheduleMetric::kMinimizeScheduleLengthRefineIlp;
  static constexpr bool kRefineIlpAtSameLength = true;

  static bool ShouldEndSearch(
      const ScheduleConstructor &schedule_constructor,
      const ScheduleConstructor &best_schedule_constructor);
};

// Policy for DFS when the objective is to maximize register-only
// occupancy for a single region.
class DfsMaximizeOccupancyPolicy : public SearchPolicyBase {
 public:
  // Use the continuous score, not the integer one. The integer
  // metric ties any two schedules in the same occupancy bracket;
  // continuous distinguishes within-bracket pressure too, which
  // gives ShouldBoundSearch real prune signal even when we're not
  // crossing a cliff.
  static constexpr ScheduleMetric kMetric =
      ScheduleMetric::kMaximizeContinuousRegisterOccupancyScore;

  // Override SearchPolicyBase: pressure-aware filter + sort.
  //
  //   Filter: if any "pure reader" (def_count == 0) is in the ready
  //   list, keep ONLY pure readers — they consume registers
  //   without producing new live ranges, so picking them now
  //   strictly relieves pressure (or holds steady). Defer all
  //   def-producing instructions to a later step.
  //
  //   Sort:
  //     - Primary: net_def_minus_kill ascending (most pressure-
  //       relief first; pure readers naturally have net <= 0).
  //     - NID ascending (LLVM input order — implicitly pressure-
  //       aware via LLVM's pre-RA scheduler).
  //     - topo_index ascending (deterministic final tiebreak).
  //
  // Subgraph proxies (no register effect on their own) treat their
  // members as the composite — see Effective* helpers in the .cpp.
  static void FilterAndSortReadyList(
      const ScheduleConstructor &working,
      SmallVectorImpl<const ScheduleNode *> &out);

  // Return true if no completion of the current partial schedule
  // can improve on best_schedule_constructor. Bounds:
  //   (a) Score-bound: working's GetMetricScore(kMetric) is
  //       non-increasing as nodes are scheduled (peak pressure
  //       grows monotonically). If working's score is already
  //       <= best's, no completion of working can strictly beat
  //       best.
  //   (b) History dominance (when kUsePressureHistoryPruning is
  //       true): the pressure-history tracker reports the
  //       current state can be pruned. See PressureHistoryTracker
  //       for what the tracker checks and why pruning is safe.
  //
  // SIDE EFFECT: IsDominatedElseRecord can mutate the tracker
  // (recording state for future comparison). See
  // PressureHistoryTracker.
  //
  // `length_history` parameter is unused here — present only
  // because DfsSearch invokes ShouldBoundSearch with a uniform
  // signature across policies. This policy doesn't opt into
  // length-history pruning.
  static bool ShouldBoundSearch(
      const ScheduleConstructor &schedule_constructor,
      const ScheduleConstructor &best_schedule_constructor,
      LengthHistoryTracker &length_history,
      PressureHistoryTracker &pressure_history);

  // Called on completed schedules after the IsBetterThan/update step.
  // Return true to end the entire search and have DfsSearch::Run
  // return whatever best is currently held. For maximize: end as
  // soon as best is at or above the function-wide occupancy ceiling
  // (no register-side improvement can raise effective occupancy
  // further).
  static bool ShouldEndSearch(
      const ScheduleConstructor &schedule_constructor,
      const ScheduleConstructor &best_schedule_constructor);

  // Override SearchPolicyBase: occupancy-max also takes the
  // TopDownSingleSplitterOnly minimal pipeline. The bracketing
  // groups the splitter's downstream slice into a contiguous
  // schedule region, which keeps consumer-side pressure peaks
  // local instead of letting unrelated work spread peaks across
  // the region.
  static SubgraphFormationPolicy MakeFormationPolicy() {
    return SubgraphFormationPolicy::TopDownSingleSplitterOnly();
  }

  // Override SearchPolicyBase: enable pressure history-domination
  // pruning. See PressureHistoryTracker.
  static constexpr bool kUsePressureHistoryPruning = true;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHPOLICIES_H
