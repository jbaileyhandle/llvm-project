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

  // Per-region wall-clock budget for the search. DfsSearch checks
  // elapsed time against this on every Recurse() entry and ends the
  // search globally (returning the best schedule found so far) once
  // the budget is exhausted. The seeded baseline (the input MF-order
  // schedule) guarantees we always have at least the input to fall
  // back on, so a timeout never produces a worse-than-input result.
  // Concrete policies may override to give one pass more budget than
  // another (e.g. a longer length pass once an occupancy ceiling has
  // been pinned).
  static constexpr int kTimeoutSecondsPerRegion = 20;
};

// Policy for DFS when the objective is to minimize schedule length for
// a single region, SUBJECT TO not dropping the region's register-only
// occupancy below the function-wide ceiling set by an earlier
// occupancy pass.
class DfsMinimizeLengthPolicy : public SearchPolicyBase {
 public:
  static constexpr ScheduleMetric kMetric =
      ScheduleMetric::kMinimizeScheduleLength;

  // Strict total order used to sort ScheduleConstructor's ready list.
  // First cut: topo_index ascending (deterministic, unique per node).
  // Planned enhancement: slack-based ordering (ALAP - ASAP, low-slack
  // critical-path nodes first, tiebreak by topo_index). Finding a
  // shorter schedule sooner means best.length drops earlier in the
  // search, so the length-LB bound prunes more subtrees.
  static bool ReadyCompare(const ScheduleNode *a, const ScheduleNode *b) {
    return a->GetTopoIndex() < b->GetTopoIndex();
  }

  // Bound the current subtree if any of:
  //   (a) the working schedule's length lower bound is at or above
  //       best's current length — no completion can strictly beat
  //       best; or
  //   (b) the working schedule's register-only occupancy has dropped
  //       below the function ceiling — no completion can recover, and
  //       we must not degrade occupancy; or
  //   (c) (when kUseLengthHistoryPruning is true) some prior visit
  //       to this same scheduled-set partition recorded a state
  //       that dominates the current prefix on every Pareto
  //       dimension — see AMDGPUHistoryDominationDesign.md §5.
  //
  // SOUNDNESS:
  //   (a) LB is monotonically non-decreasing as nodes are scheduled
  //       (current_cycle + num_unscheduled never decreases, and the
  //       running max of scheduled_cycle + cp + 1 only grows), so
  //       once LB >= best.length, every completion has length >= LB
  //       >= best.length.
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

  // Strict total order used to sort ScheduleConstructor's ready list.
  // Topo_index ascending — deterministic, unique per node. No
  // heuristic for occupancy yet; 
  static bool ReadyCompare(const ScheduleNode *a, const ScheduleNode *b) {
    return a->GetTopoIndex() < b->GetTopoIndex();
  }

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
  // signature across policies; this policy doesn't opt into
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
