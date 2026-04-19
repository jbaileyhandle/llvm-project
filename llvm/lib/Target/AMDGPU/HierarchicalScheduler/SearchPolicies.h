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

#include "ScheduleConstructor.h"
#include "llvm/ADT/SmallVector.h"

namespace llvm {
namespace hierarchical_scheduler {

class ScheduleNode;

// Policy for DFS when the objective is to maximize register-only
// occupancy for a single region. Hooks are static; see DfsSearch for
// the contract.
class DfsMaximizeOccupancyPolicy {
 public:
  static constexpr ScheduleMetric kMetric =
      ScheduleMetric::kRegisterOccupancy;

  // Return a pruned + sorted list of nodes to try next from
  // schedule_constructor's current ready list. Input is not modified.
  // First cut: no pruning, sort by ScheduleNode id ascending
  // (deterministic).
  static SmallVector<const ScheduleNode *, 16> PruneAndSortReadyList(
      const ScheduleConstructor &schedule_constructor);

  // Return true if no completion of the current partial schedule can
  // improve on best_schedule_constructor. First cut: always false
  // (never bound). Real bounds are subtle for maximize-occupancy
  // because register pressure is not monotonic.
  static bool ShouldBoundSearch(
      const ScheduleConstructor &schedule_constructor,
      const ScheduleConstructor &best_schedule_constructor);
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHPOLICIES_H
