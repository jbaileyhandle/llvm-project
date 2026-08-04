//===- DfsOccupancyDispatch.h - OccupancyPolicy -> DfsSearch ----*- C++ -*-===//
//
// One place that bridges a runtime OccupancyPolicy to the matching
// DfsSearch<PolicyT> instantiation, runs it, and returns the result. The
// occupancy DFS lives behind a compile-time policy *type*, so a runtime enum
// has to be switched onto a concrete type somewhere; both the flat occupancy
// DFS (ScheduleDAGHierarchicalScheduler) and the decompose-outer DFS
// (DecomposeAndSchedule) need that same four-way switch. Sharing it here keeps
// the policy->type mapping in a single spot -- the DFS analog of
// BfsDpSettings::ForOccupancy for BFS-DP -- so the two can't drift.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_DFSOCCUPANCYDISPATCH_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_DFSOCCUPANCYDISPATCH_H

#include "DfsSearch.h"                  // DfsSearch, SearchResult, ScheduleGraph
#include "HierarchicalConfigEnums.h"    // OccupancyPolicy
#include "SearchPolicies.h"             // the concrete DfsMaximize*OccupancyPolicy
#include "llvm/Support/ErrorHandling.h" // llvm_unreachable
#include <cstdint>
#include <optional>

namespace llvm {

class GCNSubtarget;
class MachineFunction;
class LiveIntervals;

namespace hierarchical_scheduler {

/// Construct the occupancy DfsSearch whose policy matches `policy` over
/// `graph` (per-region budget `timeout_us`), run it, tag the result winner
/// "dfs", and return it. `after_run(search, result)` runs once after Run()
/// while the typed search object is still alive: the flat path uses it to
/// print post-schedule info (PrintDfsPostScheduleInfo is a file-local template
/// over DfsSearch<PolicyT>, so it can't be called from here); callers needing
/// no post-processing pass a no-op. `AfterRunFn` is a template parameter
/// because the `search` it receives has a different type in each switch arm,
/// so the callback must be a generic lambda. The refine-spill-area policies
/// are DFS-capable, so unlike BfsDpSettings::ForOccupancy all four cases are
/// reachable here.
template <typename AfterRunFn>
SearchResult RunOccupancyDfs(OccupancyPolicy policy, ScheduleGraph &graph,
                             const GCNSubtarget &st, const MachineFunction &mf,
                             const LiveIntervals &lis,
                             std::optional<int64_t> timeout_us,
                             AfterRunFn after_run) {
  auto run_one = [&](auto search) -> SearchResult {
    SearchResult result = search.Run();
    after_run(search, result);
    result.winner = "dfs";
    return result;
  };
  switch (policy) {
  case OccupancyPolicy::kContinuousOccupancy:
    return run_one(DfsSearch<DfsMaximizeContinuousOccupancyPolicy>(
        graph, st, mf, lis, timeout_us));
  case OccupancyPolicy::kIntegerOccupancy:
    return run_one(DfsSearch<DfsMaximizeIntegerOccupancyPolicy>(
        graph, st, mf, lis, timeout_us));
  case OccupancyPolicy::kIntegerOccupancyRefineSpillArea:
    return run_one(DfsSearch<DfsMaximizeIntegerOccupancyRefineSpillAreaPolicy>(
        graph, st, mf, lis, timeout_us));
  case OccupancyPolicy::kContinuousOccupancyRefineSpillArea:
    return run_one(
        DfsSearch<DfsMaximizeContinuousOccupancyRefineSpillAreaPolicy>(
            graph, st, mf, lis, timeout_us));
  }
  llvm_unreachable("unhandled OccupancyPolicy");
}

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_DFSOCCUPANCYDISPATCH_H
