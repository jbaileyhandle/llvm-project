//===- BranchAndBoundSearch.h - B&B search over a graph --------*- C++ -*-===//
//
// Branch-and-bound search driver. Constructed with the dependencies
// it needs to build per-region ScheduleConstructors and query
// function-level state. The caller is expected to have already
// recorded and sorted the regions by the time Run() is called.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_BRANCHANDBOUNDSEARCH_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_BRANCHANDBOUNDSEARCH_H

#include "RegionInfo.h"
#include "llvm/ADT/ArrayRef.h"

namespace llvm {

class LiveIntervals;
class MachineFunction;
class SIMachineFunctionInfo;
class GCNSubtarget;

namespace hierarchical_scheduler {

class BranchAndBoundSearch {
public:
  // All referenced objects must outlive this BranchAndBoundSearch.
  // The GCNSubtarget is derived from `mf` in the constructor and
  // cached as a member.
  BranchAndBoundSearch(ArrayRef<RegionInfo> regions,
                       const MachineFunction &mf,
                       const LiveIntervals &lis,
                       SIMachineFunctionInfo *mfi);

  // Entry point. Signature TBD.
  void Run();

private:
  ArrayRef<RegionInfo> regions_;
  const MachineFunction *mf_;
  const LiveIntervals *lis_;
  const GCNSubtarget *st_;
  SIMachineFunctionInfo *mfi_;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_BRANCHANDBOUNDSEARCH_H
