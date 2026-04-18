//===- BranchAndBoundSearch.cpp - B&B search over a graph ----------------===//
//
// See BranchAndBoundSearch.h.
//
//===----------------------------------------------------------------------===//

#include "BranchAndBoundSearch.h"
#include "GCNSubtarget.h"
#include "llvm/CodeGen/MachineFunction.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

BranchAndBoundSearch::BranchAndBoundSearch(ArrayRef<RegionInfo> regions,
                                           const MachineFunction &mf,
                                           const LiveIntervals &lis,
                                           SIMachineFunctionInfo *mfi)
    : regions_(regions),
      mf_(&mf),
      lis_(&lis),
      st_(&mf.getSubtarget<GCNSubtarget>()),
      mfi_(mfi) {}

void BranchAndBoundSearch::Run() {}
