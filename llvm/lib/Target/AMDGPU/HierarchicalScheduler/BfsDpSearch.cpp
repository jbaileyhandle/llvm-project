//===- BfsDpSearch.cpp - BFS-DP scheduler integration shell --------------===//
//
// Skeleton implementation. See BfsDpSearch.h for the contract.
//
//===----------------------------------------------------------------------===//

#include "BfsDpSearch.h"

namespace llvm {
namespace hierarchical_scheduler {

BfsDpSearch::BfsDpSearch(const ScheduleGraph *graph, const GCNSubtarget *st,
                         const MachineFunction *mf,
                         ScheduleMetric metric)
    : dag_(graph, st, mf, metric) {}

bool BfsDpSearch::Run() {
  return dag_.Build();
}

}  // namespace hierarchical_scheduler
}  // namespace llvm
