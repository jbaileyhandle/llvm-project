//===- BfsDpSearch.cpp - BFS-DP scheduler integration shell --------------===//
//
// Skeleton implementation. See BfsDpSearch.h for the contract.
//
//===----------------------------------------------------------------------===//

#include "BfsDpSearch.h"

namespace llvm {
namespace hierarchical_scheduler {

BfsDpSearch::BfsDpSearch(const ScheduleGraph *graph, const GCNSubtarget *st)
    : dag_(graph, st) {}

void BfsDpSearch::Run() {
  dag_.Build();
}

}  // namespace hierarchical_scheduler
}  // namespace llvm
