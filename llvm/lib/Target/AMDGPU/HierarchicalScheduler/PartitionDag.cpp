//===- PartitionDag.cpp - BFS-DP partition graph implementation ----------===//
//
// Skeleton implementation. All bodies report_fatal_error until the
// algorithm is wired in. See PartitionDag.h for the contract.
//
//===----------------------------------------------------------------------===//

#include "PartitionDag.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {
namespace hierarchical_scheduler {

PartitionDag::PartitionDag(const ScheduleGraph *graph,
                           const GCNSubtarget *st)
    : graph_(graph), st_(st) {}

void PartitionDag::Build() {
  report_fatal_error("PartitionDag::Build not yet implemented");
}

void PartitionDag::ExpandSource(PartitionNode * /*src*/,
                                std::vector<PartitionNode *> & /*next_layer*/) {
  report_fatal_error("PartitionDag::ExpandSource not yet implemented");
}

void PartitionDag::VisitSuccessor(
    PartitionNode * /*src*/, const ScheduleNode * /*next*/,
    std::vector<PartitionNode *> & /*next_layer*/) {
  report_fatal_error("PartitionDag::VisitSuccessor not yet implemented");
}

PartitionNode *PartitionDag::FindOrInsert(
    const ScheduleConstructor & /*probe_state*/,
    std::vector<PartitionNode *> & /*next_layer*/) {
  report_fatal_error("PartitionDag::FindOrInsert not yet implemented");
}

void PartitionDag::ReconstructSchedule() {
  report_fatal_error("PartitionDag::ReconstructSchedule not yet implemented");
}

}  // namespace hierarchical_scheduler
}  // namespace llvm
