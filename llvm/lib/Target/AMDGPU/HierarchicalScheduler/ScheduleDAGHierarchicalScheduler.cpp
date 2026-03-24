//===- ScheduleDAGHierarchicalScheduler.cpp - Hierarchical Scheduler ------===//
//
// Implementation of the hierarchical instruction scheduler for AMDGPU.
//
// Currently a no-op: schedule() records regions and finalizeSchedule()
// does nothing. The actual scheduling logic will be added later.
//
//===----------------------------------------------------------------------===//

#include "ScheduleDAGHierarchicalScheduler.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "machine-scheduler"

using namespace llvm;

ScheduleDAGHierarchicalScheduler::ScheduleDAGHierarchicalScheduler(
    MachineSchedContext *C, std::unique_ptr<MachineSchedStrategy> S)
    : ScheduleDAGMILive(C, std::move(S)) {}

// Called per-region by the outer driver (scheduleRegions). We record the region
// boundaries for later use in finalizeSchedule(), rather than scheduling now.
// This is the same deferred-scheduling pattern used by ScheduleDAGOptSched
// when two-pass scheduling is enabled.
void ScheduleDAGHierarchicalScheduler::schedule() {
  regions_.push_back(std::make_pair(RegionBegin, RegionEnd));
  ++region_number_;

  // TODO: Remove this temporary print once we've confirmed the pass runs.
  llvm::outs() << "HierarchicalScheduler: recorded region "
               << region_number_ << " (" << NumRegionInstrs
               << " instrs)\n";
}

// Called once after all regions in all blocks have been visited.
// This is where the actual hierarchical scheduling logic will go.
// For now, this is a no-op — the regions are recorded but not rescheduled.
void ScheduleDAGHierarchicalScheduler::finalizeSchedule() {
  // TODO: Remove this temporary print once we've confirmed the pass runs.
  llvm::outs() << "HierarchicalScheduler: finalizeSchedule called with "
               << regions_.size() << " regions\n";

  // TODO: Implement hierarchical scheduling logic here.
  // The recorded regions are available in regions_.

  ScheduleDAGMILive::finalizeSchedule();
}
