//===- ScheduleDAGHierarchicalScheduler.h - Hierarchical Scheduler -*- C++ -*-===//
//
// Hierarchical instruction scheduler for AMDGPU.
//
// This scheduler runs as a second pre-RA scheduling pass after the normal
// AMDGPU scheduler (GCNMaxOccupancySchedStrategy). It inherits from
// ScheduleDAGMILive (the same base class used by ScheduleDAGOptSched).
//
// Like OptSched, this scheduler uses schedule() to record regions during
// the normal per-region driver loop, and finalizeSchedule() to perform
// the actual scheduling after all regions have been collected.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_H

#include "llvm/CodeGen/MachineScheduler.h"

namespace llvm {

class ScheduleDAGHierarchicalScheduler : public ScheduleDAGMILive {
  // Recorded regions from the per-region schedule() calls.
  // Each entry is a (RegionBegin, RegionEnd) pair.
  SmallVector<
      std::pair<MachineBasicBlock::iterator, MachineBasicBlock::iterator>, 32>
      regions_;

  // The number of regions seen so far (used as a region counter).
  int region_number_ = 0;

  // Whether finalizeSchedule() has started replaying regions.
  bool scheduling_started_ = false;

public:
  ScheduleDAGHierarchicalScheduler(MachineSchedContext *C,
                                   std::unique_ptr<MachineSchedStrategy> S);

  // Called per-region by the outer driver. Records the region for later
  // processing in finalizeSchedule().
  void schedule() override;

  // Called once after all regions have been visited. This is where the
  // actual hierarchical scheduling logic will be implemented.
  void finalizeSchedule() override;
};

} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_H
