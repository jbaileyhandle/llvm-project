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

#include "RegionInfo.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineScheduler.h"
#include <type_traits>

namespace llvm {
namespace hierarchical_scheduler {

class ScheduleGraph;

class ScheduleDAGHierarchicalScheduler : public ScheduleDAGMILive {
  // Regions recorded during schedule() for later processing in
  // finalizeSchedule().
  SmallVector<RegionInfo, 32> regions_;

public:
  ScheduleDAGHierarchicalScheduler(MachineSchedContext *C,
                                   std::unique_ptr<MachineSchedStrategy> S);

  // Called per-region by the outer driver. Records the region for later
  // processing in finalizeSchedule().
  void schedule() override;

  // Called once after all regions have been visited. Dispatches to the
  // configured scheduling algorithm (e.g., MaliciousScheduler).
  void finalizeSchedule() override;

  // Run the malicious scheduler over all recorded regions. For each region,
  // builds the DAG, computes the malicious schedule, and applies it.
  void RunMaliciousScheduler();

  // Main hierarchical scheduling path. Builds the LLVM DAG and our
  // ScheduleGraph for each region. Currently a no-op for scheduling — this
  // is where the hierarchical algorithm will be implemented.
  void RunHierarchicalScheduler();

  // Exercises graph algorithms on a synthetic test DAG with known structure.
  // Extended as new algorithms are added.
  void RunTestDAGShakedown();

  // Tests RegisterTracker on the first region: schedules instructions in
  // topo order and prints pressure at each step.
  void RunRegisterTrackerShakedown(ScheduleGraph &graph);

protected:
  // Apply a computed schedule order to the given region. Physically moves
  // MachineInstrs to match the order given by |scheduled_units|.
  // Must be called within a BeginRegion/EndRegion pair.
  void ApplyScheduleOrder(
      const RegionInfo &region,
      const std::vector<SUnit *> &scheduled_units);

  // Set up ScheduleDAGMILive state for the given region so that
  // moveInstruction() and other inherited methods work correctly.
  // Calls startBlock and enterRegion.
  void BeginRegion(const RegionInfo &region);

  // Clean up after scheduling a region.
  // Calls exitRegion and finishBlock.
  void EndRegion(const RegionInfo &region);

  // Process a region by calling BeginRegion, the provided action, then
  // EndRegion. Returns whatever the action returns.
  template <typename F>
  auto ProcessRegion(const RegionInfo &region, F action) {
    BeginRegion(region);
    if constexpr (std::is_void_v<decltype(action())>) {
      action();
      EndRegion(region);
    } else {
      auto result = action();
      EndRegion(region);
      return result;
    }
  }
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_H
