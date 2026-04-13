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
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineScheduler.h"
#include <type_traits>

namespace llvm {

class SIMachineFunctionInfo;

namespace hierarchical_scheduler {

class ScheduleGraph;
class ScheduleNode;

class ScheduleDAGHierarchicalScheduler : public ScheduleDAGMILive {
  // Regions recorded during schedule() for later processing in
  // finalizeSchedule().
  SmallVector<RegionInfo, 32> regions_;

  // Per-function state, set by InitFunction().
  SIMachineFunctionInfo *mfi_ = nullptr;

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

  // Run all shakedowns: synthetic test DAG, then per-region shakedowns
  // on the first region.
  void RunAllShakedowns();

  // Run shakedowns on a single region's graph: register trackers,
  // schedule length tracker, debug dumps.
  void RunRegionShakedowns(ScheduleGraph &graph);

  // Exercises graph algorithms on a synthetic test DAG with known structure.
  // Extended as new algorithms are added.
  void RunTestDAGShakedown();

  // Tests RegisterTracker on the first region: schedules instructions in
  // topo order and prints pressure at each step.
  void RunRegisterTrackerShakedown(ScheduleGraph &graph);

  // Tests GCNRegisterTracker: schedules in topo order printing pressure
  // at each step, then unschedules everything and verifies state returns
  // to zero.
  void RunGCNRegisterTrackerShakedown(ScheduleGraph &graph);

  // Cross-checks GCNRegisterTracker peak pressure against LLVM's
  // GCNUpwardRPTracker on the same instruction order. Both trackers
  // walk the same sequence; any difference is a tracking bug (or the
  // known whole-register kill overestimate).
  void VerifyGCNRegisterTracker(ScheduleGraph &graph,
                                ArrayRef<ScheduleNode *> order);

  // Tests ScheduleLengthTracker: schedules in topo order printing
  // length/bubbles at each step, then unschedules everything and
  // verifies state returns to zero.
  void RunScheduleLengthTrackerShakedown(ScheduleGraph &graph);

  // Tests ScheduleConstructor: schedules all nodes by always picking
  // the first ready node, then unschedules everything and verifies
  // round-trip.
  void RunScheduleConstructorShakedown(ScheduleGraph &graph);

  // Initialize per-function state. Called at the start of
  // RunHierarchicalScheduler / RunMaliciousScheduler. Stores mfi_
  // and resets occupancy to the pre-GCN-scheduler value.
  void InitFunction();

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
