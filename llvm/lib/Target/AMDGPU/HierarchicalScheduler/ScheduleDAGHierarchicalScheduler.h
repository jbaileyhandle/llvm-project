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
#include "llvm/ADT/STLFunctionalExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineScheduler.h"
#include <type_traits>
#include <vector>

namespace llvm {

class SIMachineFunctionInfo;

namespace hierarchical_scheduler {

class ScheduleConstructor;
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

  // Sort `regions_` ascending by the integer occupancy implied by
  // each region's original peak pressure. After the sort, `regions_[0]`
  // is the region whose original order produces the lowest occupancy
  // — the binding constraint on kernel-level occupancy.
  //
  // Uses a stable sort so that regions with the same original
  // occupancy keep their original recording order. This gives us
  // deterministic output across runs and library implementations,
  // and makes any secondary behavior (diagnostic printing,
  // downstream iteration) consistent with the order regions
  // appeared in the MachineFunction.
  void SortRegionsByOriginalRegisterOnlyOccupancyAscending();

  // Run the malicious scheduler over all recorded regions. For each region,
  // builds the DAG, computes the malicious schedule, and applies it.
  void RunMaliciousScheduler();

  // Main hierarchical scheduling path. Builds the LLVM DAG and our
  // ScheduleGraph for each region. Currently a no-op for scheduling — this
  // is where the hierarchical algorithm will be implemented.
  void RunHierarchicalScheduler();

  // Per-region graph-construction helper used by pass drivers.
  // Wraps ProcessRegion, calls buildSchedGraph, constructs a
  // ScheduleGraph from the fresh SUnits, computes its topo order,
  // and hands the graph to the callback. The graph is destroyed
  // when the callback returns; anything the callback wants to
  // persist across calls must be saved in a form stable across
  // buildSchedGraph rebuilds (e.g., MachineInstr* rather than
  // ScheduleNode* or SUnit*).
  void WithRegionGraph(
      const RegionInfo &region,
      function_ref<void(ScheduleGraph &)> callback);

  // Stub pass: schedule every region in topo order and apply.
  // Exercises the full pipeline (WithRegionGraph → ScheduleConstructor
  // → ApplyScheduleOrder) without any real search logic. Useful for
  // verifying the plumbing.
  void RunTopoPass();

  // Outer loop of the occupancy-maximization pass. Iterates regions
  // ascending by original register-only occupancy and calls
  // ScheduleRegionForMaximumOccupancy on each, tracking a running
  // kernel ceiling with the early-exit conditions documented at the
  // call site. Cross-checks MFI->getOccupancy() against
  // ComputeNonRegisterOccupancy before starting.
  void RunMaximizeOccupancyPass();

  // Per-region worker for RunMaximizeOccupancyPass. Returns the
  // maximum achieved all-factors occupancy for this region (arch max
  // ∩ LDS ∩ launch bounds ∩ best register-pressure dimension). This
  // is the per-region contribution to the kernel-wide occupancy
  // min. Currently a stub that returns the original schedule's
  // all-factors occupancy (i.e., no improvement).
  int ScheduleRegionForMaximumOccupancy(RegionInfo &region);

  // Initialize per-function state. Called at the start of
  // RunHierarchicalScheduler / RunMaliciousScheduler. Stores mfi_
  // and resets occupancy to the pre-GCN-scheduler value.
  void InitFunction();

private:
  // Run every shakedown / validation test. Called by
  // RunHierarchicalScheduler during development. All the individual
  // shakedown helpers (per-graph register trackers, schedule-
  // constructor round-trip, critical-path checks, etc.) live in
  // the anonymous namespace of Shakedowns.cpp and take whatever
  // state they need as explicit parameters; this method
  // orchestrates them with class state (MF, *LIS, EntrySU,
  // ExitSU, regions_).
  void RunAllShakedowns();

protected:
  // Apply a computed schedule order to the given region. Physically moves
  // MachineInstrs to match the order given by |scheduled_units|.
  // Every SUnit* must be non-null with a valid MachineInstr —
  // guaranteed when drawing from the SUnits vector that buildSchedGraph
  // populates (which only contains real instruction SUnits, not
  // the boundary EntrySU/ExitSU).
  // Must be called within a BeginRegion/EndRegion pair.
  void ApplyScheduleOrder(
      const RegionInfo &region,
      const std::vector<SUnit *> &scheduled_units);

  // Overload: extract the order from a completed ScheduleConstructor.
  // Walks sc.GetScheduleOrder(), skips ScheduleNodes whose SUnit is
  // null (synthetic entry/exit nodes from our ScheduleGraph that
  // don't correspond to real MachineInstrs), and delegates to the
  // SUnit* overload above.
  void ApplyScheduleOrder(
      const RegionInfo &region,
      const ScheduleConstructor &sc);

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
