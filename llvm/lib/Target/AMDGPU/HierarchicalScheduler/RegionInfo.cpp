//===- RegionInfo.cpp - Scheduling Region Info ----------------------------===//
//
// Implementation of the pressure-aware constructor: walks the region's
// instructions with GCNDownwardRPTracker and records the per-dimension
// peak pressure.
//
//===----------------------------------------------------------------------===//

#include "RegionInfo.h"
#include "GCNRegPressure.h"
#include "GCNRegisterTracker.h"
#include "GCNSubtarget.h"
#include "SIMachineFunctionInfo.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/MachineFunction.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

RegionInfo::RegionInfo(MachineBasicBlock::iterator begin,
                       MachineBasicBlock::iterator end,
                       const LiveIntervals &lis)
    : begin_(begin), end_(end) {
  // Walk [begin, end) forward, letting LLVM's downward tracker
  // compute the per-dimension peak pressure from LiveIntervals.
  GCNDownwardRPTracker tracker(lis);
  if (tracker.reset(*begin)) {
    tracker.advance(begin, end);
    original_peak_pressure_ = tracker.moveMaxPressure();
  }
  // If reset() returned false (region is empty except debug
  // values), original_peak_pressure_ stays at its default-constructed
  // zero state.

  const MachineFunction &mf = *begin->getParent()->getParent();
  const GCNSubtarget &st = mf.getSubtarget<GCNSubtarget>();
  original_register_only_occupancy_ =
      static_cast<int>(original_peak_pressure_.getOccupancy(st));

  // Ask GCNRegisterTracker's spill-regime predicate about the recorded peak:
  // the input schedule is in the spill regime if its peak exceeds the register
  // budget at the launch occupancy floor (getMinWavesPerEU) -- i.e. it would
  // spill to fit even the minimum occupancy. Same check the live-tracker
  // instance methods use, applied to the peak we already have, so occupancy and
  // spill-regime are both derived from the same recorded pressure.
  const unsigned launch_floor =
      mf.getInfo<SIMachineFunctionInfo>()->getMinWavesPerEU();
  original_in_spill_regime_ = GCNRegisterTracker::IsInSpillRegime(
      st, launch_floor, original_peak_pressure_);
}
