//===- RegionInfo.cpp - Scheduling Region Info ----------------------------===//
//
// Implementation of the pressure-aware constructor: walks the region's
// instructions with GCNDownwardRPTracker and records the per-dimension
// peak pressure.
//
//===----------------------------------------------------------------------===//

#include "RegionInfo.h"
#include "GCNRegPressure.h"
#include "llvm/CodeGen/LiveIntervals.h"

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
    input_peak_pressure_ = tracker.moveMaxPressure();
  }
  // If reset() returned false (region is empty except debug
  // values), input_peak_pressure_ stays at its default-constructed
  // zero state.
}
