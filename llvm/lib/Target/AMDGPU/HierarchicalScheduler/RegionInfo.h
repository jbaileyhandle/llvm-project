//===- RegionInfo.h - Scheduling Region Info -------------------*- C++ -*-===//
//
// A lightweight wrapper around a scheduling region's boundaries plus
// optional metadata computed at construction time. Stores RegionBegin
// and RegionEnd iterators and (when constructed with a LiveIntervals
// reference) the per-dimension peak register pressure of the region's
// instructions as they currently appear in the basic block.
//
// The pressure-aware constructor is used by the hierarchical scheduler
// when recording regions for deferred processing: we measure pressure
// on whatever order the prior pass (typically the GCN scheduler) left
// behind, and use that as a per-region scalar key for sorting regions
// by difficulty (e.g., "hardest region first" in an occupancy pass).
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_REGIONINFO_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_REGIONINFO_H

#include "GCNRegPressure.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include <iterator>

namespace llvm {

class LiveIntervals;

namespace hierarchical_scheduler {

class RegionInfo {
public:
  // Boundary-only constructor. Peak pressure is left zero; use this
  // when pressure isn't needed (e.g., tests without LiveIntervals).
  RegionInfo(MachineBasicBlock::iterator begin,
             MachineBasicBlock::iterator end)
      : begin_(begin), end_(end) {}

  // Boundary + pressure constructor. Walks [begin, end) via
  // GCNDownwardRPTracker and stores the per-dimension peak pressure.
  RegionInfo(MachineBasicBlock::iterator begin,
             MachineBasicBlock::iterator end,
             const LiveIntervals &lis);

  MachineBasicBlock::iterator Begin() const { return begin_; }
  MachineBasicBlock::iterator End() const { return end_; }

  // The parent basic block, derived from the begin iterator.
  MachineBasicBlock *GetBlock() const { return begin_->getParent(); }

  // Number of instructions in the region. Region size is exceedingly
  // unlikely to exceed int range.
  int GetNumInstrs() const {
    return static_cast<int>(std::distance(begin_, end_));
  }

  // Per-dimension peak pressure of the region's instructions at
  // construction time. "Peak" is componentwise: each dimension's
  // value is the max of that dimension across the walk, not
  // necessarily observed at the same instruction.
  const GCNRegPressure &GetInputPeakPressure() const {
    return input_peak_pressure_;
  }

  // Integer register-pressure-only occupancy implied by the peak
  // pressure on the given subtarget: min(occ_from_sgpr, occ_from_vgpr).
  // Symmetric with GCNRegisterTracker::GetRegisterOccupancy — both
  // ignore LDS and launch bounds and return just the register-
  // dimension ceiling. Lower = harder.
  unsigned GetInputRegisterOccupancy(const GCNSubtarget &st) const {
    return input_peak_pressure_.getOccupancy(st);
  }

private:
  MachineBasicBlock::iterator begin_;
  MachineBasicBlock::iterator end_;
  GCNRegPressure input_peak_pressure_;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_REGIONINFO_H
