//===- RegionInfo.h - Scheduling Region Info -------------------*- C++ -*-===//
//
// A lightweight wrapper around a scheduling region's boundaries plus
// metadata computed at construction time. Stores the region's bounds,
// anchored on the scheduling boundaries around it so they stay right after
// the region is reordered; the per-dimension peak register pressure of the
// region's instructions as they appear in the basic block at construction;
// and the region's index within its function.
//
// The constructor is used by the hierarchical scheduler
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
  // Walks [begin, end) via GCNDownwardRPTracker and stores the
  // per-dimension peak pressure. `begin` and `end` are a scheduling
  // region's bounds as the scheduling driver gives them. `index` is the
  // region's index within its function (see GetIndex).
  RegionInfo(MachineBasicBlock::iterator begin,
             MachineBasicBlock::iterator end,
             const LiveIntervals &lis, int index);

  // The region's first instruction: the one after the boundary above the
  // region, or the block's first. Found this way, it stays right after a
  // stage reorders the region (see boundary_above_).
  MachineBasicBlock::iterator Begin() const {
    if (starts_block_) {
      return block_->begin();
    }
    return std::next(boundary_above_);
  }
  MachineBasicBlock::iterator End() const { return end_; }

  // The parent basic block.
  MachineBasicBlock *GetBlock() const { return block_; }

  // Number of instructions in the region. Region size is exceedingly
  // unlikely to exceed int range.
  int GetNumInstrs() const {
    return static_cast<int>(std::distance(Begin(), end_));
  }

  // Per-dimension peak pressure of the region's instructions at
  // construction time. "Peak" is componentwise: each dimension's
  // value is the max of that dimension across the walk, not
  // necessarily observed at the same instruction.
  const GCNRegPressure &GetOriginalPeakPressure() const {
    return original_peak_pressure_;
  }

  // Integer register-pressure-only occupancy implied by the peak
  // pressure on the target subtarget: min(occ_from_sgpr, occ_from_vgpr).
  // Cached at construction — the subtarget is fixed per MachineFunction
  // and the peak pressure does not change after construction, so the
  // occupancy is a stable scalar. Symmetric with
  // GCNRegisterTracker::GetRegisterOnlyOccupancy — both ignore LDS
  // and launch bounds and return just the register-dimension
  // ceiling. Lower = harder.
  int GetOriginalRegisterOnlyOccupancy() const {
    return original_register_only_occupancy_;
  }

  // Whether the region's input schedule is in the spill regime: its peak VGPR
  // or SGPR pressure exceeds the register budget at the launch occupancy floor
  // (getMinWavesPerEU), so it would spill to fit even the minimum occupancy.
  // Mirrors GCNRegisterTracker::IsPeakInSpillRegime, computed at construction
  // from the same recorded peak pressure -- a stable scalar like the occupancy
  // above. The occupancy pass uses it to avoid skipping a spilling region even
  // when its occupancy is already at the kernel ceiling.
  bool IsOriginalInSpillRegime() const { return original_in_spill_regime_; }

  // The region's index within its function: 0 for the first region the
  // scheduling driver visits, then 1, and so on (blocks in function order,
  // each block's regions bottom-up). The hierarchical scheduler and the
  // length analyzer both number their regions this way, so this is also the
  // region's `index` in the memory viewer's dump.
  int GetIndex() const { return index_; }

private:
  // The region's block, and the instruction just above the region unless the
  // region starts the block. getSchedRegions (MachineScheduler.cpp) grows a
  // region upward until the instruction above it is a scheduling boundary, so
  // boundary_above_ is one. A boundary is in no region, and schedulers only
  // move instructions within the region they are scheduling, so it never
  // moves. The region's first instruction isn't stored: a stage that reorders
  // the region can move it down. end_ (the boundary below the region, or the
  // block's end) never moves either.
  MachineBasicBlock *block_;
  bool starts_block_;
  MachineBasicBlock::iterator boundary_above_;
  MachineBasicBlock::iterator end_;
  GCNRegPressure original_peak_pressure_;
  int original_register_only_occupancy_ = 0;
  bool original_in_spill_regime_ = false;
  int index_ = -1;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_REGIONINFO_H
