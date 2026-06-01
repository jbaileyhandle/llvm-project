//===- OccupancyTargetUtil.h - Floor-clamped MFI occupancy ops -*- C++ -*-===//
//
// Thin wrapper around SIMachineFunctionInfo's occupancy mutator that
// enforces the function's structural floor (the minimum waves-per-EU
// the kernel's launch attributes guarantee). Callers in the
// HierarchicalScheduler should go through this rather than calling
// MFI->limitOccupancy(...) directly: the underlying API has no floor
// gate (SIMachineFunctionInfo.h:1094 lowers Occupancy to any value
// at-or-below the current one), so a stray ratchet below the
// structural floor leaves MFI->getOccupancy() reporting a target
// BELOW what the kernel will actually run at -- misleading any
// downstream code that reads the target as the real achievable
// occupancy. Routing through LimitOccupancyAboveFloor keeps the
// cached target trustworthy as a single source of truth.
//
//===---------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_OCCUPANCYTARGETUTIL_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_OCCUPANCYTARGETUTIL_H

#include "SIMachineFunctionInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include <algorithm>

namespace llvm {
namespace hierarchical_scheduler {

/// Lower MFI's current occupancy target to `limit`, but clamp `limit`
/// up to MFI->getMinWavesPerEU() first so the target cannot drop below
/// the structural floor. Behaves the same as
/// SIMachineFunctionInfo::limitOccupancy except for one added rule:
/// when the proposed `limit` would drop the target below the floor,
/// the floor wins and the target settles there instead. If `limit` is
/// already >= the current target, this is a no-op (same as the
/// underlying API).
///
/// `limit` is `int` so call sites stay in the project's int-typed
/// world; the AMDGPU API takes unsigned, so the int->unsigned bridge
/// lives here. Negative values are a caller bug (fatal error).
inline void LimitOccupancyAboveFloor(SIMachineFunctionInfo &mfi, int limit) {
  if (limit < 0) {
    report_fatal_error("LimitOccupancyAboveFloor: occupancy limit must be "
                       "non-negative");
  }
  mfi.limitOccupancy(
      std::max(static_cast<unsigned>(limit), mfi.getMinWavesPerEU()));
}

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_OCCUPANCYTARGETUTIL_H
