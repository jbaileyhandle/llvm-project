//===- ContinuousOccupancyScorer.cpp - Smooth occupancy score -------------===//
//
// Builds the per-subtarget score tables. See ContinuousOccupancyScorer.h.
//
// Where occupancy comes from. "A ──▶ B" means A calls B.
//
//   LLVM (GCNSubtarget):
//
//     getOccupancyWithNumSGPRs(n)              SGPR count -> occupancy
//       ├──▶ getOccupancyWithNumSGPRsLegacy     original table; used with
//       │                                       misched.txt
//       │                                       legacy_sgpr_occupancy_table,
//       │                                       off gfx8/9, or without a
//       │                                       trap handler
//       └──▶ getOccupancyWithNumSGPRsTrapAware  default on gfx906
//
//     getMaxNumSGPRs(occ, ...)                 occupancy -> max SGPRs; not
//                                              switched by the flag: matches
//                                              the trap-aware table only
//
//     getOccupancyWithNumVGPRs(n)              VGPR count -> occupancy
//     getMaxNumVGPRs(occ)                      occupancy -> max VGPRs
//
//   Scheduler:
//
//     GCNRegisterTracker::GetRegisterOnlyOccupancy
//       ──▶ GCNRegPressure::getOccupancy
//             ──▶ getOccupancyWithNumSGPRs, getOccupancyWithNumVGPRs
//
//     ContinuousOccupancyScorer (construction)
//       ──▶ getOccupancyWithNumSGPRs, getOccupancyWithNumVGPRs
//           for every count up to the hardware limit
//
//     GCNRegisterTracker, PartitionDag ──▶ ContinuousOccupancyScorer::Score
//
// The scorer derives each occupancy's register ceiling from the
// occupancy functions instead of calling getMaxNumSGPRs/getMaxNumVGPRs,
// so its scores always agree with the occupancy the tracker reports,
// under either SGPR table.
//
//===----------------------------------------------------------------------===//

#include "ContinuousOccupancyScorer.h"
#include "GCNSubtarget.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/Support/ErrorHandling.h"
#include <memory>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

const ContinuousOccupancyScorer &
ContinuousOccupancyScorer::Get(const GCNSubtarget &st) {
  // One scorer per subtarget. Held by unique_ptr so a scorer does not
  // move when the map grows; callers keep references to it.
  static DenseMap<const GCNSubtarget *,
                  std::unique_ptr<ContinuousOccupancyScorer>>
      cache;
  std::unique_ptr<ContinuousOccupancyScorer> &entry = cache[&st];
  if (entry == nullptr) {
    entry.reset(new ContinuousOccupancyScorer(st));
  }
  return *entry;
}

ContinuousOccupancyScorer::ContinuousOccupancyScorer(const GCNSubtarget &st) {
  int max_waves = static_cast<int>(st.getMaxWavesPerEU());

  // Occupancy for each count from 0 to the hardware limit (see file
  // comment).
  int vgpr_limit = static_cast<int>(st.getAddressableNumVGPRs());
  std::vector<int> vgpr_occupancy_by_count(vgpr_limit + 1);
  for (int n = 0; n <= vgpr_limit; ++n) {
    vgpr_occupancy_by_count[n] =
        static_cast<int>(st.getOccupancyWithNumVGPRs(n));
  }

  int sgpr_limit = static_cast<int>(st.getAddressableNumSGPRs());
  std::vector<int> sgpr_occupancy_by_count(sgpr_limit + 1);
  for (int n = 0; n <= sgpr_limit; ++n) {
    sgpr_occupancy_by_count[n] =
        static_cast<int>(st.getOccupancyWithNumSGPRs(n));
  }

  vgpr_score_by_count_ = BuildScoreTable(vgpr_occupancy_by_count, max_waves);
  sgpr_score_by_count_ = BuildScoreTable(sgpr_occupancy_by_count, max_waves);
}

std::vector<int> ContinuousOccupancyScorer::BuildScoreTable(
    const std::vector<int> &occupancy_by_count, int max_waves) {
  constexpr int M = kOccScoreMultiplier;
  int limit = static_cast<int>(occupancy_by_count.size()) - 1;

  // Ceiling of each occupancy: the most registers that still give it.
  // Occupancy never rises as the count rises, so the last count seen
  // at an occupancy is its ceiling. An occupancy no count gives (e.g.
  // SGPR occupancy 9 on gfx906, where 64 SGPRs gives 10 and 65 gives
  // 8) keeps -1. The lowest occupancy's ceiling is the hardware limit.
  std::vector<int> ceiling_by_occupancy(max_waves + 1, -1);
  for (int n = 0; n <= limit; ++n) {
    ceiling_by_occupancy[occupancy_by_count[n]] = n;
  }

  std::vector<int> score_by_count(limit + 1);
  for (int n = 0; n <= limit; ++n) {
    int occ = occupancy_by_count[n];
    int ceil = ceiling_by_occupancy[occ];

    // Floor: the ceiling of the next higher occupancy that some count
    // gives, skipping ones that none does. 0 if there is none. This is
    // a search rather than ceiling_by_occupancy[occ + 1] because SGPRs
    // skip occupancies: on gfx906 no SGPR count gives occupancy 9, so
    // the band for occupancy 8 (65..80) must use occupancy 10's
    // ceiling (64) as its floor.
    int floor = 0;
    for (int higher_occ = occ + 1; higher_occ <= max_waves; ++higher_occ) {
      if (ceiling_by_occupancy[higher_occ] >= 0) {
        floor = ceiling_by_occupancy[higher_occ];
        break;
      }
    }

    // ceil > floor: floor is a count giving a higher occupancy than
    // ceil does, and occupancy never rises with count. The only way
    // to break this is a top occupancy given by count 0 alone.
    if (ceil == floor) {
      report_fatal_error("ContinuousOccupancyScorer: empty occupancy band");
    }
    // occ * M, plus up to M more the further n is below ceil.
    score_by_count[n] = M * occ + M * (ceil - n) / (ceil - floor);
  }
  return score_by_count;
}
