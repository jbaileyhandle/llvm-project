//===- ContinuousOccupancyScorer.h - Smooth occupancy score -----*- C++ -*-===//
//
// Scores register usage as occupancy plus partial credit for progress
// toward the next higher occupancy. Precomputed once per subtarget:
// ContinuousOccupancyScorer::Get(st).Score(num_vgpr, num_sgpr).
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_CONTINUOUSOCCUPANCYSCORER_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_CONTINUOUSOCCUPANCYSCORER_H

#include <algorithm>
#include <vector>

namespace llvm {

class GCNSubtarget;

namespace hierarchical_scheduler {

class ContinuousOccupancyScorer {
public:
  /// Score points per occupancy level.
  static constexpr int kOccScoreMultiplier = 1000;

  /// The scorer for `st`. Built on the first request for a given
  /// subtarget, then reused. The returned reference stays valid for
  /// the life of the process.
  static const ContinuousOccupancyScorer &Get(const GCNSubtarget &st);

  /// Score for `num_vgpr` VGPRs and `num_sgpr` SGPRs: the lower of the
  /// two per-class scores. A count past the hardware limit scores
  /// -(amount over), below every count that fits.
  int Score(int num_vgpr, int num_sgpr) const {
    return std::min(ScoreForClass(vgpr_score_by_count_, num_vgpr),
                    ScoreForClass(sgpr_score_by_count_, num_sgpr));
  }

private:
  explicit ContinuousOccupancyScorer(const GCNSubtarget &st);

  /// Table read for a count within the limit; -(amount over) past it.
  /// The limit is the table's last index.
  static int ScoreForClass(const std::vector<int> &score_by_count,
                           int num_regs) {
    int limit = static_cast<int>(score_by_count.size()) - 1;
    if (num_regs > limit) {
      return -(num_regs - limit);
    }
    return score_by_count[num_regs];
  }

  /// Score for each count 0..limit of one register class, where
  /// `occupancy_by_count[n]` is the occupancy n registers give.
  static std::vector<int>
  BuildScoreTable(const std::vector<int> &occupancy_by_count,
                  int max_waves);

  /// Index = register count (0..hardware limit), value = score.
  std::vector<int> vgpr_score_by_count_;
  std::vector<int> sgpr_score_by_count_;
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_CONTINUOUSOCCUPANCYSCORER_H
