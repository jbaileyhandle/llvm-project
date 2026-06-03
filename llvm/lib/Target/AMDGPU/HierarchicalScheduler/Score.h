//===- Score.h - Canonical comparable schedule score ----------*- C++ -*-===//
//
// Score is the canonical, single-type representation of "how good is this
// schedule by metric M." Every metric (integer occupancy, continuous
// occupancy + area, schedule length, length + ILP refinement, ...) produces
// a Score via ScheduleConstructor::GetScore(ScheduleMetric), and downstream
// code just uses < / > -- no branching on what's inside.
//
// Canonical orientation: higher = better. Score::Make() requires every slot
// to be tagged Score::Higher{x} or Score::Lower{x}; Lower values are
// negated internally so lex compare on the underlying array always means
// "left < right iff left is strictly worse than right."
//
// ScheduleMetric lives here (rather than its own header) because its sole
// purpose is to key into GetScore(); the two are tightly coupled. The
// header is small and has no ScheduleConstructor dependency, so consumers
// that only need the enum can include it without pulling in
// ScheduleConstructor.h.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCORE_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCORE_H

#include <array>
#include <cassert>
#include <cstdint>

namespace llvm {
namespace hierarchical_scheduler {

/// Criterion by which two ScheduleConstructor states are compared. Used
/// with ScheduleConstructor::GetScore (and the thin IsBetterThan wrapper
/// over it). Names are explicit about direction (kMaximize*, kMinimize*)
/// so the call site doesn't have to remember which way each metric is
/// "better".
enum class ScheduleMetric {
  /// Integer register occupancy (GetRegisterOnlyOccupancy).
  /// Coarse -- schedules in the same occupancy bracket tie.
  kMaximizeRegisterOccupancy,

  /// Continuous register occupancy score (GetContinuousOccupancyScore).
  /// Smooth within brackets -- useful when search needs to see
  /// progress toward the next higher bracket.
  kMaximizeContinuousRegisterOccupancyScore,

  /// Continuous occupancy (peak) primary, with occupancy-area
  /// (sum of per-step continuous score) as the same-peak tiebreak;
  /// higher area = pressure kept lower throughout. See
  /// GCNRegisterTracker::GetContinuousOccupancyArea.
  kMaximizeContinuousOccupancyScoreThenMaximizeContinuousOccupancyArea,

  /// Current schedule length in cycles.
  kMinimizeScheduleLength,

  /// Current schedule length in cycles, taken in the opposite
  /// direction: higher is better. Drives a "maximize length"
  /// search -- useful as a control / worst-legal-schedule baseline
  /// for comparing against the length-min objective. Still subject
  /// to the occupancy floor (the policy that uses this metric does
  /// not relax the function-wide register-occupancy constraint).
  kMaximizeScheduleLength,

  /// Length-min with ILP score as the same-length tiebreaker.
  /// Length asc primary; among same-length, higher locked-in
  /// IlpTracker::GetIlpScore wins; among same-length-same-ILP,
  /// higher continuous register occupancy wins (tertiary tiebreak
  /// for stability / pressure-headroom preference). Used by
  /// DfsMinimizeLengthRefineIlpPolicy -- that policy opts into the
  /// bound relaxation that produces same-length completions for
  /// IsBetterThan to choose among.
  kMinimizeScheduleLengthThenMaximizeIlpScoreThenMaximizeContinuousOccupancyScore,

  /// Length-min with continuous register occupancy score as the
  /// same-length tiebreaker. Length asc primary; among same-length,
  /// higher continuous register occupancy wins (more pressure
  /// headroom). Used by DfsMinimizeLengthRefineOccupancyPolicy --
  /// that policy opts into the bound relaxation that produces
  /// same-length completions for GetScore to choose among. Plain
  /// kMinimizeScheduleLength does not produce same-length
  /// completions (its policy keeps the strict-improvement bound),
  /// so it doesn't need this refinement.
  kMinimizeScheduleLengthThenMaximizeContinuousOccupancyScore,

  /// Inverted register occupancy: lower GetRegisterOnlyOccupancy is
  /// "better." TEST-ONLY -- used to drive a search toward worse
  /// register occupancy so we can verify search infrastructure
  /// (DFS, etc.) actually explores and selects against the input
  /// baseline. Not a useful production metric.
  kMinimizeRegisterOccupancy,

  /// Inverted continuous register occupancy score: lower
  /// GetContinuousOccupancyScore is "better." TEST-ONLY, parallel
  /// to kMinimizeRegisterOccupancy but uses the smooth score, so
  /// schedules that differ in within-bracket pressure (not just
  /// integer occupancy) are distinguishable. Useful for verifying
  /// DFS picks WORSE schedules even when no integer-occupancy
  /// cliff is crossed.
  kMinimizeContinuousRegisterOccupancyScore,
};

/// Canonical comparable schedule score. See file header for the design.
class Score {
 public:
  /// Tag types: every Make() slot must be one of these. Raw ints won't
  /// bind (no ToCanonical overload for them), which forces the
  /// orientation question at the point of slot population and
  /// centralizes the negation in one private overload. int64_t to safely
  /// hold GetContinuousOccupancyArea sums on long schedules; narrower
  /// integers brace-init implicitly.
  struct Higher {
    int64_t v;
  };
  struct Lower {
    int64_t v;
  };

  /// Build a Score from tagged slots, listed in priority order
  /// (slot 0 = primary, slot 1 = first tiebreaker, ...). See file
  /// header / class doc for what the implementation is doing -- the
  /// one-liner expands to "for each tagged slot, call ToCanonical;
  /// drop the results into a std::array (trailing slots zero); hand
  /// that array to Score's private constructor."
  template <typename... Args>
  static Score Make(Args... slots) {
    static_assert(sizeof...(slots) <= kMaxSlots,
                  "Score has at most kMaxSlots slots");
    return Score{std::array<int64_t, kMaxSlots>{ToCanonical(slots)...}};
  }

  bool operator<(const Score &o) const { return values_ < o.values_; }
  bool operator>(const Score &o) const { return values_ > o.values_; }
  bool operator<=(const Score &o) const { return values_ <= o.values_; }
  bool operator>=(const Score &o) const { return values_ >= o.values_; }
  bool operator==(const Score &o) const { return values_ == o.values_; }
  bool operator!=(const Score &o) const { return values_ != o.values_; }

  /// Per-slot partial-order ("Pareto") dominance: true iff every slot
  /// of *this is >= the corresponding slot of `o`. Distinct from
  /// operator>= (lex). Pareto dominance is the correct check for
  /// multi-objective DP memo tables (PressureHistoryTracker,
  /// LengthHistoryTracker): a lex-collapse compare can prune a
  /// prefix that would have won at completion when the suffix
  /// equalizes a higher-priority slot. Pareto keeps incomparable
  /// entries instead.
  bool Dominates(const Score &o) const {
    for (int i = 0; i < kMaxSlots; ++i) {
      if (values_[i] < o.values_[i]) {
        return false;
      }
    }
    return true;
  }

  /// Inverse of Dominates: true iff `o` dominates `*this`. Provided
  /// for readability at call sites that naturally phrase the check
  /// as "am I dominated by this other entry."
  bool IsDominatedBy(const Score &o) const { return o.Dominates(*this); }

 private:
  // Capacity for primary + 2 tiebreakers. Grow if a metric ever needs more.
  static constexpr int kMaxSlots = 3;

  std::array<int64_t, kMaxSlots> values_;

  // Private: external code reaches Score only through Make(). Combined
  // with the private values_, this also disables aggregate-init -- so
  // Score{1, 2, 3} with raw ints does not compile.
  explicit Score(std::array<int64_t, kMaxSlots> v) : values_(v) {}

  // Overload resolution rejects raw ints / other types: only Higher /
  // Lower bind. Negation for Lower happens here, once, not at every
  // getter site.
  static int64_t ToCanonical(Higher h) { return h.v; }
  static int64_t ToCanonical(Lower l) { return -l.v; }
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCORE_H
