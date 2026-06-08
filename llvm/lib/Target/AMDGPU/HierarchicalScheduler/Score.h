//===- Score.h - Canonical comparable schedule score ----------*- C++ -*-===//
//
// Score is the canonical, single-type representation of "how good is this
// schedule under recipe R." Every recipe (integer occupancy, continuous
// occupancy + area, schedule length, length + ILP refinement, ...) produces
// a Score via ScheduleConstructor::GetScore(ScoreRecipe), and downstream
// code just uses < / > -- no branching on what's inside.
//
// Canonical orientation: higher = better. Score::Make() requires every slot
// to be tagged Score::Higher{x} or Score::Lower{x}; Lower values are
// negated internally so lex compare on the underlying array always means
// "left < right iff left is strictly worse than right."
//
// ScoreRecipe is the CONFIGURATION (which Dimensions to measure, in what
// priority order, with what polarity). Score is the OBSERVED VALUE (the
// canonical tuple produced from a recipe applied to a ScheduleConstructor).
// Both live here because they're tightly coupled and the header is small;
// kMaxScoreSlots is shared so growing the slot count is one edit.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCORE_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCORE_H

#include <array>
#include <cassert>
#include <cstdint>
#include <optional>

namespace llvm {
namespace hierarchical_scheduler {

/// Capacity shared by Score and ScoreRecipe (primary + up to 2
/// tiebreakers). One edit to grow the slot count.
inline constexpr int kMaxScoreSlots = 3;

/// A single measurable quantity that can fill a ScoreRecipe slot. Each
/// ScoreDimension maps to a concrete getter on one of the trackers; the
/// mapping lives in ScheduleConstructor::GetScoreDimensionValue. Adding
/// a new measurable is "new ScoreDimension entry + new case there."
enum class ScoreDimension {
  /// Integer register occupancy (GCNRegisterTracker::
  /// GetRegisterOnlyOccupancy). Peak-style.
  kRegisterOcc,

  /// Continuous register occupancy score (GCNRegisterTracker::
  /// GetContinuousOccupancyScore). Peak-style; smooth within brackets.
  kContinuousOccScore,

  /// Sum-along-path of per-step continuous occupancy scores
  /// (GCNRegisterTracker::GetContinuousOccupancyArea). Sum-style;
  /// higher area = pressure kept lower throughout.
  kContinuousOccArea,

  /// Schedule length in cycles (ScheduleLengthTracker::
  /// GetCurrentCycle). Sum-style.
  kScheduleLength,

  /// Locked-in ILP score (IlpTracker::GetIlpScore). Sum-style.
  kIlpScore,
};

/// Direction the raw value is "better." Score::Higher / Score::Lower
/// do the actual canonical negation when Make() is called; this enum
/// records the configured intent for the recipe.
enum class Polarity { kMaximize, kMinimize };

/// One slot in a ScoreRecipe.
struct MetricSlot {
  ScoreDimension dim;
  Polarity pol;

  constexpr bool operator==(const MetricSlot &o) const {
    return dim == o.dim && pol == o.pol;
  }

  /// True iff this slot's contribution to the canonical (higher = better)
  /// Score only stays the same or gets WORSE as the schedule extends
  /// toward completion. Determines which leading prefix of a
  /// ScoreRecipe is safe to include in a "completion cannot improve
  /// upon best" prune compare. Peak pressure only grows, so an
  /// occupancy-maximize slot only worsens. Cycles only grow, so a
  /// length-minimize slot only worsens. Area-maximize and ILP-maximize
  /// slots, in contrast, only get BETTER over completion (sum-style
  /// raw + maximize polarity), so they're not bound-safe.
  constexpr bool OnlyWorsensOverCompletion() const {
    switch (dim) {
      case ScoreDimension::kRegisterOcc:
      case ScoreDimension::kContinuousOccScore:
        // Peak-style raw: pressure grows over completion -> occupancy
        // waves / continuous score only DROP. Canonical NI iff
        // polarity is Maximize.
        return pol == Polarity::kMaximize;
      case ScoreDimension::kContinuousOccArea:
      case ScoreDimension::kScheduleLength:
      case ScoreDimension::kIlpScore:
        // Sum-style raw: only GROWS over completion. Canonical NI
        // iff polarity is Minimize.
        return pol == Polarity::kMinimize;
    }
    return false;  // unreachable
  }
};

/// A ScoreRecipe describes HOW to compute a Score: an ordered list of
/// at most kMaxScoreSlots MetricSlots (primary + tiebreakers). The
/// recipe is the configuration; Score is the observed value computed
/// from it.
///
/// A policy declares its objective by setting `kScoreRecipe` to a
/// ScoreRecipe value. The recommended go-forward pattern is to
/// CONSTRUCT THE RECIPE INLINE in the policy class, which avoids the
/// combinatorial explosion of having to name every meaningful
/// (dim, polarity) combination upfront -- a major motivation for
/// recipes in the first place. The `score_recipes::` namespace below
/// keeps named constants for the established legacy combinations so
/// existing policies can keep their declarations terse.
///
/// Slot 0 is the primary; slot 1 is the first tiebreaker; slot 2 is
/// the second tiebreaker. Trailing nullopt slots represent "unused"
/// the same way Score's trailing zeros do.
struct ScoreRecipe {
  std::array<std::optional<MetricSlot>, kMaxScoreSlots> slots;

  /// True iff the primary slot's dimension is kScheduleLength.
  /// Length-primary recipes drive the length pass (LHT-pruned);
  /// non-length-primary recipes drive the occupancy pass
  /// (PHT-pruned).
  constexpr bool IsLengthPrimary() const {
    return slots[0] && slots[0]->dim == ScoreDimension::kScheduleLength;
  }

  /// True iff length is primary AND polarity is kMaximize. The
  /// length-max search inverts the length-axis dominance direction
  /// in LengthHistoryTracker.
  constexpr bool IsLengthMaxMode() const {
    return IsLengthPrimary() && slots[0]->pol == Polarity::kMaximize;
  }

  /// True iff length is primary AND any tiebreak slot is present.
  /// Drives DfsSearch's per-Recurse bound relaxation: when true,
  /// max_acceptable is best.length (not best.length - 1) so same-
  /// length completions are produced for the tiebreak slots to
  /// choose among.
  constexpr bool RefinesAtSameLength() const {
    return IsLengthPrimary() && slots[1].has_value();
  }

  /// True iff any slot of the recipe carries ScoreDimension d.
  constexpr bool HasDim(ScoreDimension d) const {
    for (const auto &slot : slots) {
      if (slot && slot->dim == d) {
        return true;
      }
    }
    return false;
  }

  /// Count of populated slots in the recipe (leading prefix of
  /// non-nullopt slots).
  constexpr int NumSlots() const {
    int count = 0;
    for (const auto &slot : slots) {
      if (!slot) {
        break;
      }
      count++;
    }
    return count;
  }

  /// Count of leading slots whose OnlyWorsensOverCompletion() is true
  /// (stops at the first slot that doesn't satisfy it). This is the
  /// width of the prefix safe to compare lex in a "completion cannot
  /// improve upon best" prune: working's prefix score is an UPPER
  /// bound on its completion's prefix score, so working's prefix <=
  /// best's prefix on these leading slots implies working's completion
  /// can't beat best on lex compare. Including a non-worsening slot
  /// past this prefix would break that argument.
  constexpr int NumLeadingOnlyWorseningSlots() const {
    int count = 0;
    for (const auto &slot : slots) {
      if (!slot || !slot->OnlyWorsensOverCompletion()) {
        break;
      }
      count++;
    }
    return count;
  }

  /// True iff the recipe is exactly one slot AND that slot's dimension
  /// is a register-peak metric (kRegisterOcc or kContinuousOccScore).
  /// PartitionDag / BFS-DP uses this to gate its constructor: the
  /// bottleneck DP only supports recipes of this shape. Sum-style
  /// register dims (kContinuousOccArea) are register-derived but
  /// cumulative, so they don't fit and are excluded.
  constexpr bool IsRegisterPeakMetricOnly() const {
    if (!slots[0]) {
      return false;
    }
    if (slots[1] || slots[2]) {
      return false;
    }
    return slots[0]->dim == ScoreDimension::kRegisterOcc ||
           slots[0]->dim == ScoreDimension::kContinuousOccScore;
  }
};

/// LEGACY / backwards-compat ScoreRecipe constants. Each entry names a
/// specific (dim, polarity) combination that an existing policy used
/// before the recipe refactor; keeping them named lets those policies
/// keep their declarations terse without re-spelling the slots inline.
///
/// NEW POLICIES SHOULD NOT add named entries here. Construct the
/// recipe inline in the policy class instead -- that's the whole
/// point of the recipe abstraction. Each new (dim, polarity)
/// combination otherwise multiplies the named-constant count, and
/// the rest of the system already self-configures from any recipe
/// shape.
namespace score_recipes {

/// Integer register occupancy (peak). Coarse -- schedules in the same
/// occupancy bracket tie.
inline constexpr ScoreRecipe kMaximizeRegisterOccupancy{{
    MetricSlot{ScoreDimension::kRegisterOcc, Polarity::kMaximize},
}};

/// Continuous register occupancy score (peak). Smooth within brackets.
inline constexpr ScoreRecipe kMaximizeContinuousRegisterOccupancyScore{{
    MetricSlot{ScoreDimension::kContinuousOccScore, Polarity::kMaximize},
}};

/// Continuous occupancy (peak) primary, occupancy area as the same-
/// peak tiebreak.
inline constexpr ScoreRecipe
    kMaximizeContinuousOccupancyScoreThenMaximizeContinuousOccupancyArea{{
        MetricSlot{ScoreDimension::kContinuousOccScore, Polarity::kMaximize},
        MetricSlot{ScoreDimension::kContinuousOccArea, Polarity::kMaximize},
    }};

/// Schedule length only (length asc).
inline constexpr ScoreRecipe kMinimizeScheduleLength{{
    MetricSlot{ScoreDimension::kScheduleLength, Polarity::kMinimize},
}};

/// Schedule length only (length desc). Control / worst-legal-schedule
/// baseline.
inline constexpr ScoreRecipe kMaximizeScheduleLength{{
    MetricSlot{ScoreDimension::kScheduleLength, Polarity::kMaximize},
}};

/// Length-min, ILP score as the same-length tiebreak, continuous
/// occupancy as the tertiary tiebreak.
inline constexpr ScoreRecipe
    kMinimizeScheduleLengthThenMaximizeIlpScoreThenMaximizeContinuousOccupancyScore{
        {
            MetricSlot{ScoreDimension::kScheduleLength, Polarity::kMinimize},
            MetricSlot{ScoreDimension::kIlpScore, Polarity::kMaximize},
            MetricSlot{ScoreDimension::kContinuousOccScore, Polarity::kMaximize},
        }};

/// Length-min, continuous register occupancy as the same-length
/// tiebreak.
inline constexpr ScoreRecipe
    kMinimizeScheduleLengthThenMaximizeContinuousOccupancyScore{{
        MetricSlot{ScoreDimension::kScheduleLength, Polarity::kMinimize},
        MetricSlot{ScoreDimension::kContinuousOccScore, Polarity::kMaximize},
    }};

/// TEST-ONLY: inverted integer register occupancy. Drives DFS toward
/// WORSE occupancy so search infrastructure can be verified against an
/// input baseline.
inline constexpr ScoreRecipe kMinimizeRegisterOccupancy{{
    MetricSlot{ScoreDimension::kRegisterOcc, Polarity::kMinimize},
}};

/// TEST-ONLY: inverted continuous register occupancy. Parallel to
/// kMinimizeRegisterOccupancy but uses the smooth score.
inline constexpr ScoreRecipe kMinimizeContinuousRegisterOccupancyScore{{
    MetricSlot{ScoreDimension::kContinuousOccScore, Polarity::kMinimize},
}};

} // namespace score_recipes

/// Canonical comparable schedule score. See file header for the design.
class Score {
 public:
  /// One slot's input to Make: a raw value paired with the Polarity
  /// that says which direction is "better." The negation that
  /// canonicalizes "lower = better" to "higher = better" happens
  /// inside Score (via ApplyPolarity below) -- callers never apply
  /// it themselves. Construct via Higher / Lower (compile-time-known
  /// slots) or by copying polarity from a ScoreRecipe slot.
  struct SlotInput {
    int64_t raw_value;
    Polarity polarity;
  };

  /// Factory: a SlotInput tagged "higher = better."
  static constexpr SlotInput Higher(int64_t v) {
    return {v, Polarity::kMaximize};
  }

  /// Factory: a SlotInput tagged "lower = better."
  static constexpr SlotInput Lower(int64_t v) {
    return {v, Polarity::kMinimize};
  }

  /// Build a Score from a slot-aligned array of SlotInputs. Populated
  /// optionals become canonical slots of the resulting Score; nullopt
  /// slots stay 0 (canonical-neutral for lex / Pareto compare).
  /// ApplyPolarity is invoked per populated slot, so callers never
  /// apply orientation themselves.
  ///
  /// Recipe-driven callers (ScheduleConstructor::GetScore) build the
  /// array from the recipe's slots and the tracker values, then call
  /// this factory.
  static Score Make(
      std::array<std::optional<SlotInput>, kMaxScoreSlots> inputs) {
    std::array<int64_t, kMaxScoreSlots> canonical{};
    for (int i = 0; i < kMaxScoreSlots; ++i) {
      if (inputs[i]) {
        canonical[i] = ApplyPolarity(*inputs[i]);
      }
    }
    return Score{canonical};
  }

  /// Variadic convenience for compile-time-known slot lists:
  ///   Score::Make(Score::Higher(x), Score::Lower(y));
  /// Wraps each SlotInput in an optional and packs into the array
  /// form above. Missing trailing slots stay nullopt.
  template <typename... Args>
  static Score Make(Args... inputs) {
    static_assert(sizeof...(inputs) <= kMaxScoreSlots,
                  "Score has at most kMaxScoreSlots slots");
    std::array<std::optional<SlotInput>, kMaxScoreSlots> packed{
        std::optional<SlotInput>{inputs}...};
    return Make(packed);
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
    for (int i = 0; i < kMaxScoreSlots; ++i) {
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

  /// True iff `*this` is STRICTLY worse than `o` on a lex compare
  /// over the first `num_leading_slots` slots. "Worse" is in the
  /// canonical higher = better sense (lex-less in the raw int64_t
  /// array). The number of slots to compare is the caller's choice;
  /// score-bound-prune callers supply
  /// `ScoreRecipe::NumLeadingOnlyWorseningSlots()` so the prefix is
  /// restricted to slots that monotonically worsen toward completion,
  /// but the mechanism here doesn't enforce that.
  bool IsWorseOnLeadingSlots(const Score &o,
                             int num_leading_slots) const {
    for (int i = 0; i < num_leading_slots; ++i) {
      if (values_[i] < o.values_[i]) {
        return true;
      }
      if (values_[i] > o.values_[i]) {
        return false;
      }
    }
    return false;
  }

  /// Same as IsWorseOnLeadingSlots but non-strict: true iff `*this`
  /// is at-most-as-good as `o` on the prefix (lex-less-or-equal in
  /// canonical terms).
  bool IsAtMostAsGoodOnLeadingSlots(const Score &o,
                                    int num_leading_slots) const {
    for (int i = 0; i < num_leading_slots; ++i) {
      if (values_[i] < o.values_[i]) {
        return true;
      }
      if (values_[i] > o.values_[i]) {
        return false;
      }
    }
    return true;  // tied on the prefix counts as "at most as good"
  }

 private:
  std::array<int64_t, kMaxScoreSlots> values_;

  // Private: external code reaches Score only through Make(). Combined
  // with the private values_, this also disables aggregate-init -- so
  // Score{1, 2, 3} with raw ints does not compile.
  explicit Score(std::array<int64_t, kMaxScoreSlots> v) : values_(v) {}

  /// Single negation primitive. The Make factory routes every
  /// populated slot through here, so canonicalization lives in
  /// exactly one place.
  static constexpr int64_t ApplyPolarity(SlotInput s) {
    return s.polarity == Polarity::kMaximize ? s.raw_value : -s.raw_value;
  }
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCORE_H
