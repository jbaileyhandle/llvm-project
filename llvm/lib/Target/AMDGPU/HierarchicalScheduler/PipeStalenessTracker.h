//===- PipeStalenessTracker.h - per-pipe issue-spacing credit ---*- C++ -*-===//
//
// Tracks, during schedule construction, how long each HW issue pipe
// (HwPipeClass.h) has gone without an instruction, and accumulates
// "intermix credit" — a score rewarding schedules that keep every
// pipe's instructions spread through the region rather than clumped.
// The intent is co-issue friendliness for convoying waves: the SIMD
// issues at most one instruction per arbitration category per cycle,
// each from a different wave, so nearby program-order windows that
// span many pipes give the issue arbiter more to pick from.
//
// ============================ THE SCORE =============================
//
// One fixed-point constant serves the whole scheme:
//
//   kFixedPointScale = 4096
//     - the maximum credit ONE instruction can earn (identical for
//       every pipe), and
//     - the scale of the ranking key ("due now" == 4096).
//     See the constant's declaration for why 4096 exactly.
//
// Knobs (Options):
//
//   spacing_ceiling (default 16, legal range [1, kMaxSpacingCeiling])
//     Cap on per-pipe target spacing — the gap beyond which we model
//     NO additional co-issue benefit.
//
//   curve (default kLinear)   Shape of the sub-saturation ramp.
//   track_other_pipe (default true)   See BOOKKEEPING RULES.
//
// Per region, derived once at construction:
//
//   N   = number of VISIBLE instructions in the region
//   n_p = number of visible instructions on pipe p
//
//   s_p = clamp(N / n_p, 1, spacing_ceiling)     "target spacing"
//
//   N/n_p is the gap pipe p's instructions would have if spread
//   perfectly evenly, so a pipe saturates its per-instruction credit
//   only at even-spread spacing. Dense pipes saturate at tiny gaps
//   (a majority pipe has s_p = 1 and is always saturated); rare
//   pipes must spread out to earn full credit.
//
//   Per-pipe credit table over capped staleness g in [0, s_p]:
//
//     kLinear:  table_p[g] = round(kFixedPointScale * g / s_p)
//     kSqrt:    table_p[g] = round(kFixedPointScale * sqrt(g / s_p))
//
//   Both run from table_p[0] = 0 to table_p[s_p] = kFixedPointScale.
//   Pipes differ in WHERE they saturate (s_p), never in HOW MUCH a
//   saturated instruction earns:
//
//     credit
//   4096 -|- - - - - - - - -o________________
//         |             _.-':
//         |          .-' .' :   o = saturation (gap = s_p);
//         |        .'   /   :       every gap past s_p pays
//         |      .' sqrt    :       the same 4096
//         |     / .-'       :
//         |    /.'  linear  :
//         |   /'            :
//      0 -+--'--------------+------------------ gap since pipe
//         0                s_p                   last issued
//
// Scoring a schedule: walk the instructions in order, counting only
// visible ones (position t). For an instruction on pipe p whose
// previous same-pipe issue was at position l (l = -1 at region
// entry):
//
//     gap    = t - l
//     credit = table_p[ min(gap, s_p) ]
//
// Total intermix credit = the sum of these, held in an int64_t
// (credit sums never live in plain int). An instruction earns the
// full 4096 exactly when its gap meets its pipe's target spacing; a
// curve-shaped fraction otherwise. Maximum possible total =
// kFixedPointScale * N, giving the branch-and-bound bound:
// remaining credit <= kFixedPointScale * remaining visible
// instructions.
//
// Example — N = 100, pipe S with n_S = 5 (s_S = min(100/5, 16) = 16),
// linear curve:
//
//   exhausted early:               spread evenly:
//   S..S..S..S..S..............    S...........S...........S......
//   gaps ~3 -> 768 credit each     gaps ~20 (>= 16) -> 4096 each
//
// ====================== WHY THIS SHAPE ==============================
//
// Per-pipe prevalence-derived spacing (s_p = N/n_p): with one
// uniform saturation gap instead, a rare pipe earns full credit at
// that uniform spacing, so "all its instructions used up early" ties
// with "spread across the whole region" — the measure cannot see
// tail starvation. Deriving s_p from prevalence breaks the tie: a
// pipe's gaps sum to ~N, so its total credit peaks only when every
// gap is ~s_p, i.e. at even spread.
//
// Uniform per-instruction maximum (the same 4096 for every pipe):
// the most a pipe can contribute to co-issue is hiding ALL its
// instructions behind other pipes' slots — a time saving bounded by
// its stream fraction n_p/N (Amdahl). One hidden slot is worth one
// hidden slot regardless of which pipe it belongs to, so per-
// instruction value is uniform and pipe p's total stake is
// n_p * kFixedPointScale, proportional to prevalence.
//
// Why the ceiling (spacing_ceiling): past some quietness a pipe's
// continued absence buys no further co-issue benefit, so crediting
// gap growth beyond that window is a gradient toward nothing —
// without the ceiling, a 2-instruction pipe in a 300-instruction
// region ramps credit over gaps 0..150 and the search is paid,
// continuously, to drag those instructions toward the region edges
// (a singleton pipe degenerates fully: its credit would equal its
// position, maximized at the region's end). The ceiling replaces
// the ramp past spacing_ceiling with indifference, which is the
// true hardware shape. It also keeps the staleness boundary state
// coarse (capped at s_p <= ceiling), which future history-dominance
// pruning benefits from.
//
// Why a curve option: dependences often make target spacing
// unreachable, and below saturation the linear curve is
// redistribution-neutral (gaps 5+15 tie with 10+10 when both stay
// under s_p). The sqrt curve is concave, so it strictly prefers the
// even split. Anchoring both curves at table[s_p] = kFixedPointScale
// means switching curves changes only the sub-saturation ramp, never
// the cross-pipe stakes. Any monotone non-decreasing curve preserves
// every soundness property below.
//
// ================== RANKING vs SCORING KEYS =========================
//
// Two per-node keys, both built on the SAME prevalence-normalized
// quantity gap/s_p — they differ only in whether the saturation cap
// applies:
//
//   MarginalCreditForNode — table_p[min(gap, s_p)]. The SCORING key:
//   what scheduling the node now adds to the banked score. Capped,
//   because past target spacing more quietness buys no hardware
//   benefit.
//
//   RankKeyForNode — gap/s_p UNCAPPED, in the same fixed point:
//   raw gap times the pipe's precomputed weight
//   kFixedPointScale/s_p, so ~kFixedPointScale means "due now" and
//   larger means "overdue". The ready-list ORDERING key.
//
// Below saturation the two keys order candidates identically. The
// cap must come off for ORDERING because of a procrastination
// failure at saturation: a majority pipe (s_p = 1) sits at credit
// 4096 permanently, and once a rare pipe saturates it TIES at 4096
// — and stays tied, so a credit-ranked greedy can defer it
// indefinitely on tie-breaks. Deferral is not free even though the
// deferred instruction still earns 4096 whenever it lands: the
// pipe's NEXT instruction's gap clock only starts at that issue, so
// procrastinating early instructions compresses the pipe's
// remaining spacing against the region end — the exact tail
// squeeze the measure exists to prevent. Score-flatness past
// saturation (correct: quietness stops paying) is not indifference
// about WHEN to bank a saturated credit. Uncapped overdueness keeps
// rising past due, so due pipes get issued near their target
// interval. A greedy descent ranked this way services each pipe at
// its proportional rate — the stride / deficit-round-robin pattern
// from proportional-share scheduling (Waldspurger & Weihl, OSDI
// 1995; Shreedhar & Varghese, SIGCOMM 1995): serve each client at
// its share by always picking the most-overdue.
//
// ====================== BOOKKEEPING RULES ===========================
//
// Units are VISIBLE-instruction positions, not model cycles:
// convoying waves fetch the emitted instruction stream in program
// order, so distance in that stream is the relevant window measure;
// model bubbles are not real stream positions (same reasoning as
// IlpTracker's real-instructions-not-cycles choice). Instructions
// for which IsPipeTrackingVisible is false (meta ops, coalescable
// copies) do not exist in the emitted stream and are fully ignored:
// no position bump, no credit, no staleness effect.
//
// Region-start boundary convention: every pipe starts as if it had
// just issued at position -1 (staleness grows from the region
// entry). This treats the region entry as a rendezvous point rather
// than granting free saturated credit to each pipe's first
// instruction.
//
// The kOther pipe (hardware's branch/export/internal categories +
// unrecognized opcodes) is handled per Options::track_other_pipe:
//   - true (default): tracked uniformly like the four real pipes.
//   - false: kOther instructions still occupy stream positions
//     (they bump the distance counter) but earn no credit and have
//     no staleness (queries report 0).
//
// ================== PROPERTIES CONSUMERS RELY ON ====================
//
//   - GetIntermixCredit is strictly monotone non-decreasing during
//     forward search: every credited instruction adds >= 1, because
//     table[g] >= 1 for g >= 1 (round of kFixedPointScale/s_p with
//     s_p <= kMaxSpacingCeiling << kFixedPointScale). Sound
//     required-better Pareto axis for history dominance.
//   - The per-pipe capped-staleness snapshot is a sufficient
//     statistic for future credit: append the same completion Q to
//     two prefixes over the same scheduled set, and the only
//     per-instruction credits that can differ are each pipe's FIRST
//     instruction in Q. That instruction, at visible offset d into
//     Q, earns table_p[min(d + staleness[p], s_p)] — nondecreasing
//     in staleness[p] (tables are monotone), and unchanged by
//     capping the snapshot at s_p (the argument is capped anyway for
//     d >= 0). Hence credit_A >= credit_B AND staleness_A >=
//     staleness_B elementwise imply credit(A+Q) >= credit(B+Q) for
//     EVERY completion Q — the soundness argument for using both as
//     history-dominance dimensions alongside the length dims.
//
// Lifecycle mirrors the other trackers (Schedule/Unschedule with an
// undo stack; subgraph proxies are full no-ops).
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_PIPESTALENESSTRACKER_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_PIPESTALENESSTRACKER_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "HwPipeClass.h"
#include "ScheduleGraph.h"

namespace llvm {
namespace hierarchical_scheduler {

class PipeStalenessTracker {
 public:
  /// Shape of the credit-vs-capped-staleness ramp. Both are monotone
  /// non-decreasing with table[g] >= 1 for g >= 1, so all soundness
  /// properties hold under either.
  enum class CreditCurve : int {
    kLinear = 0,  // credit proportional to capped staleness
    kSqrt,        // concave; prefers even sub-saturation splits
  };

  /// The one fixed-point scale of the scheme: maximum per-
  /// instruction credit (earned at target spacing, any pipe) AND the
  /// "due now" value of the ranking key.
  ///
  /// Why 4096: the ranking weight round(kFixedPointScale / s_p) must
  /// give DISTINCT weights to adjacent target spacings, or pipes
  /// with different prevalence rank identically. The exact values
  /// Scale/s and Scale/(s+1) differ by Scale/(s*(s+1)) ~ Scale/s^2,
  /// and integer rounding keeps them apart only if that difference
  /// is >= 1 — i.e. Scale >= s^2. With spacing legal up to
  /// kMaxSpacingCeiling = 64, the smallest sufficient scale is
  /// 64^2 = 4096. (At a scale of 64, every spacing in 33..64 would
  /// collapse to weight 1.) The credit table's own fidelity bound
  /// (distinct credits for adjacent gaps: Scale >= 2 * s_p for the
  /// sqrt ramp) is weaker, so one constant serves both.
  static constexpr int kFixedPointScale = 4096;

  /// Largest legal Options::spacing_ceiling. 64 is where a 4096
  /// scale stops satisfying Scale >= ceiling^2 (see above).
  static constexpr int kMaxSpacingCeiling = 64;

  static_assert(kFixedPointScale >= kMaxSpacingCeiling * kMaxSpacingCeiling,
                "rank weights must distinguish every legal spacing");

  struct Options {
    // Cap on per-pipe target spacing, in visible-instruction
    // positions: s_p = clamp(N/n_p, 1, spacing_ceiling). Must be in
    // [1, kMaxSpacingCeiling] (checked fatally at construction).
    int spacing_ceiling = 16;

    // How to handle the kOther pipe — see the file comment.
    bool track_other_pipe = true;

    // Sub-saturation ramp shape — see CreditCurve.
    CreditCurve curve = CreditCurve::kLinear;
  };

  /// Precomputes each node's pipe index, the per-pipe visible
  /// counts, target spacings, credit tables, and rank weights.
  /// Only real, MachineInstr-backed, IsPipeTrackingVisible nodes
  /// participate; proxies, sentinels, and invisible instructions
  /// are ignored by every mutator and query.
  PipeStalenessTracker(const ScheduleGraph &graph, const Options &options);

  /// Account for `node` being scheduled next. Participating nodes
  /// earn credit, update their pipe's last-issue position, and bump
  /// the stream position; everything else is a full no-op.
  void Schedule(const ScheduleNode *node);

  /// Reverse the matching Schedule via the back-of-stack undo
  /// record.
  void Unschedule(const ScheduleNode *node);

  /// Total intermix credit banked so far. Strictly monotone
  /// non-decreasing during forward search; final schedule score at
  /// completion. int64_t: credit sums never live in plain int.
  int64_t GetIntermixCredit() const { return intermix_credit_; }

  /// Number of VISIBLE instructions scheduled so far — equivalently,
  /// the emitted-stream position the next visible instruction will
  /// occupy. Invisible instructions (meta ops, coalescable copies)
  /// do not move this.
  int GetVisibleInstructionsIssuedCount() const {
    return visible_instructions_issued_count_;
  }

  /// Upper bound on the final intermix credit of ANY completion of
  /// the current prefix: banked credit + kFixedPointScale * remaining
  /// visible instructions (every remaining instruction earning its
  /// maximum). Sound: never underestimates. At an empty prefix this
  /// is the perfect score kFixedPointScale * N (a schedule reaching
  /// it ends the search); at completion it equals GetIntermixCredit
  /// exactly. Intended as the search's branch-and-bound prune:
  /// bound <= best completed credit => this prefix cannot win.
  int64_t GetFinalCreditUpperBound() const {
    return intermix_credit_ +
           static_cast<int64_t>(kFixedPointScale) *
               (total_visible_count_ - visible_instructions_issued_count_);
  }

  /// Target spacing s_p = clamp(N/n_p, 1, spacing_ceiling) for
  /// `pipe` in this region. For a pipe with no visible instructions
  /// (n_p == 0) this reports the ceiling; such a pipe never issues,
  /// so the value only feeds staleness reporting.
  int GetDesirableSpacing(HwPipe pipe) const {
    return desirable_spacing_by_pipe_[static_cast<int>(pipe)];
  }

  /// Capped staleness of `pipe` right now: min(distance since last
  /// issue, s_p). This is STATE, not credit — feed it through the
  /// pipe's table (CreditForPipe) to get the credit its next
  /// instruction would earn. 0 for kOther when track_other_pipe is
  /// off.
  int GetStaleness(HwPipe pipe) const;

  /// Credit an instruction on `pipe` would earn if issued now:
  /// table_p[GetStaleness(pipe)].
  int CreditForPipe(HwPipe pipe) const {
    return CreditForPipeIndex(static_cast<int>(pipe));
  }

  /// Credit that scheduling `node` NOW would add — the SCORING key
  /// (see RANKING vs SCORING in the file comment).
  ///
  /// Real visible instruction: CreditForPipe of its pipe.
  ///
  /// Subgraph start proxy: scheduling the proxy is a commitment to
  /// schedule the subgraph's members next, so the instruction that
  /// would actually issue next is one of the subgraph's initial
  /// members. We report the best case — the largest credit any
  /// initial member would earn — assuming the search picks that
  /// member first. (Same proxy treatment as
  /// IlpTracker::CloseCostForNode, except that one reports the
  /// SMALLEST member value because it measures a penalty, and this
  /// one the largest because it measures a reward.)
  ///
  /// Everything else (sentinels, invisible instructions): 0.
  int MarginalCreditForNode(const ScheduleNode *node) const;

  /// Overdueness of `node`'s pipe — the RANKING key for ready-list
  /// ordering: the same prevalence-normalized staleness the credit
  /// tables are built on, WITHOUT the saturation cap (see RANKING
  /// vs SCORING in the file comment for the procrastination failure
  /// the cap would cause). Fixed-point: raw gap times the pipe's
  /// precomputed weight kFixedPointScale/s_p. int64_t: raw gap
  /// scales with region size. Proxy/other handling mirrors
  /// MarginalCreditForNode (best case over initial members; 0 for
  /// non-participants).
  int64_t RankKeyForNode(const ScheduleNode *node) const;

  /// Per-pipe capped staleness — the boundary state for history
  /// dominance (see file comment). Entry order follows the HwPipe
  /// enum.
  std::array<int, kNumHwPipes> GetStalenessSnapshot() const;

  /// Human-readable summary.
  std::string Describe() const;

 private:
  struct UndoRecord {
    int pipe_index;
    int prior_last_issue_position;
    int credit;  // per-instruction, <= kFixedPointScale — int is safe
  };

  // ----- Construction helpers --------------------------------------------

  /// Fatal-checks the construction preconditions: spacing_ceiling in
  /// [1, kMaxSpacingCeiling] and a topo-sorted graph.
  void ValidateConstructionPreconditions(const ScheduleGraph &graph) const;

  /// Pass 1: classify every visible instruction into
  /// pipe_index_by_topo_index_ and return the per-pipe visible
  /// counts (prevalence).
  std::array<int, kNumHwPipes> ClassifyVisibleInstructions(
      const ScheduleGraph &graph);

  /// Pass 2: from the per-pipe counts, derive target spacings,
  /// credit tables, and rank weights.
  void DerivePerPipeScoring(
      const std::array<int, kNumHwPipes> &visible_count_by_pipe);

  // ----- Query helpers ----------------------------------------------------

  /// True iff this pipe index participates in credit/staleness
  /// under the current options.
  bool IsCreditedPipeIndex(int pipe_index) const;

  /// Pipe index of `node`, or -1 if it does not participate
  /// (proxy, sentinel, or invisible instruction).
  int PipeIndexForNode(const ScheduleNode *node) const;

  /// Capped staleness for a pipe index; 0 for the no-pipe index and
  /// for uncredited pipes.
  int StalenessForPipeIndex(int pipe_index) const;

  /// The single source of truth for the credit formula:
  /// table[StalenessForPipeIndex(pipe_index)]. Schedule banks
  /// exactly this value; the credit queries report it.
  int CreditForPipeIndex(int pipe_index) const;

  Options options_;

  /// Per-node pipe index (-1 = does not participate), indexed by
  /// topo index. Computed once at construction.
  std::vector<int> pipe_index_by_topo_index_;

  /// s_p per pipe — see GetDesirableSpacing.
  std::array<int, kNumHwPipes> desirable_spacing_by_pipe_;

  /// Per-pipe credit table indexed by capped staleness g in
  /// [0, s_p]; table[0] = 0, monotone non-decreasing, table[g] >= 1
  /// for g >= 1, table[s_p] = kFixedPointScale for every pipe.
  std::array<std::vector<int>, kNumHwPipes> credit_table_by_pipe_;

  /// Per-pipe fixed-point overdueness weight,
  /// kFixedPointScale / s_p — precomputed so RankKeyForNode is a
  /// single multiply.
  std::array<int, kNumHwPipes> rank_weight_by_pipe_;

  /// Stream position of each pipe's most recent issue. -1 = not yet
  /// issued in this region (the region-entry rendezvous convention).
  std::array<int, kNumHwPipes> last_issue_position_by_pipe_;

  /// Total visible instructions in the region (N) — fixed at
  /// construction; feeds GetFinalCreditUpperBound.
  int total_visible_count_ = 0;

  /// Visible instructions scheduled so far.
  int visible_instructions_issued_count_ = 0;

  /// Total credit banked so far (sum of per-instruction credits —
  /// int64_t, never plain int).
  int64_t intermix_credit_ = 0;

  /// One record per Schedule call on a participating node; nothing
  /// is pushed for no-op schedules, and Unschedule mirrors the same
  /// participation test.
  std::vector<UndoRecord> undo_stack_;
};

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_PIPESTALENESSTRACKER_H
