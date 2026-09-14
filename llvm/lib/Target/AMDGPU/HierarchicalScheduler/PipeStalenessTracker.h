//===- PipeStalenessTracker.h - per-pipe issue-spacing credit ---*- C++ -*-===//
//
// Tracks, during schedule construction, how long each HW issue pipe
// (HwPipeClass.h) has gone without an instruction, and accumulates
// "intermix credit": each scheduled instruction earns
//
//     min(staleness_cap, distance since its pipe last issued)
//
// summed over the whole schedule. Maximizing this credit drives
// every pipe's instructions toward uniform spacing: a pipe that
// clumps earns 1 per instruction instead of up to the cap, and a
// pipe hoarded for the end of the region wastes the credit its
// instructions could have harvested mid-region. The intent is
// co-issue friendliness for convoying waves — nearby program-order
// windows that span many pipes give the SIMD's per-category issue
// arbitration more to pick from (see HwPipeClass.h for the hardware
// model).
//
// Why the cap is load-bearing (not just a tuning knob): uncapped,
// each pipe's gap sum telescopes to (last occurrence position -
// first occurrence position) — the interior arrangement cancels out
// entirely and every schedule scores the same. The cap is what makes
// interior spacing matter, and it doubles as the saturation point
// past which further quietness of a pipe has no co-issue value.
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
//     The distortions this permits are negligible (a rare mid-region
//     s_barrier earns one nearly-always-saturated credit term), and
//     uniformity keeps the tracker branch-free.
//   - false: kOther instructions still occupy stream positions
//     (they bump the distance counter) but earn no credit and have
//     no staleness (queries report 0), so the measure ignores their
//     placement entirely.
//
// Properties consumers rely on:
//   - GetIntermixCredit is strictly monotone non-decreasing during
//     forward search (every credited instruction adds >= 1), so it
//     is a sound required-better Pareto axis for history dominance.
//   - The per-pipe capped-staleness snapshot is a sufficient
//     statistic for future credit: append the same completion Q to
//     two prefixes over the same scheduled set, and the only
//     per-instruction credits that can differ are each pipe's FIRST
//     instruction in Q. That instruction, at visible offset d into
//     Q, earns min(cap, d + staleness[p]) — a nondecreasing
//     function of staleness[p], and unchanged by capping the
//     snapshot (min(cap, d + s) = min(cap, d + min(s, cap)) for
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
#include <string>
#include <vector>

#include "HwPipeClass.h"
#include "ScheduleGraph.h"

namespace llvm {
namespace hierarchical_scheduler {

class PipeStalenessTracker {
 public:
  struct Options {
    // Saturation cap on per-instruction credit, in visible-
    // instruction positions. Roughly: the spacing beyond which a
    // pipe's continued quietness stops improving co-issue odds.
    int staleness_cap = 8;

    // How to handle the kOther pipe — see the file comment.
    bool track_other_pipe = true;
  };

  /// Precomputes each node's pipe index from the graph. Only real,
  /// MachineInstr-backed, IsPipeTrackingVisible nodes participate;
  /// proxies, sentinels, and invisible instructions are ignored by
  /// every mutator and query.
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
  /// completion.
  int GetIntermixCredit() const { return intermix_credit_; }

  /// Number of VISIBLE instructions scheduled so far — equivalently,
  /// the emitted-stream position the next visible instruction will
  /// occupy. Invisible instructions (meta ops, coalescable copies)
  /// do not move this.
  int GetVisibleInstructionsIssuedCount() const {
    return visible_instructions_issued_count_;
  }

  /// Capped staleness of `pipe` right now = the credit its next
  /// instruction would earn. 0 for kOther when track_other_pipe is
  /// off.
  int GetStaleness(HwPipe pipe) const;

  /// Credit that scheduling `node` NOW would add.
  ///
  /// Real visible instruction: min(cap, staleness of its pipe).
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
  ///
  /// Intended as the pass-2 ranking key and the pass-3 good-first
  /// descent order.
  int MarginalCreditForNode(const ScheduleNode *node) const;

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
    int credit;
  };

  /// True iff this pipe index participates in credit/staleness
  /// under the current options.
  bool IsCreditedPipeIndex(int pipe_index) const;

  /// The single source of truth for the credit formula: what an
  /// instruction on `pipe_index` would earn if issued at the current
  /// stream position = min(staleness_cap, distance since that pipe
  /// last issued). 0 for the no-pipe index and for uncredited pipes.
  /// Schedule banks exactly this value; GetStaleness and
  /// MarginalCreditForNode report it.
  int CreditForPipeIndex(int pipe_index) const;

  /// Pipe index of `node`, or -1 if it does not participate
  /// (proxy, sentinel, or invisible instruction).
  int PipeIndexForNode(const ScheduleNode *node) const;

  Options options_;

  /// Per-node pipe index (-1 = does not participate), indexed by
  /// topo index. Computed once at construction.
  std::vector<int> pipe_index_by_topo_index_;

  /// Stream position of each pipe's most recent issue. -1 = not yet
  /// issued in this region (the region-entry rendezvous convention).
  std::array<int, kNumHwPipes> last_issue_position_by_pipe_;

  /// Visible instructions scheduled so far.
  int visible_instructions_issued_count_ = 0;

  /// Total credit banked so far.
  int intermix_credit_ = 0;

  /// One record per Schedule call on a participating node; nothing
  /// is pushed for no-op schedules, and Unschedule mirrors the same
  /// participation test.
  std::vector<UndoRecord> undo_stack_;
};

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_PIPESTALENESSTRACKER_H
