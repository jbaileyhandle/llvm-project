//===- IlpTracker.h - Producer-to-first-consumer ILP tracking ---*- C++ -*-===//
//
// Tracks how many real instructions are issued between each producer
// def and its FIRST consumer, summed (with per-op saturation caps)
// across all such (producer, first-consumer) pairs in the schedule.
//
// Why this metric and not "cycle distance":
//   - Bubbles (idle cycles) don't help ILP. Only ISSUED work overlaps
//     with a producer's latency window. So the unit is real
//     instructions, not cycles.
//   - Only the FIRST consumer can stall the hardware on a producer's
//     latency. Subsequent consumers see the result already in
//     register; deferring them buys no ILP benefit and would extend
//     register lifetime. So each producer contributes exactly once,
//     at the first-consumer event.
//
// Why per-op desirable-spacing (the saturation cap):
//   - The AMDGPU latency model is crude. A long-latency op (memory
//     load) needs more cycles of hiding to fully cover; a short-
//     latency op (int add) needs almost none. Beyond the op's
//     "desirable spacing" further hiding gains nothing — the
//     hardware-utilization story has saturated.
//   - We model this with one per-op number: desirable_spacing.
//     Contribution at close = min(actual spacing, desirable_spacing).
//     Heavy ops (long desirable_spacing) earn larger max
//     contributions and are correspondingly costlier to close
//     prematurely; light ops (short desirable_spacing) saturate
//     quickly and barely affect ranking decisions.
//   - Single knob per op: pain-per-cycle is treated as roughly
//     uniform across ops (a stalled cycle is a stalled cycle); only
//     the number of cycles needing cover varies. So
//     desirable_spacing alone captures the latency story without a
//     separate "weight" multiplier.
//
// Terminology:
//   A "producer" here means a (vreg, def-event) pair, NOT a defining
//   instruction. A single instruction defining K vregs contributes
//   K producers to the open map. Sums over open producers are over
//   that vreg-keyed set.
//
// Live-outs and the exit sentinel:
//   The SUnit graph routes every live-out vreg through a synthetic
//   exit sentinel (no MachineInstr) as a use, which keeps liveness
//   honest. We honor that: even though the exit sentinel is not a
//   real instruction, its Schedule still CLOSES any open producer
//   among its uses. This means live-out producers naturally settle
//   at the exit-sentinel Schedule, with the same saturated
//   contribution formula. Open map is empty after a complete
//   schedule.
//
// Lifecycle (mirrors the other trackers):
//   - Constructed alongside ScheduleLengthTracker / GCNRegisterTracker
//     in ScheduleConstructor.
//   - Schedule(node):
//       - Subgraph proxy: full no-op (no undo record pushed).
//       - Sentinel (scheduling unit but no MachineInstr — entry,
//         exit): close-only. Process uses to close any matching
//         open producers; do NOT bump instructions_issued_count_;
//         do NOT open any defs (they have none in NodeRegInfo).
//         Push an undo record (which records the closes).
//       - Real instruction (MachineInstr-backed): close uses, then
//         open defs, then bump instructions_issued_count_. Push an
//         undo record.
//   - Unschedule(node) reverses exactly via the back-of-stack undo
//     record (mirrors the same proxy / sentinel filtering).
//
// Re-def of an open vreg (NOT preceded by a use of the same reg
// on the same instruction):
//   Pre-RA Machine IR is mostly SSA but not strictly — multi-def
//   instructions, two-address tied uses, inline-asm with multiple
//   `=` constraints, and certain target intrinsics can produce a
//   second def of the same vreg with no intervening read. The
//   prior value is dead in that case; nothing stalled on it; no
//   latency-hiding credit is owed. OpenDefs implicitly closes the
//   prior open at contribution 0 before installing the new
//   producer. (Read-modify-write — def is also a use — is handled
//   by ordering: ProcessUsesAsCloses runs first and closes with
//   the real saturated contribution, so by the time OpenDefs sees
//   the reg it's gone from the open map.) The dead-def filter on
//   NodeRegInfo (MachineOperand::isDead()) only catches operands
//   explicitly marked dead by upstream passes, not the global
//   "killed by re-def" pattern.
//
// Two scores, distinct properties:
//
//   GetClosedIlpScore() == closed_ilp_score_
//     Sum of saturated contributions from producers whose first
//     consumer (real instruction OR exit sentinel) has already
//     been scheduled. Re-def implicit closes contribute 0; real
//     closes contribute min(spacing, desirable_spacing(op)).
//     - Strictly monotone non-decreasing during forward search.
//       Closes (real or re-def) only ever ADD; nothing ever
//       subtracts.
//     - True lower bound on the eventual final ILP score (which
//       is GetClosedIlpScore at completion). The partial closed
//       score is a prefix sum of real future closes; the eventual
//       can only equal or exceed.
//
//   GetIlpScore()  ≈  closed_ilp_score_  +  pending
//                  =  closed_ilp_score_
//                     +  open_count * (c - 1)
//                     -  sum_open_inst_indices_
//     where c = instructions_issued_count_. Combines the closed
//     portion with the unsaturated would-close-now value of every
//     still-open producer.
//     - At completion: equals GetClosedIlpScore (open map is
//       empty), so it is the actual final ILP score.
//     - During partial schedule: the pending portion is UNSATURATED
//       — it uses raw spacing rather than min(spacing,
//       desirable_spacing). For mature open producers (those
//       already at or past their desirable_spacing) it overestimates
//       by up to (spacing - desirable_spacing) per producer. We
//       accept this approximation to keep the formula O(1) without
//       maintaining per-tier (fresh-vs-mature) sums; the closed
//       portion captures the saturation correctly at close time.
//     - Also NOT strictly monotone — re-def events drop pending by
//       (c - i - 1) without adding anything to closed, so the
//       partial score can dip.
//
// Use-site implications:
//   - Heuristic ranking: use IlpTracker::CloseCostForNode (defined
//     here so the saturation policy lives in one place). The
//     ranking comparator wants the per-node "would close how much
//     latency-hiding credit if scheduled now," not the schedule-
//     wide score.
//   - Comparing COMPLETED schedules (IsBetterThan, ShouldEndSearch
//     completion watermark): either getter works — they coincide
//     at completion. Use GetIlpScore for the headline number.
//   - Length-history dominance with ILP as a required-better
//     Pareto axis: still safe with GetIlpScore — adding ANY
//     required dim only makes dominance harder, never easier,
//     regardless of strict-bound properties. If you specifically
//     want a true lower bound, GetClosedIlpScore is the option.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_ILPTRACKER_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_ILPTRACKER_H

#include "ScheduleGraph.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include <string>
#include <vector>

namespace llvm {

namespace hierarchical_scheduler {

class GCNRegisterTracker;

class IlpTracker {
 public:
  /// Construct from a graph and the already-built pressure tracker
  /// (read for per-node defs/uses via GetNodeRegInfo). Precomputes
  /// per-node desirable_spacing (desirable_spacing_by_topo_index_) —
  /// the saturation cap on each producer's contribution. Set to 0
  /// for proxies and entry/exit sentinels (those nodes never open
  /// producers).
  IlpTracker(const ScheduleGraph &graph,
             const GCNRegisterTracker &pressure_tracker);

  /// Schedule a node. Behavior depends on node kind:
  ///   - Proxy: full no-op (no undo record pushed).
  ///   - Sentinel (scheduling-unit but no MachineInstr): close any
  ///     open producers among its uses; no count bump, no opens.
  ///   - Real MachineInstr-backed instruction: close uses, then
  ///     open defs, then bump instructions_issued_count_.
  /// Sentinel and real-instruction schedules push an undo record.
  void Schedule(const ScheduleNode *node);

  /// Reverse the last Schedule via the back-of-stack undo record.
  /// Proxy: no-op (mirrors Schedule).
  void Unschedule(const ScheduleNode *node);

  /// Headline ILP score. closed_ilp_score_ + unsaturated pending
  /// contributions from still-open producers. Equals the actual
  /// final ILP score once IsDone (open map is empty after the
  /// exit sentinel has closed all live-outs). O(1).
  int GetIlpScore() const;

  /// The closed portion of the score in isolation — sum of
  /// saturated contributions from producers whose first consumer
  /// has already been scheduled. True lower bound on the eventual
  /// final score. Useful for diagnostics and for any consumer that
  /// needs strict-bound semantics.
  int GetClosedIlpScore() const { return closed_ilp_score_; }

  /// Number of real instructions issued so far. Excludes proxies
  /// AND entry/exit sentinels — only MachineInstr-backed nodes
  /// bump this counter.
  int GetInstructionsIssuedCount() const {
    return instructions_issued_count_;
  }

  /// Number of (vreg, def-event) pairs currently in the open map —
  /// distinct vregs whose def has been scheduled but whose first
  /// consumer has not. NOT a count of distinct producing
  /// instructions; a single instruction defining K vregs
  /// contributes K open producers.
  int GetOpenProducerVregCount() const {
    return static_cast<int>(open_producer_by_reg_.size());
  }

  /// True iff `reg` currently has an open producer — i.e., any
  /// unscheduled use of `reg` would be the first consumer of a
  /// recently-issued producer.
  bool IsOpenProducer(unsigned reg) const {
    return open_producer_by_reg_.find(reg) != open_producer_by_reg_.end();
  }

  /// Desirable spacing assigned to this node's defs at construction
  /// time — the saturation cap applied at close. Returns 0 for
  /// proxies and entry/exit sentinels (those nodes never open
  /// producers).
  int GetDesirableSpacing(const ScheduleNode *node) const {
    return desirable_spacing_by_topo_index_[node->GetTopoIndex()];
  }

  /// How many real instructions have been issued since the
  /// producer of `reg` was scheduled (= c - i_R - 1, raw / not
  /// saturated). Precondition: IsOpenProducer(reg).
  int GetSpacingForOpenProducer(unsigned reg) const {
    auto it = open_producer_by_reg_.find(reg);
    return instructions_issued_count_ - it->second.inst_count - 1;
  }

  /// Desirable spacing of the open producer for `reg`, or 0 if
  /// `reg` has no open producer. Single-call replacement for
  /// "is open?" + "what saturation cap?" — used by the close-cost
  /// helper. A return of 0 unambiguously means "no open producer
  /// for this reg" because the precomputed values for real
  /// instructions are all >= 1 (default bucket).
  int GetDesirableSpacingForOpenProducer(unsigned reg) const {
    auto it = open_producer_by_reg_.find(reg);
    return (it == open_producer_by_reg_.end())
               ? 0
               : it->second.desirable_spacing;
  }

  /// Cost of scheduling `node` NOW from an ILP perspective.
  ///
  /// Real MachineInstr-backed instruction: saturated freshness
  /// penalty summed over open producers `node` would close as their
  /// first consumer. Per-close cost =
  /// max(0, desirable_spacing_R - spacing_R). Symmetric with the
  /// close-side scoring formula (contribution = min(spacing_R,
  /// desirable_spacing_R)) — both live in this class so the
  /// saturation policy has one source of truth.
  ///
  /// Subgraph start proxy: scheduling the proxy commits to
  /// scheduling its members next (subgraph members are contiguous),
  /// so the proxy's cost is the cheapest opening move available —
  /// min over SubgraphInfo::initial_members of
  /// CloseCostForNode(member). Optimistic; assumes the search will
  /// take the best first move once inside.
  ///
  /// End proxies, entry/exit sentinels: 0 (no real-instruction
  /// semantics). End proxies don't normally reach this — the length
  /// policy short-circuits on them (sole-entry invariant) — but
  /// returning 0 keeps the contract simple if one ever does.
  int CloseCostForNode(const ScheduleNode *node) const;

  /// Human-readable summary.
  std::string Describe() const;

 private:
  struct OpenProducer {
    int inst_count;          // value of instructions_issued_count_ at def
    int desirable_spacing;   // saturation cap, mirrored from the by-topo table
  };

  struct UndoRecord {
    /// Producers this Schedule closed: (reg, prior inst_count,
    /// prior desirable_spacing, contribution added to
    /// closed_ilp_score_). Re-insert the entry into the open map
    /// on Unschedule and subtract the contribution.
    struct ClosedEntry {
      // Reg keys are unsigned to match LLVM's vreg-number convention
      // and the existing DenseMap<unsigned, ...> shapes in
      // GCNRegisterTracker.
      unsigned reg;
      int prior_inst_count;
      int prior_desirable_spacing;
      int contribution;
    };
    SmallVector<ClosedEntry, 2> closed;

    /// Producers this Schedule opened. On Unschedule: erase each
    /// from the open map. We don't record the (inst_count,
    /// desirable_spacing) pair because these entries had no prior
    /// open value to restore — they go away entirely. Always empty
    /// for sentinel schedules.
    SmallVector<unsigned, 2> opened;
  };

  /// Read by the constructor for per-node defs/uses, and by
  /// CloseCostForNode and the close/open helpers for per-node
  /// reg info.
  const GCNRegisterTracker *pressure_tracker_;

  /// Per-node desirable_spacing: op-type-derived saturation cap
  /// for each producer's contribution. Indexed by topo index.
  /// Real MachineInstr-backed nodes get IlpDesirableSpacingForOp;
  /// everything else (proxies, entry/exit sentinels) gets 0.
  std::vector<int> desirable_spacing_by_topo_index_;

  /// Real instructions scheduled so far. Bumped only by Schedule
  /// on a MachineInstr-backed scheduling-unit node; decremented on
  /// the matching Unschedule. Proxies and sentinels do not move it.
  int instructions_issued_count_ = 0;

  /// Sum of saturated contributions from producers whose first
  /// consumer is already scheduled. Increases on close, decreases
  /// on un-close (Unschedule of the closing instruction or
  /// sentinel).
  int closed_ilp_score_ = 0;

  /// Open producers — vreg → (def's issue index, op-type
  /// desirable_spacing). Inserted when a real instruction defines
  /// a vreg; erased when the first consumer (real instruction OR
  /// exit sentinel) is scheduled.
  DenseMap<unsigned, OpenProducer> open_producer_by_reg_;

  /// Σ i_R over R in the open map — the only state needed for
  /// the (unsaturated) pending portion of GetIlpScore:
  ///   pending = open_count * (c - 1) - sum_open_inst_indices_.
  /// Maintained on insert/erase so GetIlpScore is O(1).
  int sum_open_inst_indices_ = 0;

  /// One record per Schedule call on a scheduling-unit node (real
  /// MI or sentinel). Proxies push nothing.
  std::vector<UndoRecord> undo_stack_;

  // ----- Schedule / Unschedule helpers -----------------------------------
  //
  // Both Schedule helpers read instructions_issued_count_ directly;
  // Schedule bumps the counter AFTER both helpers run, so both see
  // the issue-position of the instruction being scheduled (= the
  // pre-bump value).

  /// Close one open producer: append a record to `undo.closed`,
  /// add `contribution` to closed_ilp_score_, roll back
  /// sum_open_inst_indices_, and erase from the open map.
  /// Precondition: `reg` IS in the open map. Caller supplies the
  /// contribution because the two close paths score it differently:
  ///   - Real first consumer (ProcessUsesAsCloses): contribution
  ///     = min(spacing_R, desirable_spacing_R) — saturated
  ///     spacing actually used to hide latency.
  ///   - Re-def implicit close (OpenDefs): contribution = 0 —
  ///     the prior value was never consumed; nothing stalled on
  ///     it; no latency-hiding credit to give.
  void CloseOneProducer(unsigned reg, int contribution,
                        UndoRecord &undo);

  /// Close any open producers whose vreg appears in `node`'s uses.
  /// Each close uses the saturated contribution formula and
  /// delegates to CloseOneProducer.
  void ProcessUsesAsCloses(const ScheduleNode *node, UndoRecord &undo);

  /// Open a new producer for each of `node`'s defs at issue index
  /// instructions_issued_count_ with the node's desirable_spacing.
  /// If a vreg is already open (re-def — pre-RA MIR isn't strictly
  /// SSA: multi-def instructions, two-address tied uses, inline-asm,
  /// etc.), implicitly close the prior with contribution 0 first
  /// (the prior value isn't consumed; nothing stalled on it).
  /// Read-modify-write (def is also a use) is handled by the call
  /// ordering in Schedule: ProcessUsesAsCloses runs first and
  /// closes the reg with the real contribution, so by the time
  /// OpenDefs sees it the reg is gone from the open map.
  void OpenDefs(const ScheduleNode *node, UndoRecord &undo);

  /// Inverse of OpenDefs: erase each reg in `undo.opened` from
  /// the open map and roll back sum_open_inst_indices_.
  void EraseOpens(const UndoRecord &undo);

  /// Inverse of ProcessUsesAsCloses: re-insert each entry in
  /// `undo.closed` into the open map at its prior (inst_count,
  /// desirable_spacing), roll back sum_open_inst_indices_, and
  /// subtract each contribution from closed_ilp_score_. Fails
  /// loudly on reg-already-present (round-trip invariant).
  void ReinsertCloses(const UndoRecord &undo);
};

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_ILPTRACKER_H
