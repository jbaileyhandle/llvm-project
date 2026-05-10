//===- IlpTracker.h - Producer-to-first-consumer ILP tracking ---*- C++ -*-===//
//
// Tracks how many real instructions are issued between each producer
// def and its FIRST consumer, summed (with per-op weights) across
// all such (producer, first-consumer) pairs in the schedule.
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
// Why per-op weights:
//   - The AMDGPU latency model is crude. A high-weight op (memory
//     load) benefits much more per cycle of hiding than a low-weight
//     op (int add). The contribution is multiplied by the producer's
//     weight so the search prefers to fill long-latency windows.
//
// Terminology:
//   A "producer" here means a (vreg, def-event) pair, NOT a defining
//   instruction. A single instruction defining K vregs contributes
//   K producers to the open map. The lower-bound formula sums over
//   open producers in this vreg sense.
//
// Live-outs and the exit sentinel:
//   The SUnit graph routes every live-out vreg through a synthetic
//   exit sentinel (no MachineInstr) as a use, which keeps liveness
//   honest. We honor that: even though the exit sentinel is not a
//   real instruction, its Schedule still CLOSES any open producer
//   among its uses. This means live-out producers naturally settle
//   at the exit-sentinel Schedule, with contribution
//   (instructions_issued_count_ - i_R - 1) * w_R captured in the
//   closed score. Open map is empty after a complete schedule.
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
//   the real spacing-based contribution, so by the time OpenDefs
//   sees the reg it's gone from the open map.) The dead-def
//   filter on NodeRegInfo (MachineOperand::isDead()) only catches
//   operands explicitly marked dead by upstream passes, not the
//   global "killed by re-def" pattern.
//
// Two scores, distinct properties:
//
//   GetClosedIlpScore() == closed_ilp_score_
//     Sum of contributions from producers whose first consumer
//     (real instruction OR exit sentinel) has already been
//     scheduled. Re-def implicit closes contribute 0; real closes
//     contribute w_R * (c - i_R - 1).
//     - Strictly monotone non-decreasing during forward search.
//       Closes (real or re-def) only ever ADD; nothing ever
//       subtracts.
//     - True lower bound on the eventual final ILP score (which
//       is GetClosedIlpScore at completion). The partial closed
//       score is a prefix sum of real future closes; the eventual
//       can only equal or exceed.
//
//   GetIlpScore() == closed_ilp_score_
//                  + Σ_{R ∈ open}  w_R * (c - i_R - 1)
//                  = closed_ilp_score_
//                  + (c - 1) * sum_open_weights_
//                  - sum_open_weighted_indices_
//     where c = instructions_issued_count_. Combines the closed
//     portion with the would-close-now value of every still-open
//     producer.
//     - At completion: equals GetClosedIlpScore (open map is
//       empty), which equals the actual final ILP score.
//     - During partial schedule: NOT strictly monotone — can DIP
//       at a re-def event. The prior open's pending contribution
//       (w * (c - i - 1)) is removed from pending; closed gains 0;
//       net effect on GetIlpScore is a drop of w * (c - i - 1).
//       Real-consumer closes shift contribution from pending to
//       closed at the same value — no dip.
//     - Therefore NOT a strict lower bound on the eventual final
//       score either, when re-defs occur. A producer that ends up
//       re-def-killed had pending value > 0 just before the
//       re-def, but its actual final contribution is 0.
//
// Use-site implications:
//   - Heuristic ranking: use GetIlpScore. Doesn't need bound
//     semantics; the dip at re-def is informative ("that pending
//     turned out to be wasted").
//   - Comparing COMPLETED schedules (IsBetterThan, ShouldEndSearch
//     completion watermark): either getter works — they coincide
//     at completion. Use GetIlpScore for the headline number.
//   - Length-history dominance with ILP as a required-better
//     Pareto axis: still safe with GetIlpScore — adding ANY
//     required dim only makes dominance harder, never easier,
//     regardless of strict-bound properties. If you specifically
//     want a true lower bound for some other purpose,
//     GetClosedIlpScore is the option to use.
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
  /// per-node op-type weight (weight_by_topo_index_), used as each
  /// producer's contribution multiplier. Weight is 0 for proxies
  /// and entry/exit sentinels — those nodes never open producers.
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

  /// Headline ILP score. closed_ilp_score_ + pending contributions
  /// from still-open producers. Lower bound on completion score
  /// while partial; exact final score once IsDone (open map is
  /// empty after the exit sentinel has closed all live-outs).
  /// O(1) — pending portion is computed from running sums.
  int GetIlpScore() const;

  /// The closed portion of the score in isolation — sum of
  /// contributions from producers whose first consumer has already
  /// been scheduled. Useful for telemetry / diagnostics; policies
  /// should compare via GetIlpScore.
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
  /// recently-issued producer. Read by the ranking heuristic to
  /// flag "would close an open producer" candidates.
  bool IsOpenProducer(unsigned reg) const {
    return open_producer_by_reg_.find(reg) != open_producer_by_reg_.end();
  }

  /// Op-type weight assigned to this node's defs at construction
  /// time. Read by the ranking heuristic to weigh "would close a
  /// heavy producer" worse than "would close a light producer".
  /// Returns 0 for proxies and entry/exit sentinels.
  int GetWeight(const ScheduleNode *node) const {
    return weight_by_topo_index_[node->GetTopoIndex()];
  }

  /// How many real instructions have been issued since the
  /// producer of `reg` was scheduled — i.e., the contribution this
  /// producer would lock in if `reg`'s first consumer were the
  /// next instruction (= c - i_R - 1, ignoring weight).
  /// Precondition: IsOpenProducer(reg).
  int GetSpacingForOpenProducer(unsigned reg) const {
    auto it = open_producer_by_reg_.find(reg);
    return instructions_issued_count_ - it->second.inst_count - 1;
  }

  /// Human-readable summary.
  std::string Describe() const;

 private:
  struct OpenProducer {
    int inst_count;  // value of instructions_issued_count_ at def time
    int weight;      // op-type weight (mirrored from weight_by_topo_index_)
  };

  struct UndoRecord {
    /// Producers this Schedule closed: (reg, prior inst_count, prior
    /// weight, contribution added to closed_ilp_score_). Re-insert
    /// the entry into the open map on Unschedule and subtract the
    /// contribution.
    struct ClosedEntry {
      // Reg keys are unsigned to match LLVM's vreg-number convention
      // and the existing DenseMap<unsigned, ...> shapes in
      // GCNRegisterTracker.
      unsigned reg;
      int prior_inst_count;
      int prior_weight;
      int contribution;
    };
    SmallVector<ClosedEntry, 2> closed;

    /// Producers this Schedule opened. On Unschedule: erase each
    /// from the open map. We don't record the (inst_count, weight)
    /// pair because these entries had no prior open value to
    /// restore — they go away entirely. Always empty for sentinel
    /// schedules.
    SmallVector<unsigned, 2> opened;
  };

  /// Read by the constructor for per-node defs/uses; not used
  /// thereafter.
  const GCNRegisterTracker *pressure_tracker_;

  /// Per-node weight: op-type-derived multiplier for each producer's
  /// contribution. Indexed by topo index. Real MachineInstr-backed
  /// nodes get the IlpWeightForOp value; everything else (proxies,
  /// entry/exit sentinels) gets 0.
  std::vector<int> weight_by_topo_index_;

  /// Real instructions scheduled so far. Bumped only by Schedule
  /// on a MachineInstr-backed scheduling-unit node; decremented on
  /// the matching Unschedule. Proxies and sentinels do not move it.
  int instructions_issued_count_ = 0;

  /// Sum of contributions from producers whose first consumer is
  /// already scheduled. Increases on close, decreases on un-close
  /// (Unschedule of the closing instruction or sentinel).
  int closed_ilp_score_ = 0;

  /// Open producers — vreg → (def's issue index, op-type weight).
  /// Inserted when a real instruction defines a vreg; erased when
  /// the first consumer (real instruction OR exit sentinel) is
  /// scheduled.
  DenseMap<unsigned, OpenProducer> open_producer_by_reg_;

  /// Σ w_R for R in open map. Maintained on insert/erase so
  /// GetIlpScore is O(1).
  int sum_open_weights_ = 0;

  /// Σ w_R * i_R for R in open map. Same purpose as
  /// sum_open_weights_; together they let GetIlpScore be O(1).
  int sum_open_weighted_indices_ = 0;

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
  /// add `contribution` to closed_ilp_score_, roll back the
  /// running sums, and erase from the open map. Precondition:
  /// `reg` IS in the open map. Caller supplies the contribution
  /// because the two close paths score it differently:
  ///   - Real first consumer (ProcessUsesAsCloses): contribution
  ///     = w_R * (instructions_issued_count_ - i_R - 1) — the
  ///     spacing actually used to hide latency.
  ///   - Re-def implicit close (OpenDefs): contribution = 0 —
  ///     the prior value was never consumed; nothing stalled on
  ///     it; no latency-hiding credit to give.
  void CloseOneProducer(unsigned reg, int contribution,
                        UndoRecord &undo);

  /// Close any open producers whose vreg appears in `node`'s uses.
  /// Each close uses the spacing-based contribution formula and
  /// delegates to CloseOneProducer.
  void ProcessUsesAsCloses(const ScheduleNode *node, UndoRecord &undo);

  /// Open a new producer for each of `node`'s defs at issue index
  /// instructions_issued_count_ with the node's op-type weight.
  /// If a vreg is already open (re-def — pre-RA MIR isn't strictly
  /// SSA: multi-def instructions, two-address tied uses, inline-asm,
  /// etc.), implicitly close the prior with contribution 0 first
  /// (the prior value isn't consumed; nothing stalled on it).
  /// A read-modify-write pattern (def is also a use) is handled by
  /// the call ordering in Schedule: ProcessUsesAsCloses runs first
  /// and closes the reg with the real contribution, so by the time
  /// OpenDefs sees it the reg is gone from the open map and the
  /// re-def path doesn't trigger.
  void OpenDefs(const ScheduleNode *node, UndoRecord &undo);

  /// Inverse of OpenDefs: erase each reg in `undo.opened` from
  /// the open map and roll back the running sums by the producer's
  /// stored (weight, inst_count).
  void EraseOpens(const UndoRecord &undo);

  /// Inverse of ProcessUsesAsCloses: re-insert each entry in
  /// `undo.closed` into the open map at its prior (inst_count,
  /// weight), roll back the running sums, and subtract each
  /// contribution from closed_ilp_score_. Fails loudly on
  /// reg-already-present (round-trip invariant).
  void ReinsertCloses(const UndoRecord &undo);
};

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_ILPTRACKER_H
