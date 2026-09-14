//===- HwPipeClass.h - HW issue-pipe classification -------------*- C++ -*-===//
//
// Classifies a MachineInstr by the hardware issue pipe (instruction
// arbitration category) it routes to on GCN/Vega-class hardware.
//
// The hardware (GCN whitepaper / Vega ISA guide, CU front-end
// arbitration): each cycle, the waves of one SIMD are considered
// for issue, and up to 5 instructions are issued — at most one per
// wave and at most one per category — from SEVEN arbitration
// categories:
//
//   1. vector ALU
//   2. scalar ALU or scalar memory (one shared slot for both)
//   3. vector memory
//   4. LDS
//   5. branch/message
//   6. export/GDS
//   7. special/internal (s_nop, s_waitcnt, s_barrier, s_setprio, ...)
//
// Co-issue across waves therefore requires the waves' next
// instructions to sit in DIFFERENT categories — the fact the
// pipe-mixing measures built on this classification exploit.
//
// The enum tracks categories 1-4 individually and folds 5-7 into
// kOther. Categories 5-7 are real issue slots, but they have almost
// no schedulable population inside a compute scheduling region:
// branches are region terminators (outside the reordered window),
// exports don't occur in compute kernels, and waitcnts/nops are
// inserted post-RA. The one mid-region resident is s_barrier (the
// __syncthreads wave rendezvous — NOT a scheduling boundary, see
// SIInstrInfo::isSchedulingBoundary, but side-effect-chained
// against all memory ops, so nearly pinned).
//
// This is an issue-port taxonomy, NOT a latency taxonomy (see
// IlpTracker's desirable-spacing table for the latter; the two are
// intentionally independent).
//
// Visibility: some DAG nodes never become machine code. Meta
// instructions (IMPLICIT_DEF, KILL, debug ops) emit nothing, and
// copy-like pseudos (COPY, REG_SEQUENCE, subreg pseudos) are usually
// coalesced away at register allocation — with the exception of
// cross-bank COPYs (VGPR<->SGPR), which survive as vector moves.
// IsPipeTrackingVisible captures this: consumers should treat
// invisible instructions as absent from the final instruction
// stream (no pipe, no distance). Assignment is by majority outcome;
// the residual error (a surviving same-bank copy, an eliminated
// cross-bank one) is second-order for a capped spacing measure.
//
// The SIInstrInfo predicates this is built on are neither mutually
// exclusive nor aligned with the arbitration categories (isVMEM
// excludes FLAT; branches and s_barrier both satisfy isSALU), so
// ClassifyHwPipe imposes a total, disjoint classification by testing
// predicates in priority order — see the implementation.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_HWPIPECLASS_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_HWPIPECLASS_H

#include "llvm/ADT/StringRef.h"

namespace llvm {

class MachineInstr;

namespace hierarchical_scheduler {

/// One value per tracked arbitration category (hardware categories
/// 1-4 above), plus kOther for the untracked ones (5-7). Values are
/// dense from 0 so they can index fixed-size per-pipe arrays
/// (last-issue tables, staleness vectors).
enum class HwPipe : int {
  kValu = 0,  // vector ALU: VOP1/2/3/3P/C, SDWA, DPP; cross-bank COPYs
  kScalar,    // shared scalar slot: SOP* (non-branch) AND SMEM
  kVmem,      // vector memory: MUBUF/MTBUF/MIMG + FLAT (all segments)
  kLds,       // DS_*
  kOther,     // hardware categories 5-7 (branches, barriers, exports)
              // plus anything unrecognized (e.g. inline asm)
};

/// Number of HwPipe values — the dimension of per-pipe arrays.
inline constexpr int kNumHwPipes = 5;

/// True iff `mi` plausibly exists in the final instruction stream.
/// False for meta instructions (emit no code) and for copy-like
/// pseudos expected to be coalesced away at RA. Cross-bank COPYs
/// return true (they survive as vector moves). Instructions that are
/// not visible should be excluded from pipe tracking entirely:
/// they neither occupy a pipe nor add distance between neighbors.
bool IsPipeTrackingVisible(const MachineInstr &mi);

/// Classify `mi` into the issue pipe it routes to. Total function —
/// every MachineInstr gets a value — but only meaningful for
/// instructions where IsPipeTrackingVisible is true.
HwPipe ClassifyHwPipe(const MachineInstr &mi);

/// Short lowercase name for dumps/telemetry (e.g. "valu", "vmem").
StringRef HwPipeName(HwPipe pipe);

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_HWPIPECLASS_H
