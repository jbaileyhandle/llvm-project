//===- MemoryGraph.cpp - Memory-op view of a region for MLP ---------------===//
//
// See MemoryGraph.h and llvm/docs/AMDGPUMemoryGraphDesign.md.
//
//===----------------------------------------------------------------------===//

#include "MemoryGraph.h"
#include "AMDGPU.h"
#include "GCNSubtarget.h"
#include "SIInstrInfo.h"
#include "Utils/AMDGPUBaseInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineMemOperand.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// Whether a FLAT op may touch global memory, and so counts in vmcnt.
// Local copy of SIInsertWaitcnts::mayAccessVMEMThroughFlat: an op with no
// memory info may touch anything, and only info saying LDS rules it out.
bool MayTouchGlobalMemory(const MachineInstr &mi) {
  if (mi.memoperands_empty()) {
    return true;
  }
  for (const MachineMemOperand *memop : mi.memoperands()) {
    if (memop->getAddrSpace() != AMDGPUAS::LOCAL_ADDRESS) {
      return true;
    }
  }
  return false;
}

// Whether a FLAT op may touch LDS, and so counts in lgkmcnt.
// Local copy of SIInsertWaitcnts::mayAccessLDSThroughFlat.
bool MayTouchLds(const MachineInstr &mi, const GCNSubtarget &st) {
  // Only FLAT ops whose instruction definition uses lgkmcnt can reach LDS.
  // global_* and scratch_* don't, so they never count in lgkmcnt.
  if (!SIInstrInfo::usesLGKM_CNT(mi)) {
    return false;
  }
  // In tgsplit mode a FLAT op can't reach LDS.
  if (st.isTgSplitEnabled()) {
    return false;
  }
  // No memory info: it may touch anything, LDS included.
  if (mi.memoperands_empty()) {
    return true;
  }
  // Info saying LDS, or the generic flat address space (which includes LDS).
  for (const MachineMemOperand *memop : mi.memoperands()) {
    unsigned address_space = memop->getAddrSpace();
    if (address_space == AMDGPUAS::LOCAL_ADDRESS ||
        address_space == AMDGPUAS::FLAT_ADDRESS) {
      return true;
    }
  }
  return false;
}

// Read or write. A read is an op whose result later code can wait for, which
// means it writes a register. This is the compiler's own test:
// SIInsertWaitcnts::updateByEvent records a counted op against every register
// it defines, implicit ones included, and only those registers are ever
// waited on. So an atomic that returns a value is a read and one that doesn't
// is a write, with no special case.
MemAccess AccessOf(const MachineInstr &mi) {
  for (const MachineOperand &operand : mi.operands()) {
    if (operand.isReg() && operand.isDef()) {
      return MemAccess::kRead;
    }
  }
  return MemAccess::kWrite;
}

} // namespace

// Mirrors SIInsertWaitcnts::updateEventWaitcntAfter, which decides which
// counters each instruction counts in: the same families, in the same order,
// with the same conditions. The comments below note where this differs.
// Branches of that function not handled here (s_sendmsg, exports, calls) are
// listed in the implementation plan's "Left out" table or don't occur in
// compute kernels.
std::optional<MemOpInfo>
llvm::hierarchical_scheduler::ClassifyMemOp(const MachineInstr &mi,
                                            const GCNSubtarget &st) {
  const SIInstrInfo &tii = *st.getInstrInfo();

  // DS: LDS and GDS ops (ds_read, ds_write, ds_add, ...). They count only in
  // lgkmcnt. GDS is told apart as the compiler does: an always-GDS opcode, or
  // the gds bit set. A DS op that doesn't use lgkmcnt matches none of the
  // compiler's other branches either, so it isn't counted.
  if (SIInstrInfo::isDS(mi)) {
    if (!SIInstrInfo::usesLGKM_CNT(mi)) {
      return std::nullopt;
    }
    const bool is_gds = tii.isAlwaysGDS(mi.getOpcode()) ||
                        tii.hasModifiersSet(mi, AMDGPU::OpName::gds);
    return MemOpInfo{AccessOf(mi), is_gds ? MemType::kGds : MemType::kLds,
                     /*counts_in_vmcnt=*/false, /*counts_in_lgkmcnt=*/true};
  }

  // FLAT encoding. The counters come from the two helpers above, exactly as
  // the compiler decides them. isFLAT is true for two kinds of instruction:
  //   - global_* and scratch_*: they share the FLAT instruction encoding, but
  //     can only touch global memory or scratch, never LDS, so the helpers give
  //     vmcnt only, like a buffer load. We label them vector ops;
  //     SIInsertWaitcnts::updateVMCntOnly also groups them with vector memory.
  //   - flat_*: may touch global memory or LDS, depending on its memory info.
  //     One that counts in both counters turns on cautious mode (design doc
  //     section 5.2).
  if (SIInstrInfo::isFLAT(mi)) {
    const MemType type =
        SIInstrInfo::isFLATGlobal(mi) || SIInstrInfo::isFLATScratch(mi)
            ? MemType::kVector
            : MemType::kFlat;
    return MemOpInfo{AccessOf(mi), type, MayTouchGlobalMemory(mi),
                     MayTouchLds(mi, st)};
  }

  // Vector memory other than FLAT: buffer and image ops (MUBUF, MTBUF, MIMG).
  // They count only in vmcnt. Buffer cache invalidates are excluded, as in the
  // compiler: they aren't loads or stores, and it doesn't count them.
  if (SIInstrInfo::isVMEM(mi) &&
      !AMDGPU::getMUBUFIsBufferInv(mi.getOpcode())) {
    return MemOpInfo{AccessOf(mi), MemType::kVector,
                     /*counts_in_vmcnt=*/true, /*counts_in_lgkmcnt=*/false};
  }

  // Scalar memory (s_load, s_buffer_load, ...). Counts only in lgkmcnt. This
  // family also includes s_memtime, a clock read that isn't a memory access;
  // it writes a register that later code waits on, so it's a read, as the
  // compiler treats it.
  if (SIInstrInfo::isSMRD(mi)) {
    return MemOpInfo{AccessOf(mi), MemType::kScalar,
                     /*counts_in_vmcnt=*/false, /*counts_in_lgkmcnt=*/true};
  }

  return std::nullopt;
}
