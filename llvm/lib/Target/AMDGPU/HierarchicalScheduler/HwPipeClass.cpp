//===- HwPipeClass.cpp - HW issue-pipe classification ---------------------===//
//
// Implementation of issue-pipe classification.
//
// See HwPipeClass.h for the hardware model and design rationale.
//
//===----------------------------------------------------------------------===//

#include "HwPipeClass.h"
#include "SIInstrInfo.h"
#include "SIRegisterInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// True iff `mi` is a COPY whose source and destination live in
// different register banks (SGPR vs VGPR). Cross-bank copies survive
// register allocation as real vector moves (v_mov_b32 for SGPR->VGPR,
// v_readfirstlane_b32 for VGPR->SGPR — both VALU); same-bank copies
// are the coalescer's bread and butter and usually vanish.
//
// Precondition (enforced): mi is TargetOpcode::COPY, whose canonical
// form fixes operand 0 = destination register, operand 1 = source
// register. isSGPRReg resolves both virtual registers (via the MRI's
// register class) and physical ones (via the phys-reg base class),
// so pre-RA copies from physical SGPRs (kernel arguments) classify
// correctly.
bool IsCrossBankCopy(const MachineInstr &mi) {
  if (!mi.isCopy()) {
    report_fatal_error("IsCrossBankCopy requires a COPY instruction");
  }
  const MachineFunction *machine_function = mi.getMF();
  const SIRegisterInfo *register_info = static_cast<const SIRegisterInfo *>(
      machine_function->getSubtarget().getRegisterInfo());
  const MachineRegisterInfo &machine_register_info =
      machine_function->getRegInfo();
  const bool dest_is_sgpr =
      register_info->isSGPRReg(machine_register_info, mi.getOperand(0).getReg());
  const bool source_is_sgpr =
      register_info->isSGPRReg(machine_register_info, mi.getOperand(1).getReg());
  return dest_is_sgpr != source_is_sgpr;
}

}  // namespace

namespace llvm {
namespace hierarchical_scheduler {

bool IsPipeTrackingVisible(const MachineInstr &mi) {
  // isTransient = meta instructions (emit no machine code at all)
  // plus copy-like pseudos (COPY, REG_SEQUENCE, INSERT_SUBREG,
  // SUBREG_TO_REG — usually coalesced away at RA). The one transient
  // kind that usually DOES survive is the cross-bank COPY, which
  // needs a real vector move.
  if (!mi.isTransient()) {
    return true;
  }
  if (mi.isCopy()) {
    return IsCrossBankCopy(mi);
  }
  return false;
}

HwPipe ClassifyHwPipe(const MachineInstr &mi) {
  // Priority order matters — see the header comment. Broadly:
  // resolve pseudos first, then peel off the kOther hardware
  // categories that would otherwise be swallowed by isSALU (branches
  // and specials are SOPP-encoded), then the four tracked pipes.

  // COPY: only cross-bank copies are visible, and those lower to
  // VALU moves in either direction (v_mov / v_readfirstlane).
  if (mi.isCopy()) {
    return HwPipe::kValu;
  }

  // Hardware category 5 (branch/message). Branches are terminators
  // and normally sit outside scheduling regions; classified for
  // totality.
  if (mi.isBranch() || mi.isCall() || mi.isReturn()) {
    return HwPipe::kOther;
  }

  // Hardware category 7 (special/internal). Only s_barrier occurs
  // mid-region at scheduling time; the rest are listed defensively
  // (waitcnts/nops are inserted post-RA).
  switch (mi.getOpcode()) {
    case AMDGPU::S_BARRIER:
    case AMDGPU::S_WAITCNT:
    case AMDGPU::S_NOP:
    case AMDGPU::S_SLEEP:
    case AMDGPU::S_SETPRIO:
    case AMDGPU::S_SETHALT: {
      return HwPipe::kOther;
    }
    default: {
      break;
    }
  }

  // Category 3: vector memory. isVMEM covers MUBUF/MTBUF/MIMG but
  // NOT FLAT, so FLAT (flat/global/scratch segments) is added
  // explicitly.
  if (SIInstrInfo::isVMEM(mi) || SIInstrInfo::isFLAT(mi)) {
    return HwPipe::kVmem;
  }

  // Category 4: LDS.
  if (SIInstrInfo::isDS(mi)) {
    return HwPipe::kLds;
  }

  // Category 1: vector ALU (the VALU TSFlag covers VOP1/2/3/3P/C,
  // SDWA, DPP).
  if (SIInstrInfo::isVALU(mi)) {
    return HwPipe::kValu;
  }

  // Category 2: the shared scalar slot — scalar memory and scalar
  // ALU contend for one issue opportunity per cycle, so both map to
  // kScalar.
  if (SIInstrInfo::isSMRD(mi) || SIInstrInfo::isSALU(mi)) {
    return HwPipe::kScalar;
  }

  // Category 6 (export/GDS), inline asm, and any generic opcode not
  // resolved above.
  return HwPipe::kOther;
}

StringRef HwPipeName(HwPipe pipe) {
  switch (pipe) {
    case HwPipe::kValu: {
      return "valu";
    }
    case HwPipe::kScalar: {
      return "scalar";
    }
    case HwPipe::kVmem: {
      return "vmem";
    }
    case HwPipe::kLds: {
      return "lds";
    }
    case HwPipe::kOther: {
      return "other";
    }
  }
  llvm_unreachable("unhandled HwPipe value");
}

}  // namespace hierarchical_scheduler
}  // namespace llvm
