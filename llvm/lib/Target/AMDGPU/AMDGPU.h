//===-- AMDGPU.h - MachineFunction passes hw codegen --------------*- C++ -*-=//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
/// \file
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_AMDGPU_H
#define LLVM_LIB_TARGET_AMDGPU_AMDGPU_H

#include "llvm/IR/PassManager.h"
#include "llvm/Pass.h"
#include "llvm/Support/CodeGen.h"

namespace llvm {

class TargetMachine;

// GlobalISel passes
void initializeAMDGPUPreLegalizerCombinerPass(PassRegistry &);
FunctionPass *createAMDGPUPreLegalizeCombiner(bool IsOptNone);
void initializeAMDGPUPostLegalizerCombinerPass(PassRegistry &);
FunctionPass *createAMDGPUPostLegalizeCombiner(bool IsOptNone);
FunctionPass *createAMDGPURegBankCombiner(bool IsOptNone);
void initializeAMDGPURegBankCombinerPass(PassRegistry &);

void initializeAMDGPURegBankSelectPass(PassRegistry &);

// SI Passes
FunctionPass *createGCNDPPCombinePass();
FunctionPass *createSIAnnotateControlFlowPass();
FunctionPass *createSIFoldOperandsPass();
FunctionPass *createSIPeepholeSDWAPass();
FunctionPass *createSILowerI1CopiesPass();
FunctionPass *createSIShrinkInstructionsPass();
FunctionPass *createSILoadStoreOptimizerPass();
FunctionPass *createSIWholeQuadModePass();
FunctionPass *createSIFixControlFlowLiveIntervalsPass();
FunctionPass *createSIOptimizeExecMaskingPreRAPass();
FunctionPass *createSIOptimizeVGPRLiveRangePass();
FunctionPass *createSIFixSGPRCopiesPass();
FunctionPass *createSIMemoryLegalizerPass();
FunctionPass *createSIInsertWaitcntsPass();
FunctionPass *createSIPreAllocateWWMRegsPass();
FunctionPass *createSIFormMemoryClausesPass();

FunctionPass *createSIPostRABundlerPass();
FunctionPass *createAMDGPUSimplifyLibCallsPass(const TargetMachine *);
FunctionPass *createAMDGPUUseNativeCallsPass();
ModulePass *createAMDGPURemoveIncompatibleFunctionsPass(const TargetMachine *);
FunctionPass *createAMDGPUCodeGenPreparePass();
FunctionPass *createAMDGPULateCodeGenPreparePass();
FunctionPass *createAMDGPUMachineCFGStructurizerPass();
FunctionPass *createAMDGPUPropagateAttributesEarlyPass(const TargetMachine *);
ModulePass *createAMDGPUPropagateAttributesLatePass(const TargetMachine *);
FunctionPass *createAMDGPURewriteOutArgumentsPass();
ModulePass *createAMDGPULowerModuleLDSPass();
FunctionPass *createSIModeRegisterPass();
FunctionPass *createGCNPreRAOptimizationsPass();

struct AMDGPUSimplifyLibCallsPass : PassInfoMixin<AMDGPUSimplifyLibCallsPass> {
  AMDGPUSimplifyLibCallsPass(TargetMachine &TM) : TM(TM) {}
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &AM);

private:
  TargetMachine &TM;
};

struct AMDGPUUseNativeCallsPass : PassInfoMixin<AMDGPUUseNativeCallsPass> {
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &AM);
};

void initializeAMDGPUDAGToDAGISelPass(PassRegistry&);

void initializeAMDGPUMachineCFGStructurizerPass(PassRegistry&);
extern char &AMDGPUMachineCFGStructurizerID;

void initializeAMDGPUAlwaysInlinePass(PassRegistry&);

Pass *createAMDGPUAnnotateKernelFeaturesPass();
Pass *createAMDGPUAttributorPass();
void initializeAMDGPUAttributorPass(PassRegistry &);
void initializeAMDGPUAnnotateKernelFeaturesPass(PassRegistry &);
extern char &AMDGPUAnnotateKernelFeaturesID;

enum class ScanOptions : bool { DPP, Iterative };
FunctionPass *createAMDGPUAtomicOptimizerPass(ScanOptions ScanStrategy);
void initializeAMDGPUAtomicOptimizerPass(PassRegistry &);
extern char &AMDGPUAtomicOptimizerID;

ModulePass *createAMDGPUCtorDtorLoweringLegacyPass();
void initializeAMDGPUCtorDtorLoweringLegacyPass(PassRegistry &);
extern char &AMDGPUCtorDtorLoweringLegacyPassID;

FunctionPass *createAMDGPULowerKernelArgumentsPass();
void initializeAMDGPULowerKernelArgumentsPass(PassRegistry &);
extern char &AMDGPULowerKernelArgumentsID;

FunctionPass *createAMDGPUPromoteKernelArgumentsPass();
void initializeAMDGPUPromoteKernelArgumentsPass(PassRegistry &);
extern char &AMDGPUPromoteKernelArgumentsID;

struct AMDGPUPromoteKernelArgumentsPass
    : PassInfoMixin<AMDGPUPromoteKernelArgumentsPass> {
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &AM);
};

ModulePass *createAMDGPULowerKernelAttributesPass();
void initializeAMDGPULowerKernelAttributesPass(PassRegistry &);
extern char &AMDGPULowerKernelAttributesID;

struct AMDGPULowerKernelAttributesPass
    : PassInfoMixin<AMDGPULowerKernelAttributesPass> {
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &AM);
};

void initializeAMDGPUPropagateAttributesEarlyPass(PassRegistry &);
extern char &AMDGPUPropagateAttributesEarlyID;

struct AMDGPUPropagateAttributesEarlyPass
    : PassInfoMixin<AMDGPUPropagateAttributesEarlyPass> {
  AMDGPUPropagateAttributesEarlyPass(TargetMachine &TM) : TM(TM) {}
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &AM);

private:
  TargetMachine &TM;
};

void initializeAMDGPUPropagateAttributesLatePass(PassRegistry &);
extern char &AMDGPUPropagateAttributesLateID;

struct AMDGPUPropagateAttributesLatePass
    : PassInfoMixin<AMDGPUPropagateAttributesLatePass> {
  AMDGPUPropagateAttributesLatePass(TargetMachine &TM) : TM(TM) {}
  PreservedAnalyses run(Module &M, ModuleAnalysisManager &AM);

private:
  TargetMachine &TM;
};

void initializeAMDGPULowerModuleLDSPass(PassRegistry &);
extern char &AMDGPULowerModuleLDSID;

struct AMDGPULowerModuleLDSPass : PassInfoMixin<AMDGPULowerModuleLDSPass> {
  PreservedAnalyses run(Module &M, ModuleAnalysisManager &AM);
};

void initializeAMDGPURewriteOutArgumentsPass(PassRegistry &);
extern char &AMDGPURewriteOutArgumentsID;

void initializeGCNDPPCombinePass(PassRegistry &);
extern char &GCNDPPCombineID;

void initializeSIFoldOperandsPass(PassRegistry &);
extern char &SIFoldOperandsID;

void initializeSIPeepholeSDWAPass(PassRegistry &);
extern char &SIPeepholeSDWAID;

void initializeSIShrinkInstructionsPass(PassRegistry&);
extern char &SIShrinkInstructionsID;

void initializeSIFixSGPRCopiesPass(PassRegistry &);
extern char &SIFixSGPRCopiesID;

void initializeSISimplifyPredicatedCopiesPass(PassRegistry &);
extern char &SISimplifyPredicatedCopiesID;

void initializeSILowerI1CopiesPass(PassRegistry &);
extern char &SILowerI1CopiesID;

void initializeSILowerSGPRSpillsPass(PassRegistry &);
extern char &SILowerSGPRSpillsID;

void initializeSILoadStoreOptimizerPass(PassRegistry &);
extern char &SILoadStoreOptimizerID;

void initializeSIWholeQuadModePass(PassRegistry &);
extern char &SIWholeQuadModeID;

void initializeSILowerControlFlowPass(PassRegistry &);
extern char &SILowerControlFlowID;

void initializeSIPreEmitPeepholePass(PassRegistry &);
extern char &SIPreEmitPeepholeID;

void initializeSILateBranchLoweringPass(PassRegistry &);
extern char &SILateBranchLoweringPassID;

void initializeSIOptimizeExecMaskingPass(PassRegistry &);
extern char &SIOptimizeExecMaskingID;

void initializeSIPreAllocateWWMRegsPass(PassRegistry &);
extern char &SIPreAllocateWWMRegsID;

void initializeAMDGPUSimplifyLibCallsPass(PassRegistry &);
extern char &AMDGPUSimplifyLibCallsID;

void initializeAMDGPUUseNativeCallsPass(PassRegistry &);
extern char &AMDGPUUseNativeCallsID;

void initializeAMDGPUPerfHintAnalysisPass(PassRegistry &);
extern char &AMDGPUPerfHintAnalysisID;

// Passes common to R600 and SI
FunctionPass *createAMDGPUPromoteAlloca();
void initializeAMDGPUPromoteAllocaPass(PassRegistry&);
extern char &AMDGPUPromoteAllocaID;

FunctionPass *createAMDGPUPromoteAllocaToVector();
void initializeAMDGPUPromoteAllocaToVectorPass(PassRegistry&);
extern char &AMDGPUPromoteAllocaToVectorID;

struct AMDGPUPromoteAllocaPass : PassInfoMixin<AMDGPUPromoteAllocaPass> {
  AMDGPUPromoteAllocaPass(TargetMachine &TM) : TM(TM) {}
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &AM);

private:
  TargetMachine &TM;
};

struct AMDGPUPromoteAllocaToVectorPass
    : PassInfoMixin<AMDGPUPromoteAllocaToVectorPass> {
  AMDGPUPromoteAllocaToVectorPass(TargetMachine &TM) : TM(TM) {}
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &AM);

private:
  TargetMachine &TM;
};

struct AMDGPUAtomicOptimizerPass : PassInfoMixin<AMDGPUAtomicOptimizerPass> {
  AMDGPUAtomicOptimizerPass(TargetMachine &TM, ScanOptions ScanImpl)
      : TM(TM), ScanImpl(ScanImpl) {}
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &AM);

private:
  TargetMachine &TM;
  ScanOptions ScanImpl;
};

Pass *createAMDGPUStructurizeCFGPass();
FunctionPass *createAMDGPUISelDag(TargetMachine &TM,
                                  CodeGenOpt::Level OptLevel);
ModulePass *createAMDGPUAlwaysInlinePass(bool GlobalOpt = true);

struct AMDGPUAlwaysInlinePass : PassInfoMixin<AMDGPUAlwaysInlinePass> {
  AMDGPUAlwaysInlinePass(bool GlobalOpt = true) : GlobalOpt(GlobalOpt) {}
  PreservedAnalyses run(Module &M, ModuleAnalysisManager &AM);

private:
  bool GlobalOpt;
};

class AMDGPUCodeGenPreparePass
    : public PassInfoMixin<AMDGPUCodeGenPreparePass> {
private:
  TargetMachine &TM;

public:
  AMDGPUCodeGenPreparePass(TargetMachine &TM) : TM(TM){};
  PreservedAnalyses run(Function &, FunctionAnalysisManager &);
};

FunctionPass *createAMDGPUAnnotateUniformValues();

ModulePass *createAMDGPULowerKernelCallsPass();
void initializeAMDGPULowerKernelCallsPass(PassRegistry&);
extern char &AMDGPULowerKernelCallsID;

ModulePass *createAMDGPUPrintfRuntimeBinding();
void initializeAMDGPUPrintfRuntimeBindingPass(PassRegistry&);
extern char &AMDGPUPrintfRuntimeBindingID;

void initializeAMDGPUResourceUsageAnalysisPass(PassRegistry &);
extern char &AMDGPUResourceUsageAnalysisID;

struct AMDGPUPrintfRuntimeBindingPass
    : PassInfoMixin<AMDGPUPrintfRuntimeBindingPass> {
  PreservedAnalyses run(Module &M, ModuleAnalysisManager &AM);
};

ModulePass* createAMDGPUUnifyMetadataPass();
void initializeAMDGPUUnifyMetadataPass(PassRegistry&);
extern char &AMDGPUUnifyMetadataID;

struct AMDGPUUnifyMetadataPass : PassInfoMixin<AMDGPUUnifyMetadataPass> {
  PreservedAnalyses run(Module &M, ModuleAnalysisManager &AM);
};

void initializeSIOptimizeExecMaskingPreRAPass(PassRegistry&);
extern char &SIOptimizeExecMaskingPreRAID;

void initializeSIOptimizeVGPRLiveRangePass(PassRegistry &);
extern char &SIOptimizeVGPRLiveRangeID;

void initializeAMDGPUAnnotateUniformValuesPass(PassRegistry&);
extern char &AMDGPUAnnotateUniformValuesPassID;

void initializeAMDGPUCodeGenPreparePass(PassRegistry&);
extern char &AMDGPUCodeGenPrepareID;

void initializeAMDGPURemoveIncompatibleFunctionsPass(PassRegistry &);
extern char &AMDGPURemoveIncompatibleFunctionsID;

void initializeAMDGPULateCodeGenPreparePass(PassRegistry &);
extern char &AMDGPULateCodeGenPrepareID;

FunctionPass *createAMDGPURewriteUndefForPHIPass();
void initializeAMDGPURewriteUndefForPHIPass(PassRegistry &);
extern char &AMDGPURewriteUndefForPHIPassID;

void initializeSIAnnotateControlFlowPass(PassRegistry&);
extern char &SIAnnotateControlFlowPassID;

void initializeSIMemoryLegalizerPass(PassRegistry&);
extern char &SIMemoryLegalizerID;

void initializeSIModeRegisterPass(PassRegistry&);
extern char &SIModeRegisterID;

void initializeAMDGPUInsertDelayAluPass(PassRegistry &);
extern char &AMDGPUInsertDelayAluID;

void initializeSIInsertHardClausesPass(PassRegistry &);
extern char &SIInsertHardClausesID;

void initializeSIInsertWaitcntsPass(PassRegistry&);
extern char &SIInsertWaitcntsID;

void initializeSIFormMemoryClausesPass(PassRegistry&);
extern char &SIFormMemoryClausesID;

void initializeSIPostRABundlerPass(PassRegistry&);
extern char &SIPostRABundlerID;

void initializeGCNCreateVOPDPass(PassRegistry &);
extern char &GCNCreateVOPDID;

void initializeAMDGPUUnifyDivergentExitNodesPass(PassRegistry&);
extern char &AMDGPUUnifyDivergentExitNodesID;

ImmutablePass *createAMDGPUAAWrapperPass();
void initializeAMDGPUAAWrapperPassPass(PassRegistry&);
ImmutablePass *createAMDGPUExternalAAWrapperPass();
void initializeAMDGPUExternalAAWrapperPass(PassRegistry&);

void initializeAMDGPUArgumentUsageInfoPass(PassRegistry &);

ModulePass *createAMDGPUOpenCLEnqueuedBlockLoweringPass();
void initializeAMDGPUOpenCLEnqueuedBlockLoweringPass(PassRegistry &);
extern char &AMDGPUOpenCLEnqueuedBlockLoweringID;

void initializeGCNNSAReassignPass(PassRegistry &);
extern char &GCNNSAReassignID;

void initializeGCNPreRAOptimizationsPass(PassRegistry &);
extern char &GCNPreRAOptimizationsID;

FunctionPass *createAMDGPUSetWavePriorityPass();
void initializeAMDGPUSetWavePriorityPass(PassRegistry &);

void initializeGCNRewritePartialRegUsesPass(llvm::PassRegistry &);
extern char &GCNRewritePartialRegUsesID;

namespace AMDGPU {
enum TargetIndex {
  TI_CONSTDATA_START,
  TI_SCRATCH_RSRC_DWORD0,
  TI_SCRATCH_RSRC_DWORD1,
  TI_SCRATCH_RSRC_DWORD2,
  TI_SCRATCH_RSRC_DWORD3
};
}

/// OpenCL uses address spaces to differentiate between
/// various memory regions on the hardware. On the CPU
/// all of the address spaces point to the same memory,
/// however on the GPU, each address space points to
/// a separate piece of memory that is unique from other
/// memory locations.
namespace AMDGPUAS {
enum : unsigned {
  // The maximum value for flat, generic, local, private, constant and region.
  MAX_AMDGPU_ADDRESS = 8,

  FLAT_ADDRESS = 0,   ///< Address space for flat memory.
  GLOBAL_ADDRESS = 1, ///< Address space for global memory (RAT0, VTX0).
  REGION_ADDRESS = 2, ///< Address space for region memory. (GDS)

  CONSTANT_ADDRESS = 4, ///< Address space for constant memory (VTX2).
  LOCAL_ADDRESS = 3,    ///< Address space for local memory.
  PRIVATE_ADDRESS = 5,  ///< Address space for private memory.

  CONSTANT_ADDRESS_32BIT = 6, ///< Address space for 32-bit constant memory.

  BUFFER_FAT_POINTER = 7, ///< Address space for 160-bit buffer fat pointers.
                          ///< Not used in backend.

  BUFFER_RESOURCE = 8, ///< Address space for 128-bit buffer resources.

  /// Internal address spaces. Can be freely renumbered.
  STREAMOUT_REGISTER = 128, ///< Address space for GS NGG Streamout registers.
  /// end Internal address spaces.

  /// Address space for direct addressable parameter memory (CONST0).
  PARAM_D_ADDRESS = 6,
  /// Address space for indirect addressable parameter memory (VTX1).
  PARAM_I_ADDRESS = 7,

  // Do not re-order the CONSTANT_BUFFER_* enums.  Several places depend on
  // this order to be able to dynamically index a constant buffer, for
  // example:
  //
  // ConstantBufferAS = CONSTANT_BUFFER_0 + CBIdx

  CONSTANT_BUFFER_0 = 8,
  CONSTANT_BUFFER_1 = 9,
  CONSTANT_BUFFER_2 = 10,
  CONSTANT_BUFFER_3 = 11,
  CONSTANT_BUFFER_4 = 12,
  CONSTANT_BUFFER_5 = 13,
  CONSTANT_BUFFER_6 = 14,
  CONSTANT_BUFFER_7 = 15,
  CONSTANT_BUFFER_8 = 16,
  CONSTANT_BUFFER_9 = 17,
  CONSTANT_BUFFER_10 = 18,
  CONSTANT_BUFFER_11 = 19,
  CONSTANT_BUFFER_12 = 20,
  CONSTANT_BUFFER_13 = 21,
  CONSTANT_BUFFER_14 = 22,
  CONSTANT_BUFFER_15 = 23,

  // Some places use this if the address space can't be determined.
  UNKNOWN_ADDRESS_SPACE = ~0u,
};
}

namespace AMDGPU {

// FIXME: Missing constant_32bit
inline bool isFlatGlobalAddrSpace(unsigned AS) {
  return AS == AMDGPUAS::GLOBAL_ADDRESS ||
         AS == AMDGPUAS::FLAT_ADDRESS ||
         AS == AMDGPUAS::CONSTANT_ADDRESS ||
         AS > AMDGPUAS::MAX_AMDGPU_ADDRESS;
}

inline bool isExtendedGlobalAddrSpace(unsigned AS) {
  return AS == AMDGPUAS::GLOBAL_ADDRESS || AS == AMDGPUAS::CONSTANT_ADDRESS ||
         AS == AMDGPUAS::CONSTANT_ADDRESS_32BIT ||
         AS > AMDGPUAS::MAX_AMDGPU_ADDRESS;
}

static inline bool addrspacesMayAlias(unsigned AS1, unsigned AS2) {
  static_assert(AMDGPUAS::MAX_AMDGPU_ADDRESS <= 8, "Addr space out of range");

  if (AS1 > AMDGPUAS::MAX_AMDGPU_ADDRESS || AS2 > AMDGPUAS::MAX_AMDGPU_ADDRESS)
    return true;

  // This array is indexed by address space value enum elements 0 ... to 8
  // clang-format off
  static const bool ASAliasRules[9][9] = {
    /*                   Flat   Global Region  Group Constant Private Const32 BufFatPtr BufRsrc */
    /* Flat     */        {true,  true,  false, true,  true,  true,  true,  true,  true},
    /* Global   */        {true,  true,  false, false, true,  false, true,  true,  true},
    /* Region   */        {false, false, true,  false, false, false, false, false, false},
    /* Group    */        {true,  false, false, true,  false, false, false, false, false},
    /* Constant */        {true,  true,  false, false, false, false, true,  true,  true},
    /* Private  */        {true,  false, false, false, false, true,  false, false, false},
    /* Constant 32-bit */ {true,  true,  false, false, true,  false, false, true,  true},
    /* Buffer Fat Ptr  */ {true,  true,  false, false, true,  false, true,  true,  true},
    /* Buffer Resource */ {true,  true,  false, false, true,  false, true,  true,  true},
  };
  // clang-format on

  return ASAliasRules[AS1][AS2];
}

}

} // End namespace llvm

#endif
