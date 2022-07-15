#include "InterprocUncoalescedAnalysisPass.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Transforms/IPO/PassManagerBuilder.h"

#define DEBUG_TYPE "uncoalesced-analysis"

using namespace llvm;

bool InterproceduralUncoalescedAnalysisPass::runOnModule(Module &M) {
  auto &CG = getAnalysis<CallGraphWrapperPass>().getCallGraph();

  // Generate topological order of visiting function nodes.
  std::vector<Function *> functionList;
  for (scc_iterator<CallGraph *> I = scc_begin(&CG), IE = scc_end(&CG);
                                       I != IE; ++I) {
    const std::vector<CallGraphNode *> &SCCCGNs = *I;
    for (std::vector<CallGraphNode *>::const_iterator CGNI = SCCCGNs.begin(),
						    CGNIE = SCCCGNs.end();
                                                CGNI != CGNIE; ++CGNI) {
      if ((*CGNI)->getFunction()) {
        Function *F = (*CGNI)->getFunction();
        if (!F->isDeclaration()) {
          functionList.insert(functionList.begin(), F);
        }
      }
    }
  }

  // Map from function to initial argument values.
  // It is built by joining all contexts in which the function is called.
  std::map<const Function *, std::map<const Value *, MultiplierValue>> 
      FunctionArgumentValues;

  // Run analysis on functions.
  for (Function *F : functionList) {
    // Filter out function that don't target GPU
    // TODO(jobaileyhandle): Right now hard-coded for gfx906, should broaden for all HIP-compatible gpus
    // Just do !(x86 or arm)?
    if(F->hasFnAttribute("target-cpu") && F->getFnAttribute("target-cpu").getValueAsString().equals("gfx906")) {
        LLVM_DEBUG(errs() << "Analyzing function: " << F->getName());
        DominatorTree DT(*F);
        UncoalescedAnalysis UA(F, &DT, &FunctionArgumentValues);
        errs() << "Analysis Results: \n";
        GPUState st = UA.BuildInitialState();
        UA.BuildAnalysisInfo(st);
        std::set<const Instruction*> uncoalesced = UA.getUncoalescedAccesses();
        UncoalescedAccessMap_.emplace(F, uncoalesced);
    }
  }
  return false;
}

char InterproceduralUncoalescedAnalysisPass::ID = 0;
static RegisterPass<InterproceduralUncoalescedAnalysisPass>
Y("interproc-uncoalesced-analysis", "Interprocedural analysis to detect uncoalesced accesses in gpu programs.");

// This lets us run the pass with Clang.
static void registerInterproceduralUncoalescedAnalysisPass(const PassManagerBuilder &builder, legacy::PassManagerBase &manager) {
  manager.add(new InterproceduralUncoalescedAnalysisPass());
}
static RegisterStandardPasses RegisterInterproceduralUncoalescedAnalysisPass(PassManagerBuilder::EP_ModuleOptimizerEarly, registerInterproceduralUncoalescedAnalysisPass);
