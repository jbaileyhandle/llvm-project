//===--- AMDGPUPropagateAttributes.cpp --------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
/// \file
/// \brief This pass propagates attributes from kernels to the non-entry
/// functions. Most of the library functions were not compiled for specific ABI,
/// yet will be correctly compiled if proper attributes are propagated from the
/// caller.
///
/// The pass analyzes call graph and propagates ABI target features through the
/// call graph.
///
/// It can run in two modes: as a function or module pass. A function pass
/// simply propagates attributes. A module pass clones functions if there are
/// callers with different ABI. If a function is cloned all call sites will
/// be updated to use a correct clone.
///
/// A function pass is limited in functionality but can run early in the
/// pipeline. A module pass is more powerful but has to run late, so misses
/// library folding opportunities.
//
//===----------------------------------------------------------------------===//

#include "AMDGPU.h"
#include "MCTargetDesc/AMDGPUMCTargetDesc.h"
#include "Utils/AMDGPUBaseInfo.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Transforms/Utils/Cloning.h"


//========================================================
// jbaile
//========================================================
#include <cxxabi.h>
#include <fstream>
#include <iostream>
#include <sstream>

namespace {
    const std::string waves_per_eu_attr = "amdgpu-waves-per-eu";
}
//========================================================

#define DEBUG_TYPE "amdgpu-propagate-attributes"

using namespace llvm;

namespace llvm {
extern const SubtargetFeatureKV AMDGPUFeatureKV[AMDGPU::NumSubtargetFeatures-1];
}

namespace {

// Target features to propagate.
static constexpr const FeatureBitset TargetFeatures = {
  AMDGPU::FeatureWavefrontSize16,
  AMDGPU::FeatureWavefrontSize32,
  AMDGPU::FeatureWavefrontSize64
};

class AMDGPUPropagateAttributes {

  class FnProperties {
  private:
    explicit FnProperties(const FeatureBitset &&FB) : Features(FB) {}

  public:
    explicit FnProperties(const TargetMachine &TM, const Function &F) {
      Features = TM.getSubtargetImpl(F)->getFeatureBits();
    }

    bool operator == (const FnProperties &Other) const {
      if ((Features & TargetFeatures) != (Other.Features & TargetFeatures))
        return false;
      return true;
    }

    FnProperties adjustToCaller(const FnProperties &CallerProps) const {
      FnProperties New((Features & ~TargetFeatures) | CallerProps.Features);
      return New;
    }

    FeatureBitset Features;
  };

  class Clone {
  public:
    Clone(const FnProperties &Props, Function *OrigF, Function *NewF) :
      Properties(Props), OrigF(OrigF), NewF(NewF) {}

    FnProperties Properties;
    Function *OrigF;
    Function *NewF;
  };

  const TargetMachine *TM;

  // Clone functions as needed or just set attributes.
  bool AllowClone;

  // Option propagation roots.
  SmallSet<Function *, 32> Roots;

  // Clones of functions with their attributes.
  SmallVector<Clone, 32> Clones;

  // Find a clone with required features.
  Function *findFunction(const FnProperties &PropsNeeded,
                         Function *OrigF);

  // Clone function \p F and set \p NewProps on the clone.
  // Cole takes the name of original function.
  Function *cloneWithProperties(Function &F, const FnProperties &NewProps);

  // Set new function's features in place.
  void setFeatures(Function &F, const FeatureBitset &NewFeatures);

  std::string getFeatureString(const FeatureBitset &Features) const;

  // Propagate attributes from Roots.
  bool process();

  //========================================================
  // jbaile
  //========================================================
  bool process_misched_config_file(Module &M);
  //========================================================

public:
  AMDGPUPropagateAttributes(const TargetMachine *TM, bool AllowClone) :
    TM(TM), AllowClone(AllowClone) {}

  // Use F as a root and propagate its attributes.
  bool process(Function &F);

  // Propagate attributes starting from kernel functions.
  bool process(Module &M);
};

// Allows to propagate attributes early, but no cloning is allowed as it must
// be a function pass to run before any optimizations.
// TODO: We shall only need a one instance of module pass, but that needs to be
// in the linker pipeline which is currently not possible.
class AMDGPUPropagateAttributesEarly : public FunctionPass {
  const TargetMachine *TM;

public:
  static char ID; // Pass identification

  AMDGPUPropagateAttributesEarly(const TargetMachine *TM = nullptr) :
    FunctionPass(ID), TM(TM) {
    initializeAMDGPUPropagateAttributesEarlyPass(
      *PassRegistry::getPassRegistry());
  }

  bool runOnFunction(Function &F) override;
};

// Allows to propagate attributes with cloning but does that late in the
// pipeline.
class AMDGPUPropagateAttributesLate : public ModulePass {
  const TargetMachine *TM;

public:
  static char ID; // Pass identification

  AMDGPUPropagateAttributesLate(const TargetMachine *TM = nullptr) :
    ModulePass(ID), TM(TM) {
    initializeAMDGPUPropagateAttributesLatePass(
      *PassRegistry::getPassRegistry());
  }

  bool runOnModule(Module &M) override;
};

}  // end anonymous namespace.

char AMDGPUPropagateAttributesEarly::ID = 0;
char AMDGPUPropagateAttributesLate::ID = 0;

INITIALIZE_PASS(AMDGPUPropagateAttributesEarly,
                "amdgpu-propagate-attributes-early",
                "Early propagate attributes from kernels to functions",
                false, false)
INITIALIZE_PASS(AMDGPUPropagateAttributesLate,
                "amdgpu-propagate-attributes-late",
                "Late propagate attributes from kernels to functions",
                false, false)

Function *
AMDGPUPropagateAttributes::findFunction(const FnProperties &PropsNeeded,
                                        Function *OrigF) {
  // TODO: search for clone's clones.
  for (Clone &C : Clones)
    if (C.OrigF == OrigF && PropsNeeded == C.Properties)
      return C.NewF;

  return nullptr;
}

bool AMDGPUPropagateAttributes::process(Module &M) {
  for (auto &F : M.functions())
    if (AMDGPU::isKernel(F.getCallingConv()))
      Roots.insert(&F);

  return Roots.empty() ? false : process();
}

bool AMDGPUPropagateAttributes::process(Function &F) {
  Roots.insert(&F);
  return process();
}

//===========================================================
// jbaile
//===========================================================
std::string demangle(const std::string &mangled_name) {
    int status = 0;

    char *demangled = abi::__cxa_demangle(mangled_name.c_str(), nullptr, nullptr, &status);
    std::string result = (status == 0 && demangled) ? demangled : mangled_name;
    std::free(demangled);
    return result;
}

bool get_function_maximum_waves_per_eu(const Function &F, int *max_waves_per_eu) {
    if(!F.hasFnAttribute(waves_per_eu_attr)) {
        return false;
    }

    std::string existing_attr_str = F.getFnAttribute(waves_per_eu_attr).getValueAsString().str();
    size_t comma_pos = existing_attr_str.find(',');
    if(comma_pos == std::string::npos) {
        return false;
    }

    *max_waves_per_eu = std::stoi(existing_attr_str.substr(comma_pos+1));
    return true;
}

bool AMDGPUPropagateAttributes::process_misched_config_file(Module &M) {
  bool changed = false;

  std::ifstream misched_config_file("misched.txt");
  if(misched_config_file) {

      // Skip scheduler line
      std::string line;
      assert(std::getline(misched_config_file, line) && "Expected a scheduler line in misched.txt");

      // Read in function config lines
      std::unordered_map<std::string, int> function_name_to_config_waves_per_eu;
      while(std::getline(misched_config_file, line)) {
          std::istringstream iss(line);
          std::string function_name;
          int config_waves_per_eu;

          std::getline(iss, function_name, '\t');
          assert(!function_name.empty() && "Function name field in misched.txt was empty");
          assert((iss >> config_waves_per_eu) && "<desired_waves_per_eu> field in misched.txt was empty");
          assert((function_name_to_config_waves_per_eu.find(function_name) == function_name_to_config_waves_per_eu.end()) && "Duplicate function name in misched.txt");
          function_name_to_config_waves_per_eu[function_name] = config_waves_per_eu;
      }

      // Update function attributes
      for (auto &F : M.functions()) {

          // This function has a config in file
          std::string demangled_name = demangle(F.getName().str());
          if(function_name_to_config_waves_per_eu.find(demangled_name) != function_name_to_config_waves_per_eu.end()) {
              int config_waves_per_eu = function_name_to_config_waves_per_eu.at(demangled_name);

              // Clear old attribute
              F.removeFnAttr(waves_per_eu_attr);

              // Get previous maximum waves per eu. Default to 10 if no prior setting (max hardware supported)
              int exisitng_fn_max_waves_per_eu;
              bool result = get_function_maximum_waves_per_eu(F, &exisitng_fn_max_waves_per_eu);
              if(!result) {
                 exisitng_fn_max_waves_per_eu = 10;
              }

              // Set new attribute
              std::string new_waves_per_eu_pair = std::to_string(config_waves_per_eu) + "," + std::to_string(exisitng_fn_max_waves_per_eu);
              F.addFnAttr(waves_per_eu_attr, new_waves_per_eu_pair);
              changed = true;
          }
      }
  }

    return changed;
}
//===========================================================

bool AMDGPUPropagateAttributes::process() {
  bool Changed = false;
  SmallSet<Function *, 32> NewRoots;
  SmallSet<Function *, 32> Replaced;

  assert(!Roots.empty());
  Module &M = *(*Roots.begin())->getParent();

    //===========================================================
    // jbaile
    //===========================================================
  Changed |= process_misched_config_file(M);
    //===========================================================

  do {
    Roots.insert(NewRoots.begin(), NewRoots.end());
    NewRoots.clear();

    for (auto &F : M.functions()) {
      if (F.isDeclaration())
        continue;

      const FnProperties CalleeProps(*TM, F);
      SmallVector<std::pair<CallBase *, Function *>, 32> ToReplace;
      SmallSet<CallBase *, 32> Visited;

      for (User *U : F.users()) {
        Instruction *I = dyn_cast<Instruction>(U);
        if (!I)
          continue;
        CallBase *CI = dyn_cast<CallBase>(I);
        // Only propagate attributes if F is the called function. Specifically,
        // do not propagate attributes if F is passed as an argument.
        // FIXME: handle bitcasted callee, e.g.
        // %retval = call i8* bitcast (i32* ()* @f to i8* ()*)()
        if (!CI || CI->getCalledOperand() != &F)
          continue;
        Function *Caller = CI->getCaller();
        if (!Caller || !Visited.insert(CI).second)
          continue;
        if (!Roots.count(Caller) && !NewRoots.count(Caller))
          continue;

        const FnProperties CallerProps(*TM, *Caller);

        if (CalleeProps == CallerProps) {
          if (!Roots.count(&F))
            NewRoots.insert(&F);
          continue;
        }

        Function *NewF = findFunction(CallerProps, &F);
        if (!NewF) {
          const FnProperties NewProps = CalleeProps.adjustToCaller(CallerProps);
          if (!AllowClone) {
            // This may set different features on different iterations if
            // there is a contradiction in callers' attributes. In this case
            // we rely on a second pass running on Module, which is allowed
            // to clone.
            setFeatures(F, NewProps.Features);
            NewRoots.insert(&F);
            Changed = true;
            break;
          }

          NewF = cloneWithProperties(F, NewProps);
          Clones.push_back(Clone(CallerProps, &F, NewF));
          NewRoots.insert(NewF);
        }

        ToReplace.push_back(std::pair(CI, NewF));
        Replaced.insert(&F);

        Changed = true;
      }

      while (!ToReplace.empty()) {
        auto R = ToReplace.pop_back_val();
        R.first->setCalledFunction(R.second);
      }
    }
  } while (!NewRoots.empty());

  for (Function *F : Replaced) {
    if (F->use_empty())
      F->eraseFromParent();
  }

  Roots.clear();
  Clones.clear();

  return Changed;
}

Function *
AMDGPUPropagateAttributes::cloneWithProperties(Function &F,
                                               const FnProperties &NewProps) {
  LLVM_DEBUG(dbgs() << "Cloning " << F.getName() << '\n');

  ValueToValueMapTy dummy;
  Function *NewF = CloneFunction(&F, dummy);
  setFeatures(*NewF, NewProps.Features);
  NewF->setVisibility(GlobalValue::DefaultVisibility);
  NewF->setLinkage(GlobalValue::InternalLinkage);

  // Swap names. If that is the only clone it will retain the name of now
  // dead value. Preserve original name for externally visible functions.
  if (F.hasName() && F.hasLocalLinkage()) {
    std::string NewName = std::string(NewF->getName());
    NewF->takeName(&F);
    F.setName(NewName);
  }

  return NewF;
}

void AMDGPUPropagateAttributes::setFeatures(Function &F,
                                            const FeatureBitset &NewFeatures) {
  std::string NewFeatureStr = getFeatureString(NewFeatures);

  LLVM_DEBUG(dbgs() << "Set features "
                    << getFeatureString(NewFeatures & TargetFeatures)
                    << " on " << F.getName() << '\n');

  F.removeFnAttr("target-features");
  F.addFnAttr("target-features", NewFeatureStr);
}

std::string
AMDGPUPropagateAttributes::getFeatureString(const FeatureBitset &Features) const
{
  std::string Ret;
  for (const SubtargetFeatureKV &KV : AMDGPUFeatureKV) {
    if (Features[KV.Value])
      Ret += (StringRef("+") + KV.Key + ",").str();
    else if (TargetFeatures[KV.Value])
      Ret += (StringRef("-") + KV.Key + ",").str();
  }
  Ret.pop_back(); // Remove last comma.
  return Ret;
}

bool AMDGPUPropagateAttributesEarly::runOnFunction(Function &F) {
  if (!TM) {
    auto *TPC = getAnalysisIfAvailable<TargetPassConfig>();
    if (!TPC)
      return false;

    TM = &TPC->getTM<TargetMachine>();
  }

  if (!AMDGPU::isKernel(F.getCallingConv()))
    return false;

  return AMDGPUPropagateAttributes(TM, false).process(F);
}

bool AMDGPUPropagateAttributesLate::runOnModule(Module &M) {
  if (!TM) {
    auto *TPC = getAnalysisIfAvailable<TargetPassConfig>();
    if (!TPC)
      return false;

    TM = &TPC->getTM<TargetMachine>();
  }

  return AMDGPUPropagateAttributes(TM, true).process(M);
}

FunctionPass
*llvm::createAMDGPUPropagateAttributesEarlyPass(const TargetMachine *TM) {
  return new AMDGPUPropagateAttributesEarly(TM);
}

ModulePass
*llvm::createAMDGPUPropagateAttributesLatePass(const TargetMachine *TM) {
  return new AMDGPUPropagateAttributesLate(TM);
}

PreservedAnalyses
AMDGPUPropagateAttributesEarlyPass::run(Function &F,
                                        FunctionAnalysisManager &AM) {
  if (!AMDGPU::isEntryFunctionCC(F.getCallingConv()))
    return PreservedAnalyses::all();

  return AMDGPUPropagateAttributes(&TM, false).process(F)
             ? PreservedAnalyses::none()
             : PreservedAnalyses::all();
}

PreservedAnalyses
AMDGPUPropagateAttributesLatePass::run(Module &M, ModuleAnalysisManager &AM) {
  return AMDGPUPropagateAttributes(&TM, true).process(M)
             ? PreservedAnalyses::none()
             : PreservedAnalyses::all();
}
