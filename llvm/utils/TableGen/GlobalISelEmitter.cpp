//===- GlobalISelEmitter.cpp - Generate an instruction selector -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
/// \file
/// This tablegen backend emits code for use by the GlobalISel instruction
/// selector. See include/llvm/Target/GlobalISel/Target.td.
///
/// This file analyzes the patterns recognized by the SelectionDAGISel tablegen
/// backend, filters out the ones that are unsupported, maps
/// SelectionDAG-specific constructs to their GlobalISel counterpart
/// (when applicable: MVT to LLT;  SDNode to generic Instruction).
///
/// Not all patterns are supported: pass the tablegen invocation
/// "-warn-on-skipped-patterns" to emit a warning when a pattern is skipped,
/// as well as why.
///
/// The generated file defines a single method:
///     bool <Target>InstructionSelector::selectImpl(MachineInstr &I) const;
/// intended to be used in InstructionSelector::select as the first-step
/// selector for the patterns that don't require complex C++.
///
/// FIXME: We'll probably want to eventually define a base
/// "TargetGenInstructionSelector" class.
///
//===----------------------------------------------------------------------===//

#include "CodeGenDAGPatterns.h"
#include "CodeGenInstruction.h"
#include "CodeGenIntrinsics.h"
#include "CodeGenRegisters.h"
#include "CodeGenTarget.h"
#include "GlobalISelMatchTable.h"
#include "InfoByHwMode.h"
#include "SubtargetFeatureInfo.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/LowLevelType.h"
#include "llvm/CodeGen/MachineValueType.h"
#include "llvm/Support/CodeGenCoverage.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/SaveAndRestore.h"
#include "llvm/Support/ScopedPrinter.h"
#include "llvm/TableGen/Error.h"
#include "llvm/TableGen/Record.h"
#include "llvm/TableGen/TableGenBackend.h"
#include <numeric>
#include <string>

using namespace llvm;
using namespace llvm::gi;

using action_iterator = RuleMatcher::action_iterator;

#define DEBUG_TYPE "gisel-emitter"

STATISTIC(NumPatternTotal, "Total number of patterns");
STATISTIC(NumPatternImported, "Number of patterns imported from SelectionDAG");
STATISTIC(NumPatternImportsSkipped, "Number of SelectionDAG imports skipped");
STATISTIC(NumPatternsTested,
          "Number of patterns executed according to coverage information");

cl::OptionCategory GlobalISelEmitterCat("Options for -gen-global-isel");

static cl::opt<bool> WarnOnSkippedPatterns(
    "warn-on-skipped-patterns",
    cl::desc("Explain why a pattern was skipped for inclusion "
             "in the GlobalISel selector"),
    cl::init(false), cl::cat(GlobalISelEmitterCat));

static cl::opt<bool> GenerateCoverage(
    "instrument-gisel-coverage",
    cl::desc("Generate coverage instrumentation for GlobalISel"),
    cl::init(false), cl::cat(GlobalISelEmitterCat));

static cl::opt<std::string> UseCoverageFile(
    "gisel-coverage-file", cl::init(""),
    cl::desc("Specify file to retrieve coverage information from"),
    cl::cat(GlobalISelEmitterCat));

static cl::opt<bool> OptimizeMatchTable(
    "optimize-match-table",
    cl::desc("Generate an optimized version of the match table"),
    cl::init(true), cl::cat(GlobalISelEmitterCat));

namespace {

static std::string explainPredicates(const TreePatternNode *N) {
  std::string Explanation;
  StringRef Separator = "";
  for (const TreePredicateCall &Call : N->getPredicateCalls()) {
    const TreePredicateFn &P = Call.Fn;
    Explanation +=
        (Separator + P.getOrigPatFragRecord()->getRecord()->getName()).str();
    Separator = ", ";

    if (P.isAlwaysTrue())
      Explanation += " always-true";
    if (P.isImmediatePattern())
      Explanation += " immediate";

    if (P.isUnindexed())
      Explanation += " unindexed";

    if (P.isNonExtLoad())
      Explanation += " non-extload";
    if (P.isAnyExtLoad())
      Explanation += " extload";
    if (P.isSignExtLoad())
      Explanation += " sextload";
    if (P.isZeroExtLoad())
      Explanation += " zextload";

    if (P.isNonTruncStore())
      Explanation += " non-truncstore";
    if (P.isTruncStore())
      Explanation += " truncstore";

    if (Record *VT = P.getMemoryVT())
      Explanation += (" MemVT=" + VT->getName()).str();
    if (Record *VT = P.getScalarMemoryVT())
      Explanation += (" ScalarVT(MemVT)=" + VT->getName()).str();

    if (ListInit *AddrSpaces = P.getAddressSpaces()) {
      raw_string_ostream OS(Explanation);
      OS << " AddressSpaces=[";

      StringRef AddrSpaceSeparator;
      for (Init *Val : AddrSpaces->getValues()) {
        IntInit *IntVal = dyn_cast<IntInit>(Val);
        if (!IntVal)
          continue;

        OS << AddrSpaceSeparator << IntVal->getValue();
        AddrSpaceSeparator = ", ";
      }

      OS << ']';
    }

    int64_t MinAlign = P.getMinAlignment();
    if (MinAlign > 0)
      Explanation += " MinAlign=" + utostr(MinAlign);

    if (P.isAtomicOrderingMonotonic())
      Explanation += " monotonic";
    if (P.isAtomicOrderingAcquire())
      Explanation += " acquire";
    if (P.isAtomicOrderingRelease())
      Explanation += " release";
    if (P.isAtomicOrderingAcquireRelease())
      Explanation += " acq_rel";
    if (P.isAtomicOrderingSequentiallyConsistent())
      Explanation += " seq_cst";
    if (P.isAtomicOrderingAcquireOrStronger())
      Explanation += " >=acquire";
    if (P.isAtomicOrderingWeakerThanAcquire())
      Explanation += " <acquire";
    if (P.isAtomicOrderingReleaseOrStronger())
      Explanation += " >=release";
    if (P.isAtomicOrderingWeakerThanRelease())
      Explanation += " <release";
  }
  return Explanation;
}

std::string explainOperator(Record *Operator) {
  if (Operator->isSubClassOf("SDNode"))
    return (" (" + Operator->getValueAsString("Opcode") + ")").str();

  if (Operator->isSubClassOf("Intrinsic"))
    return (" (Operator is an Intrinsic, " + Operator->getName() + ")").str();

  if (Operator->isSubClassOf("ComplexPattern"))
    return (" (Operator is an unmapped ComplexPattern, " + Operator->getName() +
            ")")
        .str();

  if (Operator->isSubClassOf("SDNodeXForm"))
    return (" (Operator is an unmapped SDNodeXForm, " + Operator->getName() +
            ")")
        .str();

  return (" (Operator " + Operator->getName() + " not understood)").str();
}

/// Helper function to let the emitter report skip reason error messages.
static Error failedImport(const Twine &Reason) {
  return make_error<StringError>(Reason, inconvertibleErrorCode());
}

static Error isTrivialOperatorNode(const TreePatternNode *N) {
  std::string Explanation;
  std::string Separator;

  bool HasUnsupportedPredicate = false;
  for (const TreePredicateCall &Call : N->getPredicateCalls()) {
    const TreePredicateFn &Predicate = Call.Fn;

    if (Predicate.isAlwaysTrue())
      continue;

    if (Predicate.isImmediatePattern())
      continue;

    if (Predicate.hasNoUse())
      continue;

    if (Predicate.isNonExtLoad() || Predicate.isAnyExtLoad() ||
        Predicate.isSignExtLoad() || Predicate.isZeroExtLoad())
      continue;

    if (Predicate.isNonTruncStore() || Predicate.isTruncStore())
      continue;

    if (Predicate.isLoad() && Predicate.getMemoryVT())
      continue;

    if (Predicate.isLoad() || Predicate.isStore()) {
      if (Predicate.isUnindexed())
        continue;
    }

    if (Predicate.isLoad() || Predicate.isStore() || Predicate.isAtomic()) {
      const ListInit *AddrSpaces = Predicate.getAddressSpaces();
      if (AddrSpaces && !AddrSpaces->empty())
        continue;

      if (Predicate.getMinAlignment() > 0)
        continue;
    }

    if (Predicate.isAtomic() && Predicate.getMemoryVT())
      continue;

    if (Predicate.isAtomic() &&
        (Predicate.isAtomicOrderingMonotonic() ||
         Predicate.isAtomicOrderingAcquire() ||
         Predicate.isAtomicOrderingRelease() ||
         Predicate.isAtomicOrderingAcquireRelease() ||
         Predicate.isAtomicOrderingSequentiallyConsistent() ||
         Predicate.isAtomicOrderingAcquireOrStronger() ||
         Predicate.isAtomicOrderingWeakerThanAcquire() ||
         Predicate.isAtomicOrderingReleaseOrStronger() ||
         Predicate.isAtomicOrderingWeakerThanRelease()))
      continue;

    if (Predicate.hasGISelPredicateCode())
      continue;

    HasUnsupportedPredicate = true;
    Explanation = Separator + "Has a predicate (" + explainPredicates(N) + ")";
    Separator = ", ";
    Explanation += (Separator + "first-failing:" +
                    Predicate.getOrigPatFragRecord()->getRecord()->getName())
                       .str();
    break;
  }

  if (!HasUnsupportedPredicate)
    return Error::success();

  return failedImport(Explanation);
}

static Record *getInitValueAsRegClass(Init *V) {
  if (DefInit *VDefInit = dyn_cast<DefInit>(V)) {
    if (VDefInit->getDef()->isSubClassOf("RegisterOperand"))
      return VDefInit->getDef()->getValueAsDef("RegClass");
    if (VDefInit->getDef()->isSubClassOf("RegisterClass"))
      return VDefInit->getDef();
  }
  return nullptr;
}

static std::string getScopedName(unsigned Scope, const std::string &Name) {
  return ("pred:" + Twine(Scope) + ":" + Name).str();
}

//===- GlobalISelEmitter class --------------------------------------------===//

static Expected<LLTCodeGen> getInstResultType(const TreePatternNode *Dst) {
  ArrayRef<TypeSetByHwMode> ChildTypes = Dst->getExtTypes();
  if (ChildTypes.size() != 1)
    return failedImport("Dst pattern child has multiple results");

  std::optional<LLTCodeGen> MaybeOpTy;
  if (ChildTypes.front().isMachineValueType()) {
    MaybeOpTy =
      MVTToLLT(ChildTypes.front().getMachineValueType().SimpleTy);
  }

  if (!MaybeOpTy)
    return failedImport("Dst operand has an unsupported type");
  return *MaybeOpTy;
}

class GlobalISelEmitter {
public:
  explicit GlobalISelEmitter(RecordKeeper &RK);
  void run(raw_ostream &OS);

private:
  const RecordKeeper &RK;
  const CodeGenDAGPatterns CGP;
  const CodeGenTarget &Target;
  CodeGenRegBank &CGRegs;

  /// Keep track of the equivalence between SDNodes and Instruction by mapping
  /// SDNodes to the GINodeEquiv mapping. We need to map to the GINodeEquiv to
  /// check for attributes on the relation such as CheckMMOIsNonAtomic.
  /// This is defined using 'GINodeEquiv' in the target description.
  DenseMap<Record *, Record *> NodeEquivs;

  /// Keep track of the equivalence between ComplexPattern's and
  /// GIComplexOperandMatcher. Map entries are specified by subclassing
  /// GIComplexPatternEquiv.
  DenseMap<const Record *, const Record *> ComplexPatternEquivs;

  /// Keep track of the equivalence between SDNodeXForm's and
  /// GICustomOperandRenderer. Map entries are specified by subclassing
  /// GISDNodeXFormEquiv.
  DenseMap<const Record *, const Record *> SDNodeXFormEquivs;

  /// Keep track of Scores of PatternsToMatch similar to how the DAG does.
  /// This adds compatibility for RuleMatchers to use this for ordering rules.
  DenseMap<uint64_t, int> RuleMatcherScores;

  // Map of predicates to their subtarget features.
  SubtargetFeatureInfoMap SubtargetFeatures;

  // Rule coverage information.
  std::optional<CodeGenCoverage> RuleCoverage;

  /// Variables used to help with collecting of named operands for predicates
  /// with 'let PredicateCodeUsesOperands = 1'. WaitingForNamedOperands is set
  /// to the number of named operands that predicate expects. Store locations in
  /// StoreIdxForName correspond to the order in which operand names appear in
  /// predicate's argument list.
  /// When we visit named leaf operand and WaitingForNamedOperands is not zero,
  /// add matcher that will record operand and decrease counter.
  unsigned WaitingForNamedOperands = 0;
  StringMap<unsigned> StoreIdxForName;

  void gatherOpcodeValues();
  void gatherTypeIDValues();
  void gatherNodeEquivs();

  Record *findNodeEquiv(Record *N) const;
  const CodeGenInstruction *getEquivNode(Record &Equiv,
                                         const TreePatternNode *N) const;

  Error importRulePredicates(RuleMatcher &M, ArrayRef<Record *> Predicates);
  Expected<InstructionMatcher &>
  createAndImportSelDAGMatcher(RuleMatcher &Rule,
                               InstructionMatcher &InsnMatcher,
                               const TreePatternNode *Src, unsigned &TempOpIdx);
  Error importComplexPatternOperandMatcher(OperandMatcher &OM, Record *R,
                                           unsigned &TempOpIdx) const;
  Error importChildMatcher(RuleMatcher &Rule, InstructionMatcher &InsnMatcher,
                           const TreePatternNode *SrcChild,
                           bool OperandIsAPointer, bool OperandIsImmArg,
                           unsigned OpIdx, unsigned &TempOpIdx);

  Expected<BuildMIAction &> createAndImportInstructionRenderer(
      RuleMatcher &M, InstructionMatcher &InsnMatcher,
      const TreePatternNode *Src, const TreePatternNode *Dst);
  Expected<action_iterator> createAndImportSubInstructionRenderer(
      action_iterator InsertPt, RuleMatcher &M, const TreePatternNode *Dst,
      const TreePatternNode *Src, unsigned TempReg);
  Expected<action_iterator>
  createInstructionRenderer(action_iterator InsertPt, RuleMatcher &M,
                            const TreePatternNode *Dst);

  Expected<action_iterator> importExplicitDefRenderers(
      action_iterator InsertPt, RuleMatcher &M, BuildMIAction &DstMIBuilder,
      const TreePatternNode *Src, const TreePatternNode *Dst);

  Expected<action_iterator> importExplicitUseRenderers(
      action_iterator InsertPt, RuleMatcher &M, BuildMIAction &DstMIBuilder,
      const llvm::TreePatternNode *Dst, const TreePatternNode *Src);
  Expected<action_iterator> importExplicitUseRenderer(
      action_iterator InsertPt, RuleMatcher &Rule, BuildMIAction &DstMIBuilder,
      const TreePatternNode *DstChild, const TreePatternNode *Src);
  Error importDefaultOperandRenderers(action_iterator InsertPt, RuleMatcher &M,
                                      BuildMIAction &DstMIBuilder,
                                      DagInit *DefaultOps) const;
  Error
  importImplicitDefRenderers(BuildMIAction &DstMIBuilder,
                             const std::vector<Record *> &ImplicitDefs) const;

  void emitCxxPredicateFns(raw_ostream &OS, StringRef CodeFieldName,
                           StringRef TypeIdentifier, StringRef ArgType,
                           StringRef ArgName, StringRef AdditionalArgs,
                           StringRef AdditionalDeclarations,
                           std::function<bool(const Record *R)> Filter);
  void emitImmPredicateFns(raw_ostream &OS, StringRef TypeIdentifier,
                           StringRef ArgType,
                           std::function<bool(const Record *R)> Filter);
  void emitMIPredicateFns(raw_ostream &OS);

  /// Analyze pattern \p P, returning a matcher for it if possible.
  /// Otherwise, return an Error explaining why we don't support it.
  Expected<RuleMatcher> runOnPattern(const PatternToMatch &P);

  void declareSubtargetFeature(Record *Predicate);

  MatchTable buildMatchTable(MutableArrayRef<RuleMatcher> Rules, bool Optimize,
                             bool WithCoverage);

  /// Infer a CodeGenRegisterClass for the type of \p SuperRegNode. The returned
  /// CodeGenRegisterClass will support the CodeGenRegisterClass of
  /// \p SubRegNode, and the subregister index defined by \p SubRegIdxNode.
  /// If no register class is found, return std::nullopt.
  std::optional<const CodeGenRegisterClass *>
  inferSuperRegisterClassForNode(const TypeSetByHwMode &Ty,
                                 const TreePatternNode *SuperRegNode,
                                 const TreePatternNode *SubRegIdxNode);
  std::optional<CodeGenSubRegIndex *>
  inferSubRegIndexForNode(const TreePatternNode *SubRegIdxNode);

  /// Infer a CodeGenRegisterClass which suppoorts \p Ty and \p SubRegIdxNode.
  /// Return std::nullopt if no such class exists.
  std::optional<const CodeGenRegisterClass *>
  inferSuperRegisterClass(const TypeSetByHwMode &Ty,
                          const TreePatternNode *SubRegIdxNode);

  /// Return the CodeGenRegisterClass associated with \p Leaf if it has one.
  std::optional<const CodeGenRegisterClass *>
  getRegClassFromLeaf(const TreePatternNode *Leaf);

  /// Return a CodeGenRegisterClass for \p N if one can be found. Return
  /// std::nullopt otherwise.
  std::optional<const CodeGenRegisterClass *>
  inferRegClassFromPattern(const TreePatternNode *N);

  /// Return the size of the MemoryVT in this predicate, if possible.
  std::optional<unsigned>
  getMemSizeBitsFromPredicate(const TreePredicateFn &Predicate);

  // Add builtin predicates.
  Expected<InstructionMatcher &>
  addBuiltinPredicates(const Record *SrcGIEquivOrNull,
                       const TreePredicateFn &Predicate,
                       InstructionMatcher &InsnMatcher, bool &HasAddedMatcher);
};

void GlobalISelEmitter::gatherOpcodeValues() {
  InstructionOpcodeMatcher::initOpcodeValuesMap(Target);
}

void GlobalISelEmitter::gatherTypeIDValues() {
  LLTOperandMatcher::initTypeIDValuesMap();
}

void GlobalISelEmitter::gatherNodeEquivs() {
  assert(NodeEquivs.empty());
  for (Record *Equiv : RK.getAllDerivedDefinitions("GINodeEquiv"))
    NodeEquivs[Equiv->getValueAsDef("Node")] = Equiv;

  assert(ComplexPatternEquivs.empty());
  for (Record *Equiv : RK.getAllDerivedDefinitions("GIComplexPatternEquiv")) {
    Record *SelDAGEquiv = Equiv->getValueAsDef("SelDAGEquivalent");
    if (!SelDAGEquiv)
      continue;
    ComplexPatternEquivs[SelDAGEquiv] = Equiv;
 }

 assert(SDNodeXFormEquivs.empty());
 for (Record *Equiv : RK.getAllDerivedDefinitions("GISDNodeXFormEquiv")) {
   Record *SelDAGEquiv = Equiv->getValueAsDef("SelDAGEquivalent");
   if (!SelDAGEquiv)
     continue;
   SDNodeXFormEquivs[SelDAGEquiv] = Equiv;
 }
}

Record *GlobalISelEmitter::findNodeEquiv(Record *N) const {
  return NodeEquivs.lookup(N);
}

const CodeGenInstruction *
GlobalISelEmitter::getEquivNode(Record &Equiv, const TreePatternNode *N) const {
  if (N->getNumChildren() >= 1) {
    // setcc operation maps to two different G_* instructions based on the type.
    if (!Equiv.isValueUnset("IfFloatingPoint") &&
        MVT(N->getChild(0)->getSimpleType(0)).isFloatingPoint())
      return &Target.getInstruction(Equiv.getValueAsDef("IfFloatingPoint"));
  }

  for (const TreePredicateCall &Call : N->getPredicateCalls()) {
    const TreePredicateFn &Predicate = Call.Fn;
    if (!Equiv.isValueUnset("IfSignExtend") &&
        (Predicate.isLoad() || Predicate.isAtomic()) &&
        Predicate.isSignExtLoad())
      return &Target.getInstruction(Equiv.getValueAsDef("IfSignExtend"));
    if (!Equiv.isValueUnset("IfZeroExtend") &&
        (Predicate.isLoad() || Predicate.isAtomic()) &&
        Predicate.isZeroExtLoad())
      return &Target.getInstruction(Equiv.getValueAsDef("IfZeroExtend"));
  }

  return &Target.getInstruction(Equiv.getValueAsDef("I"));
}

GlobalISelEmitter::GlobalISelEmitter(RecordKeeper &RK)
    : RK(RK), CGP(RK), Target(CGP.getTargetInfo()),
      CGRegs(Target.getRegBank()) {}

//===- Emitter ------------------------------------------------------------===//

Error GlobalISelEmitter::importRulePredicates(RuleMatcher &M,
                                              ArrayRef<Record *> Predicates) {
  for (Record *Pred : Predicates) {
    if (Pred->getValueAsString("CondString").empty())
      continue;
    declareSubtargetFeature(Pred);
    M.addRequiredFeature(Pred);
  }

  return Error::success();
}

std::optional<unsigned> GlobalISelEmitter::getMemSizeBitsFromPredicate(
    const TreePredicateFn &Predicate) {
  std::optional<LLTCodeGen> MemTyOrNone =
      MVTToLLT(getValueType(Predicate.getMemoryVT()));

  if (!MemTyOrNone)
    return std::nullopt;

  // Align so unusual types like i1 don't get rounded down.
  return llvm::alignTo(
      static_cast<unsigned>(MemTyOrNone->get().getSizeInBits()), 8);
}

Expected<InstructionMatcher &> GlobalISelEmitter::addBuiltinPredicates(
    const Record *SrcGIEquivOrNull, const TreePredicateFn &Predicate,
    InstructionMatcher &InsnMatcher, bool &HasAddedMatcher) {
  if (Predicate.isLoad() || Predicate.isStore() || Predicate.isAtomic()) {
    if (const ListInit *AddrSpaces = Predicate.getAddressSpaces()) {
      SmallVector<unsigned, 4> ParsedAddrSpaces;

      for (Init *Val : AddrSpaces->getValues()) {
        IntInit *IntVal = dyn_cast<IntInit>(Val);
        if (!IntVal)
          return failedImport("Address space is not an integer");
        ParsedAddrSpaces.push_back(IntVal->getValue());
      }

      if (!ParsedAddrSpaces.empty()) {
        InsnMatcher.addPredicate<MemoryAddressSpacePredicateMatcher>(
            0, ParsedAddrSpaces);
        return InsnMatcher;
      }
    }

    int64_t MinAlign = Predicate.getMinAlignment();
    if (MinAlign > 0) {
      InsnMatcher.addPredicate<MemoryAlignmentPredicateMatcher>(0, MinAlign);
      return InsnMatcher;
    }
  }

  // G_LOAD is used for both non-extending and any-extending loads.
  if (Predicate.isLoad() && Predicate.isNonExtLoad()) {
    InsnMatcher.addPredicate<MemoryVsLLTSizePredicateMatcher>(
        0, MemoryVsLLTSizePredicateMatcher::EqualTo, 0);
    return InsnMatcher;
  }
  if (Predicate.isLoad() && Predicate.isAnyExtLoad()) {
    InsnMatcher.addPredicate<MemoryVsLLTSizePredicateMatcher>(
        0, MemoryVsLLTSizePredicateMatcher::LessThan, 0);
    return InsnMatcher;
  }

  if (Predicate.isStore()) {
    if (Predicate.isTruncStore()) {
      if (Predicate.getMemoryVT() != nullptr) {
        // FIXME: If MemoryVT is set, we end up with 2 checks for the MMO size.
        auto MemSizeInBits = getMemSizeBitsFromPredicate(Predicate);
        if (!MemSizeInBits)
          return failedImport("MemVT could not be converted to LLT");

        InsnMatcher.addPredicate<MemorySizePredicateMatcher>(0, *MemSizeInBits /
                                                                    8);
      } else {
        InsnMatcher.addPredicate<MemoryVsLLTSizePredicateMatcher>(
            0, MemoryVsLLTSizePredicateMatcher::LessThan, 0);
      }
      return InsnMatcher;
    }
    if (Predicate.isNonTruncStore()) {
      // We need to check the sizes match here otherwise we could incorrectly
      // match truncating stores with non-truncating ones.
      InsnMatcher.addPredicate<MemoryVsLLTSizePredicateMatcher>(
          0, MemoryVsLLTSizePredicateMatcher::EqualTo, 0);
    }
  }

  assert(SrcGIEquivOrNull != nullptr && "Invalid SrcGIEquivOrNull value");
  // No check required. We already did it by swapping the opcode.
  if (!SrcGIEquivOrNull->isValueUnset("IfSignExtend") &&
      Predicate.isSignExtLoad())
    return InsnMatcher;

  // No check required. We already did it by swapping the opcode.
  if (!SrcGIEquivOrNull->isValueUnset("IfZeroExtend") &&
      Predicate.isZeroExtLoad())
    return InsnMatcher;

  // No check required. G_STORE by itself is a non-extending store.
  if (Predicate.isNonTruncStore())
    return InsnMatcher;

  if (Predicate.isLoad() || Predicate.isStore() || Predicate.isAtomic()) {
    if (Predicate.getMemoryVT() != nullptr) {
      auto MemSizeInBits = getMemSizeBitsFromPredicate(Predicate);
      if (!MemSizeInBits)
        return failedImport("MemVT could not be converted to LLT");

      InsnMatcher.addPredicate<MemorySizePredicateMatcher>(0,
                                                           *MemSizeInBits / 8);
      return InsnMatcher;
    }
  }

  if (Predicate.isLoad() || Predicate.isStore()) {
    // No check required. A G_LOAD/G_STORE is an unindexed load.
    if (Predicate.isUnindexed())
      return InsnMatcher;
  }

  if (Predicate.isAtomic()) {
    if (Predicate.isAtomicOrderingMonotonic()) {
      InsnMatcher.addPredicate<AtomicOrderingMMOPredicateMatcher>("Monotonic");
      return InsnMatcher;
    }
    if (Predicate.isAtomicOrderingAcquire()) {
      InsnMatcher.addPredicate<AtomicOrderingMMOPredicateMatcher>("Acquire");
      return InsnMatcher;
    }
    if (Predicate.isAtomicOrderingRelease()) {
      InsnMatcher.addPredicate<AtomicOrderingMMOPredicateMatcher>("Release");
      return InsnMatcher;
    }
    if (Predicate.isAtomicOrderingAcquireRelease()) {
      InsnMatcher.addPredicate<AtomicOrderingMMOPredicateMatcher>(
          "AcquireRelease");
      return InsnMatcher;
    }
    if (Predicate.isAtomicOrderingSequentiallyConsistent()) {
      InsnMatcher.addPredicate<AtomicOrderingMMOPredicateMatcher>(
          "SequentiallyConsistent");
      return InsnMatcher;
    }
  }

  if (Predicate.isAtomicOrderingAcquireOrStronger()) {
    InsnMatcher.addPredicate<AtomicOrderingMMOPredicateMatcher>(
        "Acquire", AtomicOrderingMMOPredicateMatcher::AO_OrStronger);
    return InsnMatcher;
  }
  if (Predicate.isAtomicOrderingWeakerThanAcquire()) {
    InsnMatcher.addPredicate<AtomicOrderingMMOPredicateMatcher>(
        "Acquire", AtomicOrderingMMOPredicateMatcher::AO_WeakerThan);
    return InsnMatcher;
  }

  if (Predicate.isAtomicOrderingReleaseOrStronger()) {
    InsnMatcher.addPredicate<AtomicOrderingMMOPredicateMatcher>(
        "Release", AtomicOrderingMMOPredicateMatcher::AO_OrStronger);
    return InsnMatcher;
  }
  if (Predicate.isAtomicOrderingWeakerThanRelease()) {
    InsnMatcher.addPredicate<AtomicOrderingMMOPredicateMatcher>(
        "Release", AtomicOrderingMMOPredicateMatcher::AO_WeakerThan);
    return InsnMatcher;
  }
  HasAddedMatcher = false;
  return InsnMatcher;
}

Expected<InstructionMatcher &> GlobalISelEmitter::createAndImportSelDAGMatcher(
    RuleMatcher &Rule, InstructionMatcher &InsnMatcher,
    const TreePatternNode *Src, unsigned &TempOpIdx) {
  const auto SavedFlags = Rule.setGISelFlags(Src->getGISelFlagsRecord());

  Record *SrcGIEquivOrNull = nullptr;
  const CodeGenInstruction *SrcGIOrNull = nullptr;

  // Start with the defined operands (i.e., the results of the root operator).
  if (Src->isLeaf()) {
    Init *SrcInit = Src->getLeafValue();
    if (isa<IntInit>(SrcInit)) {
      InsnMatcher.addPredicate<InstructionOpcodeMatcher>(
          &Target.getInstruction(RK.getDef("G_CONSTANT")));
    } else
      return failedImport(
          "Unable to deduce gMIR opcode to handle Src (which is a leaf)");
  } else {
    SrcGIEquivOrNull = findNodeEquiv(Src->getOperator());
    if (!SrcGIEquivOrNull)
      return failedImport("Pattern operator lacks an equivalent Instruction" +
                          explainOperator(Src->getOperator()));
    SrcGIOrNull = getEquivNode(*SrcGIEquivOrNull, Src);

    // The operators look good: match the opcode
    InsnMatcher.addPredicate<InstructionOpcodeMatcher>(SrcGIOrNull);
  }

  unsigned OpIdx = 0;
  for (const TypeSetByHwMode &VTy : Src->getExtTypes()) {
    // Results don't have a name unless they are the root node. The caller will
    // set the name if appropriate.
    const bool OperandIsAPointer =
        SrcGIOrNull && SrcGIOrNull->isOutOperandAPointer(OpIdx);
    OperandMatcher &OM = InsnMatcher.addOperand(OpIdx++, "", TempOpIdx);
    if (auto Error = OM.addTypeCheckPredicate(VTy, OperandIsAPointer))
      return failedImport(toString(std::move(Error)) +
                          " for result of Src pattern operator");
  }

  for (const TreePredicateCall &Call : Src->getPredicateCalls()) {
    const TreePredicateFn &Predicate = Call.Fn;
    bool HasAddedBuiltinMatcher = true;
    if (Predicate.isAlwaysTrue())
      continue;

    if (Predicate.isImmediatePattern()) {
      InsnMatcher.addPredicate<InstructionImmPredicateMatcher>(Predicate);
      continue;
    }

    auto InsnMatcherOrError = addBuiltinPredicates(
        SrcGIEquivOrNull, Predicate, InsnMatcher, HasAddedBuiltinMatcher);
    if (auto Error = InsnMatcherOrError.takeError())
      return std::move(Error);

    // FIXME: This should be part of addBuiltinPredicates(). If we add this at
    // the start of addBuiltinPredicates() without returning, then there might
    // be cases where we hit the last return before which the
    // HasAddedBuiltinMatcher will be set to false. The predicate could be
    // missed if we add it in the middle or at the end due to return statements
    // after the addPredicate<>() calls.
    if (Predicate.hasNoUse()) {
      InsnMatcher.addPredicate<NoUsePredicateMatcher>();
      HasAddedBuiltinMatcher = true;
    }

    if (Predicate.hasGISelPredicateCode()) {
      if (Predicate.usesOperands()) {
        assert(WaitingForNamedOperands == 0 &&
               "previous predicate didn't find all operands or "
               "nested predicate that uses operands");
        TreePattern *TP = Predicate.getOrigPatFragRecord();
        WaitingForNamedOperands = TP->getNumArgs();
        for (unsigned i = 0; i < WaitingForNamedOperands; ++i)
          StoreIdxForName[getScopedName(Call.Scope, TP->getArgName(i))] = i;
      }
      InsnMatcher.addPredicate<GenericInstructionPredicateMatcher>(Predicate);
      continue;
    }
    if (!HasAddedBuiltinMatcher) {
      return failedImport("Src pattern child has predicate (" +
                          explainPredicates(Src) + ")");
    }
  }

  bool IsAtomic = false;
  if (SrcGIEquivOrNull && SrcGIEquivOrNull->getValueAsBit("CheckMMOIsNonAtomic"))
    InsnMatcher.addPredicate<AtomicOrderingMMOPredicateMatcher>("NotAtomic");
  else if (SrcGIEquivOrNull && SrcGIEquivOrNull->getValueAsBit("CheckMMOIsAtomic")) {
    IsAtomic = true;
    InsnMatcher.addPredicate<AtomicOrderingMMOPredicateMatcher>(
      "Unordered", AtomicOrderingMMOPredicateMatcher::AO_OrStronger);
  }

  if (Src->isLeaf()) {
    Init *SrcInit = Src->getLeafValue();
    if (IntInit *SrcIntInit = dyn_cast<IntInit>(SrcInit)) {
      OperandMatcher &OM =
          InsnMatcher.addOperand(OpIdx++, Src->getName(), TempOpIdx);
      OM.addPredicate<LiteralIntOperandMatcher>(SrcIntInit->getValue());
    } else
      return failedImport(
          "Unable to deduce gMIR opcode to handle Src (which is a leaf)");
  } else {
    assert(SrcGIOrNull &&
           "Expected to have already found an equivalent Instruction");
    if (SrcGIOrNull->TheDef->getName() == "G_CONSTANT" ||
        SrcGIOrNull->TheDef->getName() == "G_FCONSTANT") {
      // imm/fpimm still have operands but we don't need to do anything with it
      // here since we don't support ImmLeaf predicates yet. However, we still
      // need to note the hidden operand to get GIM_CheckNumOperands correct.
      InsnMatcher.addOperand(OpIdx++, "", TempOpIdx);
      return InsnMatcher;
    }

    // Special case because the operand order is changed from setcc. The
    // predicate operand needs to be swapped from the last operand to the first
    // source.

    unsigned NumChildren = Src->getNumChildren();
    bool IsFCmp = SrcGIOrNull->TheDef->getName() == "G_FCMP";

    if (IsFCmp || SrcGIOrNull->TheDef->getName() == "G_ICMP") {
      const TreePatternNode *SrcChild = Src->getChild(NumChildren - 1);
      if (SrcChild->isLeaf()) {
        DefInit *DI = dyn_cast<DefInit>(SrcChild->getLeafValue());
        Record *CCDef = DI ? DI->getDef() : nullptr;
        if (!CCDef || !CCDef->isSubClassOf("CondCode"))
          return failedImport("Unable to handle CondCode");

        OperandMatcher &OM =
          InsnMatcher.addOperand(OpIdx++, SrcChild->getName(), TempOpIdx);
        StringRef PredType = IsFCmp ? CCDef->getValueAsString("FCmpPredicate") :
                                      CCDef->getValueAsString("ICmpPredicate");

        if (!PredType.empty()) {
          OM.addPredicate<CmpPredicateOperandMatcher>(std::string(PredType));
          // Process the other 2 operands normally.
          --NumChildren;
        }
      }
    }

    // Hack around an unfortunate mistake in how atomic store (and really
    // atomicrmw in general) operands were ordered. A ISD::STORE used the order
    // <stored value>, <pointer> order. ISD::ATOMIC_STORE used the opposite,
    // <pointer>, <stored value>. In GlobalISel there's just the one store
    // opcode, so we need to swap the operands here to get the right type check.
    if (IsAtomic && SrcGIOrNull->TheDef->getName() == "G_STORE") {
      assert(NumChildren == 2 && "wrong operands for atomic store");

      const TreePatternNode *PtrChild = Src->getChild(0);
      const TreePatternNode *ValueChild = Src->getChild(1);

      if (auto Error = importChildMatcher(Rule, InsnMatcher, PtrChild, true,
                                          false, 1, TempOpIdx))
        return std::move(Error);

      if (auto Error = importChildMatcher(Rule, InsnMatcher, ValueChild, false,
                                          false, 0, TempOpIdx))
        return std::move(Error);
      return InsnMatcher;
    }

    // Match the used operands (i.e. the children of the operator).
    bool IsIntrinsic =
        SrcGIOrNull->TheDef->getName() == "G_INTRINSIC" ||
        SrcGIOrNull->TheDef->getName() == "G_INTRINSIC_W_SIDE_EFFECTS";
    const CodeGenIntrinsic *II = Src->getIntrinsicInfo(CGP);
    if (IsIntrinsic && !II)
      return failedImport("Expected IntInit containing intrinsic ID)");

    for (unsigned i = 0; i != NumChildren; ++i) {
      const TreePatternNode *SrcChild = Src->getChild(i);

      // We need to determine the meaning of a literal integer based on the
      // context. If this is a field required to be an immediate (such as an
      // immarg intrinsic argument), the required predicates are different than
      // a constant which may be materialized in a register. If we have an
      // argument that is required to be an immediate, we should not emit an LLT
      // type check, and should not be looking for a G_CONSTANT defined
      // register.
      bool OperandIsImmArg = SrcGIOrNull->isInOperandImmArg(i);

      // SelectionDAG allows pointers to be represented with iN since it doesn't
      // distinguish between pointers and integers but they are different types in GlobalISel.
      // Coerce integers to pointers to address space 0 if the context indicates a pointer.
      //
      bool OperandIsAPointer = SrcGIOrNull->isInOperandAPointer(i);

      if (IsIntrinsic) {
        // For G_INTRINSIC/G_INTRINSIC_W_SIDE_EFFECTS, the operand immediately
        // following the defs is an intrinsic ID.
        if (i == 0) {
          OperandMatcher &OM =
              InsnMatcher.addOperand(OpIdx++, SrcChild->getName(), TempOpIdx);
          OM.addPredicate<IntrinsicIDOperandMatcher>(II);
          continue;
        }

        // We have to check intrinsics for llvm_anyptr_ty and immarg parameters.
        //
        // Note that we have to look at the i-1th parameter, because we don't
        // have the intrinsic ID in the intrinsic's parameter list.
        OperandIsAPointer |= II->isParamAPointer(i - 1);
        OperandIsImmArg |= II->isParamImmArg(i - 1);
      }

      if (auto Error =
              importChildMatcher(Rule, InsnMatcher, SrcChild, OperandIsAPointer,
                                 OperandIsImmArg, OpIdx++, TempOpIdx))
        return std::move(Error);
    }
  }

  return InsnMatcher;
}

Error GlobalISelEmitter::importComplexPatternOperandMatcher(
    OperandMatcher &OM, Record *R, unsigned &TempOpIdx) const {
  const auto &ComplexPattern = ComplexPatternEquivs.find(R);
  if (ComplexPattern == ComplexPatternEquivs.end())
    return failedImport("SelectionDAG ComplexPattern (" + R->getName() +
                        ") not mapped to GlobalISel");

  OM.addPredicate<ComplexPatternOperandMatcher>(OM, *ComplexPattern->second);
  TempOpIdx++;
  return Error::success();
}

// Get the name to use for a pattern operand. For an anonymous physical register
// input, this should use the register name.
static StringRef getSrcChildName(const TreePatternNode *SrcChild,
                                 Record *&PhysReg) {
  StringRef SrcChildName = SrcChild->getName();
  if (SrcChildName.empty() && SrcChild->isLeaf()) {
    if (auto *ChildDefInit = dyn_cast<DefInit>(SrcChild->getLeafValue())) {
      auto *ChildRec = ChildDefInit->getDef();
      if (ChildRec->isSubClassOf("Register")) {
        SrcChildName = ChildRec->getName();
        PhysReg = ChildRec;
      }
    }
  }

  return SrcChildName;
}

Error GlobalISelEmitter::importChildMatcher(
    RuleMatcher &Rule, InstructionMatcher &InsnMatcher,
    const TreePatternNode *SrcChild, bool OperandIsAPointer,
    bool OperandIsImmArg, unsigned OpIdx, unsigned &TempOpIdx) {

  Record *PhysReg = nullptr;
  std::string SrcChildName = std::string(getSrcChildName(SrcChild, PhysReg));
  if (!SrcChild->isLeaf() &&
      SrcChild->getOperator()->isSubClassOf("ComplexPattern")) {
    // The "name" of a non-leaf complex pattern (MY_PAT $op1, $op2) is
    // "MY_PAT:op1:op2" and the ones with same "name" represent same operand.
    std::string PatternName = std::string(SrcChild->getOperator()->getName());
    for (unsigned i = 0; i < SrcChild->getNumChildren(); ++i) {
      PatternName += ":";
      PatternName += SrcChild->getChild(i)->getName();
    }
    SrcChildName = PatternName;
  }

  OperandMatcher &OM =
      PhysReg ? InsnMatcher.addPhysRegInput(PhysReg, OpIdx, TempOpIdx)
              : InsnMatcher.addOperand(OpIdx, SrcChildName, TempOpIdx);
  if (OM.isSameAsAnotherOperand())
    return Error::success();

  ArrayRef<TypeSetByHwMode> ChildTypes = SrcChild->getExtTypes();
  if (ChildTypes.size() != 1)
    return failedImport("Src pattern child has multiple results");

  // Check MBB's before the type check since they are not a known type.
  if (!SrcChild->isLeaf()) {
    if (SrcChild->getOperator()->isSubClassOf("SDNode")) {
      auto &ChildSDNI = CGP.getSDNodeInfo(SrcChild->getOperator());
      if (ChildSDNI.getSDClassName() == "BasicBlockSDNode") {
        OM.addPredicate<MBBOperandMatcher>();
        return Error::success();
      }
      if (SrcChild->getOperator()->getName() == "timm") {
        OM.addPredicate<ImmOperandMatcher>();

        // Add predicates, if any
        for (const TreePredicateCall &Call : SrcChild->getPredicateCalls()) {
          const TreePredicateFn &Predicate = Call.Fn;

          // Only handle immediate patterns for now
          if (Predicate.isImmediatePattern()) {
            OM.addPredicate<OperandImmPredicateMatcher>(Predicate);
          }
        }

        return Error::success();
      }
    }
  }

  // Immediate arguments have no meaningful type to check as they don't have
  // registers.
  if (!OperandIsImmArg) {
    if (auto Error =
            OM.addTypeCheckPredicate(ChildTypes.front(), OperandIsAPointer))
      return failedImport(toString(std::move(Error)) + " for Src operand (" +
                          to_string(*SrcChild) + ")");
  }

  // Check for nested instructions.
  if (!SrcChild->isLeaf()) {
    if (SrcChild->getOperator()->isSubClassOf("ComplexPattern")) {
      // When a ComplexPattern is used as an operator, it should do the same
      // thing as when used as a leaf. However, the children of the operator
      // name the sub-operands that make up the complex operand and we must
      // prepare to reference them in the renderer too.
      unsigned RendererID = TempOpIdx;
      if (auto Error = importComplexPatternOperandMatcher(
              OM, SrcChild->getOperator(), TempOpIdx))
        return Error;

      for (unsigned i = 0, e = SrcChild->getNumChildren(); i != e; ++i) {
        auto *SubOperand = SrcChild->getChild(i);
        if (!SubOperand->getName().empty()) {
          if (auto Error = Rule.defineComplexSubOperand(
                  SubOperand->getName(), SrcChild->getOperator(), RendererID, i,
                  SrcChildName))
            return Error;
        }
      }

      return Error::success();
    }

    auto MaybeInsnOperand = OM.addPredicate<InstructionOperandMatcher>(
        InsnMatcher.getRuleMatcher(), SrcChild->getName());
    if (!MaybeInsnOperand) {
      // This isn't strictly true. If the user were to provide exactly the same
      // matchers as the original operand then we could allow it. However, it's
      // simpler to not permit the redundant specification.
      return failedImport("Nested instruction cannot be the same as another operand");
    }

    // Map the node to a gMIR instruction.
    InstructionOperandMatcher &InsnOperand = **MaybeInsnOperand;
    auto InsnMatcherOrError = createAndImportSelDAGMatcher(
        Rule, InsnOperand.getInsnMatcher(), SrcChild, TempOpIdx);
    if (auto Error = InsnMatcherOrError.takeError())
      return Error;

    return Error::success();
  }

  if (SrcChild->hasAnyPredicate())
    return failedImport("Src pattern child has unsupported predicate");

  // Check for constant immediates.
  if (auto *ChildInt = dyn_cast<IntInit>(SrcChild->getLeafValue())) {
    if (OperandIsImmArg) {
      // Checks for argument directly in operand list
      OM.addPredicate<LiteralIntOperandMatcher>(ChildInt->getValue());
    } else {
      // Checks for materialized constant
      OM.addPredicate<ConstantIntOperandMatcher>(ChildInt->getValue());
    }
    return Error::success();
  }

  // Check for def's like register classes or ComplexPattern's.
  if (auto *ChildDefInit = dyn_cast<DefInit>(SrcChild->getLeafValue())) {
    auto *ChildRec = ChildDefInit->getDef();

    if (WaitingForNamedOperands) {
      auto PA = SrcChild->getNamesAsPredicateArg().begin();
      std::string Name = getScopedName(PA->getScope(), PA->getIdentifier());
      OM.addPredicate<RecordNamedOperandMatcher>(StoreIdxForName[Name], Name);
      --WaitingForNamedOperands;
    }

    // Check for register classes.
    if (ChildRec->isSubClassOf("RegisterClass") ||
        ChildRec->isSubClassOf("RegisterOperand")) {
      OM.addPredicate<RegisterBankOperandMatcher>(
          Target.getRegisterClass(getInitValueAsRegClass(ChildDefInit)));
      return Error::success();
    }

    if (ChildRec->isSubClassOf("Register")) {
      // This just be emitted as a copy to the specific register.
      ValueTypeByHwMode VT = ChildTypes.front().getValueTypeByHwMode();
      const CodeGenRegisterClass *RC
        = CGRegs.getMinimalPhysRegClass(ChildRec, &VT);
      if (!RC) {
        return failedImport(
          "Could not determine physical register class of pattern source");
      }

      OM.addPredicate<RegisterBankOperandMatcher>(*RC);
      return Error::success();
    }

    // Check for ValueType.
    if (ChildRec->isSubClassOf("ValueType")) {
      // We already added a type check as standard practice so this doesn't need
      // to do anything.
      return Error::success();
    }

    // Check for ComplexPattern's.
    if (ChildRec->isSubClassOf("ComplexPattern"))
      return importComplexPatternOperandMatcher(OM, ChildRec, TempOpIdx);

    if (ChildRec->isSubClassOf("ImmLeaf")) {
      return failedImport(
          "Src pattern child def is an unsupported tablegen class (ImmLeaf)");
    }

    // Place holder for SRCVALUE nodes. Nothing to do here.
    if (ChildRec->getName() == "srcvalue")
      return Error::success();

    const bool ImmAllOnesV = ChildRec->getName() == "immAllOnesV";
    if (ImmAllOnesV || ChildRec->getName() == "immAllZerosV") {
      auto MaybeInsnOperand = OM.addPredicate<InstructionOperandMatcher>(
          InsnMatcher.getRuleMatcher(), SrcChild->getName(), false);
      InstructionOperandMatcher &InsnOperand = **MaybeInsnOperand;

      ValueTypeByHwMode VTy = ChildTypes.front().getValueTypeByHwMode();

      const CodeGenInstruction &BuildVector
        = Target.getInstruction(RK.getDef("G_BUILD_VECTOR"));
      const CodeGenInstruction &BuildVectorTrunc
        = Target.getInstruction(RK.getDef("G_BUILD_VECTOR_TRUNC"));

      // Treat G_BUILD_VECTOR as the canonical opcode, and G_BUILD_VECTOR_TRUNC
      // as an alternative.
      InsnOperand.getInsnMatcher().addPredicate<InstructionOpcodeMatcher>(
          ArrayRef({&BuildVector, &BuildVectorTrunc}));

      // TODO: Handle both G_BUILD_VECTOR and G_BUILD_VECTOR_TRUNC We could
      // theoretically not emit any opcode check, but getOpcodeMatcher currently
      // has to succeed.
      OperandMatcher &OM =
          InsnOperand.getInsnMatcher().addOperand(0, "", TempOpIdx);
      if (auto Error =
              OM.addTypeCheckPredicate(VTy, false /* OperandIsAPointer */))
        return failedImport(toString(std::move(Error)) +
                            " for result of Src pattern operator");

      InsnOperand.getInsnMatcher().addPredicate<VectorSplatImmPredicateMatcher>(
          ImmAllOnesV ? VectorSplatImmPredicateMatcher::AllOnes
                      : VectorSplatImmPredicateMatcher::AllZeros);
      return Error::success();
    }

    return failedImport(
        "Src pattern child def is an unsupported tablegen class");
  }

  return failedImport("Src pattern child is an unsupported kind");
}

Expected<action_iterator> GlobalISelEmitter::importExplicitUseRenderer(
    action_iterator InsertPt, RuleMatcher &Rule, BuildMIAction &DstMIBuilder,
    const TreePatternNode *DstChild, const TreePatternNode *Src) {

  const auto &SubOperand = Rule.getComplexSubOperand(DstChild->getName());
  if (SubOperand) {
    DstMIBuilder.addRenderer<RenderComplexPatternOperand>(
        *std::get<0>(*SubOperand), DstChild->getName(),
        std::get<1>(*SubOperand), std::get<2>(*SubOperand));
    return InsertPt;
  }

  if (!DstChild->isLeaf()) {
    if (DstChild->getOperator()->isSubClassOf("SDNodeXForm")) {
      auto Child = DstChild->getChild(0);
      auto I = SDNodeXFormEquivs.find(DstChild->getOperator());
      if (I != SDNodeXFormEquivs.end()) {
        Record *XFormOpc = DstChild->getOperator()->getValueAsDef("Opcode");
        if (XFormOpc->getName() == "timm") {
          // If this is a TargetConstant, there won't be a corresponding
          // instruction to transform. Instead, this will refer directly to an
          // operand in an instruction's operand list.
          DstMIBuilder.addRenderer<CustomOperandRenderer>(*I->second,
                                                          Child->getName());
        } else {
          DstMIBuilder.addRenderer<CustomRenderer>(*I->second,
                                                   Child->getName());
        }

        return InsertPt;
      }
      return failedImport("SDNodeXForm " + Child->getName() +
                          " has no custom renderer");
    }

    // We accept 'bb' here. It's an operator because BasicBlockSDNode isn't
    // inline, but in MI it's just another operand.
    if (DstChild->getOperator()->isSubClassOf("SDNode")) {
      auto &ChildSDNI = CGP.getSDNodeInfo(DstChild->getOperator());
      if (ChildSDNI.getSDClassName() == "BasicBlockSDNode") {
        DstMIBuilder.addRenderer<CopyRenderer>(DstChild->getName());
        return InsertPt;
      }
    }

    // Similarly, imm is an operator in TreePatternNode's view but must be
    // rendered as operands.
    // FIXME: The target should be able to choose sign-extended when appropriate
    //        (e.g. on Mips).
    if (DstChild->getOperator()->getName() == "timm") {
      DstMIBuilder.addRenderer<CopyRenderer>(DstChild->getName());
      return InsertPt;
    } else if (DstChild->getOperator()->getName() == "imm") {
      DstMIBuilder.addRenderer<CopyConstantAsImmRenderer>(DstChild->getName());
      return InsertPt;
    } else if (DstChild->getOperator()->getName() == "fpimm") {
      DstMIBuilder.addRenderer<CopyFConstantAsFPImmRenderer>(
          DstChild->getName());
      return InsertPt;
    }

    if (DstChild->getOperator()->isSubClassOf("Instruction")) {
      auto OpTy = getInstResultType(DstChild);
      if (!OpTy)
        return OpTy.takeError();

      unsigned TempRegID = Rule.allocateTempRegID();
      InsertPt = Rule.insertAction<MakeTempRegisterAction>(
          InsertPt, *OpTy, TempRegID);
      DstMIBuilder.addRenderer<TempRegRenderer>(TempRegID);

      auto InsertPtOrError = createAndImportSubInstructionRenderer(
          ++InsertPt, Rule, DstChild, Src, TempRegID);
      if (auto Error = InsertPtOrError.takeError())
        return std::move(Error);
      return InsertPtOrError.get();
    }

    return failedImport("Dst pattern child isn't a leaf node or an MBB" + llvm::to_string(*DstChild));
  }

  // It could be a specific immediate in which case we should just check for
  // that immediate.
  if (const IntInit *ChildIntInit =
          dyn_cast<IntInit>(DstChild->getLeafValue())) {
    DstMIBuilder.addRenderer<ImmRenderer>(ChildIntInit->getValue());
    return InsertPt;
  }

  // Otherwise, we're looking for a bog-standard RegisterClass operand.
  if (auto *ChildDefInit = dyn_cast<DefInit>(DstChild->getLeafValue())) {
    auto *ChildRec = ChildDefInit->getDef();

    ArrayRef<TypeSetByHwMode> ChildTypes = DstChild->getExtTypes();
    if (ChildTypes.size() != 1)
      return failedImport("Dst pattern child has multiple results");

    std::optional<LLTCodeGen> OpTyOrNone;
    if (ChildTypes.front().isMachineValueType())
      OpTyOrNone = MVTToLLT(ChildTypes.front().getMachineValueType().SimpleTy);
    if (!OpTyOrNone)
      return failedImport("Dst operand has an unsupported type");

    if (ChildRec->isSubClassOf("Register")) {
      DstMIBuilder.addRenderer<AddRegisterRenderer>(Target, ChildRec);
      return InsertPt;
    }

    if (ChildRec->isSubClassOf("RegisterClass") ||
        ChildRec->isSubClassOf("RegisterOperand") ||
        ChildRec->isSubClassOf("ValueType")) {
      if (ChildRec->isSubClassOf("RegisterOperand") &&
          !ChildRec->isValueUnset("GIZeroRegister")) {
        DstMIBuilder.addRenderer<CopyOrAddZeroRegRenderer>(
            DstChild->getName(), ChildRec->getValueAsDef("GIZeroRegister"));
        return InsertPt;
      }

      DstMIBuilder.addRenderer<CopyRenderer>(DstChild->getName());
      return InsertPt;
    }

    if (ChildRec->isSubClassOf("SubRegIndex")) {
      CodeGenSubRegIndex *SubIdx = CGRegs.getSubRegIdx(ChildRec);
      DstMIBuilder.addRenderer<ImmRenderer>(SubIdx->EnumValue);
      return InsertPt;
    }

    if (ChildRec->isSubClassOf("ComplexPattern")) {
      const auto &ComplexPattern = ComplexPatternEquivs.find(ChildRec);
      if (ComplexPattern == ComplexPatternEquivs.end())
        return failedImport(
            "SelectionDAG ComplexPattern not mapped to GlobalISel");

      const OperandMatcher &OM = Rule.getOperandMatcher(DstChild->getName());
      DstMIBuilder.addRenderer<RenderComplexPatternOperand>(
          *ComplexPattern->second, DstChild->getName(),
          OM.getAllocatedTemporariesBaseID());
      return InsertPt;
    }

    return failedImport(
        "Dst pattern child def is an unsupported tablegen class");
  }

  // Handle the case where the MVT/register class is omitted in the dest pattern
  // but MVT exists in the source pattern.
  if (isa<UnsetInit>(DstChild->getLeafValue())) {
    for (unsigned NumOp = 0; NumOp < Src->getNumChildren(); NumOp++)
      if (Src->getChild(NumOp)->getName() == DstChild->getName()) {
        DstMIBuilder.addRenderer<CopyRenderer>(Src->getChild(NumOp)->getName());
        return InsertPt;
      }
  }
  return failedImport("Dst pattern child is an unsupported kind");
}

Expected<BuildMIAction &> GlobalISelEmitter::createAndImportInstructionRenderer(
    RuleMatcher &M, InstructionMatcher &InsnMatcher, const TreePatternNode *Src,
    const TreePatternNode *Dst) {
  auto InsertPtOrError = createInstructionRenderer(M.actions_end(), M, Dst);
  if (auto Error = InsertPtOrError.takeError())
    return std::move(Error);

  action_iterator InsertPt = InsertPtOrError.get();
  BuildMIAction &DstMIBuilder = *static_cast<BuildMIAction *>(InsertPt->get());

  for (auto PhysInput : InsnMatcher.getPhysRegInputs()) {
    InsertPt = M.insertAction<BuildMIAction>(
        InsertPt, M.allocateOutputInsnID(),
        &Target.getInstruction(RK.getDef("COPY")));
    BuildMIAction &CopyToPhysRegMIBuilder =
        *static_cast<BuildMIAction *>(InsertPt->get());
    CopyToPhysRegMIBuilder.addRenderer<AddRegisterRenderer>(Target,
                                                            PhysInput.first,
                                                            true);
    CopyToPhysRegMIBuilder.addRenderer<CopyPhysRegRenderer>(PhysInput.first);
  }

  if (auto Error =
          importExplicitDefRenderers(InsertPt, M, DstMIBuilder, Src, Dst)
              .takeError())
    return std::move(Error);

  if (auto Error =
          importExplicitUseRenderers(InsertPt, M, DstMIBuilder, Dst, Src)
              .takeError())
    return std::move(Error);

  return DstMIBuilder;
}

Expected<action_iterator>
GlobalISelEmitter::createAndImportSubInstructionRenderer(
    const action_iterator InsertPt, RuleMatcher &M, const TreePatternNode *Dst,
    const TreePatternNode *Src, unsigned TempRegID) {
  auto InsertPtOrError = createInstructionRenderer(InsertPt, M, Dst);

  // TODO: Assert there's exactly one result.

  if (auto Error = InsertPtOrError.takeError())
    return std::move(Error);

  BuildMIAction &DstMIBuilder =
      *static_cast<BuildMIAction *>(InsertPtOrError.get()->get());

  // Assign the result to TempReg.
  DstMIBuilder.addRenderer<TempRegRenderer>(TempRegID, true);

  InsertPtOrError = importExplicitUseRenderers(InsertPtOrError.get(), M,
                                               DstMIBuilder, Dst, Src);
  if (auto Error = InsertPtOrError.takeError())
    return std::move(Error);

  // We need to make sure that when we import an INSERT_SUBREG as a
  // subinstruction that it ends up being constrained to the correct super
  // register and subregister classes.
  auto OpName = Target.getInstruction(Dst->getOperator()).TheDef->getName();
  if (OpName == "INSERT_SUBREG") {
    auto SubClass = inferRegClassFromPattern(Dst->getChild(1));
    if (!SubClass)
      return failedImport(
          "Cannot infer register class from INSERT_SUBREG operand #1");
    std::optional<const CodeGenRegisterClass *> SuperClass =
        inferSuperRegisterClassForNode(Dst->getExtType(0), Dst->getChild(0),
                                       Dst->getChild(2));
    if (!SuperClass)
      return failedImport(
          "Cannot infer register class for INSERT_SUBREG operand #0");
    // The destination and the super register source of an INSERT_SUBREG must
    // be the same register class.
    M.insertAction<ConstrainOperandToRegClassAction>(
        InsertPt, DstMIBuilder.getInsnID(), 0, **SuperClass);
    M.insertAction<ConstrainOperandToRegClassAction>(
        InsertPt, DstMIBuilder.getInsnID(), 1, **SuperClass);
    M.insertAction<ConstrainOperandToRegClassAction>(
        InsertPt, DstMIBuilder.getInsnID(), 2, **SubClass);
    return InsertPtOrError.get();
  }

  if (OpName == "EXTRACT_SUBREG") {
    // EXTRACT_SUBREG selects into a subregister COPY but unlike most
    // instructions, the result register class is controlled by the
    // subregisters of the operand. As a result, we must constrain the result
    // class rather than check that it's already the right one.
    auto SuperClass = inferRegClassFromPattern(Dst->getChild(0));
    if (!SuperClass)
      return failedImport(
        "Cannot infer register class from EXTRACT_SUBREG operand #0");

    auto SubIdx = inferSubRegIndexForNode(Dst->getChild(1));
    if (!SubIdx)
      return failedImport("EXTRACT_SUBREG child #1 is not a subreg index");

    const auto SrcRCDstRCPair =
      (*SuperClass)->getMatchingSubClassWithSubRegs(CGRegs, *SubIdx);
    assert(SrcRCDstRCPair->second && "Couldn't find a matching subclass");
    M.insertAction<ConstrainOperandToRegClassAction>(
      InsertPt, DstMIBuilder.getInsnID(), 0, *SrcRCDstRCPair->second);
    M.insertAction<ConstrainOperandToRegClassAction>(
      InsertPt, DstMIBuilder.getInsnID(), 1, *SrcRCDstRCPair->first);

    // We're done with this pattern!  It's eligible for GISel emission; return
    // it.
    return InsertPtOrError.get();
  }

  // Similar to INSERT_SUBREG, we also have to handle SUBREG_TO_REG as a
  // subinstruction.
  if (OpName == "SUBREG_TO_REG") {
    auto SubClass = inferRegClassFromPattern(Dst->getChild(1));
    if (!SubClass)
      return failedImport(
        "Cannot infer register class from SUBREG_TO_REG child #1");
    auto SuperClass = inferSuperRegisterClass(Dst->getExtType(0),
                                              Dst->getChild(2));
    if (!SuperClass)
      return failedImport(
        "Cannot infer register class for SUBREG_TO_REG operand #0");
    M.insertAction<ConstrainOperandToRegClassAction>(
      InsertPt, DstMIBuilder.getInsnID(), 0, **SuperClass);
    M.insertAction<ConstrainOperandToRegClassAction>(
      InsertPt, DstMIBuilder.getInsnID(), 2, **SubClass);
    return InsertPtOrError.get();
  }

  if (OpName == "REG_SEQUENCE") {
    auto SuperClass = inferRegClassFromPattern(Dst->getChild(0));
    M.insertAction<ConstrainOperandToRegClassAction>(
      InsertPt, DstMIBuilder.getInsnID(), 0, **SuperClass);

    unsigned Num = Dst->getNumChildren();
    for (unsigned I = 1; I != Num; I += 2) {
      const TreePatternNode *SubRegChild = Dst->getChild(I + 1);

      auto SubIdx = inferSubRegIndexForNode(SubRegChild);
      if (!SubIdx)
        return failedImport("REG_SEQUENCE child is not a subreg index");

      const auto SrcRCDstRCPair =
        (*SuperClass)->getMatchingSubClassWithSubRegs(CGRegs, *SubIdx);
      assert(SrcRCDstRCPair->second && "Couldn't find a matching subclass");
      M.insertAction<ConstrainOperandToRegClassAction>(
        InsertPt, DstMIBuilder.getInsnID(), I, *SrcRCDstRCPair->second);
    }

    return InsertPtOrError.get();
  }

  M.insertAction<ConstrainOperandsToDefinitionAction>(InsertPt,
                                                      DstMIBuilder.getInsnID());
  return InsertPtOrError.get();
}

Expected<action_iterator> GlobalISelEmitter::createInstructionRenderer(
    action_iterator InsertPt, RuleMatcher &M, const TreePatternNode *Dst) {
  Record *DstOp = Dst->getOperator();
  if (!DstOp->isSubClassOf("Instruction")) {
    if (DstOp->isSubClassOf("ValueType"))
      return failedImport(
          "Pattern operator isn't an instruction (it's a ValueType)");
    return failedImport("Pattern operator isn't an instruction");
  }
  CodeGenInstruction *DstI = &Target.getInstruction(DstOp);

  // COPY_TO_REGCLASS is just a copy with a ConstrainOperandToRegClassAction
  // attached. Similarly for EXTRACT_SUBREG except that's a subregister copy.
  StringRef Name = DstI->TheDef->getName();
  if (Name == "COPY_TO_REGCLASS" || Name == "EXTRACT_SUBREG")
    DstI = &Target.getInstruction(RK.getDef("COPY"));

  return M.insertAction<BuildMIAction>(InsertPt, M.allocateOutputInsnID(),
                                       DstI);
}

Expected<action_iterator> GlobalISelEmitter::importExplicitDefRenderers(
    action_iterator InsertPt, RuleMatcher &M, BuildMIAction &DstMIBuilder,
    const TreePatternNode *Src, const TreePatternNode *Dst) {
  const CodeGenInstruction *DstI = DstMIBuilder.getCGI();
  const unsigned SrcNumDefs = Src->getExtTypes().size();
  const unsigned DstNumDefs = DstI->Operands.NumDefs;
  if (DstNumDefs == 0)
    return InsertPt;

  for (unsigned I = 0; I < SrcNumDefs; ++I)
    DstMIBuilder.addRenderer<CopyRenderer>(DstI->Operands[I].Name);

  // Some instructions have multiple defs, but are missing a type entry
  // (e.g. s_cc_out operands).
  if (Dst->getExtTypes().size() < DstNumDefs)
    return failedImport("unhandled discarded def");

  for (unsigned I = SrcNumDefs; I < DstNumDefs; ++I) {
    const TypeSetByHwMode &ExtTy = Dst->getExtType(I);
    if (!ExtTy.isMachineValueType())
      return failedImport("unsupported typeset");

    auto OpTy = MVTToLLT(ExtTy.getMachineValueType().SimpleTy);
    if (!OpTy)
      return failedImport("unsupported type");

    unsigned TempRegID = M.allocateTempRegID();
    InsertPt =
      M.insertAction<MakeTempRegisterAction>(InsertPt, *OpTy, TempRegID);
    DstMIBuilder.addRenderer<TempRegRenderer>(TempRegID, true, nullptr, true);
  }

  return InsertPt;
}

Expected<action_iterator> GlobalISelEmitter::importExplicitUseRenderers(
    action_iterator InsertPt, RuleMatcher &M, BuildMIAction &DstMIBuilder,
    const llvm::TreePatternNode *Dst, const llvm::TreePatternNode *Src) {
  const CodeGenInstruction *DstI = DstMIBuilder.getCGI();
  CodeGenInstruction *OrigDstI = &Target.getInstruction(Dst->getOperator());

  StringRef Name = OrigDstI->TheDef->getName();
  unsigned ExpectedDstINumUses = Dst->getNumChildren();

  // EXTRACT_SUBREG needs to use a subregister COPY.
  if (Name == "EXTRACT_SUBREG") {
    if (!Dst->getChild(1)->isLeaf())
      return failedImport("EXTRACT_SUBREG child #1 is not a leaf");
    DefInit *SubRegInit = dyn_cast<DefInit>(Dst->getChild(1)->getLeafValue());
    if (!SubRegInit)
      return failedImport("EXTRACT_SUBREG child #1 is not a subreg index");

    CodeGenSubRegIndex *SubIdx = CGRegs.getSubRegIdx(SubRegInit->getDef());
    const TreePatternNode *ValChild = Dst->getChild(0);
    if (!ValChild->isLeaf()) {
      // We really have to handle the source instruction, and then insert a
      // copy from the subregister.
      auto ExtractSrcTy = getInstResultType(ValChild);
      if (!ExtractSrcTy)
        return ExtractSrcTy.takeError();

      unsigned TempRegID = M.allocateTempRegID();
      InsertPt = M.insertAction<MakeTempRegisterAction>(
        InsertPt, *ExtractSrcTy, TempRegID);

      auto InsertPtOrError = createAndImportSubInstructionRenderer(
        ++InsertPt, M, ValChild, Src, TempRegID);
      if (auto Error = InsertPtOrError.takeError())
        return std::move(Error);

      DstMIBuilder.addRenderer<TempRegRenderer>(TempRegID, false, SubIdx);
      return InsertPt;
    }

    // If this is a source operand, this is just a subregister copy.
    Record *RCDef = getInitValueAsRegClass(ValChild->getLeafValue());
    if (!RCDef)
      return failedImport("EXTRACT_SUBREG child #0 could not "
                          "be coerced to a register class");

    CodeGenRegisterClass *RC = CGRegs.getRegClass(RCDef);

    const auto SrcRCDstRCPair =
      RC->getMatchingSubClassWithSubRegs(CGRegs, SubIdx);
    if (SrcRCDstRCPair) {
      assert(SrcRCDstRCPair->second && "Couldn't find a matching subclass");
      if (SrcRCDstRCPair->first != RC)
        return failedImport("EXTRACT_SUBREG requires an additional COPY");
    }

    StringRef RegOperandName = Dst->getChild(0)->getName();
    if (const auto &SubOperand = M.getComplexSubOperand(RegOperandName)) {
      DstMIBuilder.addRenderer<RenderComplexPatternOperand>(
          *std::get<0>(*SubOperand), RegOperandName, std::get<1>(*SubOperand),
          std::get<2>(*SubOperand), SubIdx);
      return InsertPt;
    }

    DstMIBuilder.addRenderer<CopySubRegRenderer>(RegOperandName, SubIdx);
    return InsertPt;
  }

  if (Name == "REG_SEQUENCE") {
    if (!Dst->getChild(0)->isLeaf())
      return failedImport("REG_SEQUENCE child #0 is not a leaf");

    Record *RCDef = getInitValueAsRegClass(Dst->getChild(0)->getLeafValue());
    if (!RCDef)
      return failedImport("REG_SEQUENCE child #0 could not "
                          "be coerced to a register class");

    if ((ExpectedDstINumUses - 1) % 2 != 0)
      return failedImport("Malformed REG_SEQUENCE");

    for (unsigned I = 1; I != ExpectedDstINumUses; I += 2) {
      const TreePatternNode *ValChild = Dst->getChild(I);
      const TreePatternNode *SubRegChild = Dst->getChild(I + 1);

      if (DefInit *SubRegInit =
              dyn_cast<DefInit>(SubRegChild->getLeafValue())) {
        CodeGenSubRegIndex *SubIdx = CGRegs.getSubRegIdx(SubRegInit->getDef());

        auto InsertPtOrError =
            importExplicitUseRenderer(InsertPt, M, DstMIBuilder, ValChild, Src);
        if (auto Error = InsertPtOrError.takeError())
          return std::move(Error);
        InsertPt = InsertPtOrError.get();
        DstMIBuilder.addRenderer<SubRegIndexRenderer>(SubIdx);
      }
    }

    return InsertPt;
  }

  // Render the explicit uses.
  unsigned DstINumUses = OrigDstI->Operands.size() - OrigDstI->Operands.NumDefs;
  if (Name == "COPY_TO_REGCLASS") {
    DstINumUses--; // Ignore the class constraint.
    ExpectedDstINumUses--;
  }

  // NumResults - This is the number of results produced by the instruction in
  // the "outs" list.
  unsigned NumResults = OrigDstI->Operands.NumDefs;

  // Number of operands we know the output instruction must have. If it is
  // variadic, we could have more operands.
  unsigned NumFixedOperands = DstI->Operands.size();

  // Loop over all of the fixed operands of the instruction pattern, emitting
  // code to fill them all in. The node 'N' usually has number children equal to
  // the number of input operands of the instruction.  However, in cases where
  // there are predicate operands for an instruction, we need to fill in the
  // 'execute always' values. Match up the node operands to the instruction
  // operands to do this.
  unsigned Child = 0;

  // Similarly to the code in TreePatternNode::ApplyTypeConstraints, count the
  // number of operands at the end of the list which have default values.
  // Those can come from the pattern if it provides enough arguments, or be
  // filled in with the default if the pattern hasn't provided them. But any
  // operand with a default value _before_ the last mandatory one will be
  // filled in with their defaults unconditionally.
  unsigned NonOverridableOperands = NumFixedOperands;
  while (NonOverridableOperands > NumResults &&
         CGP.operandHasDefault(DstI->Operands[NonOverridableOperands - 1].Rec))
    --NonOverridableOperands;

  unsigned NumDefaultOps = 0;
  for (unsigned I = 0; I != DstINumUses; ++I) {
    unsigned InstOpNo = DstI->Operands.NumDefs + I;

    // Determine what to emit for this operand.
    Record *OperandNode = DstI->Operands[InstOpNo].Rec;

    // If the operand has default values, introduce them now.
    if (CGP.operandHasDefault(OperandNode) &&
        (InstOpNo < NonOverridableOperands || Child >= Dst->getNumChildren())) {
      // This is a predicate or optional def operand which the pattern has not
      // overridden, or which we aren't letting it override; emit the 'default
      // ops' operands.

      const CGIOperandList::OperandInfo &DstIOperand = DstI->Operands[InstOpNo];
      DagInit *DefaultOps = DstIOperand.Rec->getValueAsDag("DefaultOps");
      if (auto Error = importDefaultOperandRenderers(
            InsertPt, M, DstMIBuilder, DefaultOps))
        return std::move(Error);
      ++NumDefaultOps;
      continue;
    }

    auto InsertPtOrError = importExplicitUseRenderer(InsertPt, M, DstMIBuilder,
                                                     Dst->getChild(Child), Src);
    if (auto Error = InsertPtOrError.takeError())
      return std::move(Error);
    InsertPt = InsertPtOrError.get();
    ++Child;
  }

  if (NumDefaultOps + ExpectedDstINumUses != DstINumUses)
    return failedImport("Expected " + llvm::to_string(DstINumUses) +
                        " used operands but found " +
                        llvm::to_string(ExpectedDstINumUses) +
                        " explicit ones and " + llvm::to_string(NumDefaultOps) +
                        " default ones");

  return InsertPt;
}

Error GlobalISelEmitter::importDefaultOperandRenderers(
    action_iterator InsertPt, RuleMatcher &M, BuildMIAction &DstMIBuilder,
    DagInit *DefaultOps) const {
  for (const auto *DefaultOp : DefaultOps->getArgs()) {
    std::optional<LLTCodeGen> OpTyOrNone;

    // Look through ValueType operators.
    if (const DagInit *DefaultDagOp = dyn_cast<DagInit>(DefaultOp)) {
      if (const DefInit *DefaultDagOperator =
              dyn_cast<DefInit>(DefaultDagOp->getOperator())) {
        if (DefaultDagOperator->getDef()->isSubClassOf("ValueType")) {
          OpTyOrNone = MVTToLLT(getValueType(
                                  DefaultDagOperator->getDef()));
          DefaultOp = DefaultDagOp->getArg(0);
        }
      }
    }

    if (const DefInit *DefaultDefOp = dyn_cast<DefInit>(DefaultOp)) {
      auto Def = DefaultDefOp->getDef();
      if (Def->getName() == "undef_tied_input") {
        unsigned TempRegID = M.allocateTempRegID();
        M.insertAction<MakeTempRegisterAction>(InsertPt, *OpTyOrNone,
                                               TempRegID);
        InsertPt = M.insertAction<BuildMIAction>(
          InsertPt, M.allocateOutputInsnID(),
          &Target.getInstruction(RK.getDef("IMPLICIT_DEF")));
        BuildMIAction &IDMIBuilder = *static_cast<BuildMIAction *>(
          InsertPt->get());
        IDMIBuilder.addRenderer<TempRegRenderer>(TempRegID);
        DstMIBuilder.addRenderer<TempRegRenderer>(TempRegID);
      } else {
        DstMIBuilder.addRenderer<AddRegisterRenderer>(Target, Def);
      }
      continue;
    }

    if (const IntInit *DefaultIntOp = dyn_cast<IntInit>(DefaultOp)) {
      DstMIBuilder.addRenderer<ImmRenderer>(DefaultIntOp->getValue());
      continue;
    }

    return failedImport("Could not add default op");
  }

  return Error::success();
}

Error GlobalISelEmitter::importImplicitDefRenderers(
    BuildMIAction &DstMIBuilder,
    const std::vector<Record *> &ImplicitDefs) const {
  if (!ImplicitDefs.empty())
    return failedImport("Pattern defines a physical register");
  return Error::success();
}

std::optional<const CodeGenRegisterClass *>
GlobalISelEmitter::getRegClassFromLeaf(const TreePatternNode *Leaf) {
  assert(Leaf && "Expected node?");
  assert(Leaf->isLeaf() && "Expected leaf?");
  Record *RCRec = getInitValueAsRegClass(Leaf->getLeafValue());
  if (!RCRec)
    return std::nullopt;
  CodeGenRegisterClass *RC = CGRegs.getRegClass(RCRec);
  if (!RC)
    return std::nullopt;
  return RC;
}

std::optional<const CodeGenRegisterClass *>
GlobalISelEmitter::inferRegClassFromPattern(const TreePatternNode *N) {
  if (!N)
    return std::nullopt;

  if (N->isLeaf())
    return getRegClassFromLeaf(N);

  // We don't have a leaf node, so we have to try and infer something. Check
  // that we have an instruction that we an infer something from.

  // Only handle things that produce a single type.
  if (N->getNumTypes() != 1)
    return std::nullopt;
  Record *OpRec = N->getOperator();

  // We only want instructions.
  if (!OpRec->isSubClassOf("Instruction"))
    return std::nullopt;

  // Don't want to try and infer things when there could potentially be more
  // than one candidate register class.
  auto &Inst = Target.getInstruction(OpRec);
  if (Inst.Operands.NumDefs > 1)
    return std::nullopt;

  // Handle any special-case instructions which we can safely infer register
  // classes from.
  StringRef InstName = Inst.TheDef->getName();
  bool IsRegSequence = InstName == "REG_SEQUENCE";
  if (IsRegSequence || InstName == "COPY_TO_REGCLASS") {
    // If we have a COPY_TO_REGCLASS, then we need to handle it specially. It
    // has the desired register class as the first child.
    const TreePatternNode *RCChild = N->getChild(IsRegSequence ? 0 : 1);
    if (!RCChild->isLeaf())
      return std::nullopt;
    return getRegClassFromLeaf(RCChild);
  }
  if (InstName == "INSERT_SUBREG") {
    const TreePatternNode *Child0 = N->getChild(0);
    assert(Child0->getNumTypes() == 1 && "Unexpected number of types!");
    const TypeSetByHwMode &VTy = Child0->getExtType(0);
    return inferSuperRegisterClassForNode(VTy, Child0, N->getChild(2));
  }
  if (InstName == "EXTRACT_SUBREG") {
    assert(N->getNumTypes() == 1 && "Unexpected number of types!");
    const TypeSetByHwMode &VTy = N->getExtType(0);
    return inferSuperRegisterClass(VTy, N->getChild(1));
  }

  // Handle destination record types that we can safely infer a register class
  // from.
  const auto &DstIOperand = Inst.Operands[0];
  Record *DstIOpRec = DstIOperand.Rec;
  if (DstIOpRec->isSubClassOf("RegisterOperand")) {
    DstIOpRec = DstIOpRec->getValueAsDef("RegClass");
    const CodeGenRegisterClass &RC = Target.getRegisterClass(DstIOpRec);
    return &RC;
  }

  if (DstIOpRec->isSubClassOf("RegisterClass")) {
    const CodeGenRegisterClass &RC = Target.getRegisterClass(DstIOpRec);
    return &RC;
  }

  return std::nullopt;
}

std::optional<const CodeGenRegisterClass *>
GlobalISelEmitter::inferSuperRegisterClass(
    const TypeSetByHwMode &Ty, const TreePatternNode *SubRegIdxNode) {
  assert(SubRegIdxNode && "Expected subregister index node!");
  // We need a ValueTypeByHwMode for getSuperRegForSubReg.
  if (!Ty.isValueTypeByHwMode(false))
    return std::nullopt;
  if (!SubRegIdxNode->isLeaf())
    return std::nullopt;
  DefInit *SubRegInit = dyn_cast<DefInit>(SubRegIdxNode->getLeafValue());
  if (!SubRegInit)
    return std::nullopt;
  CodeGenSubRegIndex *SubIdx = CGRegs.getSubRegIdx(SubRegInit->getDef());

  // Use the information we found above to find a minimal register class which
  // supports the subregister and type we want.
  auto RC =
      Target.getSuperRegForSubReg(Ty.getValueTypeByHwMode(), CGRegs, SubIdx,
                                  /* MustBeAllocatable */ true);
  if (!RC)
    return std::nullopt;
  return *RC;
}

std::optional<const CodeGenRegisterClass *>
GlobalISelEmitter::inferSuperRegisterClassForNode(
    const TypeSetByHwMode &Ty, const TreePatternNode *SuperRegNode,
    const TreePatternNode *SubRegIdxNode) {
  assert(SuperRegNode && "Expected super register node!");
  // Check if we already have a defined register class for the super register
  // node. If we do, then we should preserve that rather than inferring anything
  // from the subregister index node. We can assume that whoever wrote the
  // pattern in the first place made sure that the super register and
  // subregister are compatible.
  if (std::optional<const CodeGenRegisterClass *> SuperRegisterClass =
          inferRegClassFromPattern(SuperRegNode))
    return *SuperRegisterClass;
  return inferSuperRegisterClass(Ty, SubRegIdxNode);
}

std::optional<CodeGenSubRegIndex *> GlobalISelEmitter::inferSubRegIndexForNode(
    const TreePatternNode *SubRegIdxNode) {
  if (!SubRegIdxNode->isLeaf())
    return std::nullopt;

  DefInit *SubRegInit = dyn_cast<DefInit>(SubRegIdxNode->getLeafValue());
  if (!SubRegInit)
    return std::nullopt;
  return CGRegs.getSubRegIdx(SubRegInit->getDef());
}

Expected<RuleMatcher> GlobalISelEmitter::runOnPattern(const PatternToMatch &P) {
  // Keep track of the matchers and actions to emit.
  int Score = P.getPatternComplexity(CGP);
  RuleMatcher M(P.getSrcRecord()->getLoc());
  RuleMatcherScores[M.getRuleID()] = Score;
  M.addAction<DebugCommentAction>(llvm::to_string(*P.getSrcPattern()) +
                                  "  =>  " +
                                  llvm::to_string(*P.getDstPattern()));

  SmallVector<Record *, 4> Predicates;
  P.getPredicateRecords(Predicates);
  if (auto Error = importRulePredicates(M, Predicates))
    return std::move(Error);

  // Next, analyze the pattern operators.
  TreePatternNode *Src = P.getSrcPattern();
  TreePatternNode *Dst = P.getDstPattern();

  // If the root of either pattern isn't a simple operator, ignore it.
  if (auto Err = isTrivialOperatorNode(Dst))
    return failedImport("Dst pattern root isn't a trivial operator (" +
                        toString(std::move(Err)) + ")");
  if (auto Err = isTrivialOperatorNode(Src))
    return failedImport("Src pattern root isn't a trivial operator (" +
                        toString(std::move(Err)) + ")");

  // The different predicates and matchers created during
  // addInstructionMatcher use the RuleMatcher M to set up their
  // instruction ID (InsnVarID) that are going to be used when
  // M is going to be emitted.
  // However, the code doing the emission still relies on the IDs
  // returned during that process by the RuleMatcher when issuing
  // the recordInsn opcodes.
  // Because of that:
  // 1. The order in which we created the predicates
  //    and such must be the same as the order in which we emit them,
  //    and
  // 2. We need to reset the generation of the IDs in M somewhere between
  //    addInstructionMatcher and emit
  //
  // FIXME: Long term, we don't want to have to rely on this implicit
  // naming being the same. One possible solution would be to have
  // explicit operator for operation capture and reference those.
  // The plus side is that it would expose opportunities to share
  // the capture accross rules. The downside is that it would
  // introduce a dependency between predicates (captures must happen
  // before their first use.)
  InstructionMatcher &InsnMatcherTemp = M.addInstructionMatcher(Src->getName());
  unsigned TempOpIdx = 0;

  const auto SavedFlags = M.setGISelFlags(P.getSrcRecord());

  auto InsnMatcherOrError =
      createAndImportSelDAGMatcher(M, InsnMatcherTemp, Src, TempOpIdx);
  if (auto Error = InsnMatcherOrError.takeError())
    return std::move(Error);
  InstructionMatcher &InsnMatcher = InsnMatcherOrError.get();

  if (Dst->isLeaf()) {
    Record *RCDef = getInitValueAsRegClass(Dst->getLeafValue());
    if (RCDef) {
      const CodeGenRegisterClass &RC = Target.getRegisterClass(RCDef);

      // We need to replace the def and all its uses with the specified
      // operand. However, we must also insert COPY's wherever needed.
      // For now, emit a copy and let the register allocator clean up.
      auto &DstI = Target.getInstruction(RK.getDef("COPY"));
      const auto &DstIOperand = DstI.Operands[0];

      OperandMatcher &OM0 = InsnMatcher.getOperand(0);
      OM0.setSymbolicName(DstIOperand.Name);
      M.defineOperand(OM0.getSymbolicName(), OM0);
      OM0.addPredicate<RegisterBankOperandMatcher>(RC);

      auto &DstMIBuilder =
          M.addAction<BuildMIAction>(M.allocateOutputInsnID(), &DstI);
      DstMIBuilder.addRenderer<CopyRenderer>(DstIOperand.Name);
      DstMIBuilder.addRenderer<CopyRenderer>(Dst->getName());
      M.addAction<ConstrainOperandToRegClassAction>(0, 0, RC);

      // We're done with this pattern!  It's eligible for GISel emission; return
      // it.
      ++NumPatternImported;
      return std::move(M);
    }

    return failedImport("Dst pattern root isn't a known leaf");
  }

  // Start with the defined operands (i.e., the results of the root operator).
  Record *DstOp = Dst->getOperator();
  if (!DstOp->isSubClassOf("Instruction"))
    return failedImport("Pattern operator isn't an instruction");

  auto &DstI = Target.getInstruction(DstOp);
  StringRef DstIName = DstI.TheDef->getName();

  unsigned DstNumDefs = DstI.Operands.NumDefs,
           SrcNumDefs = Src->getExtTypes().size();
  if (DstNumDefs < SrcNumDefs) {
    if (DstNumDefs != 0)
      return failedImport("Src pattern result has more defs than dst MI (" +
                          to_string(SrcNumDefs) + " def(s) vs " +
                          to_string(DstNumDefs) + " def(s))");

    bool FoundNoUsePred = false;
    for (const auto &Pred : InsnMatcher.predicates()) {
      if ((FoundNoUsePred = isa<NoUsePredicateMatcher>(Pred.get())))
        break;
    }
    if (!FoundNoUsePred)
      return failedImport("Src pattern result has " + to_string(SrcNumDefs) +
                          " def(s) without the HasNoUse predicate set to true "
                          "but Dst MI has no def");
  }

  // The root of the match also has constraints on the register bank so that it
  // matches the result instruction.
  unsigned OpIdx = 0;
  unsigned N = std::min(DstNumDefs, SrcNumDefs);
  for (unsigned I = 0; I < N; ++I) {
    const TypeSetByHwMode &VTy = Src->getExtType(I);

    const auto &DstIOperand = DstI.Operands[OpIdx];
    Record *DstIOpRec = DstIOperand.Rec;
    if (DstIName == "COPY_TO_REGCLASS") {
      DstIOpRec = getInitValueAsRegClass(Dst->getChild(1)->getLeafValue());

      if (DstIOpRec == nullptr)
        return failedImport(
            "COPY_TO_REGCLASS operand #1 isn't a register class");
    } else if (DstIName == "REG_SEQUENCE") {
      DstIOpRec = getInitValueAsRegClass(Dst->getChild(0)->getLeafValue());
      if (DstIOpRec == nullptr)
        return failedImport("REG_SEQUENCE operand #0 isn't a register class");
    } else if (DstIName == "EXTRACT_SUBREG") {
      auto InferredClass = inferRegClassFromPattern(Dst->getChild(0));
      if (!InferredClass)
        return failedImport("Could not infer class for EXTRACT_SUBREG operand #0");

      // We can assume that a subregister is in the same bank as it's super
      // register.
      DstIOpRec = (*InferredClass)->getDef();
    } else if (DstIName == "INSERT_SUBREG") {
      auto MaybeSuperClass = inferSuperRegisterClassForNode(
          VTy, Dst->getChild(0), Dst->getChild(2));
      if (!MaybeSuperClass)
        return failedImport(
            "Cannot infer register class for INSERT_SUBREG operand #0");
      // Move to the next pattern here, because the register class we found
      // doesn't necessarily have a record associated with it. So, we can't
      // set DstIOpRec using this.
      OperandMatcher &OM = InsnMatcher.getOperand(OpIdx);
      OM.setSymbolicName(DstIOperand.Name);
      M.defineOperand(OM.getSymbolicName(), OM);
      OM.addPredicate<RegisterBankOperandMatcher>(**MaybeSuperClass);
      ++OpIdx;
      continue;
    } else if (DstIName == "SUBREG_TO_REG") {
      auto MaybeRegClass = inferSuperRegisterClass(VTy, Dst->getChild(2));
      if (!MaybeRegClass)
        return failedImport(
            "Cannot infer register class for SUBREG_TO_REG operand #0");
      OperandMatcher &OM = InsnMatcher.getOperand(OpIdx);
      OM.setSymbolicName(DstIOperand.Name);
      M.defineOperand(OM.getSymbolicName(), OM);
      OM.addPredicate<RegisterBankOperandMatcher>(**MaybeRegClass);
      ++OpIdx;
      continue;
    } else if (DstIOpRec->isSubClassOf("RegisterOperand"))
      DstIOpRec = DstIOpRec->getValueAsDef("RegClass");
    else if (!DstIOpRec->isSubClassOf("RegisterClass"))
      return failedImport("Dst MI def isn't a register class" +
                          to_string(*Dst));

    OperandMatcher &OM = InsnMatcher.getOperand(OpIdx);
    OM.setSymbolicName(DstIOperand.Name);
    M.defineOperand(OM.getSymbolicName(), OM);
    OM.addPredicate<RegisterBankOperandMatcher>(
        Target.getRegisterClass(DstIOpRec));
    ++OpIdx;
  }

  auto DstMIBuilderOrError =
      createAndImportInstructionRenderer(M, InsnMatcher, Src, Dst);
  if (auto Error = DstMIBuilderOrError.takeError())
    return std::move(Error);
  BuildMIAction &DstMIBuilder = DstMIBuilderOrError.get();

  // Render the implicit defs.
  // These are only added to the root of the result.
  if (auto Error = importImplicitDefRenderers(DstMIBuilder, P.getDstRegs()))
    return std::move(Error);

  DstMIBuilder.chooseInsnToMutate(M);

  // Constrain the registers to classes. This is normally derived from the
  // emitted instruction but a few instructions require special handling.
  if (DstIName == "COPY_TO_REGCLASS") {
    // COPY_TO_REGCLASS does not provide operand constraints itself but the
    // result is constrained to the class given by the second child.
    Record *DstIOpRec =
        getInitValueAsRegClass(Dst->getChild(1)->getLeafValue());

    if (DstIOpRec == nullptr)
      return failedImport("COPY_TO_REGCLASS operand #1 isn't a register class");

    M.addAction<ConstrainOperandToRegClassAction>(
        0, 0, Target.getRegisterClass(DstIOpRec));

    // We're done with this pattern!  It's eligible for GISel emission; return
    // it.
    ++NumPatternImported;
    return std::move(M);
  }

  if (DstIName == "EXTRACT_SUBREG") {
    auto SuperClass = inferRegClassFromPattern(Dst->getChild(0));
    if (!SuperClass)
      return failedImport(
        "Cannot infer register class from EXTRACT_SUBREG operand #0");

    auto SubIdx = inferSubRegIndexForNode(Dst->getChild(1));
    if (!SubIdx)
      return failedImport("EXTRACT_SUBREG child #1 is not a subreg index");

    // It would be nice to leave this constraint implicit but we're required
    // to pick a register class so constrain the result to a register class
    // that can hold the correct MVT.
    //
    // FIXME: This may introduce an extra copy if the chosen class doesn't
    //        actually contain the subregisters.
    assert(Src->getExtTypes().size() == 1 &&
             "Expected Src of EXTRACT_SUBREG to have one result type");

    const auto SrcRCDstRCPair =
      (*SuperClass)->getMatchingSubClassWithSubRegs(CGRegs, *SubIdx);
    if (!SrcRCDstRCPair) {
      return failedImport("subreg index is incompatible "
                          "with inferred reg class");
    }

    assert(SrcRCDstRCPair->second && "Couldn't find a matching subclass");
    M.addAction<ConstrainOperandToRegClassAction>(0, 0, *SrcRCDstRCPair->second);
    M.addAction<ConstrainOperandToRegClassAction>(0, 1, *SrcRCDstRCPair->first);

    // We're done with this pattern!  It's eligible for GISel emission; return
    // it.
    ++NumPatternImported;
    return std::move(M);
  }

  if (DstIName == "INSERT_SUBREG") {
    assert(Src->getExtTypes().size() == 1 &&
           "Expected Src of INSERT_SUBREG to have one result type");
    // We need to constrain the destination, a super regsister source, and a
    // subregister source.
    auto SubClass = inferRegClassFromPattern(Dst->getChild(1));
    if (!SubClass)
      return failedImport(
          "Cannot infer register class from INSERT_SUBREG operand #1");
    auto SuperClass = inferSuperRegisterClassForNode(
        Src->getExtType(0), Dst->getChild(0), Dst->getChild(2));
    if (!SuperClass)
      return failedImport(
          "Cannot infer register class for INSERT_SUBREG operand #0");
    M.addAction<ConstrainOperandToRegClassAction>(0, 0, **SuperClass);
    M.addAction<ConstrainOperandToRegClassAction>(0, 1, **SuperClass);
    M.addAction<ConstrainOperandToRegClassAction>(0, 2, **SubClass);
    ++NumPatternImported;
    return std::move(M);
  }

  if (DstIName == "SUBREG_TO_REG") {
    // We need to constrain the destination and subregister source.
    assert(Src->getExtTypes().size() == 1 &&
           "Expected Src of SUBREG_TO_REG to have one result type");

    // Attempt to infer the subregister source from the first child. If it has
    // an explicitly given register class, we'll use that. Otherwise, we will
    // fail.
    auto SubClass = inferRegClassFromPattern(Dst->getChild(1));
    if (!SubClass)
      return failedImport(
          "Cannot infer register class from SUBREG_TO_REG child #1");
    // We don't have a child to look at that might have a super register node.
    auto SuperClass =
        inferSuperRegisterClass(Src->getExtType(0), Dst->getChild(2));
    if (!SuperClass)
      return failedImport(
          "Cannot infer register class for SUBREG_TO_REG operand #0");
    M.addAction<ConstrainOperandToRegClassAction>(0, 0, **SuperClass);
    M.addAction<ConstrainOperandToRegClassAction>(0, 2, **SubClass);
    ++NumPatternImported;
    return std::move(M);
  }

  if (DstIName == "REG_SEQUENCE") {
    auto SuperClass = inferRegClassFromPattern(Dst->getChild(0));

    M.addAction<ConstrainOperandToRegClassAction>(0, 0, **SuperClass);

    unsigned Num = Dst->getNumChildren();
    for (unsigned I = 1; I != Num; I += 2) {
      TreePatternNode *SubRegChild = Dst->getChild(I + 1);

      auto SubIdx = inferSubRegIndexForNode(SubRegChild);
      if (!SubIdx)
        return failedImport("REG_SEQUENCE child is not a subreg index");

      const auto SrcRCDstRCPair =
        (*SuperClass)->getMatchingSubClassWithSubRegs(CGRegs, *SubIdx);

      M.addAction<ConstrainOperandToRegClassAction>(0, I,
                                                    *SrcRCDstRCPair->second);
    }

    ++NumPatternImported;
    return std::move(M);
  }

  M.addAction<ConstrainOperandsToDefinitionAction>(0);

  // We're done with this pattern!  It's eligible for GISel emission; return it.
  ++NumPatternImported;
  return std::move(M);
}

// Emit imm predicate table and an enum to reference them with.
// The 'Predicate_' part of the name is redundant but eliminating it is more
// trouble than it's worth.
void GlobalISelEmitter::emitCxxPredicateFns(
    raw_ostream &OS, StringRef CodeFieldName, StringRef TypeIdentifier,
    StringRef ArgType, StringRef ArgName, StringRef AdditionalArgs,
    StringRef AdditionalDeclarations,
    std::function<bool(const Record *R)> Filter) {
  std::vector<const Record *> MatchedRecords;
  const auto &Defs = RK.getAllDerivedDefinitions("PatFrags");
  std::copy_if(Defs.begin(), Defs.end(), std::back_inserter(MatchedRecords),
               [&](Record *Record) {
                 return !Record->getValueAsString(CodeFieldName).empty() &&
                        Filter(Record);
               });

  if (!MatchedRecords.empty()) {
    OS << "// PatFrag predicates.\n"
       << "enum {\n";
    std::string EnumeratorSeparator =
        (" = GIPFP_" + TypeIdentifier + "_Invalid + 1,\n").str();
    for (const auto *Record : MatchedRecords) {
      OS << "  GIPFP_" << TypeIdentifier << "_Predicate_" << Record->getName()
         << EnumeratorSeparator;
      EnumeratorSeparator = ",\n";
    }
    OS << "};\n";
  }

  OS << "bool " << Target.getName() << "InstructionSelector::test" << ArgName
     << "Predicate_" << TypeIdentifier << "(unsigned PredicateID, " << ArgType << " "
     << ArgName << AdditionalArgs <<") const {\n"
     << AdditionalDeclarations;
  if (!AdditionalDeclarations.empty())
    OS << "\n";
  if (!MatchedRecords.empty())
    OS << "  switch (PredicateID) {\n";
  for (const auto *Record : MatchedRecords) {
    OS << "  case GIPFP_" << TypeIdentifier << "_Predicate_"
       << Record->getName() << ": {\n"
       << "    " << Record->getValueAsString(CodeFieldName) << "\n"
       << "    llvm_unreachable(\"" << CodeFieldName
       << " should have returned\");\n"
       << "    return false;\n"
       << "  }\n";
  }
  if (!MatchedRecords.empty())
    OS << "  }\n";
  OS << "  llvm_unreachable(\"Unknown predicate\");\n"
     << "  return false;\n"
     << "}\n";
}

void GlobalISelEmitter::emitImmPredicateFns(
    raw_ostream &OS, StringRef TypeIdentifier, StringRef ArgType,
    std::function<bool(const Record *R)> Filter) {
  return emitCxxPredicateFns(OS, "ImmediateCode", TypeIdentifier, ArgType,
                             "Imm", "", "", Filter);
}

void GlobalISelEmitter::emitMIPredicateFns(raw_ostream &OS) {
  return emitCxxPredicateFns(
      OS, "GISelPredicateCode", "MI", "const MachineInstr &", "MI",
      ", const std::array<const MachineOperand *, 3> &Operands",
      "  const MachineFunction &MF = *MI.getParent()->getParent();\n"
      "  const MachineRegisterInfo &MRI = MF.getRegInfo();\n"
      "  (void)MRI;",
      [](const Record *R) { return true; });
}

MatchTable
GlobalISelEmitter::buildMatchTable(MutableArrayRef<RuleMatcher> Rules,
                                   bool Optimize, bool WithCoverage) {
  std::vector<Matcher *> InputRules;
  for (Matcher &Rule : Rules)
    InputRules.push_back(&Rule);

  if (!Optimize)
    return MatchTable::buildTable(InputRules, WithCoverage);

  unsigned CurrentOrdering = 0;
  StringMap<unsigned> OpcodeOrder;
  for (RuleMatcher &Rule : Rules) {
    const StringRef Opcode = Rule.getOpcode();
    assert(!Opcode.empty() && "Didn't expect an undefined opcode");
    if (OpcodeOrder.count(Opcode) == 0)
      OpcodeOrder[Opcode] = CurrentOrdering++;
  }

  llvm::stable_sort(InputRules, [&OpcodeOrder](const Matcher *A,
                                               const Matcher *B) {
    auto *L = static_cast<const RuleMatcher *>(A);
    auto *R = static_cast<const RuleMatcher *>(B);
    return std::make_tuple(OpcodeOrder[L->getOpcode()], L->getNumOperands()) <
           std::make_tuple(OpcodeOrder[R->getOpcode()], R->getNumOperands());
  });

  for (Matcher *Rule : InputRules)
    Rule->optimize();

  std::vector<std::unique_ptr<Matcher>> MatcherStorage;
  std::vector<Matcher *> OptRules =
      optimizeRules<GroupMatcher>(InputRules, MatcherStorage);

  for (Matcher *Rule : OptRules)
    Rule->optimize();

  OptRules = optimizeRules<SwitchMatcher>(OptRules, MatcherStorage);

  return MatchTable::buildTable(OptRules, WithCoverage);
}

void GlobalISelEmitter::run(raw_ostream &OS) {
  if (!UseCoverageFile.empty()) {
    RuleCoverage = CodeGenCoverage();
    auto RuleCoverageBufOrErr = MemoryBuffer::getFile(UseCoverageFile);
    if (!RuleCoverageBufOrErr) {
      PrintWarning(SMLoc(), "Missing rule coverage data");
      RuleCoverage = std::nullopt;
    } else {
      if (!RuleCoverage->parse(*RuleCoverageBufOrErr.get(), Target.getName())) {
        PrintWarning(SMLoc(), "Ignoring invalid or missing rule coverage data");
        RuleCoverage = std::nullopt;
      }
    }
  }

  // Track the run-time opcode values
  gatherOpcodeValues();
  // Track the run-time LLT ID values
  gatherTypeIDValues();

  // Track the GINodeEquiv definitions.
  gatherNodeEquivs();

  emitSourceFileHeader(("Global Instruction Selector for the " +
                       Target.getName() + " target").str(), OS);
  std::vector<RuleMatcher> Rules;
  // Look through the SelectionDAG patterns we found, possibly emitting some.
  for (const PatternToMatch &Pat : CGP.ptms()) {
    ++NumPatternTotal;

    auto MatcherOrErr = runOnPattern(Pat);

    // The pattern analysis can fail, indicating an unsupported pattern.
    // Report that if we've been asked to do so.
    if (auto Err = MatcherOrErr.takeError()) {
      if (WarnOnSkippedPatterns) {
        PrintWarning(Pat.getSrcRecord()->getLoc(),
                     "Skipped pattern: " + toString(std::move(Err)));
      } else {
        consumeError(std::move(Err));
      }
      ++NumPatternImportsSkipped;
      continue;
    }

    if (RuleCoverage) {
      if (RuleCoverage->isCovered(MatcherOrErr->getRuleID()))
        ++NumPatternsTested;
      else
        PrintWarning(Pat.getSrcRecord()->getLoc(),
                     "Pattern is not covered by a test");
    }
    Rules.push_back(std::move(MatcherOrErr.get()));
  }

  // Comparison function to order records by name.
  auto orderByName = [](const Record *A, const Record *B) {
    return A->getName() < B->getName();
  };

  std::vector<Record *> ComplexPredicates =
      RK.getAllDerivedDefinitions("GIComplexOperandMatcher");
  llvm::sort(ComplexPredicates, orderByName);

  std::vector<StringRef> CustomRendererFns;
  transform(RK.getAllDerivedDefinitions("GICustomOperandRenderer"),
            std::back_inserter(CustomRendererFns), [](const auto &Record) {
              return Record->getValueAsString("RendererFn");
            });
  // Sort and remove duplicates to get a list of unique renderer functions, in
  // case some were mentioned more than once.
  llvm::sort(CustomRendererFns);
  CustomRendererFns.erase(
      std::unique(CustomRendererFns.begin(), CustomRendererFns.end()),
      CustomRendererFns.end());

  unsigned MaxTemporaries = 0;
  for (const auto &Rule : Rules)
    MaxTemporaries = std::max(MaxTemporaries, Rule.countRendererFns());

  OS << "#ifdef GET_GLOBALISEL_PREDICATE_BITSET\n"
     << "const unsigned MAX_SUBTARGET_PREDICATES = " << SubtargetFeatures.size()
     << ";\n"
     << "using PredicateBitset = "
        "llvm::PredicateBitsetImpl<MAX_SUBTARGET_PREDICATES>;\n"
     << "#endif // ifdef GET_GLOBALISEL_PREDICATE_BITSET\n\n";

  OS << "#ifdef GET_GLOBALISEL_TEMPORARIES_DECL\n"
     << "  mutable MatcherState State;\n"
     << "  typedef "
        "ComplexRendererFns("
     << Target.getName()
     << "InstructionSelector::*ComplexMatcherMemFn)(MachineOperand &) const;\n"

     << "  typedef void(" << Target.getName()
     << "InstructionSelector::*CustomRendererFn)(MachineInstrBuilder &, const "
        "MachineInstr &, int) "
        "const;\n"
     << "  const ISelInfoTy<PredicateBitset, ComplexMatcherMemFn, "
        "CustomRendererFn> "
        "ISelInfo;\n";
  OS << "  static " << Target.getName()
     << "InstructionSelector::ComplexMatcherMemFn ComplexPredicateFns[];\n"
     << "  static " << Target.getName()
     << "InstructionSelector::CustomRendererFn CustomRenderers[];\n"
     << "  bool testImmPredicate_I64(unsigned PredicateID, int64_t Imm) const "
        "override;\n"
     << "  bool testImmPredicate_APInt(unsigned PredicateID, const APInt &Imm) "
        "const override;\n"
     << "  bool testImmPredicate_APFloat(unsigned PredicateID, const APFloat "
        "&Imm) const override;\n"
     << "  const int64_t *getMatchTable() const override;\n"
     << "  bool testMIPredicate_MI(unsigned PredicateID, const MachineInstr &MI"
        ", const std::array<const MachineOperand *, 3> &Operands) "
        "const override;\n"
     << "#endif // ifdef GET_GLOBALISEL_TEMPORARIES_DECL\n\n";

  OS << "#ifdef GET_GLOBALISEL_TEMPORARIES_INIT\n"
     << ", State(" << MaxTemporaries << "),\n"
     << "ISelInfo(TypeObjects, NumTypeObjects, FeatureBitsets"
     << ", ComplexPredicateFns, CustomRenderers)\n"
     << "#endif // ifdef GET_GLOBALISEL_TEMPORARIES_INIT\n\n";

  OS << "#ifdef GET_GLOBALISEL_IMPL\n";
  SubtargetFeatureInfo::emitSubtargetFeatureBitEnumeration(SubtargetFeatures,
                                                           OS);

  // Separate subtarget features by how often they must be recomputed.
  SubtargetFeatureInfoMap ModuleFeatures;
  std::copy_if(SubtargetFeatures.begin(), SubtargetFeatures.end(),
               std::inserter(ModuleFeatures, ModuleFeatures.end()),
               [](const SubtargetFeatureInfoMap::value_type &X) {
                 return !X.second.mustRecomputePerFunction();
               });
  SubtargetFeatureInfoMap FunctionFeatures;
  std::copy_if(SubtargetFeatures.begin(), SubtargetFeatures.end(),
               std::inserter(FunctionFeatures, FunctionFeatures.end()),
               [](const SubtargetFeatureInfoMap::value_type &X) {
                 return X.second.mustRecomputePerFunction();
               });

  SubtargetFeatureInfo::emitComputeAvailableFeatures(
    Target.getName(), "InstructionSelector", "computeAvailableModuleFeatures",
      ModuleFeatures, OS);


  OS << "void " << Target.getName() << "InstructionSelector"
    "::setupGeneratedPerFunctionState(MachineFunction &MF) {\n"
    "  AvailableFunctionFeatures = computeAvailableFunctionFeatures("
    "(const " << Target.getName() << "Subtarget *)&MF.getSubtarget(), &MF);\n"
    "}\n";

  SubtargetFeatureInfo::emitComputeAvailableFeatures(
      Target.getName(), "InstructionSelector",
      "computeAvailableFunctionFeatures", FunctionFeatures, OS,
      "const MachineFunction *MF");

  // Emit a table containing the LLT objects needed by the matcher and an enum
  // for the matcher to reference them with.
  std::vector<LLTCodeGen> TypeObjects;
  append_range(TypeObjects, KnownTypes);
  llvm::sort(TypeObjects);
  OS << "// LLT Objects.\n"
     << "enum {\n";
  for (const auto &TypeObject : TypeObjects) {
    OS << "  ";
    TypeObject.emitCxxEnumValue(OS);
    OS << ",\n";
  }
  OS << "};\n";
  OS << "const static size_t NumTypeObjects = " << TypeObjects.size() << ";\n"
     << "const static LLT TypeObjects[] = {\n";
  for (const auto &TypeObject : TypeObjects) {
    OS << "  ";
    TypeObject.emitCxxConstructorCall(OS);
    OS << ",\n";
  }
  OS << "};\n\n";

  // Emit a table containing the PredicateBitsets objects needed by the matcher
  // and an enum for the matcher to reference them with.
  std::vector<std::vector<Record *>> FeatureBitsets;
  FeatureBitsets.reserve(Rules.size());
  for (auto &Rule : Rules)
    FeatureBitsets.push_back(Rule.getRequiredFeatures());
  llvm::sort(FeatureBitsets, [&](const std::vector<Record *> &A,
                                 const std::vector<Record *> &B) {
    if (A.size() < B.size())
      return true;
    if (A.size() > B.size())
      return false;
    for (auto Pair : zip(A, B)) {
      if (std::get<0>(Pair)->getName() < std::get<1>(Pair)->getName())
        return true;
      if (std::get<0>(Pair)->getName() > std::get<1>(Pair)->getName())
        return false;
    }
    return false;
  });
  FeatureBitsets.erase(
      std::unique(FeatureBitsets.begin(), FeatureBitsets.end()),
      FeatureBitsets.end());
  OS << "// Feature bitsets.\n"
     << "enum {\n"
     << "  GIFBS_Invalid,\n";
  for (const auto &FeatureBitset : FeatureBitsets) {
    if (FeatureBitset.empty())
      continue;
    OS << "  " << getNameForFeatureBitset(FeatureBitset) << ",\n";
  }
  OS << "};\n"
     << "const static PredicateBitset FeatureBitsets[] {\n"
     << "  {}, // GIFBS_Invalid\n";
  for (const auto &FeatureBitset : FeatureBitsets) {
    if (FeatureBitset.empty())
      continue;
    OS << "  {";
    for (const auto &Feature : FeatureBitset) {
      const auto &I = SubtargetFeatures.find(Feature);
      assert(I != SubtargetFeatures.end() && "Didn't import predicate?");
      OS << I->second.getEnumBitName() << ", ";
    }
    OS << "},\n";
  }
  OS << "};\n\n";

  // Emit complex predicate table and an enum to reference them with.
  OS << "// ComplexPattern predicates.\n"
     << "enum {\n"
     << "  GICP_Invalid,\n";
  for (const auto &Record : ComplexPredicates)
    OS << "  GICP_" << Record->getName() << ",\n";
  OS << "};\n"
     << "// See constructor for table contents\n\n";

  emitImmPredicateFns(OS, "I64", "int64_t", [](const Record *R) {
    bool Unset;
    return !R->getValueAsBitOrUnset("IsAPFloat", Unset) &&
           !R->getValueAsBit("IsAPInt");
  });
  emitImmPredicateFns(OS, "APFloat", "const APFloat &", [](const Record *R) {
    bool Unset;
    return R->getValueAsBitOrUnset("IsAPFloat", Unset);
  });
  emitImmPredicateFns(OS, "APInt", "const APInt &", [](const Record *R) {
    return R->getValueAsBit("IsAPInt");
  });
  emitMIPredicateFns(OS);
  OS << "\n";

  OS << Target.getName() << "InstructionSelector::ComplexMatcherMemFn\n"
     << Target.getName() << "InstructionSelector::ComplexPredicateFns[] = {\n"
     << "  nullptr, // GICP_Invalid\n";
  for (const auto &Record : ComplexPredicates)
    OS << "  &" << Target.getName()
       << "InstructionSelector::" << Record->getValueAsString("MatcherFn")
       << ", // " << Record->getName() << "\n";
  OS << "};\n\n";

  OS << "// Custom renderers.\n"
     << "enum {\n"
     << "  GICR_Invalid,\n";
  for (const auto &Fn : CustomRendererFns)
    OS << "  GICR_" << Fn << ",\n";
  OS << "};\n";

  OS << Target.getName() << "InstructionSelector::CustomRendererFn\n"
     << Target.getName() << "InstructionSelector::CustomRenderers[] = {\n"
     << "  nullptr, // GICR_Invalid\n";
  for (const auto &Fn : CustomRendererFns)
    OS << "  &" << Target.getName() << "InstructionSelector::" << Fn << ",\n";
  OS << "};\n\n";

  llvm::stable_sort(Rules, [&](const RuleMatcher &A, const RuleMatcher &B) {
    int ScoreA = RuleMatcherScores[A.getRuleID()];
    int ScoreB = RuleMatcherScores[B.getRuleID()];
    if (ScoreA > ScoreB)
      return true;
    if (ScoreB > ScoreA)
      return false;
    if (A.isHigherPriorityThan(B)) {
      assert(!B.isHigherPriorityThan(A) && "Cannot be more important "
                                           "and less important at "
                                           "the same time");
      return true;
    }
    return false;
  });

  OS << "bool " << Target.getName()
     << "InstructionSelector::selectImpl(MachineInstr &I, CodeGenCoverage "
        "&CoverageInfo) const {\n"
     << "  MachineFunction &MF = *I.getParent()->getParent();\n"
     << "  MachineRegisterInfo &MRI = MF.getRegInfo();\n"
     << "  const PredicateBitset AvailableFeatures = getAvailableFeatures();\n"
     << "  NewMIVector OutMIs;\n"
     << "  State.MIs.clear();\n"
     << "  State.MIs.push_back(&I);\n\n"
     << "  if (executeMatchTable(*this, OutMIs, State, ISelInfo"
     << ", getMatchTable(), TII, MRI, TRI, RBI, AvailableFeatures"
     << ", CoverageInfo)) {\n"
     << "    return true;\n"
     << "  }\n\n"
     << "  return false;\n"
     << "}\n\n";

  const MatchTable Table =
      buildMatchTable(Rules, OptimizeMatchTable, GenerateCoverage);
  OS << "const int64_t *" << Target.getName()
     << "InstructionSelector::getMatchTable() const {\n";
  Table.emitDeclaration(OS);
  OS << "  return ";
  Table.emitUse(OS);
  OS << ";\n}\n";
  OS << "#endif // ifdef GET_GLOBALISEL_IMPL\n";

  OS << "#ifdef GET_GLOBALISEL_PREDICATES_DECL\n"
     << "PredicateBitset AvailableModuleFeatures;\n"
     << "mutable PredicateBitset AvailableFunctionFeatures;\n"
     << "PredicateBitset getAvailableFeatures() const {\n"
     << "  return AvailableModuleFeatures | AvailableFunctionFeatures;\n"
     << "}\n"
     << "PredicateBitset\n"
     << "computeAvailableModuleFeatures(const " << Target.getName()
     << "Subtarget *Subtarget) const;\n"
     << "PredicateBitset\n"
     << "computeAvailableFunctionFeatures(const " << Target.getName()
     << "Subtarget *Subtarget,\n"
     << "                                 const MachineFunction *MF) const;\n"
     << "void setupGeneratedPerFunctionState(MachineFunction &MF) override;\n"
     << "#endif // ifdef GET_GLOBALISEL_PREDICATES_DECL\n";

  OS << "#ifdef GET_GLOBALISEL_PREDICATES_INIT\n"
     << "AvailableModuleFeatures(computeAvailableModuleFeatures(&STI)),\n"
     << "AvailableFunctionFeatures()\n"
     << "#endif // ifdef GET_GLOBALISEL_PREDICATES_INIT\n";
}

void GlobalISelEmitter::declareSubtargetFeature(Record *Predicate) {
  if (SubtargetFeatures.count(Predicate) == 0)
    SubtargetFeatures.emplace(
        Predicate, SubtargetFeatureInfo(Predicate, SubtargetFeatures.size()));
}

} // end anonymous namespace

//===----------------------------------------------------------------------===//

static TableGen::Emitter::OptClass<GlobalISelEmitter>
    X("gen-global-isel", "Generate GlobalISel selector");
