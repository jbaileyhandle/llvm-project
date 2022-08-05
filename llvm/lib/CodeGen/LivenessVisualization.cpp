//===- LivenessVisualization.cpp - Live Interval Analysis -------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
/// \file This file implements the LiveInterval analysis pass which is used
/// by the Linear Scan Register allocator. This pass linearizes the
/// basic blocks of the function in DFS order and computes live intervals for
/// each virtual and physical register.
//
//===----------------------------------------------------------------------===//

#include "LiveDebugVariables.h"
#include "llvm/CodeGen/LivenessVisualization.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/iterator_range.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/CodeGen/LiveInterval.h"
#include "llvm/CodeGen/LiveIntervals.h"
#include "llvm/CodeGen/LiveIntervalCalc.h"
#include "llvm/CodeGen/LiveStacks.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineBlockFrequencyInfo.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBundle.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/CodeGen/SlotIndexes.h"
#include "llvm/CodeGen/StackMaps.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/CodeGen/VirtRegMap.h"
#include "llvm/Config/llvm-config.h"
#include "llvm/IR/Statepoint.h"
#include "llvm/MC/LaneBitmask.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Pass.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iterator>
#include <tuple>
#include <utility>

using namespace llvm;

#define DEBUG_TYPE "livenessvisualization"

char LivenessVisualization::ID = 0;
char &llvm::LivenessVisualizationID = LivenessVisualization::ID;
INITIALIZE_PASS_BEGIN(LivenessVisualization, "livenessvisualization",
                "Live value visualization", false, false)
INITIALIZE_PASS_DEPENDENCY(AAResultsWrapperPass)
INITIALIZE_PASS_DEPENDENCY(MachineDominatorTree)
INITIALIZE_PASS_DEPENDENCY(SlotIndexes)
INITIALIZE_PASS_DEPENDENCY(LiveIntervals)
INITIALIZE_PASS_DEPENDENCY(LiveDebugVariables)
INITIALIZE_PASS_DEPENDENCY(LiveVariables)
INITIALIZE_PASS_END(LivenessVisualization, "livenessvisualization",
                "Live value visualization", false, false)


LivenessVisualization::GraphBB::GraphBB(LivenessVisualization *LVpass, MachineBasicBlock &MBB) {
    name = MBB.getFullName();
    label_ostream_ptr = std::make_unique<raw_string_ostream>(label_str);
    *label_ostream_ptr << name << " [shape=box; label=\"" << name;
}

LivenessVisualization::LivenessVisualization() : MachineFunctionPass(ID) {
  initializeLivenessVisualizationPass(*PassRegistry::getPassRegistry());
}

LivenessVisualization::~LivenessVisualization() {}

void LivenessVisualization::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.setPreservesAll();
  AU.addRequired<LiveIntervals>();
  AU.addPreserved<LiveIntervals>();
  MachineFunctionPass::getAnalysisUsage(AU);
}

bool LivenessVisualization::doInitialization(Module &M) {
    printf("LivenessVisualization doInitialization\n");

    MachineFunctionPass::doInitialization(M);
    return false;
}

bool LivenessVisualization::runOnMachineFunction(MachineFunction &fn) {

    MF = &fn;
    MRI = &MF->getRegInfo();
    LIA = &getAnalysis<LiveIntervals>();
    TRI = MF->getSubtarget().getRegisterInfo();
    TII = MF->getSubtarget().getInstrInfo();
    SlotIndexes *indexes = LIA->getSlotIndexes();

    outs() << "\n\nFunction: " << MF->getName() << "\n";
    /*
    for (MachineBasicBlock &MBB : *MF) {
        std::string bb_str;
        raw_string_ostream bb_stream(bb_str);
        MBB.printAsOperand(bb_stream, false);
        outs() << "\tBB:"  << bb_str << "\n";
        for (MachineInstr &MI : MBB) {
            std::string mi_str;
            raw_string_ostream mi_stream(mi_str);
            if(MI.getDebugLoc().isImplicitCode()) {
                mi_stream << "ImplicitCode";
            } else {
                MI.getDebugLoc().print(mi_stream);
            }
            mi_stream << ":\t";
            MI.print(mi_stream);
            outs() << "\t\t" << mi_str;
        }
    }
    printf("Running on fucntion\n");
    */

    /// Keeps a live range set for each register unit to track fixed physreg
    /// interference.
    /*
    for(size_t i=0; i < LIA->getNumRegUnits(); ++i) {
        LiveRange &LR = LIA->getRegUnit(i);
        outs() << printRegUnit(i, TRI) << ' ' << LR << "\n";
    }
    */
    
    // Dump virtual ranges
    for(unsigned i = 0; i < MRI->getNumVirtRegs(); ++i) {
        Register reg = Register::index2VirtReg(i);
        if(LIA->hasInterval(reg)) {
            LiveInterval &interval = LIA->getInterval(reg);
            outs() << interval << "\n";
            for(auto segment_itr = interval.begin(); segment_itr != interval.end(); ++segment_itr) {
                outs() << "\t" << *segment_itr << "\n";
            }
            /*
            for(auto& subrange : interval.subranges()) {
                outs() << "\t" << subrange << "\n";
            }
            */
        }
    }

    for(SlotIndex si = indexes->getZeroIndex();; si = indexes->getNextNonNullIndex(si)) {
        MachineInstr *mi = indexes->getInstructionFromIndex(si);
        outs() << si << "\n";
        if(mi != nullptr) {
            outs() << *mi << "\n";
        }

        if(si == indexes->getLastIndex()) {
            break;
        }
    }


    return false;
}

bool LivenessVisualization::doFinalization(Module &M) {
    printf("LivenessVisualization doFinalization\n");
    MachineFunctionPass::doFinalization(M);
    return false;
}
