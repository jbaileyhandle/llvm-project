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
#include "llvm/Demangle/Demangle.h"
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
#include <iomanip>
#include <iostream>
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

std::string LivenessVisualization::GraphBB::getSanitizedFuncName(const MachineBasicBlock &MBB) {
    std::string name = MBB.getFullName();
    for(auto& character : name) {
        if(!isalnum(character) && character != '_') {
            character = '_';
        }
    }
    return name;
}

LivenessVisualization::GraphBB::GraphBB(LivenessVisualization *LVpass, const MachineBasicBlock &MBB) {
    indexes_ = LVpass->LIA_->getSlotIndexes();
    name_ = getSanitizedFuncName(MBB);
    MBB_ = &MBB;
    LVpass_ = LVpass;

    label_str_ = name_;
    addNewlineToLabel();
}

void LivenessVisualization::GraphBB::addNewlineToLabel() {
        label_str_ += "\\l";
}

void LivenessVisualization::GraphBB::addChildren(std::unordered_map<const MachineBasicBlock*, GraphBB>& mbb_to_gbb) {
    for(const MachineBasicBlock* successor_bb : MBB_->successors()) {
        assert(mbb_to_gbb.find(successor_bb) != mbb_to_gbb.end());
        children_.push_back(&mbb_to_gbb.at(successor_bb));
    }
}

void LivenessVisualization::GraphBB::addInstructionAtSlotIndex(SlotIndex si) {
    MachineInstr *mi = indexes_->getInstructionFromIndex(si);
    addNewlineToLabel();
    if(mi != nullptr) {
        // TODO add debug info, as below.
        std::string mi_str;
        raw_string_ostream mi_ostream(mi_str);

        // Add location info.
        if(mi->getDebugLoc().isImplicitCode()) {
            mi_ostream << "ImplicitCode";
        } else {
            mi->getDebugLoc().print(mi_ostream);
        }
        mi_ostream << ":";
        label_str_ += mi_str;
        addNewlineToLabel();

        mi_str.clear();
        mi_str.resize(location_padding_amount, ' ');

        // Add instruciton.
        mi_ostream << *mi;
        auto new_mi_str_end = std::remove(mi_str.begin(), mi_str.end(), '\n');
        mi_str.resize(new_mi_str_end - mi_str.begin());
        label_str_ += mi_str;
    }
    addNewlineToLabel();
}

std::unique_ptr<std::vector<Register>> LivenessVisualization::GraphBB::getLiveVirtRegsAtSlotIndex(SlotIndex si) {
    std::unique_ptr<std::vector<Register>> live_virt_registers = std::make_unique<std::vector<Register>>();

    for(unsigned i = 0; i < LVpass_->MRI_->getNumVirtRegs(); ++i) {
        Register reg = Register::index2VirtReg(i);

        if(LVpass_->LIA_->hasInterval(reg)) {
            const LiveInterval &interval = LVpass_->LIA_->getInterval(reg);
            if(interval.liveAt(si)) {
                live_virt_registers->push_back(reg);
            }
        }
    }

    return std::move(live_virt_registers);
}

void LivenessVisualization::GraphBB::addRegistersAtSlotIndex(SlotIndex si) {
    std::unique_ptr<std::vector<Register>> live_virt_registers = getLiveVirtRegsAtSlotIndex(si);
    // Get physical registers
    std::string reg_str(location_padding_amount, ' ');
    raw_string_ostream reg_ostream(reg_str);

    reg_str += "(" + std::to_string(live_virt_registers->size()) + "): ";
    for(const auto& reg : *live_virt_registers) {
        reg_ostream << printVRegOrUnit(reg, LVpass_->TRI_) << ", ";
    }
    label_str_ += reg_str;
    addNewlineToLabel();
}

void LivenessVisualization::GraphBB::addSlotIndex(SlotIndex si) {
    addRegistersAtSlotIndex(si);
    addInstructionAtSlotIndex(si);
}

void LivenessVisualization::GraphBB::emitConnections(std::ofstream &dot_file) const {
    for(GraphBB* child : children_) {
        dot_file << "\t" << name_ << " -> " << child->name_ << "\n";
    }
}

void LivenessVisualization::GraphBB::emitNode(std::ofstream &dot_file) const {
    dot_file << "\t" << name_ << " [shape=box; label=\"" << label_str_ << "\"]\n";
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

std::string LivenessVisualization::getSanitizedFuncName(const MachineFunction *fn) {
    std::string func_name = demangle(fn->getName().str());
    std::replace(func_name.begin(), func_name.end(), '(', '_');
    std::replace(func_name.begin(), func_name.end(), ')', '_');
    auto new_func_name_end = std::remove(func_name.begin(), func_name.end(), ' ');
    func_name.resize(new_func_name_end - func_name.begin());
    return std::move(func_name);
}

void LivenessVisualization::buildGraphBBs() {
    SlotIndexes *indexes = LIA_->getSlotIndexes();

    // Make GraphBB objects.
    for (const MachineBasicBlock &MBB : *MF_) {
        assert(mbb_to_gbb_.find(&MBB) == mbb_to_gbb_.end());
        mbb_to_gbb_.emplace(&MBB, GraphBB(this, MBB));
    }

    // Make connections between GraphBB's and their children.
    for (const MachineBasicBlock &MBB : *MF_) {
        assert(mbb_to_gbb_.find(&MBB) != mbb_to_gbb_.end());
        GraphBB& gbb = mbb_to_gbb_.find(&MBB)->second;
        gbb.addChildren(mbb_to_gbb_);
    }

    // Add information at each index.
    for (const MachineBasicBlock &MBB : *MF_) {
        GraphBB& gbb = mbb_to_gbb_.at(&MBB);
        for(SlotIndex si = indexes->getMBBStartIdx(&MBB); si <= indexes->getMBBEndIdx(&MBB); si = indexes->getNextNonNullIndex(si)) {
            gbb.addSlotIndex(si);
            if(si == indexes->getLastIndex()) {
                break;
            }
        }
    }
}

void LivenessVisualization::emitGraphBBs(std::ofstream &dot_file) const {
    for (const MachineBasicBlock &MBB : *MF_) {
        assert(mbb_to_gbb_.find(&MBB) != mbb_to_gbb_.end());
        const GraphBB &gbb = mbb_to_gbb_.find(&MBB)->second;
        gbb.emitConnections(dot_file);
        gbb.emitNode(dot_file);
    }
}

bool LivenessVisualization::runOnMachineFunction(MachineFunction &fn) {

    // Init variables.
    MF_ = &fn;
    MRI_ = &MF_->getRegInfo();
    LIA_ = &getAnalysis<LiveIntervals>();
    TRI_ = MF_->getSubtarget().getRegisterInfo();
    TII_ = MF_->getSubtarget().getInstrInfo();
    std::string func_name = getSanitizedFuncName(MF_);

    std::ofstream dot_file;
    dot_file.open(func_name + ".dot");
    dot_file << "digraph {\n";

    outs() << "\n\nFunction: " << MF_->getName() << "\n";

    buildGraphBBs();
    emitGraphBBs(dot_file);

    dot_file << "}\n";

    /*
    for (const MachineBasicBlock &MBB : *MF_) {
        for (const MachineInstr &MI : MBB) {
            outs() << MI << "\n";
        }
    }
    */



    /*
    for(SlotIndex si = indexes->getZeroIndex();; si = indexes->getNextNonNullIndex(si)) {
        const MachineBasicBlock* slot_mbb = indexes->getMBBFromIndex(si);
        assert(mbb_to_gbb_.find(slot_mbb) != mbb_to_gbb_.end());
        GraphBB& gbb = mbb_to_gbb_.at(slot_mbb);
        gbb.AddSlotIndex(si);

        if(si == indexes->getLastIndex()) {
            break;
        }
    }
    */

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


    /*
    SlotIndexes *indexes = LIA_->getSlotIndexes();
    
    // Dump virtual ranges
    for(unsigned i = 0; i < MRI_->getNumVirtRegs(); ++i) {
        Register reg = Register::index2VirtReg(i);
        if(LIA_->hasInterval(reg)) {
            const LiveInterval &interval = LIA_->getInterval(reg);
            outs() << interval << "\n";
            for(auto segment_itr = interval.begin(); segment_itr != interval.end(); ++segment_itr) {
                outs() << "\t" << *segment_itr << "\n";
            }
            
            // for(auto& subrange : interval.subranges()) {
            //     outs() << "\t" << subrange << "\n";
            // }
            
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
    */


    return false;
}

bool LivenessVisualization::doFinalization(Module &M) {
    printf("LivenessVisualization doFinalization\n");
    MachineFunctionPass::doFinalization(M);
    return false;
}
