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
#include "llvm/Target/TargetMachine.h"
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
#include <sstream>
#include <tuple>
#include <utility>
#include <unordered_set>

using namespace llvm;

#define DEBUG_TYPE "livenessvisualization"

// Left justification padding.
#define LOCATION_PADDING_AMOUNT 35

#define SEMI_HOT_PERCENT 0.8
const int col_width[] = {6,6,6,5,30,45};


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

template <typename T>
std::string objPtrToString(T *input) {
    std::string str;
    raw_string_ostream ostream(str);
    ostream << *input;
    return str;
}

std::string LivenessVisualization::getSanitizedXdotLabelStr(const std::string& str) {
    std::string cpy = str;
    for(char& character : cpy) {
        if(character == '"') {
            character = '_';
        }
    }
    return cpy;
}

std::string LivenessVisualization::GraphBB::getSanitizedMBBName(const MachineBasicBlock &MBB) {
    std::string name = MBB.getFullName();
    for(char& character : name) {
        if(!isalnum(character) && character != '_') {
            character = '_';
        }
    }
    return name;
}

LivenessVisualization::GraphBB::GraphBB(LivenessVisualization *LVpass, const MachineBasicBlock &MBB) {
    indexes_ = LVpass->member_vars_.LIA_->getSlotIndexes();
    name_ = getSanitizedMBBName(MBB);
    MBB_ = &MBB;
    LVpass_ = LVpass;

    label_str_ = name_;
    addNewlineToLabel();
}

void LivenessVisualization::GraphBB::addNewlineToLabel() {
        label_str_ += "\\l";
}

// TODO: remove arg and just access map from LVpass_
void LivenessVisualization::GraphBB::addChildren(std::unordered_map<const MachineBasicBlock*, GraphBB>& mbb_to_gbb) {
    for(const MachineBasicBlock* successor_bb : MBB_->successors()) {
        assert(mbb_to_gbb.find(successor_bb) != mbb_to_gbb.end());
        children_.push_back(&mbb_to_gbb.at(successor_bb));
    }
}

void LivenessVisualization::GraphBB::addInstructionAtSlotIndex(SlotIndex si, SlotIndexInfo& info) {
    MachineInstr *mi = indexes_->getInstructionFromIndex(si);
    if(mi != nullptr) {
        // Add instruciton.
        std::string mi_str = objPtrToString(mi);
        mi_str.erase(std::remove(mi_str.begin(), mi_str.end(), '\n'), mi_str.end());
        label_str_ += std::string(LOCATION_PADDING_AMOUNT, ' ') + mi_str;
        info.mi_str_ = mi_str;
    }
    addNewlineToLabel();
}

void LivenessVisualization::GraphBB::addInstructionLocationAtSlotIndex(SlotIndex si, SlotIndexInfo &info) {
    MachineInstr *mi = indexes_->getInstructionFromIndex(si);
    if(mi != nullptr) {
        std::string mi_str;
        raw_string_ostream mi_ostream(mi_str);

        // Add location info.
        if(mi->getDebugLoc().isImplicitCode()) {
            mi_ostream << "ImplicitCode";
        } else {
            mi->getDebugLoc().print(mi_ostream);
        }
        label_str_ += mi_str;
        info.src_location_ = mi_str;
    }
    addNewlineToLabel();
}

std::vector<LivenessVisualization::GraphBB::RegSegment> LivenessVisualization::GraphBB::getLiveVirtRegsAtSlotIndex(const SlotIndex si, SlotIndexInfo &info) {
    std::vector<RegSegment> live_virt_registers;

    for(unsigned i = 0; i < LVpass_->member_vars_.MRI_->getNumVirtRegs(); ++i) {
        Register reg = Register::index2VirtReg(i);

        if(LVpass_->member_vars_.LIA_->hasInterval(reg)) {
            const LiveInterval *interval = &(LVpass_->member_vars_.LIA_->getInterval(reg));
            if(interval->liveAt(si)) {
                live_virt_registers.push_back(RegSegment(reg, interval->getSegmentContaining(si)));
                info.live_virt_registers_.push_back(reg);
            }
        }
    }

    return live_virt_registers;
}

std::vector<LivenessVisualization::GraphBB::RegSegment> LivenessVisualization::GraphBB::getLivePhysRegsAtSlotIndex(const SlotIndex si) {
    std::vector<RegSegment> live_phys_registers;

    for(unsigned reg=0; reg < LVpass_->member_vars_.TRI_->getNumRegUnits(); ++reg) {
        LiveRange &range = LVpass_->member_vars_.LIA_->getRegUnit(reg);
        if(range.liveAt(si)) {
            live_phys_registers.push_back(RegSegment(reg, range.getSegmentContaining(si)));
        }
    }
    return live_phys_registers;
}

void LivenessVisualization::GraphBB::addSetOfLiveRegs(std::vector<RegSegment>& live_registers, std::string label) {
    std::string reg_str(LOCATION_PADDING_AMOUNT, ' ');
    reg_str += "(" + std::to_string(live_registers.size()) + ")[" + label + "]: ";
    for(const auto& reg_segment : live_registers) {
        auto printable_vreg_or_unit = printVRegOrUnit(reg_segment.reg_, LVpass_->member_vars_.TRI_);
        reg_str += objPtrToString(&printable_vreg_or_unit) + ":" + objPtrToString(reg_segment.segment_) + ", ";
    }
    label_str_ += reg_str;
    addNewlineToLabel();
}

int LivenessVisualization::GraphBB::getMaxVirtLive() const {
    return max_virt_live_;
}

void LivenessVisualization::GraphBB::addRegistersAtSlotIndex(SlotIndex si, SlotIndexInfo &info) {
    std::vector<RegSegment> live_virt_registers = getLiveVirtRegsAtSlotIndex(si, info);
    addSetOfLiveRegs(live_virt_registers, "virt");
    max_virt_live_ = std::max(max_virt_live_, (int)live_virt_registers.size());

    std::vector<RegSegment> live_registers = getLivePhysRegsAtSlotIndex(si);
    addSetOfLiveRegs(live_registers, "phys");

    live_registers.insert(live_registers.end(), live_virt_registers.begin(), live_virt_registers.end());
    addSetOfLiveRegs(live_registers, "comb");
}

void LivenessVisualization::GraphBB::markHotspot(int function_max_virt_live) {
    percent_max_virt_live_ = (float)max_virt_live_ / (float)function_max_virt_live;
}

std::string LivenessVisualization::GraphBB::getHotspotAttr() const {
    std::string attr_str;
    if(percent_max_virt_live_ >= SEMI_HOT_PERCENT) {
        std::string color("blue");
        if(percent_max_virt_live_ == 1.0) {
            color = "purple";
        }
        attr_str += std::string("color=") + color + "; fontcolor=" + color + "; fontsize=20;";

    }

    return attr_str;
}

void LivenessVisualization::GraphBB::addSlotIndex(SlotIndex si, SlotIndexInfo& info) {
    addNewlineToLabel();
    addNewlineToLabel();

    label_str_ += "si " + objPtrToString(&si);

    addNewlineToLabel();
    addInstructionLocationAtSlotIndex(si, info);
    addRegistersAtSlotIndex(si, info);
    addInstructionAtSlotIndex(si, info);
}

void LivenessVisualization::GraphBB::emitConnections(std::ofstream &dot_file) const {
    for(GraphBB* child : children_) {
        dot_file << "\t" << name_ << " -> " << child->name_ << "\n";
    }
}

void LivenessVisualization::GraphBB::emitNode(std::ofstream &dot_file) const {
    //dot_file << "\t" << name_ << "[shape=box; label=\"" << getSanitizedXdotLabelStr(label_str_) << "\"; " << getHotspotAttr() << "]\n";
    dot_file << "\t" << name_ << "[shape=box; fontname=\"Courier New\", label=\"" << getSanitizedXdotLabelStr(getLinearReport()) << "\"; " << getHotspotAttr() << "]\n";
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
    return func_name;
}

void LivenessVisualization::buildGraphBBs() {

    // Make GraphBB objects.
    for (const MachineBasicBlock &MBB : *(member_vars_.MF_)) {
        assert(member_vars_.mbb_to_gbb_.find(&MBB) == member_vars_.mbb_to_gbb_.end());
        member_vars_.mbb_to_gbb_.emplace(&MBB, GraphBB(this, MBB));
    }

    // Make connections between GraphBB's and their children.
    for (const MachineBasicBlock &MBB : *(member_vars_.MF_)) {
        assert(member_vars_.mbb_to_gbb_.find(&MBB) != member_vars_.mbb_to_gbb_.end());
        GraphBB& gbb = member_vars_.mbb_to_gbb_.find(&MBB)->second;
        gbb.addChildren(member_vars_.mbb_to_gbb_);
    }

    // Add information at each index in GraphBB's.
    for (const MachineBasicBlock &MBB : *(member_vars_.MF_)) {
        GraphBB& gbb = member_vars_.mbb_to_gbb_.at(&MBB);
        for(SlotIndex si = member_vars_.indexes_->getMBBStartIdx(&MBB); si <= member_vars_.indexes_->getMBBEndIdx(&MBB); si = member_vars_.indexes_->getNextNonNullIndex(si)) {
            auto elem_success = member_vars_.si_to_info_.emplace(si, SlotIndexInfo(si, this));
            assert(elem_success.second == true);
            gbb.addSlotIndex(si, member_vars_.si_to_info_.at(si));
            if(si == member_vars_.indexes_->getLastIndex()) {
                break;
            }
        }
    }

    // ID hotspots.
    for (const MachineBasicBlock &MBB : *(member_vars_.MF_)) {
        GraphBB& gbb = member_vars_.mbb_to_gbb_.at(&MBB);
        member_vars_.function_max_virt_live_ = std::max(member_vars_.function_max_virt_live_, gbb.getMaxVirtLive());
    }
    for (const MachineBasicBlock &MBB : *(member_vars_.MF_)) {
        GraphBB& gbb = member_vars_.mbb_to_gbb_.at(&MBB);
        gbb.markHotspot(member_vars_.function_max_virt_live_);
        for(SlotIndex si = member_vars_.indexes_->getMBBStartIdx(&MBB); si <= member_vars_.indexes_->getMBBEndIdx(&MBB); si = member_vars_.indexes_->getNextNonNullIndex(si)) {
            SlotIndexInfo &info = member_vars_.si_to_info_.at(si);
            info.percent_virt_live_registers_ = (float)info.live_virt_registers_.size() / (float)member_vars_.function_max_virt_live_;
            if(si == member_vars_.indexes_->getLastIndex()) {
                break;
            }
        }
    }
}

std::vector<Register> LivenessVisualization::getLiveVirtRegsInRange(SlotIndex begin, SlotIndex end_inclusive) const {
    std::unordered_set<unsigned> enumerated_reg_ids;
    std::vector<Register> registers;
    for(SlotIndex si = begin; si <= end_inclusive; si = member_vars_.indexes_->getNextNonNullIndex(si)) {
        for(Register reg : member_vars_.si_to_info_.at(si).live_virt_registers_) {
            if(enumerated_reg_ids.find(reg) == enumerated_reg_ids.end()) {
                enumerated_reg_ids.insert(reg);
                registers.push_back(reg);
            }
        }
        if(si == member_vars_.indexes_->getLastIndex()) {
            break;
        }
    }
    return registers;
}

char LivenessVisualization::SlotIndexInfo::getRegUsageSymbol(Register reg) const {
    assert(mi_ != nullptr);
    bool live = std::find(live_virt_registers_.begin(), live_virt_registers_.end(), reg) != live_virt_registers_.end();
    // Note - could also possible use readsWritesVirtualRegister here
    bool read = mi_->readsRegister(reg, LVpass_->member_vars_.TRI_);
    bool write = mi_->modifiesRegister(reg, LVpass_->member_vars_.TRI_);
    if(read && write) {
        return 'X';
    } else if(read) {
        return '^';
    } else if (write) {
        return 'v';
    } else if (live) {
        return ':';
    } else {
        return ' ';
    }
}

std::string LivenessVisualization::regHeaderStr(const std::vector<Register>& registers) const {
    std::stringstream sstream;
    std::string dummy;

    sstream << std::right << std::setw(col_width[0]) << "slot" << " | ";
    sstream << std::right << std::setw(col_width[1]) << "\%peak" << " | ";
    sstream << std::right << std::setw(col_width[2]) << "#live" << " | ";
    for(Register reg : registers) {
        auto printable_vreg_or_unit = printVRegOrUnit(reg, member_vars_.TRI_);
        sstream << std::setw(col_width[3]) << objPtrToString(&printable_vreg_or_unit);
    }
    sstream << " | ";
    sstream << std::left << std::setw(col_width[4]) << "src" << " | ";

    return sstream.str();
}

std::string LivenessVisualization::SlotIndexInfo::toString(const std::vector<Register>& registers) const {
    std::stringstream sstream;

    sstream << std::right << std::setw(col_width[0]) << objPtrToString(&si_) << " | ";
    sstream << std::right << std::setw(col_width[1]-1) << std::fixed << std::setprecision(0) << percent_virt_live_registers_*100.0 << "% | ";
    sstream << std::right << std::setw(col_width[2]) << live_virt_registers_.size() << " | ";
    for(Register reg : registers) {
        sstream << std::setw(col_width[3]) << getRegUsageSymbol(reg);
    }
    sstream << " | ";
    /*
    std::string mi_str_copy = mi_str_;
    mi_str_copy.resize(std::min(mi_str_copy.size(), 200 - sstream.str().size()));
    */
    sstream << std::left << std::setw(col_width[4]) << src_location_ << " | ";
    sstream << std::left << std::setw(col_width[5]) << mi_str_;

    return sstream.str();
}


std::string LivenessVisualization::getLinearReport(std::string title, std::string newline, SlotIndex begin, SlotIndex end_inclusive) const {
    std::vector<Register> registers = getLiveVirtRegsInRange(begin, end_inclusive);

    std::stringstream linear_report;
    linear_report << title << newline;
    linear_report << regHeaderStr(registers) << newline;
    for(SlotIndex si = begin; si <= end_inclusive; si = member_vars_.indexes_->getNextNonNullIndex(si)) {
        const SlotIndexInfo& info = member_vars_.si_to_info_.at(si);
        if(info.mi_ != nullptr) {
            std::string si_str = member_vars_.si_to_info_.at(si).toString(registers);
            linear_report << si_str << newline;
        }

        if(si == member_vars_.indexes_->getLastIndex()) {
            break;
        }
    }

    return linear_report.str();
}

std::string LivenessVisualization::GraphBB::getLinearReport() const {
    return LVpass_->getLinearReport(name_, "\\l", indexes_->getMBBStartIdx(MBB_), indexes_->getMBBEndIdx(MBB_));
}

void LivenessVisualization::emitLinearReport(std::string name_stub, SlotIndex begin, SlotIndex end_inclusive) const {
    std::ofstream linear_file;
    linear_file.open(name_stub + "_linearReport.txt");
    std::string linear_report = getLinearReport(name_stub, "\n", begin, end_inclusive);
    linear_file << linear_report;
}

void LivenessVisualization::GraphBB::emitLinearReport() const {
    LVpass_->emitLinearReport(LVpass_->member_vars_.sanitized_func_name_ + "--" + name_, indexes_->getMBBStartIdx(MBB_), indexes_->getMBBEndIdx(MBB_));
}

void LivenessVisualization::emitGraphBBs(std::ofstream &dot_file) const {
    for (const MachineBasicBlock &MBB : *(member_vars_.MF_)) {
        //assert(mbb_to_gbb_.find(&MBB) != mbb_to_gbb_.end());
        const GraphBB &gbb = member_vars_.mbb_to_gbb_.find(&MBB)->second;
        gbb.emitConnections(dot_file);
        gbb.emitNode(dot_file);
        gbb.emitLinearReport();
    }
}

// TODO: put in constructor
LivenessVisualization::MemberVars::MemberVars(const MachineFunction &fn, LiveIntervals *LIA) {
    MF_ = &fn;
    MRI_ = &MF_->getRegInfo();
    LIA_ = LIA;
    indexes_ = LIA_->getSlotIndexes();
    TRI_ = MF_->getSubtarget().getRegisterInfo();
    TII_ = MF_->getSubtarget().getInstrInfo();
    sanitized_func_name_ = getSanitizedFuncName(MF_);
}

bool LivenessVisualization::runOnMachineFunction(MachineFunction &fn) {
    // Only care about GPU code.
    if(!(fn.getTarget().getTargetTriple().isAMDGPU() || fn.getTarget().getTargetTriple().isNVPTX())) {
        return false;
    }

    // Init variables.
    member_vars_ = MemberVars(fn, &getAnalysis<LiveIntervals>());

    std::ofstream dot_file;
    dot_file.open(member_vars_.sanitized_func_name_+ ".dot");
    dot_file << "digraph {\n";

    outs() << "\n\nFunction: " << member_vars_.MF_->getName() << "\n";

    buildGraphBBs();
    emitGraphBBs(dot_file);

    dot_file << "}\n";

    return false;
}

bool LivenessVisualization::doFinalization(Module &M) {
    printf("LivenessVisualization doFinalization\n");
    MachineFunctionPass::doFinalization(M);
    return false;
}
