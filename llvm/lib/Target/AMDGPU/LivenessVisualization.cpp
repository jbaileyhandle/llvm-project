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

#include "LivenessVisualization.h"
#include "llvm/Demangle/Demangle.h"
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
#define SLOT_COL_WIDTH 6
#define PERCENT_PEAK_COL_WIDTH 6
#define NUM_LIVE_REGS_COL_WIDTH 6
#define REG_USAGE_TRUNC_LENGTH 6
#define REG_USAGE_COL_WIDTH 6
#define REG_RW_WIDTH  50
#define SRC_COL_WIDTH 30
#define ASM_COL_WIDTH 45


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
std::string objPtrToString(const T *input) {
    std::string str;
    raw_string_ostream ostream(str);
    ostream << *input;
    return str;
}

// Get machine instruction location string
std::string getMachineInstrSrcLocation(const MachineInstr *mi) {
    assert(mi != nullptr);
    std::string src_location_str;
    raw_string_ostream mi_ostream(src_location_str);

    // Add location info.
    if(mi->getDebugLoc().isImplicitCode()) {
        mi_ostream << "ImplicitCode";
    } else {
        mi->getDebugLoc().print(mi_ostream);
    }

    return src_location_str;
}

// Return string for machine instruction mi
std::string getMachineInstrStr(const MachineInstr* mi) {
    assert(mi != nullptr);
    std::string mi_str = objPtrToString(mi);
    mi_str.erase(std::remove(mi_str.begin(), mi_str.end(), '\n'), mi_str.end());
    return mi_str;
}

// Return string for machine instruction mi, excluding debug info
std::string getMachineInstrStrWithoutDebugInfo(const MachineInstr* mi) {
    std::string mi_str = getMachineInstrStr(mi);
    size_t debug_pos = mi_str.find("debug-location");
    if(debug_pos != std::string::npos) {
        mi_str = mi_str.substr(0, debug_pos);
    }
    return mi_str;
}

// Grab opcode from MachineInstr
// Uses heuristic / assumption that first word starting w/ a
// capital letter is the opcode
std::string getMachineInstrOpcode(const MachineInstr* mi) {
    std::string mi_str = getMachineInstrStr(mi);
    std::istringstream stream(mi_str);
    std::string opcode;

    while (stream >> opcode) {
        if (!opcode.empty() && isupper(static_cast<unsigned char>(opcode[0]))) {
            return opcode;
        }
    }

    // If no word that starts with a capital letter is found, return an empty string.
    return "";
}

// Grab opcode + debug location from MachineInstr
std::string getMachineInstrOpcodeAndDebugLocation(const MachineInstr* mi) {
    std::string opcode = getMachineInstrOpcode(mi);
    std::string mi_str = getMachineInstrStr(mi);

    size_t debug_pos = mi_str.find("debug-location");
    if(debug_pos == std::string::npos) {
        return opcode; 
    }

    return opcode + ",\t" + mi_str.substr(debug_pos);

}

// Get a modified version of str which is safe to emit as an xdot label.
std::string getSanitizedXdotLabelStr(const std::string& str) {
    std::string cpy = str;
    for(char& character : cpy) {
        if(character == '"') {
            character = '_';
        }
    }
    return cpy;
}

std::string LivenessVisualization::GraphBB::getSanitizedMBBName() const {
    std::string name = MBB_->getFullName();
    for(char& character : name) {
        if(!isalnum(character) && character != '_') {
            character = '_';
        }
    }
    return name;
}

std::string LivenessVisualization::getRegString(Register reg) const {
    auto printable_vreg_or_unit = printVRegOrUnit(reg, member_vars_.TRI_);
    return objPtrToString(&printable_vreg_or_unit);
}


LivenessVisualization::SlotIndexInfo::SlotIndexInfo(SlotIndex si, const GraphBB *graph_bb, const LivenessVisualization *LVpass): si_(si), mi_(LVpass->member_vars_.indexes_->getInstructionFromIndex(si)), graph_bb_(graph_bb), LVpass_(LVpass) {
    addInstructionStr();
    addInstructionLocationStr();
    addRegisters();
}

// Constructor helper to save string of source locatoin of slot index
void LivenessVisualization::SlotIndexInfo::addInstructionLocationStr() {
    if(mi_ != nullptr) {
        src_location_ = getMachineInstrSrcLocation(mi_);
    }
}

// Constructor helper to save instruction string
void LivenessVisualization::SlotIndexInfo::addInstructionStr() {
    if(mi_ != nullptr) {
        mi_str_ = getMachineInstrStr(mi_);
    }
}

// Constructor helpers to save registers at slot index
void LivenessVisualization::SlotIndexInfo::addRegisters() {
    addLiveVirtRegs();
    addLivePhysRegs();
    addCombinedRegs();
    addReadRegs();
    addWriteRegs();
}

void LivenessVisualization::SlotIndexInfo::addLiveVirtRegs() {

    for(unsigned i = 0; i < LVpass_->member_vars_.MRI_->getNumVirtRegs(); ++i) {
        Register reg = Register::index2VirtReg(i);

        if(LVpass_->member_vars_.LIA_->hasInterval(reg)) {
            const LiveInterval *interval = &(LVpass_->member_vars_.LIA_->getInterval(reg));
            if(interval->liveAt(si_)) {
                if(LVpass_->isSGPR(reg)) {
                    live_virtual_scalar_registers_.push_back(reg);
                } else if(LVpass_->isVGPR(reg)) {
                    live_virtual_vector_registers_.push_back(reg);
                }
                /* 
                } else if (LVpass_->isAGPR(reg)) {
                    live_virtual_vector_registers_.push_back(reg);
                }
                */
            }
        }
    }
}

void LivenessVisualization::SlotIndexInfo::addLivePhysRegs() {

    for(unsigned reg=0; reg < LVpass_->member_vars_.TRI_->getNumRegUnits(); ++reg) {
        // See https://llvm.org/doxygen/LiveIntervals_8h_source.html#l00387
        LiveRange &range = LVpass_->member_vars_.LIA_->getRegUnit(reg);

        if(range.liveAt(si_)) {
            if(LVpass_->isSGPR(reg)) {
                live_physical_scalar_registers_.push_back(reg);
            } else if(LVpass_->isVGPR(reg)) {
                live_physical_vector_registers_.push_back(reg);
            }
        }
    }
}

void LivenessVisualization::SlotIndexInfo::addCombinedRegs() {
    live_vector_registers_.insert(live_vector_registers_.end(), live_physical_vector_registers_.begin(), live_physical_vector_registers_.end());
    live_vector_registers_.insert(live_vector_registers_.end(), live_virtual_vector_registers_.begin(), live_virtual_vector_registers_.end());
    live_scalar_registers_.insert(live_scalar_registers_.end(), live_physical_scalar_registers_.begin(), live_physical_scalar_registers_.end());
    live_scalar_registers_.insert(live_scalar_registers_.end(), live_virtual_scalar_registers_.begin(), live_virtual_scalar_registers_.end());
    live_registers_.insert(live_registers_.end(), live_scalar_registers_.begin(), live_scalar_registers_.end());
    live_registers_.insert(live_registers_.end(), live_vector_registers_.begin(), live_vector_registers_.end());
}

void LivenessVisualization::SlotIndexInfo::addReadRegs() {
    if(mi_ == nullptr) {
        return;
    }
    for(auto reg : live_vector_registers_) {
        bool read = mi_->readsRegister(reg, LVpass_->member_vars_.TRI_);
        if(read) {
            read_vector_registers_.insert(reg);
        }
    }

    // Strangely, this method misses some register reads,
    // as the reads do not show up in returned uses()...
    /*
    for (const MachineOperand &use : mi_->uses()) {
        if (!use.isReg()) {
            return;
        }
        read_vector_registers_.insert(use.getReg());
    }
    */
}

void LivenessVisualization::SlotIndexInfo::addWriteRegs() {
    if(mi_ == nullptr) {
        return;
    }
    // Have to use this approach here (vs scanning live registers as in addReadRegs)
    // because 1st definitions of a reg are not considered live at that slot
    for (const MachineOperand &def : mi_->defs()) {
        assert(def.isReg());
        auto dest_reg = def.getReg();
        if (LVpass_->isVGPR(dest_reg)) {
            write_vector_registers_.insert(dest_reg);
        }
    }
}

LivenessVisualization::GraphBB::GraphBB(LivenessVisualization *LVpass, const MachineBasicBlock *MBB) {
    indexes_ = LVpass->member_vars_.LIA_->getSlotIndexes();
    MBB_ = MBB;
    name_ = getSanitizedMBBName();
    LVpass_ = LVpass;

    // Add information at each index in GraphBB's
    for(SlotIndex si = getStartIdx(); si <= getEndIdx(); si = indexes_->getNextNonNullIndex(si)) {
        si_info_list_.emplace(si_info_list_.end(), si, this, LVpass_);
        if(si == indexes_->getLastIndex()) {
            break;
        }
    }

    // Add live vector registers.
    addLiveVectorRegsInBB();
    setMaxLiveVectorRegs();
}

// Get first / last slot index in GraphBB
SlotIndex LivenessVisualization::GraphBB::getStartIdx() const {
    return indexes_->getMBBStartIdx(MBB_);
}
SlotIndex LivenessVisualization::GraphBB::getEndIdx() const {
    return indexes_->getMBBEndIdx(MBB_);
}

// TODO: remove arg and just access map from LVpass_
void LivenessVisualization::GraphBB::addChildren(std::unordered_map<const MachineBasicBlock*, GraphBB>& mbb_to_gbb) {
    for(const MachineBasicBlock* successor_bb : MBB_->successors()) {
        assert(mbb_to_gbb.find(successor_bb) != mbb_to_gbb.end());
        GraphBB *child = &mbb_to_gbb.at(successor_bb);
        children_.push_back(child);
        child->addParent(this);
    }
}

bool LivenessVisualization::GraphBB::dominates(const GraphBB *other) const {
    const MachineBasicBlock *other_mbb = other->getMBB();
    return LVpass_->member_vars_.DT_->dominates(MBB_, other_mbb);
}

bool LivenessVisualization::GraphBB::isDominatedBy(const GraphBB *other) const {
    return other->dominates(this);
}

int LivenessVisualization::GraphBB::getMaxVectorRegistersLive() const {
    return max_vector_registers_live_;
}

void LivenessVisualization::GraphBB::markHotspot(int function_max_vector_registers_live) {
    percent_max_vector_registers_live_ = (float)max_vector_registers_live_ / (float)function_max_vector_registers_live;
    for(auto &info : si_info_list_) {
        info.setLivenessPercent(function_max_vector_registers_live);
    }
}

std::string LivenessVisualization::GraphBB::getHotspotAttr() const {
    std::string attr_str;
    if(percent_max_vector_registers_live_ >= SEMI_HOT_PERCENT) {
        std::string color("blue");
        if(percent_max_vector_registers_live_ == 1.0) {
            color = "purple";
        }
        attr_str += std::string("color=") + color + "; fontcolor=" + color + "; fontsize=20;";

    }

    return attr_str;
}

void LivenessVisualization::GraphBB::emitXdotConnections(std::ofstream &dot_file) const {
    for(GraphBB* child : children_) {
        dot_file << "\t" << name_ << " -> " << child->name_ << "\n";
    }
}

void LivenessVisualization::GraphBB::emitXdotNode(std::ofstream &dot_file) const {
    std::string linear_report = getLinearReport(name_, false);
    dot_file << "\t" << name_ << "[shape=box; fontname=\"Courier New\", label=\"" << getSanitizedXdotLabelStr(linear_report) << "\"; " << getHotspotAttr() << "]\n";
}

LivenessVisualization::LivenessVisualization() : MachineFunctionPass(ID) {
  initializeLivenessVisualizationPass(*PassRegistry::getPassRegistry());
}

LivenessVisualization::~LivenessVisualization() {}

void LivenessVisualization::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.setPreservesAll();
  AU.addRequired<LiveIntervals>();
  AU.addPreserved<LiveIntervals>();
  AU.addRequired<MachineDominatorTree>();
  AU.addPreserved<MachineDominatorTree>();
  MachineFunctionPass::getAnalysisUsage(AU);
}

bool LivenessVisualization::doInitialization(Module &M) {
    // printf("LivenessVisualization doInitialization\n");

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
        auto iter_success = member_vars_.mbb_to_gbb_.try_emplace(&MBB, this, &MBB);
        assert(iter_success.second);
    }

    // Make connections between GraphBB's and their children.
    for (const MachineBasicBlock &MBB : *(member_vars_.MF_)) {
        assert(member_vars_.mbb_to_gbb_.find(&MBB) != member_vars_.mbb_to_gbb_.end());
        GraphBB& gbb = member_vars_.mbb_to_gbb_.find(&MBB)->second;
        gbb.addChildren(member_vars_.mbb_to_gbb_);
    }

    // ID hotspots.
    for (const MachineBasicBlock &MBB : *(member_vars_.MF_)) {
        GraphBB& gbb = member_vars_.mbb_to_gbb_.at(&MBB);
        member_vars_.function_max_vector_registers_live_ = std::max(member_vars_.function_max_vector_registers_live_, gbb.getMaxVectorRegistersLive());
    }
    for (const MachineBasicBlock &MBB : *(member_vars_.MF_)) {
        GraphBB& gbb = member_vars_.mbb_to_gbb_.at(&MBB);
        gbb.markHotspot(member_vars_.function_max_vector_registers_live_);
    }

    // Generate breadth-first ordering
    setGbbLevels();
}

void LivenessVisualization::setGbbLevels() {
    // Find entry block
    const MachineBasicBlock *entry_mbb = nullptr;
    for (const MachineBasicBlock &MBB : *(member_vars_.MF_)) {
        if(MBB.isEntryBlock()) {
            entry_mbb = &MBB;
            break;
        }
    }

    // Initialize entry gbb
    assert(entry_mbb != nullptr);
    GraphBB *entry_gbb = &member_vars_.mbb_to_gbb_.at(entry_mbb);
    entry_gbb->setLevel(0);

    std::queue<GraphBB *> queue;
    for(GraphBB *child : entry_gbb->getChildren()) {
        queue.push(child);
    }

    // Use modified dijkstra's algorithm to assign levels
    int max_level = 0;
    while(!queue.empty()) {
        GraphBB *gbb = queue.front();
        queue.pop();

        int init_level = gbb->getLevel();
        std::vector<int> parent_levels;
        for(GraphBB *parent : gbb->getParents()) {
            parent_levels.push_back(parent->getLevel());
        }
        gbb->setLevel(*std::max_element(parent_levels.begin(), parent_levels.end()) + 1);
        int new_level = gbb->getLevel();

        max_level = std::max(max_level, new_level);
        if(new_level != init_level) {
            for(GraphBB *child : gbb->getChildren()) {
                // Don't loop back on back edges
                if(!child->dominates(gbb)) {
                    queue.push(child);
                }
            }
        }
    }

    // Put Gbbs in map by level
    member_vars_.level_to_gbbs_.resize(max_level + 1);
    for (const MachineBasicBlock &MBB : *(member_vars_.MF_)) {
        GraphBB *gbb = &member_vars_.mbb_to_gbb_.at(&MBB);
        member_vars_.level_to_gbbs_[gbb->getLevel()].push_back(gbb);
    }
}

void LivenessVisualization::GraphBB::addLiveVectorRegsInBB() {
    for(const auto &info : si_info_list_) {
        for(Register reg : info.live_vector_registers_) {
            live_vector_regs_in_BB_.insert(reg);
        }
    }
}

void LivenessVisualization::GraphBB::setMaxLiveVectorRegs() {
    for(const auto &info : si_info_list_) {
        max_vector_registers_live_ = std::max(max_vector_registers_live_, info.getNumLiveVectorRegisters());
    }
}

char LivenessVisualization::SlotIndexInfo::getRegUsageSymbol(Register reg) const {
    assert(mi_ != nullptr);
    bool live = std::find(live_registers_.begin(), live_registers_.end(), reg) != live_registers_.end();
    bool read = (read_vector_registers_.find(reg) != read_vector_registers_.end());
    bool write = (write_vector_registers_.find(reg) != write_vector_registers_.end());

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

std::string LivenessVisualization::GraphBB::getLinearReportHeaderStr(bool text_version) const {
    std::stringstream sstream;
    std::string dummy;

    sstream << std::right << std::setw(SLOT_COL_WIDTH) << "slot" << " | ";
    sstream << std::right << std::setw(PERCENT_PEAK_COL_WIDTH) << "\%peak" << " | ";
    if(!text_version) {
        sstream << std::right << std::setw(NUM_LIVE_REGS_COL_WIDTH) << "#live" << " | ";
        for(Register reg : live_vector_regs_in_BB_) {
            sstream << std::setw(REG_USAGE_COL_WIDTH) << LVpass_->getRegString(reg).substr(0, REG_USAGE_TRUNC_LENGTH);
        }
        sstream << " | ";
    }
    sstream << std::left << std::setw(REG_RW_WIDTH) << "Reg R/W" << " | ";
    sstream << std::left << std::setw(SRC_COL_WIDTH) << "src";

    return sstream.str();
}

std::string LivenessVisualization::SlotIndexInfo::toString(bool text_version) const {
    std::stringstream sstream;

    sstream << std::right << std::setw(SLOT_COL_WIDTH) << objPtrToString(&si_) << " | ";
    sstream << std::right << std::setw(PERCENT_PEAK_COL_WIDTH-1) << std::fixed << std::setprecision(0) << percent_vector_live_registers_*100.0 << "% | ";
    if(!text_version) {
        sstream << std::right << std::setw(NUM_LIVE_REGS_COL_WIDTH) << live_vector_registers_.size() << " | ";
        for(Register reg : graph_bb_->getLiveVectorRegsInBB()) {
            sstream << std::setw(REG_USAGE_COL_WIDTH) << getRegUsageSymbol(reg);
        }
        sstream << " | ";
        /*
        std::string mi_str_copy = mi_str_;
        mi_str_copy.resize(std::min(mi_str_copy.size(), 200 - sstream.str().size()));
        */
    }

    std::stringstream usage_sstream;
    for(auto reg : read_vector_registers_) {
        usage_sstream << "read " << LVpass_->getRegString(reg) << ", ";
    }
    for(auto reg : write_vector_registers_) {
        usage_sstream << "write " << LVpass_->getRegString(reg) << ", ";
    }
    sstream << std::left << std::setw(REG_RW_WIDTH) << usage_sstream.str() << " | ";
    sstream << std::left << std::setw(SRC_COL_WIDTH) << src_location_ << " | ";
    sstream << std::left << std::setw(ASM_COL_WIDTH) << mi_str_;

    return sstream.str();
}

std::string LivenessVisualization::GraphBB::getLinearReport(std::string title, bool text_version) const {
    std::string newline = text_version ? "\n" : "\\l";

    std::stringstream linear_report;
    linear_report << title << newline;
    linear_report << getLinearReportHeaderStr(text_version) << newline;
    for(const auto &info : si_info_list_) {
        if(info.mi_ != nullptr) {
            std::string si_str = info.toString(text_version);
            linear_report << si_str << newline;
        }
    }
    return linear_report.str();
}

void LivenessVisualization::GraphBB::emitTextReport(std::ofstream &text_file) const {
    std::string file_name = LVpass_->member_vars_.sanitized_func_name_ + "--" + name_ + "_linearReport.txt";
    std::string linear_report = getLinearReport(name_, true);
    text_file << linear_report << "\n";
}

void LivenessVisualization::emitGraphBBs(std::ofstream &dot_file, std::ofstream &text_file) const {
    for(auto &gbb_vect : member_vars_.level_to_gbbs_) {
        for(GraphBB *gbb : gbb_vect) {
            gbb->emitXdotConnections(dot_file);
            gbb->emitXdotNode(dot_file);
            gbb->emitTextReport(text_file);
        }
    }
}

void::LivenessVisualization::emit_assembly(const MachineFunction &fn) const {
    // Open files
    std::ofstream assembly_file;
    std::ofstream assembly_without_debug_info_file;
    std::ofstream assembly_opcode_file;
    std::ofstream assembly_opcode_debug_file;
    assembly_file.open(member_vars_.sanitized_func_name_+ ".jbaile_assembly");
    assembly_without_debug_info_file.open(member_vars_.sanitized_func_name_+ ".jbaile_assembly_without_debug_info");
    assembly_opcode_file.open(member_vars_.sanitized_func_name_+ ".jbaile_assembly_opcodes");
    assembly_opcode_debug_file.open(member_vars_.sanitized_func_name_+ ".jbaile_assembly_opcodes_debug");

    // Traverse BBs in order in which they are / will be laid out
    for(const auto &mbb : fn) {
        for(const auto &mi : mbb) {
            assembly_file << getMachineInstrStr(&mi) << "\n";
            assembly_without_debug_info_file << getMachineInstrStrWithoutDebugInfo(&mi) << "\n";
            assembly_opcode_file << getMachineInstrOpcode(&mi) << "\n";
            assembly_opcode_debug_file << getMachineInstrOpcodeAndDebugLocation(&mi) << "\n";
        }
    }
}

// This is how SIRegisterInfo figures out register type.
// It doesn't really seem to work very well for physical registers for some reason.
/*
bool LivenessVisualization::isSGPR(Register reg) const {
    return member_vars_.SI_TRI_->isSGPRReg(*member_vars_.MRI_, reg);
}

bool LivenessVisualization::isVGPR(Register reg) const {
    return member_vars_.SI_TRI_->isVGPR(*member_vars_.MRI_, reg);
}

bool LivenessVisualization::isAGPR(Register reg) const {
    return member_vars_.SI_TRI_->isAGPR(*member_vars_.MRI_, reg);
}
*/

// This is how AMDGPURegisterBankInfo figures out register type,
/*
bool LivenessVisualization::regIsFromRegisterBank(Register reg, unsigned bank_id) const {
    const RegisterBank *bank = member_vars_.AMD_RBI_->getRegBank(reg, *member_vars_.MRI_, *member_vars_.TRI_);
    return (bank->getID() == bank_id);
}
*/

bool LivenessVisualization::regIsFromRegisterBank(Register reg, unsigned bank_id) const {
    const RegisterBank *bank = member_vars_.AMD_RBI_->getRegBank(reg, *member_vars_.MRI_, *member_vars_.TRI_);
    std::string reg_str = getRegString(reg);
    bool sgpr_search = bank_id == AMDGPU::SGPRRegBankID;
    bool vgpr_search = bank_id == AMDGPU::VGPRRegBankID;
    bool has_vgpr_text = reg_str.find("VGPR") != std::string::npos;
    bool has_sgpr_text = reg_str.find("SGPR") != std::string::npos;
    bool force_sgpr = reg.isPhysical() && has_sgpr_text;
    bool force_vgpr = reg.isPhysical() && has_vgpr_text;

    // Cheap hack to get physical registers work properly
    if(force_sgpr) {
        assert(!force_vgpr);
        return sgpr_search;
    }
    if(force_vgpr) {
        assert(!force_sgpr);
        return vgpr_search;
    }

    if(bank != nullptr) {
        return (bank->getID() == bank_id);
    } else {
        // Note: Originally, did fall back to SI_TRI_ calls to deal w/
        // physical registers (which are dropped by getID method, 
        // which return nullptr). However, I had to correct w/ string searching,
        // so it's not clear that the alternate method is really necessary.
        switch(bank_id) {
            case AMDGPU::SGPRRegBankID:
                return member_vars_.SI_TRI_->isSGPRReg(*member_vars_.MRI_, reg);
                break;
            case AMDGPU::VGPRRegBankID:
                return member_vars_.SI_TRI_->isVGPR(*member_vars_.MRI_, reg);
                break;
            case AMDGPU::AGPRRegBankID:
                return member_vars_.SI_TRI_->isAGPR(*member_vars_.MRI_, reg);
                break;
            default:
                exit(1);
        }
    }
}

bool LivenessVisualization::isSGPR(Register reg) const {
    return regIsFromRegisterBank(reg, AMDGPU::SGPRRegBankID);
}

bool LivenessVisualization::isVGPR(Register reg) const {
    return regIsFromRegisterBank(reg, AMDGPU::VGPRRegBankID);
}

bool LivenessVisualization::isAGPR(Register reg) const {
    return regIsFromRegisterBank(reg, AMDGPU::AGPRRegBankID);
}

// MemberVars constructor
LivenessVisualization::MemberVars::MemberVars(const MachineFunction &fn, LiveIntervals *LIA, MachineDominatorTree *DT) {
    MF_ = &fn;
    LIA_ = LIA;
    DT_ = DT;
    MRI_ = &MF_->getRegInfo();
    indexes_ = LIA_->getSlotIndexes();
    TRI_ = MF_->getSubtarget().getRegisterInfo();
    TII_ = MF_->getSubtarget().getInstrInfo();
    sanitized_func_name_ = getSanitizedFuncName(MF_);
    GCN_ST_ = &MF_->getSubtarget<GCNSubtarget>();
    AMD_RBI_ = GCN_ST_->getRegBankInfo();
    SI_TRI_ = GCN_ST_->getRegisterInfo();
}

bool LivenessVisualization::runOnMachineFunction(MachineFunction &fn) {

    // Only care about GPU code.
    if(!(fn.getTarget().getTargetTriple().isAMDGPU() || fn.getTarget().getTargetTriple().isNVPTX())) {
        return false;
    }

    // Init variables.
    member_vars_ = MemberVars(fn, &getAnalysis<LiveIntervals>(), &getAnalysis<MachineDominatorTree>());

    outs() << "\n\nLivenessVisualization @ Function: " << member_vars_.MF_->getName() << "\n";

    // Open dot file
    std::ofstream dot_file;
    dot_file.open(member_vars_.sanitized_func_name_+ ".jbaile_lv_dot");
    dot_file << "digraph {\n";

    // Open text file
    std::ofstream text_file;
    text_file.open(member_vars_.sanitized_func_name_+ ".jbaile_lv_linear_reg_report");

    buildGraphBBs();
    emitGraphBBs(dot_file, text_file);

    dot_file << "}\n";


    // Emit assembly
    emit_assembly(fn);

    return false;
}

bool LivenessVisualization::doFinalization(Module &M) {
    // printf("LivenessVisualization doFinalization\n");
    MachineFunctionPass::doFinalization(M);
    return false;
}
