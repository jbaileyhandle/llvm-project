//===- LivenessVisualizer.h - Live Interval Visualization -----------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements a liveness visualization pass.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CODEGEN_LIVENESSVISUALIZATION_H
#define LLVM_CODEGEN_LIVENESSVISUALIZATION_H

//#include "../lib/Target/AMDGPU/GCNSubtarget.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/IndexedMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/LiveInterval.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/SlotIndexes.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/MC/LaneBitmask.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/ErrorHandling.h"
#include <cassert>
#include <cstdint>
#include <fstream>
#include <unordered_map>
#include <utility>

namespace llvm {

extern cl::opt<bool> UseSegmentSetForPhysRegs;

class AAResults;
class BitVector;
class LiveIntervalCalc;
class MachineBlockFrequencyInfo;
class MachineDominatorTree;
class MachineFunction;
class MachineInstr;
class MachineRegisterInfo;
class raw_ostream;
class TargetInstrInfo;
class VirtRegMap;

  class LivenessVisualization : public MachineFunctionPass {
  public:
    static char ID;

    LivenessVisualization();
    ~LivenessVisualization() override;

    void getAnalysisUsage(AnalysisUsage &AU) const override;

    // Pass entry points;
    bool doInitialization(Module &M) override;
    bool runOnMachineFunction(MachineFunction&) override;
    bool doFinalization(Module &M) override;

    // Helper to get register as human readable string
    std::string getRegString(Register reg) const;

    class GraphBB;

    // Store information about liveness at a SlotIndex.
    class SlotIndexInfo {
    public:
        SlotIndexInfo(SlotIndex si, const GraphBB *graph_bb, const LivenessVisualization *LVpass);

        // Return a string representing the SlotIndexInfo.
        // "registers" indicates for which registers to report register
        // liveness.
        std::string toString() const;

        // For the register "reg" at this slot index, return
        // ' ' if the register is not live
        // ':' if the register is live but not read or written by mi_
        // 'v' if the register is written
        // '^' if the register is read
        // 'X' if the register is both read and written
        char getRegUsageSymbol(Register reg) const;

        int getNumLiveVectorRegisters() const {
            return (int)live_vector_registers_.size();
        }

        void setLivenessPercent(int function_max_vector_registers_live) {
            percent_vector_live_registers_ = (float)getNumLiveVectorRegisters() / (float)function_max_vector_registers_live;
        }

        // TODO: Move some of this to private?
        const SlotIndex si_;
        float percent_vector_live_registers_;
        std::vector<Register> live_virtual_vector_registers_;
        std::vector<Register> live_virtual_scalar_registers_;
        std::vector<Register> live_physical_vector_registers_;
        std::vector<Register> live_physical_scalar_registers_;
        std::vector<Register> live_vector_registers_;
        std::vector<Register> live_scalar_registers_;
        std::vector<Register> live_registers_;
        const MachineInstr * const mi_;
        std::string mi_str_;
        std::string src_location_;

        const GraphBB *graph_bb_;
        const LivenessVisualization *LVpass_;

    private:
        // Constructor helper to save string of source locatoin of slot index
        void addInstructionLocationStr();

        // Constructor helpers to save registers at slot index
        void addRegisters();
        void addLiveVirtRegs();
        void addLivePhysRegs();
        void addCombinedRegs();

        // Constructor helper to save instruction string
        void addInstructionStr();
    };

    // Class used to build up information for a basic
    // block in the final dot file.
    class GraphBB {
    public:
        GraphBB(LivenessVisualization*, const MachineBasicBlock *MBB);

        // Link this GraphBB to its children GraphBB's
        void addChildren(std::unordered_map<const MachineBasicBlock*, GraphBB>& mbb_to_gbb);

        // Mark register usage as percent of max
        void markHotspot(int function_max_vector_registers_live);

        // Add text descriptor of connections to children to descriptor.
        void emitXdotConnections(std::ofstream& dot_file) const;

        // Write a linear / text report showing register liveness for the BB.
        void emitTextReport() const;

        // Add node descriptor to descriptor.
        void emitXdotNode(std::ofstream& dot_file) const;

        // Vector registers that are live in the BB, ordered by first
        // instance of liveness
        std::vector<Register> getLiveVectorRegsInBB() const {
            return live_vector_regs_in_BB_;
        }

        // Get start and end Slot Indexes
        SlotIndex getStartIdx() const;
        SlotIndex getEndIdx() const;

        std::string getName() const {
            return name_;
        }

        // Generate a string showing register liveness in the BB.
        // title is placed in the first line of the stirng.
        // newline is inserted between each line.
        std::string getLinearReport(std::string title, std::string newline) const;

        // Return max_vector_registers_live_
        int getMaxVectorRegistersLive() const;

        class RegSegment {
        public:
            RegSegment(const unsigned reg, const LiveRange::Segment *segment): reg_(reg), segment_(segment) {}
            unsigned reg_;
            const LiveRange::Segment *segment_;
        };

    private:

        // Construcotr helper to populate vector registers that are live in the BB, ordered
        // by first instance of liveness.
        void addLiveVectorRegsInBB();

        // Construcotr helper to determine the greatest number of vector registers
        // live at once
        void setMaxLiveVectorRegs();

        // Return a string w/ graphviz attributes to mark this BB as a hotspot
        // Return an empty string if this BB is not a hotspot.
        std::string getHotspotAttr() const;

        // Return physical registers that are live at the SlotIndex.
        std::vector<RegSegment> getLivePhysRegsAtSlotIndex(const SlotIndex);

        // Helper function to generate header of register portion of linear report
        std::string getLinearReportHeaderStr() const;

        // Return an xdot-friendly name for the basic block.
        std::string getSanitizedMBBName() const;

        // Get a report showing register liveness for the BB.
        std::string getLinearReport() const;

        // List of info about SlotIndexes in BB in order of instructions.
        // Really, want this to be std::vector, but this won't compile
        // becuase operator= of SlotIndexInfo is not valid because of
        // const member variables.
        std::list<SlotIndexInfo> si_info_list_;

        // Vector registers that are live in the BB, ordered by first
        // instance of liveness
        std::vector<Register> live_vector_regs_in_BB_;

        // The successor nodes of this basic block. 
        std::vector<GraphBB*> children_;

        // Unique name of the basic block.
        std::string name_;

        // Greatest number of vector registers live at any point in the BB.
        int max_vector_registers_live_ = 0;

        // "max_vector_live_" as a percent of function_max_vector_live_.
        float percent_max_vector_registers_live_;

        // MachineBasicBlock which this graph node models.
        const MachineBasicBlock *MBB_;

        // Liveness pass
        const LivenessVisualization *LVpass_;

        // Slot indexes.
        SlotIndexes *indexes_;

    };


  private:
    // Build up the GraphBBs.
    void buildGraphBBs();

    // Emit the GraphBBs to dot_file.
    void emitGraphBBs(std::ofstream &dot_file) const;

    // Get the name of the function, modified cleanliness.
    static std::string getSanitizedFuncName(const MachineFunction *fn);

    // Initialize member variables to run pass on a new function.
    void initVarsPerFunction(const MachineFunction &fn);

    // Use a struct to hold members so that we don't forget to reset
    // a member for a new function.
    class MemberVars {
    public:
        MemberVars() {}
        MemberVars(const MachineFunction &fn, LiveIntervals *LIA);

        // Map MachineBasicBlock's of the function to the GraphBB's
        // representing the basic blocks in the dot graph.
        std::unordered_map<const MachineBasicBlock*, GraphBB> mbb_to_gbb_;

        // Greatest number of vector registers live at any point in the function.
        int function_max_vector_registers_live_ = 0;

        std::string sanitized_func_name_;
        LiveIntervals *LIA_ = nullptr;
        SlotIndexes *indexes_ = nullptr;
        const MachineFunction *MF_ = nullptr;
        const MachineRegisterInfo *MRI_ = nullptr;
        const TargetRegisterInfo *TRI_ = nullptr;
        const TargetInstrInfo *TII_ = nullptr;
        //const GCNSubtarget GCN_substarget_;
    };

    MemberVars member_vars_;
  };

} // end namespace llvm

#endif
