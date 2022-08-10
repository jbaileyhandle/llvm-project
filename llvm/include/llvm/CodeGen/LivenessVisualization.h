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

    // Class used to build up information for a basic
    // block in the final dot file.
    class GraphBB {
    public:
        GraphBB(LivenessVisualization*, const MachineBasicBlock &MBB);

        // Link this GraphBB to its children GraphBB's
        void addChildren(std::unordered_map<const MachineBasicBlock*, GraphBB>& mbb_to_gbb);

        // Add information for the given slot index to the node.
        void addSlotIndex(const SlotIndex);

        // Color BB if it is a register pressure hotspot.
        void markHotspot(int function_max_virt_live);

        // Add text descriptor of connections to children to descriptor.
        void emitConnections(std::ofstream& dot_file) const;

        // Add node descriptor to descriptor.
        void emitNode(std::ofstream& dot_file) const;

        // Return max_virt_live.
        int getMaxVirtLive() const;

        // Return an xdot-friendly name for the basic block.
        std::string getSanitizedFuncName(const MachineBasicBlock &MBB);

        class RegSegment {
        public:
            RegSegment(const unsigned reg, const LiveRange::Segment *segment): reg_(reg), segment_(segment) {}
            unsigned reg_;
            const LiveRange::Segment *segment_;
        };

    private:

        // Add instructions at the SlotIndex to node description.
        void addInstructionAtSlotIndex(const SlotIndex);

        // Add instruction location at the SlotIndex to node description.
        void addInstructionLocationAtSlotIndex(const SlotIndex);

        // Add the virtual instruciton for the LiveInterval to node description
        // if it is live at at the given SlotIndex.
        void addRegistersAtSlotIndex(const SlotIndex);

        // Helper function to addRegistersAtSlotIndex for printing registers to label.
        void addSetOfLiveRegs(std::vector<RegSegment>& live_registers, std::string label);

        // Add left-justified newline to label.
        void addNewlineToLabel();

        // Return a string w/ graphviz attributes to mark this BB as a hotspot
        // Return an empty string if this BB is not a hotspot.
        std::string getHotspotAttr() const;

        // Return virtual registers that are live at the SlotIndex.
        std::vector<RegSegment> getLiveVirtRegsAtSlotIndex(const SlotIndex);

        // Return virtual registers that are live at the SlotIndex.
        std::vector<RegSegment> getLivePhysRegsAtSlotIndex(const SlotIndex);


        // The successor nodes of this basic block. 
        std::vector<GraphBB*> children_;

        // Slot indexes.
        SlotIndexes *indexes_;

        // For building up a descriptor (instructions, live registers) of
        // this basic block.
        std::string label_str_;

        const LivenessVisualization *LVpass_;

        // Unique name of the basic block.
        std::string name_;

        // Greatest number of virtual registers live at any point in the BB.
        int max_virt_live_ = 0;
        // "max_virt_live_" as a percent of function_max_virt_live_.
        float percent_max_virt_live_;

        // MachineBasicBlock which this graph node models.
        const MachineBasicBlock *MBB_;
    };


  private:
    // Build up the GraphBBs.
    void buildGraphBBs();

    // Emit the GraphBBs to dot_file.
    void emitGraphBBs(std::ofstream &dot_file) const;

    // Get the name of the function, modified cleanliness.
    static std::string getSanitizedFuncName(const MachineFunction *fn);

    // Map MachineBasicBlock's of the function to the GraphBB's
    // representing the basic blocks in the dot graph.
    std::unordered_map<const MachineBasicBlock*, GraphBB> mbb_to_gbb_;

    // Greatest number of virtual registers live at any point in the function.
    int function_max_virt_live_ = 0;

    LiveIntervals *LIA_;
    const MachineFunction* MF_;
    const MachineRegisterInfo* MRI_;
    const TargetRegisterInfo* TRI_;
    const TargetInstrInfo* TII_;
  };

} // end namespace llvm

#endif
