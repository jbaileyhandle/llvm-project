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

    /// Pass entry point;
    bool doInitialization(Module &M) override;
    bool runOnMachineFunction(MachineFunction&) override;
    bool doFinalization(Module &M) override;

  private:

    // Class used to build up information for a basic
    // block in the final dot file.
    class GraphBB {
    public:
        GraphBB(LivenessVisualization*, MachineBasicBlock &MBB);

        std::string EmitDescription();
        void AddInstructionAtSlotIndex(SlotIndex);
        void AddVirtReg(LiveInterval&);

        std::list<GraphBB*> children;
    private:
        LivenessVisualization *LVpass;
        std::string name;
        std::string label_str;
        std::unique_ptr<raw_string_ostream> label_ostream_ptr;
    };

    LiveIntervals *LIA;
    MachineFunction* MF;
    MachineRegisterInfo* MRI;
    const TargetRegisterInfo* TRI;
    const TargetInstrInfo* TII;
  };

} // end namespace llvm

#endif
