//===- InstrText.h - One-line text of a MachineInstr ------------*- C++ -*-===//
//
// The text the viewers' dumps (MemoryViz, ScheduleLengthAnalysis) print for an
// instruction, and that order files name instructions by (OrderReplay). It is
// defined once so an order file made from a dump always matches the compiler.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_INSTRTEXT_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_INSTRTEXT_H

#include <string>

namespace llvm {

class MachineInstr;

namespace hierarchical_scheduler {

// One line of an instruction's MIR text: no debug location, newlines and tabs
// flattened to spaces, leading and trailing spaces trimmed.
std::string InstrText(const MachineInstr &mi);

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_INSTRTEXT_H
