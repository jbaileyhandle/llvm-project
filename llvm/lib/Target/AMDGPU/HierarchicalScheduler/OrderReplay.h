//===- OrderReplay.h - Replay an instruction order from a file ---*- C++ -*-===//
//
// An order file gives one function's instruction order, region by region.
// misched.txt names order files with `order_file <path>` lines; after the
// configured scheduler has run, the hierarchical scheduler's pass puts each
// named function's regions in its file's order.
//
// File format (JSON):
//   {
//     "function": "<mangled function name>",
//     "regions": [
//       {"block": <block number>, "instrs": ["<MIR text>", ...]},
//       ...
//     ]
//   }
// Regions are listed in program order. Each instruction is named by its MIR
// text as the memory viewer's dump prints it. See
// llvm/docs/AMDGPUMemoryViewerReorderPlan.md.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_ORDERREPLAY_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_ORDERREPLAY_H

#include "llvm/ADT/StringRef.h"
#include <string>

namespace llvm {
namespace hierarchical_scheduler {

// Return the path of the order file that names `function_name` (mangled), or
// nullptr if none does. The returned pointer stays valid for the whole
// compile.
//
// The first call reads the "function" field of every order file in
// HierarchicalConfig::Get().order_files; later calls reuse what it read.
// Fatal on a file that can't be read, isn't valid JSON, or has no "function"
// string, and on two files naming the same function.
const std::string *FindOrderFile(StringRef function_name);

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_ORDERREPLAY_H
