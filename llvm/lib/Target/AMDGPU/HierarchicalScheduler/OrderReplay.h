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
//       {"index": <region index>, "block": <block number>,
//        "instrs": ["<MIR text>", ...]},
//       ...
//     ]
//   }
// A region is named by its index (RegionInfo::GetIndex), the same number the
// memory viewer's dump gives it. Each instruction is named by its MIR text as
// the dump prints it (InstrText). See
// llvm/docs/AMDGPUMemoryViewerReorderPlan.md.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_ORDERREPLAY_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_ORDERREPLAY_H

#include "llvm/ADT/StringRef.h"
#include "llvm/CodeGen/ScheduleDAG.h"
#include <map>
#include <string>
#include <vector>

namespace llvm {
namespace hierarchical_scheduler {

// One region of an order file: its block's number, and its instructions'
// texts (InstrText) in the order to replay.
struct OrderFileRegion {
  int block_number = -1;
  std::vector<std::string> instr_texts;
};

// One order file, read whole.
struct OrderFile {
  std::string path;
  std::string function_name;
  // Keyed by region index (RegionInfo::GetIndex).
  std::map<int, OrderFileRegion> regions;
};

// Return the order file that names `function_name` (mangled), or nullptr if
// none does. The returned pointer stays valid for the whole compile.
//
// The first call reads every order file in HierarchicalConfig::Get().order_files;
// later calls reuse what it read. Fatal on a file that can't be read, isn't
// valid JSON, or doesn't follow the format above (including a region index
// listed twice), and on two files naming the same function.
const OrderFile *FindOrderFile(StringRef function_name);

// Return a region's instructions in the order `order_file` gives them.
// `sunits` are the region's SUnits as buildSchedGraph made them;
// `region_index` and `block_number` identify the region.
//
// Fatal if the file has no region with that index, if that region's block
// number or instruction texts (each text the same number of times) differ
// from the compiler's region, or if the file's order puts an instruction
// above one it depends on. Each text in the file's region is then paired
// with the first instruction, in the region's current order, that has that
// text and isn't paired yet.
//
// The caller builds the region's graph beforehand and applies the returned
// order afterwards, as with ComputeMaliciousSchedule.
std::vector<SUnit *> ComputeReplayOrder(const OrderFile &order_file,
                                        int region_index, int block_number,
                                        std::vector<SUnit> &sunits);

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_ORDERREPLAY_H
