//===- SearchOutcomeLog.h - Per-search outcome CSV ------------*- C++ -*-===//
//
// Records one row per region schedule search to search_outcomes.csv in the
// compile CWD, for offline parsing alongside the subgraph_dags dumps. A plain
// (decompose-off) region is one "region" row; a decompose region is one row
// per subgraph (slot sub0/sub1/...) plus an "outer" row carrying the region
// totals. Gated by HierarchicalConfig.dump_search_outcomes; no-op otherwise.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHOUTCOMELOG_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHOUTCOMELOG_H

#include "SearchTerminationCause.h"
#include <optional>
#include <string>

namespace llvm {
namespace hierarchical_scheduler {

// One search outcome. Region-level totals (vgpr/sgpr/len/improved) are set on
// the "region"/"outer" slot; subgraph rows carry only the per-search outcome
// and leave the optionals unset. Length fields stay unset for occupancy rows.
struct SearchOutcome {
  std::string function;  // MachineFunction name; region indices reset per
                         // function, so this separates kernels in the file
  std::string pass;   // "occ" | "len"
  int region;
  std::string slot;   // "region" | "outer" | "sub0" | "sub1" | ...
  int nodes;
  SearchTerminationCause term_cause;
  std::string winner;            // "bfs" | "dfs" | "input" | "none"
  std::optional<float> bfs_pct;  // BFS depth reached / nodes; unset if N/A
  // Per-backend throughput, where each ran (both set on a DFS fallback).
  // The writer derives bfs_rate/dfs_rate = steps/ms columns from these.
  std::optional<int> bfs_ms, dfs_ms;
  std::optional<int> bfs_steps, dfs_steps;
  std::optional<int> orig_vgpr, orig_sgpr, fin_vgpr, fin_sgpr;
  std::optional<int> orig_len, fin_len;
  std::optional<bool> improved;
};

// Append one row (no-op unless dump_search_outcomes is set). Buffered; written
// on FlushSearchOutcomes().
void RecordSearchOutcome(const SearchOutcome &row);

// Write buffered rows to ./search_outcomes.csv (header once) and clear them.
// Call at end of scheduling. No-op unless dump_search_outcomes is set.
void FlushSearchOutcomes();

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SEARCHOUTCOMELOG_H
