//===- SubgraphDagDump.h - Instrument subgraph formation as JSON -*- C++ -*-===//
//
// Instrumentation hook for subgraph formation. FormSubgraphs calls
// MaybeDumpSubgraphDag once membership has been computed and before
// proxies are inserted, so it observes the actual formation each
// scheduling pass performs on the real (still-flat) region graph.
//
// When the misched.txt DumpSubgraphDag option is set, it writes the
// region's flat dependency DAG with subgraph membership overlaid to a
// Cytoscape.js-shaped JSON file under
// ./subgraph_dags/<pass>/<func>_r<region>_g<graphId>_occ<score>.json, for
// viewing in HierarchicalScheduler/viz/subgraph_dag_viewer.html.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SUBGRAPHDAGDUMP_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SUBGRAPHDAGDUMP_H

#include "SubgraphInfo.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/StringRef.h"
#include <memory>
#include <string>

namespace llvm {
namespace hierarchical_scheduler {

class ScheduleGraph;

/// RAII marker for the scheduling pass currently running. The dump
/// hook fires deep inside FormSubgraphs and can't see which pass it
/// belongs to, so the scheduler wraps each pass (occupancy, length)
/// in one of these; MaybeDumpSubgraphDag reads the active label to
/// pick the output subdirectory. Scopes restore the previous label on
/// destruction, so nesting is safe.
class SubgraphDagDumpPassScope {
 public:
  explicit SubgraphDagDumpPassScope(StringRef pass_name);
  ~SubgraphDagDumpPassScope();
  SubgraphDagDumpPassScope(const SubgraphDagDumpPassScope &) = delete;
  SubgraphDagDumpPassScope &operator=(const SubgraphDagDumpPassScope &) =
      delete;

 private:
  std::string previous_;
};

/// RAII marker for the sorted region[i] currently being scheduled.
/// Like SubgraphDagDumpPassScope, the dump hook fires deep inside
/// FormSubgraphs and can't see which region it belongs to, so the
/// scheduler wraps each region's scheduling in one of these and
/// MaybeDumpSubgraphDag tags the output (filename + JSON meta) with the
/// index. -1 (the default when no scope is active) means the dump fired
/// outside a pass's per-region loop.
class SubgraphDagDumpRegionScope {
 public:
  explicit SubgraphDagDumpRegionScope(int region_index);
  ~SubgraphDagDumpRegionScope();
  SubgraphDagDumpRegionScope(const SubgraphDagDumpRegionScope &) = delete;
  SubgraphDagDumpRegionScope &operator=(const SubgraphDagDumpRegionScope &) =
      delete;

 private:
  int previous_;
};

/// Called by FormSubgraphs after membership is computed and before
/// proxies are inserted (so `graph` is flat and its edges are the real
/// dependency edges). Returns immediately unless the misched.txt
/// DumpSubgraphDag option is set.
///
/// `graph` is the flat region graph; `infos` is the membership about
/// to be installed (each SubgraphInfo's `members` are nodes of
/// `graph`). Writes one Cytoscape.js JSON file under
/// ./subgraph_dags/<active-pass>/. Reports a fatal error if the
/// directory or file cannot be written — a requested dump that
/// silently goes missing would mislead, so we fail loudly instead.
void MaybeDumpSubgraphDag(
    const ScheduleGraph &graph,
    ArrayRef<std::unique_ptr<SubgraphInfo>> infos);

}  // namespace hierarchical_scheduler
}  // namespace llvm

#endif  // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SUBGRAPHDAGDUMP_H
