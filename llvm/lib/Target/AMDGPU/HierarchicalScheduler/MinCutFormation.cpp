//===- MinCutFormation.cpp - dagP min-cut subgraph formation -------------===//
//
// Group a region's instructions by acyclic min-cut of the data-dependency
// DAG, computed in-process by the linked dagP partitioner. The region is
// serialized to a temp .dot, read back with dagP_read_graph, and partitioned
// with dagP_partition_from_dgraph — all in this process (no child process).
// Building dagP's in-memory graph directly (skipping the temp file) is a
// possible later optimization; for now we reuse dagP's own .dot reader.
//
// Map-back is fact-based (read from dagP's own output, not guessed):
//   * .dot node names are graph_local_ids.
//   * dagP renumbers names to internal ids and records the translation in
//     "<input>.nodemappings" ("<name> <internal>" per line).
//   * dagP_partition_from_dgraph fills parts[1..nVrtx], indexed by internal id.
//   => part(node) = parts[ nodemappings[node.GetGraphLocalId()] ].
//
//===----------------------------------------------------------------------===//

#include "MinCutFormation.h"

#include "ScheduleGraph.h"
#include "SubgraphInfo.h"

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/Twine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/raw_ostream.h"

#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

// dagP is C. dagP.h is extern "C"-guarded, but dgraph.h / option.h are not, so
// wrap all three. Included last so dagP's macros don't leak into the LLVM
// headers above.
extern "C" {
#include "dagP.h"
#include "dgraph.h"
#include "option.h"
}

namespace llvm {
namespace hierarchical_scheduler {
namespace {

// Subgraphs smaller than this are dropped (proxy overhead outweighs the
// benefit), matching SubgraphFormation's EmitIfLargeEnough.
constexpr int kMinSubgraphSize = 2;

// Edge weights handed to dagP. dagP minimizes the total weight of cut edges,
// and it requires every weight to be positive: internally it treats a node's
// weighted edge-count to a part being zero as "no neighbor in that part," and
// relies on that for both its refinement heap and its acyclicity test. A
// zero-weight edge is invisible to that count, which corrupts the heap (dagP
// aborts) and can fool the acyclicity check. So every strong edge gets a
// positive weight. Data edges are the real objective (a register-pressure
// proxy) and get a large weight; non-data strong edges exist only to enforce
// acyclicity and get weight 1, so they break ties but never outweigh keeping a
// data edge uncut. ecType is 64-bit (long long) in dagP, so there is no
// overflow risk at these magnitudes.
constexpr int kDataEdgeWeight = 1000;
constexpr int kNonDataEdgeWeight = 1;

// The region's real instruction nodes (excludes entry/exit sentinels; no
// proxies exist at formation time). Pointers stay valid: nodes_ has reserved
// capacity, so the later InsertSubgraphProxies won't reallocate.
SmallVector<ScheduleNode *, 64> CollectRealNodes(ScheduleGraph &graph) {
  SmallVector<ScheduleNode *, 64> real_nodes;
  for (ScheduleNode &node : graph.Nodes()) {
    if (node.IsRealInstruction()) {
      real_nodes.push_back(&node);
    }
  }
  return real_nodes;
}

// Serialize the real-node subgraph to `dot_path` in dagP .dot format. Every
// strong edge between two real nodes is emitted so acyclicity is enforced over
// all of them. Data edges (the cut objective) carry kDataEdgeWeight; non-data
// strong edges, present only for acyclicity, carry kNonDataEdgeWeight. All
// weights are positive (see the constants above for why). Returns the number
// of edges written.
int WriteDotFile(ArrayRef<ScheduleNode *> real_nodes, StringRef dot_path) {
  std::error_code ec;
  raw_fd_ostream out(dot_path, ec, sys::fs::OF_Text);
  if (ec) {
    report_fatal_error(Twine("MinCutFormation: cannot open temp dot file '") +
                       dot_path + "': " + ec.message());
  }
  out << "digraph G {\n";
  // dagP's reader rejects an edge naming a node it hasn't seen declared, so
  // declare every node first.
  for (const ScheduleNode *n : real_nodes) {
    out << n->GetGraphLocalId() << ";\n";
  }
  int num_edges = 0;
  for (const ScheduleNode *n : real_nodes) {
    // Collapse multi-edges between the same ordered pair (e.g. a data edge
    // plus an anti/output edge to the same successor) into a single edge:
    // dagP requires a simple graph and corrupts its heaps on duplicates. The
    // collapsed weight is the SUM of the parallel edges' weights, so the
    // simple-graph cut cost matches the true multigraph cut cost (each data
    // dependency crossing a boundary is another value live across it).
    std::map<int, int> succ_weight; // successor graph_local_id -> summed weight
    for (const ScheduleEdge &e : n->Successors()) {
      if (!e.IsStrongEdge() || !e.node_->IsRealInstruction() || e.node_ == n) {
        continue;
      }
      succ_weight[e.node_->GetGraphLocalId()] +=
          e.IsDataEdge() ? kDataEdgeWeight : kNonDataEdgeWeight;
    }
    for (const auto &succ : succ_weight) {
      out << n->GetGraphLocalId() << "->" << succ.first
          << " [weight=" << succ.second << "];\n";
      ++num_edges;
    }
  }
  out << "}\n";
  out.close();
  return num_edges;
}

// Partition the .dot at `dot` into `k` parts with dagP, in-process. Returns the
// part id of each vertex indexed by dagP internal id (result[1..nVrtx]; index 0
// unused). As a side effect dagP_read_graph writes "<dot>.nodemappings". All of
// dagP's C API + manual memory lives here. const_cast is safe: dagP reads the
// path / copies it.
std::vector<int> PartitionDotFile(const std::string &dot, int k,
                                  const MinCutSettings &settings) {
  MLGP_option opt;
  dagP_init_parameters(&opt, k);
  dagP_init_filename(&opt, const_cast<char *>(dot.c_str()));
  opt.ratio = settings.imbalance_ratio;
  opt.seed = settings.seed;
  opt.use_binary_input = 0; // don't create/read a .bin cache
  // dagP's default initial partitioner (IP_CONPAR, constraint partitioning)
  // routes through the undirected partitioner, which needs METIS/Scotch — not
  // linked here. dagP_init_parameters leaves the default in place (only the
  // rMLGP CLI parser swaps in a native fallback), so force dagP's native
  // greedy initial partitioner ourselves.
  opt.conpar = 0;
  opt.inipart = IP_GGG_TWO;

  dgraph G;
  dagP_read_graph(const_cast<char *>(dot.c_str()), &G, &opt);

  idxType *raw = static_cast<idxType *>(
      calloc(static_cast<size_t>(G.nVrtx) + 1, sizeof(idxType)));
  if (raw == nullptr) {
    report_fatal_error("MinCutFormation: parts allocation failed");
  }
  dagP_partition_from_dgraph(&G, &opt, raw);

  std::vector<int> parts(static_cast<size_t>(G.nVrtx) + 1, 0);
  for (idxType i = 1; i <= G.nVrtx; ++i) {
    parts[i] = static_cast<int>(raw[i]);
  }

  free(raw);
  dagP_free_graph(&G);
  dagP_free_option(&opt);
  return parts;
}

// Parse dagP's "<name> <internal>" lines into name(graph_local_id) -> internal.
std::map<int, int> ParseNodeMappings(const std::string &path) {
  std::ifstream in(path);
  if (!in) {
    report_fatal_error(Twine("MinCutFormation: cannot open '") + path + "'");
  }
  std::map<int, int> name_to_internal;
  int name, internal;
  while (in >> name >> internal) {
    name_to_internal[name] = internal;
  }
  return name_to_internal;
}

// Map each real node to its part (part(node) = parts[nodemappings[id]]), group
// by part, and build a SubgraphInfo per group, dropping singletons.
std::vector<std::unique_ptr<SubgraphInfo>>
BuildInfosFromPartition(ArrayRef<ScheduleNode *> real_nodes,
                        const std::map<int, int> &name_to_internal,
                        const std::vector<int> &parts) {
  std::map<int, SmallVector<ScheduleNode *, 16>> part_to_members;
  for (ScheduleNode *node : real_nodes) {
    auto it = name_to_internal.find(node->GetGraphLocalId());
    if (it == name_to_internal.end()) {
      report_fatal_error("MinCutFormation: node absent from nodemappings");
    }
    const int internal = it->second;
    if (internal < 1 || internal >= static_cast<int>(parts.size())) {
      report_fatal_error("MinCutFormation: internal id out of range");
    }
    part_to_members[parts[internal]].push_back(node);
  }

  std::vector<std::unique_ptr<SubgraphInfo>> infos;
  for (auto &kv : part_to_members) {
    if (static_cast<int>(kv.second.size()) < kMinSubgraphSize) {
      continue;
    }
    infos.push_back(std::make_unique<SubgraphInfo>(
        kv.second, "mincut_subgraph_" + std::to_string(kv.first)));
  }
  return infos;
}

} // namespace

std::vector<std::unique_ptr<SubgraphInfo>>
BuildSubgraphInfosByMinCut(ScheduleGraph &graph,
                           const MinCutSettings &settings) {
  SmallVector<ScheduleNode *, 64> real_nodes = CollectRealNodes(graph);
  const int n = static_cast<int>(real_nodes.size());

  // k = ceil(n / target), clamped to max_parts when set. Skip regions too
  // small to yield 2+ subgraphs.
  int k = (n + settings.target_subgraph_size - 1) /
          settings.target_subgraph_size;
  if (settings.max_parts > 0 && k > settings.max_parts) {
    k = settings.max_parts;
  }
  if (k < 2) {
    return {};
  }

  SmallString<128> dot_path;
  if (std::error_code ec =
          sys::fs::createTemporaryFile("mincut", "dot", dot_path)) {
    report_fatal_error(Twine("MinCutFormation: createTemporaryFile failed: ") +
                       ec.message());
  }
  const std::string dot(dot_path.begin(), dot_path.end());

  if (WriteDotFile(real_nodes, dot) == 0) {
    // No strong edges among real nodes — nothing to cut, and dagP rejects an
    // empty edge set.
    sys::fs::remove(dot);
    return {};
  }

  const std::vector<int> parts = PartitionDotFile(dot, k, settings);
  const std::map<int, int> name_to_internal =
      ParseNodeMappings(dot + ".nodemappings");

  sys::fs::remove(dot);
  sys::fs::remove(dot + ".nodemappings");

  return BuildInfosFromPartition(real_nodes, name_to_internal, parts);
}

} // namespace hierarchical_scheduler
} // namespace llvm
