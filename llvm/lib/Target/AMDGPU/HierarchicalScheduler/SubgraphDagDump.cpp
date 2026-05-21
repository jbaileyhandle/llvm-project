//===- SubgraphDagDump.cpp - Instrument subgraph formation as JSON ---------===//
//
// Implementation. See SubgraphDagDump.h for the contract.
//
//===----------------------------------------------------------------------===//

#include "SubgraphDagDump.h"

#include "GCNRegisterTracker.h"
#include "ScheduleConstructor.h"
#include "ScheduleGraph.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/Analysis/MachineInstrSchedulerConfig.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/raw_ostream.h"

#include <string>

namespace llvm {
namespace hierarchical_scheduler {

namespace {

// Root output directory, relative to the compile's CWD (where
// misched.txt is read from). Per-pass subdirectories live under it.
constexpr const char *kRootDir = "subgraph_dags";

// Active pass label, set by SubgraphDagDumpPassScope. Empty when no
// pass scope is active (e.g., a dump fired outside the occupancy /
// length passes), in which case dumps land in the "unknown" subdir.
std::string g_current_pass;

bool DumpEnabled() {
  return MachineInstrSchedulerConfig::GetConfig().HasSchedulingOption(
      MachineInstrSchedulerConfig::SchedulerOption::DumpSubgraphDag);
}

// Human-meaningful identity of a dumped region, derived from the
// graph at the dump point. graph_id is the universal unique key
// (every ScheduleGraph has one); function/block locate it in the
// source; occ_score (continuous occupancy of the input order) shows
// how pressured it is.
struct DumpIdentity {
  std::string function;
  std::string block;
  int64_t graph_id;
  int occ_score;
};

DumpIdentity DeriveIdentity(const ScheduleGraph &graph) {
  DumpIdentity id;
  id.graph_id = graph.GetId();
  id.occ_score = graph.GetInputScheduleConstructor()
                     .GetPressureTracker()
                     .GetContinuousOccupancyScore();
  id.function = "unknown";
  id.block = "unknown";
  // Function/block come from any real instruction in the region.
  for (const ScheduleNode *node : graph.GetTopoOrder()) {
    SUnit *su = node->IsSchedulingUnit() ? node->GetSUnit() : nullptr;
    if (su && su->getInstr()) {
      const MachineInstr *mi = su->getInstr();
      const MachineBasicBlock *mbb = mi->getParent();
      id.function = mi->getMF()->getName().str();
      // MIR blocks are usually unnamed; the number is always valid.
      // Mirror MIR's "bb.N[.name]" form so the block is identifiable.
      id.block = "bb." + std::to_string(mbb->getNumber());
      if (!mbb->getName().empty()) {
        id.block += "." + mbb->getName().str();
      }
      break;
    }
  }
  return id;
}

// Make `s` safe to embed in a filename: keep [A-Za-z0-9._-], replace
// the rest with '_', and cap the length (mangled C++ names can be
// enormous).
std::string SanitizeForFilename(StringRef s) {
  constexpr size_t kMaxLen = 120;
  std::string out;
  for (char c : s.take_front(kMaxLen)) {
    bool keep = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                (c >= '0' && c <= '9') || c == '.' || c == '_' || c == '-';
    out += keep ? c : '_';
  }
  return out.empty() ? std::string("unknown") : out;
}

// Append `s` to `out` as the contents of a JSON string (no surrounding
// quotes), escaping what JSON requires.
void AppendJsonEscaped(std::string &out, StringRef s) {
  for (char c : s) {
    switch (c) {
    case '"': out += "\\\""; break;
    case '\\': out += "\\\\"; break;
    case '\n': out += "\\n"; break;
    case '\r': out += "\\r"; break;
    case '\t': out += "\\t"; break;
    default:
      if (static_cast<unsigned char>(c) < 0x20) {
        char buf[8];
        std::snprintf(buf, sizeof(buf), "\\u%04x",
                      static_cast<unsigned char>(c));
        out += buf;
      } else {
        out += c;
      }
    }
  }
}

// Concise node label: opcode mnemonic for real instructions, debug
// name for the Entry/Exit sentinels.
std::string NodeLabel(const ScheduleNode *node) {
  if (!node->GetDebugName().empty()) {
    return node->GetDebugName().str();
  }
  SUnit *su = node->GetSUnit();
  if (su && su->getInstr()) {
    const MachineInstr *mi = su->getInstr();
    const TargetInstrInfo *tii = mi->getMF()->getSubtarget().getInstrInfo();
    return tii->getName(mi->getOpcode()).str();
  }
  return "n" + std::to_string(node->GetId());
}

// "instr" for real instruction nodes, "sentinel" for Entry/Exit.
const char *NodeKind(const ScheduleNode *node) {
  SUnit *su = node->GetSUnit();
  return (su && su->getInstr()) ? "instr" : "sentinel";
}

const char *EdgeKindName(ScheduleEdge::Kind kind) {
  switch (kind) {
  case ScheduleEdge::kData: return "data";
  case ScheduleEdge::kAnti: return "anti";
  case ScheduleEdge::kOutput: return "output";
  case ScheduleEdge::kBarrier: return "barrier";
  case ScheduleEdge::kMayAliasMem: return "may_alias";
  case ScheduleEdge::kMustAliasMem: return "must_alias";
  case ScheduleEdge::kArtificial: return "artificial";
  case ScheduleEdge::kSubgraphOrderEdge: return "subgraph_order";
  case ScheduleEdge::kCluster: return "cluster";
  case ScheduleEdge::kWeak: return "weak";
  }
  return "unknown";
}

// Node -> owning subgraph's id, for nodes that are members of some
// subgraph. Keyed by node pointer so it doesn't depend on any id
// space.
DenseMap<const ScheduleNode *, int64_t> BuildMembership(
    ArrayRef<std::unique_ptr<SubgraphInfo>> infos) {
  DenseMap<const ScheduleNode *, int64_t> node_to_subgraph;
  for (const auto &info : infos) {
    for (const ScheduleNode *member : info->members) {
      node_to_subgraph[member] = info->id;
    }
  }
  return node_to_subgraph;
}

void AppendMeta(std::string &out, const DumpIdentity &id, StringRef pass,
                int num_nodes, int num_instr_nodes, int num_subgraphs) {
  out += "  \"meta\": {\n";
  out += "    \"graph_id\": " + std::to_string(id.graph_id) + ",\n";
  out += "    \"function\": \"";
  AppendJsonEscaped(out, id.function);
  out += "\",\n    \"block\": \"";
  AppendJsonEscaped(out, id.block);
  out += "\",\n    \"pass\": \"";
  AppendJsonEscaped(out, pass);
  out += "\",\n    \"occ_score\": " + std::to_string(id.occ_score) + ",\n";
  out += "    \"num_nodes\": " + std::to_string(num_nodes) + ",\n";
  out += "    \"num_instr_nodes\": " + std::to_string(num_instr_nodes) + ",\n";
  out += "    \"num_subgraphs\": " + std::to_string(num_subgraphs) + "\n";
  out += "  },\n";
}

void AppendSubgraphs(std::string &out,
                     ArrayRef<std::unique_ptr<SubgraphInfo>> infos) {
  out += "  \"subgraphs\": [\n";
  for (int i = 0; i < static_cast<int>(infos.size()); ++i) {
    out += "    {\"id\": \"sg" + std::to_string(infos[i]->id) +
           "\", \"name\": \"";
    AppendJsonEscaped(out, infos[i]->debug_name);
    out += "\", \"size\": " + std::to_string(infos[i]->members.size()) + "}";
    out += (i + 1 < static_cast<int>(infos.size())) ? ",\n" : "\n";
  }
  out += "  ],\n";
}

void AppendNodes(std::string &out, ArrayRef<ScheduleNode *> topo,
                 const DenseMap<const ScheduleNode *, int64_t> &membership) {
  out += "    \"nodes\": [\n";
  for (int n = 0; n < static_cast<int>(topo.size()); ++n) {
    const ScheduleNode *node = topo[n];
    out += "      {\"data\": {\"id\": \"n" + std::to_string(node->GetId()) +
           "\", \"label\": \"";
    AppendJsonEscaped(out, NodeLabel(node));
    out += "\", \"detail\": \"";
    AppendJsonEscaped(out, node->ToString());
    out += "\", \"topo\": " + std::to_string(node->GetTopoIndex());
    out += ", \"kind\": \"" + std::string(NodeKind(node)) + "\"";
    auto it = membership.find(node);
    if (it != membership.end()) {
      out += ", \"subgraph\": \"sg" + std::to_string(it->second) + "\"";
    } else {
      out += ", \"subgraph\": null";
    }
    out += "}}";
    out += (n + 1 < static_cast<int>(topo.size())) ? ",\n" : "\n";
  }
  out += "    ],\n";
}

void AppendEdges(std::string &out, ArrayRef<ScheduleNode *> topo) {
  out += "    \"edges\": [\n";
  int edge_index = 0;
  bool first = true;
  for (const ScheduleNode *node : topo) {
    for (const ScheduleEdge &edge : node->Successors()) {
      if (!first) {
        out += ",\n";
      }
      first = false;
      out += "      {\"data\": {\"id\": \"e" + std::to_string(edge_index++) +
             "\", \"source\": \"n" + std::to_string(node->GetId()) +
             "\", \"target\": \"n" + std::to_string(edge.node_->GetId()) +
             "\", \"kind\": \"" + EdgeKindName(edge.kind_) +
             "\", \"latency\": " + std::to_string(edge.latency_) +
             ", \"weak\": " + (edge.IsWeakEdge() ? "true" : "false") + "}}";
    }
  }
  out += first ? "    ]\n" : "\n    ]\n";
}

std::string BuildJson(const ScheduleGraph &graph,
                      ArrayRef<std::unique_ptr<SubgraphInfo>> infos,
                      const DenseMap<const ScheduleNode *, int64_t> &membership,
                      const DumpIdentity &id, StringRef pass) {
  ArrayRef<ScheduleNode *> topo = graph.GetTopoOrder();
  int num_instr_nodes = 0;
  for (const ScheduleNode *node : topo) {
    if (NodeKind(node) == StringRef("instr")) {
      ++num_instr_nodes;
    }
  }

  std::string json;
  json.reserve(topo.size() * 128);
  json += "{\n";
  AppendMeta(json, id, pass, topo.size(), num_instr_nodes, infos.size());
  AppendSubgraphs(json, infos);
  json += "  \"elements\": {\n";
  AppendNodes(json, topo, membership);
  AppendEdges(json, topo);
  json += "  }\n";
  json += "}\n";
  return json;
}

// Write `json` to subgraph_dags/<pass>/<func>_g<graphId>_occ<score>.json.
// Fatal on any failure — a requested dump that silently goes missing
// would mislead.
void WriteDumpFile(const DumpIdentity &id, StringRef pass, StringRef json) {
  std::string dir = std::string(kRootDir) + "/" + pass.str();
  if (std::error_code ec = sys::fs::create_directories(dir)) {
    report_fatal_error(Twine("MaybeDumpSubgraphDag: cannot create '") + dir +
                       "': " + ec.message());
  }
  std::string path = dir + "/" + SanitizeForFilename(id.function) + "_g" +
                     std::to_string(id.graph_id) + "_occ" +
                     std::to_string(id.occ_score) + ".json";
  std::error_code ec;
  raw_fd_ostream os(path, ec, sys::fs::OF_Text);
  if (ec) {
    report_fatal_error(Twine("MaybeDumpSubgraphDag: cannot open '") + path +
                       "': " + ec.message());
  }
  os << json;
}

}  // namespace

SubgraphDagDumpPassScope::SubgraphDagDumpPassScope(StringRef pass_name)
    : previous_(g_current_pass) {
  g_current_pass = pass_name.str();
}

SubgraphDagDumpPassScope::~SubgraphDagDumpPassScope() {
  g_current_pass = previous_;
}

void MaybeDumpSubgraphDag(
    const ScheduleGraph &graph,
    ArrayRef<std::unique_ptr<SubgraphInfo>> infos) {
  if (!DumpEnabled()) {
    return;
  }
  // The hook must run before InsertSubgraphProxies, so the graph is
  // still flat. A proxy here means the hook is misplaced — fail loudly.
  for (const ScheduleNode *node : graph.GetTopoOrder()) {
    if (node->IsSubgraphProxy()) {
      report_fatal_error(
          "MaybeDumpSubgraphDag: graph already contains subgraph proxies; "
          "the dump hook must run before InsertSubgraphProxies");
    }
  }

  std::string pass = g_current_pass.empty() ? "unknown" : g_current_pass;
  DumpIdentity id = DeriveIdentity(graph);
  DenseMap<const ScheduleNode *, int64_t> membership = BuildMembership(infos);
  std::string json = BuildJson(graph, infos, membership, id, pass);
  WriteDumpFile(id, pass, json);
}

}  // namespace hierarchical_scheduler
}  // namespace llvm
