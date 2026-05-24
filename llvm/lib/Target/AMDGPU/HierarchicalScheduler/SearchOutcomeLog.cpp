//===- SearchOutcomeLog.cpp - Per-search outcome CSV ---------------------===//
//
// Implementation. See SearchOutcomeLog.h for the contract.
//
//===----------------------------------------------------------------------===//

#include "SearchOutcomeLog.h"

#include "HierarchicalConfig.h"
#include <fstream>
#include <vector>

namespace llvm {
namespace hierarchical_scheduler {

namespace {

constexpr const char *kFile = "search_outcomes.csv";
std::vector<SearchOutcome> g_rows;

bool Enabled() { return HierarchicalConfig::Get().dump_search_outcomes; }

const char *CauseStr(SearchTerminationCause c) {
  switch (c) {
  case SearchTerminationCause::kFullyExplored:
    return "fully_explored";
  case SearchTerminationCause::kTimedOut:
    return "timed_out";
  case SearchTerminationCause::kPolicySatisfied:
    return "policy_satisfied";
  }
  return "unknown";
}

void AppendOpt(std::string &out, const std::optional<int> &v) {
  out += ',';
  if (v.has_value()) {
    out += std::to_string(*v);
  }
}

void AppendOpt(std::string &out, const std::optional<float> &v) {
  out += ',';
  if (v.has_value()) {
    out += std::to_string(*v);
  }
}

}  // namespace

void RecordSearchOutcome(const SearchOutcome &row) {
  if (!Enabled()) {
    return;
  }
  g_rows.push_back(row);
}

void FlushSearchOutcomes() {
  if (!Enabled() || g_rows.empty()) {
    return;
  }
  std::ofstream f(kFile, std::ios::app);
  if (f.tellp() == 0) {
    f << "pass,region,slot,nodes,term_cause,winner,bfs_pct,"
         "orig_vgpr,orig_sgpr,fin_vgpr,fin_sgpr,orig_len,fin_len,improved\n";
  }
  for (const SearchOutcome &r : g_rows) {
    std::string line = r.pass + "," + std::to_string(r.region) + "," + r.slot +
                       "," + std::to_string(r.nodes) + "," +
                       CauseStr(r.term_cause) + "," + r.winner;
    AppendOpt(line, r.bfs_pct);
    AppendOpt(line, r.orig_vgpr);
    AppendOpt(line, r.orig_sgpr);
    AppendOpt(line, r.fin_vgpr);
    AppendOpt(line, r.fin_sgpr);
    AppendOpt(line, r.orig_len);
    AppendOpt(line, r.fin_len);
    line += ',';
    if (r.improved.has_value()) {
      line += *r.improved ? "1" : "0";
    }
    f << line << "\n";
  }
  g_rows.clear();
}

}  // namespace hierarchical_scheduler
}  // namespace llvm
