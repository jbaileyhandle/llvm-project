//===- HierarchicalConfig.cpp - Build typed HS config -----------*- C++ -*-===//
//
// See HierarchicalConfig.h. Build() resolves, per pass, built-in
// defaults -> named preset -> explicit scoped keys, maps strings to the
// typed enums, validates the axis constraints, and fatals on any
// unknown scope, key, or value.
//
//===----------------------------------------------------------------------===//

#include "HierarchicalConfig.h"

#include "llvm/Analysis/MachineInstrSchedulerConfig.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/Twine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

#include <cstdlib>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// Render "<scope>.<key>" (or just "<key>" for the empty top-level scope)
// for error messages.
std::string ScopedKeyName(StringRef scope, StringRef key) {
  return scope.empty() ? key.str() : (scope.str() + "." + key.str());
}

[[noreturn]] void BadValue(StringRef scope, StringRef key, StringRef val,
                           StringRef expected) {
  report_fatal_error(Twine("HierarchicalConfig: invalid value '") + val +
                     "' for '" + ScopedKeyName(scope, key) + "' (expected " +
                     expected + ")");
}

[[noreturn]] void UnknownKey(StringRef scope, StringRef key) {
  report_fatal_error(Twine("HierarchicalConfig: unknown setting '") +
                     ScopedKeyName(scope, key) + "'");
}

// --- value parsers (each fatals on an unrecognized value) ---

SubgraphFormationStrategy ParseFormation(StringRef scope, StringRef key, StringRef v) {
  if (v == "none") {
    return SubgraphFormationStrategy::kNone;
  }
  if (v == "domtree") {
    return SubgraphFormationStrategy::kDomTree;
  }
  if (v == "mincut") {
    return SubgraphFormationStrategy::kMinCut;
  }
  BadValue(scope, key, v, "none|domtree|mincut");
}

Search ParseSearch(StringRef scope, StringRef key, StringRef v) {
  if (v == "dfs") {
    return Search::kDfs;
  }
  if (v == "bfsdp") {
    return Search::kBfsDp;
  }
  if (v == "bfsdp+dfs") {
    return Search::kBfsDpDfs;
  }
  BadValue(scope, key, v, "dfs|bfsdp|bfsdp+dfs");
}

SubgraphScheduleMode ParseMode(StringRef scope, StringRef key, StringRef v) {
  if (v == "serialized") {
    return SubgraphScheduleMode::kSerialized;
  }
  if (v == "interleaved") {
    return SubgraphScheduleMode::kInterleaved;
  }
  BadValue(scope, key, v, "serialized|interleaved");
}

LengthPolicy ParseLengthPolicy(StringRef scope, StringRef key, StringRef v) {
  if (v == "min") {
    return LengthPolicy::kMin;
  }
  if (v == "min+refine-ilp") {
    return LengthPolicy::kMinRefineIlp;
  }
  if (v == "min+refine-occupancy") {
    return LengthPolicy::kMinRefineOccupancy;
  }
  if (v == "max") {
    return LengthPolicy::kMax;
  }
  BadValue(scope, key, v, "min|min+refine-ilp|min+refine-occupancy|max");
}

bool ParseBool(StringRef scope, StringRef key, StringRef v) {
  if (v == "true" || v == "on") {
    return true;
  }
  if (v == "false" || v == "off") {
    return false;
  }
  BadValue(scope, key, v, "true|false");
}

int ParseInt(StringRef scope, StringRef key, StringRef v) {
  int out = 0;
  // getAsInteger returns true on failure (rejects trailing junk).
  if (v.getAsInteger(10, out)) {
    BadValue(scope, key, v, "an integer");
  }
  return out;
}

float ParseFloat(StringRef scope, StringRef key, StringRef v) {
  std::string s = v.str();
  char *end = nullptr;
  float out = std::strtof(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0') {
    BadValue(scope, key, v, "a number");
  }
  return out;
}

// --- per-key application ---

// Apply a key shared by both passes (formation + its params + mode) onto
// the referenced fields. Returns true if `key` was one of those keys
// (handled), false otherwise (so the caller can try its pass-specific
// keys, then fatal on a genuine unknown).
bool ApplyFormationKey(StringRef scope, StringRef key, StringRef val,
                       FormationConfig &fc) {
  if (key == "formation") {
    fc.strategy = ParseFormation(scope, key, val);
    return true;
  }
  if (key == "mode") {
    fc.mode = ParseMode(scope, key, val);
    return true;
  }
  if (key == "formation.ratio") {
    fc.min_cut.imbalance_ratio = ParseFloat(scope, key, val);
    return true;
  }
  if (key == "formation.target_size") {
    fc.min_cut.target_subgraph_size = ParseInt(scope, key, val);
    return true;
  }
  return false;
}

void ApplyOccupancyKey(StringRef key, StringRef val, OccupancyConfig &c) {
  StringRef scope = "occupancy";
  if (ApplyFormationKey(scope, key, val, c.formation)) {
    return;
  }
  if (key == "search") {
    c.search = ParseSearch(scope, key, val);
    return;
  }
  if (key == "decompose") {
    c.decompose = ParseBool(scope, key, val);
    return;
  }
  if (key == "search.timeout") {
    c.timeout_ms = ParseInt(scope, key, val);
    return;
  }
  if (key == "search.fallback_timeout") {
    c.fallback_timeout_ms = ParseInt(scope, key, val);
    return;
  }
  UnknownKey(scope, key);
}

void ApplyLengthKey(StringRef key, StringRef val, LengthConfig &c) {
  StringRef scope = "length";
  if (ApplyFormationKey(scope, key, val, c.formation)) {
    return;
  }
  if (key == "policy") {
    c.policy = ParseLengthPolicy(scope, key, val);
    return;
  }
  UnknownKey(scope, key);
}

// --- presets ---
//
// A preset is a named bundle of "key value" pairs applied to a pass
// scope BEFORE explicit keys (which override). Applying a preset runs
// its pairs through the same per-scope key parser, so a preset key the
// target pass doesn't have (e.g. `search` on the length pass, or
// `policy` on the occupancy pass) is the same fatal "unknown setting" as
// if the user wrote it directly. That implicitly scopes each preset to
// the pass whose keys it uses.
//
// The occupancy presets capture today's real configurations and carry
// the search budget each path uses now (flat BFS-DP: 10s, matching
// BfsDpSettings::ForOccupancyPass; decompose: 5s, matching
// BfsDpWithDfsFallback), so wiring (step 3) is a no-behavior-change swap.
// `max-length` is the length-pass baseline: flat DFS, maximize length.
using PresetPairs = std::vector<std::pair<StringRef, StringRef>>;

std::optional<PresetPairs> GetPreset(StringRef name) {
  if (name == "flat-dfs") {
    return PresetPairs{
        {"formation", "none"}, {"search", "dfs"}, {"decompose", "off"}};
  }
  if (name == "flat-bfsdp") {
    return PresetPairs{{"formation", "none"},
                       {"search", "bfsdp"},
                       {"decompose", "off"},
                       {"search.timeout", "10000"}};
  }
  if (name == "decompose-mincut") {
    return PresetPairs{{"decompose", "on"},
                       {"formation", "mincut"},
                       {"search", "bfsdp+dfs"},
                       {"mode", "serialized"},
                       {"search.timeout", "5000"}};
  }
  if (name == "decompose-mincut-interleave") {
    return PresetPairs{{"decompose", "on"},
                       {"formation", "mincut"},
                       {"search", "bfsdp+dfs"},
                       {"mode", "interleaved"},
                       {"search.timeout", "5000"}};
  }
  if (name == "decompose-domtree") {
    return PresetPairs{{"decompose", "on"},
                       {"formation", "domtree"},
                       {"search", "bfsdp+dfs"},
                       {"mode", "serialized"},
                       {"search.timeout", "5000"}};
  }
  if (name == "max-length") {
    return PresetPairs{{"formation", "none"}, {"policy", "max"}};
  }
  return std::nullopt;
}

PresetPairs LoadPreset(StringRef name) {
  std::optional<PresetPairs> preset = GetPreset(name);
  if (!preset) {
    report_fatal_error(Twine("HierarchicalConfig: unknown preset '") + name +
                       "'");
  }
  return *preset;
}

// The preset name for a scope is its `preset` key, if any.
StringRef GetPresetName(const std::map<std::string, std::string> *kv) {
  if (!kv) {
    return StringRef();
  }
  auto it = kv->find("preset");
  if (it == kv->end()) {
    return StringRef();
  }
  return StringRef(it->second);
}

void BuildOccupancy(const std::map<std::string, std::string> *kv,
                    OccupancyConfig &c) {
  StringRef preset_name = GetPresetName(kv);
  if (!preset_name.empty()) {
    for (const auto &p : LoadPreset(preset_name)) {
      ApplyOccupancyKey(p.first, p.second, c);
    }
  }
  if (kv) {
    for (const auto &p : *kv) {
      if (p.first != "preset") {
        ApplyOccupancyKey(p.first, p.second, c);
      }
    }
  }
}

void BuildLength(const std::map<std::string, std::string> *kv, LengthConfig &c) {
  StringRef preset_name = GetPresetName(kv);
  if (!preset_name.empty()) {
    for (const auto &p : LoadPreset(preset_name)) {
      ApplyLengthKey(p.first, p.second, c);
    }
  }
  if (kv) {
    for (const auto &p : *kv) {
      if (p.first != "preset") {
        ApplyLengthKey(p.first, p.second, c);
      }
    }
  }
}

// --- constraint validation (axis constraints, design doc §4.3) ---

void ValidateOccupancy(const OccupancyConfig &c) {
  if (c.decompose && c.formation.strategy == SubgraphFormationStrategy::kNone) {
    report_fatal_error(
        "HierarchicalConfig: occupancy.decompose requires formation != none");
  }
  if (c.formation.mode == SubgraphScheduleMode::kInterleaved &&
      c.formation.strategy == SubgraphFormationStrategy::kNone) {
    report_fatal_error("HierarchicalConfig: occupancy.mode=interleaved "
                       "requires formation != none");
  }
  if (c.fallback_timeout_ms.has_value() && c.search != Search::kBfsDpDfs) {
    report_fatal_error("HierarchicalConfig: occupancy.search.fallback_timeout "
                       "requires search=bfsdp+dfs");
  }
}

void ValidateLength(const LengthConfig &c) {
  if (c.formation.mode == SubgraphScheduleMode::kInterleaved &&
      c.formation.strategy == SubgraphFormationStrategy::kNone) {
    report_fatal_error("HierarchicalConfig: length.mode=interleaved requires "
                       "formation != none");
  }
}

// --- enum -> string (for ToString) ---

StringRef FormationName(SubgraphFormationStrategy f) {
  switch (f) {
  case SubgraphFormationStrategy::kNone:
    return "none";
  case SubgraphFormationStrategy::kDomTree:
    return "domtree";
  case SubgraphFormationStrategy::kMinCut:
    return "mincut";
  }
  return "?";
}

StringRef SearchName(Search s) {
  switch (s) {
  case Search::kDfs:
    return "dfs";
  case Search::kBfsDp:
    return "bfsdp";
  case Search::kBfsDpDfs:
    return "bfsdp+dfs";
  }
  return "?";
}

StringRef ModeName(SubgraphScheduleMode m) {
  switch (m) {
  case SubgraphScheduleMode::kSerialized:
    return "serialized";
  case SubgraphScheduleMode::kInterleaved:
    return "interleaved";
  }
  return "?";
}

StringRef LengthPolicyName(LengthPolicy p) {
  switch (p) {
  case LengthPolicy::kMin:
    return "min";
  case LengthPolicy::kMinRefineIlp:
    return "min+refine-ilp";
  case LengthPolicy::kMinRefineOccupancy:
    return "min+refine-occupancy";
  case LengthPolicy::kMax:
    return "max";
  }
  return "?";
}

} // namespace

HierarchicalConfig
HierarchicalConfig::Build(const MachineInstrSchedulerConfig &cfg) {
  const std::map<std::string, std::map<std::string, std::string>> &scoped =
      cfg.GetAllScopedSettings();

  // Reject unknown scopes up front (only the empty top-level scope,
  // "occupancy", and "length" are known).
  for (const auto &entry : scoped) {
    const std::string &scope = entry.first;
    if (scope.empty() || scope == "occupancy" || scope == "length") {
      continue;
    }
    report_fatal_error(Twine("HierarchicalConfig: unknown scope '") + scope +
                       "'");
  }

  auto find_scope =
      [&](StringRef s) -> const std::map<std::string, std::string> * {
    auto it = scoped.find(s.str());
    return it == scoped.end() ? nullptr : &it->second;
  };

  HierarchicalConfig hs;

  BuildOccupancy(find_scope("occupancy"), hs.occupancy);
  BuildLength(find_scope("length"), hs.length);

  ValidateOccupancy(hs.occupancy);
  ValidateLength(hs.length);

  // Top-level (empty-scope) global toggles.
  if (const std::map<std::string, std::string> *top = find_scope("")) {
    for (const auto &p : *top) {
      StringRef k = p.first;
      StringRef v = p.second;
      if (k == "malicious") {
        hs.malicious = ParseBool("", k, v);
      } else if (k == "run_shakedowns") {
        hs.run_shakedowns = ParseBool("", k, v);
      } else if (k == "dump_subgraph_dag") {
        hs.dump_subgraph_dag = ParseBool("", k, v);
      } else if (k == "dump_search_outcomes") {
        hs.dump_search_outcomes = ParseBool("", k, v);
      } else if (k == "scale_edge_latencies") {
        hs.scale_edge_latencies = ParseBool("", k, v);
      } else {
        UnknownKey("", k);
      }
    }
  }

  // Cross-global invariant: the subgraph-DAG dump targets real region
  // graphs (it derives identity and target info from real instructions),
  // while shakedowns exercise synthetic test graphs that have none.
  // Dumping during shakedowns is meaningless and unsupported, so the two
  // toggles are mutually exclusive.
  if (hs.dump_subgraph_dag && hs.run_shakedowns) {
    report_fatal_error("HierarchicalConfig: dump_subgraph_dag and "
                       "run_shakedowns cannot both be enabled; shakedowns "
                       "operate on synthetic graphs that are not dumpable");
  }

  return hs;
}

const HierarchicalConfig &HierarchicalConfig::Get() {
  static const HierarchicalConfig cfg =
      Build(MachineInstrSchedulerConfig::GetConfig());
  return cfg;
}

std::string HierarchicalConfig::ToString() const {
  std::string out;
  raw_string_ostream os(out);
  os << "HierarchicalConfig:\n";
  os << "\toccupancy: formation=" << FormationName(occupancy.formation.strategy)
     << " search=" << SearchName(occupancy.search)
     << " decompose=" << (occupancy.decompose ? "on" : "off")
     << " mode=" << ModeName(occupancy.formation.mode)
     << " ratio=" << occupancy.formation.min_cut.imbalance_ratio
     << " target_size=" << occupancy.formation.min_cut.target_subgraph_size
     << " timeout_ms=" << occupancy.timeout_ms
     << " fallback_timeout_ms=" << occupancy.GetFallbackTimeoutMs()
     << (occupancy.fallback_timeout_ms.has_value() ? "" : " (=timeout)")
     << "\n";
  os << "\tlength: formation=" << FormationName(length.formation.strategy)
     << " mode=" << ModeName(length.formation.mode)
     << " ratio=" << length.formation.min_cut.imbalance_ratio
     << " target_size=" << length.formation.min_cut.target_subgraph_size
     << " policy=" << LengthPolicyName(length.policy) << "\n";
  os << "\tglobals: malicious=" << (malicious ? "on" : "off")
     << " run_shakedowns=" << (run_shakedowns ? "on" : "off")
     << " dump_subgraph_dag=" << (dump_subgraph_dag ? "on" : "off")
     << " dump_search_outcomes=" << (dump_search_outcomes ? "on" : "off")
     << " scale_edge_latencies=" << (scale_edge_latencies ? "on" : "off")
     << "\n";
  return os.str();
}

void HierarchicalConfig::DebugPrint() const {
  llvm::outs() << "===== HierarchicalConfig (typed, from misched.txt) =====\n";
  llvm::outs() << ToString();
  llvm::outs() << "========================================================\n";
  llvm::outs().flush();
}
