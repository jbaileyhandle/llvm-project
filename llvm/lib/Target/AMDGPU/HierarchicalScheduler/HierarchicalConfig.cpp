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

#include <cmath>
#include <cstdlib>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using namespace llvm;
using namespace llvm::hierarchical_scheduler;

namespace {

// Format a setting's fully-qualified "<scope>.<key>" name (as written in
// misched.txt) into a string for error messages. The generic layer guarantees
// a non-empty scope and key reach here, so there is no empty-scope case.
std::string FormatQualifiedKey(StringRef scope, StringRef key) {
  return scope.str() + "." + key.str();
}

[[noreturn]] void BadValue(StringRef scope, StringRef key, StringRef val,
                           StringRef expected) {
  report_fatal_error(Twine("misched.txt: invalid value '") + val + "' for '" +
                     FormatQualifiedKey(scope, key) + "' (expected " + expected +
                     ")");
}

[[noreturn]] void UnknownKey(StringRef scope, StringRef key) {
  report_fatal_error(Twine("misched.txt: unknown setting '") +
                     FormatQualifiedKey(scope, key) + "'");
}

// --- config-axis enums ---
//
// One {spelling, value} table per axis enum is the single source of truth for
// that axis's vocabulary. ParseEnum (string -> value) and EnumName (value ->
// string, for ToString) both read it, and ParseEnum builds BadValue's
// "expected" list from it -- so the accepted spellings live in exactly one
// place instead of three hand-synced copies (parse chain, name chain, error
// string).

template <typename E> struct EnumSpec {
  StringRef name;
  E value;
};

// Map a spelling to its enum value, or fatal (with an "expected a|b|c" list
// built from the table) if it is not a known spelling.
template <typename E, size_t N>
E ParseEnum(const EnumSpec<E> (&table)[N], StringRef scope, StringRef key,
            StringRef v) {
  for (const EnumSpec<E> &spec : table) {
    if (v == spec.name) {
      return spec.value;
    }
  }
  std::string expected;
  for (const EnumSpec<E> &spec : table) {
    if (!expected.empty()) {
      expected += "|";
    }
    expected += spec.name;
  }
  BadValue(scope, key, v, expected);
}

// Map an enum value back to its spelling. The value always comes from a parsed
// config, so it is always present in the table.
template <typename E, size_t N>
StringRef EnumName(const EnumSpec<E> (&table)[N], E value) {
  for (const EnumSpec<E> &spec : table) {
    if (spec.value == value) {
      return spec.name;
    }
  }
  llvm_unreachable("config enum value missing from its name table");
}

const EnumSpec<SubgraphFormationStrategy> kFormationStrategies[] = {
    {"none", SubgraphFormationStrategy::kNone},
    {"domtree", SubgraphFormationStrategy::kDomTree},
    {"mincut", SubgraphFormationStrategy::kMinCut},
};

const EnumSpec<Search> kSearches[] = {
    {"dfs", Search::kDfs},
    {"bfsdp", Search::kBfsDp},
    {"bfsdp+dfs", Search::kBfsDpDfs},
};

const EnumSpec<OccupancyPolicy> kOccupancyPolicies[] = {
    {"continuous", OccupancyPolicy::kContinuousOccupancy},
    {"integer", OccupancyPolicy::kIntegerOccupancy},
    {"integer+refine-spill-area",
     OccupancyPolicy::kIntegerOccupancyRefineSpillArea},
    {"continuous+refine-spill-area",
     OccupancyPolicy::kContinuousOccupancyRefineSpillArea},
};

const EnumSpec<SubgraphScheduleMode> kScheduleModes[] = {
    {"serialized", SubgraphScheduleMode::kSerialized},
    {"interleaved", SubgraphScheduleMode::kInterleaved},
};

const EnumSpec<LengthPolicy> kLengthPolicies[] = {
    {"min", LengthPolicy::kMin},
    {"min+refine-ilp", LengthPolicy::kMinRefineIlp},
    {"min+refine-occupancy", LengthPolicy::kMinRefineOccupancy},
    {"min+bounded-spill-signals", LengthPolicy::kMinBoundedSpillSignals},
    {"max", LengthPolicy::kMax},
};

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
  double out = 0.0;
  // getAsDouble is locale-independent and rejects empty / trailing junk (unlike
  // strtof); also reject non-finite (nan/inf) so a bad value can't flow into
  // formation.
  if (v.getAsDouble(out) || !std::isfinite(out)) {
    BadValue(scope, key, v, "a finite number");
  }
  return static_cast<float>(out);
}

// --- per-key application ---

// Apply a key shared by both passes (formation + its params + mode) onto
// the referenced fields. Returns true if `key` was one of those keys
// (handled), false otherwise (so the caller can try its pass-specific
// keys, then fatal on a genuine unknown).
bool ApplyFormationKey(StringRef scope, StringRef key, StringRef val,
                       FormationConfig &fc) {
  if (key == "formation") {
    fc.strategy = ParseEnum(kFormationStrategies, scope, key, val);
    return true;
  }
  if (key == "mode") {
    fc.mode = ParseEnum(kScheduleModes, scope, key, val);
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
    c.search = ParseEnum(kSearches, scope, key, val);
    return;
  }
  if (key == "inner_search") {
    c.inner_search = ParseEnum(kSearches, scope, key, val);
    return;
  }
  if (key == "decompose") {
    c.decompose = ParseBool(scope, key, val);
    return;
  }
  if (key == "policy") {
    c.policy = ParseEnum(kOccupancyPolicies, scope, key, val);
    return;
  }
  if (key == "decompose_recursive") {
    c.decompose_recursive = ParseBool(scope, key, val);
    return;
  }
  if (key == "decompose_max_parts") {
    c.decompose_max_parts = ParseInt(scope, key, val);
    return;
  }
  if (key == "max_occ_above_input") {
    c.max_occ_above_input = ParseInt(scope, key, val);
    return;
  }
  if (key == "optimize_every_region_past_occupancy_target") {
    c.optimize_every_region_past_occupancy_target = ParseBool(scope, key, val);
    return;
  }
  // These *.timeout keys are given in microseconds (the search's internal unit),
  // consistent with the misched time_per_instr_*_us settings.
  if (key == "search.timeout") {
    c.timeout_us = ParseInt(scope, key, val);
    return;
  }
  if (key == "search.fallback_timeout") {
    c.fallback_timeout_us = ParseInt(scope, key, val);
    return;
  }
  if (key == "inner_search.timeout") {
    c.inner_timeout_us = ParseInt(scope, key, val);
    return;
  }
  if (key == "inner_search.fallback_timeout") {
    c.inner_fallback_timeout_us = ParseInt(scope, key, val);
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
    c.policy = ParseEnum(kLengthPolicies, scope, key, val);
    return;
  }
  // length.search.timeout is given in microseconds (see ApplyOccupancyKey).
  if (key == "search.timeout") {
    c.timeout_us = ParseInt(scope, key, val);
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
// DecomposeAndScheduleOptions::Make), so wiring (step 3) is a no-behavior-change swap.
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
                       {"search.timeout", "10000000"}}; // 10s in us
  }
  if (name == "decompose-mincut") {
    return PresetPairs{{"decompose", "on"},
                       {"formation", "mincut"},
                       {"search", "bfsdp+dfs"},
                       {"mode", "serialized"},
                       {"search.timeout", "5000000"}}; // 5s in us
  }
  if (name == "decompose-mincut-interleave") {
    return PresetPairs{{"decompose", "on"},
                       {"formation", "mincut"},
                       {"search", "dfs"},
                       {"mode", "interleaved"},
                       {"search.timeout", "5000000"}}; // 5s in us
  }
  if (name == "decompose-domtree") {
    return PresetPairs{{"decompose", "on"},
                       {"formation", "domtree"},
                       {"search", "dfs"},
                       {"mode", "serialized"},
                       {"search.timeout", "5000000"}}; // 5s in us
  }
  if (name == "max-length") {
    return PresetPairs{{"formation", "none"}, {"policy", "max"}};
  }
  return std::nullopt;
}

PresetPairs LoadPreset(StringRef name) {
  std::optional<PresetPairs> preset = GetPreset(name);
  if (!preset) {
    report_fatal_error(Twine("misched.txt: unknownpreset '") + name +
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
        // Track explicit (non-preset) flat-timeout keys: only these conflict
        // with a per-instruction budget (a preset-supplied timeout is a default
        // the per-instruction budget silently overrides).
        if (p.first == "search.timeout") {
          c.timeout_explicitly_set = true;
        }
        if (p.first == "inner_search.timeout") {
          c.inner_timeout_explicitly_set = true;
        }
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
        // Explicit (non-preset) flat timeout conflicts with a per-instruction
        // budget (see BuildOccupancy).
        if (p.first == "search.timeout") {
          c.timeout_explicitly_set = true;
        }
        ApplyLengthKey(p.first, p.second, c);
      }
    }
  }
}

// --- constraint validation (axis constraints, design doc §4.3) ---

// Formation params are shared by both passes, so validate them in one place.
void ValidateFormation(StringRef scope, const FormationConfig &fc) {
  if (fc.mode == SubgraphScheduleMode::kInterleaved &&
      fc.strategy == SubgraphFormationStrategy::kNone) {
    report_fatal_error(Twine("misched.txt: ") + scope +
                       ".mode=interleaved requires formation != none");
  }
  // target_size feeds k = ceil(N / target_size) in min-cut: 0 divides by zero,
  // and a negative size is meaningless.
  if (fc.min_cut.target_subgraph_size < 1) {
    report_fatal_error(Twine("misched.txt: ") + scope +
                       ".formation.target_size must be >= 1");
  }
  // imbalance_ratio is a cap multiplier where 1.0 forces even part sizes; below
  // 1.0 is degenerate (non-finite was already rejected by ParseFloat).
  if (fc.min_cut.imbalance_ratio < 1.0f) {
    report_fatal_error(Twine("misched.txt: ") + scope +
                       ".formation.ratio must be >= 1.0");
  }
}

void ValidateOccupancy(const OccupancyConfig &c) {
  ValidateFormation("occupancy", c.formation);
  // The two occupancy-target knobs pull opposite directions: one removes the
  // target so the pass squeezes maximally on every region, the other caps it.
  if (c.optimize_every_region_past_occupancy_target &&
      c.max_occ_above_input.has_value()) {
    report_fatal_error(
        "misched.txt: occupancy.optimize_every_region_past_occupancy_target "
        "and occupancy.max_occ_above_input are mutually exclusive");
  }
  if (c.decompose && c.formation.strategy == SubgraphFormationStrategy::kNone) {
    report_fatal_error(
        "misched.txt: occupancy.decompose requires formation != none");
  }
  // Recursive decompose is a refinement of mincut decompose; it is meaningless
  // without both.
  if (c.decompose_recursive &&
      (!c.decompose ||
       c.formation.strategy != SubgraphFormationStrategy::kMinCut)) {
    report_fatal_error("misched.txt: occupancy.decompose_recursive requires "
                       "occupancy.decompose and formation=mincut");
  }
  // A per-level part cap below 2 cannot split.
  if (c.decompose_recursive && c.decompose_max_parts < 2) {
    report_fatal_error(
        "misched.txt: occupancy.decompose_max_parts must be >= 2");
  }
  if (c.fallback_timeout_us.has_value() && c.search != Search::kBfsDpDfs) {
    report_fatal_error("misched.txt: occupancy.search.fallback_timeout "
                       "requires search=bfsdp+dfs");
  }
  // Timeouts are wall-clock budgets in us: 0 means "no timeout" (run to
  // completion); a negative budget is meaningless.
  if (c.timeout_us < 0) {
    report_fatal_error("misched.txt: occupancy.search.timeout must be "
                       ">= 0 (0 = no timeout)");
  }
  if (c.fallback_timeout_us.has_value() && *c.fallback_timeout_us < 0) {
    report_fatal_error("misched.txt: occupancy.search.fallback_timeout "
                       "must be >= 0 (0 = no timeout)");
  }
  if (c.max_occ_above_input.has_value() && *c.max_occ_above_input < 0) {
    report_fatal_error(
        "misched.txt: occupancy.max_occ_above_input must be >= 0");
  }
  // BFS-DP isn't equipped for spill-area; the refine-spill-area policies are
  // DFS-only.
  if ((c.policy == OccupancyPolicy::kIntegerOccupancyRefineSpillArea ||
       c.policy == OccupancyPolicy::kContinuousOccupancyRefineSpillArea) &&
      c.search != Search::kDfs) {
    report_fatal_error(
        "misched.txt: occupancy.policy refine-spill-area variants "
        "require occupancy.search=dfs (BFS-DP is not equipped for spill area)");
  }
  // BFS-DP maximizes regardless of the function occupancy target, so the cap
  // is only honored by DFS (whose ShouldEndSearch reads that target).
  if (c.max_occ_above_input.has_value() && c.search != Search::kDfs) {
    report_fatal_error("misched.txt: occupancy.max_occ_above_input "
                       "requires occupancy.search=dfs (BFS-DP ignores the cap)");
  }
  // Decompose's outer search has no "BFS-DP only" branch — it always pairs
  // BFS-DP with a DFS fallback. search=bfsdp (no-fallback) only makes sense
  // on the flat path; rejecting it under decompose=on avoids the silent
  // promotion to bfsdp+dfs.
  if (c.decompose && c.search == Search::kBfsDp) {
    report_fatal_error(
        "misched.txt: occupancy.decompose=on with occupancy.search=bfsdp "
        "is not supported (decompose's outer always pairs BFS-DP with a DFS "
        "fallback; use search=bfsdp+dfs or search=dfs)");
  }
}

void ValidateLength(const LengthConfig &c) {
  ValidateFormation("length", c.formation);
}

} // namespace

HierarchicalConfig
HierarchicalConfig::Build(const MachineInstrSchedulerConfig &cfg) {
  const std::map<std::string, std::map<std::string, std::string>> &scoped =
      cfg.GetAllScopedSettings();

  // Reject unknown scopes up front. Only "occupancy" and "length" are known;
  // the generic config only routes dotted "<scope>.<key>" settings into the
  // scoped store (the global toggles are bare flags now).
  for (const auto &entry : scoped) {
    const std::string &scope = entry.first;
    if (scope == "occupancy" || scope == "length") {
      continue;
    }
    report_fatal_error(Twine("misched.txt: unknownscope '") + scope +
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

  // Global toggles are bare flags on the generic config (their spelling ->
  // field mapping and validation live there); mirror them into typed fields.
  const MachineInstrSchedulerConfig::Flags &flags = cfg.GetFlags();
  hs.malicious = flags.malicious;
  hs.run_shakedowns = flags.run_shakedowns;
  hs.dump_subgraph_dag = flags.dump_subgraph_dag;
  hs.dump_search_outcomes = flags.dump_search_outcomes;
  hs.scale_edge_latencies = flags.scale_edge_latencies;
  hs.skip_occupancy_pass = flags.skip_occupancy_pass;
  hs.skip_length_pass = flags.skip_length_pass;
  hs.length_ignore_occupancy = flags.length_ignore_occupancy;
  hs.min_adjusted_length = flags.min_adjusted_length;
  // Ignoring the occupancy target in the length pass makes the maximize-occupancy
  // pass pointless, so it implies skipping it.
  if (hs.length_ignore_occupancy) {
    hs.skip_occupancy_pass = true;
  }
  // The min-adjusted-length pass sweeps tiers from the post-occupancy-pass
  // kernel ceiling down to the launch floor, seeding every tier's search
  // with the occupancy pass's output (feasible at every tier at or below
  // the ceiling). Without the occupancy pass there is no trustworthy top
  // tier and no feasible seed, so the combination is rejected rather than
  // silently sweeping from an unproven ceiling. length_ignore_occupancy
  // implies skip_occupancy_pass, so it is rejected transitively.
  if (hs.min_adjusted_length && hs.skip_occupancy_pass) {
    report_fatal_error("misched.txt: min_adjusted_length requires the "
                       "occupancy pass (remove skip_occupancy_pass / "
                       "length_ignore_occupancy)");
  }

  // Per-instruction scheduling-time budgets are unscoped global settings;
  // mirror them into the pass they drive (occupancy / length).
  const MachineInstrSchedulerConfig::GlobalSettings &global_settings =
      cfg.GetGlobalSettings();
  hs.occupancy.time_per_instr_us = global_settings.time_per_instr_occupancy_us;
  hs.length.time_per_instr_us = global_settings.time_per_instr_length_us;

  // The per-instruction occupancy budget only supports plain and single-level
  // decompose, all-DFS, and cannot coexist with an explicit flat timeout.
  if (hs.occupancy.time_per_instr_us.has_value()) {
    if (hs.occupancy.search != Search::kDfs) {
      report_fatal_error("misched.txt: time_per_instr_occupancy_us requires "
                         "occupancy.search=dfs");
    }
    if (hs.occupancy.decompose &&
        hs.occupancy.inner_search != Search::kDfs) {
      report_fatal_error("misched.txt: time_per_instr_occupancy_us requires "
                         "occupancy.inner_search=dfs");
    }
    if (hs.occupancy.decompose_recursive) {
      report_fatal_error("misched.txt: time_per_instr_occupancy_us is not "
                         "supported with occupancy.decompose_recursive");
    }
    if (hs.occupancy.timeout_explicitly_set) {
      report_fatal_error("misched.txt: set either time_per_instr_occupancy_us "
                         "or occupancy.search.timeout, not both");
    }
    if (hs.occupancy.inner_timeout_explicitly_set) {
      report_fatal_error("misched.txt: set either time_per_instr_occupancy_us "
                         "or occupancy.inner_search.timeout, not both");
    }
  }
  if (hs.length.time_per_instr_us.has_value() &&
      hs.length.timeout_explicitly_set) {
    report_fatal_error("misched.txt: set either time_per_instr_length_us or "
                       "length.search.timeout, not both");
  }

  // Cross-global invariant: the subgraph-DAG dump targets real region
  // graphs (it derives identity and target info from real instructions),
  // while shakedowns exercise synthetic test graphs that have none.
  // Dumping during shakedowns is meaningless and unsupported, so the two
  // toggles are mutually exclusive.
  if (hs.dump_subgraph_dag && hs.run_shakedowns) {
    report_fatal_error("misched.txt: dump_subgraph_dag and "
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
  os << "\toccupancy: formation="
     << EnumName(kFormationStrategies, occupancy.formation.strategy)
     << " search=" << EnumName(kSearches, occupancy.search)
     << " inner_search=" << EnumName(kSearches, occupancy.inner_search)
     << " policy=" << EnumName(kOccupancyPolicies, occupancy.policy)
     << " decompose=" << (occupancy.decompose ? "on" : "off")
     << " decompose_recursive="
     << (occupancy.decompose_recursive ? "on" : "off")
     << " decompose_max_parts=" << occupancy.decompose_max_parts
     << " max_occ_above_input="
     << (occupancy.max_occ_above_input.has_value()
             ? std::to_string(*occupancy.max_occ_above_input)
             : "off")
     << " optimize_every_region_past_occupancy_target="
     << (occupancy.optimize_every_region_past_occupancy_target ? "on" : "off")
     << " mode=" << EnumName(kScheduleModes, occupancy.formation.mode)
     << " ratio=" << occupancy.formation.min_cut.imbalance_ratio
     << " target_size=" << occupancy.formation.min_cut.target_subgraph_size
     << " timeout_us=" << occupancy.timeout_us
     << " fallback_timeout_us=" << occupancy.GetFallbackTimeoutUs()
     << (occupancy.fallback_timeout_us.has_value() ? "" : " (=timeout)")
     << " inner_timeout_us=" << occupancy.inner_timeout_us
     << " inner_fallback_timeout_us=" << occupancy.GetInnerFallbackTimeoutUs()
     << (occupancy.inner_fallback_timeout_us.has_value() ? "" : " (=inner_timeout)")
     << " time_per_instr_us="
     << (occupancy.time_per_instr_us.has_value()
             ? std::to_string(*occupancy.time_per_instr_us)
             : "off")
     << "\n";
  os << "\tlength: formation="
     << EnumName(kFormationStrategies, length.formation.strategy)
     << " mode=" << EnumName(kScheduleModes, length.formation.mode)
     << " ratio=" << length.formation.min_cut.imbalance_ratio
     << " target_size=" << length.formation.min_cut.target_subgraph_size
     << " policy=" << EnumName(kLengthPolicies, length.policy)
     << " timeout_us=" << length.timeout_us << " time_per_instr_us="
     << (length.time_per_instr_us.has_value()
             ? std::to_string(*length.time_per_instr_us)
             : "off")
     << "\n";
  os << "\tglobals: malicious=" << (malicious ? "on" : "off")
     << " run_shakedowns=" << (run_shakedowns ? "on" : "off")
     << " dump_subgraph_dag=" << (dump_subgraph_dag ? "on" : "off")
     << " dump_search_outcomes=" << (dump_search_outcomes ? "on" : "off")
     << " scale_edge_latencies=" << (scale_edge_latencies ? "on" : "off")
     << " skip_occupancy_pass=" << (skip_occupancy_pass ? "on" : "off")
     << " skip_length_pass=" << (skip_length_pass ? "on" : "off")
     << " length_ignore_occupancy=" << (length_ignore_occupancy ? "on" : "off")
     << " min_adjusted_length=" << (min_adjusted_length ? "on" : "off")
     << "\n";
  return os.str();
}

void HierarchicalConfig::DebugPrint() const {
  llvm::outs() << "===== HierarchicalConfig (typed, from misched.txt) =====\n";
  llvm::outs() << ToString();
  llvm::outs() << "========================================================\n";
  llvm::outs().flush();
}
