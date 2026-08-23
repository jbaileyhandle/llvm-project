#include "llvm/Analysis/MachineInstrSchedulerConfig.h"

#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/ADT/Twine.h"
#include "llvm/Demangle/Demangle.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/ErrorHandling.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <sstream>

using namespace llvm;

namespace {
    const std::string waves_per_eu_attr = "amdgpu-waves-per-eu";

    // A flag binding pairs a misched.txt spelling with the Flags field it
    // controls. The field is held as a pointer-to-data-member
    // (`bool Flags::*`): it names *which* bool field of Flags, independent of
    // any object. Bind it to an instance with the `.*` operator --
    // `flags.*binding.field` -- to read or write that field. One binding
    // therefore serves both the parser (write) and ToString (read), so the
    // list of valid flags lives in exactly one place (kFlagBindings).
    using Flags = MachineInstrSchedulerConfig::Flags;
    struct FlagBinding {
        StringRef name;
        bool Flags::*field;
    };
    const FlagBinding kFlagBindings[] = {
        {"disable_post_ra_scheduling", &Flags::disable_post_ra_scheduling},
        {"enable_runtime_unroll", &Flags::enable_runtime_unroll},
        {"disable_licm", &Flags::disable_licm},
        {"disable_mem_clustering", &Flags::disable_mem_clustering},
        {"disable_max_occ_effective_max_waves_cap",
         &Flags::disable_max_occ_effective_max_waves_cap},
        {"run_on_all_functions", &Flags::run_on_all_functions},
        {"run_regardless_of_heuristic_outcome", &Flags::run_regardless_of_heuristic_outcome},
        {"use_continuous_occupancy_score", &Flags::use_continuous_occupancy_score},
        {"malicious", &Flags::malicious},
        {"run_shakedowns", &Flags::run_shakedowns},
        {"dump_subgraph_dag", &Flags::dump_subgraph_dag},
        {"dump_search_outcomes", &Flags::dump_search_outcomes},
        {"scale_edge_latencies", &Flags::scale_edge_latencies},
        {"skip_occupancy_pass", &Flags::skip_occupancy_pass},
        {"skip_length_pass", &Flags::skip_length_pass},
        {"length_ignore_occupancy", &Flags::length_ignore_occupancy},
        {"min_adjusted_length", &Flags::min_adjusted_length},
    };

    // Return the binding for flag `name`, or nullptr if `name` is not a known
    // flag spelling.
    const FlagBinding *FindFlag(StringRef name) {
        for (const auto &binding : kFlagBindings) {
            if (name == binding.name) {
                return &binding;
            }
        }
        return nullptr;
    }

    // The same single-source-of-truth pattern for unscoped global integer
    // settings: each binding pairs a misched.txt spelling with the GlobalSettings
    // field it controls, held as a pointer-to-data-member
    // (`std::optional<int> GlobalSettings::*`). One binding serves both the
    // parser (write) and ToString (read), so the list of valid global settings
    // lives in exactly one place (kGlobalIntSettingBindings).
    using GlobalSettings = MachineInstrSchedulerConfig::GlobalSettings;
    struct GlobalIntSettingBinding {
        StringRef name;
        std::optional<int> GlobalSettings::*field;
    };
    const GlobalIntSettingBinding kGlobalIntSettingBindings[] = {
        {"unroll_threshold", &GlobalSettings::unroll_threshold},
        {"partial_unroll_threshold", &GlobalSettings::partial_unroll_threshold},
        {"runtime_unroll_factor", &GlobalSettings::runtime_unroll_factor},
        {"time_per_instr_occupancy_us", &GlobalSettings::time_per_instr_occupancy_us},
        {"time_per_instr_length_us", &GlobalSettings::time_per_instr_length_us},
        {"vmem_load_latency", &GlobalSettings::vmem_load_latency},
        {"smem_load_latency", &GlobalSettings::smem_load_latency},
        {"lds_load_latency", &GlobalSettings::lds_load_latency},
    };

    // Return the binding for global setting `name`, or nullptr if `name` is not
    // a known global-setting spelling.
    const GlobalIntSettingBinding *FindGlobalIntSetting(StringRef name) {
        for (const auto &binding : kGlobalIntSettingBindings) {
            if (name == binding.name) {
                return &binding;
            }
        }
        return nullptr;
    }

    // The same single-source-of-truth pattern for unscoped string-valued global
    // settings (e.g. `timing_model=fast_memory`), held as a pointer-to-data-
    // member (`std::optional<std::string> GlobalSettings::*`). One binding serves
    // both the parser (write) and ToString (read).
    struct GlobalStringSettingBinding {
        StringRef name;
        std::optional<std::string> GlobalSettings::*field;
    };
    const GlobalStringSettingBinding kGlobalStringSettingBindings[] = {
        {"timing_model", &GlobalSettings::timing_model},
    };

    // Return the binding for string setting `name`, or nullptr if `name` is not
    // a known string-setting spelling.
    const GlobalStringSettingBinding *FindGlobalStringSetting(StringRef name) {
        for (const auto &binding : kGlobalStringSettingBindings) {
            if (name == binding.name) {
                return &binding;
            }
        }
        return nullptr;
    }

    // Parse a `<min>,<max>` waves pair -- the grammar shared by a per-kernel
    // `kernel .../<min>,<max>` line's pair form and the global
    // `all_kernels_occupancy = <min>,<max>` setting. Each entry is >= 0; the `0`
    // sentinel preserves that bound (left unset in the output). Fatal error on a
    // non-pair, a negative/non-integer entry, or min > max. `context` names the
    // caller in error messages (e.g. the offending line).
    void ParseWavesPair(StringRef spec, const Twine &context,
                        std::optional<int> &min_waves,
                        std::optional<int> &max_waves) {
        SmallVector<StringRef, 2> wave_fields;
        spec.split(wave_fields, ',');
        int parsed_min = 0;
        int parsed_max = 0;
        if (wave_fields.size() != 2 ||
            wave_fields[0].getAsInteger(10, parsed_min) || parsed_min < 0 ||
            wave_fields[1].getAsInteger(10, parsed_max) || parsed_max < 0) {
            report_fatal_error(context + ": invalid waves pair '" + spec +
                               "' (expected `<min>,<max>` with each >= 0; 0 "
                               "preserves that bound)");
        }
        // 0 = preserve => leave that bound unset.
        if (parsed_min != 0) {
            min_waves = parsed_min;
        }
        if (parsed_max != 0) {
            max_waves = parsed_max;
        }
        if (min_waves.has_value() && max_waves.has_value() &&
            *min_waves > *max_waves) {
            report_fatal_error(context + ": min waves (" + Twine(*min_waves) +
                               ") > max waves (" + Twine(*max_waves) + ")");
        }
    }

    // Split a string according to whitespace
    // Return split strings as a vector
    std::vector<std::string> SplitByWhitespace(const std::string &input) {
        std::istringstream stream(input);
        std::vector<std::string> tokens{
            std::istream_iterator<std::string>{stream},
            std::istream_iterator<std::string>{}
        };
        return tokens;
    }

} // end namespace

std::string MachineInstrSchedulerConfig::DemangleFunctionSignature(const std::string &mangled_signature) {
    char *demangled = llvm::itaniumDemangle(mangled_signature);
    if (!demangled) {
        // Names not using Itanium mangling (e.g. C symbols, runtime helpers
        // like __cxa_pure_virtual) are returned unchanged.
        return mangled_signature;
    }
    std::string result(demangled);
    std::free(demangled);
    return result;
}

const MachineInstrSchedulerConfig &MachineInstrSchedulerConfig::GetConfig() {
    static MachineInstrSchedulerConfig mi_config;
    return mi_config;
}

bool MachineInstrSchedulerConfig::HasConfig() const {
    return has_config_;
}

bool MachineInstrSchedulerConfig::IsAcoOptSched() const {
    return mi_scheduler_ == Scheduler::AcoOptSched;
}

bool MachineInstrSchedulerConfig::IsBnbOptSched() const {
    return mi_scheduler_ == Scheduler::BnbOptSched;
}

bool MachineInstrSchedulerConfig::IsOptSched() const {
    return IsAcoOptSched() || IsBnbOptSched();
}

bool MachineInstrSchedulerConfig::IsHierarchicalScheduler() const {
    return mi_scheduler_ == Scheduler::HierarchicalScheduler;
}

MachineInstrSchedulerConfig::Scheduler MachineInstrSchedulerConfig::GetScheduler() const {
    return mi_scheduler_;
}

MachineInstrSchedulerConfig::Scheduler
MachineInstrSchedulerConfig::GetSchedulerFromName(llvm::StringRef name) {
    return StringSwitch<Scheduler>(name)
        .Case("MaxOccupancy", Scheduler::MaxOccupancy)
        .Case("MaxIlp", Scheduler::MaxIlp)
        .Case("IterativeMaxOccupancy", Scheduler::IterativeMaxOccupancy)
        .Case("IterativeMaxIlp", Scheduler::IterativeMaxIlp)
        .Case("AcoOptSched", Scheduler::AcoOptSched)
        .Case("BnbOptSched", Scheduler::BnbOptSched)
        .Case("HierarchicalScheduler", Scheduler::HierarchicalScheduler)
        .Default(Scheduler::InvalidOption);
}

bool MachineInstrSchedulerConfig::SetFlagIfKnown(llvm::StringRef name) {
    if (const FlagBinding *binding = FindFlag(name)) {
        flags_.*(binding->field) = true;
        return true;
    }
    return false;
}

bool MachineInstrSchedulerConfig::SetGlobalIntSettingIfKnown(llvm::StringRef key,
                                                          llvm::StringRef value) {
    const GlobalIntSettingBinding *binding = FindGlobalIntSetting(key);
    if (!binding) {
        return false;
    }
    // Every global setting is a non-negative integer (a threshold or a count).
    int parsed = 0;
    if (value.getAsInteger(10, parsed) || parsed < 0) {
        report_fatal_error(Twine("misched.txt: setting '") + key +
                           "' has invalid value '" + value +
                           "' (expected an integer >= 0)");
    }
    global_settings_.*(binding->field) = parsed;
    return true;
}

bool MachineInstrSchedulerConfig::SetGlobalStringSettingIfKnown(llvm::StringRef key,
                                                          llvm::StringRef value) {
    const GlobalStringSettingBinding *binding = FindGlobalStringSetting(key);
    if (!binding) {
        return false;
    }
    // Stored uninterpreted; the consumer validates it (e.g. AMDGPU rejects an
    // unknown timing_model name when it swaps the sched model).
    global_settings_.*(binding->field) = value.str();
    return true;
}

bool MachineInstrSchedulerConfig::TrySetSchedulerFromToken(llvm::StringRef tok) {
    Scheduler scheduler = GetSchedulerFromName(tok);
    if (scheduler == Scheduler::InvalidOption) {
        return false;
    }
    if (scheduler_set_ && scheduler != mi_scheduler_) {
        report_fatal_error(Twine("misched.txt: conflicting scheduler '") + tok +
                           "' (already set to '" + GetSchedulerAsString() + "')");
    }
    mi_scheduler_ = scheduler;
    scheduler_set_ = true;
    return true;
}

void MachineInstrSchedulerConfig::ApplySetting(llvm::StringRef key,
                                               llvm::StringRef value) {
    size_t dot = key.find('.');
    if (dot == StringRef::npos) {
        // all_kernels_occupancy = <min>,<max>: a global occupancy default for
        // every function, sharing the per-kernel `<min>,<max>` grammar. It is a
        // pair, so it is parsed here rather than through the scalar int table;
        // stored split into the two GlobalSettings bounds.
        if (key == "all_kernels_occupancy") {
            ParseWavesPair(value, "misched.txt: all_kernels_occupancy",
                           global_settings_.all_kernels_min_waves,
                           global_settings_.all_kernels_max_waves);
            return;
        }
        // Unscoped global setting -> its typed field (integer, then string).
        if (SetGlobalIntSettingIfKnown(key, value)) {
            return;
        }
        if (SetGlobalStringSettingIfKnown(key, value)) {
            return;
        }
        // A known flag written in "key=value" form: point at the bare-flag
        // spelling rather than calling the name unknown.
        if (FindFlag(key)) {
            report_fatal_error(Twine("misched.txt: '") + key +
                               "' is a flag, not a setting; write it bare "
                               "(drop '=" + value + "')");
        }
        report_fatal_error(Twine("misched.txt: unknown setting '") + key + "'");
    }
    // Scoped "<scope>.<subkey>=<value>". Both halves must be non-empty, and a
    // given "<scope>.<subkey>" may be set only once -- the order-independent
    // grammar makes a silent last-wins overwrite a real ambiguity.
    StringRef scope = key.substr(0, dot);
    StringRef subkey = key.substr(dot + 1);
    if (scope.empty() || subkey.empty()) {
        report_fatal_error(Twine("misched.txt: malformed scoped setting '") + key +
                           "' (empty scope or key)");
    }
    std::map<std::string, std::string> &scope_settings = scoped_[scope.str()];
    if (scope_settings.count(subkey.str())) {
        report_fatal_error(Twine("misched.txt: duplicate setting '") + key + "'");
    }
    // Stored uninterpreted here; HierarchicalConfig validates the scope/key and
    // maps it to a typed field.
    scope_settings[subkey.str()] = value.str();
}

void MachineInstrSchedulerConfig::ParseOptionToken(const std::string &token) {
    size_t eq = token.find('=');
    if (eq == std::string::npos) {
        // A bare token is a scheduler name or a boolean flag.
        StringRef tok(token);
        if (TrySetSchedulerFromToken(tok)) {
            return;
        }
        if (SetFlagIfKnown(tok)) {
            return;
        }
        report_fatal_error(Twine("misched.txt: unknown option '") + tok + "'");
    }
    // A token with '=' is a "<key>=<value>" setting; the key must be non-empty.
    if (eq == 0) {
        report_fatal_error(Twine("misched.txt: setting has an empty key: '") +
                           token + "'");
    }
    ApplySetting(StringRef(token).substr(0, eq), StringRef(token).substr(eq + 1));
}

void MachineInstrSchedulerConfig::ParseKernelLine(const std::string &rest) {
    // Grammar: <m|d>/<signature>/[waves]. Split keeping empty fields so a
    // missing field is rejected, not silently shifted into another position
    // (e.g. `d//4` must fail, not read "4" as the signature).
    SmallVector<StringRef, 4> fields;
    StringRef(rest).split(fields, '/');
    if (fields.size() < 2 || fields.size() > 3) {
        report_fatal_error(Twine("misched.txt: malformed kernel line '") + rest +
                           "' (expected `kernel <m|d>/<signature>/[waves]`)");
    }

    StringRef tag = fields[0];
    StringRef signature = fields[1];
    if (signature.empty()) {
        report_fatal_error(Twine("misched.txt: kernel line '") + rest +
                           "' has an empty signature");
    }

    std::string demangled_func_signature;
    if (tag == "d" || tag == "D") {
        demangled_func_signature = signature.str();
    } else if (tag == "m" || tag == "M") {
        demangled_func_signature = DemangleFunctionSignature(signature.str());
    } else {
        report_fatal_error(Twine("misched.txt: kernel line '") + rest +
                           "' must start with 'm' (mangled) or 'd' (demangled), got '" +
                           tag + "'");
    }

    // The optional 3rd field is a per-function occupancy override, in one of two
    // mutually exclusive forms:
    //   <T>            single value: an occupancy limit consumed only by OptSched
    //                  (must be >= 1). Non-OptSched schedulers reject it.
    //   <min>,<max>    a bound pair for the amdgpu-waves-per-eu attribute; each
    //                  entry is a pure replacement of that bound, and 0 means
    //                  "preserve the function's existing bound".
    // Absent => just register the function (no override), which still opts it into
    // OptSched, since OptSched's per-function opt-in is "has a config entry".
    std::optional<int> optsched_occupancy_limit;
    std::optional<int> min_waves;
    std::optional<int> max_waves;
    if (fields.size() == 3) {
        SmallVector<StringRef, 2> wave_fields;
        fields[2].split(wave_fields, ',');
        if (wave_fields.size() == 1) {
            int parsed = 0;
            if (wave_fields[0].getAsInteger(10, parsed) || parsed < 1) {
                report_fatal_error(Twine("misched.txt: kernel line '") + rest +
                                   "' has invalid waves '" + fields[2] +
                                   "' (expected an integer >= 1)");
            }
            optsched_occupancy_limit = parsed;
        } else if (wave_fields.size() == 2) {
            ParseWavesPair(fields[2],
                           Twine("misched.txt: kernel line '") + rest + "'",
                           min_waves, max_waves);
        } else {
            report_fatal_error(Twine("misched.txt: kernel line '") + rest +
                               "' has malformed waves field '" + fields[2] +
                               "' (expected `<waves>` or `<min>,<max>`)");
        }
    }

    if (demangled_func_signature_to_config_.count(demangled_func_signature)) {
        report_fatal_error(Twine("misched.txt: duplicate kernel configuration for '") +
                           demangled_func_signature + "'");
    }
    demangled_func_signature_to_config_.emplace(
        demangled_func_signature,
        FunctionConfig(demangled_func_signature, optsched_occupancy_limit,
                       min_waves, max_waves));
}

MachineInstrSchedulerConfig::MachineInstrSchedulerConfig() {
    // misched.txt is a line-oriented, order-independent config. Each non-blank,
    // non-comment line is either a `kernel <m|d>/<sig>/[waves]` per-function
    // line or a whitespace-separated list of option tokens (a scheduler name,
    // a bare flag, or a `<key>=<value>` setting). Options may be split across
    // any number of lines; exactly one scheduler name must appear.
    //
    // Config location: if the MISCHED_CONFIG_FILE environment variable is set,
    // it must name a readable config file -- anything else (missing file, empty
    // value) is a fatal error, so an intended config can never silently fail to
    // apply. If unset, fall back to misched.txt in the compiler's CWD. The env
    // var exists because the CWD lookup breaks under out-of-source builds:
    // cmake runs the compiler from build/, not from the directory where the
    // driver script wrote misched.txt. Environment propagates through the whole
    // cmake/make/hipcc subprocess chain; CWD does not.
    const char *env_config_path = ::getenv("MISCHED_CONFIG_FILE");
    if (env_config_path != nullptr) {
        std::ifstream probe(env_config_path);
        if (!probe) {
            report_fatal_error(Twine("MISCHED_CONFIG_FILE is set but not "
                                     "readable: '") + env_config_path + "'");
        }
    }
    std::ifstream misched_config_file(
        env_config_path != nullptr ? env_config_path : "misched.txt");
    if (misched_config_file) {
        has_config_ = true;

        std::string line;
        while (std::getline(misched_config_file, line)) {
            StringRef trimmed = StringRef(line).trim();
            if (trimmed.empty() || trimmed.front() == '#') {
                continue;
            }

            size_t ws = trimmed.find_first_of(" \t");
            StringRef first_word = trimmed.substr(0, ws);
            if (first_word == "kernel") {
                // Signatures contain spaces, so a kernel line is not
                // whitespace-tokenized: the text after the keyword is parsed
                // as `<m|d>/<signature>/[waves]`.
                StringRef rest = (ws == StringRef::npos) ? StringRef() : trimmed.substr(ws).trim();
                ParseKernelLine(rest.str());
                continue;
            }

            for (const std::string &token : SplitByWhitespace(trimmed.str())) {
                ParseOptionToken(token);
            }
        }

        if (!scheduler_set_) {
            report_fatal_error("misched.txt: no scheduler specified");
        }

        // all_kernels_occupancy (a blanket occupancy default) is mutually
        // exclusive with any per-kernel waves override: mixing a global default
        // with a per-kernel `<min>,<max>` (or the single-value OptSched limit) is
        // ambiguous. Checked after the full parse so it holds regardless of line
        // order. A bare `kernel <m|d>/<sig>` with no waves field is fine -- it
        // only registers the function and carries no override.
        if (global_settings_.all_kernels_min_waves.has_value() ||
            global_settings_.all_kernels_max_waves.has_value()) {
            for (const auto &entry : demangled_func_signature_to_config_) {
                const FunctionConfig &function_config = entry.second;
                if (function_config.min_waves_per_eu_.has_value() ||
                    function_config.max_waves_per_eu_.has_value() ||
                    function_config.optsched_occupancy_limit_.has_value()) {
                    report_fatal_error(
                        Twine("misched.txt: all_kernels_occupancy cannot be "
                              "combined with a per-kernel waves setting for '") +
                        function_config.func_signature_ + "'");
                }
            }
        }

        DebugPrint();
    }
}

const MachineInstrSchedulerConfig::FunctionConfig *MachineInstrSchedulerConfig::GetFunctionConfigFromDemangledFunctionSignature(const std::string &demangled_signature) const {
    auto name_config_itr = demangled_func_signature_to_config_.find(demangled_signature);
    if(name_config_itr == demangled_func_signature_to_config_.end()) {
        return nullptr;
    }
    return &(name_config_itr->second);
}

const MachineInstrSchedulerConfig::FunctionConfig *MachineInstrSchedulerConfig::GetFunctionConfigFromMangledFunctionSignature(const std::string &mangled_signature) const {
    std::string demangled_name = DemangleFunctionSignature(mangled_signature);
    return GetFunctionConfigFromDemangledFunctionSignature(demangled_name);
}

const MachineInstrSchedulerConfig::FunctionConfig *MachineInstrSchedulerConfig::GetFunctionConfigFromMangledFunctionSignature(const llvm::StringRef &mangled_signature) const {
    return GetFunctionConfigFromMangledFunctionSignature(mangled_signature.str());
}

bool MachineInstrSchedulerConfig::HasFunctionConfigForDemangledFunctionSignature(const std::string &demangled_signature) const {
    auto name_config_itr = demangled_func_signature_to_config_.find(demangled_signature);
    return name_config_itr != demangled_func_signature_to_config_.end();
}

bool MachineInstrSchedulerConfig::HasFunctionConfigForMangledFunctionSignature(const std::string &mangled_signature) const {
    std::string demangled_name = DemangleFunctionSignature(mangled_signature);
    return HasFunctionConfigForDemangledFunctionSignature(demangled_name);
}

bool MachineInstrSchedulerConfig::HasFunctionConfigForMangledFunctionSignature(const llvm::StringRef &mangled_signature) const {
    return HasFunctionConfigForMangledFunctionSignature(mangled_signature.str());
}

bool MachineInstrSchedulerConfig::HasFunctionConfig(const Function &function) const {
    return HasFunctionConfigForMangledFunctionSignature(function.getName().str());
}

const MachineInstrSchedulerConfig::FunctionConfig *MachineInstrSchedulerConfig::GetFunctionConfig(const Function &function) const {
    return GetFunctionConfigFromMangledFunctionSignature(function.getName().str());
}

void MachineInstrSchedulerConfig::SetFunctionWavesPerEUAttributeBasedOnConfig(Function &function) const {
    // OptSched configurations use a different mechanism to control register pressure / occupancy
    if(IsOptSched()) {
        return;
    }
    // Resolve the effective occupancy bounds for this function: its per-kernel
    // `kernel .../<min>,<max>` entry if it has one, otherwise the global
    // all_kernels_occupancy default that applies to every function. The two are
    // mutually exclusive by construction (rejected at parse time), so this is a
    // plain either/or -- never a merge.
    std::optional<int> min_override;
    std::optional<int> max_override;
    if (HasFunctionConfig(function)) {
        const FunctionConfig *config = GetFunctionConfig(function);

        // The configured scheduler is NOT OptSched (we returned above otherwise),
        // so a single-value occupancy limit is a misuse: it is an OptSched-only
        // input. Non-OptSched schedulers steer occupancy through
        // amdgpu-waves-per-eu, which needs the explicit `<min>,<max>` pair form.
        if(config->optsched_occupancy_limit_.has_value()) {
            report_fatal_error(Twine("misched.txt: single-value waves for '") +
                               function.getName() +
                               "' is OptSched-only; use `<min>,<max>` instead");
        }

        min_override = config->min_waves_per_eu_;
        max_override = config->max_waves_per_eu_;
    } else {
        // No per-kernel entry: fall back to the global default (each nullopt if
        // all_kernels_occupancy was not set).
        min_override = global_settings_.all_kernels_min_waves;
        max_override = global_settings_.all_kernels_max_waves;
    }

    // The `<min>,<max>` pair splits into two very different things.
    //
    // The MAX is a "don't optimize occupancy past this" ceiling. It is NEVER
    // written to the amdgpu-waves-per-eu attribute: an attribute max pads reserved
    // registers and caps the waves the runtime launches regardless of real
    // register use, which is exactly what we want to avoid. Instead each scheduler
    // consumes the max its own way, so only schedulers with a place to consume it
    // support it; the rest must reject rather than silently ignore:
    //   - MaxOccupancy: applied as the occupancy target at scheduler start
    //     (createGCNMaxOccupancyMachineScheduler calls MFI->limitOccupancy(max)).
    //   - HierarchicalScheduler: consumed internally (GetPerKernelOccupancyTarget).
    //   - MaxIlp: ignored -- it does not hold an occupancy ceiling anyway.
    if(max_override.has_value()) {
        Scheduler scheduler = GetScheduler();
        bool max_supported = scheduler == Scheduler::MaxOccupancy ||
                             scheduler == Scheduler::HierarchicalScheduler ||
                             scheduler == Scheduler::MaxIlp;
        if(!max_supported) {
            report_fatal_error(Twine("misched.txt: `<min>,<max>` max-waves for '") +
                               function.getName() +
                               "' is only supported with MaxOccupancy, "
                               "HierarchicalScheduler, or MaxIlp");
        }
    }

    // The MIN is a genuine occupancy / register-allocator floor and is written to
    // amdgpu-waves-per-eu for any scheduler (positional: a lone value is the min,
    // so the max field stays at the hardware default -- never a cap, no padding).
    // An unset min (the `0` sentinel) leaves the function's waves-per-eu alone.
    // getWavesPerEU is patched (honor_misched_min) to keep a misched min even when
    // it falls below the flat-work-group-size floor, instead of discarding it back
    // to the default range.
    if(!min_override.has_value()) {
        return;
    }
    function.removeFnAttr(waves_per_eu_attr);
    function.addFnAttr(waves_per_eu_attr, std::to_string(*min_override));
}

std::optional<int>
MachineInstrSchedulerConfig::GetEffectiveMaxWavesPerEUForFunction(
    const Function &function) const {
    // Per-kernel max wins when present; otherwise the global default. Mutually
    // exclusive by construction, so this is a plain either/or.
    if (const FunctionConfig *config = GetFunctionConfig(function)) {
        if (config->max_waves_per_eu_.has_value()) {
            return config->max_waves_per_eu_;
        }
    }
    return global_settings_.all_kernels_max_waves;
}

MachineInstrSchedulerConfig::FunctionConfig::FunctionConfig(const std::string &demangled_signature,
                                                            std::optional<int> optsched_occupancy_limit,
                                                            std::optional<int> min_waves_per_eu,
                                                            std::optional<int> max_waves_per_eu) {
    func_signature_ = demangled_signature;
    optsched_occupancy_limit_ = optsched_occupancy_limit;
    min_waves_per_eu_ = min_waves_per_eu;
    max_waves_per_eu_ = max_waves_per_eu;
}

std::string MachineInstrSchedulerConfig::FunctionConfig::ToString() const {
    std::string result = "\t" + func_signature_ +  "\n";
    if(optsched_occupancy_limit_.has_value()) {
        result += "\t\toptsched_occupancy_limit_:" + std::to_string(optsched_occupancy_limit_.value()) + "\n";
    }
    if(min_waves_per_eu_.has_value()) {
        result += "\t\tmin_waves_per_eu_:" + std::to_string(min_waves_per_eu_.value()) + "\n";
    }
    if(max_waves_per_eu_.has_value()) {
        result += "\t\tmax_waves_per_eu_:" + std::to_string(max_waves_per_eu_.value()) + "\n";
    }
    return result;
}

std::string MachineInstrSchedulerConfig::GetSchedulerAsString() const {
    return scheduler_to_str_.at(mi_scheduler_);
}

std::string MachineInstrSchedulerConfig::ToString() const {
    std::string result;

    // Scheduler
    result += "Scheduler: " + GetSchedulerAsString() + "\n";

    // Flags (only those that are set)
    std::string flag_lines;
    for (const auto &binding : kFlagBindings) {
        if (flags_.*(binding.field)) {
            flag_lines += "\t\t" + binding.name.str() + "\n";
        }
    }
    if (!flag_lines.empty()) {
        result += "\tFlags:\n" + flag_lines;
    }

    // Global settings (only those that are set)
    std::string setting_lines;
    for (const auto &binding : kGlobalIntSettingBindings) {
        if (std::optional<int> value = global_settings_.*(binding.field)) {
            setting_lines += "\t\t" + binding.name.str() + " = " +
                             std::to_string(*value) + "\n";
        }
    }
    for (const auto &binding : kGlobalStringSettingBindings) {
        if (const std::optional<std::string> &value = global_settings_.*(binding.field)) {
            setting_lines += "\t\t" + binding.name.str() + " = " + *value + "\n";
        }
    }
    // all_kernels_occupancy is a pair stored outside the scalar table; emit it in
    // the same `<min>,<max>` form it was written (0 = an unset/preserved bound).
    if (global_settings_.all_kernels_min_waves.has_value() ||
        global_settings_.all_kernels_max_waves.has_value()) {
        setting_lines +=
            "\t\tall_kernels_occupancy = " +
            std::to_string(global_settings_.all_kernels_min_waves.value_or(0)) +
            "," +
            std::to_string(global_settings_.all_kernels_max_waves.value_or(0)) +
            "\n";
    }
    if (!setting_lines.empty()) {
        result += "\tGlobal settings:\n" + setting_lines;
    }

    // Scoped settings
    if (!scoped_.empty()) {
        result += "\tScoped settings:\n";
        for (const auto &scope_entry : scoped_) {
            for (const auto &kv : scope_entry.second) {
                std::string scope_prefix =
                    scope_entry.first.empty() ? "" : scope_entry.first + ".";
                result += "\t\t" + scope_prefix + kv.first + " = " + kv.second + "\n";
            }
        }
    }

    // Per-func options
    for(const auto &signature_config : demangled_func_signature_to_config_) {
        result += signature_config.second.ToString();
    }

    return result;
}

void MachineInstrSchedulerConfig::DebugPrint() const {
    llvm::outs() << "============= Howdy from MachineInstrSchedulerConfig ================\n";
    llvm::outs() << ToString();
    llvm::outs().flush();
    llvm::outs() << "=====================================================================\n";
}
