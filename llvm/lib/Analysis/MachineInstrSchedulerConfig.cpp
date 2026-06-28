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
        {"use_jbaile_custom_timing_model", &Flags::use_jbaile_custom_timing_model},
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

    // Return the function's configured minimum + maximum waves per eu
    std::optional<std::pair<int, int>> GetFunctionExistingWavesPerEu(const Function &F) {
        if(!F.hasFnAttribute(waves_per_eu_attr)) {
            return std::nullopt;
        }

        std::string existing_attr_str = F.getFnAttribute(waves_per_eu_attr).getValueAsString().str();
        size_t comma_pos = existing_attr_str.find(',');
        if(comma_pos == std::string::npos) {
            return std::nullopt;
        }

        return std::make_pair(std::stoi(existing_attr_str.substr(0,comma_pos)), std::stoi(existing_attr_str.substr(comma_pos+1)));
    }

    // Return the function's configured maximum waves per eu
    std::optional<int> GetFunctionExistingMaximumWavesPerEu(const Function &F) {
        std::optional<std::pair<int, int>> min_max = GetFunctionExistingWavesPerEu(F);
        if(min_max.has_value()) {
            return min_max->second;
        }
        return std::nullopt;
    }

    int GetOrInferFunctionExistingMaximumWavesPerEU(const Function &function) {
        std::optional<int> existing_maximum_waves_per_eu = GetFunctionExistingMaximumWavesPerEu(function);
        return existing_maximum_waves_per_eu.value_or(10);
    }

    // Return the function's configured minimum waves per eu
    std::optional<int> GetFunctionExistingMinimumWavesPerEu(const Function &F) {
        std::optional<std::pair<int, int>> min_max = GetFunctionExistingWavesPerEu(F);
        if(min_max.has_value()) {
            return min_max->first;
        }
        return std::nullopt;
    }

    int GetOrInferFunctionExistingMinimumWavesPerEU(const Function &function) {
        std::optional<int> existing_minimum_waves_per_eu = GetFunctionExistingMinimumWavesPerEu(function);
        return existing_minimum_waves_per_eu.value_or(1);
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

bool MachineInstrSchedulerConfig::SetGlobalSettingIfKnown(llvm::StringRef key,
                                                          llvm::StringRef value) {
    // No unscoped global settings yet; the unroll knobs are added in a later
    // step (each becomes a typed field handled here).
    (void)key;
    (void)value;
    return false;
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
        // Unscoped global setting -> its typed field.
        if (SetGlobalSettingIfKnown(key, value)) {
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

    // waves is the optional 3rd field. Present => the per-function waves/occupancy
    // target: OptSched reads it as an occupancy limit (OptSchedGCNTarget), others
    // apply it as the amdgpu-waves-per-eu attribute. Absent => just register the
    // function (no target override), which still opts it into OptSched, since
    // OptSched's per-function opt-in is "has a config entry".
    std::optional<int> waves;
    if (fields.size() == 3) {
        int parsed = 0;
        if (fields[2].getAsInteger(10, parsed) || parsed < 1) {
            report_fatal_error(Twine("misched.txt: kernel line '") + rest +
                               "' has invalid waves '" + fields[2] +
                               "' (expected an integer >= 1)");
        }
        waves = parsed;
    }

    if (demangled_func_signature_to_config_.count(demangled_func_signature)) {
        report_fatal_error(Twine("misched.txt: duplicate kernel configuration for '") +
                           demangled_func_signature + "'");
    }
    demangled_func_signature_to_config_.emplace(
        demangled_func_signature,
        FunctionConfig(demangled_func_signature, waves));
}

MachineInstrSchedulerConfig::MachineInstrSchedulerConfig() {
    // misched.txt is a line-oriented, order-independent config. Each non-blank,
    // non-comment line is either a `kernel <m|d>/<sig>/[waves]` per-function
    // line or a whitespace-separated list of option tokens (a scheduler name,
    // a bare flag, or a `<key>=<value>` setting). Options may be split across
    // any number of lines; exactly one scheduler name must appear.
    std::ifstream misched_config_file("misched.txt");
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
    if(!HasFunctionConfig(function)) {
        return;
    }
    const std::optional<int> &minimum_waves_per_eu_opt = GetFunctionConfig(function)->waves_per_eu_;
    if(!minimum_waves_per_eu_opt.has_value()) {
        return;
    }

    // TODO: A kernel must speicfy maximum threaeds per block to unlock >64 registers per thread. Do we want to force this here?
    /*
    TODO: Do we actually want to preserve maximum waves per eu? Maybe!
    But the question is - do we want to overwrite the perscribed maximum_waves_per_eu
    with something else? Possibilities:
    maximum_waves_per_eu = maximum_waves_per_eu form attribute
    or
    maximum_waves_per_eu = std::max(minimum_waves_per_eu, maximum_waves_per_eu from attribute)
    or
    maximum_waves_per_eu = minimum_waves_per_eu
    */
    int minimum_waves_per_eu = minimum_waves_per_eu_opt.value();
    int maximum_waves_per_eu = GetOrInferFunctionExistingMaximumWavesPerEU(function);
    maximum_waves_per_eu = std::max(minimum_waves_per_eu, maximum_waves_per_eu);

    // Clear old attribute
    function.removeFnAttr(waves_per_eu_attr);

    // Set new attribute
    std::string new_waves_per_eu_pair = std::to_string(minimum_waves_per_eu) + "," + std::to_string(maximum_waves_per_eu);
    function.addFnAttr(waves_per_eu_attr, new_waves_per_eu_pair);
}

MachineInstrSchedulerConfig::FunctionConfig::FunctionConfig(const std::string &demangled_signature, std::optional<int> waves_per_eu) {
    func_signature_ = demangled_signature;
    waves_per_eu_ = waves_per_eu;
}

std::string MachineInstrSchedulerConfig::FunctionConfig::ToString() const {
    std::string result = "\t" + func_signature_ +  "\n";
    if(waves_per_eu_.has_value()) {
        result += "\t\twaves_per_eu_:" + std::to_string(waves_per_eu_.value()) + "\n";
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
