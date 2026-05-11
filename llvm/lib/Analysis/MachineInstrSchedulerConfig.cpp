#include "llvm/Analysis/MachineInstrSchedulerConfig.h"

#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/ADT/Twine.h"
#include "llvm/Demangle/Demangle.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/ErrorHandling.h"

#include <cassert>
#include <cstdio>
#include <fstream>
#include <sstream>

using namespace llvm;

namespace {
    const std::string waves_per_eu_attr = "amdgpu-waves-per-eu";

    // Split a string according to delimiter
    // Return split strings as a vector
    std::vector<std::string> SplitByDelimter(const std::string &input, char delimiter='/') {

        std::vector<std::string> tokens;
        std::stringstream ss(input);
        std::string token;
        while(std::getline(ss, token, delimiter)) {
            if(!token.empty()) {
                tokens.push_back(token);
            }
        }
        return tokens;
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

bool MachineInstrSchedulerConfig::HasSchedulingOption(MachineInstrSchedulerConfig::SchedulerOption option) const {
    return (options_.find(option) != options_.end());
}

bool MachineInstrSchedulerConfig::IsPostRASchedulingDisabled() const {
    return HasSchedulingOption(SchedulerOption::DisablePostRAScheduling);
}

MachineInstrSchedulerConfig::Scheduler MachineInstrSchedulerConfig::GetScheduler() const {
    return mi_scheduler_;
}

MachineInstrSchedulerConfig::SchedulerOption MachineInstrSchedulerConfig::GetSchedulerOptionFromString(const std::string &str) {
    SchedulerOption option = StringSwitch<SchedulerOption>(llvm::StringRef(str))
        .Case("DisablePostRAScheduling", SchedulerOption::DisablePostRAScheduling)
        .Case("RunOnAllFunctions", SchedulerOption::RunOnAllFunctions)
        .Case("RunRegardlessOfHeurisitcOutcome", SchedulerOption::RunRegardlessOfHeurisitcOutcome)
        .Case("UseContinuousOccupancyScore", SchedulerOption::UseContinuousOccupancyScore)
        .Case("MaliciousScheduler", SchedulerOption::MaliciousScheduler)
        .Case("RunShakedowns", SchedulerOption::RunShakedowns)
        .Default(SchedulerOption::InvalidOption);

    if (option == SchedulerOption::InvalidOption) {
        llvm::report_fatal_error("Invalid SchedulerOption: " + llvm::StringRef(str));
    }
    return option;
}

bool MachineInstrSchedulerConfig::IsValidOptionForScheduler(SchedulerOption option, Scheduler scheduler) {
    switch (option) {
    // Generic — valid for any scheduler
    case SchedulerOption::DisablePostRAScheduling:
        return true;
    // OptSched-specific
    case SchedulerOption::RunOnAllFunctions:
    case SchedulerOption::RunRegardlessOfHeurisitcOutcome:
    case SchedulerOption::UseContinuousOccupancyScore:
        return (scheduler == Scheduler::AcoOptSched || scheduler == Scheduler::BnbOptSched);
    // HierarchicalScheduler-specific
    case SchedulerOption::MaliciousScheduler:
    case SchedulerOption::RunShakedowns:
        return (scheduler == Scheduler::HierarchicalScheduler);
    default:
        return false;
    }
}

void MachineInstrSchedulerConfig::InitSchedulerOptions(const std::vector<std::string> &option_strings) {
    for (const auto &str : option_strings) {
        SchedulerOption option = GetSchedulerOptionFromString(str);
        if (!IsValidOptionForScheduler(option, mi_scheduler_)) {
            llvm::report_fatal_error("Option '" + llvm::StringRef(str) +
                "' is not valid for scheduler '" + llvm::StringRef(GetSchedulerAsString()) + "'");
        }
        options_.insert(option);
    }
}


MachineInstrSchedulerConfig::MachineInstrSchedulerConfig() {
    std::ifstream misched_config_file("misched.txt");
    if(misched_config_file) {
        has_config_ = true;

        // Parse 1st line
        std::string line;
        std::getline(misched_config_file, line);
        std::vector<std::string> first_line_tokens = SplitByWhitespace(line);
        std::string misched = first_line_tokens[0];

        // Set scheduler
        llvm::StringRef misched_ref(misched);
        mi_scheduler_ = StringSwitch<Scheduler>(misched_ref)
            .Case("MaxOccupancy", Scheduler::MaxOccupancy)
            .Case("MaxIlp", Scheduler::MaxIlp)
            .Case("IterativeMaxOccupancy", Scheduler::IterativeMaxOccupancy)
            .Case("IterativeMaxIlp", Scheduler::IterativeMaxIlp)
            .Case("AcoOptSched", Scheduler::AcoOptSched)
            .Case("BnbOptSched", Scheduler::BnbOptSched)
            .Case("HierarchicalScheduler", Scheduler::HierarchicalScheduler)
            .Default(Scheduler::InvalidOption);

        if(mi_scheduler_ == Scheduler::InvalidOption) {
            llvm::report_fatal_error("Invalid machine instruction scheduler: " + misched_ref);
        }

        // Parse and validate options
        InitSchedulerOptions(std::vector<std::string>(first_line_tokens.begin()+1, first_line_tokens.end()));

        // Read in per-func info
        while(std::getline(misched_config_file, line)) {

            // Skip lines that have been commented out
            if (!line.empty() && line.at(0) == '#') {
                continue;
            }

            // Make per-line config object && register
            std::vector<std::string> func_tokens = SplitByDelimter(line);
            std::string demangled_func_signature;
            if(func_tokens[0] == "d" || func_tokens[0] == "D") {
                demangled_func_signature = func_tokens[1];
            } else if (func_tokens[0] == "m" || func_tokens[0] == "M") {
                demangled_func_signature = DemangleFunctionSignature(func_tokens[1]);
            } else {
                llvm::report_fatal_error("First field of function config in misched.txt must indicate mangled (m/M) or demangled (d/D) function name");
            }

            if(demangled_func_signature_to_config_.find(demangled_func_signature) != demangled_func_signature_to_config_.end()) {
                llvm::report_fatal_error("In processing misched, found duplicate function configurations");
            }
            demangled_func_signature_to_config_.emplace(std::piecewise_construct,
                std::forward_as_tuple(demangled_func_signature), 
                std::forward_as_tuple(demangled_func_signature, func_tokens));
        }
    }

    DebugPrint(); 
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

MachineInstrSchedulerConfig::FunctionConfig::FunctionConfig(const std::string &demangled_signature, const std::vector<std::string> &tokens) {
    func_signature_ = std::move(demangled_signature);
    if(tokens.size() > 2 && !tokens[2].empty()) {
        waves_per_eu_ = std::stoi(tokens[2]);
    }
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

std::string MachineInstrSchedulerConfig::GetSchedulerOptionAsString(SchedulerOption option) const {
    return option_to_str_.at(option);
}

std::string MachineInstrSchedulerConfig::ToString() const {
    std::string result;

    // Scheduler
    result += "Scheduler: " + GetSchedulerAsString() + "\n";

    // Options
    if (!options_.empty()) {
        result += "\tOptions:\n";
        for (const auto option : options_) {
            result += "\t\t" + GetSchedulerOptionAsString(option) + "\n";
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

