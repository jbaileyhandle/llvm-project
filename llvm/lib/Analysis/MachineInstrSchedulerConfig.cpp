#include "llvm/Analysis/MachineInstrSchedulerConfig.h"

#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/ErrorHandling.h"

#include <cassert>
#include <cxxabi.h>
#include <cstdio>
#include <fstream>
#include <sstream>

using namespace llvm;

namespace {
    const std::string waves_per_eu_attr = "amdgpu-waves-per-eu";

    // Split a string according to delimiter
    // Return split strings as a vector
    std::vector<std::string> Split(const std::string &input, char delimiter='/') {
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

    // Pass in a function's mangled name
    // Returns de-mangled name
    std::string DemangleFunctionSignature(const std::string &mangled_signature) {
        int status = 0;

        char *demangled = abi::__cxa_demangle(mangled_signature.c_str(), nullptr, nullptr, &status);
        std::string result = (status == 0 && demangled) ? demangled : mangled_signature;
        std::free(demangled);
        return result;
    }

    // Return the function's configured maximum waves per eu
    std::optional<int> GetFunctionExistingMaximumWavesPerEu(const Function &F) {
        if(!F.hasFnAttribute(waves_per_eu_attr)) {
            return std::nullopt;
        }

        std::string existing_attr_str = F.getFnAttribute(waves_per_eu_attr).getValueAsString().str();
        size_t comma_pos = existing_attr_str.find(',');
        if(comma_pos == std::string::npos) {
            return std::nullopt;
        }

        return std::stoi(existing_attr_str.substr(comma_pos+1));
    }

    int GetOrInferFunctionExistingMaximumWavesPerEU(const Function &function) {
        std::optional<int> existing_maximum_waves_per_eu = GetFunctionExistingMaximumWavesPerEu(function);
        return existing_maximum_waves_per_eu.value_or(10);
    }

} // end namespace

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

MachineInstrSchedulerConfig::Scheduler MachineInstrSchedulerConfig::GetScheduler() const {
    assert(Scheduler != config);
    return mi_scheduler_;
}

MachineInstrSchedulerConfig::MachineInstrSchedulerConfig() {
    std::ifstream misched_config_file("misched.txt");
    if(misched_config_file) {
        has_config_ = true;

        // Parse 1st line
        std::string line;
        std::getline(misched_config_file, line);
        std::vector<std::string> first_line_tokens = Split(line);
        assert((first_line_tokens.length() == 1) && Expected exactly one word in first line of misched.txt);
        std::string misched = first_line_tokens[0];

        // Set scheduler
        llvm::StringRef misched_ref(misched);
        mi_scheduler_ = StringSwitch<Scheduler>(misched_ref)
            .Case("MaxOccupancy", Scheduler::MaxOccupancy)
            .Case("MaxIlp", Scheduler::MaxIlp)
            .Case("IterativeMaxOccupancy", Scheduler::IterativeMaxOccupancy)
            .Case("IterativeMaxIlp", Scheduler::IterativeMaxIlp)
            .Case("AcoOptSched", Scheduler::AcoOptSched)
            .Default(Scheduler::InvalidOption);

        if(mi_scheduler_ == Scheduler::InvalidOption) {
            llvm::report_fatal_error("Invalid machine instruction scheduler: " + misched_ref);
        }

        // Read in per-func info
        while(std::getline(misched_config_file, line)) {

            // Skip lines that have been commented out
            if (!line.empty() && line.at(0) == '#') {
                continue;
            }

            // Make per-line config object && register
            std::vector<std::string> func_tokens = Split(line);
            if(demangled_func_signature_to_config_.find(func_tokens[0]) != demangled_func_signature_to_config_.end()) {
                llvm::report_fatal_error("In processing misched, found duplicate function configurations");
            }
            demangled_func_signature_to_config_.emplace(func_tokens[0], func_tokens);
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

bool MachineInstrSchedulerConfig::HasFunctionConfigForDemangledFunctionSignature(const std::string &demangled_signature) const {
    auto name_config_itr = demangled_func_signature_to_config_.find(demangled_signature);
    return name_config_itr != demangled_func_signature_to_config_.end();
}

bool MachineInstrSchedulerConfig::HasFunctionConfigForMangledFunctionSignature(const std::string &mangled_signature) const {
    std::string demangled_name = DemangleFunctionSignature(mangled_signature);
    return HasFunctionConfigForDemangledFunctionSignature(demangled_name);
}

bool MachineInstrSchedulerConfig::HasFunctionConfig(const Function &function) const {
    return HasFunctionConfigForMangledFunctionSignature(function.getName().str());
}

const MachineInstrSchedulerConfig::FunctionConfig *MachineInstrSchedulerConfig::GetFunctionConfig(const Function &function) const {
    return GetFunctionConfigFromMangledFunctionSignature(function.getName().str());
}

void MachineInstrSchedulerConfig::SetFunctionWavesPerEUAttributeBasedOnConfig(Function &function) const {
    if(!HasFunctionConfig(function)) {
        return;
    }

    /*
    TODO: Do we actually want to preserve maximum waves per eu? Maybe! A kernel
    must speicfy maximum waves per eu to unlock >64 registers per thread.
    But the question is - do we want to overwrite the perscribed maximum_waves_per_eu
    with something else? Possibilities:
    maximum_waves_per_eu = maximum_waves_per_eu form attribute
    or
    maximum_waves_per_eu = std::max(minimum_waves_per_eu, maximum_waves_per_eu from attribute)
    or
    maximum_waves_per_eu = minimum_waves_per_eu
    */
    int maximum_waves_per_eu = GetOrInferFunctionExistingMaximumWavesPerEU(function);
    int minimum_waves_per_eu = GetFunctionConfig(function)->waves_per_eu_;
    maximum_waves_per_eu = std::max(minimum_waves_per_eu, maximum_waves_per_eu);

    // Clear old attribute
    function.removeFnAttr(waves_per_eu_attr);

    // Set new attribute
    std::string new_waves_per_eu_pair = std::to_string(minimum_waves_per_eu) + "," + std::to_string(maximum_waves_per_eu);
    function.addFnAttr(waves_per_eu_attr, new_waves_per_eu_pair);
}

MachineInstrSchedulerConfig::FunctionConfig::FunctionConfig(const std::vector<std::string> &tokens) {
    func_signature_ = std::move(tokens[0]);
    waves_per_eu_ = std::stoi(tokens[1]);
}

std::string MachineInstrSchedulerConfig::FunctionConfig::ToString() const {
    std::string result = "\t" + func_signature_ +  "\n";
    result += "\t\twaves_per_eu_:" + std::to_string(waves_per_eu_) + "\n";
    return result;
}

std::string MachineInstrSchedulerConfig::GetSchedulerAsString() const {
    return scheduler_to_str_.at(mi_scheduler_);
}

std::string MachineInstrSchedulerConfig::ToString() const {
    std::string result;
    result += "Scheduler: " + GetSchedulerAsString() + "\n";
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

