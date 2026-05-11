#ifndef LLVM_CODEGEN_MACHINE_INSTR_SCHEDULER_CONFIG_H
#define LLVM_CODEGEN_MACHINE_INSTR_SCHEDULER_CONFIG_H

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <vector>

#include "llvm/IR/Function.h"

namespace llvm {

class MachineInstrSchedulerConfig {
    public:
        enum class Scheduler {
            Default,
            InvalidOption,
            MaxOccupancy,
            MaxIlp,
            IterativeMaxOccupancy,
            IterativeMaxIlp,
            AcoOptSched,
            BnbOptSched,
            HierarchicalScheduler
        };
        enum class SchedulerOption {
            // Generic (any scheduler)
            DisablePostRAScheduling,
            // OptSched-specific (AcoOptSched, BnbOptSched)
            RunOnAllFunctions,
            RunRegardlessOfHeurisitcOutcome,
            UseContinuousOccupancyScore,
            // HierarchicalScheduler-specific
            MaliciousScheduler,
            RunShakedowns,
            // Sentinel
            InvalidOption
        };

        static bool IsValidOptionForScheduler(SchedulerOption option, Scheduler scheduler);

        // A class to represent per-function configuration info
        class FunctionConfig {
            public:
                std::string ToString() const;

                FunctionConfig(const std::string &demangled_signature, const std::vector<std::string> &tokens);
                std::string func_signature_;
                std::optional<int> waves_per_eu_;
        };


        // Demangle a C++ (Itanium ABI) mangled function signature.
        // Fatal error if demangling fails.
        static std::string DemangleFunctionSignature(const std::string &mangled_signature);

        // Return true if there is scheduler configuration
        bool HasConfig() const;

        // Return the configuration
        static const MachineInstrSchedulerConfig &GetConfig();
        
        // Return true if we have configuration information for function
        bool HasFunctionConfig(const Function &function) const;
        bool HasFunctionConfigForMangledFunctionSignature(const llvm::StringRef &mangled_signature) const;

        // Return the FunctionConfig for the function with a given mangled signature
        // Return nullptr if not found
        const FunctionConfig *GetFunctionConfigFromMangledFunctionSignature(const llvm::StringRef &mangled_signature) const;

        // If there is a configuration for function, set waves per eu attribute
        // for the function based on the configuration
        void SetFunctionWavesPerEUAttributeBasedOnConfig(Function &function) const;

        // Return the configured scheduler
        Scheduler GetScheduler() const;

        // Return true if AcoOptSched is the configured scheduler
        bool IsAcoOptSched() const;

        // Return true if BnbOptSched is the configured scheduler
        bool IsBnbOptSched() const;

        // Return true if any OptSched variant (ACO or BnB) is the configured scheduler
        bool IsOptSched() const;

        // Return true if HierarchicalScheduler is the configured scheduler
        bool IsHierarchicalScheduler() const;

        // Return true if option is set
        bool HasSchedulingOption(SchedulerOption option) const;

        // Convenience: return true if post-RA scheduling is disabled
        bool IsPostRASchedulingDisabled() const;

        // Debug printing stuff
        void DebugPrint() const;
        std::string ToString() const;

    private:
        MachineInstrSchedulerConfig();

        // Return the configurd scheduler as a string
        std::string GetSchedulerAsString() const;

        // Return the string-equivalent of a scheduler option
        std::string GetSchedulerOptionAsString(SchedulerOption option) const;

        // Return the FunctionConfig for the function with a given demangled signature
        // Return nullptr if not found
        const FunctionConfig *GetFunctionConfigFromDemangledFunctionSignature(const std::string &demangled_signature) const;

        // Return the FunctionConfig for the function with a given mangled signature
        // Return nullptr if not found
        const FunctionConfig *GetFunctionConfigFromMangledFunctionSignature(const std::string &mangled_signature) const;

        // Return true iff we had a function configuration for function with the
        // given demangled signature
        bool HasFunctionConfigForDemangledFunctionSignature(const std::string &demangled_signature) const;

        // Return true iff we had a function configuration for function with the
        // given mangled signature
        bool HasFunctionConfigForMangledFunctionSignature(const std::string &mangled_signature) const;

        // Get the configuration for a function
        const FunctionConfig *GetFunctionConfig(const Function &function) const;

        // Parse and validate options from misched.txt tokens
        void InitSchedulerOptions(const std::vector<std::string> &option_strings);

        // Convert a string to the corresponding SchedulerOption
        SchedulerOption GetSchedulerOptionFromString(const std::string &str);

        bool has_config_ = false;
        Scheduler mi_scheduler_ = Scheduler::Default;
        std::unordered_map<std::string, FunctionConfig> demangled_func_signature_to_config_;
        std::set<SchedulerOption> options_;
        inline static const std::unordered_map<Scheduler, std::string> scheduler_to_str_ {
            {Scheduler::Default, "Default"},
            {Scheduler::MaxOccupancy, "MaxOccupancy"},
            {Scheduler::MaxIlp, "MaxIlp"},
            {Scheduler::IterativeMaxOccupancy, "IterativeMaxOccupancy"},
            {Scheduler::IterativeMaxIlp, "IterativeMaxIlp"},
            {Scheduler::AcoOptSched, "AcoOptSched"},
            {Scheduler::BnbOptSched, "BnbOptSched"},
            {Scheduler::HierarchicalScheduler, "HierarchicalScheduler"}
        };
        inline static const std::unordered_map<SchedulerOption, std::string> option_to_str_ {
            {SchedulerOption::InvalidOption, "InvalidOption"},
            {SchedulerOption::DisablePostRAScheduling, "DisablePostRAScheduling"},
            {SchedulerOption::RunOnAllFunctions, "RunOnAllFunctions"},
            {SchedulerOption::RunRegardlessOfHeurisitcOutcome, "RunRegardlessOfHeurisitcOutcome"},
            {SchedulerOption::UseContinuousOccupancyScore, "UseContinuousOccupancyScore"},
            {SchedulerOption::MaliciousScheduler, "MaliciousScheduler"},
            {SchedulerOption::RunShakedowns, "RunShakedowns"}
        };


        MachineInstrSchedulerConfig(const MachineInstrSchedulerConfig &) = delete;
        void operator=(const MachineInstrSchedulerConfig &) = delete;
};

} // end namespace llvm

#endif // LLVM_CODEGEN_MACHINE_INSTR_SCHEDULER_CONFIG_H
