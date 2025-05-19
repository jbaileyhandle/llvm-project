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
            AcoOptSched
        };
        enum class AcoOption{
            RunOnAllFunctions,
            RunRegardlessOfHeurisitcOutcome,
            InvalidOption
        };

        // A class to represent per-function configuration info
        class FunctionConfig {
            public:
                std::string ToString() const;

                FunctionConfig(const std::vector<std::string> &tokens);
                std::string func_signature_;
                std::optional<int> waves_per_eu_;
        };


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

        // Return true if any variant of the AcoOptSched
        // is the configured scheduler
        bool IsAcoOptSched() const;

        // Return true if option is set
        bool HasAcoOption(AcoOption option) const;

        // Debug printing stuff
        void DebugPrint() const;
        std::string ToString() const;

    private:
        MachineInstrSchedulerConfig();

        // Return the configurd scheduler as a string
        std::string GetSchedulerAsString() const;

        // Return the string-equivalnet of the ACO option
        std::string GetAcoOptionAsString(AcoOption option) const;

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

        // Set options for ACO scheduler
        void InitAcoOptions(const std::vector<std::string> &options);

        // Conert a string to the corresponding AcoOption
        AcoOption GetAcoOptionFromString(const std::string &str);

        bool has_config_ = false;
        Scheduler mi_scheduler_ = Scheduler::Default;
        std::unordered_map<std::string, FunctionConfig> demangled_func_signature_to_config_;
        std::set<AcoOption> aco_options_;
        inline static const std::unordered_map<Scheduler, std::string> scheduler_to_str_ {
            {Scheduler::Default, "Default"},
            {Scheduler::MaxOccupancy, "MaxOccupancy"},
            {Scheduler::MaxIlp, "MaxIlp"},
            {Scheduler::IterativeMaxOccupancy, "IterativeMaxOccupancy"},
            {Scheduler::IterativeMaxIlp, "IterativeMaxIlp"},
            {Scheduler::AcoOptSched, "AcoOptSched"}
        };
        inline static const std::unordered_map<AcoOption, std::string> aco_option_to_str_ {
            {AcoOption::InvalidOption, "InvalidOption"},
            {AcoOption::RunOnAllFunctions, "RunOnAllFunctions"},
            {AcoOption::RunRegardlessOfHeurisitcOutcome, "RunRegardlessOfHeurisitcOutcome"}
        };


        MachineInstrSchedulerConfig(const MachineInstrSchedulerConfig &) = delete;
        void operator=(const MachineInstrSchedulerConfig &) = delete;
};

} // end namespace llvm

#endif // LLVM_CODEGEN_MACHINE_INSTR_SCHEDULER_CONFIG_H
