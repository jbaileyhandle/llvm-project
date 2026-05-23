#ifndef LLVM_CODEGEN_MACHINE_INSTR_SCHEDULER_CONFIG_H
#define LLVM_CODEGEN_MACHINE_INSTR_SCHEDULER_CONFIG_H

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <vector>
#include <map>
#include <optional>

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
            // Length-min policy selectors for HierarchicalScheduler.
            // When neither is set, length-min runs with the plain
            // DfsMinimizeLengthPolicy (kNone).
            LengthMinRefineIlp,
            LengthMinRefineOccupancy,
            // HierarchicalScheduler: replace the length-min pass
            // with a length-MAX pass (DfsMaximizeLengthPolicy).
            // Useful as a control / worst-legal-schedule baseline
            // for comparing against the length-min objective. Still
            // subject to the occupancy floor enforced by the
            // earlier occupancy pass. Mutually exclusive with the
            // LengthMin* refine options.
            MaximizeLength,
            // Generic: swap the AMDGPU subtarget's MCSchedModel to
            // jbaile's custom gfx906 model. Affects every consumer of
            // sched-model latency (LLVM's MachineScheduler,
            // OptSched, HierarchicalScheduler, register-pressure
            // analyses, etc.). Valid for any scheduler.
            UseJbaileCustomTimingModel,
            // HierarchicalScheduler: skip subgraph formation in both
            // the occupancy and length passes. DFS searches operate
            // on the flat (un-formed) graph. Useful for isolating
            // the cost/effect of formation when iterating on policy
            // changes or comparing schedulers.
            SkipSubgraphFormation,
            // HierarchicalScheduler: divide every SDep latency by the
            // function's current target occupancy (MFI->getOccupancy)
            // when building the schedule graph, using
            // ceil(latency/divisor) with a floor of 1. Models the
            // fact that other waves on the same SIMD cover most of
            // the memory latency at runtime, so the wave-visible
            // latency of a long-latency op is ~ raw / occupancy.
            // Quick first-pass approximation of the occupancy-aware
            // latency idea — applies to ALL edges, not just memory.
            ScaleEdgeLatenciesByTargetOccupancy,
            // HierarchicalScheduler: in the occupancy-maximization
            // pass, schedule every region with the BFS / dynamic-
            // programming partition search (BfsDpSearch) instead of
            // the default DFS occupancy search.
            BfsDpForOccupancy,
            // HierarchicalScheduler: in the occupancy-maximization
            // pass, run the full DecomposeAndSchedule pipeline with
            // the BfsDpWithDfsFallback preset — form subgraphs,
            // schedule each in isolation (BFS-DP continuous, DFS
            // fallback at matching metric), lock with order edges,
            // then run the outer search (BFS-DP integer seeded with
            // the region's original occupancy, DFS fallback at the
            // matching integer-metric policy). Only affects the
            // occupancy pass; the length pass is unchanged. Mutually
            // exclusive with BfsDpForOccupancy and
            // SkipSubgraphFormation.
            DecomposeForOccupancy,
            // HierarchicalScheduler: after subgraph formation, dump
            // each region's DAG (nodes, edges, subgraph membership) to
            // a Cytoscape.js JSON file under ./subgraph_dags/ for
            // offline visualization. Observational only — does not
            // change scheduling. See
            // HierarchicalScheduler/viz/subgraph_dag_viewer.html.
            DumpSubgraphDag,
            // HierarchicalScheduler: in the DecomposeAndSchedule path (i.e.
            // with DecomposeForOccupancy), form subgraphs by acyclic min-cut
            // of the data-dependency DAG (via the dagP partitioner) instead of
            // the dom-tree pipeline. No effect unless that path runs. Mutually
            // exclusive with SkipSubgraphFormation. See
            // HierarchicalScheduler/MinCutFormation.{h,cpp}.
            MinCutFormation,
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

        // Look up a scoped "<scope>.<key> = <value>" setting parsed from
        // misched.txt. Returns the raw value string if present, std::nullopt
        // otherwise. The generic config stores these uninterpreted; the
        // per-scheduler layer (e.g. AMDGPU HierarchicalConfig) maps them to
        // typed fields and validates them. A dotless top-level setting is
        // stored under the empty scope.
        std::optional<llvm::StringRef> GetScopedSetting(llvm::StringRef scope,
                                                        llvm::StringRef key) const;

        // Return the entire scope -> key -> value store. The per-scheduler
        // layer iterates this to validate (fatal on an unknown scope or key)
        // and to read every setting; GetScopedSetting is the point-read
        // counterpart. Still uninterpreted at this layer.
        const std::map<std::string, std::map<std::string, std::string>> &
        GetAllScopedSettings() const { return scoped_; }

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

        // Parse a single "<scope>.<key>=<value>" (or dotless "<key>=<value>")
        // token into the scoped_ store. The left-hand side is split on its
        // first dot: the first segment is the scope, the remainder (which may
        // itself contain dots, e.g. "search.timeout") is the key. A dotless
        // left-hand side lands under the empty scope.
        void ParseScopedSetting(const std::string &token);

        // Convert a string to the corresponding SchedulerOption
        SchedulerOption GetSchedulerOptionFromString(const std::string &str);

        bool has_config_ = false;
        Scheduler mi_scheduler_ = Scheduler::Default;
        std::unordered_map<std::string, FunctionConfig> demangled_func_signature_to_config_;
        std::set<SchedulerOption> options_;
        // Dumb scope -> key -> value store for "<scope>.<key> = <value>"
        // settings. Uninterpreted here; read via GetScopedSetting and given
        // meaning by the per-scheduler config layer.
        std::map<std::string, std::map<std::string, std::string>> scoped_;
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
            {SchedulerOption::RunShakedowns, "RunShakedowns"},
            {SchedulerOption::LengthMinRefineIlp, "LengthMinRefineIlp"},
            {SchedulerOption::LengthMinRefineOccupancy, "LengthMinRefineOccupancy"},
            {SchedulerOption::MaximizeLength, "MaximizeLength"},
            {SchedulerOption::UseJbaileCustomTimingModel, "UseJbaileCustomTimingModel"},
            {SchedulerOption::SkipSubgraphFormation, "SkipSubgraphFormation"},
            {SchedulerOption::ScaleEdgeLatenciesByTargetOccupancy,
             "ScaleEdgeLatenciesByTargetOccupancy"},
            {SchedulerOption::BfsDpForOccupancy, "BfsDpForOccupancy"},
            {SchedulerOption::DecomposeForOccupancy, "DecomposeForOccupancy"},
            {SchedulerOption::DumpSubgraphDag, "DumpSubgraphDag"},
            {SchedulerOption::MinCutFormation, "MinCutFormation"}
        };


        MachineInstrSchedulerConfig(const MachineInstrSchedulerConfig &) = delete;
        void operator=(const MachineInstrSchedulerConfig &) = delete;
};

} // end namespace llvm

#endif // LLVM_CODEGEN_MACHINE_INSTR_SCHEDULER_CONFIG_H
