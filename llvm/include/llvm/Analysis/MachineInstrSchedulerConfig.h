#ifndef LLVM_CODEGEN_MACHINE_INSTR_SCHEDULER_CONFIG_H
#define LLVM_CODEGEN_MACHINE_INSTR_SCHEDULER_CONFIG_H

#include <string>
#include <unordered_map>
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

        // Every boolean flag settable as a bare token in misched.txt (e.g.
        // `skip_length_pass`). The flag's spelling IS its field name; the
        // spelling -> field table in the .cpp (kFlagSpecs) is the single
        // source of truth for which flags are valid. Read via GetFlags().
        struct Flags {
            // Generic / middle-end (any scheduler).
            bool disable_post_ra_scheduling = false;
            bool enable_runtime_unroll = false;
            // Skip LICM on the device (AMDGPU) side -- an occupancy experiment:
            // LICM hoists loop-invariant values, lengthening live ranges and
            // raising register pressure. Consumed in LICM.cpp.
            bool disable_licm = false;
            // Disable memory-op clustering for every scheduler. All of them
            // (MaxOccupancy pre-RA, the shared post-RA pass, OptSched, and the
            // HierarchicalScheduler) build their load/store cluster edges through
            // createLoad/StoreClusterDAGMutation, so gating those factories on
            // this flag turns clustering off uniformly. Consumed in
            // MachineScheduler.cpp. (Note: MaxILP adds no cluster mutation
            // pre-RA, so only its post-RA schedule is affected.)
            bool disable_mem_clustering = false;
            // Per-kernel occupancy target, MaxOccupancy side: when a
            // `kernel d/<sig>/<min>,<max>` line supplies a `max`, MaxOccupancy's
            // occupancy-raising reschedule stages (the UnclusteredHighRP bump and
            // the PreRARemat init cap -- the only callers of
            // GetEffectiveMaxWavesPerEU) clamp to that max so they will not
            // ratchet occupancy above the configured target. Set this flag to
            // skip that clamp: GetEffectiveMaxWavesPerEU returns the raw attribute
            // max, so those stages maximize occupancy up to the attribute max as
            // usual. Scoped to those two sites only -- the MFI-ctor cap, the
            // MaxOccupancy factory, and Hierarchical's ApplyOccupancyTargetCap are
            // unaffected. Consumed in GCNSchedStrategy.cpp.
            bool disable_max_occ_effective_max_waves_cap = false;
            // OptSched (AcoOptSched, BnbOptSched).
            bool run_on_all_functions = false;
            bool run_regardless_of_heuristic_outcome = false;
            bool use_continuous_occupancy_score = false;
            // HierarchicalScheduler toggles.
            bool malicious = false;
            bool run_shakedowns = false;
            bool dump_subgraph_dag = false;
            bool dump_search_outcomes = false;
            bool scale_edge_latencies = false;
            bool skip_occupancy_pass = false;
            bool skip_length_pass = false;
            // HierarchicalScheduler length pass: ignore the occupancy target.
            // The length DFS normally prunes any schedule whose register-only
            // occupancy would fall below the function occupancy target ("Gate 1"
            // in SearchPolicies.cpp); with this set, that gate is skipped so the
            // length pass optimizes length unconstrained by occupancy (the
            // spill-regression gates still apply). Implies skip_occupancy_pass:
            // maximizing occupancy first is pointless when the length pass will
            // ignore the target anyway. Consumed via HierarchicalConfig.
            bool length_ignore_occupancy = false;
        };

        // Every unscoped global setting writable as `<name>=<value>` in
        // misched.txt (e.g. `unroll_threshold=300`, `timing_model=fast_memory`).
        // Each is std::optional so an unset knob leaves the consumer's own
        // default untouched. The spelling -> field tables in the .cpp
        // (kGlobalIntSettingBindings for ints, kGlobalStringSettingBindings for strings)
        // are the single source of truth for which settings are valid. Read via
        // GetGlobalSettings(). These are generic middle-end knobs (e.g. the
        // loop-unroll preferences AMDGPU reads in getUnrollingPreferences, or the
        // alternate sched-model name AMDGPU swaps to in GCNSubtarget).
        struct GlobalSettings {
            std::optional<int> unroll_threshold;
            std::optional<int> partial_unroll_threshold;
            std::optional<int> runtime_unroll_factor;
            // Per-instruction scheduling time budget (ms per instruction in a
            // region), applied to BOTH OptSched and the HierarchicalScheduler so
            // the two can be compared given equal wall-clock effort. Separate
            // knobs for the occupancy pass and the length pass (usually set
            // equal). A region's budget is this value times its instruction
            // count. Consumed by OptSched (BnB region/length timeouts; ACO host
            // loop deadline) and the HierarchicalScheduler (per-region search
            // timeout). Microseconds so that sub-millisecond per-region budgets
            // can be expressed and enforced (the schedulers carry the budget in
            // microseconds internally). unset = leave each scheduler's own
            // default in place.
            std::optional<int> time_per_instr_occupancy_us;
            std::optional<int> time_per_instr_length_us;
            // Name of an alternate target sched (timing) model to swap in, e.g.
            // "fast_memory". Interpreted by the target (AMDGPU); unknown names are
            // rejected there, not here.
            std::optional<std::string> timing_model;
            // Global occupancy target applied to EVERY function, written as
            // `all_kernels_occupancy = <min>,<max>` -- the same `<min>,<max>`
            // grammar (and `0`-preserves-that-bound semantics) as a per-kernel
            // `kernel .../<min>,<max>` line, but with no signature: the default
            // for all functions. The min becomes an amdgpu-waves-per-eu floor on
            // every function; the max becomes the occupancy ceiling each
            // scheduler consumes its own way (MaxOccupancy / HierarchicalScheduler
            // / MaxIlp). An explicit per-kernel `kernel` line overrides this
            // global for that one function. unset = no global override. Parsed as
            // a pair (not via the int table) so it shares the kernel-line waves
            // grammar; stored split into the two bounds below.
            std::optional<int> all_kernels_min_waves;
            std::optional<int> all_kernels_max_waves;
        };

        // A class to represent per-function configuration info
        class FunctionConfig {
            public:
                std::string ToString() const;

                FunctionConfig(const std::string &demangled_signature,
                               std::optional<int> optsched_occupancy_limit,
                               std::optional<int> min_waves_per_eu,
                               std::optional<int> max_waves_per_eu);

                std::string func_signature_;

                // Single-value `kernel .../<T>` form: an occupancy limit consumed
                // only by OptSched. Non-OptSched schedulers reject it (see
                // SetFunctionWavesPerEUAttributeBasedOnConfig) -- they target
                // occupancy through the `<min>,<max>` pair form below. Mutually
                // exclusive with the pair.
                std::optional<int> optsched_occupancy_limit_;

                // Pair `kernel .../<min>,<max>` form: per-kernel amdgpu-waves-per-eu
                // overrides. nullopt = preserve the function's existing bound (the
                // misched.txt `0` sentinel maps to nullopt here). Applied by
                // SetFunctionWavesPerEUAttributeBasedOnConfig as a pure replacement
                // of each set bound (no min/max merge).
                std::optional<int> min_waves_per_eu_;
                std::optional<int> max_waves_per_eu_;
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

        // Effective occupancy-max ceiling for `function`: its per-kernel
        // `kernel .../<min>,<max>` max if it has one, otherwise the global
        // all_kernels_occupancy max. The two are mutually exclusive (rejected at
        // parse time), so at most one is ever set. Every occupancy-max consumer
        // (the MFI-ctor cap, the MaxOccupancy target, Hierarchical's
        // ApplyOccupancyTargetCap) routes through this so a global default reaches
        // them all. nullopt = no ceiling configured.
        std::optional<int> GetEffectiveMaxWavesPerEUForFunction(const Function &function) const;

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

        // The typed boolean flags parsed from misched.txt. Read e.g.
        // `GetConfig().GetFlags().skip_length_pass`.
        const Flags &GetFlags() const { return flags_; }

        // The typed unscoped global settings parsed from misched.txt. Read e.g.
        // `GetConfig().GetGlobalSettings().unroll_threshold`.
        const GlobalSettings &GetGlobalSettings() const { return global_settings_; }

        // Convenience: return true if post-RA scheduling is disabled
        bool IsPostRASchedulingDisabled() const { return flags_.disable_post_ra_scheduling; }

        // The occupancy.* / length.* scoped "<scope>.<key> = <value>" settings,
        // stored uninterpreted here and given typed meaning by AMDGPU's
        // HierarchicalConfig. (Transitional: these fold into this config's own
        // typed fields in a later step; the bare-flag bools already have.)
        const std::map<std::string, std::map<std::string, std::string>> &
        GetAllScopedSettings() const { return scoped_; }

        // Debug printing stuff
        void DebugPrint() const;
        std::string ToString() const;

        //========================================================================================
        // jbaile
        //========================================================================================
        // Return the configured scheduler as a string (e.g. "MaxIlp").
        std::string GetSchedulerAsString() const;
        //========================================================================================

    private:
        MachineInstrSchedulerConfig();

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

        // Parse one whitespace-delimited option token from a non-`kernel` line.
        // A bare token is a scheduler name or a boolean flag (a Flags field). A
        // `<key>=<value>` token is a setting: a scoped `<scope>.<key>=<value>`
        // (occupancy.* / length.*) is stored uninterpreted for
        // HierarchicalConfig, while an unscoped global setting (e.g. an unroll
        // knob like `unroll_threshold`) is applied to its own typed field.
        // An unknown or malformed token is a fatal error.
        void ParseOptionToken(const std::string &token);

        // Set the scheduler from a bare scheduler-name token; fatal on a
        // second, conflicting name. Returns false if `tok` is not a scheduler
        // name.
        bool TrySetSchedulerFromToken(llvm::StringRef tok);

        // Apply a `<key>=<value>` setting: an unscoped key to its typed global
        // field (via SetGlobalIntSettingIfKnown), a scoped `<scope>.<subkey>` to
        // the uninterpreted scoped store.
        void ApplySetting(llvm::StringRef key, llvm::StringRef value);

        // If `name` is a known flag spelling, set that flag and return true;
        // otherwise return false. Defined over kFlagBindings (in the .cpp), the
        // single list of valid flags.
        bool SetFlagIfKnown(llvm::StringRef name);

        // If `key` is a known unscoped global setting, parse `value` into its
        // typed GlobalSettings field and return true; otherwise return false.
        // Defined over kGlobalIntSettingBindings (in the .cpp), the single list of
        // valid global settings. Fatal on a value that is not a non-negative int.
        bool SetGlobalIntSettingIfKnown(llvm::StringRef key, llvm::StringRef value);

        // If `key` is a known unscoped string-valued global setting, store
        // `value` in its typed GlobalSettings field and return true; otherwise
        // return false. Defined over kGlobalStringSettingBindings (in the .cpp). The
        // value is stored uninterpreted -- its validity is the consumer's job
        // (e.g. AMDGPU rejects an unknown timing_model name).
        bool SetGlobalStringSettingIfKnown(llvm::StringRef key, llvm::StringRef value);

        // Parse a `kernel <m|d>/<signature>/ ...` per-function line (the text
        // after the leading `kernel` keyword).
        void ParseKernelLine(const std::string &rest);

        // Map a scheduler-name token to the Scheduler enum, or
        // Scheduler::InvalidOption if it is not a scheduler name.
        static Scheduler GetSchedulerFromName(llvm::StringRef name);

        bool has_config_ = false;
        Scheduler mi_scheduler_ = Scheduler::Default;
        bool scheduler_set_ = false;
        std::unordered_map<std::string, FunctionConfig> demangled_func_signature_to_config_;

        // Typed boolean flags (the spelling -> field table is in the .cpp).
        Flags flags_;

        // Typed unscoped global settings (the spelling -> field table is in the
        // .cpp). Each field is unset until misched.txt provides a value.
        GlobalSettings global_settings_;

        // Transitional store for occupancy.* / length.* scoped settings, read
        // by HierarchicalConfig. Folded into typed fields in a later step.
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


        MachineInstrSchedulerConfig(const MachineInstrSchedulerConfig &) = delete;
        void operator=(const MachineInstrSchedulerConfig &) = delete;
};

} // end namespace llvm

#endif // LLVM_CODEGEN_MACHINE_INSTR_SCHEDULER_CONFIG_H
