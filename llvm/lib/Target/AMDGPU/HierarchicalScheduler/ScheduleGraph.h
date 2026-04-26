//===- ScheduleGraph.h - Schedule graph -------------------------*- C++ -*-===//
//
// Defines ScheduleNode and ScheduleGraph, the data structures for
// scheduling an AMDGPU region.
//
// A ScheduleNode is one of:
//   - A scheduling unit (IsSchedulingUnit() == true): wraps a
//     single SUnit — a real MachineInstr-backed instruction or a
//     synthetic entry/exit sentinel.
//   - A subgraph START proxy (IsSubgraphStartProxy() == true): owns
//     a SubgraphInfo. Pushes a scope when scheduled.
//   - A subgraph END proxy (IsSubgraphEndProxy() == true): holds a
//     raw back-pointer to the same SubgraphInfo. Pops the scope
//     when scheduled.
// IsSubgraphProxy() returns true for either start or end — useful
// for code (trackers, etc.) that should self-skip on any proxy.
//
// A ScheduleGraph is a FLAT collection of ScheduleNodes with edges
// between them. There is no nested-ScheduleGraph hierarchy: under
// the single-graph-hybrid design (Approach B,
// AMDGPUClusteringDesign.md), proxies and their subgraph members
// coexist in the same graph. Each subgraph contributes a start
// proxy on the predecessor boundary and an end proxy on the
// successor boundary; the original member-to-member and
// member-to-external edges are kept intact (the new artificials
// are added on top so the original real edges keep their data
// latencies for the length tracker).
//
// Example layout: subgraph S = {B, C}, external pred A, external
// succ D. The graph holds A, B, C, D, P_start, P_end:
//
//   A ───────► B       (original)
//   A ───────► C       (original)
//   A ·······► P_start (artificial, kSubgraphOrderEdge)
//   P_start ·► B       (artificial, latency 0)
//   P_start ·► C       (artificial, latency 0)
//   B ───────► C       (original, intra-subgraph)
//   B ·······► P_end   (artificial, kSubgraphOrderEdge)
//   C ·······► P_end   (artificial, kSubgraphOrderEdge)
//   C ───────► D       (original, member → external)
//   P_end ···► D       (artificial, kSubgraphOrderEdge)
//
// Members B and C each gain one extra predecessor (the artificial
// P_start → M edge) which keeps them out of any ready list until
// P_start is scheduled. P_end is gated on every member having been
// scheduled. External successor D is gated both by the real
// member→D edge AND by the artificial P_end→D edge, so it can't
// become ready until the subgraph has fully exited.
//
// Node-storage discipline. ScheduleEdge stores raw `ScheduleNode *`
// pointers into the `nodes_` SmallVector. A vector reallocation
// would invalidate every stored edge pointer (silent UB). Two
// mechanisms keep this safe:
//   1. BuildFromSUnits reserves `3 * sunits.size() + 2` slots up
//      front — enough headroom to add up to two proxies per real
//      instruction (worst case: every instruction is its own
//      subgraph) plus the entry/exit sentinels, all without
//      reallocating.
//   2. EmplaceNode hard-fails with report_fatal_error if it would
//      ever cause `nodes_` to grow past its reserved capacity.
// Together these let post-construction passes (e.g.,
// InsertSubgraphProxies for Approach B) add proxy nodes after the
// graph has edges, without invalidating any edge pointers.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULEGRAPH_H
#define LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULEGRAPH_H

#include "SubgraphInfo.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/Register.h"
#include "llvm/MC/LaneBitmask.h"
#include "llvm/CodeGen/ScheduleDAG.h"
#include "llvm/CodeGen/SlotIndexes.h"
#include <cassert>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace llvm {

class GCNSubtarget;
class LiveIntervals;
class MachineFunction;
class MachineRegisterInfo;

namespace hierarchical_scheduler {

class DominatorTree;
class RegionInfo;
class ScheduleConstructor;
class ScheduleGraph;
class ScheduleNode;
struct ReducedGraph;

/// A register paired with the lane mask indicating which sub-register
/// lanes are relevant. Used on entry/exit nodes where LiveIntervals
/// tells us exactly which lanes are live-in or live-out.
struct RegWithLaneMask {
  Register reg;
  LaneBitmask mask;
};

/// An edge between two ScheduleNodes. Stores the full edge kind (mirroring
/// LLVM's SDep::Kind and SDep::OrderKind) so no information is lost during
/// translation. Classification helpers (IsDataEdge, IsStrongEdge, etc.)
/// provide the coarser categories needed by scheduling algorithms.
struct ScheduleEdge {
  // Enum ordering matters: range checks below assume
  //   [kData .. kArtificial]      = strong AND latency-contributing
  //   kSubgraphOrderEdge          = strong but NOT latency-contributing
  //   [kCluster .. kWeak]         = weak
  // Keep new strong-but-non-latency kinds sandwiched between kArtificial
  // and kCluster to preserve the cheap comparisons.
  enum Kind {
    // Register dependencies (from SDep::Kind).
    kData,         // True data dependence (RAW). One node produces a value
                   // that the other consumes. Carries latency.
    kAnti,         // Anti-dependence (WAR). One node reads a register before
                   // the other writes it. Pre-RA: physical registers only.
    kOutput,       // Output dependence (WAW). Both nodes write the same
                   // register.

    // Strong ordering constraints (from SDep::Order + SDep::OrderKind).
    // These block readiness: a node cannot be scheduled until all strong
    // predecessors have been scheduled.
    kBarrier,      // Scheduling barrier (calls, inline asm, unmodeled
                   // side effects). Everything above must come before,
                   // everything below must come after.
    kMayAliasMem,  // Memory operations that might alias. Conservative
                   // ordering when alias analysis cannot prove independence.
    kMustAliasMem, // Memory operations that definitely alias. Strict
                   // ordering to preserve memory semantics.
    kArtificial,   // Arbitrary strong edge, typically added by DAG mutations
                   // (e.g., to prevent reordering within a cluster).

    //========================================================================
    // End of latency
    //========================================================================

    // Subgraph proxy wiring. Strong (blocks readiness — a subgraph member
    // cannot be scheduled until its proxy enters scope), but does NOT
    // contribute to schedule-length / critical-path because the proxy
    // itself does not issue. Reserved for future hierarchical scheduling;
    // not produced anywhere yet.
    kSubgraphOrderEdge,

    //========================================================================
    // End of strong
    //========================================================================

    // Weak ordering constraints. These are scheduling hints that do NOT
    // block readiness. The scheduler may violate them if doing so produces
    // a better schedule.
    kCluster,      // Clustering hint: prefer scheduling these nodes
                   // adjacently so the hardware can merge memory operations.
    kWeak,         // Arbitrary weak hint.
  };

  ScheduleNode *node_; // The connected node (predecessor or successor).
  Kind kind_;
  int latency_; // Latency in cycles. Only meaningful for kData edges.

  ScheduleEdge(ScheduleNode *node, Kind kind, int latency = 0)
      : node_(node), kind_(kind), latency_(latency) {}

  // Classification helpers. Member functions defined in the class body are
  // implicitly inline per the C++ standard, so these have zero call overhead.

  /// True for register data dependencies (RAW). Raw kind classifier
  /// only — for "does this edge count toward schedule length / critical
  /// path", use IsLatencyEdge instead.
  bool IsDataEdge() const { return kind_ == kData; }

  /// True for all edges that block readiness: register dependencies
  /// (Data, Anti, Output), strong ordering constraints (Barrier,
  /// MayAliasMem, MustAliasMem, Artificial), and subgraph proxy
  /// wiring (kSubgraphOrderEdge).
  bool IsStrongEdge() const { return kind_ <= kSubgraphOrderEdge; }

  /// True for edges that are hints only and do not block readiness.
  bool IsWeakEdge() const { return kind_ >= kCluster; }

  /// True for edges that contribute to schedule-length / critical-path
  /// calculations. Single source of truth for ScheduleLengthTracker and
  /// ScheduleGraph::ComputeCriticalPathFromExit.
  ///
  /// At IssueWidth=1 every strong edge forces successor.cycle >=
  /// predecessor.cycle + 1 (no two instructions co-issue), so every
  /// strong edge contributes — except kSubgraphOrderEdge, where the
  /// proxy node itself does not issue and the cycle delta lives
  /// inside the subgraph instead. cp computations weight contributing
  /// edges by max(1, edge.latency_), so model-zero strong edges still
  /// account for the IssueWidth=1 ordering gap.
  bool IsLatencyEdge() const { return kind_ <= kArtificial; }
};

/// A node in a ScheduleGraph. Either a scheduling unit (wrapping a
/// single SUnit) or a subgraph proxy (standing in for a group of
/// other scheduling units — see AMDGPUClusteringDesign.md for the
/// subgraph mechanism). Uses std::variant to make the unit/proxy
/// distinction type-safe — it is impossible to have both an SUnit
/// and a subgraph payload, or neither.
///
/// Note: assert() is compiled out in release builds (NDEBUG is defined),
/// so the GetSUnit check has zero cost in production. It exists only
/// to catch misuse during development and debug builds.
class ScheduleNode {
public:
  /// Create a scheduling-unit node wrapping a single SUnit. The
  /// node's graph-local id is drawn from `top_level_graph`'s counter
  /// — that is, the root of the graph hierarchy this node will live
  /// in. For a flat (non-nested) graph this is just the graph being
  /// built. `top_level_graph` must not be null; `su` may be null (see
  /// the debug-name overload below).
  ScheduleNode(SUnit *su, ScheduleGraph *top_level_graph);

  /// Create a scheduling-unit node with a debug name (for test DAGs
  /// without real SUnits). `top_level_graph` must not be null; `su`
  /// may be null.
  ScheduleNode(SUnit *su, std::string debug_name,
               ScheduleGraph *top_level_graph);

  /// Create the START subgraph-proxy node owning a SubgraphInfo.
  /// The SubgraphInfo holds the subgraph's members, debug name, and
  /// external predecessors/successors; the start proxy is the
  /// boundary marker on the predecessor side (it pushes a scope
  /// when scheduled). Ownership of `info` transfers into this node;
  /// the info's lifetime then equals this node's lifetime, which
  /// equals the graph's lifetime. Both `info` and `top_level_graph`
  /// must be non-null.
  ScheduleNode(std::unique_ptr<SubgraphInfo> info,
               ScheduleGraph *top_level_graph);

  /// Create the END subgraph-proxy node that back-references an
  /// already-existing SubgraphInfo. The end proxy is the boundary
  /// marker on the successor side (it pops the scope when
  /// scheduled). It does NOT own `info` — the START proxy does;
  /// the end proxy holds only a raw back-pointer. Both `info` and
  /// `top_level_graph` must be non-null.
  ScheduleNode(SubgraphInfo *info, ScheduleGraph *top_level_graph);

  /// True if this node represents a single scheduling unit (wraps
  /// one SUnit — either a real MachineInstr-backed SUnit or a
  /// synthetic entry/exit sentinel). False if this node is any
  /// kind of subgraph proxy (start or end).
  ///
  /// Partitions every ScheduleNode exactly once, together with
  /// IsSubgraphProxy. Used by Schedule dispatch, trackers, and
  /// graph-level unit counting.
  bool IsSchedulingUnit() const {
    return std::holds_alternative<SUnit *>(content_);
  }

  /// True if this node is a subgraph proxy of either flavor
  /// (start or end). Inverse of IsSchedulingUnit. Useful for
  /// trackers / pressure / length code that should self-skip on
  /// any proxy regardless of which boundary it marks.
  bool IsSubgraphProxy() const { return !IsSchedulingUnit(); }

  /// True if this node is the START proxy of a subgraph (owns the
  /// SubgraphInfo, gates entry into the subgraph scope).
  bool IsSubgraphStartProxy() const {
    return std::holds_alternative<std::unique_ptr<SubgraphInfo>>(content_);
  }

  /// True if this node is the END proxy of a subgraph (raw
  /// back-pointer to the SubgraphInfo, gates exit from the
  /// subgraph scope).
  bool IsSubgraphEndProxy() const {
    return std::holds_alternative<SubgraphInfo *>(content_);
  }

  /// Access the wrapped SUnit. Only valid for scheduling-unit nodes.
  SUnit *GetSUnit() const {
    assert(IsSchedulingUnit() && "GetSUnit called on subgraph proxy");
    return std::get<SUnit *>(content_);
  }

  /// Access the SubgraphInfo associated with this proxy node. Works
  /// for either the start proxy (returns the owned info) or the end
  /// proxy (returns the back-referenced info). Reports fatal error
  /// if called on a scheduling-unit node.
  SubgraphInfo *GetSubgraphInfo() const;

  /// The subgraph proxy this node belongs to (the visibility-rule
  /// key for scope-stacked scheduling). Null if this node lives at
  /// the top level of the graph hierarchy. Populated by
  /// InsertSubgraphProxies for each subgraph's members.
  ScheduleNode *GetParentSubgraphProxy() const {
    return parent_subgraph_proxy_;
  }
  void SetParentSubgraphProxy(ScheduleNode *proxy) {
    parent_subgraph_proxy_ = proxy;
  }

  /// Read-only edge access.
  ArrayRef<ScheduleEdge> Successors() const { return successors_; }
  ArrayRef<ScheduleEdge> Predecessors() const { return predecessors_; }

  int NumPredecessors() const { return static_cast<int>(predecessors_.size()); }
  int NumSuccessors() const { return static_cast<int>(successors_.size()); }

  /// Sort successor edges by ascending topo index of the destination node.
  /// Called by ComputeTopologicalOrder after topo indices are assigned.
  /// Required by the transitive reduction algorithm (process closest
  /// successors first).
  void SortSuccessorsByTopoIndex() {
    llvm::sort(successors_, [](const ScheduleEdge &a, const ScheduleEdge &b) {
      return a.node_->GetTopoIndex() < b.node_->GetTopoIndex();
    });
  }

  /// Topological index. Set by ScheduleGraph::ComputeTopologicalOrder().
  /// -1 means not yet computed.
  int GetTopoIndex() const { return topo_index_; }
  void SetTopoIndex(int index) { topo_index_ = index; }

  /// Universally unique ID across all ScheduleNode and ScheduleGraph
  /// instances. Assigned from a shared counter at construction time.
  int64_t GetId() const { return id_; }

  /// Graph-local ID, dense in [0, top_level_graph.GetNumGraphLocalIds()).
  /// Drawn from the top-level graph's counter at construction time.
  /// Suitable as a vector index for per-node precomputations
  /// (critical-path-from-exit, ALAP/ASAP, etc.) — unlike GetId(),
  /// values do not skip ahead between regions/graphs.
  int GetGraphLocalId() const { return graph_local_id_; }

  /// Human-readable description of this node. Format:
  ///   Scheduling unit (instr):    "[3] S_LOAD_DWORD ..."
  ///   Scheduling unit (debug):    "[7:A]"
  ///   Scheduling unit (no SU):    "[7]"
  ///   Subgraph proxy:             "[12:proxy(name,5)]"
  ///                               (members.size() = 5)
  std::string ToString() const;

  /// Registers defined and used by this node, with lane masks
  /// indicating which sub-register lanes are affected. Populated
  /// during graph construction from MachineInstr (scheduling-unit
  /// nodes via ExtractRegInfo — full mask), LiveIntervals (entry/exit
  /// nodes — per-lane mask), or subgraph boundaries (subgraph
  /// proxies, future Phase 1).
  ArrayRef<RegWithLaneMask> RegDefs() const { return reg_defs_; }
  ArrayRef<RegWithLaneMask> RegUses() const { return reg_uses_; }
  void AddRegDef(Register reg, LaneBitmask mask) {
    reg_defs_.push_back({reg, mask});
  }
  void AddRegUse(Register reg, LaneBitmask mask) {
    reg_uses_.push_back({reg, mask});
  }

  /// Extract virtual register defs/uses from this node's MachineInstr
  /// and store them in reg_defs_/reg_uses_. Skips physical registers and
  /// undef uses. No-op for nodes without a MachineInstr.
  void ExtractRegInfo();

private:
  // ScheduleGraph is the only allowed mutator of edge state — it routes
  // edge addition through ScheduleGraph::AddEdge so that derived caches
  // (topo, cp, dom, reduced) get invalidated automatically. Making
  // AddSuccessor private + friending ScheduleGraph enforces that physically.
  friend class ScheduleGraph;

  /// Add a successor edge from this node to the target node. Automatically
  /// adds the matching predecessor edge on the target, ensuring edges are
  /// always symmetric. This works because C++ access control is per-class,
  /// not per-instance — a ScheduleNode method can access private members
  /// of any other ScheduleNode.
  ///
  /// Private — callers go through ScheduleGraph::AddEdge so cache
  /// invalidation is centralized.
  void AddSuccessor(ScheduleEdge edge) {
    successors_.push_back(edge);
    edge.node_->predecessors_.push_back(
        ScheduleEdge(this, edge.kind_, edge.latency_));
  }

  int64_t id_;
  int graph_local_id_;
  // Three alternatives, one per node kind:
  //   index 0 (SUnit *):                   scheduling-unit node
  //   index 1 (unique_ptr<SubgraphInfo>):  start proxy (owns info)
  //   index 2 (SubgraphInfo *):            end proxy (back-references info)
  // The variant index IS the discriminator — see IsSchedulingUnit /
  // IsSubgraphStartProxy / IsSubgraphEndProxy.
  std::variant<SUnit *, std::unique_ptr<SubgraphInfo>, SubgraphInfo *> content_;
  SmallVector<ScheduleEdge> successors_;
  SmallVector<ScheduleEdge> predecessors_;
  SmallVector<RegWithLaneMask> reg_defs_;
  SmallVector<RegWithLaneMask> reg_uses_;
  int topo_index_ = -1;
  std::string debug_name_;

  /// The proxy of the subgraph that contains this node as a direct
  /// member, or null if this node sits at the top level of the
  /// graph hierarchy. Acts as the visibility-rule key for
  /// scope-stacked scheduling: this node is visible at scope `S`
  /// iff this field equals `S.subgraph_proxy`. Populated by
  /// InsertSubgraphProxies; see GetParentSubgraphProxy /
  /// SetParentSubgraphProxy.
  ScheduleNode *parent_subgraph_proxy_ = nullptr;
};

/// A graph of ScheduleNodes. Used at every level of the hierarchy:
///   - Leaf level: each node wraps an SUnit (instruction)
///   - Quotient levels: each node is a group containing a subgraph
///
/// Construction is two-phase: add all nodes first, then add edges. This
/// ensures node pointers (stored in edges) remain stable.
class ScheduleGraph {
public:
  ScheduleGraph();

  // Destructor defined in .cpp because unique_ptr<DominatorTree> needs
  // the full DominatorTree definition to destroy, which is only available
  // in the .cpp (forward-declared here to avoid circular includes).
  ~ScheduleGraph();

  // ScheduleGraph is non-copyable AND non-movable. The self-referential
  // top_level_ pointer cannot be safely copied or moved without custom
  // logic that's a footgun to maintain (every new member would need to
  // be remembered in the move op). Factories return std::unique_ptr,
  // which keeps the graph at a fixed heap address — so the value of
  // top_level_ stays valid for the graph's entire lifetime regardless
  // of how the unique_ptr is moved around.
  ScheduleGraph(const ScheduleGraph &) = delete;
  ScheduleGraph &operator=(const ScheduleGraph &) = delete;
  ScheduleGraph(ScheduleGraph &&) = delete;
  ScheduleGraph &operator=(ScheduleGraph &&) = delete;

  int64_t GetId() const { return id_; }

  /// Graph-local ID for this graph itself, drawn from the top-level
  /// graph's counter. For a top-level graph (no parent) this is always
  /// 0 — the graph self-assigns id 0 by calling its own counter, then
  /// nodes added to it take 1..N. For a subgraph (future), the counter
  /// is forwarded to the top-level parent so all ids in the hierarchy
  /// are drawn from one dense space.
  int GetGraphLocalId() const { return graph_local_id_; }

  /// Hand out the next graph-local id and increment the counter.
  /// Forwards to the top-level parent so subgraph nodes share the
  /// top-level counter (currently top_level_ is always `this` since
  /// subgraph nesting isn't wired yet). Called by ScheduleNode and
  /// ScheduleGraph constructors.
  int GetAndIncrementGraphLocalId() {
    return top_level_->next_graph_local_id_++;
  }

  /// Total number of graph-local ids handed out so far on this
  /// hierarchy's counter (== top_level_->next_graph_local_id_).
  /// Useful as the size for per-node vectors indexed by
  /// GetGraphLocalId().
  int GetNumGraphLocalIds() const {
    return top_level_->next_graph_local_id_;
  }

  /// Build a leaf-level graph by copying the dependency structure from an
  /// existing SUnit DAG. Each SUnit becomes a leaf ScheduleNode, and SDep
  /// edges are translated to ScheduleEdges.
  ///
  /// Creates our own entry and exit nodes rather than using LLVM's
  /// EntrySU/ExitSU. The entry node defines all live-in registers and
  /// has edges to all root nodes (no predecessors among real instructions).
  /// The exit node uses all live-out registers and has edges from all
  /// leaf nodes (no successors among real instructions).
  ///
  /// Live-in/live-out information comes from LiveIntervals. Register
  /// defs/uses are extracted from MachineInstrs and stored on each node
  /// for use by the RegisterTracker.
  static std::unique_ptr<ScheduleGraph> BuildFromSUnits(
      MutableArrayRef<SUnit> sunits,
      const GCNSubtarget &st,
      const MachineFunction &mf,
      const LiveIntervals &lis,
      const MachineRegisterInfo &mri,
      const RegionInfo &region);

  /// Build a synthetic test DAG with known structure for testing algorithms
  /// like topological sort, transitive reduction, dominator trees, and
  /// critical-path-from-exit. Does not depend on LLVM SUnits — nodes are
  /// leaves wrapping nullptr.
  ///
  /// 7 nodes (A-H, no B), 9 edges (all kData):
  ///
  ///              A
  ///            / | \
  ///           /  |  \
  ///          /   |   \
  ///         v    |    v
  ///        H     |     C
  ///        |     |   /  \
  ///        |     v  v    v
  ///        |      D      E
  ///        |       \    /
  ///        |        v  v
  ///        |         F
  ///         \       /
  ///          \     /
  ///           v   v
  ///             G
  ///
  /// Edge latencies:
  ///   A→H=1  A→D=3  A→C=2
  ///   C→D=1  C→E=4
  ///   D→F=2  E→F=1
  ///   H→G=5  F→G=2
  ///
  /// Expected cp_from_exit values (longest latency-weighted path to G):
  ///   G=0, H=5, F=2, E=3, D=4, C=7, A=9
  /// Critical path: A→C→E→F→G (length 2+4+1+2 = 9).
  static std::unique_ptr<ScheduleGraph> BuildTestDAG();

  /// Build a synthetic test DAG specifically for exercising
  /// GetLengthLowerBound. Shape and latencies chosen so the LB
  /// transitions several times through the forward schedule — both
  /// from the first term (bubbles advancing current_cycle past the
  /// node-count floor) and from the second term (new max of
  /// scheduled_cycle + cp_from_exit).
  ///
  /// 6 nodes, 6 edges:
  ///
  ///   N0 --(3)--> N1 --(5)--> N3 --(1)--> N5
  ///    \                                   ^
  ///     \-(1)--> N2 --(1)--> N4 --(2)------/
  ///
  /// Expected cp_from_exit: N0=9, N1=6, N2=3, N3=1, N4=2, N5=0.
  /// Topo order (Kahn's): N0, N1, N2, N3, N4, N5.
  /// Expected LB after scheduling k nodes (k=0..6):
  ///   {6, 10, 10, 10, 11, 12, 12}
  /// Transitions: 6->10 (2nd term kicks in via N0, contrib=0+9+1=10),
  /// 10->11 (1st term overtakes due to bubble at N3, cc=9 + 2 left),
  /// 11->12 (2nd term jumps via N4, contrib=9+2+1=12).
  static std::unique_ptr<ScheduleGraph> BuildLengthLowerBoundTestDAG();

  /// Build a synthetic test DAG that contains a cycle, for testing that
  /// ComputeTopologicalOrder correctly detects it and calls
  /// report_fatal_error.
  ///
  /// Structure (3 nodes, 3 edges — cycle between B and C):
  ///
  ///     A
  ///     |
  ///     v
  ///     B <---.
  ///     |     |
  ///     v     |
  ///     C ----'
  ///
  /// Edges: A→B, B→C, C→B
  static std::unique_ptr<ScheduleGraph> BuildTestDAGWithCycle();

  /// Build a synthetic test DAG used by RunSubgraphContiguityShakedown
  /// to demonstrate that wrapping {A, B, C} as a subgraph forces the
  /// chain to be scheduled contiguously, even when an unconstrained
  /// scheduler would have interleaved X and Y between A and B to fill
  /// the latency bubbles.
  ///
  /// Structure (6 nodes — E is a synthetic exit so the graph has a
  /// single source A and single sink E, satisfying
  /// ValidateAndComputeTopologicalOrder's invariants):
  ///
  ///     A ──0──► X ──0──► E
  ///     A ──0──► Y ──0──► E
  ///     A ──5──► B ──5──► C ──0──► E
  ///
  /// In the UNCLUSTERED graph, X and Y depend only on A. After A is
  /// scheduled, they are immediately ready; a topo-asc picker
  /// schedules them while B's ready cycle is still in the future —
  /// filling the bubble. Resulting length: 12 cycles.
  ///
  /// In the CLUSTERED graph (after wrapping {A, B, C} as a subgraph),
  /// X, Y, and E become external successors of the subgraph (E via
  /// C→E), so InsertSubgraphProxies adds artificial end_proxy → X,
  /// end_proxy → Y, end_proxy → E edges. Topology then FORCES X and
  /// Y to come after end_proxy in any valid topo order — they
  /// cannot be scheduled until the chain has fully exited. Resulting
  /// length: 14 cycles. The 2-cycle delta is the bubble that X and Y
  /// filled in the unclustered case.
  ///
  /// The X-vs-chain ordering is structurally forced (by the new
  /// artificial edges), not dependent on Kahn's-FIFO seed order,
  /// so the test outcome is deterministic regardless of internal
  /// topo-sort details.
  static std::unique_ptr<ScheduleGraph> BuildContiguityTestDAG();

  /// Human-readable identifier for this graph. Format: "graph[ID]"
  std::string ToString() const;

  /// Access the node list.
  MutableArrayRef<ScheduleNode> Nodes() { return nodes_; }
  ArrayRef<ScheduleNode> Nodes() const { return nodes_; }
  int Size() const { return static_cast<int>(nodes_.size()); }

  /// Total number of scheduling-unit nodes in this graph (real +
  /// entry/exit sentinels; excludes subgraph proxies). Phase 0: no
  /// proxies exist, so this equals Size(). Lazily counted; cached.
  int NumSchedulingUnits() const;

  /// Reserve capacity in nodes_ sized for a graph that starts with
  /// `initial_count` nodes and may subsequently have up to
  /// `initial_count` subgraphs inserted, where each subgraph adds
  /// TWO proxy nodes (start and end). Plus 2 entry/exit sentinels.
  /// Total: initial_count + 2*initial_count + 2 = 3*initial_count + 2.
  /// This is the headroom InsertSubgraphProxies needs to emplace
  /// proxies without reallocating nodes_ — and a reallocation would
  /// invalidate every ScheduleNode * stored in any ScheduleEdge.
  /// Every graph builder (BuildFromSUnits + the test-DAG factories)
  /// calls this exactly once, before emplacing any nodes.
  void ReserveNodes(int initial_count) {
    nodes_.reserve(initial_count * 3 + 2);
  }

  /// Add an edge from `from` to `to`. The graph routes all edge
  /// additions through here so that derived caches (topo, cp,
  /// reduced, dom, ...) are invalidated automatically — see
  /// InvalidateDerivedData. Callers should NOT call ScheduleNode
  /// methods to add edges directly; that path is private.
  void AddEdge(ScheduleNode *from, ScheduleNode *to,
               ScheduleEdge::Kind kind, int latency = 0) {
    from->AddSuccessor(ScheduleEdge(to, kind, latency));
    InvalidateDerivedData();
  }

  /// Add subgraph-proxy nodes to this graph, TWO per SubgraphInfo
  /// in `infos` (start proxy + end proxy). For each SubgraphInfo:
  ///   1. Emplace the START proxy node owning the SubgraphInfo
  ///      (ownership of the unique_ptr transfers in here).
  ///   2. Set the SubgraphInfo's subgraph_proxy backpointer (= start).
  ///   3. Set parent_subgraph_proxy on each member (was nullptr
  ///      → now the start proxy).
  ///   4. Add a kSubgraphOrderEdge from each ext_predecessor to the
  ///      start proxy (gates start readiness on externals).
  ///   5. Add a kSubgraphOrderEdge from the start proxy to each
  ///      member (gates each member's readiness on start).
  ///   6. Emplace the END proxy node, holding a raw back-pointer to
  ///      the SubgraphInfo (ownership stays with start). Set
  ///      end_proxy's parent_subgraph_proxy to the start proxy so
  ///      the end proxy lives INSIDE the subgraph scope.
  ///   7. Set the SubgraphInfo's end_proxy backpointer.
  ///   8. Add a kSubgraphOrderEdge from each member to the end
  ///      proxy (gates end readiness on every member having been
  ///      scheduled).
  ///   9. Add a kSubgraphOrderEdge from the end proxy to each
  ///      ext_successor (gates external successors on subgraph exit).
  ///
  /// `infos` is consumed: each unique_ptr is moved into a start
  /// proxy node. Original member-to-member and member-to-external
  /// edges are NOT modified — they keep their data latencies for
  /// the length tracker; the new artificials are purely additive.
  ///
  /// Preconditions are checked internally with report_fatal_error
  /// (Phase 1b — flat two-level only):
  ///   - No node appears as a member of more than one SubgraphInfo.
  ///   - Every member is a scheduling unit (no nesting — members
  ///     are not themselves subgraph proxies).
  ///
  /// On return, the graph is in a queryable state for the
  /// scheduler:
  ///   - Topological order is recomputed. Its cycle check IS the
  ///     re-entrancy detector for the just-inserted subgraphs:
  ///     the artificial edges close any path through an external
  ///     node between two members, so re-entrancy surfaces as a
  ///     graph cycle reported by
  ///     ValidateAndComputeTopologicalOrder.
  ///   - Critical-path-from-exit is recomputed (scheduler
  ///     precondition).
  ///
  /// Reduced graph and dominator tree are NOT recomputed. Those
  /// analyses are inputs to subgraph-formation heuristics; once
  /// subgraphs have been formed and inserted here, the formation
  /// work is done and they hold no further value. (They would
  /// also be measuring a different graph — one with proxy nodes
  /// and artificial edges — so the post-insertion versions
  /// wouldn't match what the formation pass was working with
  /// anyway.)
  ///
  /// No-op if `infos` is empty: no mutation, no recompute.
  void InsertSubgraphProxies(
      std::vector<std::unique_ptr<SubgraphInfo>> infos);

  /// Umbrella entry point: verify single-source / single-sink (under
  /// both the strong-edge and all-edge interpretations, for safety)
  /// and compute topological order. This is the standard "graph
  /// construction is complete, derive what's needed for analysis"
  /// hand-off. Re-runnable if the graph is later mutated (e.g., by
  /// adding subgraph proxy nodes); previously-derived data is
  /// invalidated and recomputed.
  ///
  /// Cycle detection fires first inside ComputeTopologicalOrder; if
  /// the graph contains a cycle, the source/sink checks are skipped
  /// and the cycle is reported instead. report_fatal_error on any
  /// invariant violation.
  void ValidateAndComputeTopologicalOrder(bool include_weak_edges = false);

  /// Access the computed topological order. Only valid after
  /// ComputeTopologicalOrder() has been called.
  ArrayRef<ScheduleNode *> GetTopoOrder() const { return topo_order_; }

  /// Whether ComputeTopologicalOrder() has been called and not been
  /// invalidated by a subsequent graph mutation.
  bool IsTopoSorted() const { return !topo_order_.empty(); }

  /// Compute the longest latency-weighted path from each node to the
  /// exit node, considering only latency-carrying edges (see
  /// ScheduleEdge::IsLatencyEdge). Used for schedule-length lower
  /// bounds during branch-and-bound length search.
  ///
  /// Recurrence (base case: exit has cp_from_exit = 0):
  ///   cp_from_exit[n] = max over latency-edge succs s of
  ///                         (edge.latency + cp_from_exit[s.target])
  ///
  /// Implemented as a single reverse-topological pass — O(V + E).
  /// Requires ComputeTopologicalOrder to have been called first
  /// (fires report_fatal_error otherwise).
  ///
  /// Storage: std::vector<int> indexed by ScheduleNode::GetTopoIndex().
  /// Sized to Size() (one slot per node, no waste).
  void ComputeCriticalPathFromExit();

  /// Whether ComputeCriticalPathFromExit() has been called and not
  /// been invalidated since.
  bool HasCriticalPathFromExit() const {
    return !critical_path_from_exit_by_topo_index_.empty();
  }

  /// Longest latency-weighted path to the exit node, keyed by topo
  /// index. Bare lookup — caller is responsible for ensuring CP has
  /// been computed (consumers like the length pass driver should
  /// report_fatal_error_unless HasCriticalPathFromExit() at entry).
  int GetCriticalPathFromExitByTopoIndex(int topo_idx) const {
    return critical_path_from_exit_by_topo_index_[topo_idx];
  }

  /// Convenience: same as above but extracts the topo index from
  /// the node.
  int GetCriticalPathFromExit(const ScheduleNode *node) const {
    return GetCriticalPathFromExitByTopoIndex(node->GetTopoIndex());
  }

  /// Critical path length of this graph: cp_from_exit at the source
  /// node. Every graph we process has a single source (the synthetic
  /// entry in BuildFromSUnits, the unique root in test DAGs), so this
  /// equals max_n cp_from_exit[n]. Cached during
  /// ComputeCriticalPathFromExit; caller must ensure
  /// HasCriticalPathFromExit().
  int GetCriticalPathLength() const { return *critical_path_length_; }

  /// Lower bound on schedule length implied by graph structure alone:
  ///   max(NumSchedulingUnits, GetCriticalPathLength() + 1)
  /// NumSchedulingUnits covers the IssueWidth=1 floor (one cycle per
  /// unit); CriticalPathLength + 1 covers the latency-chain floor
  /// (the +1 is the cycle the latency-sink itself occupies). For
  /// chain-like graphs CP+1 dominates; for parallel/branchy graphs
  /// NumSchedulingUnits can.
  /// Cached during ComputeCriticalPathFromExit; caller must ensure
  /// HasCriticalPathFromExit().
  int GetGraphLengthFloor() const { return *graph_length_floor_; }

  /// Compute the transitive reduction of this graph. Stores the result
  /// internally. Requires topo sort to have been computed first.
  ///
  /// Uses a single-pass algorithm: iterate nodes in reverse topo order,
  /// process successors in ascending topo order (closest first). If a
  /// successor is already reachable via a previously processed closer
  /// path, the direct edge is redundant and discarded.
  ///
  /// Each edge is visited once. For each kept edge, we OR two BitVectors
  /// of size V (V/64 word operations). For sparse DAGs with E ~ kV edges
  /// (k = average degree), this gives ~kV^2/64 total word operations.
  /// Temporary space: one BitVector per node = V^2/8 bytes total.
  void ComputeTransitiveReduction();

  /// Whether ComputeTransitiveReduction() has been called.
  bool IsReduced() const { return reduced_graph_ != nullptr; }

  /// Access the reduced graph. Only valid after ComputeTransitiveReduction().
  const ReducedGraph &GetReducedGraph() const { return *reduced_graph_; }

  /// Whether ComputeTransitiveReduction() has been called and its
  /// reachability matrix is available. Same lifetime as the reduced
  /// graph itself — both are populated together and cleared together
  /// in InvalidateDerivedData.
  bool HasReachability() const {
    return !reachability_by_topo_index_.empty();
  }

  /// Does the node at topo index `from_topo_idx` reach the node at
  /// topo index `to_topo_idx` along directed edges in the original
  /// (non-reduced) graph? A node always reaches itself.
  ///
  /// Bare lookup; caller is responsible for ensuring HasReachability
  /// (HasReachability() == IsReduced()).
  bool IsReachableInDag(int from_topo_idx, int to_topo_idx) const {
    return reachability_by_topo_index_[from_topo_idx].test(to_topo_idx);
  }

  /// Convenience: same as the topo-index form, but takes node
  /// pointers and unpacks the topo indices internally.
  bool IsReachableInDag(const ScheduleNode *from,
                        const ScheduleNode *to) const {
    return IsReachableInDag(from->GetTopoIndex(), to->GetTopoIndex());
  }

  /// Compute the dominator tree from the transitively reduced graph.
  /// Stores the result internally. Requires transitive reduction to have
  /// been computed first.
  void ComputeDominatorTree();

  /// Whether ComputeDominatorTree() has been called.
  bool HasDominatorTree() const { return dom_tree_ != nullptr; }

  /// Access the dominator tree. Only valid after ComputeDominatorTree().
  const DominatorTree &GetDominatorTree() const { return *dom_tree_; }

  /// Human-readable dump of the dominator tree. Convenience wrapper
  /// that passes this graph to DominatorTree::ToString for node names.
  std::string DominatorTreeToString() const;

  /// Access the input-order ScheduleConstructor — a fully-populated
  /// schedule whose order matches the SUnit order observed at
  /// BuildFromSUnits time (i.e., the MachineFunction's instruction
  /// order at the moment this graph was constructed). Populated by
  /// BuildFromSUnits as Phase 4 of construction.
  ///
  /// Naming note: deliberately "input" rather than "original". Only
  /// the first pass run on a region is guaranteed to see the truly
  /// original GCN scheduler order; subsequent passes may or may not
  /// see a different order, depending on whether earlier passes
  /// actually modified the MachineFunction. "Input" describes the
  /// relationship to the current pass without overclaiming history.
  const ScheduleConstructor &GetInputScheduleConstructor() const {
    return *input_schedule_constructor_;
  }

private:
  /// Compute topological order using Kahn's algorithm (iterative
  /// BFS-based). Populates topo_order_ and sets topo_index_ on each
  /// node. Detects cycles (via Kahn's incomplete-coverage signal) and
  /// fires report_fatal_error if found.
  ///
  /// If include_weak_edges is false, only strong edges constrain the
  /// ordering. If true, weak edges (Cluster, Weak) also count as
  /// predecessors that must be scheduled first.
  ///
  /// Calls InvalidateDerivedData() at entry so re-running with a
  /// different include_weak_edges doesn't leave stale downstream
  /// caches keyed by the previous topo order.
  ///
  /// PRIVATE: external callers go through
  /// ValidateAndComputeTopologicalOrder so structural invariants are
  /// also verified.
  void ComputeTopologicalOrder(bool include_weak_edges);

  int64_t id_;

  /// Root of the hierarchy this graph belongs to. For top-level graphs
  /// this is `this`; for subgraphs (future) it's the ultimate root.
  /// Pointer (not reference) because it must be re-bindable when a
  /// subgraph is attached to a parent.
  ScheduleGraph *top_level_ = this;

  /// Counter that hands out graph-local ids. Only meaningful on the
  /// top-level graph; subgraphs forward to their top-level via
  /// GetAndIncrementGraphLocalId.
  int next_graph_local_id_ = 0;

  /// This graph's own graph-local id. Set in the constructor by
  /// calling GetAndIncrementGraphLocalId on itself, so a top-level
  /// graph always gets id 0.
  int graph_local_id_;

  std::vector<ScheduleNode> nodes_;

  /// Populated by ComputeTopologicalOrder. Empty == "not computed
  /// or invalidated since." IsTopoSorted() = !topo_order_.empty().
  std::vector<ScheduleNode *> topo_order_;

  /// Populated by ComputeCriticalPathFromExit. Indexed by
  /// ScheduleNode::GetTopoIndex(). Empty == "not computed or
  /// invalidated since." Sized to Size() (one slot per node).
  std::vector<int> critical_path_from_exit_by_topo_index_;

  /// Cached max over critical_path_from_exit_by_topo_index_. Populated
  /// at the end of ComputeCriticalPathFromExit; reset in
  /// InvalidateDerivedData.
  std::optional<int> critical_path_length_;

  /// Cached max(NumSchedulingUnits, critical_path_length_ + 1).
  /// Populated at the end of ComputeCriticalPathFromExit; reset in
  /// InvalidateDerivedData.
  std::optional<int> graph_length_floor_;

  /// Lazily-computed scheduling-unit count (see NumSchedulingUnits()).
  /// Populated on first call; cleared on any graph mutation via
  /// InvalidateDerivedData. `mutable` so NumSchedulingUnits() can
  /// stay const.
  mutable std::optional<int> cached_num_scheduling_units_;

  std::unique_ptr<ReducedGraph> reduced_graph_;

  /// Per-node DAG-reachability bitvectors, populated at the end of
  /// ComputeTransitiveReduction (the matrix is built there anyway as
  /// a working set for redundancy detection — we just keep it
  /// instead of discarding). reachability_by_topo_index_[i].test(j)
  /// is true iff the node at topo index i can reach the node at
  /// topo index j along directed edges. Outer size == graph size;
  /// each inner BitVector has graph-size bits. Storage cost: V^2/8
  /// bytes per graph (~125 KB at V=1000, ~500 KB at V=2000).
  /// Cleared by InvalidateDerivedData.
  std::vector<BitVector> reachability_by_topo_index_;

  std::unique_ptr<DominatorTree> dom_tree_;
  std::unique_ptr<ScheduleConstructor> input_schedule_constructor_;

  /// Clear all derived caches. Called by every graph-mutating op
  /// (AddEdge, EmplaceNode, future EmplaceSubgraphProxyNode, ...)
  /// and by ComputeTopologicalOrder (since re-running with
  /// different options would invalidate downstream).
  void InvalidateDerivedData();

  /// Emplace a new ScheduleNode into nodes_ and invalidate derived
  /// caches. Variadic forwarder over ScheduleNode's constructors.
  /// Returns a reference to the new node, matching the
  /// std::vector::emplace_back signature.
  ///
  /// CONVENTION: every node-addition site inside ScheduleGraph
  /// methods MUST go through this helper, NOT direct
  /// nodes_.emplace_back. The helper centralizes the
  /// invalidate-on-mutation guarantee. Direct nodes_.emplace_back
  /// is not enforced as private (still accessible to ScheduleGraph
  /// methods) but should be avoided.
  ///
  /// HARD INVARIANT: nodes_ must have spare capacity at the moment
  /// of emplacement. Reallocation would invalidate every
  /// ScheduleNode * stored in any ScheduleEdge anywhere in the
  /// graph (a silent UB hazard). Callers reserve up front via
  /// nodes_.reserve(...). If we'd reallocate here, fail loudly
  /// instead of silently corrupting stored pointers.
  template <typename... Args>
  ScheduleNode &EmplaceNode(Args &&...args) {
    if (nodes_.size() == nodes_.capacity()) {
      report_fatal_error(
          "ScheduleGraph::EmplaceNode would reallocate nodes_ — "
          "caller must reserve capacity before adding to a graph "
          "that has stored ScheduleNode pointers in edges");
    }
    nodes_.emplace_back(std::forward<Args>(args)...);
    InvalidateDerivedData();
    return nodes_.back();
  }

  /// Mutation step for one SubgraphInfo, called by
  /// InsertSubgraphProxies. Emplaces a proxy node owning the
  /// SubgraphInfo, sets backpointer + parent_subgraph_proxy, adds
  /// the kSubgraphOrderEdge artificial edges. See implementation
  /// for the full step list.
  void EmplaceProxyAndWireEdges(std::unique_ptr<SubgraphInfo> info);

  // --- Construction helpers (used by BuildFromSUnits) ---

  /// Phase 1: For each non-boundary SUnit, append a leaf ScheduleNode
  /// to nodes_ (in SUnit iteration order, which is MachineFunction
  /// instruction order) and record the mapping in sunit_to_node.
  void CreateLeafNodesFromSUnits(
      MutableArrayRef<SUnit> sunits,
      DenseMap<const SUnit *, ScheduleNode *> &sunit_to_node);

  /// Phase 2: Translate SDep successor edges into ScheduleEdges
  /// between leaf nodes. Skips edges to/from LLVM's boundary nodes.
  void AddEdgesBetweenLeafNodes(
      const DenseMap<const SUnit *, ScheduleNode *> &sunit_to_node);

  /// Phase 3: Create entry and exit nodes, wire them to root/leaf
  /// nodes, and populate their register defs/uses from LiveIntervals.
  /// Must be called after all real instruction nodes and edges are
  /// added.
  void CreateEntryAndExitNodes(const LiveIntervals &lis,
                               const MachineRegisterInfo &mri,
                               SlotIndex region_begin_idx,
                               SlotIndex region_end_idx);

  /// Phase 4: Construct input_schedule_constructor_ and populate it
  /// by Schedule()-ing each leaf node in nodes_ storage order. Phase
  /// 1's emplacement order matches MF order (since SUnits come in MF
  /// order from buildSchedGraph), so iterating nodes_ directly avoids
  /// needing the sunit_to_node map here. Two monotonicity checks
  /// (ScheduleNode::id_ and SUnit::NodeNum) verify the leaf order
  /// during the replay; the resulting schedule is then cross-checked
  /// against the region's MF iterator range via
  /// VerifyInputScheduleMatchesMFOrder.
  void PopulateInputScheduleConstructor(const GCNSubtarget &st,
                                        const MachineFunction &mf,
                                        const LiveIntervals &lis,
                                        const RegionInfo &region);

  /// Cross-check that input_schedule_constructor_'s schedule order
  /// matches [region.Begin(), region.End()) at the MachineInstr-
  /// pointer level. Stronger evidence than the id_/NodeNum
  /// monotonicity in Phase 4 — those verify structural ordering on
  /// internal counters; this one walks the actual MF range and
  /// compares MachineInstr* directly. Called once from Phase 4
  /// after the replay completes.
  void VerifyInputScheduleMatchesMFOrder(const RegionInfo &region) const;
};

/// Lightweight adjacency-list representation produced by transitive
/// reduction. Indexed by topological index for O(1) access.
///
/// Storage is O(V + E_reduced). The outer std::vector is sized to the
/// number of nodes; each inner SmallVector holds only the edges that
/// survived the reduction (SmallVector because most nodes have few
/// successors, so inline storage avoids per-node heap allocation).
struct ReducedGraph {
  int size;                                          // number of nodes
  std::vector<SmallVector<int>> successors_by_topo_index;
  std::vector<SmallVector<int>> predecessors_by_topo_index;

  /// Construct with a given number of nodes. Allocates empty adjacency
  /// lists of the specified size.
  explicit ReducedGraph(int num_nodes)
      : size(num_nodes),
        successors_by_topo_index(num_nodes),
        predecessors_by_topo_index(num_nodes) {}
};

} // namespace hierarchical_scheduler
} // namespace llvm

#endif // LLVM_LIB_TARGET_AMDGPU_HIERARCHICALSCHEDULER_SCHEDULEGRAPH_H
