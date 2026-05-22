# Copy of the dagP acyclic DAG partitioner source, built as a standalone
# static library that AMDGPUCodeGen links for MinCutFormation. Upstream repo
# + commit are in PROVENANCE.txt; LICENSE is alongside.
#
# Built WITHOUT METIS/Scotch: dagP_METIS / dagP_SCOTCH are left undefined, so
# the undirected-initial-partitioner code paths compile out and the only
# dependency is libm. Kept as its own target (not folded into
# LLVMAMDGPUCodeGen) with warnings suppressed, so LLVM's stricter flags
# (e.g. -Werror) don't apply to this third-party C/C++.

add_library(dagp STATIC
  src/common/dgraph.c
  src/common/dgraphTraversal.c
  src/common/utils.c
  src/common/info.c
  src/common/undirPartitioning.c
  src/common/option.c
  src/common/debug.c
  src/common/clustering.c
  src/common/dgraphReader.c
  src/common/dgraphDotReader.cpp
  src/recBisection/vcycle2way.c
  src/recBisection/initialBisection.c
  src/recBisection/rvcycle.c
  src/recBisection/dgraphBisection.c
  src/recBisection/refinementBis.c
  src/recBisection/dagP.c
)

# dagP's sources include headers as "<name>.h" resolved against these three
# roots (mirrors the SConstruct's -Isrc -Isrc/common -Isrc/recBisection).
# PUBLIC so the dirs propagate to whatever links dagp — MinCutFormation.cpp
# (compiled into AMDGPUCodeGen) includes dagP.h.
target_include_directories(dagp PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/common
  ${CMAKE_CURRENT_SOURCE_DIR}/src/recBisection
)

set_target_properties(dagp PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Third-party C/C++: suppress warnings so LLVM's -W/-Werror flags don't fail
# dagP's build.
target_compile_options(dagp PRIVATE -w)

# dagP calls into libm.
target_link_libraries(dagp PUBLIC m)
