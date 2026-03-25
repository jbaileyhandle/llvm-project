# CLAUDE.md — Project Instructions

## Coding Style

- Follow the Google C++ Style Guide (https://google.github.io/styleguide/cppguide.html)
  when it does not contradict existing LLVM names or conventions.
- When writing new code modeled after existing LLVM code, follow the style rules
  in this file, not the style of the code being referenced.
- Always use `{}` for if/else/while/for blocks, even for single-line bodies.
- Naming conventions (Google style, except where LLVM names must be matched):
  - Variables and function parameters: `snake_case` (e.g., `region_count`)
  - Local variables: `snake_case`
  - Class data members: `snake_case_` with trailing underscore (e.g., `region_count_`)
  - Functions and methods: `PascalCase` (e.g., `FindOptimalSchedule`)
  - Classes and structs: `PascalCase` (e.g., `ScheduleDAGHierarchicalScheduler`)
  - Constants and enumerators: `kPascalCase` (e.g., `kMaxRegions`)
  - Namespaces: `snake_case`
  - When overriding or calling existing LLVM APIs, match LLVM's naming for those
    symbols (e.g., `schedule()`, `finalizeSchedule()`, `NumRegionInstrs`).
- Do not use `unsigned` types to indicate a value is non-negative — use `int`.
  Use `unsigned` only for bit manipulation or when required by an existing API
  (e.g., LLVM interfaces that return `unsigned`). This avoids subtle bugs from
  signed/unsigned implicit conversions.
- Mark all changes to pre-existing LLVM files (files not created by jbaile) with markers:
  ```
  //========================================================================================
  // jbaile
  //========================================================================================
  ... new code ...
  //========================================================================================
  ```

## Preferences

- When writing new code, explain what each part does and why.
- Don't proceed to the next step without confirmation when the user is reviewing.
- Cite sources when making claims about conventions or best practices.
- Do not add "Co-Authored-By: Claude" or similar to commit messages. It is not
  informative and creates clutter.

## Build

- This is a custom LLVM fork with HIP/ROCm support targeting AMDGPU (gfx906).
- The build directory is not in the source tree — check before assuming build paths.
