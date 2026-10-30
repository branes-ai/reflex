---
title: Build & Test
description: Configure, build, and test reflex with CMake presets.
---

Reflex is a header-only C++20 library. The CMake build exists to compile and run
the tests and benchmarks. It requires **CMake 4.0 or newer** and Ninja.

## Linux

```bash
cmake --preset gcc-debug          # or gcc-release, clang-debug, clang-release
cmake --build --preset gcc-debug
ctest --preset gcc-debug
```

Debug presets keep the controllers' contract `assert`s live; Release presets
compile them out and are the ones to use for benchmarks:

```bash
cmake --preset gcc-release && cmake --build --preset gcc-release
./build/gcc-release/bench/pid_bench
```

## Windows (Visual Studio 2022)

Open the folder in Visual Studio and pick the `msvc` preset, or from a
Developer PowerShell:

```powershell
cmake --preset msvc
cmake --build --preset msvc-debug
ctest --preset msvc-debug
```

The CMake bundled with Visual Studio 2022 is 3.x; point Visual Studio at a
CMake 4.x install (Tools → Options → CMake).

## Using reflex from another project

```cmake
include(FetchContent)
FetchContent_Declare(reflex
    GIT_REPOSITORY https://github.com/branes-ai/reflex.git
    GIT_TAG        <release tag>)
FetchContent_MakeAvailable(reflex)  # tests/bench are off when not top-level

target_link_libraries(my_flight_controller PRIVATE branes::reflex)
```

## MTL5 and Universal

Tests and examples use [MTL5](https://github.com/stillwater-sc/mtl5) (linear
algebra) and [Universal](https://github.com/stillwater-sc/universal) (posits,
cfloat, fixed-point). Both are header-only Stillwater sister projects. CMake
uses an installed copy if `find_package` finds one; otherwise it fetches only
their headers at the versions pinned in `cmake/deps.cmake`. Neither is fetched
when tests are off, so projects that consume reflex do not pull them.

To build against local checkouts (no network needed):

```bash
cmake --preset gcc-debug \
  -DFETCHCONTENT_SOURCE_DIR_MTL5=$HOME/dev/stillwater/clones/mtl5 \
  -DFETCHCONTENT_SOURCE_DIR_UNIVERSAL=$HOME/dev/stillwater/clones/universal
```

Their headers are treated as system headers, so their warnings don't trip
reflex's warnings-as-errors build. To see them, for example while fixing
them upstream, add
`-DREFLEX_SHOW_DEP_WARNINGS=ON --compile-no-warning-as-error`.

## Adding a test

Drop a `<name>.cpp` Catch2 file into `tests/<layer>/` (`tests/sdk/` for the
library, `tests/examples/` for end-to-end examples). It is picked up, built,
and registered with CTest automatically (see `cmake/compile_all.cmake`). Test
and benchmark targets build with strict warnings as errors.
