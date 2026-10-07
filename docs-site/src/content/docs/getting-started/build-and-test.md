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

## Adding a test

Drop a `<name>.cpp` Catch2 file into `tests/<layer>/`. It is picked up, built,
and registered with CTest automatically (see `cmake/compile_all.cmake`). Test
and benchmark targets build with strict warnings as errors.
