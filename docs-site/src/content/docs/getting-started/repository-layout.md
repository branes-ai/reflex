---
title: Repository Layout
description: Where things live in the reflex repository.
---

```text
reflex/
├── .github/workflows/    # CI, docs deployment, release-please, commitlint
├── bench/                # Micro-benchmarks (per-update latency); not CI-gated
├── cmake/                # deps.cmake (Catch2), compile_all.cmake, warnings.cmake
├── docs/                 # Design notes, assessments, session records
├── docs-site/            # This site: Starlight + Doxygen API reference
├── scripts/              # Developer and CI helper scripts
├── sdk/
│   └── include/branes/reflex/
│       └── control/      # Controllers (pid.hpp, ...)
├── tests/
│   └── sdk/              # Catch2 tests, one executable per file
├── tools/                # Host-side developer tools (simulation, tuning, plotting)
├── CMakeLists.txt
└── CMakePresets.json
```

The public API is everything under `sdk/include/branes/reflex/`, in the
`branes::reflex` namespace, exposed through the `branes::reflex` CMake target.
