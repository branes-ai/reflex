# ADR 0001 — Third-party license compatibility

**Status:** Accepted
**Date:** 2026-10-31
**Source:** the stack's license posture, set in cortex's [ADR 0001](https://github.com/branes-ai/cortex/blob/main/docs/adr/0001-third-party-license-compatibility.md)
**Related:** [ADR-0002](0002-no-gpl-clean-room-policy.md)

## Context

reflex is MIT-licensed (see the top-level [`LICENSE`](../../LICENSE)), like the rest of the Branes.AI stack. Its controllers, filters and models are meant to be redistributed inside partners' flight software, so every dependency has to be compatible with MIT and carry its notices. cortex records the same posture in its ADR 0001. reflex had followed it without recording it.

## Dependency license matrix

| Dependency | License | Compatible with MIT reflex? | How it is used | Obligation |
|---|---|---|---|---|
| MTL5 | MIT | Yes | FetchContent: the vectors and matrices | Bundle its LICENSE in a distribution |
| Universal | MIT | Yes | FetchContent, tests only: the components are generic and never name a Universal type | Bundle its LICENSE with source |
| mp-dsp | MIT | Yes | FetchContent, tests only: the reference the `dsp/` tests compare against | Bundle its LICENSE with source |
| Catch2 v3 | BSL-1.0 | Yes | FetchContent, tests only | None for object code; bundle with source |
| Emscripten (emsdk) | MIT **or** University of Illinois/NCSA | Yes | Build tool for the WebAssembly module the docs site runs | None for the tool |
| Emscripten's runtime libraries: musl, LLVM libc++, libc++abi and compiler-rt | MIT; Apache-2.0 WITH LLVM-exception | Yes | Linked into the WebAssembly module | The LLVM exception waives the notice requirement for compiled code; the musl notice goes with a distribution of the module |

The site's trace player (`packages/trace-player`) is reflex's own code, MIT-licensed.

No GPL, LGPL, AGPL, MPL or SSPL dependency is accepted. [ADR-0002](0002-no-gpl-clean-room-policy.md) is the wider policy: no GPL code for any purpose, including the flight stacks most often used as references.

## Decision

1. **reflex stays MIT-licensed.** Every dependency must be compatible with MIT.
2. **A PR that adds a dependency must add it to this table**, with its license and its obligation. A dependency that is dual-licensed is used under its permissive branch.
3. **Any file of someone else's work committed to reflex** (copied, ported or generated from a third-party source) is listed, with its license text, in a top-level `THIRD_PARTY_NOTICES.md`, as synapse does. The PR that adds the first such file creates it.

## Consequences

- This table answers a partner's license review: what reflex links, and under which terms.
- **Scope.** This covers what reflex ships, the code that builds it and the tests, the WebAssembly module included. The docs site's toolchain is not part of it. Astro brings in sharp and libvips (LGPL-3.0) to process images, but they only build the documentation, in CI and on developers' machines.
- **Current state:** reflex's dependencies are those in the table. It contains no third-party code, so it has no `THIRD_PARTY_NOTICES.md` yet. No release artifact bundles the dependencies' license files, because reflex has no packaged release.
