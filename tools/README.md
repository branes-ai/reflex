# tools/

Host-side developer tools: simulation drivers, gain-tuning and step-response
utilities, log and trace inspectors. Tools may use heavier dependencies than the
header-only library (plotting, file I/O), but nothing here is a dependency of
`branes::reflex`. They build when reflex is the top-level project
(`REFLEX_BUILD_TOOLS`).

## trace/

The `reflex-trace/1` writer and the scenarios that produce the traces played on
the docs site (see the *Trace Player* page under Standards).

- `trace/include/branes/reflex/trace/writer.hpp`: the header-only writer
  (`branes::reflex_trace`). It enforces the format's invariants as samples are
  recorded. `trace/CMakeLists.txt` defines it even when reflex is a
  dependency and the tools are off, so a consuming repo can export its own
  traces (racing-drone's mission replays).
- `trace/scenarios/<block>/<scenario>.cpp`: one executable each, built as
  `trace_<block>_<scenario>` (target `reflex_traces` builds all of them). Each
  writes `$REFLEX_TRACE_DIR/<block>/<scenario>.json`. Headers next to the
  scenarios hold shared recording code (for example
  `quadrotor/quadrotor_trace.hpp`) and are not built on their own. A scenario
  that a live widget runs keeps its body in `<scenario>.hpp`, as a function of
  its parameters; the `.cpp` calls it with the defaults.

Regenerate the committed docs traces after a model change:

```sh
scripts/gen-traces.sh gcc-release docs-site/src/traces
```

CI regenerates them with gcc and clang and fails if any differs from the
committed copy under the `regen` equivalence profile.

## wasm/

`reflex_wasm.cpp`: the docs site's live widgets. It compiles the widget
scenarios (`trace/scenarios/<block>/<scenario>.hpp`) to a WebAssembly ES
module with Emscripten and exposes them through a small C ABI,
`reflex_run(scenario, p0, p1, p2)`, which returns a `reflex-trace/1` JSON
string. Under Emscripten, `tools/` builds only this module.

```sh
scripts/install-emsdk.sh ~/emsdk     # the release pinned in scripts/emsdk-version
source ~/emsdk/emsdk_env.sh
scripts/build-wasm.sh                # → docs-site/public/wasm/reflex.{mjs,wasm}
(cd docs-site && npm run wasm-agree) # Wasm against the native traces, `wasm` profile
```

Without the module, the docs site still builds and plays the recorded
traces; the sliders then report that live runs are unavailable.
