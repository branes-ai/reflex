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
  recorded.
- `trace/scenarios/<block>/<scenario>.cpp`: one executable each, built as
  `trace_<block>_<scenario>` (target `reflex_traces` builds all of them). Each
  writes `$REFLEX_TRACE_DIR/<block>/<scenario>.json`.

Regenerate the committed docs traces after a model change:

```sh
scripts/gen-traces.sh gcc-release docs-site/src/traces
```

CI regenerates them with gcc and clang and fails if any differs from the
committed copy under the `regen` equivalence profile.
