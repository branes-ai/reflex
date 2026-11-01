# 2026-10-31: animation plan P4 and P5, the shared trace player

Finished the [animation plan](../plan/animation/README.md): the scenario
gallery (P4) merged, and the trace player became a package that racing-drone
uses for its race replays (P5). The plan is marked complete.

## Done

### reflex

| PR | What |
|---|---|
| [#63](https://github.com/branes-ai/reflex/pull/63) | P4, the scenario gallery and nightly runs. Review fixes: `check.mjs` records a non-JSON regenerated trace as invalid and keeps going; `--publish DIR` copies only valid traces; the deploy runs clang, then gcc with `--publish`, so the site shows the gcc run and says so |
| [#65](https://github.com/branes-ai/reflex/pull/65) | P5, `packages/trace-player` (`@branes-ai/trace-player`): `TracePlayer.astro`, the plot and view code, the schema, `validate`, `diff`, `status`, `channels`, and `live` (the Wasm ABI). `branes::reflex_trace` moved to `tools/trace/` and is defined for consumers too. The release workflow attaches `npm pack`'s tarball to each release |
| [#68](https://github.com/branes-ai/reflex/pull/68) | The package README installs from v0.9.0 |
| [#72](https://github.com/branes-ai/reflex/pull/72) | The animation plan: P5 done, status `complete`, and the shared-player decision taken |

Releases: v0.8.0 (P4), v0.9.0 (P5, the first with
`branes-ai-trace-player-0.9.0.tgz`), v0.9.1 (#67, #68).

### racing-drone

| PR | What |
|---|---|
| [#84](https://github.com/branes-ai/racing-drone/pull/84) | docs-kit v0.1.0 → v0.2.1, and the `@site/*` alias |
| [#87](https://github.com/branes-ai/racing-drone/pull/87) | `trace_export` (a SITL log to a `reflex-trace/1` replay), `sim/replay_trace.hpp` (the writer, with its own JSON), `tools/gen_replay.sh`, and the [race replay](https://branes-ai.github.io/racing-drone/guides/mission-replay/) page, pinned to the v0.9.1 player |

## Decisions

- **The player is a package in reflex**, not its own repository: the player,
  the schema and the traces change together. It is versioned with reflex and
  pinned by release-tarball URL, not published to an npm registry. Recorded in
  the plan's Decisions.
- **reflex's site installs the package as a packed copy** (`.npmrc`
  `install-links=true`), the same files a release holds, so a file missing
  from `files` breaks reflex's build first. `scripts/player-sync.mjs`
  refreshes the copy from every `pre*` hook that builds or reads traces.
- **racing-drone writes traces with its own small writer.** Its reflex pin
  predates `branes::reflex_trace`, and its configure fails if anatomy pins a
  different reflex, so a pin bump would have been a three-repo change.
- **The player's defaults stay reflex's.** New props for another repo's
  traces: `repo` (the footer's links), `provenanceNote` (how the trace is
  kept), and a plot's `setpointLabel`. An event button now seeks to the first
  sample at or after the event. The footer links the scenario at the trace's
  commit, or the default branch when the commit is not a hash.
- **The racing-drone replay is a record**, stamped with its commit by
  `gen_replay.sh`; CI does not regenerate it.

## What went wrong

- **Release order.** Release PR #64 (0.8.0) merged before #65, so v0.8.0 has
  no package and the README's v0.8.0 URL never existed. release-please cut
  0.9.0 with the package; #68 fixed the README, and landed in 0.9.1. Lesson:
  a release-attached asset exists only from the first release after its
  workflow lands, so merge the feature before the pending release PR.
- **A refactor froze a value.** Hoisting the event tolerance into a constant
  (for review) froze it at the recorded trace; a live widget's `load()`
  replaces the time base. CodeRabbit caught it; it is a function again.

## Open

- **BB-R5**, the deployment guide in the building block standard for grade
  A. The five blocks with pages all have one.
- The package README pins v0.9.0, one release behind. It works, and the
  URL must name a version (a release tarball has no unversioned alias), and
  names it twice, in the tag and in the file name. Keeping it current needs
  an update that changes both on each release, and a check in the
  `trace-player` release job that the README's URL matches the version it
  uploads; a single release-please version marker on the line may update
  only the first.
- racing-drone's replay view does not draw the course; the events step
  through the gate passages.
