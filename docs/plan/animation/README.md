---
status: complete
---
# Plan: block pages and animation

The plan that builds the [animation standard](../../standards/animation.md):
four pages per block, with animations generated from the block's own code.
The standard holds the rules (the four pages, ANI-1 to ANI-12, the trace
schema, the boundary with mission SITL); the
[assessment](../../assessments/animation.md) says why. This plan was part of
the standard's page until it moved here, to be published with the other
plans on the [architecture site](https://branes-ai.github.io/architecture/).

## Phases

| Phase | Deliverables | Exit criteria |
|---|---|---|
| **P0. Infrastructure** | `reflex-trace/1` schema and a C++ writer in `tools/`; `TracePlayer` Astro component (time base, controls, keyboard, reduced motion); plot panel built on the existing `SignalFigure` style with a time cursor; `VehicleView` (three.js: axes, grid, Quad X glyph with rotor numbers and spin arrows, thrust vectors, body axes, trail); CI staleness and size-budget checks | A test trace plays on a docs page in both themes and at phone width; changing the model makes CI fail until the trace is regenerated |
| **P1. Quadrotor model pilot** | Explainer: rotor to thrust and torque; motor-lag step; roll spin-up coupling into translation; torque-free tumbling with asymmetric inertia (the intermediate-axis instability), with energy and angular momentum plotted flat. V&V: mirror relation side by side (BB-V4); trajectories against dt (BB-V2); ground contact. Deployment: full-throttle punch-out into saturation; battery sag. Developer guide: an invariant-violation replay | Each animation meets ANI-1 to ANI-12; page review by one newcomer and one integrator |
| **P2. Live widgets** | Emscripten build target (pinned emsdk) of the model, mixer, and cascade with a small C ABI; explainer widgets with sliders for τ, inertia, and gains; CI checks that Wasm and native traces agree | Widgets run the real headers; agreement within the golden tolerance on every live scenario, at its default parameters |
| **P3. Control and DSP blocks** | PID: windup and the anti-windup clamp, with the integrator state shown against saturation. Mixer: rotor thrust bars at their limits, and the torque given up first. Cascade: closed-loop roll step and waypoint run, with each loop's setpoint shown. Filters: a signal through an RPM-tracked notch during a throttle sweep, with an animated spectrogram | Four pages per block for each of these blocks |
| **P4. Scenario gallery** | Nightly scenario runs (BB-I2) publish their traces; a gallery page lists every scenario with playback and its checks | Gallery regenerates nightly; a failing scenario shows as failing |
| **P5. Shared with racing-drone** | `TracePlayer` and the schema published for reuse (an npm package or a versioned asset); racing-drone exports mission replays in `reflex-trace/1` | A racing-drone replay plays in the same player |

## Progress

P0 is done: the [Trace Player](../../standards/trace-player.mdx) page plays
its test trace, and the `Docs traces` CI job runs the staleness and size
checks. P1 is done: the quadrotor model has its four
pages, the [explainer](../../model/quadrotor-explainer.mdx),
[developer guide](../../model/quadrotor.mdx), [V&V plan](../../model/quadrotor-vv.mdx#animations),
and [deployment guide](../../model/quadrotor-deployment.mdx), with ten animations from
scenarios in `tools/trace/scenarios/quadrotor/`. The exit review, by one
newcomer and one integrator, found them clear (2026-10-26), so the method
continues to P2. P2 is built: three live widgets, two on the
[quadrotor explainer](../../model/quadrotor-explainer.mdx) (τ, and the three
moments of inertia) and one now on the
[cascade explainer](../../control/cascade-explainer.mdx#2-the-inner-loops-a-roll-step)
(attitude and rate gains), run their scenarios' C++ in the browser. CI
checks each against its native trace. P3 was built one block per PR.
The PID controller has its four pages: the
[explainer](../../control/pid-explainer.mdx) with three live widgets (the gains,
the derivative filter, and windup), the [developer guide](../../control/pid.mdx),
the [V&V plan](../../control/pid-vv.mdx), and the
[deployment guide](../../control/pid-deployment.mdx), from six scenarios in
`tools/trace/scenarios/pid/`. The mixer has its four too: the
[explainer](../../control/mixer-explainer.mdx) with two live widgets (the
saturation priority, and a closed-loop comparison with per-rotor clipping),
the [developer guide](../../control/mixer.mdx), the [V&V plan](../../control/mixer-vv.mdx),
and the [deployment guide](../../control/mixer-deployment.mdx), from three
scenarios in `tools/trace/scenarios/mixer/`. Its rotor thrusts are drawn as
bars at the cursor, a panel the player gained for it. The cascade has its
four too: the [explainer](../../control/cascade-explainer.mdx) with two live widgets
(the roll step's gains, and a waypoint square's speed and feed-forward), the
[developer guide](../../control/cascade.mdx), the [V&V plan](../../control/cascade-vv.mdx),
and the [deployment guide](../../control/cascade-deployment.mdx), from five
scenarios in `tools/trace/scenarios/cascade/`. Each loop's setpoint is
shown, and on the square the position loop's setpoint and attitude target
fly as a ghost vehicle. The gyro filters have their four too, which
completes P3: the [explainer](../../dsp/filters-explainer.mdx) with a live
widget (the notches' quality factor and the rotor speed error, through a
throttle sweep), the [developer guide](../../dsp/filters.md), the
[V&V plan](../../dsp/filters-vv.mdx), and the
[deployment guide](../../dsp/filters-deployment.mdx), from four scenarios in
`tools/trace/scenarios/filters/`. The player gained a spectrogram panel for
them: the scenario's C++ computes each spectrum, and the player paints it,
filling in as the trace plays, with the notch centres drawn over it. P4
is built: the [scenario gallery](../../scenarios/gallery.mdx) lists every
scenario by block, with its checks, its status, and the players that
explain it, and gives each scenario a generated page that plots every
channel. The deploy workflow runs the suite with gcc and clang on every
deploy and nightly, publishes the site from that run, and fails after
publishing when a scenario fails. A check can be marked `"expect": "fail"`
for a scenario that shows the edge of the envelope (`filters/low_rate`),
and CI fails any other failing check. P5 is done, which completes the plan.
The player, the schema, and its validator are one npm package,
`@branes-ai/trace-player`, in `packages/trace-player`
([#65](https://github.com/branes-ai/reflex/pull/65)). Each reflex release
attaches it as a tarball, first v0.9.0, and reflex's own site builds from
the same packed files. racing-drone's `trace_export` writes a SITL log as a
`reflex-trace/1` replay, and its
[race replay](https://branes-ai.github.io/racing-drone/guides/mission-replay/)
plays two laps of baseline B0 in the same player, pinned to the v0.9.1
tarball ([racing-drone#87](https://github.com/branes-ai/racing-drone/pull/87)).
The player gained options for a trace from another repository: the repository
its footer links to, how the trace is kept, and the dashed line's label.

## Order

P0 and P1 come first: the quadrotor model has the richest dynamics and the
most V&V content to illustrate. P1 is also the test of whether the method
pays for itself. If newcomers and integrators don't find the pages clearer
than the static ones, stop at P1 and keep the player for V&V pages only.
P2 goes after P1, because Wasm adds a toolchain to CI and only explainers
need it.

## Decisions

Taken:

- **three.js on the docs site.** Taken: the 3-D view uses three.js. It
  loads as a separate chunk (about 120 KB gzipped, view code included) only
  on pages with a 3-D view, and only when the player scrolls near the
  viewport. The size-budget check enforces the placement.
- **Emscripten in CI.** Taken in P2: a pinned emsdk on the docs
  workflows. The `Docs traces` job and the deploy workflow each install the
  release named in `scripts/emsdk-version` and build the module from source
  with `scripts/build-wasm.sh`. No binary is committed and no artifact passes
  between workflows, so the module always matches the commit it ships with.
- **Sidebar.** Taken in P1: each block gets a sidebar group with its four
  pages (Model › Quadrotor › Explainer, Developer Guide, V&V Plan,
  Deployment Guide). The pages keep their URLs (`model/quadrotor`,
  `model/quadrotor-vv`) and the new ones sit beside them
  (`model/quadrotor-explainer`, `model/quadrotor-deployment`), so nothing
  moves and no redirects are needed. The datasheet joins the V&V page when
  WS5 produces it.
- **Where the shared player lives.** Taken in P5: a package in reflex,
  `packages/trace-player`, rather than its own repository, so the player
  and the schema change with the traces reflex writes, in one PR. It is
  versioned with reflex and shipped as a tarball on each release, which
  other sites pin by URL, rather than published to an npm registry. racing-drone
  writes its traces with its own small writer, since its reflex pin
  predates `branes::reflex_trace`.

Still open:

1. **BB-R5** (deployment guide) added to the building block standard for
   grade A. The five blocks this plan covers each have one: the quadrotor
   model, the PID controller, the mixer, the cascade, and the gyro filters.
