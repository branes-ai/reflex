---
title: Block Pages and Animation
description: The four pages every reflex block publishes, how animations are generated from the block's own code, where reflex animation ends and mission SITL begins, and the plan to build it.
---

Each reflex block captures dynamics that are hard to see in equations and
tables: motor lag feeding into attitude, saturation priority in the mixer,
windup in a PID, a notch tracking a rotor harmonic. Animations show those
dynamics directly. They are also a way to mislead: a smooth animation of a
wrong trace builds false confidence faster than any table. This page defines
how reflex uses animation so that it explains without misleading. It covers:

- the four pages each block publishes, and what animation does on each;
- the rules every animation follows (generated from the block's own code,
  with provenance, paired with the check it illustrates);
- the boundary with mission SITL in consuming repos such as racing-drone;
- the infrastructure and the order of work.

## Assessment

**Add it, at the block level, with strict rules.**

- **Reflex has to own block-level animation; mission SITL cannot.** A
  racing-drone SITL scene shows a mission, with gates, a course, perception,
  and every block running at once. It cannot isolate one block's behaviour,
  so it cannot show why motor lag costs phase margin, or which torque the
  mixer gives up first. Those explanations belong next to the block, in the
  repo that defines it and that its consumers read.
- **Animation helps with exactly the things these blocks are hard on.**
  Attitude on SO(3), cross-coupling, saturation, and time delay are
  difficult to understand from plots alone, and easy to see in a 3-D view
  synchronized with plots.
- **It reuses evidence that is already planned.** The V&V plans already
  produce traces: scenarios, golden traces, closed-loop runs (BB-C1, BB-D1,
  BB-I2). An animation is a view of a trace the harness produces anyway, so
  the marginal cost is the player, not the content.
- **The risk is real and manageable.** Animations go stale and can look
  convincing while being wrong. Both risks are handled by the same rule that
  already governs figures (BB-C1): everything is generated from the block's
  code, carries its provenance, is checked for staleness in CI, and is
  paired with a quantitative check.

Mission SITL in racing-drone keeps everything that is about the mission. The
two share a trace schema and a player, so they look consistent and nothing
is built twice ([boundary](#boundary-with-mission-sitl)).

## Four pages per block

| Page | Reader and question | Content | Role of animation |
|---|---|---|---|
| **Explainer** | A newcomer: how does this work, and why? | The physics or algorithm, built up one idea at a time | Central. Interactive widgets running the real block in the browser (Wasm), with parameters to vary |
| **Developer guide** | A contributor: how do I change, test, or debug it? | API, conventions, contracts, harnesses, debugging workflow | Supporting. Replays of the debug tools, such as an invariant violation found by the monitor and replayed |
| **V&V and datasheet** | A reviewer: what is shown, and how well? | The V&V plan, evidence ledger, generated datasheet | Illustrative only. Each animation sits next to the metric it illustrates and states the pass criterion |
| **Deployment guide** | An integrator: how do I use it on my vehicle, and where does it break? | Parameter identification, rates and timing budget, envelope, failure behaviour, consumer contract | Shows the envelope edges and failure modes, such as saturation, rotor loss, and battery sag |

The current usage page becomes the developer guide. The V&V plan and the
datasheet together become the V&V page. The explainer and deployment pages
are new, and can be short for small blocks. The
[quadrotor control guide](../control/quadrotor-control.md) is already an
explainer for the control layer as a whole.

Pages are grouped per block in the sidebar:

```text
Model
└── Quadrotor
    ├── Explainer
    ├── Developer guide      (today's model/quadrotor)
    ├── V&V and datasheet    (today's model/quadrotor-vv + datasheet)
    └── Deployment guide
```

## Animation rules

### Provenance and truthfulness

- **ANI-1. Generated from the block's code.** An animation renders a trace
  produced by the block's own C++: a `characterize/`, `sim/`, or test
  executable, or the same headers compiled to WebAssembly. Dynamics are never
  re-implemented in JavaScript, and motion is never keyframed by hand.
- **ANI-2. Provenance on screen.** Each trace records the commit, block,
  scenario, seed, compiler, and the simulation and trace step sizes. The
  player shows them in a footer, with a link to the scenario source.
- **ANI-3. Stale traces fail CI.** CI regenerates each committed trace and
  compares it with the committed copy under the `regen`
  [equivalence profile](#equivalence-profiles). The profile defines how
  `t`, `channels`, `events`, and `checks` are compared. The fields that say
  what the trace *is* must match exactly: `schema`, `frames`, `units`,
  `vehicle`, and,
  in `provenance`, `block`, `scenario`, `seed`, `dt_sim`, and `dt_trace`.
  `provenance.commit` and `provenance.compiler` are excluded, because they
  change with every rebuild. A model
  change that alters a trace fails the docs build until the trace is
  regenerated, in the same PR.
- **ANI-4. Paired with a check.** A V&V animation names the requirement it
  illustrates (BB-V4, BB-I3, …) and the quantitative check behind it, and
  its caption states what to look for and the pass criterion. An animation
  is never the evidence itself.
- **ANI-5. Honest time and scale.** Time runs at 1× unless the player shows
  the rate (for example "0.1×"). Exaggerated geometry, such as attitude ×5 or
  a scaled thrust vector, is labelled on screen. Nothing is exaggerated
  silently.
- **ANI-6. Conventions visible.** World and body axes (z up), rotor numbers,
  and spin directions are drawn in the same PX4 Quad X convention as the
  model, and a legend states the frames.

### Pedagogy

- **ANI-7. One idea per animation.** Each animation answers one question,
  stated as its title ("Why does the roll step overshoot?"). Explainers build
  up in sequence: rotor, then thrust, torque, attitude, translation.
- **ANI-8. Predict, then show.** Explainer widgets ask the reader what will
  happen before playing, where that helps, for example "Increase τ: does the
  overshoot grow or shrink?"
- **ANI-9. Only parameters that teach.** Interactive widgets expose the two
  or three parameters that drive the idea, with ranges inside the block's
  validity envelope (BB-S2), and say so when the reader reaches an edge.

### Accessibility and cost

- **ANI-10. Readers control playback.** Animations start paused and respect
  `prefers-reduced-motion`. They have play, pause, scrub, and speed
  controls, keyboard operation, and a text description of what happens.
- **ANI-11. Theme and layout.** They follow the docs' light and dark theme,
  as the SVG figures already do, and work at phone width.
- **ANI-12. Size budget.** Sizes are gzip-compressed bytes (`gzip -9`) of
  the files as committed or as built:
  - each trace file: ≤ 250 KB, after downsampling to the display rate;
  - the WebAssembly module plus its JavaScript loader: ≤ 400 KB together;
  - three.js is outside the budget, but appears only in the built bundles of
    pages that use the 3-D view.

  A CI script lists each counted file with its size and fails over budget.

## Three delivery tiers

| Tier | Mechanism | Used for | Evidence? |
|---|---|---|---|
| **Trace playback** | A committed trace (JSON) played by a `TracePlayer` component: a 3-D vehicle view synchronized with SVG plots and a shared time cursor | V&V, deployment, and developer pages | It shows evidence, as a view of a CI-checked trace |
| **Live in the browser** | The block's headers compiled to WebAssembly with Emscripten; the page reruns the scenario's C++ as the reader moves sliders, and plays the new trace | Explainers | No: interactive, labelled "exploration, not evidence". CI checks that Wasm and native traces agree at the default parameters of every live scenario under the `wasm` [equivalence profile](#equivalence-profiles) |
| **Development viewer** | MCAP traces from the scenario runner (BB-D1), opened in Foxglove or Rerun | Local debugging of any scenario | No: a tool, not a page |

Why not video: video can't follow the site theme, can't be scrubbed against
plots, can't carry machine-checkable provenance, and can't be diffed. It is
also large in git. The trace is the artifact, and the player is code.

### Trace schema

One schema, `reflex-trace/1`, serves reflex pages and mission replays:

```json
{
  "schema": "reflex-trace/1",
  "provenance": {
    "commit": "804c060", "block": "model/quadrotor",
    "scenario": "sim/scenarios/quadrotor/roll_step.json", "seed": 7,
    "compiler": "gcc 13.2 -O2", "dt_sim": 0.0005, "dt_trace": 0.01
  },
  "frames": { "world": "z-up", "body": "x-forward y-left z-up", "rotors": "px4-quad-x" },
  "t": [0.0, 0.01],
  "channels": {
    "position": [[0, 0, 1], [0, 0, 1.0001]],
    "attitude": [[1, 0, 0, 0], [0.9999978, 0.0021, 0, 0]],
    "rotor_speeds": [[1660, 1660, 1660, 1660], [1632, 1688, 1688, 1632]],
    "roll_setpoint": [0.0, 0.349],
    "roll_integrator": [0.0, 0.0001]
  },
  "units": { "roll_integrator": "N·m" },
  "events": [{ "t": 0.0, "label": "roll step 20°" }],
  "checks": [{ "id": "BB-I2", "metric": "overshoot", "unit": "1",
               "value": 0.071, "limit": 0.10, "comparison": "<=", "pass": true }]
}
```

#### Field definitions

The schema is normative. A JSON Schema file,
`docs-site/schemas/reflex-trace-1.schema.json`, checks structure and types.
A validator, `docs-site/src/lib/trace/validate.js`, checks the semantic
rules a JSON Schema cannot express: lengths, monotonicity, and unit norm.
CI validates every committed trace, and `TracePlayer` refuses a trace that
fails, which fails the docs build. The C++ writer
(`tools/trace/include/branes/reflex/trace/writer.hpp`) enforces the same
rules as a scenario records samples. The [Trace Player](trace-player.mdx)
page documents the tooling.

- **Units.** SI throughout, and every angle in radians, including setpoints
  and checks. Degrees appear only in human-readable `label` text.
- **`t`** is time in seconds since the start of the scenario, starting at 0,
  strictly increasing, and uniformly spaced at `dt_trace` when `dt_trace` is
  given.
- **Cardinality.** Each channel has exactly one entry per element of `t`. An
  entry is a number, or an array of the channel's fixed length.
- **Reserved channels.** These have fixed shapes and units, and the 3-D view
  reads them:

  | Channel | Shape | Unit and frame |
  |---|---|---|
  | `position` | [x, y, z] | m, world |
  | `velocity` | [x, y, z] | m/s, world |
  | `attitude` | [w, x, y, z] | Hamilton unit quaternion, body to world, scalar first; norm within 1e-6 |
  | `body_rates` | [x, y, z] | rad/s, body |
  | `rotor_speeds` | [4] | rad/s, PX4 Quad X order |
  | `rotor_thrusts` | [4] | N, PX4 Quad X order |
  | `wrench` | [T, τx, τy, τz] | N and N·m, body |
  | `<quantity>_setpoint` | as `<quantity>` | as `<quantity>`; `roll_setpoint`, `pitch_setpoint`, `yaw_setpoint` in rad |

- **Other channels** must declare a unit in `units`. `"1"` means
  dimensionless. The player plots them but doesn't interpret them.
- **`events`** have `t` within the range of `t`, and a display `label`.
- **`checks`** have `id` (a requirement ID), `metric`, `unit`, `value`,
  `limit`, `comparison` (`<=` or `>=`), and `pass`. `pass` is computed by
  the harness and never by the player. Check `id`s are unique within a
  trace, because the equivalence profiles match checks by `id`.
- **`vehicle`** (optional) holds geometry for the 3-D view: `arm_length` in
  m. Without it, the view draws the RP-1 default airframe.
- **`provenance.equivalence`** (optional) declares an override for a chaotic
  scenario: a `justification`, and optionally a comparison `horizon` in s
  and per-channel `tolerances` (`rtol`, `atol`).
- **Versioning.** Adding optional fields or reserved channels keeps
  `reflex-trace/1`. Any change to an existing field's meaning, shape, or unit
  makes it `reflex-trace/2`.

### Equivalence profiles

Two traces are equivalent under a profile when all of these hold:

1. Their `t` arrays are identical.
2. The compared channels have the same names, and each channel has the same
   shape in both traces.
3. Every component of every sample of every compared channel satisfies
   $|a - b| \le a_{\text{tol}} + r_{\text{tol}}\,|b|$. Here $b$ is the
   reference. Tolerances apply per component, not to a vector norm, so
   `position` is checked on x, y, and z separately. Attitude is the one
   exception, compared as a single angle (below).
4. `events` match exactly.
5. Each check matches the reference check with the same `id`. Its `metric`,
   `unit`, `limit`, `comparison`, and `pass` match exactly, and its `value`
   satisfies the component rule with $r_{\text{tol}}$ = 1e-9 and
   $a_{\text{tol}}$ = 1e-9 in the check's unit. Both traces have the same
   set of check `id`s.
The `trace_diff` tool (BB-D3), `npm run trace-diff` in `docs-site/`,
implements these profiles and reports the first sample and channel that
fail.

| Profile | Used by | Compared | $r_{\text{tol}}$ | $a_{\text{tol}}$ |
|---|---|---|---|---|
| `regen` | ANI-3: regenerated against committed, possibly with a different compiler | All channels, `events`, and `checks` | 1e-9 | position 1e-9 m; velocity 1e-9 m/s; attitude 1e-9 rad; body rates 1e-9 rad/s; rotor speeds and their setpoints 1e-6 rad/s; other channels 1e-9 in their unit |
| `wasm` | Tier 2: Wasm against native, every live scenario at its default parameters | As `regen`: all channels, `events`, and `checks` | 1e-9 | as `regen` |

- **Why `wasm` compares everything.** It first compared only the reserved
  channels, which are the ones the 3-D view reads. The control blocks' live
  scenarios (P3) have none: a PID trace is all block-specific channels. So
  `wasm` now compares what `regen` does. The module and the native build run
  the same scenario function, so they agree to roundoff on every channel,
  and on the events and checks computed from them.
- **Attitude** is compared as the rotation angle
  $2\,\mathrm{asin}(\lVert \mathrm{vec}(q_a^{-1} \otimes q_b) \rVert)$, so
  q and −q are equal.
- **No resampling.** Both traces are written at the same `dt_trace` from the
  same scenario, so their `t` arrays must be identical. A trace that needs
  resampling fails the comparison.
- **Why 1e-9.** Compiler differences (FMA contraction, operation order) and
  libm differences produce errors of order ε ≈ 2e-16 per operation, which
  stay below 1e-9 over the 10⁴–10⁵ steps of a docs scenario when the
  dynamics are not chaotic.
- **Chaotic scenarios**, such as the intermediate-axis tumble, amplify
  roundoff exponentially. Such a trace declares an override in
  `provenance.equivalence`: a shorter comparison horizon or looser tolerances
  per channel, with a one-line justification. Overrides are reviewed like
  golden-file changes (BB-R4).

## Boundary with mission SITL

| Concern | Reflex | Mission repo (racing-drone) |
|---|---|---|
| Scope | One block, or the reference closed loop (BB-I1) | The full stack in a mission |
| Scene | Minimal: axes, ground grid, vehicle glyph, force vectors | Course, gates, environment, other agents |
| Question answered | How the block behaves, and why | Whether the mission succeeds, and how fast |
| Scenarios | Block characterization, envelope edges, faults | Mission profiles, races, operator workflows |
| Rendering | Docs-site player (Tier 1) and Wasm widgets (Tier 2) | Its SITL renderer (Gazebo or another engine), plus the same player for docs |
| Shared | Trace schema `reflex-trace/1` and the `TracePlayer` component, published from reflex | Consumes both, and exports mission replays in the schema |

The rule: if an animation needs mission assets or more than the reference
closed loop, it belongs in the mission repo. If it explains a block, it
belongs in reflex, even when the scenario was found in mission SITL. Such a
scenario is reduced to a block-level reproduction and added to reflex's
scenario suite, which is how mission findings reach the block's V&V.

## How this fits the block workflow

Block work is driven by the agent working through the racing-drone issues.
To keep that agent's PRs small, the standard splits the work:

- **Block PRs** keep adding what they already add: tests, a usage page, and
  closed-loop scenarios. Each scenario writes a `reflex-trace/1` file when
  `REFLEX_TRACE_DIR` is set, which is the same opt-in pattern the gyro
  example uses. That is the only animation-related requirement on a block PR.
- **Documentation PRs**, separate and in this workstream, add the explainer,
  deployment page, and animations from those traces.

Animation is not part of a block's grade. Grades measure evidence, and an
animation is a view of evidence. One addition to the building block standard
is proposed: **BB-R5, deployment guide**, required for grade A, because
integrators need the envelope and failure behaviour in a form they can act
on.

## Plan

| Phase | Deliverables | Exit criteria |
|---|---|---|
| **P0. Infrastructure** | `reflex-trace/1` schema and a C++ writer in `tools/`; `TracePlayer` Astro component (time base, controls, keyboard, reduced motion); plot panel built on the existing `SignalFigure` style with a time cursor; `VehicleView` (three.js: axes, grid, Quad X glyph with rotor numbers and spin arrows, thrust vectors, body axes, trail); CI staleness and size-budget checks | A test trace plays on a docs page in both themes and at phone width; changing the model makes CI fail until the trace is regenerated |
| **P1. Quadrotor model pilot** | Explainer: rotor to thrust and torque; motor-lag step; roll spin-up coupling into translation; torque-free tumbling with asymmetric inertia (the intermediate-axis instability), with energy and angular momentum plotted flat. V&V: mirror relation side by side (BB-V4); trajectories against dt (BB-V2); ground contact. Deployment: full-throttle punch-out into saturation; battery sag. Developer guide: an invariant-violation replay | Each animation meets ANI-1 to ANI-12; page review by one newcomer and one integrator |
| **P2. Live widgets** | Emscripten build target (pinned emsdk) of the model, mixer, and cascade with a small C ABI; explainer widgets with sliders for τ, inertia, and gains; CI checks that Wasm and native traces agree | Widgets run the real headers; agreement within the golden tolerance on every live scenario, at its default parameters |
| **P3. Control and DSP blocks** | PID: windup and the anti-windup clamp, with the integrator state shown against saturation. Mixer: rotor thrust bars at their limits, and the torque given up first. Cascade: closed-loop roll step and waypoint run, with each loop's setpoint shown. Filters: a signal through an RPM-tracked notch during a throttle sweep, with an animated spectrogram | Four pages per block for each of these blocks |
| **P4. Scenario gallery** | Nightly scenario runs (BB-I2) publish their traces; a gallery page lists every scenario with playback and its checks | Gallery regenerates nightly; a failing scenario shows as failing |
| **P5. Shared with racing-drone** | `TracePlayer` and the schema published for reuse (an npm package or a versioned asset); racing-drone exports mission replays in `reflex-trace/1` | A racing-drone replay plays in the same player |

**Status.** P0 is done: the [Trace Player](trace-player.mdx) page plays
its test trace, and the `Docs traces` CI job runs the staleness and size
checks. P1 is done: the quadrotor model has its four
pages, the [explainer](../model/quadrotor-explainer.mdx),
[developer guide](../model/quadrotor.mdx), [V&V plan](../model/quadrotor-vv.mdx#animations),
and [deployment guide](../model/quadrotor-deployment.mdx), with ten animations from
scenarios in `tools/trace/scenarios/quadrotor/`. The exit review, by one
newcomer and one integrator, found them clear (2026-10-26), so the method
continues to P2. P2 is built: three live widgets, two on the
[quadrotor explainer](../model/quadrotor-explainer.mdx) (τ, and the three
moments of inertia) and one now on the
[cascade explainer](../control/cascade-explainer.mdx#2-the-inner-loops-a-roll-step)
(attitude and rate gains), run their scenarios' C++ in the browser. CI
checks each against its native trace. P3 is under way, one block per PR.
The PID controller has its four pages: the
[explainer](../control/pid-explainer.mdx) with three live widgets (the gains,
the derivative filter, and windup), the [developer guide](../control/pid.mdx),
the [V&V plan](../control/pid-vv.mdx), and the
[deployment guide](../control/pid-deployment.mdx), from six scenarios in
`tools/trace/scenarios/pid/`. The mixer has its four too: the
[explainer](../control/mixer-explainer.mdx) with two live widgets (the
saturation priority, and a closed-loop comparison with per-rotor clipping),
the [developer guide](../control/mixer.mdx), the [V&V plan](../control/mixer-vv.mdx),
and the [deployment guide](../control/mixer-deployment.mdx), from three
scenarios in `tools/trace/scenarios/mixer/`. Its rotor thrusts are drawn as
bars at the cursor, a panel the player gained for it. The cascade has its
four too: the [explainer](../control/cascade-explainer.mdx) with two live widgets
(the roll step's gains, and a waypoint square's speed and feed-forward), the
[developer guide](../control/cascade.mdx), the [V&V plan](../control/cascade-vv.mdx),
and the [deployment guide](../control/cascade-deployment.mdx), from five
scenarios in `tools/trace/scenarios/cascade/`. Each loop's setpoint is
shown, and on the square the position loop's setpoint and attitude target
fly as a ghost vehicle. The filters follow.

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

Still open:

1. **BB-R5** (deployment guide) added to the building block standard for
   grade A.
2. **Where the shared player lives** for P5: in reflex's docs-site, or a
   small separate package that reflex and racing-drone both depend on.
