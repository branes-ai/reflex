# @branes-ai/trace-player

The `reflex-trace/1` trace format and the player that shows it on a docs
site, shared by reflex and the repos that build on it. It contains:

- **`TracePlayer.astro`**: an Astro component that plays one trace. A three.js
  vehicle view and SVG plots share one time cursor, with playback controls,
  keyboard control, the trace's events and checks, and its provenance.
  Optionally it is a live widget, with sliders that rerun the scenario in
  WebAssembly.
- **The schema**, `schemas/reflex-trace-1.schema.json`, and a validator that
  adds the semantic rules (`validate`).
- **The CI helpers**: trace equivalence under a named tolerance profile
  (`diff`), and a check's status against what the trace expects of it
  (`status`).

The format and the rules every animation follows (ANI-1 to ANI-12) are in
reflex's [animation standard](https://branes-ai.github.io/reflex/standards/animation/).
The player's behaviour is on its [standard page](https://branes-ai.github.io/reflex/standards/trace-player/).

## Install

The package ships with each reflex release as an npm tarball. Install it by
URL in the docs site; npm pins the URL in `package.json`:

```sh
version=0.9.2  # x-release-please-version
npm install "https://github.com/branes-ai/reflex/releases/download/v$version/branes-ai-trace-player-$version.tgz"
```

`version` is the latest release; release-please updates it, and the release
job checks it.

Its version is reflex's. It brings `three` and `ajv` with it.

The host is a Starlight site (docs-kit builds one), with Astro 5 or later.
The player uses Starlight's theme variables (`--sl-color-accent`,
`--sl-text-sm`) and its `data-theme` attribute for dark mode.

## Use

In an MDX page:

```mdx
import TracePlayer from '@branes-ai/trace-player/TracePlayer.astro';
import hop from '../traces/scripted_hop.json';

<TracePlayer
  trace={hop}
  title="Does the hop clear the first gate?"
  description="Twelve seconds of the scripted hop: ..."
  plots={[
    { channel: 'position', title: 'Position', labels: ['x', 'y', 'z'] },
  ]}
  repo="https://github.com/branes-ai/racing-drone"
  provenanceNote="a recorded SITL run"
/>
```

The trace is validated at build time, so an invalid trace fails the build.
The props:

| Prop | |
|---|---|
| `trace` | The parsed trace, usually a JSON import |
| `title` | The one question the animation answers (ANI-7) |
| `description` | A text description of what happens (ANI-10) |
| `plots` | Time plots (`{ channel, title, labels, setpoint?, setpointLabel?, colors? }`), bar panels (`kind: 'bars'`), and spectrograms (`kind: 'spectrogram'`) |
| `view` | Show the 3-D view; the default is true, and the view needs `position` and `attitude` channels |
| `thrustScale` | The thrust-arrow scale in m/N, shown on screen (ANI-5) |
| `ghost` | A second run drawn translucent beside the first: `{ position, attitude, thrusts? }` channel names |
| `live` | `{ scenario, params, wasm }`: sliders that rerun the scenario in a WebAssembly module (see `src/lib/live.js` for the module's contract) |
| `prompt` | For a live widget: what to predict before moving a slider (ANI-8) |
| `repo` | The GitHub repository the footer links the commit and the scenario to (the scenario as it was at that commit); the default is reflex, so set it to yours |
| `provenanceNote` | How the trace is kept, at the end of the footer; the default, "checked by CI against a fresh run", is true of reflex's traces |

In CI, validate the traces a site commits:

```js
import { validateTrace } from '@branes-ai/trace-player/validate';
import { traceStatus, failing } from '@branes-ai/trace-player/status';
```

## Write a trace

From C++, link reflex's header-only writer, `branes::reflex_trace`
(`<branes/reflex/trace/writer.hpp>`). It is defined whenever reflex is added
to a CMake build, as a subproject or with FetchContent. A writer in another
language must produce the same JSON; `validateTrace` is the reference for
what is accepted.

## Develop

The package lives in reflex's `packages/trace-player`. reflex's docs site
installs it as a packed copy (`docs-site/.npmrc`), the same files a release
tarball holds. `npm run build` and the trace scripts refresh that copy when
the source changes.
