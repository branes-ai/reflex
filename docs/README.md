# Documentation map

These documents are published at <https://branes-ai.github.io/reflex/>. The site is generated from this folder by [docs-kit](https://github.com/branes-ai/docs-kit) on every merge to main: edit the documents here, never the site. [`index.mdx`](index.mdx) is the landing page, and this file is the map. The pages with live widgets are MDX, and import the site's components and traces from `docs-site/src/` through `@site/`.

| Folder | Contents |
|---|---|
| [getting-started/](getting-started/introduction.md) | [Introduction](getting-started/introduction.md), [Build & Test](getting-started/build-and-test.md), [Repository Layout](getting-started/repository-layout.md) |
| [standards/](standards/building-blocks.md) | [Building Block Standard](standards/building-blocks.md), [Block Pages and Animation](standards/animation.md), [Trace Player](standards/trace-player.mdx) |
| [model/](model/quadrotor-explainer.mdx) | [How the Quadrotor Model Works](model/quadrotor-explainer.mdx), [Quadrotor Model](model/quadrotor.mdx), [Quadrotor V&V Plan](model/quadrotor-vv.mdx), [Quadrotor Model Deployment](model/quadrotor-deployment.mdx) |
| [control/](control/quadrotor-control.md) | [Quadrotor Control Guide](control/quadrotor-control.md), [How the PID Controller Works](control/pid-explainer.mdx), [PID Controller](control/pid.mdx), [PID V&V Plan](control/pid-vv.mdx), [PID Deployment](control/pid-deployment.mdx), [How the Mixer Works](control/mixer-explainer.mdx), [Mixer](control/mixer.mdx), [Mixer V&V Plan](control/mixer-vv.mdx), [Mixer Deployment](control/mixer-deployment.mdx), [How the Cascade Works](control/cascade-explainer.mdx), [Cascaded Controller](control/cascade.mdx), [Cascade V&V Plan](control/cascade-vv.mdx), [Cascade Deployment](control/cascade-deployment.mdx) |
| [dsp/](dsp/filters.md) | [Gyro Filters](dsp/filters.md) |
| [examples/](examples/gyro-lowpass.mdx) | [Mixed-Precision Gyro Filter](examples/gyro-lowpass.mdx) |
| [Roadmap](roadmap.md) | From classical PID to fault-tolerant non-linear MPC. |

Engineering notes that are not published yet go in `arch/`, `assessments/` and `sessions/` as they land; the plans and decisions (`plan/`, `adr/`) are published on the [architecture site](https://branes-ai.github.io/architecture/reflex/).
