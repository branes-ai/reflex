---
title: Introduction
description: Why the Branes.AI platform separates the deliberative cortex from the involuntary reflex layer.
---

Mammals split their nervous system into two very different kinds of
processing. The **cortex** handles deliberate, high-level work: perceiving the
world, building a model of it, and deciding where to go. It is powerful, but
slow, and it can be wrong. The **involuntary nervous system** (spinal reflexes,
the brainstem, the autonomic system) works without conscious thought. It keeps
you balanced and pulls your hand off a hot stove before you feel the pain. It
is fast, bounded, and always running.

The Branes.AI platform follows the same split:

| Repo | Biological analogue | Responsibility | Time scale |
|---|---|---|---|
| [`cortex`](https://github.com/branes-ai/cortex) | Cerebral cortex | Perception, world building, path planning | ~5–100 Hz, best effort |
| [`reflex`](https://github.com/branes-ai/reflex) | Brainstem, spinal cord | Stabilization, trajectory tracking, optimal control, fault recovery, safety envelopes | ~100 Hz–8 kHz, hard real-time |
| [`racing-drone`](https://github.com/branes-ai/racing-drone) | The whole organism | Mission and flight controller assembled from both | — |

## Design principles

- **Determinism over throughput.** Every update has a bounded, constant
  execution time. Nothing on the hot path allocates, blocks, or throws.
- **The reflex layer never depends on the cortex to stay safe.** Commands from
  the cortex are setpoints to track, never something the vehicle needs in order
  to stay stable.
- **No dynamic reconfiguration.** Gains and limits are fixed when a controller
  is constructed. Changing them means building a new controller through the
  lifecycle, never mutating one in the hot path.
- **Type-generic numerics.** Controllers are templates over the scalar type, so
  mixed-precision and custom-arithmetic studies use the same code that flies.
