---
title: Animation in reflex
description: Whether reflex should animate its blocks, and on what terms. It should, at the block level, with strict rules.
---

The assessment behind the [animation standard](../standards/animation.md),
written with it in [#35](https://github.com/branes-ai/reflex/pull/35). The [animation plan](../plan/animation/README.md)
builds it.

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
two are to share a trace schema and a player, so they look consistent and
nothing is built twice ([boundary](../standards/animation.md#boundary-with-mission-sitl)).
The schema is reflex's today; sharing the player with racing-drone is P5 of
the [plan](../plan/animation/README.md), not yet done.
