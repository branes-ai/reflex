# ADR 0002 — No GPL contact; clean-room implementation from papers

**Status:** Accepted
**Date:** 2026-10-31
**Source:** the stack's no-GPL policy, set in cortex's [ADR 0002](https://github.com/branes-ai/cortex/blob/main/docs/adr/0002-no-gpl-clean-room-policy.md)
**Related:** [ADR-0001](0001-third-party-license-compatibility.md)

## Context

cortex's ADR 0002 rules out GPL code anywhere in cortex, for any purpose. The reason applies to reflex unchanged: partners redistribute the stack inside their own certified products, and any GPL obligation, even one confined to a test, is something their legal reviews would have to clear.

In flight control, the pull towards GPL code is strong. Several of the most widely used open flight stacks are GPL-licensed:
- Betaflight, INAV and Cleanflight (GPL-3.0)
- ArduPilot (GPL-3.0)
- Paparazzi (GPL-2.0)

PX4 (BSD-3-Clause) is permissive. reflex already follows PX4's conventions (the Quad X rotor order), and its cascade V&V plan names a code-to-code comparison with PX4's multicopter controllers.

## Decision

**No GPL-licensed code anywhere in reflex, for any purpose.** Not as a dependency, not in a test target, not as a build-time tool in what reflex builds, and not as a reference oracle.

- Controllers, filters and models are implemented **clean-room from papers, textbooks and specifications**, plus permissively licensed (MIT / BSD / Apache-2.0) references.
- **Check a reference's license before opening its code.** Reading GPL source and then writing the same thing "from scratch" is not clean-room. A GPL project may be consulted for its structure only, never for its code. This includes tuning tables and filter coefficients, which are code too.
- **Test oracles are in-house or permissive:** reflex's own reference models and recorded traces, closed-form results from the literature, permissive libraries (mp-dsp is the `dsp/` tests' reference), and permissive stacks such as PX4. Never compare against the output of a GPL flight stack (Betaflight, INAV, ArduPilot, …), its SITL or its logs.
- **Conventions are not code.** Following a convention that a GPL project also uses (a rotor order, a frame, a unit) is fine, when it is taken from a permissive source or a specification.
- LGPL, MPL and SSPL candidates are flagged for explicit review and default to rejected.

## Consequences

- Each controller costs more to build: the design comes from the literature, and the V&V uses reflex's own models and permissive references.
- PX4 is the comparison of record for code-to-code checks, as the cascade V&V plan already assumes.
- Partners can redistribute reflex under MIT terms with no copyleft review.
- **Current state:** reflex's controllers, filters and models are written in-house. The only external flight stack it refers to is PX4, for conventions and a planned comparison. No GPL code or oracle is used.
