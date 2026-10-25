---
title: Building Block Standard
description: What a reflex building block must ship, how it is verified, characterized, and validated, and how it earns a quality grade.
---

Reflex is a building-block repository. Mission repositories such as
[racing-drone](https://github.com/branes-ai/racing-drone) pull models,
controllers, and estimators from it and build on them without re-checking
them. That only works if every block carries its own evidence: what it claims,
how each claim was checked, and how to re-run the check.

This page defines that evidence. Each requirement has an ID (`BB-S1`, `BB-V2`,
…). Per-block V&V plans and datasheets cite these IDs, and a block's grade is
the set of requirements it meets.

## Kinds of evidence

The standard keeps four questions apart, following the simulation-credibility
literature (Oberkampf & Roy; ASME V&V 10/20; NASA-STD-7009):

| Question | Name | Typical evidence |
|---|---|---|
| Is the code correct? | **Testing** | Unit tests, contracts, sanitizers, property tests, fuzzing |
| Does it solve its equations correctly? | **Verification** | Analytic solutions, observed order of accuracy, conservation, symmetry, derivative checks, code-to-code comparison |
| What does it do, how accurately, at what cost? | **Characterization** | Generated datasheet: responses, error budgets, numeric-type accuracy, latency |
| Are they the right equations for the job? | **Validation** | Comparison with measured data, with stated metrics, acceptance criteria, and a validity envelope |

Passing tests says nothing about validation, and a validated model with an
unverified integrator is not validated. A block reports all four separately,
first in isolation and then integrated in a closed loop
([BB-I](#integration-i)).

## What a block ships

| Item | Location |
|---|---|
| Header with the spec (BB-S1…S5) | `sdk/include/branes/reflex/<layer>/<block>.hpp` |
| Usage page with its grade | `docs-site/src/content/docs/<layer>/<block>.md` |
| V&V plan: requirement status, evidence ledger, milestones | `docs-site/src/content/docs/<layer>/<block>-vv.md` |
| Datasheet: generated figures and tables | `docs-site/src/content/docs/<layer>/<block>-datasheet.mdx` |
| Unit and contract tests | `tests/sdk/` |
| Verification tests | `tests/verification/` |
| Property tests | `tests/property/` |
| Fuzz targets | `fuzz/` |
| Characterization executables | `characterize/<block>/` |
| Validation pipeline (data-driven) | `validate/<block>/` |
| Closed-loop scenarios | `sim/scenarios/<block>/` |
| Debug tooling (trace, replay, diff) | `tools/` |
| Benchmarks | `bench/` |

## Requirements

### Specification (S)

- **BB-S1. Spec in the header.** Equations or semantics, conventions (frames,
  units, signs, orderings), the data model (members, ownership, mutability),
  numbered invariants, and pre/postconditions on every operation. Written
  before the implementation.
- **BB-S2. Validity envelope and non-goals.** What the block does not model,
  and the operating ranges where it is expected to hold. Until validation
  (BB-L4) replaces it with a measured envelope, mark it as expected.
- **BB-S3. Parameters.** Units, valid ranges, a validator function that
  rejects every violation, and defaults traceable to a named source (a vehicle
  file, a datasheet, an identification run).
- **BB-S4. Scalar-type support.** The scalar types the block supports, and the
  minimum precision at which its invariants and tolerances hold. A tolerance
  must scale with the type's precision or the floor must be documented.
- **BB-S5. Resource contract.** Allocation, exceptions, determinism, and
  worst-case execution time behaviour, stated and tested.

### Testing (T)

- **BB-T1. Unit and contract tests.** Each invariant and each operation
  contract has a test. Each validator rejects each violation it names.
- **BB-T2. Compiler matrix.** GCC, Clang, and MSVC; Debug (contracts live) and
  Release (contracts compiled out).
- **BB-T3. Sanitizers.** The full suite runs clean under AddressSanitizer and
  UndefinedBehaviorSanitizer.
- **BB-T4. Property-based tests.** Generated valid inputs (parameters, states,
  commands, step sizes) check that the invariants and postconditions hold.
  The seed is logged, and a failure prints a minimal reproducer.
- **BB-T5. Fuzzing.** Every entry point that takes structured numeric input
  has a libFuzzer target, run under the sanitizers, with a committed seed
  corpus.
- **BB-T6. Coverage.** Line and branch coverage of the block's header is
  reported. Target: ≥ 95 % line, ≥ 90 % branch, with exclusions justified.
- **BB-T7. Mutation score.** Mutation testing (Mull) measures whether the
  suite detects injected faults. Target: ≥ 80 % of mutants killed, and every
  surviving mutant either killed by a new test or explained as equivalent.

### Verification (V)

- **BB-V1. Analytic solutions.** Every term of the equations is exercised by
  at least one case with a closed-form answer.
- **BB-V2. Observed order of accuracy.** Every discretization shows its
  theoretical convergence order on a smooth problem, measured by a step-size
  sweep and fitted on the pre-roundoff range.
- **BB-V3. Structure.** Conserved quantities are conserved, dissipative terms
  dissipate, and manifold constraints (unit quaternions, positive
  definiteness) hold.
- **BB-V4. Symmetry and metamorphic relations.** Transformations with a known
  effect on the output (rotations, reflections, translations, parameter
  scalings) produce that effect.
- **BB-V5. Derivatives.** If the block is linearized by a consumer (MPC, EKF,
  LQR), its Jacobians are checked against finite differences and an
  independent method (forward-mode dual numbers), and against a hand-derived
  linearization at a named operating point.
- **BB-V6. Code-to-code comparison.** Golden traces from an independent
  implementation, configured to the same equations, agree within a stated
  tolerance. The frame and ordering mapping between the two is itself tested.
- **BB-V7. Stability limits.** Stability bounds in the contract (such as
  maximum step size) are confirmed empirically, with margin.

### Characterization (C)

- **BB-C1. Generated datasheet.** Each figure and table comes from a
  `characterize/` executable and a figure script, reproducible with one
  documented command. Each records the commit that produced it. Hand-typed
  numbers are not allowed.
- **BB-C2. Dynamic response.** Step and frequency response, and the
  linearization (eigenvalues, input matrix) at the operating points consumers
  use.
- **BB-C3. Numerical error budget.** Error against step size and against
  scalar type (float, double, and the Universal types the block supports)
  over consumer-relevant horizons, relative to a high-precision reference.
- **BB-C4. Cost.** Latency per call per scalar type and compiler, stack usage,
  and code size; target-hardware numbers when a target exists.
- **BB-C5. Sensitivity and uncertainty.** Which parameters the outputs depend
  on most, and how identified parameter uncertainty propagates to output
  spread.

### Validation (L)

For a physical model, validation compares the model with measured data. For a
controller or estimator, validation runs in closed loop: on a verified and
validated plant model first, then on hardware-in-the-loop or flight. A block
with no physical counterpart marks each L requirement N/A, with a reason.

- **BB-L1. Data with provenance.** Each dataset is named with its source,
  license, and vehicle. Large data is fetched by a script with checksums, not
  committed.
- **BB-L2. Identified parameters.** Parameters fitted from data are evaluated
  on held-out flights, never only on the fitting set.
- **BB-L3. Consumer metrics.** Metrics match the consumer's use: an MPC model
  is judged by open-loop prediction error over the MPC horizon, not by
  one-step error.
- **BB-L4. Measured envelope.** Error is reported against the operating
  variables (speed, rate, thrust), and the envelope where acceptance holds is
  published. Outside it the block is explicitly unvalidated.
- **BB-L5. Pre-stated acceptance.** Acceptance criteria are written down
  before the evaluation runs, and derived from what the consumer tolerates.

### Integration (I)

A block that passes every requirement above can still fail as part of a
system. Feedback hides some component errors, such as a wrong inertia under a
high-gain loop, and amplifies others: a few milliseconds of unmodelled latency
change stability margins. Integration evidence runs the block inside a closed
loop in a virtual environment. The block is either the system under test
(controllers, estimators) or the plant that other systems are tested
against (models).

- **BB-I1. Reference closed loop.** The block runs in reflex's `sim/`
  harness: plant, sensor models (noise, bias, quantization, latency), actuator
  models (update rate, quantization, delay), the controller or estimator
  under test, and an environment (wind, turbulence). All of it is stepped in
  lockstep at the real loop rates.
- **BB-I2. Scenario suite.** Declarative scenarios cover nominal flight, the
  edges of the envelope, saturation, and injected faults (rotor loss, sensor
  dropout, gusts, battery depletion). Each has pass criteria, and the suite
  runs in CI.
- **BB-I3. Closed-loop validation.** For a block used as a SITL plant, or
  validated in SITL, closed-loop metrics in simulation are compared with the
  same metrics in real flight, using the same controller and gains: step
  response, oscillation frequency and damping, tracking error, actuator
  saturation. Acceptance is stated beforehand (BB-L5).
- **BB-I4. Timing.** Deterministic lockstep mode for CI, and a real-time mode
  that reports loop jitter, overruns, and the block's share of the tick.
- **BB-I5. Processor in the loop.** The block is cross-compiled for the
  flight processor, and the golden scenarios run there: traces are compared
  with the host's, and cycles per call are reported. Hardware-in-the-loop
  follows in the consuming repo.
- **BB-I6. Interoperability.** Where a consumer tests its own stack against
  the block, a bridge (PX4 SITL, Gazebo, or an FMI 3.0 FMU) exposes it with
  the same conventions, and a test pins those conventions.

### Debugging (D)

- **BB-D1. Traces and scenarios.** A scenario runner drives the block from a
  declarative scenario and records a trace in a standard format (CSV for
  scripts, MCAP for Foxglove and Rerun).
- **BB-D2. Invariant monitor and exact replay.** A monitor names the first
  violated invariant, with its step and field, and writes a repro file with
  exact (hex-float) inputs that replays bit-identically.
- **BB-D3. Trace diff.** A tool reports the first divergence between two
  traces beyond a per-field tolerance.

### Release and consumption (R)

- **BB-R1. Versioning.** Semantic versioning. Public API and convention
  changes are breaking and called out in the changelog.
- **BB-R2. Evidence bundle.** Each release regenerates the datasheet, runs
  validation, and publishes test results, datasheet, validation report, and
  credibility scorecard together.
- **BB-R3. Consumer contract tests.** Each downstream repo pins the behaviours
  it relies on in its own tests against the released block.
- **BB-R4. Golden regression.** Numerical output is pinned by golden traces.
  A change to a golden file is a deliberate commit with its rationale.

## Grades

| Grade | Meaning | Requires |
|---|---|---|
| **C** Usable | Specified and unit-tested | S1, S3, S5, T1, T2, a benchmark |
| **B** Verified | Solves its equations correctly | C + S2, S4, T3, T4, V1–V4, V7, R4 |
| **A** Characterized | Its behaviour, accuracy, and cost are published, in isolation and in closed loop | B + T5, T6, V5, V6, C1–C4, D1–D3, I1, I2, I4, R1 |
| **A+** Validated | Shown fit for purpose against reality | A + T7, C5, L1–L5, I3, I5, I6 (where a consumer needs it), R2, R3 |

The grade sits at the top of the block's usage page and links to its V&V
plan. A grade can drop: a requirement that regresses, such as a sanitizer
finding or a stale datasheet, drops the block to the highest grade still fully
met.

## CI tiers

| Tier | Trigger | Runs |
|---|---|---|
| PR | Every push to a PR | Lint, compiler matrix, unit, verification, property tests (small N), one sanitizer preset, coverage report, golden regression, smoke scenarios |
| Nightly | Scheduled | Property tests (large N), fuzzing (time-boxed), cross-compiler golden comparison, numeric-type sweep, full scenario suite |
| Weekly | Scheduled | Mutation testing |
| Release | release-please PR | Regenerate the datasheet, run validation (open- and closed-loop), processor-in-the-loop run, assemble and publish the evidence bundle |

## Evidence rules

- Each claim on a datasheet or plan points to the harness that produces it
  and the CI tier that gates it (the block's *evidence ledger*).
- Each random process is seeded, and the seed is recorded with its output.
- Each tolerance is justified: derived from the scalar type's epsilon, the
  method's error constant, or a consumer requirement. It is never tuned until
  the test passes.
- Figures are committed SVGs generated from harness output, as on the
  [gyro filter page](/examples/gyro-lowpass/). The docs build does not run C++.

## References

- W. L. Oberkampf, C. J. Roy, *Verification and Validation in Scientific
  Computing*, Cambridge University Press, 2010.
- ASME V&V 10 (computational solid mechanics), V&V 20 (fluid dynamics and
  heat transfer), V&V 40 (credibility assessment by context of use).
- NASA-STD-7009, *Standard for Models and Simulations*: credibility factors
  and assessment.
- Modelica Association, *Functional Mock-up Interface 3.0*: model exchange
  and co-simulation.
- P. J. Roache, "Code Verification by the Method of Manufactured Solutions",
  *J. Fluids Eng.* 124(1), 2002.
- T. Y. Chen et al., "Metamorphic Testing: A Review of Challenges and
  Opportunities", *ACM Computing Surveys* 51(1), 2018.
