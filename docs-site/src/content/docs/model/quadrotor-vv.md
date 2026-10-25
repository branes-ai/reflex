---
title: Quadrotor V&V Plan
description: The plan, harnesses, and evidence ledger that take QuadrotorModel from grade C to A+ under the building block standard, in isolation and in closed loop.
---

This plan takes [`QuadrotorModel`](/model/quadrotor/) to grade **A+** under
the [building block standard](/standards/building-blocks/). It lists where the
model stands against each requirement, the harnesses that produce the missing
evidence, the acceptance criteria, and a sequence of PRs.

**Scope.** The plan verifies, characterizes, and validates the model as
specified. It does not change the equations. Where validation shows a missing
physical effect (rotor drag, ground effect), the finding goes to the
[model backlog](#model-backlog-from-validation) and is handled as a separate
feature, with its own pass through this plan.

**Consumers and context of use.** racing-drone uses the model for SITL at
1–2 kHz, and the Phase 2–3 controllers will use it as the MPC prediction
model over horizons of roughly 0.1–1 s. The SITL use needs robustness and
determinism. The MPC use needs accurate Jacobians and accurate multi-step
prediction. The acceptance criteria below follow from those two uses.

## Where it stands

Assessment on 2026-10-25, against commit `0b42690`.

| Req | Status | Evidence today / gap |
|---|---|---|
| S1 spec | ✅ | Equations, conventions, I1–I5, `step()` contract in `quadrotor.hpp` |
| S2 envelope | ❌ | Non-modelled effects and expected ranges not stated |
| S3 parameters | ✅ | `quadrotor_params_valid`, units, RP-1 defaults from `vehicles/rp1.yaml` |
| S4 scalar types | ❌ | I2 tolerance hard-coded to 1e-6 (see [findings](#findings-that-shape-the-plan)); only float and double tested |
| S5 resource | ◐ | Pure, no allocation, no exceptions, documented in the header; determinism tested. No worst-case execution-time bound stated or tested: the bench reports a mean |
| T1 unit | ✅ | `tests/sdk/quadrotor_model.cpp`, 12 cases |
| T2 matrix | ✅ | GCC/Clang Debug+Release, MSVC |
| T3 sanitizers | ❌ | No sanitizer preset |
| T4 properties | ❌ | — |
| T5 fuzzing | ❌ | — |
| T6 coverage | ❌ | Not measured |
| T7 mutation | ❌ | — |
| V1 analytic | ◐ | Hover, free fall, motor lag are exact. Torque and drag are checked for sign only, not magnitude |
| V2 order | ◐ | Probed once (below); not in CI |
| V3 structure | ◐ | Torque-free energy and angular momentum; unit quaternion. No dissipation check |
| V4 metamorphic | ❌ | — |
| V5 derivatives | ❌ | No Jacobian, no linearization check |
| V6 code-to-code | ❌ | — |
| V7 stability | ◐ | dt ≤ τ derived (RK4 limit ≈ 2.785 τ); not confirmed empirically |
| C1–C5 | ◐ | `bench/quadrotor_bench` gives ns/step on x86; nothing else |
| D1–D3 | ❌ | — |
| L1–L5 | ❌ | — |
| BB-I1–I6 closed loop | ❌ | No closed-loop harness, scenarios, or SITL-to-flight comparison |
| R1, R4 | ◐ | release-please semver; no golden traces |
| R2, R3 | ❌ | — |

**Grade today: below C.** Every C requirement except S5 is met. S5 lacks a
stated and tested worst-case execution-time bound, which M1 adds.

The integration requirements are written BB-I1 to BB-I6 throughout, so
they are not confused with the model's invariants I1–I5.

### Baseline: observed order of accuracy

A one-off probe on 2026-10-25 used non-symmetric inertia, linear and
quadratic drag, initial rates (2, −1, 3) rad/s, and a constant differential
command. It ran for 0.64 s against a reference at dt = τ/1024 (double, GCC
`-O2`, contracts compiled out). The 20 ms row is outside `step()`'s contract
(dt ≤ 10 ms) and is shown only as the first point of the sweep. The orders
from 5 ms down use in-contract points only.

| dt | position err | p | body-rate err | p |
|---|---|---|---|---|
| 20 ms (out of contract) | 1.58e-5 | — | 5.79e-5 | — |
| 10 ms | 6.97e-7 | (4.50) | 2.46e-6 | (4.55) |
| 5 ms | 3.73e-8 | 4.23 | 1.30e-7 | 4.24 |
| 2.5 ms | 2.17e-9 | 4.10 | 7.49e-9 | 4.12 |
| 1.25 ms | 1.29e-10 | 4.07 | 4.51e-10 | 4.05 |
| 0.625 ms | 7.87e-12 | 4.04 | 2.98e-11 | 3.92 |

RK4 keeps its fourth order with the per-stage quaternion renormalization
(commit `0b42690`). The probe also showed two traps the harness must avoid.
The `acos` attitude-error metric saturates near 1e-8 rad, and the rotor
speeds settle to the command (32 τ) long before the end, leaving only
roundoff. Both are covered in [WS2](#ws2-verification).

## Findings that shape the plan

1. **I2 cannot hold for 16-bit types.** `is_unit(q, T(1e-6))` is a fixed
   tolerance. bfloat16 and posit16 resolve roughly 1e-2 to 1e-3 near 1, so
   every state fails I2 and Debug builds assert. Decide in WS1: scale the
   tolerance with `std::numeric_limits<T>::epsilon()`, or set a 32-bit floor
   in S4. The number-type sweep (C3) needs this decided first.
2. **`step()` is not smooth everywhere.** The post-RK4 rotor clamp, the
   monotone energy clamp, and ground contact are non-differentiable. An MPC
   should linearize `derivative()` in free flight, or `step()` away from those
   branches. V5 verifies the smooth regime and documents the non-smooth one.
3. **No invariant diagnostics.** `quadrotor_state_valid` returns a bool. The
   invariant monitor (D2) needs to know which invariant failed and on which
   field. Add a `quadrotor_state_violation()` that returns an enum, and keep
   the bool as a wrapper.
4. **Contact is a stop, not a contact model.** A vehicle that reaches the
   ground keeps its attitude, even inverted, and a grounded vehicle whose
   vertical thrust never exceeds its weight stays put. That is right for
   takeoff and idle in SITL, and wrong for landing or crash studies. Record
   this in S2.
5. **Bit-identical results hold within one build only.** GCC defaults to
   `-ffp-contract=fast` in GNU mode, so on targets with FMA the same source
   can round differently from Clang or MSVC. The determinism test holds per
   build. Cross-compiler agreement needs a tolerance, or `-ffp-contract=off`
   for consumers that need bitwise replay (see [decisions](#decisions-needed)).

## Workstreams

### WS1. Specification completion (S2, S4)

- Add a **Validity envelope** section to the header and the usage page:
  - Not modelled: rotor drag and blade flapping, induced-velocity thrust loss
    at speed, ground effect, motor saturation dynamics beyond the clamp,
    contact dynamics, wind, and aerodynamic interference between rotors.
  - Expected envelope until L4 replaces it: airspeed below about 5 m/s, body
    rates below about 10 rad/s, rotor speeds between 10 % and 95 % of ω_max.
- Resolve finding 1. Either the I2 tolerance becomes `k · epsilon<T>()` with
  `k` derived from the error of one normalize plus one RK4 step, or S4 states
  that the minimum is 24-bit significand precision. Document the choice and
  test it.
- Add `quadrotor_state_violation()` (finding 3).
- Complete S5 with a worst-case execution-time statement. `step()` has a
  fixed operation count: four `derivative()` calls, no data-dependent loops,
  and one contact branch. Its time is bounded except for subnormal operands,
  which are slow on x86. Either state the bound with flush-to-zero required
  of the caller, or measure the subnormal case. Extend `quadrotor_bench` to
  report p99.9 and maximum latency over generated valid states, including
  values near the subnormal range, alongside the mean.

### WS2. Verification

New directory `tests/verification/`, picked up by `compile_all` and gated on
every PR.

**`quadrotor_analytic.cpp` (V1).** Each term of the equations gets a
closed-form magnitude check:

| Case | Closed form |
|---|---|
| Vertical climb with linear drag | $v_z(t) = v_\infty\,(1 - e^{-c t/m})$ with $v_\infty = (\Sigma T - mg)/c$, $c$ = `drag_linear[2]` |
| Terminal velocity, quadratic drag | $v_\infty = \sqrt{(\Sigma T - mg)/c_2}$ |
| Single-axis roll spin-up from rest | Left rotors (1, 2) at $T_h + \Delta T$, right rotors (0, 3) at $T_h - \Delta T$, already at speed: $\dot\omega_x = 4 a\,\Delta T / J_{xx}$ with $a = L/\sqrt2$, no yaw torque, and $\phi(t) = \tfrac12 \dot\omega_x t^2$ |
| Pure yaw spin-up | $\dot\omega_z = -k_m\,\Sigma_i s_i T_i / J_{zz}$ |
| Battery energy | $E(t) = \Sigma k_p \omega_i^3\, t$ at constant rotor speed |
| Sag | Hover speed at energy $E$ equals $\omega_h/\sqrt{s(E)}$ |

**`quadrotor_convergence.cpp` (V2).** The convergence harness, hardened by
the probe:

- The command is constant, or piecewise constant with switches on multiples
  of the coarsest dt. Under a zero-order hold, a command that changes inside
  a step measures the hold, not the integrator.
- The test samples at t = 2τ, so the rotor speeds are still moving.
- Attitude error is $2\,\mathrm{asin}(\lVert \mathrm{vec}(q_a^{-1} \otimes q_b)\rVert)$,
  never `acos` of a dot product.
- Sweep dt over h₀, h₀/2, …, h₀/32 with h₀ = min(10 ms, τ), so that every
  point is inside `step()`'s contract, against h₀/1024. Fit p on pairs whose
  error exceeds 1e3 × the roundoff floor.
- Pass: p ∈ [3.7, 4.4] for position, velocity, attitude, rates, and rotor
  speeds, in double. Float runs the same sweep with a floor at 1e3 ε.

**`quadrotor_structure.cpp` (V3).**

- Torque-free rotation conserves kinetic energy and angular momentum (moved
  from `tests/sdk`).
- With zero thrust and zero drag, mechanical energy $\tfrac12 m v^2 + mgz$ is
  conserved during free fall.
- Drag dissipates: level, at hover thrust, with an initial horizontal
  velocity, the horizontal speed decreases monotonically.
- The quaternion norm stays within the I2 tolerance over 10⁶ steps.

**`quadrotor_metamorphic.cpp` (V4).** Each relation runs from a random
airborne state through 0.5 s of random piecewise-constant commands:

| Relation | Transformation | Expected |
|---|---|---|
| World yaw | Rotate the initial state by ψ about world z | Trajectory rotated by ψ; rates and rotor speeds unchanged |
| Translation | Shift x, y (and z above ground) | Trajectory shifted |
| Body 180° yaw | Swap commands and initial rotor speeds 0↔1 and 2↔3; rotate the initial attitude by π about body z and negate the initial x, y rates | Same world trajectory; body x, y rates negated; rotor speeds permuted |
| Mirror | Reflect y → −y; swap commands and initial rotor speeds 0↔2 and 1↔3; q → (w, −x, y, −z) | Mirrored trajectory; roll and yaw rates negate; rotor speeds permuted |
| Mass scaling | Scale m, J, k_f, `max_thrust`, and drag by α (ω_max unchanged) | Identical kinematics; energy excluded unless k_p is scaled too |

The tolerance is a few hundred ε times the trajectory norm. Rotations
reorder floating-point operations, so these are not bitwise.

**`quadrotor_jacobian.cpp` (V5).** Linearize `derivative()` at a point.
Perturb the attitude in the tangent space, $q \otimes \exp(\delta\theta/2)$,
so the state has 16 error coordinates: δp, δv, δθ, δω, δΩ (energy
excluded). The closed form below is at identity attitude. Then check three independent ways:

1. Central differences with a step sweep, using the plateau value.
2. Forward-mode dual numbers. A small `Dual<double>` test type exercises the
   template genericity at the same time.
3. At hover, with no drag, the hand-derived linearization:

$$
\delta\dot v = \begin{pmatrix} g\,\delta\theta_y \\ -g\,\delta\theta_x \\ \tfrac{2 k_f \omega_h}{m}\sum_i \delta\Omega_i \end{pmatrix},\quad
\delta\dot\theta = \delta\omega,\quad
\delta\dot\omega = J^{-1} M\,\delta\Omega,\quad
\delta\dot\Omega = \tfrac{1}{\tau}(\delta u - \delta\Omega)
$$

with mixer column $M_{:,i} = 2 k_f \omega_h\,(r_{y,i},\,-r_{x,i},\,-s_i k_m)^\top$.
The spectrum is 12 zeros and 4 at $-1/\tau$. Linear drag moves the velocity
modes to $-D_{\mathrm{lin}}/m$. Pass: FD and dual agree to 1e-7 relative, and
dual matches the closed form to 1e-12.

The test also checks the discrete step Jacobian against $\exp(A\,dt)$ (Taylor
series, 16×16) to $O(dt^5)$ at hover. The MPC may use either.

**`quadrotor_stability.cpp` (V7).** The stability limit lies outside
`step()`'s contract (dt ≤ min(10 ms, τ)), so `step()` cannot probe it
without tripping its precondition. The test drives the motor-lag equation
from `derivative()` through a test-only RK4 driver, and checks that the
driver matches `step()` bit for bit inside the contract. For a pure motor
step, bisect the largest dt at which the rotor error decays. Pass: the limit is within 2 % of
2.785 τ, which confirms the contract's dt ≤ τ margin. Confirm also that for
dt ≤ τ the rotor response never overshoots (RK4's amplification factor at
z = −1 is 0.375 > 0).

### WS3. Robustness (T3–T7)

- **Sanitizers.** Add a `clang-asan-ubsan` preset with
  `-fsanitize=address,undefined -fno-sanitize-recover=all` and a CI job that
  runs the full suite under it.
- **Property tests** (`tests/property/quadrotor_properties.cpp`). Use
  RapidCheck for generators and shrinking (BSD-2, permissive). Generators
  draw parameters within ±50 % of RP-1 and projected valid; airborne and
  grounded valid states; commands in [−0.2, 1.2]·ω_max (exercising the
  clamp); dt in (0, min(10 ms, τ)]. Properties:
  1. `step()` preserves I2–I5.
  2. Purity: the same inputs give the same bits, in any call order.
  3. Energy is non-decreasing.
  4. Rotor speeds move toward the clamped command without overshoot.
  5. Ground contact: a grounded vehicle with sub-weight vertical thrust
     keeps its position and attitude, has zero velocity and body rates, and
     stays grounded. It is not a fixed point of the full state: its rotor
     speeds still follow the command (property 4) and its energy still
     accrues (property 3).
  6. `derivative()` is finite for every valid state.
  7. Validators reject NaN, ±inf, and each bound by one ulp. They also accept
     −0 and subnormals where the bound allows.

  PR runs use 10⁴ cases, nightly runs 10⁶.
- **Fuzzing** (`fuzz/quadrotor_step_fuzz.cpp`). libFuzzer maps bytes to
  parameters, state, command, and dt, projects them valid, and checks the
  postcondition under ASan and UBSan. A second target feeds raw bytes to the
  validators, which must return false and never trap. Commit a seed corpus.
  Nightly runs are time-boxed to 30 minutes; findings become regression cases
  in `tests/sdk`.
- **Coverage.** `llvm-cov` on the Clang Debug run, reported per PR. Targets
  are from BB-T6.
- **Mutation.** Mull on the model header, weekly. Each surviving mutant gets
  a test or a written equivalence argument in the plan's appendix.

### WS4. Code-to-code reference (V6)

- The primary reference is **RotorPy** (UPenn, Python). Configure it to
  match the equations here: body drag only, rotor drag and other aerodynamic
  extras off, same motor model. `tools/reference/rotorpy_golden.py` writes
  golden CSVs for 8 scenarios into `tests/verification/golden/`, recording
  the RotorPy version.
- The frame and rotor-order mapping (PX4 Quad X, z up) is its own test: a
  single-rotor step must produce the same torque sign in both.
- **PX4 SIH** is the second reference for the hover and attitude cases,
  because PX4 is what racing-drone's autopilot side will compare against.
- Pass: position within 1e-6 m and attitude within 1e-6 rad over 2 s at dt =
  1 ms. Any larger disagreement is investigated to its cause, which may be in
  either implementation.
- Map the equations term by term before comparing. For example, if the
  reference's parasitic drag is $\lVert v\rVert v$ rather than the per-axis
  $v \odot \lvert v\rvert$ used here, scenarios with quadratic drag compare
  only along a single body axis.

### WS5. Characterization and datasheet (C1–C5)

Characterization executables live in `characterize/quadrotor/` and write
JSON into `REFLEX_CHAR_DIR`.
`docs-site/scripts/gen-quadrotor-figures.mjs` renders the SVGs into
`docs-site/src/figures/quadrotor/`, following the gyro-filter pattern. The
datasheet page is `model/quadrotor-datasheet.mdx`.

| ID | Figure or table | Content |
|---|---|---|
| C1 | Header block | Commit, compiler, flags, RP-1 parameter set |
| C2a | Motor step response | Rotor speed against time; τ marked |
| C2b | Hover linearization | A and B matrices, eigenvalues, controllability rank |
| C2c | Frequency response | Bode plots of rotor command to roll, pitch, yaw rate, and vertical acceleration at hover; motor pole visible |
| C2d | Operating map | Hover rotor speed against mass and battery state; thrust-to-weight; maximum climb acceleration; maximum angular acceleration per axis |
| C3a | Convergence | Error against dt, log-log, per state group, with slope-4 guide |
| C3b | Stability | Rotor-error decay against dt/τ; limit marked |
| C3c | Numeric types | float, posit⟨32,2⟩, cfloat binary32, and (if WS1 allows) posit⟨16,1⟩ and bfloat16 against double over 0.1, 1, and 10 s hover-and-maneuver runs; invariant violations per type |
| C4 | Cost | ns/step per type for GCC, Clang, MSVC; `-fstack-usage` stack bytes; object code size of `step()` |
| C5a | Sensitivity | Normalized sensitivity of 1 s position and attitude to each parameter at hover and in a 3 m/s maneuver |
| C5b | Uncertainty | Monte Carlo trajectory spread from the identified parameter covariance (after WS7) |

The datasheet is regenerated on every release (R2). A release fails if any
committed figure is older than the model header.

### WS6. Debug harness (D1–D3)

All of it lives in `tools/` and links against the SDK; nothing is added to
the SDK headers except `quadrotor_state_violation()`.

- **Scenario runner, `tools/quadrotor_sim`.** A JSON scenario sets
  parameter overrides, the initial state, dt, the duration, and a command
  schedule, or a closed loop built from the `sim/` harness in
  [WS9](#ws9-closed-loop-virtual-environment-bb-i1i6). The
  runner writes a CSV trace and, optionally, MCAP using foxglove/mcap's
  header-only C++ writer (MIT). The traces open in Foxglove or Rerun for 3-D
  playback.
- **Invariant monitor.** A wrapper steps the model and checks
  `quadrotor_state_violation()` after every step. On the first violation it
  writes a repro file: parameters, state, command, and dt as hex floats, plus
  the step index and the violation.
- **Replay.** `quadrotor_sim --replay repro.json` re-executes one step
  bit-identically under a debugger.
- **Trace diff, `tools/trace_diff`.** Given two traces and per-field
  tolerances, it reports the first divergent step and field. It is used by
  R4 golden regression and when a compiler or flag change alters results.

### WS7. Validation (L1–L5)

**Data (L1)**

| Dataset | Why | Has rotor speeds |
|---|---|---|
| NeuroBEM (Bauersheim, Foehn, Scaramuzza, RSS 2021) | Aggressive racing-quad flight: mocap pose, IMU, motor speeds | Yes |
| Blackbird (MIT, 2018) | A second vehicle, moderate to aggressive flight | Yes |
| RP-1 flight logs (own) | Validates the shipped parameter set: Betaflight blackbox with bidirectional-DShot RPM, plus mocap or an onboard estimate | Yes |

Confirm each dataset's channels and units before M9 depends on them.
`validate/quadrotor/fetch.py` downloads each dataset with checksums into a
git-ignored cache. Check each license before any derived data is committed.

**Pipeline**

1. **Preprocessing.** Align time, differentiate mocap with a documented
   smoother, and map frames and rotor order to the model's conventions.
   Prove the mapping with the single-rotor torque-sign check from WS4.
2. **Identification (L2).** `validate/quadrotor/sysid.py` fits k_f, k_m, J,
   drag, and τ by least squares on force and torque residuals, split by
   flight 70/30. It reports the parameter covariance, which feeds C5b.
3. **Replay.** The C++ executable `validate/quadrotor/replay` reads each
   flight segment and outputs model predictions. The model is never
   re-implemented in Python. It has two modes:
   - **Measured-state replay** isolates the rigid body and aerodynamics.
     Measured rotor speeds must not be passed as `RotorCommand`, because
     `step()` treats commands as motor-lag targets and the replayed thrust
     would lag the log. Instead, a replay driver integrates `derivative()`
     with RK4, sets `QuadrotorState::rotor_speeds` at each stage from the
     log, interpolated, and discards the rotor-speed derivative. A test
     checks that the driver agrees with `step()` when the logged speeds are
     constant and equal to the command.
   - **Command replay** calls `step()` with the logged commands, and so
     evaluates the motor model as well.
4. **Metrics (L3).** Two families:
   - **Force and torque residuals.** Accelerometer specific force against
     $F_b/m$, and differentiated gyro against $\dot\omega$: RMSE and NRMSE,
     binned by airspeed, body rate, and thrust.
   - **Open-loop prediction.** From each sample, roll the model forward in
     measured-state replay and, where commands are logged, in command
     replay.
     Report position, velocity, and attitude error percentiles at 50, 100,
     250, and 500 ms.
5. **Envelope (L4).** The region of (airspeed, body rate, thrust) where the
   acceptance criteria hold, published as a figure on the datasheet. Outside
   it the page says "unvalidated".

**Acceptance criteria (L5).** These are provisional and fixed before the
first evaluation run. They are revised only from the MPC's tolerance once
Phase 2 defines it, and never from the results:

| Metric (inside the envelope, held-out flights) | Target |
|---|---|
| Specific-force NRMSE | ≤ 10 % |
| Angular-acceleration NRMSE | ≤ 15 % |
| 100 ms prediction, position error p95 | ≤ 3 cm |
| 250 ms prediction, attitude error p95 | ≤ 3° |
| RP-1 hover rotor speed against log median | ≤ 3 % |

### WS8. Release and consumers (R1–R4)

- **R4 golden traces.** Eight scenarios in double, compared with trace diff
  on every PR. Changing a golden file needs a commit that explains the
  numerical change.
- **R2 evidence bundle.** The release workflow runs the Release tier and
  attaches `quadrotor-evidence-<version>.tar.gz`: CTest JUnit XML, coverage,
  mutation report, datasheet SVGs and JSON, validation report, and the
  credibility scorecard below. The datasheet goes to the docs site.
- **R3 consumer contract.** racing-drone adds
  `tests/contract/quadrotor_contract.cpp`. It pins hover speed for its
  vehicle files, the rotor order and spin, the frames, and a 1 s golden
  trajectory against the pinned reflex version.
- **Credibility scorecard.** Each of NASA-STD-7009's credibility factors
  (verification, validation, input pedigree, results uncertainty, results
  robustness, use history, M&S management, people qualifications) is scored
  0–4, with a link to its evidence. A+ requires ≥ 3 on verification,
  validation, input pedigree, and results uncertainty.

### WS9. Closed-loop virtual environment (BB-I1–I6)

The model has two roles in closed loop, and the harness serves both:

- **As the plant, it is what is being validated.** SITL is one of its two
  uses, so it must reproduce the vehicle's *closed-loop* behaviour, not only
  its open-loop physics. Feedback changes which model errors matter. A wrong
  inertia barely shows under a stiff rate loop, while an error of a few
  milliseconds in motor lag or command latency moves the phase margin and
  the oscillation frequency. Open-loop prediction error (WS7) cannot detect
  that.
- **As the fixture, it tests the systems under test.** Reflex's PID cascade
  now, and the MPC and estimators later, are validated in this loop. Their
  closed-loop claims are only as credible as the plant, so the controller V&V
  plans depend on this model reaching B first.

**Harness (BB-I1): `sim/`.** It is a lockstep co-simulation with an explicit
rate for each component:

| Component | Content | Rate |
|---|---|---|
| Plant | `QuadrotorModel<double>` | 2 kHz (dt = 0.5 ms) |
| IMU model | White noise and bias random walk from the RP-1 IMU datasheet; quantization; filter group delay; sample latency | 1–8 kHz, configurable |
| Actuator model | ESC command update rate, DShot quantization (2000 steps), command latency | Loop rate |
| Controller under test | Reflex PID cascade (rate, attitude, velocity, position); MPC later | Its own rate, with a scheduler that injects jitter |
| State source | Truth, truth plus noise, or reflex's IMU estimator once it exists | Configurable |
| Environment | Constant wind; Dryden turbulence; gust steps | Plant rate |

It is deterministic and seeded: the same scenario and seed give the same
trace. The scenario runner from WS6 is its front end.

**Scenario suite (BB-I2): `sim/scenarios/quadrotor/`.** Each scenario has
two kinds of pass criteria. *Plant criteria* are I2–I5 at every step, energy
monotone, and no clamp or contact chatter. *Closed-loop metrics* are
recorded for BB-I3.

| Scenario | Exercises |
|---|---|
| Hover hold, 10 s, with sensor noise | Equilibrium, noise propagation |
| Roll and pitch steps ±20°; yaw step 90° | Torque paths, mixer, motor lag in the loop |
| Vertical step 1 m; full-throttle punch-out | Thrust path, rotor saturation |
| Figure-8 tracking at 3, 6, 10 m/s | Drag, coupling; the edge of the expected envelope |
| Flip (360° roll) | Large-angle attitude, saturation recovery |
| Wind 5 m/s plus gust | Drag under disturbance |
| Battery depletion | Sag, and that the loop compensates |
| Takeoff and landing | Ground contact entry and exit |
| One rotor to zero at hover | Plant stays valid; the expected loss of yaw control appears |
| IMU dropout, 50 ms | Plant under an open-loop interval |

Smoke runs (hover, one step, takeoff) gate every PR; the full suite runs
nightly.

**Closed-loop validation (BB-I3).** In the RP-1 flight campaign from WS7, fly
the same controller and gains as the simulation, and the same maneuvers:
attitude steps, vertical steps, figure-8. Compare:

| Metric | Provisional acceptance |
|---|---|
| Attitude-step rise time and settling time | Within 15 % of flight |
| Attitude-step overshoot | Within 5 percentage points |
| Dominant attitude-oscillation frequency and damping ratio (from rate-loop sweeps or chirps) | Within 10 % |
| Figure-8 tracking RMSE | Within 25 % |
| Fraction of time any motor is saturated | Within 5 percentage points |

If the closed-loop metrics fail while open-loop prediction (WS7) passes, the
fault is usually in the latency chain (IMU, filter, scheduler, ESC), not in
the rigid body. That is why the sensor and actuator models are part of the
harness, and why their latencies are measured on RP-1, not assumed.

**Timing (BB-I4).** Lockstep mode is the default and CI-gated. A real-time
mode runs the loop against the wall clock at 2 kHz and reports jitter,
overruns, and the model's share of each tick.

**Processor in the loop (BB-I5).** Cross-compile the model for the RP-1
flight controller's MCU, run the R4 golden scenarios on the target over a
serial or semihosting link, and compare the traces with the host's using
`trace_diff`. The tolerance depends on decision 2 (FMA policy). Report cycles
per `step()` for float and double on the target; this is the real cost entry
for the datasheet's C4. Hardware-in-the-loop with the real flight
controller belongs to racing-drone.

**Interop (BB-I6).** racing-drone's flight stack must be testable against
this plant without re-implementing it. Provide one bridge (see decision 6):
a PX4 SITL simulator bridge, a Gazebo system plugin, or an FMI 3.0
co-simulation FMU. The bridge has a convention test: a single-rotor step
gives the same torque sign through it as through the native API.

## Evidence ledger

| Claim | Evidence | Harness | Gate |
|---|---|---|---|
| RK4 is fourth order on the full model | p ∈ [3.7, 4.4], all state groups | `tests/verification/quadrotor_convergence` | PR |
| Each equation term has the right magnitude | Closed-form cases | `tests/verification/quadrotor_analytic` | PR |
| Physics symmetries hold | Five metamorphic relations | `tests/verification/quadrotor_metamorphic` | PR |
| Jacobians are correct | FD, dual, and closed form agree | `tests/verification/quadrotor_jacobian` | PR |
| dt ≤ τ is a safe step bound | Measured limit 2.785 τ ± 2 % | `tests/verification/quadrotor_stability` | PR |
| Invariants hold for all valid inputs | 10⁶ property cases; fuzzing with no findings | `tests/property`, `fuzz/` | Nightly |
| No undefined behaviour | Clean ASan and UBSan run | `clang-asan-ubsan` preset | PR |
| The tests detect faults | Mutation score ≥ 80 % | Mull | Weekly |
| Agrees with an independent model | Within 1e-6 over 2 s | `tests/verification/golden` (RotorPy) | PR |
| Numeric-type error budget | C3c figure | `characterize/quadrotor/numeric_types` | Release |
| Matches real flight within its envelope | L5 table met on held-out flights | `validate/quadrotor` | Release |
| Results reproduce | Golden traces; repro and replay | `tools/trace_diff`, `tools/quadrotor_sim` | PR |
| Valid as a closed-loop plant across the scenario suite | Plant criteria on 10 scenarios | `sim/scenarios/quadrotor` | PR (smoke), Nightly (full) |
| SITL reproduces closed-loop flight | BB-I3 table met on RP-1 flights | `sim/` + `validate/quadrotor` | Release |
| Behaves identically on the flight MCU | PIL traces within tolerance; cycles per step | `sim/pil` | Release |

## Milestones

Each milestone is one or two PRs and ends at a grade.

| # | PR scope | Workstreams | Grade after |
|---|---|---|---|
| M1 | Envelope section, I2 tolerance decision, `quadrotor_state_violation()`, worst-case timing statement and bench | WS1 | **C** |
| M2 | `tests/verification/`: analytic, convergence, structure, stability, metamorphic | WS2 (V1–V4, V7) | C |
| M3 | Sanitizer preset and job, RapidCheck property suite, golden traces with `trace_diff` | WS3 (T3, T4), WS6 (D3), WS8 (R4) | **B** |
| M4 | Jacobian harness with the dual-number type | WS2 (V5) | B |
| M5 | Fuzz targets, nightly workflow, coverage report | WS3 (T5, T6) | B |
| M6 | RotorPy and PX4 SIH golden references | WS4 (V6) | B |
| M7 | `characterize/quadrotor`, figure script, datasheet page | WS5 (C1–C4) | B |
| M8 | Scenario runner, invariant monitor, replay, MCAP | WS6 (D1, D2) | B |
| M9 | `sim/` harness: sensor, actuator, and environment models; PID cascade in the loop; scenario suite; real-time mode | WS9 (BB-I1, BB-I2, BB-I4) | **A** |
| M10 | Dataset fetch, preprocessing, sysid, replay, metrics, envelope; closed-loop sim-to-flight comparison | WS7, WS9 (BB-I3) | A |
| M11 | Processor-in-the-loop on the RP-1 MCU; the bridge for racing-drone | WS9 (BB-I5, BB-I6) | A |
| M12 | Mutation testing, uncertainty propagation, evidence bundle, scorecard, racing-drone contract test | WS3 (T7), WS5 (C5), WS8 (R2, R3) | **A+** |

M1–M3 come first: they are cheap and they reach grade B. M4 comes next
because the Phase 2 MPC work depends on it. M9 also unblocks the PID
cascade's own closed-loop validation. M10 is the long pole, because it
depends on data access and RP-1 flight time. Plan the flight campaign once,
covering both open-loop data (WS7) and closed-loop maneuvers (BB-I3), and
start dataset fetching and preprocessing alongside M4–M9.

## Decisions needed

1. **Minimum scalar precision.** Scale the I2 tolerance with epsilon so that
   16-bit types are supported, or set a 32-bit floor (finding 1).
2. **Floating-point contraction policy.** Ship `-ffp-contract=off` on the
   `branes::reflex` target for bitwise cross-compiler replay, or accept
   tolerance-level agreement and keep the speed of FMA.
3. **New dependencies.** RapidCheck (tests only), foxglove/mcap (tools
   only), RotorPy (offline golden generation only). All are permissive
   licenses and none reaches the SDK interface.
4. **Validation compute and data.** Whether the Release-tier validation runs
   on GitHub-hosted runners with cached data, or on a self-hosted runner.
5. **RP-1 flight-test campaign.** Who flies it, the instrumentation (RPM
   telemetry and mocap), and the maneuver set that covers the envelope.
   The same campaign must fly the BB-I3 closed-loop maneuvers with logged
   gains.
6. **Interop bridge.** PX4 SITL bridge, Gazebo plugin, or FMI 3.0 FMU,
   whichever racing-drone's stack actually runs against.
7. **Location of `sim/`.** In reflex as a reference closed loop for every
   block (recommended), with mission scenarios such as gates and courses in
   racing-drone.

## Model backlog from validation

Effects that validation is expected to expose. Each one becomes a model
feature with its own pass through this plan, not a change made inside it:

- Rotor drag and blade flapping, proportional to rotor speed times in-plane
  airspeed. This is the dominant missing term at racing speeds in the
  NeuroBEM results.
- Thrust loss from induced velocity in fast forward flight and climb.
- Ground effect below about two rotor diameters.
- An asymmetric motor response (spin-up faster than spin-down) and
  voltage-dependent saturation.
- A contact model with friction and tip-over, if landing studies need it.
