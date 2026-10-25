---
title: Cascaded Controller
description: branes::reflex::control::Cascade — position, attitude, and rate loops for multirotors, from state to wrench.
---

`#include <branes/reflex/control/cascade.hpp>`

**Grade: C** (specified and unit-tested) under the
[building block standard](/standards/building-blocks/). Its closed-loop
evidence is design verification on a nominal plant, not validation: see
[what the evidence shows](#what-the-evidence-shows).

`branes::reflex::control::Cascade<T>` is the multirotor flight controller
behind racing-drone's B0 baseline. It turns a vehicle state and a setpoint
into a body wrench (collective thrust and torque) for the
[mixer](/control/mixer/). The interface is plain state in and wrench out, so
the same cascade runs in SITL, in hardware-in-the-loop, and on the flight
controller.

```
PositionController ──(q_sp, T)──▶ AttitudeController ──(ω_sp)──▶ RateController ──(τ)──▶ Mixer
   position.hpp                      attitude.hpp                    rate.hpp
```

Each loop is its own header and can be used alone. The conventions are the
[quadrotor model's](/model/quadrotor/): world frame z up, body frame x
forward, y left, z up.

## Usage

```cpp
using namespace branes::reflex;
const model::QuadrotorParams<double> vehicle{};             // RP-1
const control::Mixer<double> mixer(control::mixer_params_from(vehicle));
control::Cascade<double> cascade(control::cascade_config_from(vehicle));

control::PositionSetpoint<double> sp;
sp.position = vec3(0.0, 0.0, 2.0);
// Every 1 ms:
const control::Wrench<double> w = cascade.update(state, sp, 1e-3);
const control::MixerOutput<double> out = mixer.mix(w);
cascade.feedback(out);  // the integrators see the mixer's saturation next tick
const model::RotorCommand<double> u = mixer.to_rotor_command(out.thrusts);
```

`update()` runs once per rate-loop tick. The attitude loop runs every
`attitude_divider` ticks and the position loop every `position_divider`
ticks (by default 1 kHz, 250 Hz, and 50 Hz), each holding its output in
between. A second overload takes an `AttitudeTarget` instead of a position
setpoint and skips the position loop (attitude mode).

## The loops

**Rate** (`rate.hpp`). One PID per body axis outputs angular acceleration,
and the torque is τ = J·α + ω × Jω. Because the gains act on acceleration,
they are in 1/s and one loop shape carries across vehicle sizes; the
ω × Jω term cancels the plant's gyroscopic coupling.

**Attitude** (`attitude.hpp`). Tilt-prioritized control after Brescianini and
D'Andrea. The error q_e = q⁻¹ ⊗ q_sp is split into a tilt q_red, which moves
the thrust axis, followed by a yaw q_yaw about it. Roll and pitch rates come
from the tilt alone and the yaw rate from the yaw alone, so the thrust axis
converges at its own gain whatever the heading does. The error magnitude
2·sin(θ/2) grows up to 180°, q and −q give the same output, and no Euler
angles appear.

**Position** (`position.hpp`). A position P loop makes a velocity setpoint
(limited horizontally and vertically), velocity PIDs make an acceleration,
and the specific force f = a + g·e₃ is saturated vertical first: the vertical
component is clamped to the thrust range, then the horizontal part to the
tilt limit and the remaining thrust. The attitude setpoint puts the body z
axis along f with the commanded heading, and the collective thrust is f
projected on the current body z axis, so altitude holds while the tilt
converges.

## Anti-windup

Each PID freezes its integral while its own output is saturated and the
error pushes further in. Two outer reports extend that:

- The mixer's `Saturation` report (`saturation_from()`): roll and pitch when
  the mixer scaled them down, yaw when it gave yaw up, and the collective
  when it shifted the thrust. `Pid::update()` takes the direction as
  `external_saturation` (+1 cut high, −1 cut low).
- The position loop's force limits: a velocity integrator freezes while its
  axis was cut on the previous tick.

## Default gains

`cascade_config_from(params)` designs the loops from the vehicle by bandwidth
separation, starting from the motor lag τ (ω_n = 1/τ):

| Loop | Gain | Why |
|---|---|---|
| Rate | k_p = 1/τ, k_d = 0.4, k_i = k_p·ω_n/10 | PD on the measurement places the closed loop at ω_n with ζ = 0.7 |
| Attitude | ω_n/5 roll and pitch, ω_n/10 yaw | Keeps overshoot bounded with slow motors, more inertia, and delay |
| Velocity | k_v = ω_n/14, k_i = k_v²/10, integral ≤ g/2 | The integral bound covers a 33 % hover-thrust error |
| Position | ω_n/32 | With the velocity loop, ζ ≈ 0.76 |

The header derives each choice. To use other gains, fill a `CascadeConfig`
directly; every config passes through a validator.

## What the evidence shows

The plant in the tests is `QuadrotorModel`, which has no independent
reference yet (its [V&V plan](/model/quadrotor-vv/) schedules one). So the
closed-loop results show that the cascade works on that model, not that it
flies a real vehicle. Three measures keep the claim honest:

1. **Pre-stated criteria.** The acceptance criteria were written into
   [reflex#20](https://github.com/branes-ai/reflex/issues/20) before the
   tests ran, and `tests/sdk/cascade_closed_loop.cpp` checks them as stated.
2. **Hand-derived reference plants.** Small rate steps on each axis, and a
   small vertical step, are run on the model and on linear plants derived by
   hand (inertia or mass behind a first-order motor lag) that share no code
   with it. The two must agree to 1 % of the step. This also checks the
   model's torque and thrust magnitudes.
3. **Margins over model error.** The cascade must stay stable with overshoot
   ≤ 25 % at all 64 vertices of a perturbation set: inertia ±30 %, mass
   ±20 %, motor lag ×0.5 to ×2, thrust coefficient ±10 %, and 0 or 4 ms of
   command delay. The rate loop's phase margin is computed analytically from
   its transfer function at the nominal point and at the slowest corner.

| Criterion (nominal plant) | Pass if |
|---|---|
| Hover hold, 10 s, from a disturbed start | Error < 1 cm after 2 s |
| 30° roll step | 90 % in ≤ 0.25 s, overshoot ≤ 10 %, within 2° by 0.6 s |
| 180° roll step | Within 2° by 1.5 s; −q flies identically |
| Tilt and yaw step | The tilt converges as fast as with no yaw step |
| 1 m step in x and in z | Overshoot ≤ 10 %, within 2 % by 3 s |
| Waypoint square, 10 m sides, ≤ 5 m/s | Cross-track RMS ≤ 0.3 m, max ≤ 0.6 m |
| 2 s of thrust saturation | Overshoot within 5 points of an unsaturated step |

Validation needs flight data. The scenarios move to the `sim/` closed-loop
harness when it exists, and comparing simulated with flown closed-loop
behaviour on RP-1 then validates the model and the controller together.

## Not covered

- Throttle-PID attenuation, I-term relax, and gain scheduling on dynamic
  pressure.
- INDI as an alternative rate loop.
- Drag compensation in the position loop; pass it in as acceleration
  feed-forward.
- Staleness handling of a collective-thrust-and-body-rates command.
- Feed-forward differentiated automatically from the setpoint.
- Sensor noise and filtering: the tests feed the controller the true state.

## Cost

`bench/cascade_bench` reports nanoseconds per tick, per scalar type, and per
loop on the host. These are latency characterization figures, not a timing
bound.
