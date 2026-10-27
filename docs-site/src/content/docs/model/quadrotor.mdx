---
title: Quadrotor Model
description: branes::reflex::model::QuadrotorModel — rigid-body quadrotor dynamics for SITL and MPC.
---

`#include <branes/reflex/model/quadrotor.hpp>`

**Grade: C** (specified and unit-tested) under the
[building block standard](/standards/building-blocks/). The
[V&V plan](/model/quadrotor-vv/) covers the path to A+.

`branes::reflex::model::QuadrotorModel<T>` is the quadrotor model behind
software-in-the-loop simulation and, later, the model-predictive controllers.
It is a pure function of (state, command, dt): the same inputs give
bit-identical outputs. It never allocates and never throws, and it is templated
on the scalar type.

## Conventions

- World frame: z up. Body frame: x forward, y left, z up.
- Rotor order (PX4 Quad X): 0 front-right CCW, 1 rear-left CCW, 2 front-left CW,
  3 rear-right CW, spin as seen from above.
- A CCW rotor pushes the body CW: its yaw torque is −k_m·T.

## Dynamics

| Quantity | Equation |
|---|---|
| Rotor speed | ω̇ᵢ = (clamp(uᵢ, 0, ω_max) − ωᵢ) / τ |
| Thrust | Tᵢ = s · k_f · ωᵢ², with battery scale s = 1 − sag · min(E / capacity, 1) |
| Position | ṗ = v |
| Velocity | v̇ = R(q) F_b / m − g e_z, with F_b = (0, 0, ΣTᵢ) − D_lin ⊙ v_b − D_quad ⊙ v_b ⊙ \|v_b\|, v_b = Rᵀv |
| Attitude | q̇ = ½ q ⊗ (0, ω) |
| Body rates | ω̇ = J⁻¹ (τ − ω × Jω), τ = Σ rᵢ × (0, 0, Tᵢ) + (0, 0, −spinᵢ · k_m · Tᵢ) |
| Energy | Ė = Σ k_p ωᵢ³ |

`step()` integrates with classical RK4 and renormalizes the attitude. It then
applies ground contact. A grounded vehicle stays put while its world-vertical
thrust does not exceed its weight, and a falling vehicle stops at `ground_z`.

## Validity envelope

Not modelled:
- rotor drag and blade flapping;
- induced-velocity thrust loss in fast forward flight and climb;
- ground effect;
- motor dynamics beyond the first-order lag and the speed clamp;
- contact dynamics: contact is a stop, with no friction, bounce, or tip-over;
- wind, and aerodynamic interference between rotors.

Expected envelope, until validation replaces it with a measured one:
airspeed below about 5 m/s, body rates below about 10 rad/s, and rotor
speeds between 10 % and 95 % of ω_max. Outside it the model is unvalidated.

## Invariants

| ID | Invariant | Reported as |
|---|---|---|
| I1 | Every parameter finite and within its bounds | `quadrotor_params_valid()` is false |
| I2 | Unit attitude: \|‖q‖² − 1\| ≤ 8·ε of the scalar type | `StateViolation::attitude_not_unit` |
| I3 | 0 ≤ ωᵢ ≤ ω_max, energy ≥ 0 | `rotor_speed`, `energy` |
| I4 | Every value finite | `non_finite` (checked first) |
| I5 | position.z ≥ ground_z | `below_ground` |

`quadrotor_state_violation(x)` returns the first violated state invariant,
and `invariant_id()` names it. `quadrotor_state_valid(x)` is the same check
as a bool.

## Scalar types

The I2 tolerance scales with the scalar type's epsilon, so I2–I5 hold at any
precision. The tests run them in double, float, posit<16,1>, and bfloat16.
Every parameter must be a normal number of the type: IEEE binary16 cannot
represent RP-1's k_f = 8.9e-7, so `quadrotor_params_valid()` rejects it.
How accurate the model is at reduced precision is a characterization result,
not a guarantee.

## Timing

`step()` never allocates, throws, or makes system calls. It runs four
`derivative()` calls and has one data-dependent branch, ground contact, so
its operation count is fixed for each contact arm. A test counts the
operations with an instrumented scalar over generated states. Subnormal
operands are slow on x86 and some FPUs, so a caller that needs a time bound
runs with flush-to-zero. A numeric bound is specific to a processor, and is
measured on the flight MCU, not on the host.

## Usage

```cpp
using namespace branes::reflex;
const model::QuadrotorModel<double> m(model::QuadrotorParams<double>{});  // RP-1 defaults
model::QuadrotorState<double> x;
x.on_ground = true;
const double w = m.hover_rotor_speed();
for (int i = 0; i < 1000; ++i) {
    x = m.step(x, model::RotorCommand<double>{{w, w, w, w}}, 1e-3);
}
```

## Math types

`Vec3<T>` and `Mat3<T>` (`branes/reflex/math/vec3.hpp`) are MTL5 fixed-size
types with stack storage. The 3-D kernels (`dot`, `cross`, `mul`) are written
out element by element, so they never reach MTL5's threaded paths.
`Quat<T>` (`branes/reflex/math/quat.hpp`) is reflex's own type, because MTL5
has no quaternion.

## Cost

`bench/quadrotor_bench` measures about 120–130 ns per RK4 step in a GCC
Release build on x86. A 1–2 kHz SITL loop spends well under 0.1 % of its tick
on the model.
