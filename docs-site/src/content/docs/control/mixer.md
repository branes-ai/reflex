---
title: Mixer
description: branes::reflex::control::Mixer — Quad X thrust and torque allocation with saturation priority.
---

`#include <branes/reflex/control/mixer.hpp>`

`branes::reflex::control::Mixer<T>` turns a body-frame wrench (collective
thrust plus roll, pitch, and yaw torque) into four rotor thrusts, and back. It
uses the conventions of the [quadrotor model](/model/quadrotor/): body frame
x forward, y left, z up, and PX4 Quad X rotor order (0 front-right CCW,
1 rear-left CCW, 2 front-left CW, 3 rear-right CW). It is stateless apart from
its parameters, constant time, allocation-free, and templated on the scalar
type.

## Allocation

With rotors at (±a, ±a, 0), a = L/√2, and yaw coefficient k_m:

| Wrench | T₀ | T₁ | T₂ | T₃ |
|---|---|---|---|---|
| F | 1 | 1 | 1 | 1 |
| τx / a | −1 | +1 | +1 | −1 |
| τy / a | −1 | +1 | −1 | +1 |
| τz / k_m | −1 | −1 | +1 | +1 |

The rows are orthogonal, so the inverse is closed form:
Tᵢ = F/4 + sxᵢ·τx/(4a) + syᵢ·τy/(4a) + szᵢ·τz/(4k_m).
`allocate()` is that inverse with no limits, and `wrench()` is the forward map.

## Saturation priority

`mix()` keeps every rotor in [T_min, T_max] and gives up the demand in a fixed
order:

1. **Roll and pitch** shrink, by a common factor σ that keeps their direction,
   only when they cannot fit at any collective thrust.
2. **Collective thrust** moves as little as possible to make room for roll and
   pitch. It moves both ways: at zero throttle it rises to keep attitude
   authority (airmode). Arming and motor idle belong to the state machine
   around the mixer.
3. **Yaw** is scaled by the largest s ∈ [0, 1] that still fits.

A demand that already fits comes back unchanged, and the output is
continuous in the demand. `MixerOutput` reports the thrusts, the wrench they
achieve, σ (`rp_scale`), s (`yaw_scale`), and whether the collective moved, so
the rate loop can stop integrating on saturated axes.

## Usage

```cpp
using namespace branes::reflex;
const model::QuadrotorParams<double> vehicle{};             // RP-1 defaults
const control::Mixer<double> mixer(control::mixer_params_from(vehicle));
const auto out = mixer.mix({20.0, vec3(0.1, -0.05, 0.02)}); // F [N], τ [N·m]
const model::RotorCommand<double> u = mixer.to_rotor_command(out.thrusts);
```

`to_rotor_command()` inverts T = k_f·ω² for a full battery. The control loops
absorb battery sag.
