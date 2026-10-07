---
title: PID Controller
description: branes::reflex::Pid — a discrete PID controller for flight-control inner loops.
---

`#include <branes/reflex/control/pid.hpp>`

`branes::reflex::Pid<Real>` is a discrete-time PID controller with the
refinements a flight controller needs: derivative on measurement, a derivative
low-pass filter, integral clamping, conditional-integration anti-windup, and
output saturation.

## Control law

With setpoint $r$, measurement $y$, error $e = r - y$ and sample period
$\Delta t$, each update computes

$$
u_k = \operatorname{sat}_{[u_{\min},\,u_{\max}]}\!\left(K_p e_k + I_k - K_d \dot{y}_k\right)
$$

**Derivative on measurement.** The D term uses $-\dot{y}$ rather than $\dot{e}$.
For a constant setpoint they are equal, but a step in $r$ produces no
derivative kick. The rate is low-pass filtered with time constant $\tau$:

$$
\dot{y}_k = \frac{\tau\,\dot{y}_{k-1} + \Delta t\,\dfrac{y_k - y_{k-1}}{\Delta t}}{\tau + \Delta t}
$$

$\tau = 0$ disables the filter. The first update after construction or
`reset()` has no history, so its rate is zero.

**Integral.** The integral is kept in output units, $I_k = K_i \sum e\,\Delta t$,
clamped to $[I_{\min}, I_{\max}]$.

**Anti-windup (conditional integration).** If the unsaturated output exceeds a
limit *and* the error would push it further past that limit, the integral is
frozen for that step. If the error would pull the output back into range, it
integrates normally. After a long saturation (for example, a setpoint the
actuator cannot reach), the loop recovers immediately instead of waiting for a
wound-up integral to unwind. The closed-loop test
`anti-windup recovers quickly from an unreachable setpoint` pins this: the loop
settles in under 0.2 s, versus ~2.4 s for a naive integrator.

## Usage

```cpp
#include <branes/reflex/control/pid.hpp>

using branes::reflex::Pid;
using branes::reflex::PidConfig;

PidConfig<float> cfg;
cfg.kp = 0.8f;
cfg.ki = 2.0f;
cfg.kd = 0.02f;
cfg.output_min = -1.0f;      // normalized torque command
cfg.output_max = 1.0f;
cfg.integral_min = -0.3f;    // cap the integral's authority
cfg.integral_max = 0.3f;
cfg.derivative_tau = 0.001f; // ~160 Hz derivative filter

Pid<float> roll_rate{cfg};

// In the 8 kHz rate loop:
float torque = roll_rate.update(rate_setpoint, gyro_x, dt);
```

## Contracts

| | |
|---|---|
| **Construction** | Requires `pid_config_valid(cfg)`: non-negative gains, `min <= max` for both limit pairs, `derivative_tau >= 0`. Checked by `assert`. |
| **`update`** | Always returns a value in `[output_min, output_max]`. A non-positive `dt` is a timing fault: the controller returns its last output and leaves its state unchanged. |
| **`reset`** | Returns to the constructed state: zero integral and rate (clamped into range if zero is outside it), unprimed derivative. |
| **Real-time** | `noexcept`, no allocation, constant time, usable in `constexpr`. |

Gains are fixed at construction; there is no setter. To retune, construct a new
controller through the lifecycle (`Unconfigured → Inactive → Active`).

## Scalar types

`Pid<Real>` uses only `+ - * /`, `<`, and construction from an integer, so it
instantiates for `float`, `double`, and custom arithmetic types such as
Universal posits.
