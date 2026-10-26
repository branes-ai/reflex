---
title: Gyro Filters
description: branes::reflex::dsp — biquad low-pass and notch sections, and RPM-tracked notches at the rotor harmonics.
---

`#include <branes/reflex/dsp/biquad.hpp>`
`#include <branes/reflex/dsp/rpm_notch.hpp>`

**Grade: C** (specified and unit-tested) under the
[building block standard](/standards/building-blocks/). See
[what the evidence shows](#what-the-evidence-shows).

The rate loop runs on the gyro, and the gyro sees motor and propeller
vibration as well as the vehicle's motion. These filters remove the
vibration before the [rate controller](/control/cascade/) differentiates it:
a low-pass section for broadband noise, and notches that follow the rotor
speeds for the narrow peaks at the rotor harmonics. Every filter costs phase
at the rate loop's crossover, so the filter chain is part of the loop's
phase-margin budget.

## Sections and designs

`Biquad<T>` realizes

$$
H(z) = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2}}{1 + a_1 z^{-1} + a_2 z^{-2}}
$$

in Direct Form I, which keeps the last two inputs and outputs as its state.
Every stored value is a signal sample, so the state stays within the signal's
range, and new coefficients take effect without a jump, which the RPM notch
relies on when it retunes every tick.

| Design | Result |
|---|---|
| `butterworth_lowpass(fc, fs)` | 2nd-order Butterworth; unity at DC, $1/\sqrt{2}$ at $f_c$, zero at $f_s/2$ |
| `first_order_lowpass(fc, fs)` | First-order ("PT1") low-pass; unity at DC, $1/\sqrt{2}$ at $f_c$ |
| `notch(f0, q, fs)` | Cookbook notch; exactly zero at $f_0$, unity at DC and $f_s/2$ |

All three use the bilinear transform with prewarping, so each is exact at its
design frequency. They require $0 < f < f_s/2$; a violated precondition
asserts, and a release build returns the pass-through section. The notch's
−3 dB width is

$$
\Delta f = \frac{f_s}{\pi}\arctan\frac{\sin\omega_0}{2q}, \qquad \omega_0 = 2\pi f_0/f_s,
$$

which tends to $f_0/q$ well below Nyquist and is narrower near it.

`response(c, f, fs)` gives the complex gain and `group_delay(c, f, fs)` the
group delay in seconds, from the closed form rather than a numerical
derivative. Both are evaluated in `double`, for design-time budgets.

## Usage

```cpp
using namespace branes::reflex;

// A 100 Hz gyro low-pass at the 8 kHz gyro rate, one per axis.
const auto lpf = dsp::butterworth_lowpass(100.0f, 8000.0f);
std::array<dsp::Biquad<float>, 3> gyro_lpf{dsp::Biquad<float>(lpf), dsp::Biquad<float>(lpf),
                                           dsp::Biquad<float>(lpf)};

// RPM notches at three harmonics of each of four rotors.
dsp::RpmNotchConfig<float> cfg;
cfg.sample_rate = 8000.0f;
dsp::RpmNotch<float> rpm(cfg);

// Every gyro sample:
rpm.update_rotor_speeds(rotor_speeds);  // rad/s, from ESC telemetry or the model
Vec3<float> g = rpm.apply(gyro);
for (std::size_t i = 0; i < 3; ++i) {
    g(i) = gyro_lpf[i].update(g(i));
}
```

## Faults

- **A non-finite sample** (or an output that overflows the scalar type)
  makes a section return its last output and keep its state. A single bad
  gyro sample does not corrupt the filter.
- **A non-finite or negative rotor speed** makes `RpmNotch` keep that rotor's
  previous notches.
- **A section starts at a steady state**: `Biquad(c, x0)` and `reset(x0)` set
  the history as if the input had been `x0` forever, so a filter started on a
  live signal has no start-up transient.

## RPM notches

`RpmNotch<T, Rotors = 4, MaxHarmonics = 3>` places a notch at each harmonic
$h\,\omega/2\pi$ of each rotor and runs each gyro axis through all of them in
series. The three axes share each notch's coefficients.

| Parameter | Default | Meaning |
|---|---|---|
| `sample_rate` | 1000 Hz | Rate of `apply()` |
| `harmonics` | 3 | Harmonics per rotor, 1 … `MaxHarmonics` |
| `q` | 5 | Quality factor of every notch |
| `min_hz` | 100 Hz | Lowest centre; a lower harmonic is clamped here |
| `fade_hz` | 50 Hz | The notch fades in linearly over [min_hz, min_hz + fade_hz] |

A notch whose harmonic lies above $0.45 f_s$ is parked there with weight 0.
Each notch's output is blended as $y \leftarrow y + w\,(N(y) - y)$, and every
section keeps updating at $w = 0$, so its history is live when it fades back
in. `rpm_notch_config_valid` checks a configuration.

## Contracts

| | |
|---|---|
| **Invariants** | `Biquad`: valid (finite, stable) coefficients and a finite history. `RpmNotch`: a valid configuration, every centre in [min_hz, 0.45·fs], every weight in [0, 1]. Checked by `assert` after each operation. |
| **Real-time** | `noexcept`, no allocation. `Biquad::update` runs 9 operations; `RpmNotch::apply` runs a count that depends only on the configuration; `update_rotor_speeds` is bounded by its all-in-range call. `tests/sdk/dsp_resource.cpp` checks all three. |
| **Scalar types** | Any type with `+ − × ÷ <`, `sin`, `cos`, `abs`, `isfinite`, and construction from `double`. Tested in `double` and `float`. |

16-bit types (`posit<16,1>`, IEEE binary16) are not supported: a notch near
DC does not survive coefficient rounding in them. The depth they reach is
printed by the `16-bit types` test case for reference.

## What the evidence shows

A filter's correctness is a property of its transfer function, so these
tests are verification against analytic answers, and they need no physical
reference. The open question is whether the vibration actually sits at the
rotor harmonics. Spectral tests on RP-1 flight logs answer it (the
[control guide's](/control/quadrotor-control/) F3).

The acceptance criteria were written into
[reflex#21](https://github.com/branes-ai/reflex/issues/21) before the tests
ran. Each tolerance is derived from a bound (the impulse response's tail for
the start-up transient, and a per-step rounding bound carried through
$1/A(z)$), never tuned to pass.

| Criterion | Pass if |
|---|---|
| Butterworth and first-order designs | DC gain 1 and $1/\sqrt{2}$ at $f_c$ to 1e-12; Butterworth magnitude monotone |
| Measured tone gains, 20 frequencies | Match `response()` within the derived bound |
| Notch | Analytic gain at $f_0$ ≤ 1e-12; a tone at $f_0$ attenuated ≥ 60 dB (double), ≥ 40 dB (float); −3 dB edges where the closed form puts them |
| Group delay | Measured from tone phases, matches `group_delay()` to 50 µs |
| Delay budget | Rate-loop phase margin ≥ 45° on the nominal vehicle with a Butterworth at 16× crossover and hover-speed RPM notches in the loop |
| RPM tracking | Four rotors spread ±5 %, ramped to 1.5× hover in 1 s: vibration residual ≤ −30 dB, 5 Hz motion within 2 % |
| Fast retuning | Full-range speed swings within one motor time constant keep the output within the series ℓ1 gain bound |
| Code-to-code | Designs and outputs match [stillwater-sc/mp-dsp](https://github.com/stillwater-sc/mp-dsp), an independent implementation, within the derived bounds |

mp-dsp is a test-only dependency. Its designs throw on invalid input and do
not build with `-fno-exceptions`
([mp-dsp#217](https://github.com/stillwater-sc/mp-dsp/issues/217)), so the
flight path keeps reflex's own.

## Not covered

- Choosing the filter settings per vehicle class: the consumer composes the
  chain (racing-drone#16).
- FFT-tracked dynamic notches.
- Anti-alias filtering and the IMU sample rate; harmonics above $f_s/2$ alias
  and are not tracked.

## Cost

`bench/filter_bench` reports nanoseconds per `Biquad::update`, per 3-axis
`RpmNotch::apply`, and per `update_rotor_speeds`, in `float` and `double`, on
the host. These are latency characterization figures, not a timing bound.
