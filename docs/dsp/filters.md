---
title: Gyro Filters
description: Developer guide for branes::reflex::dsp, the gyro filters. Covers the biquad section and its designs, RPM-tracked notches at the rotor harmonics, fault handling, the contracts, and the API.
sidebar:
  label: "Developer Guide"
  group: "Gyro Filters"
---

`#include <branes/reflex/dsp/biquad.hpp>`
`#include <branes/reflex/dsp/rpm_notch.hpp>`

**Grade: C**, under the
[building block standard](../standards/building-blocks.md). The
[V&V plan](filters-vv.mdx) has the assessment, the acceptance criteria,
and the path to A.

This is the developer guide: the designs, the RPM notches, faults, the
contracts, and the API. The [explainer](filters-explainer.mdx) shows the
filters at work, with spectrograms of a throttle sweep and a widget that
runs them in your browser. The [deployment guide](filters-deployment.mdx)
covers the gyro rate, the telemetry, choosing the settings, and wiring
the chain into a flight stack.

The rate loop runs on the gyro, and the gyro sees motor and propeller
vibration as well as the vehicle's motion. These filters remove the
vibration before the [rate controller](../control/cascade.mdx) differentiates it:
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
- **A non-finite input component, or a notch blend that overflows,** makes
  `RpmNotch::apply` return that axis's last output; the other axes carry on.
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
in. At $w = 0$ the blend is skipped and the stage passes its input through
exactly. `rpm_notch_config_valid` checks a configuration. The
[explainer](filters-explainer.mdx#3-staying-out-of-the-control-band)
shows the floor and the fade during a throttle sweep.

## Contracts

| | |
|---|---|
| **Invariants** | `Biquad`: valid (finite, stable) coefficients and a finite history. `RpmNotch`: a valid configuration, every centre in [min_hz, 0.45·fs], every weight in [0, 1], a finite last output. Checked by `assert` after each operation. |
| **Real-time** | `noexcept`, no allocation. Bounded operation counts: `Biquad::update` runs 9 operations on a finite sample and none on a faulted one; `RpmNotch::apply` runs at most its count with every notch blended (fewer while notches are faded out or an axis faults); `update_rotor_speeds` is bounded by its all-in-range call. `tests/sdk/dsp_resource.cpp` checks all three. |
| **Scalar types** | Filtering and design: any type with `+ − × ÷ <`, `sin`, `cos`, `abs`, `isfinite`, and construction from `double`. The analysis helpers `response()` and `group_delay()` also need `static_cast<double>` from the type. Tested in `double` and `float`. |

16-bit types (`posit<16,1>`, IEEE binary16) are not supported: a notch near
DC does not survive coefficient rounding in them. The depth they reach is
printed by the `16-bit types` test case for reference.

## API

| Member | Contract |
|---|---|
| `BiquadCoefficients<T>` | b0, b1, b2, a1, a2 of H(z); the default is the pass-through section |
| `biquad_coefficients_valid(c)` | Finite, and the poles strictly inside the unit circle (I1) |
| `design_frequency_valid(f, fs)` | 0 < f < fs/2, both finite |
| `butterworth_lowpass(fc, fs)`, `first_order_lowpass(fc, fs)`, `notch(f0, q, fs)` | Pre: a valid design frequency (and q > 0). Post: a valid section meeting its design. On a violation: asserts, and returns the pass-through section in release |
| `dc_gain(c)`, `response(c, f, fs)`, `group_delay(c, f, fs)` | The gain at DC; the complex gain at f; the group delay in seconds, from the closed form. The last two evaluate in double |
| `Biquad(c, x0 = 0)` | Pre: c valid, x0 finite. Post: the steady state for a constant input x0 |
| `update(x)` | One sample. A non-finite input or an overflow returns the last output and keeps the state |
| `set_coefficients(c)` | New coefficients, same history. Pre: c valid; ignored in release otherwise |
| `reset(x0 = 0)`, `coefficients()`, `output()` | Restart at a steady state; the section; the last output |
| `RpmNotchConfig<T>`, `rpm_notch_config_valid<T, H>(c)` | The configuration and its validator (I1) |
| `RpmNotch<T, Rotors = 4, MaxHarmonics = 3>(cfg)` | Pre: cfg valid. Post: every notch at min_hz with weight 0, so the input passes through |
| `update_rotor_speeds(ω)` | Rotor speeds in rad/s. Post: each notch placed, faded, or parked; a non-finite or negative speed holds that rotor's notches |
| `apply(x)` | One 3-axis sample through every notch in series. Post: a finite output; a faulted axis returns its last output |
| `reset()`, `config()`, `center_hz(r, h)`, `weight(r, h)`, `section(r, h, i)` | Clear the history, keep the notches; inspection |

`Biquad` holds two invariants: valid coefficients (I1) and a finite history
(I2). `RpmNotch` holds four: a valid configuration (I1), every centre in
[min_hz, 0.45·fs] (I2), every weight in [0, 1] (I3), and a finite last
output (I4). Debug builds assert them after every operation.

## Not covered

- Choosing the filter settings per vehicle class: the consumer composes the
  chain (racing-drone#16).
- FFT-tracked dynamic notches.
- Anti-alias filtering and the IMU sample rate; harmonics above $f_s/2$ alias
  and are not tracked.

The [deployment guide](filters-deployment.mdx) covers choosing the gyro
rate and the settings, and the cost per call.
