---
title: Mixed-Precision Gyro Filter
description: A Butterworth gyro low-pass filter in MTL5 state-space form, compared across double, float, posits, IEEE half, and fixed point.
---

Source: `tests/examples/dsp_lowpass.cpp` (runs in CI on every build).

Flight controllers low-pass the gyro signal to strip motor and propeller
vibration before the rate loop sees it. This example builds that filter and
asks a hardware question: **which number format should it run in?**

## The filter

A 2nd-order Butterworth low-pass, $f_c = 30$ Hz at $f_s = 1$ kHz, designed
with the bilinear transform (prewarped):

$$
K = \tan\frac{\pi f_c}{f_s}, \qquad
H(z) = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2}}{1 + a_1 z^{-1} + a_2 z^{-2}}
$$

realized in transposed direct form II as a 2-state state-space system with
MTL5 dense matrices:

$$
x_{k+1} = A x_k + B u_k, \qquad y_k = C x_k + D u_k
$$

The input is a synthetic gyro signal: 2 Hz body motion (0.5 rad/s) plus 180 Hz
motor vibration (0.3 rad/s). In `double` the design meets its spec:
$|H(2\,\text{Hz})| = 0.99999$, and the 180 Hz vibration is cut by **33.1 dB**.

## Number formats compared

The same filter runs in each format, and its output is compared with the
`double` reference (signal RMS 0.354 rad/s):

| Format | Bits | Error RMS (rad/s) | SNR |
|---|---|---|---|
| `float` | 32 | 5.5e-7 | 116.1 dB |
| `posit<32,2>` | 32 | 2.6e-8 | **142.8 dB** |
| IEEE half (`cfloat<16,5>`) | 16 | 5.0e-3 | 37.0 dB |
| Q3.12 fixed point (`fixpnt<16,12>`) | 16 | 3.3e-3 | 40.6 dB |
| `posit<16,1>` | 16 | 1.1e-3 | **49.9 dB** |

Identical under GCC 13 and Clang 18.

**Reading it.** A gyro signal lives near magnitude 1, where posits put most of
their precision (tapered accuracy). At 32 bits that makes `posit<32,2>` about
27 dB more accurate than `float`. At 16 bits, the width an embedded DSP path or
a narrow accelerator datapath would use, `posit<16,1>` beats both IEEE half
(+13 dB) and the classic Q-format fixed-point choice (+9 dB). Fixed point needs
the designer to pick the binary point; the posit needs no such tuning.

## MTL5 note

The filter's input enters as a 1-vector (`B * u` with `u` a `dense_vector`)
rather than `B * scalar`. MTL5's scalar×matrix and scalar×vector operators
currently accept only built-in arithmetic types
([stillwater-sc/mtl5#538](https://github.com/stillwater-sc/mtl5/issues/538)).
