#pragma once

// The derivative filter: how much to smooth a noisy measurement's rate.
//
// The height loop of terms.hpp (z̈ = u, here with no offset; k_p = 16,
// k_i = 8, k_d = 8, at 1 kHz) holds 1 m, and at 1 s steps to 1.2 m. The
// height sensor adds uniform noise of ±1 mm, from a seeded generator. Two
// Pid controllers fly two copies of the loop on the same noise:
//  - raw: derivative_tau = 0. The D term differentiates the noise: ±1 mm over
//    1 ms is ±2 m/s, times k_d, so the command jumps by up to 16 m/s² from
//    one tick to the next;
//  - filtered: derivative_tau = τ (5 ms), a first-order low-pass on the
//    measured rate. The command is eight times smoother, and the step
//    response is unchanged. The filter delays the rate the D term damps
//    with by about τ, which costs nothing while τ is small against the
//    loop's own time scale (1/k_d ≈ 0.13 s): up to about 0.3 s the step's
//    overshoot (17 %) changes little. Beyond that the damping arrives too
//    late, and the overshoot doubles by 0.5 s.
// The D term acts on the measurement, not the error, so the setpoint step
// itself does not kick the command.
//
// The parameter is τ of the filtered run. derivative_filter.cpp writes the
// trace at the default; the explainer's live widget runs this function,
// compiled to WebAssembly, at the reader's τ.

#include "pid_trace.hpp"

#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>

namespace branes::reflex::trace::scenarios::pid_derivative_filter {

/// The widget's parameter: τ of the filtered run [s], in [0.5 ms, 500 ms].
struct Params {
    double tau = 0.005;
};

namespace trace = branes::reflex::trace;
namespace pt = branes::reflex::trace::pid;

inline constexpr int kStepsPerSample = 4;
inline constexpr double kDtTrace = pt::kDt * kStepsPerSample;
inline constexpr int kSamples = 1001;  // 0 … 4 s
inline constexpr long kStep = 1000;    // tick of the setpoint step
inline constexpr double kNoise = 0.001;
inline constexpr long kSeed = 7;

inline trace::Trace run(const Params& p) {
    if (!(p.tau >= 0.0005 && p.tau <= 0.5)) {
        throw std::invalid_argument("pid_derivative_filter: tau outside [0.5 ms, 500 ms]");
    }
    PidConfig<double> raw;
    raw.kp = 16;
    raw.ki = 8;
    raw.kd = 8;
    raw.output_min = raw.integral_min = -20;
    raw.output_max = raw.integral_max = 20;
    PidConfig<double> filtered = raw;
    filtered.derivative_tau = p.tau;
    Pid<double> ctl[2] = {Pid<double>{raw}, Pid<double>{filtered}};
    pt::DoubleIntegrator plant[2] = {{.z = 1}, {.z = 1}};

    // std::mt19937's output sequence is fixed by the standard, unlike the
    // distributions, so the noise is the same on every compiler.
    std::mt19937 rng(static_cast<std::uint32_t>(kSeed));
    const auto noise = [&] { return kNoise * (2.0 * static_cast<double>(rng()) / 4294967295.0 - 1.0); };

    trace::Trace tr({.commit = {},
                     .block = "control/pid",
                     .scenario = "tools/trace/scenarios/pid/derivative_filter.hpp",
                     .seed = kSeed,
                     .compiler = {},
                     .dt_sim = pt::kDt,
                     .dt_trace = kDtTrace});
    tr.channel("height", 2, "m");
    tr.channel("setpoint", 1, "m");
    tr.channel("command", 2, "m/s²");
    tr.channel("measured_rate", 2, "m/s");

    double u[2] = {0, 0};
    for (long n = 0;; ++n) {
        const double r = n < kStep ? 1.0 : 1.2;
        const double e = noise();
        for (int i = 0; i < 2; ++i) {
            u[i] = ctl[i].update(r, plant[i].z + e, pt::kDt);
        }
        if (n % kStepsPerSample == 0) {
            tr.sample(static_cast<double>(n) * pt::kDt);
            tr.set("height", {plant[0].z, plant[1].z});
            tr.set("setpoint", r);
            tr.set("command", {u[0], u[1]});
            tr.set("measured_rate", {ctl[0].measurement_rate(), ctl[1].measurement_rate()});
            if (n / kStepsPerSample + 1 == kSamples) {
                break;
            }
        }
        for (int i = 0; i < 2; ++i) {
            plant[i].step(u[i], 0, pt::kDt);
        }
    }

    tr.event(static_cast<double>(kStep) * pt::kDt, "setpoint 1.2 m");
    return tr;
}

}  // namespace branes::reflex::trace::scenarios::pid_derivative_filter
