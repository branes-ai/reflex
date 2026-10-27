#pragma once

// Motor lag: a collective speed step from hover, and back.
//
// The vehicle hovers at 1 m. At 0.05 s every rotor is commanded 10 % above
// the hover speed, and at 0.25 s back to the hover speed. Each rotor follows
// its command as a first-order lag with time constant τ (RP-1: 20 ms), so
// thrust arrives late and leaves late; the vehicle keeps the vertical speed it
// gained, because no controller brakes it.
//
// Check (BB-V1): the rotor speed against the closed-form step response
// ω(t) = u + (ω₀ − u)·exp(−(t − t₀)/τ), at every sample. RK4 at dt = 1 ms is
// within about (dt/τ)⁵/120 of exp(−dt/τ) per step: 3e-9 at τ = 20 ms, so the
// error stays near 1e-6 rad/s against a limit of 1e-4 rad/s (6e-8 of the
// hover speed). At small τ the error grows as τ⁻⁵, and the check can fail.
//
// The parameter is τ. motor_lag.cpp writes the trace at RP-1's τ; the
// explainer's live widget runs this function, compiled to WebAssembly, at the
// reader's τ.

#include "quadrotor_trace.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace branes::reflex::trace::scenarios::motor_lag {

/// The widget's parameter: the motor time constant τ [s], in [5 ms, 100 ms].
struct Params {
    double tau = 0.02;
};

namespace model = branes::reflex::model;
namespace trace = branes::reflex::trace;
namespace qt = branes::reflex::trace::quadrotor;

inline constexpr double kDtSim = 0.001;
inline constexpr int kStepsPerSample = 2;
inline constexpr double kDtTrace = kDtSim * kStepsPerSample;
inline constexpr int kSamples = 201;  // 0 … 0.4 s
inline constexpr int kUp = 50;        // step index of the step up
inline constexpr int kDown = 250;     // and of the step down
inline constexpr double kStep = 0.1;  // fraction of the hover speed

inline qt::Command command(int step, double hover) {
    const double w = step >= kUp && step < kDown ? hover * (1 + kStep) : hover;
    return {{w, w, w, w}};
}

inline trace::Trace run(const Params& p) {
    if (!(p.tau >= 0.005 && p.tau <= 0.1)) {
        throw std::invalid_argument("motor_lag: tau outside [5 ms, 100 ms]");
    }
    model::QuadrotorParams<double> params{};
    params.motor_time_constant = p.tau;
    const qt::Model quad(params);
    const double hover = quad.hover_rotor_speed();
    const double tau = params.motor_time_constant;
    qt::State x = qt::hover_state(quad, 1.0);

    trace::Trace tr({.commit = {},
                     .block = "model/quadrotor",
                     .scenario = "tools/trace/scenarios/quadrotor/motor_lag.hpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = kDtSim,
                     .dt_trace = kDtTrace});
    tr.vehicle_arm_length(params.arm_length);
    qt::declare_state(tr);
    tr.channel("total_thrust", 1, "N");
    tr.channel("weight", 1, "N");

    // Closed-form response of one rotor at step index n: piecewise, from the
    // speed reached at the previous switch.
    const double high = hover * (1 + kStep);
    const auto analytic = [&](int n) {
        if (n <= kUp) {
            return hover;
        }
        const double up = high + (hover - high) * std::exp(-(std::min(n, kDown) - kUp) * kDtSim / tau);
        if (n <= kDown) {
            return up;
        }
        return hover + (up - hover) * std::exp(-(n - kDown) * kDtSim / tau);
    };

    double error_max = 0;
    for (int k = 0; k < kSamples; ++k) {
        const int n = k * kStepsPerSample;
        tr.sample(k * kDtTrace);
        qt::record_state(tr, quad, x, command(n, hover));
        double total = 0;
        for (const double f : quad.rotor_thrusts(x)) {
            total += f;
        }
        tr.set("total_thrust", total);
        tr.set("weight", params.mass * params.gravity);
        for (const double w : x.rotor_speeds) {
            error_max = std::max(error_max, std::abs(w - analytic(n)));
        }
        if (k + 1 == kSamples) {
            break;
        }
        for (int i = 0; i < kStepsPerSample; ++i) {
            x = quad.step(x, command(n + i, hover), kDtSim);
        }
    }

    tr.event(kUp * kDtSim, "command +10 % on every rotor");
    tr.event((kUp + static_cast<int>(std::lround(tau / kDtSim))) * kDtSim, "one time constant later: 63 % of the step");
    tr.event(kDown * kDtSim, "command back to the hover speed");
    tr.check({.id = "BB-V1",
              .metric = "rotor_speed_error_vs_closed_form_max",
              .unit = "rad/s",
              .value = error_max,
              .limit = 1e-4,
              .comparison = "<="});
    return tr;
}

}  // namespace branes::reflex::trace::scenarios::motor_lag
