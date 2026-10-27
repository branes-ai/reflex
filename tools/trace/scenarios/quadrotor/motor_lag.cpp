// Motor lag: a collective speed step from hover, and back.
//
// The vehicle hovers at 1 m. At 0.05 s every rotor is commanded 10 % above
// the hover speed, and at 0.25 s back to the hover speed. Each rotor follows
// its command as a first-order lag with τ = 20 ms, so thrust arrives late and
// leaves late; the vehicle keeps the vertical speed it gained, because no
// controller brakes it.
//
// Check (BB-V1): the rotor speed against the closed-form step response
// ω(t) = u + (ω₀ − u)·exp(−(t − t₀)/τ), at every sample. RK4 at dt = τ/20 is
// within about (dt/τ)⁵/120 ≈ 3e-9 of exp(−dt/τ) per step, so the error stays
// near 1e-6 rad/s; the limit is 1e-4 rad/s (6e-8 of the hover speed). Writes
// $REFLEX_TRACE_DIR/quadrotor/motor_lag.json.

#include "quadrotor_trace.hpp"

#include <algorithm>
#include <cmath>

namespace {

namespace model = branes::reflex::model;
namespace trace = branes::reflex::trace;
namespace qt = branes::reflex::trace::quadrotor;

constexpr double kDtSim = 0.001;
constexpr int kStepsPerSample = 2;
constexpr double kDtTrace = kDtSim * kStepsPerSample;
constexpr int kSamples = 201;  // 0 … 0.4 s
constexpr int kUp = 50;        // step index of the step up
constexpr int kDown = 250;     // and of the step down
constexpr double kStep = 0.1;  // fraction of the hover speed

qt::Command command(int step, double hover) {
    const double w = step >= kUp && step < kDown ? hover * (1 + kStep) : hover;
    return {{w, w, w, w}};
}

int run() {
    const model::QuadrotorParams<double> params{};
    const qt::Model quad(params);
    const double hover = quad.hover_rotor_speed();
    const double tau = params.motor_time_constant;
    qt::State x = qt::hover_state(quad, 1.0);

    trace::Trace tr({.commit = {},
                     .block = "model/quadrotor",
                     .scenario = "tools/trace/scenarios/quadrotor/motor_lag.cpp",
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
    tr.event((kUp + 20) * kDtSim, "one time constant later: 63 % of the step");
    tr.event(kDown * kDtSim, "command back to the hover speed");
    tr.check({.id = "BB-V1",
              .metric = "rotor_speed_error_vs_closed_form_max",
              .unit = "rad/s",
              .value = error_max,
              .limit = 1e-4,
              .comparison = "<="});
    return qt::write(tr, "quadrotor/motor_lag.json");
}

}  // namespace

int main() {
    return qt::guarded("motor_lag", run);
}
