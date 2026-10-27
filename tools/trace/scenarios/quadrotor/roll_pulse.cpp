// Open-loop roll pulse from hover: the P0 test trace for the docs-site player.
//
// The vehicle hovers at 1 m. At 0.1 s the left rotors are commanded up and the right rotors down for 0.12 s, then the
// reverse for 0.12 s, then hover speed again. Motor lag delays each torque step; the vehicle ends up rolled by about
// 10° (right side down), with no controller to level it, so it slides to the right (−y) and sinks. The scenario shows
// rotor lag, roll, and the coupling of attitude into translation, without any controller in the loop.
//
// Positive roll speeds up the left rotors: 1 (RL) and 2 (FL) in PX4 Quad X
// order. Writes $REFLEX_TRACE_DIR/quadrotor/roll_pulse.json.

#include "quadrotor_trace.hpp"

#include <algorithm>
#include <cmath>

namespace {

namespace model = branes::reflex::model;
namespace trace = branes::reflex::trace;
namespace qt = branes::reflex::trace::quadrotor;

constexpr double kDtSim = 0.001;  // ≤ min(0.01 s, τ = 0.02 s)
constexpr int kStepsPerSample = 10;
constexpr double kDtTrace = kDtSim * kStepsPerSample;
constexpr int kSamples = 151;  // 0 … 1.5 s
// Pulse timing in integration steps, so the switch times do not depend on how
// a compiler rounds t: +Δ over [100, 220), −Δ over [220, 340).
constexpr int kPulseStart = 100;
constexpr int kPulseHalf = 120;
constexpr double kDelta = 40;  // rad/s added to the left rotors, removed from the right

model::RotorCommand<double> command(int step, double hover) {
    double d = 0;
    if (step >= kPulseStart && step < kPulseStart + kPulseHalf) {
        d = kDelta;
    } else if (step >= kPulseStart + kPulseHalf && step < kPulseStart + 2 * kPulseHalf) {
        d = -kDelta;
    }
    // Rotors 0 FR, 1 RL, 2 FL, 3 RR: left = 1, 2.
    return {{hover - d, hover + d, hover + d, hover - d}};
}

int run() {
    const model::QuadrotorParams<double> params{};
    const model::QuadrotorModel<double> quad(params);
    const double hover = quad.hover_rotor_speed();

    model::QuadrotorState<double> x = qt::hover_state(quad, 1.0);

    // commit and compiler left empty: filled from REFLEX_TRACE_COMMIT and the
    // predefined macros.
    trace::Trace tr({.commit = {},
                     .block = "model/quadrotor",
                     .scenario = "tools/trace/scenarios/quadrotor/roll_pulse.cpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = kDtSim,
                     .dt_trace = kDtTrace});
    tr.vehicle_arm_length(params.arm_length);
    qt::declare_state(tr);

    double norm_error_max = 0;
    for (int k = 0; k < kSamples; ++k) {
        tr.sample(k * kDtTrace);
        qt::record_state(tr, quad, x, command(k * kStepsPerSample, hover));
        if (k + 1 == kSamples) {
            break;
        }
        for (int i = 0; i < kStepsPerSample; ++i) {
            x = quad.step(x, command(k * kStepsPerSample + i, hover), kDtSim);
            const auto& q = x.attitude;
            norm_error_max = std::max(norm_error_max, std::abs(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z - 1));
        }
    }

    tr.event(0.1, "left rotors +40 rad/s, right rotors -40 rad/s: roll torque");
    tr.event(0.22, "reversed: braking torque");
    tr.event(0.34, "hover command: the vehicle stays rolled about 10°");
    tr.check({.id = "BB-V3",
              .metric = "attitude_norm_error_max",
              .unit = "1",
              .value = norm_error_max,
              .limit = model::QuadrotorModel<double>::attitude_tolerance(),
              .comparison = "<="});

    return qt::write(tr, "quadrotor/roll_pulse.json");
}

}  // namespace

int main() {
    return qt::guarded("roll_pulse", run);
}
