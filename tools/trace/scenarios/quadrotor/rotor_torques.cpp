// One rotor at a time, on the ground: which way each rotor pushes and twists
// the body.
//
// The vehicle sits on the ground with its rotors stopped. Each rotor in turn
// is commanded to 90 % of the hover speed for 0.4 s, alone, then all four
// together. One rotor makes a quarter of 81 % of the weight, so the vehicle
// never lifts off and the body stays put (ground contact): the trace shows the
// thrust and the body torque each rotor makes, without any motion to follow.
//
// The torque is the model's own: J ⊙ ω̇ from derivative(), at zero body rates
// (a grounded vehicle has ω = 0, so the gyroscopic term vanishes). Writes
// $REFLEX_TRACE_DIR/quadrotor/rotor_torques.json.

#include "quadrotor_trace.hpp"

#include <cstddef>
#include <string>

namespace {

namespace model = branes::reflex::model;
namespace trace = branes::reflex::trace;
namespace qt = branes::reflex::trace::quadrotor;

constexpr double kDtSim = 0.001;
constexpr int kStepsPerSample = 10;
constexpr double kDtTrace = kDtSim * kStepsPerSample;
constexpr int kStart = 100;                                                   // steps of idle before rotor 0
constexpr int kWindow = 400;                                                  // steps per rotor
constexpr int kSamples = (kStart + 5 * kWindow + 300) / kStepsPerSample + 1;  // 0 … 2.4 s
constexpr double kFraction = 0.9;                                             // of the hover speed

const char* const kRotorNames[] = {
    "0 (front right, CCW)", "1 (rear left, CCW)", "2 (front left, CW)", "3 (rear right, CW)"};

qt::Command command(int step, double speed) {
    qt::Command u{};
    if (step < kStart) {
        return u;
    }
    const int window = (step - kStart) / kWindow;
    for (std::size_t i = 0; i < model::kRotors; ++i) {
        u.speeds[i] = window == static_cast<int>(i) || window == 4 ? speed : 0;
    }
    return u;
}

int run() {
    const model::QuadrotorParams<double> params{};
    const qt::Model quad(params);
    const double speed = kFraction * quad.hover_rotor_speed();

    qt::State x;
    x.on_ground = true;

    trace::Trace tr({.commit = {},
                     .block = "model/quadrotor",
                     .scenario = "tools/trace/scenarios/quadrotor/rotor_torques.cpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = kDtSim,
                     .dt_trace = kDtTrace});
    tr.vehicle_arm_length(params.arm_length);
    qt::declare_state(tr);
    tr.channel("total_thrust", 1, "N");
    tr.channel("weight", 1, "N");
    tr.channel("body_torque", 3, "N·m");

    for (int k = 0; k < kSamples; ++k) {
        const qt::Command u = command(k * kStepsPerSample, speed);
        tr.sample(k * kDtTrace);
        qt::record_state(tr, quad, x, u);
        double total = 0;
        for (const double f : quad.rotor_thrusts(x)) {
            total += f;
        }
        const auto d = quad.derivative(x, u);
        tr.set("total_thrust", total);
        tr.set("weight", params.mass * params.gravity);
        tr.set("body_torque",
               {params.inertia[0] * d.body_rates(0),
                params.inertia[1] * d.body_rates(1),
                params.inertia[2] * d.body_rates(2)});
        if (k + 1 == kSamples) {
            break;
        }
        for (int i = 0; i < kStepsPerSample; ++i) {
            x = quad.step(x, command(k * kStepsPerSample + i, speed), kDtSim);
        }
    }

    for (int i = 0; i < 4; ++i) {
        tr.event((kStart + i * kWindow) * kDtSim, std::string("rotor ") + kRotorNames[i] + " alone");
    }
    tr.event((kStart + 4 * kWindow) * kDtSim, "all four: the torques cancel; thrust 81 % of the weight");
    tr.event((kStart + 5 * kWindow) * kDtSim, "all rotors commanded to stop");
    return qt::write(tr, "quadrotor/rotor_torques.json");
}

}  // namespace

int main() {
    return qt::guarded("rotor_torques", run);
}
