// Full-throttle punch-out into saturation, then a roll command the rotors
// cannot fully follow.
//
// The vehicle hovers at 1 m. At 0.1 s every rotor is commanded to ω_max, the
// speed of maximum thrust, and the vehicle climbs at about 3.9 g. At 0.5 s a
// roll command adds 20 rad/s to the left rotors and removes it from the right
// rotors. The left rotors are already at ω_max, and the model clamps their
// commands there, so only the right rotors respond: the vehicle rolls at half
// the torque the command asks for, and loses thrust as it does. A mixer that
// keeps attitude authority at full throttle must lower the collective itself
// (desaturate); the model only clamps.
//
// The run leaves the model's expected envelope twice, and the events say
// when: rotor speeds above 95 % of ω_max, and airspeed above 5 m/s. Outside
// it the model is unvalidated (it has no induced-velocity thrust loss and no
// rotor drag). Writes $REFLEX_TRACE_DIR/quadrotor/punch_out.json.

#include "quadrotor_trace.hpp"

#include <algorithm>
#include <cmath>

namespace {

namespace model = branes::reflex::model;
namespace trace = branes::reflex::trace;
namespace qt = branes::reflex::trace::quadrotor;

constexpr double kDtSim = 0.001;
constexpr int kStepsPerSample = 10;
constexpr double kDtTrace = kDtSim * kStepsPerSample;
constexpr int kSamples = 81;  // 0 … 0.8 s
constexpr int kPunch = 100;
constexpr int kRoll = 500;
constexpr double kDelta = 20;  // rad/s

qt::Command command(int step, double hover, double w_max) {
    if (step < kPunch) {
        return {{hover, hover, hover, hover}};
    }
    const double d = step < kRoll ? 0 : kDelta;
    return {{w_max - d, w_max + d, w_max + d, w_max - d}};
}

int run() {
    const model::QuadrotorParams<double> params{};
    const qt::Model quad(params);
    const double hover = quad.hover_rotor_speed();
    const double w_max = quad.max_rotor_speed();
    qt::State x = qt::hover_state(quad, 1.0);

    trace::Trace tr({.commit = {},
                     .block = "model/quadrotor",
                     .scenario = "tools/trace/scenarios/quadrotor/punch_out.cpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = kDtSim,
                     .dt_trace = kDtTrace});
    tr.vehicle_arm_length(params.arm_length);
    qt::declare_state(tr);
    tr.channel("rotor_speed_max", 1, "rad/s");
    tr.channel("speed", 1, "m/s");
    tr.channel("body_torque", 3, "N·m");

    double rotor_edge = -1;
    double speed_edge = -1;
    for (int k = 0; k < kSamples; ++k) {
        const int n = k * kStepsPerSample;
        const qt::Command u = command(n, hover, w_max);
        tr.sample(k * kDtTrace);
        qt::record_state(tr, quad, x, u);
        const double speed = branes::reflex::norm(x.velocity);
        const auto d = quad.derivative(x, u);
        tr.set("rotor_speed_max", w_max);
        tr.set("speed", speed);
        // J ω̇ + ω × J ω = τ: the torque the rotors make, from the model's own
        // derivative.
        const auto& j = params.inertia;
        const auto& w = x.body_rates;
        const auto jw = branes::reflex::vec3(j[0] * w(0), j[1] * w(1), j[2] * w(2));
        const auto gyro = branes::reflex::cross(w, jw);
        tr.set("body_torque",
               {j[0] * d.body_rates(0) + gyro(0), j[1] * d.body_rates(1) + gyro(1), j[2] * d.body_rates(2) + gyro(2)});
        if (rotor_edge < 0 && *std::max_element(x.rotor_speeds.begin(), x.rotor_speeds.end()) > 0.95 * w_max) {
            rotor_edge = k * kDtTrace;
        }
        if (speed_edge < 0 && speed > 5) {
            speed_edge = k * kDtTrace;
        }
        if (k + 1 == kSamples) {
            break;
        }
        for (int i = 0; i < kStepsPerSample; ++i) {
            x = quad.step(x, command(n + i, hover, w_max), kDtSim);
        }
    }

    tr.event(kPunch * kDtSim, "full throttle: every rotor commanded to ω_max");
    if (rotor_edge >= 0) {
        tr.event(rotor_edge, "rotor speeds above 95 % of ω_max: outside the expected envelope");
    }
    if (speed_edge >= 0) {
        tr.event(speed_edge, "airspeed above 5 m/s: outside the expected envelope");
    }
    tr.event(kRoll * kDtSim, "roll command ±20 rad/s: the left rotors are clamped, only the right respond");
    return qt::write(tr, "quadrotor/punch_out.json");
}

}  // namespace

int main() {
    return qt::guarded("punch_out", run);
}
