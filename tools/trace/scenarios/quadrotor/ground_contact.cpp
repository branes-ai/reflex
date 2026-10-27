// Ground contact: take-off, and a tilted landing.
//
// The vehicle starts on the ground with its rotors stopped. At 0.2 s every
// rotor is commanded to 90 % of the hover speed: 81 % of the weight, so the
// vehicle stays put while the rotors spin up. At 0.8 s the command rises to
// 110 %, and the vehicle lifts off once its vertical thrust passes its weight.
// At 1.6 s the command drops to 85 %, with a roll pulse on top (left rotors
// 80 rad/s faster than the right for 0.12 s, then the reverse), so the
// vehicle descends rolled right side down, and lands tilted.
// Contact is a stop: velocity and body rates go to zero at once, and the
// vehicle keeps its tilt, with no bounce, slide, or tip-over (finding 4 of
// the V&V plan).
//
// Check (BB-T4, property 5 of the V&V plan): while grounded with vertical
// thrust at or below the weight, the vehicle neither moves nor lifts off: the
// number of such steps that leave the ground or change the position or the
// attitude, compared exactly, is 0.
// Writes $REFLEX_TRACE_DIR/quadrotor/ground_contact.json.

#include "quadrotor_trace.hpp"

namespace {

namespace model = branes::reflex::model;
namespace trace = branes::reflex::trace;
namespace qt = branes::reflex::trace::quadrotor;

constexpr double kDtSim = 0.001;
constexpr int kStepsPerSample = 10;
constexpr double kDtTrace = kDtSim * kStepsPerSample;
constexpr int kSamples = 401;  // 0 … 4 s
constexpr int kSpinUp = 200;
constexpr int kTakeOff = 800;
constexpr int kDescend = 1600;
constexpr int kPulseHalf = 120;
constexpr double kDelta = 80;  // rad/s, as in roll_pulse.cpp but stronger

qt::Command command(int step, double hover) {
    if (step < kSpinUp) {
        return {};
    }
    if (step < kTakeOff) {
        const double w = 0.9 * hover;
        return {{w, w, w, w}};
    }
    if (step < kDescend) {
        const double w = 1.1 * hover;
        return {{w, w, w, w}};
    }
    // Left rotors (1, 2) faster, then slower: a roll to the right that stops.
    const double w = 0.85 * hover;
    double d = 0;
    if (step < kDescend + kPulseHalf) {
        d = kDelta;
    } else if (step < kDescend + 2 * kPulseHalf) {
        d = -kDelta;
    }
    return {{w - d, w + d, w + d, w - d}};
}

int run() {
    const model::QuadrotorParams<double> params{};
    const qt::Model quad(params);
    const double hover = quad.hover_rotor_speed();
    const double weight = params.mass * params.gravity;

    qt::State x;
    x.on_ground = true;

    trace::Trace tr({.commit = {},
                     .block = "model/quadrotor",
                     .scenario = "tools/trace/scenarios/quadrotor/ground_contact.cpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = kDtSim,
                     .dt_trace = kDtTrace});
    tr.vehicle_arm_length(params.arm_length);
    qt::declare_state(tr);
    tr.channel("on_ground", 1, "1");
    tr.channel("vertical_thrust", 1, "N");
    tr.channel("weight", 1, "N");

    const auto vertical_thrust = [&](const qt::State& s) {
        double total = 0;
        for (const double f : quad.rotor_thrusts(s)) {
            total += f;
        }
        const auto& q = s.attitude;
        return (1 - 2 * (q.x * q.x + q.y * q.y)) * total;  // R(2, 2) · ΣT
    };

    int grounded_steps_moved = 0;
    double lift_off = -1;
    double touch_down = -1;
    for (int k = 0; k < kSamples; ++k) {
        const int n = k * kStepsPerSample;
        tr.sample(k * kDtTrace);
        qt::record_state(tr, quad, x, command(n, hover));
        tr.set("on_ground", x.on_ground ? 1.0 : 0.0);
        tr.set("vertical_thrust", vertical_thrust(x));
        tr.set("weight", weight);
        if (k + 1 == kSamples) {
            break;
        }
        for (int i = 0; i < kStepsPerSample; ++i) {
            const qt::State before = x;
            x = quad.step(x, command(n + i, hover), kDtSim);
            if (before.on_ground && vertical_thrust(before) <= weight) {
                const auto& a = before;
                const bool moved = !x.on_ground || x.position(0) != a.position(0) || x.position(1) != a.position(1) ||
                                   x.position(2) != a.position(2) || x.attitude.w != a.attitude.w ||
                                   x.attitude.x != a.attitude.x || x.attitude.y != a.attitude.y ||
                                   x.attitude.z != a.attitude.z;
                grounded_steps_moved += moved ? 1 : 0;
            }
            const double t = (n + i + 1) * kDtSim;
            if (before.on_ground && !x.on_ground && lift_off < 0) {
                lift_off = t;
            }
            if (!before.on_ground && x.on_ground && touch_down < 0) {
                touch_down = t;
            }
        }
    }

    tr.event(kSpinUp * kDtSim, "rotors to 90 % of hover speed: 81 % of the weight, stays on the ground");
    tr.event(kTakeOff * kDtSim, "rotors to 110 %: lifts off when the vertical thrust passes the weight");
    if (lift_off > 0) {
        tr.event(lift_off, "lift-off");
    }
    tr.event(kDescend * kDtSim, "rotors to 85 % with a roll pulse: descends rolled right");
    if (touch_down > 0) {
        tr.event(touch_down, "touch-down: velocity and rates stop at once; the tilt stays");
    }
    tr.check({.id = "BB-T4",
              .metric = "grounded_steps_moved",
              .unit = "1",
              .value = static_cast<double>(grounded_steps_moved),
              .limit = 0,
              .comparison = "<="});
    return qt::write(tr, "quadrotor/ground_contact.json");
}

}  // namespace

int main() {
    return qt::guarded("ground_contact", run);
}
