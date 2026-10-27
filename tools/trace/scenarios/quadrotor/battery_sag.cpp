// Battery sag: how much thrust headroom is left as the battery drains.
//
// The vehicle hovers at 1 m while its battery drains. Thrust per rotor speed
// falls with the energy drawn, s = 1 − sag · min(E / capacity, 1), so holding
// the weight needs faster rotors as the battery empties, and the most thrust
// the vehicle can make falls. The scenario holds hover with the command a
// perfect altitude controller would give: each step, the speed whose thrust
// at the current scale s carries a quarter of the weight. The rotors trail
// the rising command by the motor lag, so thrust stays just short of the
// weight and the vehicle sinks about 0.5 m over the run; an altitude loop's
// integrator would take that up.
//
// Parameters changed from RP-1, to show the effect in seconds: battery_sag
// 0.3 (30 % of thrust lost when empty), power_coefficient 8e-9 W/(rad/s)³
// (about 150 W at hover), and battery_capacity 1.5 kJ instead of 100 kJ, so the
// battery empties in about 8 s. Writes
// $REFLEX_TRACE_DIR/quadrotor/battery_sag.json.

#include "quadrotor_trace.hpp"

namespace {

namespace model = branes::reflex::model;
namespace trace = branes::reflex::trace;
namespace qt = branes::reflex::trace::quadrotor;

constexpr double kDtSim = 0.001;
constexpr int kStepsPerSample = 50;
constexpr double kDtTrace = kDtSim * kStepsPerSample;
constexpr int kSamples = 241;  // 0 … 12 s

int run() {
    model::QuadrotorParams<double> params{};
    params.battery_sag = 0.3;
    params.power_coefficient = 8e-9;
    params.battery_capacity = 1500;
    const qt::Model quad(params);
    const double weight = params.mass * params.gravity;
    const double w_max = quad.max_rotor_speed();
    qt::State x = qt::hover_state(quad, 1.0);

    const auto command = [&](const qt::State& s) {
        const double w = quad.rotor_speed_for_thrust(weight / 4 / quad.thrust_scale(s.energy));
        return qt::Command{{w, w, w, w}};
    };

    trace::Trace tr({.commit = {},
                     .block = "model/quadrotor",
                     .scenario = "tools/trace/scenarios/quadrotor/battery_sag.cpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = kDtSim,
                     .dt_trace = kDtTrace});
    tr.vehicle_arm_length(params.arm_length);
    qt::declare_state(tr);
    tr.channel("rotor_speed_max", 1, "rad/s");
    tr.channel("battery_used", 1, "1");
    tr.channel("thrust_scale", 1, "1");
    tr.channel("thrust_to_weight_max", 1, "1");

    double empty = -1;
    for (int k = 0; k < kSamples; ++k) {
        tr.sample(k * kDtTrace);
        qt::record_state(tr, quad, x, command(x));
        const double s = quad.thrust_scale(x.energy);
        tr.set("rotor_speed_max", w_max);
        tr.set("battery_used", x.energy < params.battery_capacity ? x.energy / params.battery_capacity : 1.0);
        tr.set("thrust_scale", s);
        tr.set("thrust_to_weight_max", 4 * s * params.max_thrust / weight);
        if (empty < 0 && x.energy >= params.battery_capacity) {
            empty = k * kDtTrace;
        }
        if (k + 1 == kSamples) {
            break;
        }
        for (int i = 0; i < kStepsPerSample; ++i) {
            x = quad.step(x, command(x), kDtSim);
        }
    }

    if (empty >= 0) {
        tr.event(empty, "battery empty: thrust per rotor speed is 70 % of full; it falls no further");
    }
    return qt::write(tr, "quadrotor/battery_sag.json");
}

}  // namespace

int main() {
    return qt::guarded("battery_sag", run);
}
