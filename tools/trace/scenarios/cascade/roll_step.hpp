#pragma once

// A 30° roll step through the full loop: cascade → mixer → QuadrotorModel.
//
// The vehicle hovers at 2 m. At 0.1 s the cascade, in attitude mode, is given
// a target of 30° roll at hover thrust. The attitude loop turns the angle
// error into a roll-rate setpoint, the rate loop turns the rate error into a
// torque, the mixer turns the wrench into rotor thrusts, and the model flies
// them. The loops run at the cascade's rates (rate 1 kHz, attitude 250 Hz),
// and the controller sees the true state. The vehicle holds the roll, so it
// accelerates to the right and sinks: a constant attitude at hover thrust is
// not a hover.
//
// The parameters are the roll and pitch gains of the attitude loop (K, in
// 1/s) and of the rate loop (k_p, in 1/s). Every other gain and limit is
// cascade_config_from(RP-1): K = 10 /s and k_p = 50 /s by default. roll_step.cpp
// writes the trace at those defaults; the cascade page's live widget runs this
// function, compiled to WebAssembly, at the reader's gains.
//
// Check (BB-I2): the overshoot of the roll angle over the 30° step, at most
// 10 %, the closed-loop acceptance criterion of reflex#20 for this step.

#include <branes/reflex/control/cascade.hpp>
#include <branes/reflex/control/mixer.hpp>

#include "../quadrotor/quadrotor_trace.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace branes::reflex::trace::scenarios::roll_step {

/// The widget's parameters, with the ranges the sliders offer.
struct Params {
    double attitude_gain = 10;  ///< K for roll and pitch [1/s], in [2, 30]
    double rate_gain = 50;      ///< k_p for roll and pitch [1/s], in [10, 150]
};

namespace model = branes::reflex::model;
namespace control = branes::reflex::control;
namespace trace = branes::reflex::trace;
namespace qt = branes::reflex::trace::quadrotor;

inline constexpr double kDtSim = 0.001;  // the rate-loop period
inline constexpr int kStepsPerSample = 5;
inline constexpr double kDtTrace = kDtSim * kStepsPerSample;
inline constexpr int kSamples = 201;  // 0 … 1 s
inline constexpr int kStart = 100;    // tick of the step
inline constexpr double kRoll = std::numbers::pi / 6;

inline trace::Trace run(const Params& p) {
    if (!(p.attitude_gain >= 2 && p.attitude_gain <= 30) || !(p.rate_gain >= 10 && p.rate_gain <= 150)) {
        throw std::invalid_argument("roll_step: gains outside the widget's ranges");
    }
    const model::QuadrotorParams<double> vehicle{};
    const qt::Model quad(vehicle);
    control::CascadeConfig<double> cfg = control::cascade_config_from(vehicle);
    cfg.attitude.gain[0] = cfg.attitude.gain[1] = p.attitude_gain;
    cfg.rate.axis[0].kp = cfg.rate.axis[1].kp = p.rate_gain;
    if (!control::cascade_config_valid(cfg)) {
        throw std::invalid_argument("roll_step: invalid cascade configuration");
    }
    control::Cascade<double> cascade(cfg);
    const control::Mixer<double> mixer(control::mixer_params_from(vehicle));

    const double weight = vehicle.mass * vehicle.gravity;
    const control::AttitudeTarget<double> level{Quat<double>{}, weight};
    const control::AttitudeTarget<double> rolled{Quat<double>{std::cos(kRoll / 2), std::sin(kRoll / 2), 0, 0}, weight};
    qt::State x = qt::hover_state(quad, 2.0);

    trace::Trace tr({.commit = {},
                     .block = "control/cascade",
                     .scenario = "tools/trace/scenarios/cascade/roll_step.hpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = kDtSim,
                     .dt_trace = kDtTrace});
    tr.vehicle_arm_length(vehicle.arm_length);
    qt::declare_state(tr);
    tr.channel("roll_setpoint", 1);
    tr.channel("body_rates_setpoint", 3);
    tr.channel("body_torque", 3, "N·m");

    double roll_max = 0;
    for (int n = 0; n <= kSamples * kStepsPerSample; ++n) {
        const control::VehicleState<double> seen{x.position, x.velocity, x.attitude, x.body_rates};
        const bool stepped = n >= kStart;
        const control::Wrench<double> w = cascade.update(seen, stepped ? rolled : level, kDtSim);
        const control::MixerOutput<double> out = mixer.mix(w);
        cascade.feedback(out);
        const qt::Command u = mixer.to_rotor_command(out.thrusts);
        if (n % kStepsPerSample == 0) {
            tr.sample(n * kDtSim);
            qt::record_state(tr, quad, x, u);
            const Vec3<double>& sp = cascade.rate_setpoint();
            tr.set("roll_setpoint", stepped ? kRoll : 0.0);
            tr.set("body_rates_setpoint", {sp(0), sp(1), sp(2)});
            tr.set("body_torque", {out.achieved.torque(0), out.achieved.torque(1), out.achieved.torque(2)});
            if (n / kStepsPerSample + 1 == kSamples) {
                break;
            }
        }
        x = quad.step(x, u, kDtSim);
        roll_max = std::max(roll_max, qt::roll_pitch_yaw(x.attitude)[0]);
    }

    tr.event(kStart * kDtSim, "roll target 30°, at hover thrust");
    tr.check({.id = "BB-I2",
              .metric = "roll_overshoot",
              .unit = "1",
              .value = std::max(0.0, roll_max / kRoll - 1),
              .limit = 0.10,
              .comparison = "<="});
    return tr;
}

}  // namespace branes::reflex::trace::scenarios::roll_step
