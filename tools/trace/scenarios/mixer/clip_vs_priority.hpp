#pragma once

// Why the priority matters: a tilt and a yaw at once, at the throttle's
// extremes, flown through mix() and through plain per-rotor clipping.
//
// Two RP-1 vehicles hover at 50 m (clear of the ground for the whole run),
// each with its own cascade in attitude mode. At 0.1 s both are given the
// same target: yaw 60°, then roll 30° about the new heading, at a collective
// of the throttle times 4·T_max. By default the throttle is 1, a full
// punch-out: every rotor is asked for T_max before any torque.
//  - priority: mix() keeps roll and pitch whole, so the tilt (the direction
//    of thrust) comes first; it gives yaw up before the collective, and
//    reports each cut to the cascade's integrators (Cascade::feedback()), as
//    the cascade page wires it;
//  - clipped: allocate() with each rotor clamped to [T_min, T_max] on its
//    own, and no report. A clamp removes torque from whichever rotors hit a
//    limit, so the torque the rotors make points in a different direction
//    from the demand: yaw takes authority from the tilt.
// At the default the priority run's tilt error falls below 3° 0.15 s after
// the step and stays there, and its collective averages 47.5 N; its heading
// lags, 39° of 60° at 1 s, because yaw is what it gives up. The clipped run
// turns its heading to 59°, but its tilt wobbles up to 5° off the target
// until 0.77 s after the step, and its collective drops to 36 N at first and
// averages 45 N. At mid throttle nothing saturates, and the runs fly alike.
//
// Check (BB-I2), on the priority run: from 0.2 s after the step on, the tilt
// error within 3° (0.052 rad), 10 % of the 30° step, the band of the
// cascade's closed-loop roll step.
//
// The parameter is the throttle (F as a fraction of 4·T_max).
// clip_vs_priority.cpp writes the trace at the default; the explainer's live
// widget runs this function, compiled to WebAssembly, at the reader's value.

#include <branes/reflex/control/cascade.hpp>

#include "mixer_trace.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace branes::reflex::trace::scenarios::mixer_clip_vs_priority {

/// The widget's parameters, with the ranges the sliders offer.
struct Params {
    double throttle = 1;  ///< F / (4·T_max), in [0, 1]
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
inline constexpr int kSettle = 200;   // ticks after the step before the check's window
inline constexpr double kRoll = std::numbers::pi / 6;
inline constexpr double kYaw = std::numbers::pi / 3;

/// The angle between the body z axes of two attitudes [rad]: the tilt error.
inline double tilt_between(const Quat<double>& a, const Quat<double>& b) {
    const auto z = [](const Quat<double>& q) {
        return std::array<double, 3>{
            2 * (q.x * q.z + q.w * q.y), 2 * (q.y * q.z - q.w * q.x), 1 - 2 * (q.x * q.x + q.y * q.y)};
    };
    const auto za = z(a);
    const auto zb = z(b);
    return std::acos(std::clamp(za[0] * zb[0] + za[1] * zb[1] + za[2] * zb[2], -1.0, 1.0));
}

/// allocate(w), each rotor clamped on its own: a mixer with no priority.
inline control::MixerOutput<double> clip(const control::Mixer<double>& m, const control::Wrench<double>& w) {
    control::MixerOutput<double> out;
    out.thrusts = m.allocate(w);
    for (double& t : out.thrusts) {
        t = std::clamp(t, m.params().min_thrust, m.params().max_thrust);
    }
    out.achieved = m.wrench(out.thrusts);
    return out;
}

inline trace::Trace run(const Params& p) {
    if (!(p.throttle >= 0 && p.throttle <= 1)) {
        throw std::invalid_argument("mixer_clip_vs_priority: throttle outside the widget's range");
    }
    const model::QuadrotorParams<double> vehicle{};
    const qt::Model quad(vehicle);
    const control::CascadeConfig<double> cfg = control::cascade_config_from(vehicle);
    control::Cascade<double> cascade[2] = {control::Cascade<double>(cfg), control::Cascade<double>(cfg)};
    const control::Mixer<double> mixer(control::mixer_params_from(vehicle));

    const double weight = vehicle.mass * vehicle.gravity;
    const double punch = p.throttle * 4 * vehicle.max_thrust;
    const control::AttitudeTarget<double> level{Quat<double>{}, weight};
    // Yaw kYaw, then roll kRoll (ZYX).
    const Quat<double> qz{std::cos(kYaw / 2), 0, 0, std::sin(kYaw / 2)};
    const Quat<double> qx{std::cos(kRoll / 2), std::sin(kRoll / 2), 0, 0};
    const control::AttitudeTarget<double> rolled{qz * qx, punch};
    qt::State x[2] = {qt::hover_state(quad, 50.0), qt::hover_state(quad, 50.0)};

    trace::Trace tr({.commit = {},
                     .block = "control/mixer",
                     .scenario = "tools/trace/scenarios/mixer/clip_vs_priority.hpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = kDtSim,
                     .dt_trace = kDtTrace});
    tr.vehicle_arm_length(vehicle.arm_length);
    qt::declare_state(tr);
    tr.channel("clip_position", 3, "m");
    tr.channel("clip_attitude", 4, "1");
    tr.channel("clip_rotor_thrusts", 4, "N");
    tr.channel("tilt_error", 2, "rad");
    tr.channel("yaw", 2, "rad");
    tr.channel("yaw_setpoint", 1);
    tr.channel("collective", 2, "N");
    tr.channel("collective_demand", 1, "N");
    tr.channel("roll_torque", 2, "N·m");
    tr.channel("yaw_torque", 2, "N·m");

    double tilt_max = 0;  // the priority run's, from kSettle after the step
    for (int n = 0;; ++n) {
        const bool stepped = n >= kStart;
        control::MixerOutput<double> out[2];
        qt::Command u[2];
        for (int i = 0; i < 2; ++i) {
            const control::VehicleState<double> seen{x[i].position, x[i].velocity, x[i].attitude, x[i].body_rates};
            const control::Wrench<double> w = cascade[i].update(seen, stepped ? rolled : level, kDtSim);
            out[i] = i == 0 ? mixer.mix(w) : clip(mixer, w);
            if (i == 0) {
                cascade[i].feedback(out[i]);
            }
            u[i] = mixer.to_rotor_command(out[i].thrusts);
        }
        if (n % kStepsPerSample == 0) {
            tr.sample(n * kDtSim);
            qt::record_state(tr, quad, x[0], u[0]);
            const Quat<double>& q = x[1].attitude;
            tr.set("clip_position", {x[1].position(0), x[1].position(1), x[1].position(2)});
            tr.set("clip_attitude", {q.w, q.x, q.y, q.z});
            tr.set("clip_rotor_thrusts", quad.rotor_thrusts(x[1]));
            const Quat<double>& target = (stepped ? rolled : level).attitude;
            tr.set("tilt_error", {tilt_between(x[0].attitude, target), tilt_between(x[1].attitude, target)});
            tr.set("yaw", {qt::roll_pitch_yaw(x[0].attitude)[2], qt::roll_pitch_yaw(x[1].attitude)[2]});
            tr.set("yaw_setpoint", stepped ? kYaw : 0.0);
            tr.set("collective", {out[0].achieved.thrust, out[1].achieved.thrust});
            tr.set("collective_demand", stepped ? punch : weight);
            tr.set("roll_torque", {out[0].achieved.torque(0), out[1].achieved.torque(0)});
            tr.set("yaw_torque", {out[0].achieved.torque(2), out[1].achieved.torque(2)});
            if (n / kStepsPerSample + 1 == kSamples) {
                break;
            }
        }
        for (int i = 0; i < 2; ++i) {
            x[i] = quad.step(x[i], u[i], kDtSim);
        }
        if (n >= kStart + kSettle) {
            tilt_max = std::max(tilt_max, tilt_between(x[0].attitude, rolled.attitude));
        }
    }

    tr.event(kStart * kDtSim, "target: yaw 60°, roll 30°, collective at the throttle");
    tr.check({.id = "BB-I2",
              .metric = "tilt_error_after_0.2s",
              .unit = "rad",
              .value = tilt_max,
              .limit = 0.1 * kRoll,
              .comparison = "<="});
    return tr;
}

}  // namespace branes::reflex::trace::scenarios::mixer_clip_vs_priority
