#pragma once

// A waypoint square, the closed-loop path test of reflex#20, with each loop's
// setpoint recorded.
//
// RP-1 hovers at a corner of a 10 m square at 5 m, in position mode. The
// setpoint runs around the square, each leg a triangle speed profile: from
// rest at a corner to the peak speed v at mid-leg and back to rest at the
// next corner, at a constant acceleration a = v²/10. Each setpoint carries
// the profile's velocity and acceleration as feed-forward, scaled by the
// feed-forward fraction. Two seconds of hover at the last corner end the run.
//  - The position loop turns the position error into a velocity setpoint
//    and adds the velocity feed-forward;
//  - the velocity loop turns the velocity error into an acceleration, adds
//    the acceleration feed-forward, and tilts the thrust axis toward it;
//  - the attitude and rate loops fly that tilt, as in the roll step.
// At the defaults the path loops have little to correct: the vehicle stays
// within 0.11 m of the setpoint and 0.07 m of the square. Without the
// feed-forward the position loop is the only source of motion, and it needs
// an error to move at all: the vehicle trails the setpoint by up to 2.7 m
// along each leg and reaches its top speed of 4.5 m/s late. It stays on the
// square (within 0.09 m), because each leg is straight and ends at rest. At
// 8 m/s with full feed-forward, the profile's 6.4 m/s² plus the corrections
// reach the 40° tilt limit, and the vehicle runs 0.6 m wide of the corners.
//
// Check (BB-I2): the RMS cross-track error, the distance from the square's
// edges, at most 0.3 m, the criterion of reflex#20 for v = 5 m/s.
//
// The parameters are the peak speed and the feed-forward fraction.
// waypoint.cpp writes the trace at the defaults (5 m/s, full feed-forward);
// the explainer's live widget runs this function, compiled to WebAssembly,
// at the reader's values.

#include "cascade_trace.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace branes::reflex::trace::scenarios::cascade_waypoint {

/// The widget's parameters, with the ranges the sliders offer.
struct Params {
    double speed = 5;        ///< peak speed v along each leg [m/s], in [2, 8]
    double feedforward = 1;  ///< fraction of the velocity and acceleration feed-forward, in [0, 1]
};

namespace control = branes::reflex::control;
namespace trace = branes::reflex::trace;
namespace qt = branes::reflex::trace::quadrotor;
namespace ct = branes::reflex::trace::cascade;

inline constexpr int kStepsPerSample = 40;
inline constexpr double kSide = 10;
inline constexpr double kHeight = 5;
inline constexpr double kHold = 2;  // hover at the end [s]
inline constexpr double kCorners[5][2] = {{0, 0}, {10, 0}, {10, 10}, {0, 10}, {0, 0}};

/// The distance from p to the nearest edge of the square, in the horizontal plane [m].
inline double cross_track(const Vec3<double>& p) {
    double best = 1e9;
    for (int k = 0; k < 4; ++k) {
        const double ax = kCorners[k][0];
        const double ay = kCorners[k][1];
        const double vx = kCorners[k + 1][0] - ax;
        const double vy = kCorners[k + 1][1] - ay;
        const double u = std::clamp(((p(0) - ax) * vx + (p(1) - ay) * vy) / (vx * vx + vy * vy), 0.0, 1.0);
        best = std::min(best, std::hypot(ax + u * vx - p(0), ay + u * vy - p(1)));
    }
    return best;
}

inline trace::Trace run(const Params& p) {
    if (!(p.speed >= 2 && p.speed <= 8) || !(p.feedforward >= 0 && p.feedforward <= 1)) {
        throw std::invalid_argument("cascade_waypoint: parameters outside the widget's ranges");
    }
    const double v = p.speed;
    const double accel = v * v / kSide;
    const double leg = 2 * kSide / v;
    const double t_end = 4 * leg + kHold;
    const int ticks = static_cast<int>(std::lround(t_end / ct::kDtSim));

    // The setpoint at time t: position along the profile, feed-forward scaled.
    const auto setpoint = [&](double t) {
        const int k = std::min(3, static_cast<int>(t / leg));
        const double s = std::min(t - k * leg, leg);
        double d = kSide;
        double vel = 0;
        double acc = 0;
        if (s < leg / 2) {
            d = 0.5 * accel * s * s;
            vel = accel * s;
            acc = accel;
        } else if (s < leg) {
            const double r = s - leg / 2;
            d = kSide / 2 + v * r - 0.5 * accel * r * r;
            vel = v - accel * r;
            acc = -accel;
        }
        const double ux = (kCorners[k + 1][0] - kCorners[k][0]) / kSide;
        const double uy = (kCorners[k + 1][1] - kCorners[k][1]) / kSide;
        control::PositionSetpoint<double> sp;
        sp.position = vec3(kCorners[k][0] + ux * d, kCorners[k][1] + uy * d, kHeight);
        sp.velocity = vec3(p.feedforward * ux * vel, p.feedforward * uy * vel, 0.0);
        sp.acceleration = vec3(p.feedforward * ux * acc, p.feedforward * uy * acc, 0.0);
        return std::pair{sp, vel};
    };

    const model::QuadrotorParams<double> vehicle{};
    ct::Flight f(vehicle, control::cascade_config_from(vehicle), kHeight);

    trace::Trace tr({.commit = {},
                     .block = "control/cascade",
                     .scenario = "tools/trace/scenarios/cascade/waypoint.hpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = ct::kDtSim,
                     .dt_trace = ct::kDtSim * kStepsPerSample});
    tr.vehicle_arm_length(vehicle.arm_length);
    qt::declare_state(tr);
    tr.channel("setpoint_position", 3, "m");
    tr.channel("target_attitude", 4, "1");
    tr.channel("cross_track", 1, "m");
    tr.channel("along_track_lag", 1, "m");
    tr.channel("speed", 2, "m/s");
    tr.channel("tilt", 2, "rad");

    double sum2 = 0;
    for (int n = 0; n <= ticks; ++n) {
        const double t = n * ct::kDtSim;
        const auto [sp, profile_speed] = setpoint(t);
        f.control(sp);
        if (n % kStepsPerSample == 0) {
            tr.sample(t);
            qt::record_state(tr, f.quad, f.x, f.u);
            const Quat<double>& q = f.cascade.target().attitude;
            tr.set("setpoint_position", {sp.position(0), sp.position(1), sp.position(2)});
            tr.set("target_attitude", {q.w, q.x, q.y, q.z});
            tr.set("cross_track", cross_track(f.x.position));
            tr.set("along_track_lag", norm(sub(sp.position, f.x.position)));
            tr.set("speed", {profile_speed, norm(f.x.velocity)});
            tr.set("tilt", {ct::tilt_of(q), ct::tilt_of(f.x.attitude)});
        }
        const double c = cross_track(f.x.position);
        sum2 += c * c;
        f.advance();
    }

    for (int k = 0; k < 4; ++k) {
        tr.event(k * leg, k == 0 ? "leg 1: the setpoint sets off along +x" : "corner: the setpoint turns 90°");
    }
    tr.event(4 * leg, "back at the start: hover");
    tr.check({.id = "BB-I2",
              .metric = "cross_track_rms",
              .unit = "m",
              .value = std::sqrt(sum2 / (ticks + 1)),
              .limit = 0.3,
              .comparison = "<="});
    return tr;
}

}  // namespace branes::reflex::trace::scenarios::cascade_waypoint
