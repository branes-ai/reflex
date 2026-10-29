#pragma once

// Shared by the cascade trace scenarios: one RP-1 vehicle flown by a cascade
// through mix(), wired as the cascade page shows it, and the tilt error. Not
// a scenario itself (CMake builds only the .cpp files in this directory).

#include <branes/reflex/control/cascade.hpp>
#include <branes/reflex/control/mixer.hpp>
#include <branes/reflex/math/quat.hpp>
#include <branes/reflex/math/vec3.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include "../quadrotor/quadrotor_trace.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace branes::reflex::trace::cascade {

namespace model = branes::reflex::model;
namespace control = branes::reflex::control;
namespace qt = branes::reflex::trace::quadrotor;

inline constexpr double kDtSim = 0.001;  // the rate-loop period

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

/// The tilt of an attitude from level [rad].
inline double tilt_of(const Quat<double>& q) {
    return tilt_between(q, Quat<double>{});
}

/// The heading error of q against q_sp [rad]: the yaw about the thrust axis
/// left once the tilt is removed. The error q⁻¹ ⊗ q_sp is split as the
/// attitude loop splits it, into the shortest rotation q_red that moves the
/// body z axis onto the target's, followed by a yaw about it.
inline double heading_between(const Quat<double>& q, const Quat<double>& q_sp) {
    const Quat<double> e = conjugate(q) * q_sp;
    const Vec3<double> e3 = vec3(0.0, 0.0, 1.0);
    const Vec3<double> z = rotate(e, e3);  // the target's thrust axis, in the body frame
    const Vec3<double> axis = cross(e3, z);
    const double s = norm(axis);
    const double angle = std::atan2(s, dot(e3, z));
    const Vec3<double> rv = s > 1e-12 ? scale(angle / s, axis) : vec3(0.0, 0.0, 0.0);
    const Quat<double> yaw = conjugate(from_rotation_vector(rv)) * e;
    return 2 * std::atan2(std::abs(yaw.z), std::abs(yaw.w));
}

/// One vehicle in closed loop: cascade → mix() → feedback → QuadrotorModel,
/// one rate-loop tick per control() and advance(), with the controller
/// seeing the true state.
struct Flight {
    model::QuadrotorParams<double> vehicle;
    qt::Model quad;
    control::Cascade<double> cascade;
    control::Mixer<double> mixer;
    qt::State x;
    control::Wrench<double> wrench{};
    control::MixerOutput<double> out{};
    qt::Command u{};

    Flight(const model::QuadrotorParams<double>& p, const control::CascadeConfig<double>& cfg, double height)
        : vehicle(p), quad(p), cascade(cfg), mixer(control::mixer_params_from(p)), x(qt::hover_state(quad, height)) {}

    /// Runs the controller on the current state; advance() then flies the command.
    template <typename Setpoint>
    void control(const Setpoint& sp) {
        const control::VehicleState<double> seen{x.position, x.velocity, x.attitude, x.body_rates};
        wrench = cascade.update(seen, sp, kDtSim);
        out = mixer.mix(wrench);
        cascade.feedback(out);
        u = mixer.to_rotor_command(out.thrusts);
    }

    void advance() {
        x = quad.step(x, u, kDtSim);
    }
};

}  // namespace branes::reflex::trace::cascade
