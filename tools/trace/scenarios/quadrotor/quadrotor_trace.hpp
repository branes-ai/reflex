#pragma once

// Shared recording for the quadrotor trace scenarios: the standard state
// channels, Euler angles for the plots, and the write-and-report tail. Not a
// scenario itself (CMake builds only the .cpp files in this directory).

#include <branes/reflex/model/quadrotor.hpp>
#include <branes/reflex/trace/writer.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <string>

namespace branes::reflex::trace::quadrotor {

namespace model = branes::reflex::model;

using Model = model::QuadrotorModel<double>;
using State = model::QuadrotorState<double>;
using Command = model::RotorCommand<double>;

/// Roll, pitch, yaw (ZYX Tait–Bryan) of a body-to-world quaternion [rad].
inline std::array<double, 3> roll_pitch_yaw(const Quat<double>& q) {
    const double roll = std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
    const double pitch = std::asin(std::clamp(2 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
    const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    return {roll, pitch, yaw};
}

/// Declares the state channels that record_state() fills: position, velocity,
/// attitude, body_rates, rotor_speeds, rotor_speeds_setpoint, rotor_thrusts,
/// and roll_pitch_yaw.
inline void declare_state(Trace& tr) {
    tr.channel("position", 3);
    tr.channel("velocity", 3);
    tr.channel("attitude", 4);
    tr.channel("body_rates", 3);
    tr.channel("rotor_speeds", 4);
    tr.channel("rotor_speeds_setpoint", 4);
    tr.channel("rotor_thrusts", 4);
    tr.channel("roll_pitch_yaw", 3, "rad");
}

/// Records state x and command u into the current sample. The setpoint is the
/// command as given, before the model clamps it to [0, ω_max].
inline void record_state(Trace& tr, const Model& m, const State& x, const Command& u) {
    tr.set("position", {x.position(0), x.position(1), x.position(2)});
    tr.set("velocity", {x.velocity(0), x.velocity(1), x.velocity(2)});
    tr.set("attitude", {x.attitude.w, x.attitude.x, x.attitude.y, x.attitude.z});
    tr.set("body_rates", {x.body_rates(0), x.body_rates(1), x.body_rates(2)});
    tr.set("rotor_speeds", x.rotor_speeds);
    tr.set("rotor_speeds_setpoint", u.speeds);
    tr.set("rotor_thrusts", m.rotor_thrusts(x));
    tr.set("roll_pitch_yaw", roll_pitch_yaw(x.attitude));
}

/// A hovering state at height z: level, at rest, rotors at hover speed.
inline State hover_state(const Model& m, double z) {
    State x;
    x.position = vec3(0.0, 0.0, z);
    const double w = m.hover_rotor_speed();
    x.rotor_speeds = {w, w, w, w};
    return x;
}

/// Writes the trace to $REFLEX_TRACE_DIR/<relative> and returns the exit code.
inline int write(const Trace& tr, const std::string& relative) {
    const auto path = output_path(relative);
    if (path.empty()) {
        std::fprintf(stderr, "set REFLEX_TRACE_DIR to write %s\n", relative.c_str());
        return 2;
    }
    tr.write(path);
    std::printf("wrote %s (%zu samples)\n", path.string().c_str(), tr.samples());
    return 0;
}

/// Runs a scenario body, reporting an exception as exit code 1.
template <typename F>
int guarded(const char* name, F run) {
    try {
        return run();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "%s: %s\n", name, e.what());
        return 1;
    }
}

}  // namespace branes::reflex::trace::quadrotor
