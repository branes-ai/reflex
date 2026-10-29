// Writes $REFLEX_TRACE_DIR/cascade/tilt_yaw.json: tilt first, heading after.
//
// Two RP-1 vehicles hover at 20 m, each with its own cascade in attitude
// mode at hover thrust. At 0.1 s both are given a 30° roll; the first is
// also given a 90° turn about its tilted thrust axis (q_sp = q_roll ⊗
// q_yaw), so both targets point the thrust the same way and differ only in
// heading. The attitude loop splits the error into a tilt and a yaw about the
// thrust axis, and runs each at its own gain: roll and pitch at 10 /s, yaw at
// 5 /s and at most 4 rad/s. So the turning vehicle tilts exactly as the other
// does and drifts along the same path, while its heading follows later.
//
// Check (BB-I2), the closed-loop criterion of reflex#20: the tilt of the
// turning vehicle is within 2° of its target no later than the other's, to
// one attitude-loop period (4 ms).

#include "cascade_trace.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

int main() {
    namespace trace = branes::reflex::trace;
    namespace qt = branes::reflex::trace::quadrotor;
    namespace ct = branes::reflex::trace::cascade;
    namespace control = branes::reflex::control;
    using branes::reflex::Quat;

    return qt::guarded("tilt_yaw", [] {
        constexpr int kStepsPerSample = 5;
        constexpr int kSamples = 301;  // 0 … 1.5 s
        constexpr int kStart = 100;    // tick of the step
        constexpr double kRoll = std::numbers::pi / 6;
        constexpr double kYaw = std::numbers::pi / 2;
        constexpr double kBand = 2 * std::numbers::pi / 180;

        const branes::reflex::model::QuadrotorParams<double> vehicle{};
        const control::CascadeConfig<double> cfg = control::cascade_config_from(vehicle);
        ct::Flight f[2] = {ct::Flight(vehicle, cfg, 20.0), ct::Flight(vehicle, cfg, 20.0)};

        const double weight = vehicle.mass * vehicle.gravity;
        const Quat<double> q_roll{std::cos(kRoll / 2), std::sin(kRoll / 2), 0, 0};
        const Quat<double> q_yaw{std::cos(kYaw / 2), 0, 0, std::sin(kYaw / 2)};
        const control::AttitudeTarget<double> level{Quat<double>{}, weight};
        const control::AttitudeTarget<double> target[2] = {{q_roll * q_yaw, weight}, {q_roll, weight}};

        trace::Trace tr({.commit = {},
                         .block = "control/cascade",
                         .scenario = "tools/trace/scenarios/cascade/tilt_yaw.cpp",
                         .seed = 0,
                         .compiler = {},
                         .dt_sim = ct::kDtSim,
                         .dt_trace = ct::kDtSim * kStepsPerSample});
        tr.vehicle_arm_length(vehicle.arm_length);
        qt::declare_state(tr);
        tr.channel("tilt_position", 3, "m");
        tr.channel("tilt_attitude", 4, "1");
        tr.channel("tilt_rotor_thrusts", 4, "N");
        tr.channel("tilt_error", 2, "rad");
        tr.channel("heading_error", 1, "rad");
        tr.channel("rate_setpoint", 3, "rad/s");
        tr.channel("separation", 1, "m");

        double last_out[2] = {0, 0};  // the last time each tilt was outside the band
        for (int n = 0;; ++n) {
            const bool stepped = n >= kStart;
            for (int i = 0; i < 2; ++i) {
                f[i].control(stepped ? target[i] : level);
            }
            if (n % kStepsPerSample == 0) {
                const double t = n * ct::kDtSim;
                tr.sample(t);
                qt::record_state(tr, f[0].quad, f[0].x, f[0].u);
                const Quat<double>& q = f[1].x.attitude;
                tr.set("tilt_position", {f[1].x.position(0), f[1].x.position(1), f[1].x.position(2)});
                tr.set("tilt_attitude", {q.w, q.x, q.y, q.z});
                tr.set("tilt_rotor_thrusts", f[1].quad.rotor_thrusts(f[1].x));
                const auto& a0 = (stepped ? target[0] : level).attitude;
                const auto& a1 = (stepped ? target[1] : level).attitude;
                tr.set("tilt_error", {ct::tilt_between(f[0].x.attitude, a0), ct::tilt_between(f[1].x.attitude, a1)});
                tr.set("heading_error", ct::heading_between(f[0].x.attitude, a0));
                const auto& r = f[0].cascade.rate_setpoint();
                tr.set("rate_setpoint", {r(0), r(1), r(2)});
                const auto d = branes::reflex::sub(f[0].x.position, f[1].x.position);
                tr.set("separation", branes::reflex::norm(d));
                if (n / kStepsPerSample + 1 == kSamples) {
                    break;
                }
            }
            for (int i = 0; i < 2; ++i) {
                f[i].advance();
                if (stepped && ct::tilt_between(f[i].x.attitude, target[i].attitude) > kBand) {
                    last_out[i] = (n + 1) * ct::kDtSim;
                }
            }
        }

        // Both must settle, or equal "last outside" times would pass the check vacuously.
        for (int i = 0; i < 2; ++i) {
            if (ct::tilt_between(f[i].x.attitude, target[i].attitude) > kBand) {
                throw std::runtime_error("tilt_yaw: a vehicle did not settle within 2° of its target");
            }
        }
        tr.event(kStart * ct::kDtSim, "target: roll 30°; the solid vehicle also turns 90°");
        tr.check({.id = "BB-I2",
                  .metric = "tilt_settle_delay_from_yaw",
                  .unit = "s",
                  .value = last_out[0] - last_out[1],
                  .limit = 4e-3,
                  .comparison = "<="});
        return qt::write(tr, "cascade/tilt_yaw.json");
    });
}
