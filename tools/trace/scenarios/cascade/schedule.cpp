// Writes $REFLEX_TRACE_DIR/cascade/schedule.json: the cascade's three loop
// rates, every rate-loop tick of the first 100 ms of a moving setpoint.
//
// RP-1 hovers at 5 m in position mode. From 0 s the position setpoint moves
// forward at 5 m/s, with no feed-forward, so the position error grows on
// every position tick. The position loop runs on every 20th tick (50 Hz) and
// holds its target, a tilt toward +x, in between; the attitude loop runs on
// every 4th tick (250 Hz) and holds its rate setpoint; the rate loop makes a
// new torque on every tick (1 kHz). Sampled at every tick, each held output
// is a staircase with its loop's period.
//
// Check (BB-T1): updates off the schedule, counted as a change in the
// position loop's target on a tick that is not a multiple of 20, or in the
// rate setpoint on one that is not a multiple of 4. It must be 0.

#include "cascade_trace.hpp"

int main() {
    namespace trace = branes::reflex::trace;
    namespace qt = branes::reflex::trace::quadrotor;
    namespace ct = branes::reflex::trace::cascade;
    namespace control = branes::reflex::control;
    using branes::reflex::Quat;
    using branes::reflex::Vec3;
    using branes::reflex::vec3;

    return qt::guarded("schedule", [] {
        constexpr int kTicks = 100;
        constexpr double kSpeed = 5;
        const branes::reflex::model::QuadrotorParams<double> vehicle{};
        const control::CascadeConfig<double> cfg = control::cascade_config_from(vehicle);
        ct::Flight f(vehicle, cfg, 5.0);
        control::PositionSetpoint<double> sp;

        trace::Trace tr({.commit = {},
                         .block = "control/cascade",
                         .scenario = "tools/trace/scenarios/cascade/schedule.cpp",
                         .seed = 0,
                         .compiler = {},
                         .dt_sim = ct::kDtSim,
                         .dt_trace = ct::kDtSim});
        tr.channel("velocity_setpoint_x", 1, "m/s");
        tr.channel("target_tilt", 1, "rad");
        tr.channel("pitch_rate_setpoint", 1, "rad/s");
        tr.channel("pitch_torque", 1, "N·m");
        tr.channel("tick", 1, "1");

        int off_schedule = 0;
        Quat<double> last_target = f.cascade.target().attitude;
        Vec3<double> last_rate = f.cascade.rate_setpoint();
        for (int n = 0; n <= kTicks; ++n) {
            sp.position = vec3(kSpeed * n * ct::kDtSim, 0.0, 5.0);
            f.control(sp);
            const Quat<double>& q = f.cascade.target().attitude;
            const Vec3<double>& r = f.cascade.rate_setpoint();
            const bool target_moved =
                q.w != last_target.w || q.x != last_target.x || q.y != last_target.y || q.z != last_target.z;
            const bool rate_moved = r(0) != last_rate(0) || r(1) != last_rate(1) || r(2) != last_rate(2);
            off_schedule += (target_moved && n % static_cast<int>(cfg.position_divider) != 0) ? 1 : 0;
            off_schedule += (rate_moved && n % static_cast<int>(cfg.attitude_divider) != 0) ? 1 : 0;
            last_target = q;
            last_rate = r;

            tr.sample(n * ct::kDtSim);
            tr.set("velocity_setpoint_x", f.cascade.position().velocity_setpoint()(0));
            tr.set("target_tilt", ct::tilt_of(q));
            tr.set("pitch_rate_setpoint", r(1));
            tr.set("pitch_torque", f.wrench.torque(1));
            tr.set("tick", static_cast<double>(n));
            f.advance();
        }

        tr.event(0.0, "the setpoint sets off at 5 m/s; all three loops run, with no error yet");
        tr.event(0.020, "position loop: 0.1 m behind, so a 3.3° tilt target");
        tr.event(0.024, "attitude loop, 4 ms later: the next rate setpoint toward that tilt");
        tr.check({.id = "BB-T1",
                  .metric = "updates_off_schedule",
                  .unit = "1",
                  .value = static_cast<double>(off_schedule),
                  .limit = 0,
                  .comparison = "<="});
        return qt::write(tr, "cascade/schedule.json");
    });
}
