// Writes $REFLEX_TRACE_DIR/cascade/climb.json: a climb against the thrust
// limit, and the velocity integrator held while the limit cuts.
//
// RP-1 hovers at 5 m in position mode, with the collective capped at
// 1.25·m·g (rather than 80 % of 4·T_max) and the climb-rate limit raised to
// 10 m/s, so that a 10 m climb asks for more thrust than the cap for over
// 2 s. At 0 s the setpoint moves to 15 m. The vertical velocity setpoint
// jumps to the 10 m/s limit, the velocity PID's output sits at its own limit
// (an acceleration of g up, 19.6 N in all), and the position loop cuts the
// force to the 12.3 N cap and reports the cut. Each holds the integral: the
// PID's own clamp, and the reported cut on the next position tick. So the
// integral stays at 0 through the 2.2 s of the cut instead of winding up,
// and the climb ends without overshoot. It settles from below instead, 2 %
// short at 5 s: the integral went negative while the vehicle braked, and
// unwinds over the next few seconds. A second vehicle, uncapped, flies a
// 1 m step for comparison; the trace records its normalized height alongside.
//
// Check (BB-I2), the closed-loop criterion of reflex#20: the capped climb's
// overshoot is at most 5 points above the unsaturated step's.

#include "cascade_trace.hpp"

#include <algorithm>
#include <stdexcept>

int main() {
    namespace trace = branes::reflex::trace;
    namespace qt = branes::reflex::trace::quadrotor;
    namespace ct = branes::reflex::trace::cascade;
    namespace control = branes::reflex::control;
    using branes::reflex::vec3;

    return qt::guarded("climb", [] {
        constexpr int kStepsPerSample = 20;
        constexpr int kTicks = 15000;  // 15 s
        constexpr double kStart = 5;
        constexpr double kClimb = 10;
        constexpr double kStep = 1;

        const branes::reflex::model::QuadrotorParams<double> vehicle{};
        const double weight = vehicle.mass * vehicle.gravity;
        control::CascadeConfig<double> capped = control::cascade_config_from(vehicle);
        capped.position.max_thrust = 1.25 * weight;
        capped.position.max_speed_up = 10.0;
        ct::Flight f(vehicle, capped, kStart);
        ct::Flight ref(vehicle, control::cascade_config_from(vehicle), kStart);
        control::PositionSetpoint<double> sp;
        sp.position = vec3(0.0, 0.0, kStart + kClimb);
        control::PositionSetpoint<double> sp_ref;
        sp_ref.position = vec3(0.0, 0.0, kStart + kStep);

        trace::Trace tr({.commit = {},
                         .block = "control/cascade",
                         .scenario = "tools/trace/scenarios/cascade/climb.cpp",
                         .seed = 0,
                         .compiler = {},
                         .dt_sim = ct::kDtSim,
                         .dt_trace = ct::kDtSim * kStepsPerSample});
        tr.channel("height", 2, "m");
        tr.channel("vertical_velocity", 2, "m/s");
        tr.channel("collective", 3, "N");
        tr.channel("vertical_integral", 1, "m/s²");
        tr.channel("vertical_cut", 1, "1");
        tr.channel("step_fraction", 2, "1");

        double z_max = kStart;
        double z_ref_max = kStart;
        double clipped = 0;
        for (int n = 0; n <= kTicks; ++n) {
            f.control(sp);
            ref.control(sp_ref);
            const auto& pos = f.cascade.position();
            if (n % kStepsPerSample == 0) {
                tr.sample(n * ct::kDtSim);
                tr.set("height", {f.x.position(2), sp.position(2)});
                tr.set("vertical_velocity", {f.x.velocity(2), pos.velocity_setpoint()(2)});
                // The vertical force the velocity loop asks for, before the cap.
                const double demand = vehicle.mass * (pos.axis(2).output() + vehicle.gravity);
                tr.set("collective", {demand, f.cascade.target().thrust, capped.position.max_thrust});
                tr.set("vertical_integral", pos.axis(2).integral());
                tr.set("vertical_cut", static_cast<double>(pos.cut()[2]));
                tr.set("step_fraction", {(f.x.position(2) - kStart) / kClimb, (ref.x.position(2) - kStart) / kStep});
            }
            clipped += pos.cut()[2] != 0 ? ct::kDtSim : 0;
            f.advance();
            ref.advance();
            z_max = std::max(z_max, f.x.position(2));
            z_ref_max = std::max(z_ref_max, ref.x.position(2));
        }

        const double overshoot = std::max(0.0, (z_max - kStart) / kClimb - 1);
        const double overshoot_ref = std::max(0.0, (z_ref_max - kStart) / kStep - 1);
        if (clipped < 2.0) {
            throw std::runtime_error("climb: the thrust cap cut for less than 2 s");
        }
        tr.event(0.0, "setpoint 10 m up; the vertical force is cut to the cap");
        tr.check({.id = "BB-I2",
                  .metric = "overshoot_above_unsaturated",
                  .unit = "1",
                  .value = overshoot - overshoot_ref,
                  .limit = 0.05,
                  .comparison = "<="});
        return qt::write(tr, "cascade/climb.json");
    });
}
