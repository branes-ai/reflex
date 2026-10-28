// Timing faults: what update() does with a bad dt, as a debugging replay.
//
// The windup loop's PI controller (k_p = 1, k_i = 10; output and integral
// ±1) drives a first-order plant (T = 0.1 s) toward a 0.5 setpoint, stepped
// from 0 at 20 ms. The plant runs on real time at 1 kHz. The controller is
// called with the dt its scheduler measured, and during the transient the
// scheduler misbehaves:
//  - at 50 ms a duplicated tick passes dt = 0;
//  - at 70 ms the clock steps back and passes dt = −1 ms;
//  - from 100 ms the controller misses 49 ticks (the actuator holds its last
//    command), and then runs with dt = 50 ms.
// update() treats a non-positive dt as a fault: it returns the last output
// and leaves its state alone. A long dt is valid, so the integral takes one
// 50 ms step, k_i·e·dt, and catches up at once. update() cannot tell a
// late tick from a slow loop: the scheduler has to detect the overrun.
//
// Check (BB-T1): over the two faulted ticks, the largest change of the
// output and of the integral, exactly 0 (the contract test "non-positive dt
// holds the output and state").
//
// Writes $REFLEX_TRACE_DIR/pid/timing_fault.json.

#include "pid_trace.hpp"

#include <algorithm>
#include <cmath>

namespace {

namespace trace = branes::reflex::trace;
namespace pt = branes::reflex::trace::pid;
using branes::reflex::Pid;
using branes::reflex::PidConfig;

constexpr int kSamples = 251;  // 0 … 0.25 s, every tick
constexpr long kStep = 20;
constexpr long kDuplicate = 50;
constexpr long kBackward = 70;
constexpr long kGapStart = 100;
constexpr long kGapEnd = 149;  // first tick after the gap

trace::Trace run() {
    PidConfig<double> cfg;
    cfg.kp = 1;
    cfg.ki = 10;
    Pid<double> ctl{cfg};
    pt::FirstOrderPlant plant;

    trace::Trace tr({.commit = {},
                     .block = "control/pid",
                     .scenario = "tools/trace/scenarios/pid/timing_fault.cpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = pt::kDt,
                     .dt_trace = pt::kDt});
    tr.channel("response", 1, "1");
    tr.channel("setpoint", 1, "1");
    tr.channel("command", 1, "1");
    tr.channel("integral", 1, "1");
    tr.channel("dt_passed", 1, "s");

    double u = 0;
    double held = 0;  // largest change of output or integral on a faulted tick
    for (long n = 0;; ++n) {
        const double r = n < kStep ? 0.0 : 0.5;
        double dt = pt::kDt;
        if (n == kDuplicate) {
            dt = 0;
        } else if (n == kBackward) {
            dt = -pt::kDt;
        } else if (n == kGapEnd) {
            dt = static_cast<double>(kGapEnd - kGapStart + 1) * pt::kDt;
        }
        const bool skipped = n >= kGapStart && n < kGapEnd;
        if (!skipped) {
            const double u0 = ctl.output();
            const double i0 = ctl.integral();
            u = ctl.update(r, plant.x, dt);
            if (dt <= 0) {
                held = std::max({held, std::abs(u - u0), std::abs(ctl.integral() - i0)});
            }
        }
        tr.sample(static_cast<double>(n) * pt::kDt);
        tr.set("response", plant.x);
        tr.set("setpoint", r);
        tr.set("command", u);
        tr.set("integral", ctl.integral());
        tr.set("dt_passed", skipped ? 0.0 : dt);
        if (n + 1 == kSamples) {
            break;
        }
        plant.step(u, pt::kDt);
    }

    tr.event(static_cast<double>(kStep) * pt::kDt, "setpoint 0.5");
    tr.event(static_cast<double>(kDuplicate) * pt::kDt, "duplicated tick: dt = 0");
    tr.event(static_cast<double>(kBackward) * pt::kDt, "clock steps back: dt = −1 ms");
    tr.event(static_cast<double>(kGapStart) * pt::kDt, "the controller misses 49 ticks");
    tr.event(static_cast<double>(kGapEnd) * pt::kDt, "it runs again, with dt = 50 ms");
    tr.check({.id = "BB-T1",
              .metric = "output_and_integral_change_on_faulted_ticks",
              .unit = "1",
              .value = held,
              .limit = 0,
              .comparison = "<="});
    return tr;
}

}  // namespace

int main() {
    namespace qt = branes::reflex::trace::quadrotor;
    return qt::guarded("timing_fault", [] { return qt::write(run(), "pid/timing_fault.json"); });
}
