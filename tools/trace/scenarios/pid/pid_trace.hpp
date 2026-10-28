#pragma once

// Shared pieces of the Pid trace scenarios: the toy plants the controller
// drives, and a loop period. The plants are the ones the closed-loop tests
// use (tests/sdk/pid_closed_loop.cpp), so a trace and its test show the same
// loop:
//  - FirstOrderPlant: T·ẋ = −x + K·u, a body rate driven through a motor lag;
//  - DoubleIntegrator: z̈ = u + d, a height under an acceleration command, with
//    d an unmodelled offset (a vehicle heavier than its controller believes).
// Both are stepped by explicit (semi-implicit for the double integrator)
// Euler at the loop period, as in the tests.

#include <branes/reflex/control/pid.hpp>

#include "../quadrotor/quadrotor_trace.hpp"  // write(), guarded()

namespace branes::reflex::trace::pid {

inline constexpr double kDt = 0.001;  // 1 kHz loop

struct FirstOrderPlant {
    double gain = 1;
    double time_constant = 0.1;
    double x = 0;

    void step(double u, double dt) {
        x += dt * (-x + gain * u) / time_constant;
    }
};

struct DoubleIntegrator {
    double z = 0;
    double v = 0;

    void step(double u, double d, double dt) {
        v += dt * (u + d);
        z += dt * v;
    }
};

/// Output and integral limits wide enough that the anti-windup never engages:
/// what a controller without anti-windup does ahead of a saturating actuator.
inline void open_limits(PidConfig<double>& cfg) {
    cfg.output_min = cfg.integral_min = -1e3;
    cfg.output_max = cfg.integral_max = 1e3;
}

}  // namespace branes::reflex::trace::pid
