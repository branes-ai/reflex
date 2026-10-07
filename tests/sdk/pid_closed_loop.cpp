// Closed-loop regression tests: Pid drives simple plant models of the loops it
// will run on a multirotor, integrated at a flight-controller rate. These pin
// the end-to-end behavior (convergence, zero steady-state error, disturbance
// rejection, bounded overshoot) rather than single-step arithmetic.

#include <branes/reflex/control/pid.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>

using branes::reflex::Pid;
using branes::reflex::PidConfig;
using Catch::Matchers::WithinAbs;

namespace {

constexpr double kDt = 1.0 / 1000.0;  // 1 kHz loop

// First-order plant: T * dx/dt = -x + K * u (e.g. body rate driven by a
// motor-speed command with a lag).
struct FirstOrderPlant {
    double gain;
    double time_constant;
    double x{0.0};

    void step(double u, double dt) {
        x += dt * (-x + gain * u) / time_constant;
    }
};

// Double integrator with a constant disturbance acceleration: z'' = u + d
// (e.g. altitude under a thrust command, with d an unmodeled weight offset).
struct DoubleIntegrator {
    double disturbance;
    double z{0.0};
    double v{0.0};

    void step(double u, double dt) {
        v += dt * (u + disturbance);
        z += dt * v;
    }
};

}  // namespace

TEST_CASE("pid closed loop: PI drives a first-order plant to zero steady-state error", "[pid][closed-loop]") {
    PidConfig<double> cfg;
    cfg.kp = 2.0;
    cfg.ki = 20.0;
    FirstOrderPlant plant{.gain = 0.8, .time_constant = 0.05};
    Pid<double> pid{cfg};

    const double setpoint = 0.5;
    for (int i = 0; i < 3000; ++i) {  // 3 s
        plant.step(pid.update(setpoint, plant.x, kDt), kDt);
    }
    REQUIRE_THAT(plant.x, WithinAbs(setpoint, 1e-4));
}

TEST_CASE("pid closed loop: PID holds a double integrator against a constant disturbance", "[pid][closed-loop]") {
    PidConfig<double> cfg;
    cfg.kp = 16.0;
    cfg.ki = 8.0;
    cfg.kd = 8.0;
    cfg.output_min = -20.0;
    cfg.output_max = 20.0;
    cfg.integral_min = -20.0;
    cfg.integral_max = 20.0;
    cfg.derivative_tau = 0.005;
    DoubleIntegrator plant{.disturbance = -3.0};
    Pid<double> pid{cfg};

    const double setpoint = 1.0;
    double peak = 0.0;
    for (int i = 0; i < 15000; ++i) {  // 15 s
        plant.step(pid.update(setpoint, plant.z, kDt), kDt);
        peak = std::max(peak, plant.z);
    }
    // Integral action cancels the disturbance: no steady-state offset, and the
    // integral settles at the force that cancels it.
    REQUIRE_THAT(plant.z, WithinAbs(setpoint, 1e-3));
    REQUIRE_THAT(pid.integral(), WithinAbs(3.0, 1e-2));
    // Well-damped gains: overshoot stays bounded.
    REQUIRE(peak < 1.3 * setpoint);
}

TEST_CASE("pid closed loop: anti-windup recovers quickly from an unreachable setpoint", "[pid][closed-loop]") {
    // The actuator can hold at most x = 1, so a setpoint of 1.5 keeps the loop
    // saturated for 2 s. A naive integrator winds up to ~10 in that time and
    // then takes ~2.4 s to unwind after the setpoint drops to 0.5 (the output
    // stays pinned high). With conditional integration the integral never
    // winds up, so the loop settles in well under 0.2 s.
    PidConfig<double> cfg;  // output and integral range [-1, 1]
    cfg.kp = 1.0;
    cfg.ki = 10.0;
    FirstOrderPlant plant{.gain = 1.0, .time_constant = 0.1};
    Pid<double> pid{cfg};

    for (int i = 0; i < 2000; ++i) {  // 2 s against an unreachable setpoint
        plant.step(pid.update(1.5, plant.x, kDt), kDt);
    }
    // Conditional integration parks the output at the edge of saturation
    // rather than winding the integral up behind it.
    REQUIRE(pid.output() > 0.99);
    REQUIRE(pid.integral() < 0.6);

    int settle_steps = -1;
    for (int i = 0; i < 4000; ++i) {
        plant.step(pid.update(0.5, plant.x, kDt), kDt);
        if (settle_steps < 0 && std::abs(plant.x - 0.5) < 0.02) {
            settle_steps = i;
        }
        REQUIRE(std::isfinite(plant.x));
    }
    REQUIRE(settle_steps >= 0);
    REQUIRE(settle_steps * kDt < 0.2);
    REQUIRE_THAT(plant.x, WithinAbs(0.5, 1e-4));
}
