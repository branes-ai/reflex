// Unit tests for branes::reflex::Pid — one behavior per case, each driven by
// hand-computable inputs so the expected value follows directly from the
// control law in pid.hpp.

#include <branes/reflex/control/pid.hpp>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using branes::reflex::Pid;
using branes::reflex::pid_config_valid;
using branes::reflex::PidConfig;
using Catch::Matchers::WithinAbs;

namespace {

PidConfig<double> wide_limits() {
    PidConfig<double> cfg;
    cfg.output_min = -1000.0;
    cfg.output_max = 1000.0;
    cfg.integral_min = -1000.0;
    cfg.integral_max = 1000.0;
    return cfg;
}

}  // namespace

TEST_CASE("pid: proportional term is kp times the error", "[pid]") {
    auto cfg = wide_limits();
    cfg.kp = 2.5;
    Pid<double> pid{cfg};
    REQUIRE_THAT(pid.update(10.0, 4.0, 0.01), WithinAbs(15.0, 1e-12));
    REQUIRE_THAT(pid.update(-1.0, 1.0, 0.01), WithinAbs(-5.0, 1e-12));
}

TEST_CASE("pid: integral accumulates ki * e * dt", "[pid]") {
    auto cfg = wide_limits();
    cfg.ki = 4.0;
    Pid<double> pid{cfg};
    for (int i = 0; i < 100; ++i) {
        pid.update(1.0, 0.0, 0.01);  // e = 1 for 1 s in total
    }
    REQUIRE_THAT(pid.integral(), WithinAbs(4.0, 1e-9));
    REQUIRE_THAT(pid.output(), WithinAbs(4.0, 1e-9));
}

TEST_CASE("pid: output saturates at the configured limits", "[pid]") {
    PidConfig<double> cfg;  // output range [-1, 1]
    cfg.kp = 100.0;
    Pid<double> pid{cfg};
    REQUIRE(pid.update(1.0, 0.0, 0.01) == 1.0);
    REQUIRE(pid.update(-1.0, 0.0, 0.01) == -1.0);
}

TEST_CASE("pid: integral is clamped to its bounds", "[pid]") {
    auto cfg = wide_limits();
    cfg.ki = 1.0;
    cfg.integral_min = -0.5;
    cfg.integral_max = 0.5;
    Pid<double> pid{cfg};
    for (int i = 0; i < 1000; ++i) {
        pid.update(1.0, 0.0, 0.01);
    }
    REQUIRE(pid.integral() == 0.5);
}

TEST_CASE("pid: anti-windup freezes the integral while saturated", "[pid]") {
    PidConfig<double> cfg;  // output and integral range [-1, 1]
    cfg.kp = 2.0;
    cfg.ki = 1.0;
    Pid<double> pid{cfg};

    // A large error saturates the output through the P term alone; the
    // integral must not wind up behind it.
    for (int i = 0; i < 500; ++i) {
        REQUIRE(pid.update(1.0, 0.0, 0.01) == 1.0);
    }
    REQUIRE(pid.integral() == 0.0);

    // When the error reverses, the output responds immediately instead of
    // waiting for a wound-up integral to unwind.
    REQUIRE(pid.update(0.0, 0.25, 0.01) < 0.0);
}

TEST_CASE("pid: integration resumes when the error pulls out of saturation", "[pid]") {
    PidConfig<double> cfg;  // output and integral range [-1, 1]
    cfg.kd = 1.0;
    cfg.ki = 1.0;
    Pid<double> pid{cfg};
    pid.update(0.0, 0.0, 0.01);

    // The measurement jumps up at 50 units/s: the D term drives the output to
    // saturation low while the error is positive. Integrating that error pulls
    // the output back toward the range, so the integral must not be frozen.
    REQUIRE(pid.update(1.0, 0.5, 0.01) == -1.0);
    REQUIRE_THAT(pid.integral(), WithinAbs(0.005, 1e-12));  // ki * e * dt = 1 * 0.5 * 0.01
}

TEST_CASE("pid: derivative acts on measurement, so setpoint steps do not kick", "[pid]") {
    auto cfg = wide_limits();
    cfg.kd = 1.0;
    Pid<double> pid{cfg};
    pid.update(0.0, 0.0, 0.01);
    // Setpoint jumps, measurement constant: no D contribution.
    REQUIRE(pid.update(100.0, 0.0, 0.01) == 0.0);
    // Measurement rises at 2 units/s: D term opposes it with -kd * 2.
    REQUIRE_THAT(pid.update(100.0, 0.02, 0.01), WithinAbs(-2.0, 1e-9));
}

TEST_CASE("pid: first update has no derivative history", "[pid]") {
    auto cfg = wide_limits();
    cfg.kd = 1.0;
    Pid<double> pid{cfg};
    REQUIRE(pid.update(0.0, 50.0, 0.01) == 0.0);
    REQUIRE(pid.measurement_rate() == 0.0);
}

TEST_CASE("pid: derivative filter attenuates a rate step", "[pid]") {
    auto cfg = wide_limits();
    cfg.kd = 1.0;
    cfg.derivative_tau = 0.09;  // alpha = dt / (tau + dt) = 0.1
    Pid<double> pid{cfg};
    pid.update(0.0, 0.0, 0.01);
    pid.update(0.0, 0.01, 0.01);  // raw rate 1
    REQUIRE_THAT(pid.measurement_rate(), WithinAbs(0.1, 1e-12));
    for (int i = 2; i < 500; ++i) {
        pid.update(0.0, 0.01 * i, 0.01);
    }
    REQUIRE_THAT(pid.measurement_rate(), WithinAbs(1.0, 1e-9));
}

TEST_CASE("pid: non-positive dt holds the output and state", "[pid]") {
    auto cfg = wide_limits();
    cfg.kp = 1.0;
    cfg.ki = 1.0;
    Pid<double> pid{cfg};
    const double u = pid.update(1.0, 0.0, 0.1);
    const double integral = pid.integral();
    REQUIRE(pid.update(50.0, 0.0, 0.0) == u);
    REQUIRE(pid.update(50.0, 0.0, -0.1) == u);
    REQUIRE(pid.integral() == integral);
}

TEST_CASE("pid: reset returns to the constructed state", "[pid]") {
    auto cfg = wide_limits();
    cfg.kp = 1.0;
    cfg.ki = 1.0;
    cfg.kd = 1.0;
    Pid<double> pid{cfg};
    for (int i = 0; i < 10; ++i) {
        pid.update(1.0, 0.1 * i, 0.01);
    }
    pid.reset();
    REQUIRE(pid.integral() == 0.0);
    REQUIRE(pid.output() == 0.0);
    REQUIRE(pid.measurement_rate() == 0.0);
    // Unprimed again: no derivative from the jump since the last sample.
    REQUIRE(pid.update(0.0, 0.0, 0.01) == 0.0);
}

TEST_CASE("pid: reset state is clamped into a range that excludes zero", "[pid]") {
    PidConfig<double> cfg;
    cfg.output_min = 0.2;  // e.g. idle-thrust floor
    cfg.output_max = 1.0;
    cfg.integral_min = 0.1;
    cfg.integral_max = 0.5;
    Pid<double> pid{cfg};
    REQUIRE(pid.output() == 0.2);
    REQUIRE(pid.integral() == 0.1);
}

TEST_CASE("pid: config validation", "[pid]") {
    PidConfig<double> cfg;
    REQUIRE(pid_config_valid(cfg));
    cfg.kp = -1.0;
    REQUIRE_FALSE(pid_config_valid(cfg));
    cfg = {};
    cfg.output_min = 2.0;
    REQUIRE_FALSE(pid_config_valid(cfg));
    cfg = {};
    cfg.integral_max = -2.0;
    REQUIRE_FALSE(pid_config_valid(cfg));
    cfg = {};
    cfg.derivative_tau = -0.01;
    REQUIRE_FALSE(pid_config_valid(cfg));
}

TEMPLATE_TEST_CASE("pid: same control law across scalar types", "[pid][template]", float, double) {
    PidConfig<TestType> cfg;
    cfg.kp = TestType(2);
    cfg.ki = TestType(1);
    Pid<TestType> pid{cfg};
    const TestType dt = TestType(0.5);
    // P = 2 * 0.25 = 0.5, I = 1 * 0.25 * 0.5 = 0.125
    REQUIRE_THAT(static_cast<double>(pid.update(TestType(1), TestType(0.75), dt)), WithinAbs(0.625, 1e-6));
}

TEST_CASE("pid: usable in constant expressions", "[pid]") {
    constexpr double u = [] {
        PidConfig<double> cfg;
        cfg.kp = 0.5;
        Pid<double> pid{cfg};
        return pid.update(1.0, 0.0, 0.01);
    }();
    STATIC_REQUIRE(u == 0.5);
}
