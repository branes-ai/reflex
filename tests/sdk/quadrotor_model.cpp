// Unit tests for branes::reflex::model::QuadrotorModel and the math types under
// it. Each case checks one physical property against a hand-derivable answer:
// hover equilibrium, free fall, torque-free rotation, torque signs, the motor
// lag, battery sag, ground contact, and determinism. Run in double and float.

#include <branes/reflex/math/quat.hpp>
#include <branes/reflex/math/vec3.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <limits>
#include <type_traits>

using namespace branes::reflex;
using namespace branes::reflex::model;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

template <typename T>
QuadrotorParams<T> rp1() {
    return QuadrotorParams<T>{};  // the defaults are RP-1's (racing-drone vehicles/rp1.yaml)
}

/// Every rotor commanded to the same speed.
template <typename T>
RotorCommand<T> all(T w) {
    return RotorCommand<T>{{w, w, w, w}};
}

/// An airborne vehicle at height z with every rotor already at speed w.
template <typename T>
QuadrotorState<T> airborne(T z, T w) {
    QuadrotorState<T> x;
    x.position = vec3(T(0), T(0), z);
    x.rotor_speeds = {w, w, w, w};
    return x;
}

template <typename T>
QuadrotorState<T> run(const QuadrotorModel<T>& m, QuadrotorState<T> x, const RotorCommand<T>& u, T dt, int steps) {
    for (int i = 0; i < steps; ++i) {
        x = m.step(x, u, dt);
    }
    return x;
}

}  // namespace

TEST_CASE("math: cross product, rotation, and the exponential map", "[model]") {
    const auto ex = vec3(1.0, 0.0, 0.0);
    const auto ey = vec3(0.0, 1.0, 0.0);
    const auto ez = cross(ex, ey);
    REQUIRE(ez(0) == 0.0);
    REQUIRE(ez(1) == 0.0);
    REQUIRE(ez(2) == 1.0);

    // A quarter turn about z takes x to y.
    const auto q = from_rotation_vector(vec3(0.0, 0.0, std::acos(-1.0) / 2));
    const auto r = rotate(q, ex);
    REQUIRE_THAT(r(0), WithinAbs(0.0, 1e-15));
    REQUIRE_THAT(r(1), WithinAbs(1.0, 1e-15));
    REQUIRE(is_unit(q, 1e-12));

    // q ⊗ q* is the identity; R(q) is orthonormal (Rᵀ R v == v).
    const auto id = q * conjugate(q);
    REQUIRE_THAT(id.w, WithinAbs(1.0, 1e-15));
    const auto v = vec3(0.3, -0.7, 2.0);
    const auto back = mul_transpose(to_matrix(q), mul(to_matrix(q), v));
    REQUIRE_THAT(back(1), WithinAbs(-0.7, 1e-15));
}

TEST_CASE("params: I1 accepts RP-1 and rejects each violation", "[model]") {
    REQUIRE(quadrotor_params_valid(rp1<double>()));
    auto p = rp1<double>();
    p.mass = 0;
    REQUIRE_FALSE(quadrotor_params_valid(p));
    p = rp1<double>();
    p.inertia[1] = -1;
    REQUIRE_FALSE(quadrotor_params_valid(p));
    p = rp1<double>();
    p.battery_sag = 1.0;
    REQUIRE_FALSE(quadrotor_params_valid(p));
    p = rp1<double>();
    p.drag_quadratic[2] = -0.1;
    REQUIRE_FALSE(quadrotor_params_valid(p));

    // Every parameter must be finite: +inf passes "> 0" and NaN fails every bound silently.
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    p = rp1<double>();
    p.torque_coefficient = inf;
    REQUIRE_FALSE(quadrotor_params_valid(p));
    p = rp1<double>();
    p.ground_z = nan;
    REQUIRE_FALSE(quadrotor_params_valid(p));
    p = rp1<double>();
    p.inertia[0] = inf;
    REQUIRE_FALSE(quadrotor_params_valid(p));
}

TEMPLATE_TEST_CASE("hover: four rotors at hover speed hold position", "[model]", double, float) {
    using T = TestType;
    const QuadrotorModel<T> m(rp1<T>());
    const T w = m.hover_rotor_speed();
    const auto x = run(m, airborne(T(2), w), all(w), T(1e-3), 1000);  // 1 s
    const T tol = std::is_same_v<T, double> ? T(1e-9) : T(1e-3);
    REQUIRE_THAT(double(x.position(2)), WithinAbs(2.0, double(tol)));
    REQUIRE_THAT(double(x.velocity(2)), WithinAbs(0.0, double(tol)));
    REQUIRE_THAT(double(x.body_rates(2)), WithinAbs(0.0, double(tol)));  // yaw torques cancel
    REQUIRE(m.quadrotor_state_valid(x));
}

TEMPLATE_TEST_CASE("free fall: z(t) = z0 - g t^2 / 2 (RK4 is exact for a quadratic)", "[model]", double, float) {
    using T = TestType;
    const QuadrotorModel<T> m(rp1<T>());
    const auto x = run(m, airborne(T(10), T(0)), all(T(0)), T(1e-3), 500);  // 0.5 s, no drag
    const double g = 9.80665, t = 0.5;
    const double tol = std::is_same_v<T, double> ? 1e-9 : 1e-3;
    REQUIRE_THAT(double(x.position(2)), WithinAbs(10.0 - 0.5 * g * t * t, tol));
    REQUIRE_THAT(double(x.velocity(2)), WithinAbs(-g * t, tol));
}

TEST_CASE("torque-free rotation conserves energy and angular momentum", "[model]") {
    const QuadrotorModel<double> m(rp1<double>());
    auto x = airborne(1000.0, 0.0);  // high enough not to land
    x.body_rates = vec3(1.0, 2.0, 3.0);
    const auto& J = m.params().inertia;
    const auto energy = [&](const QuadrotorState<double>& s) {
        return 0.5 * (J[0] * s.body_rates(0) * s.body_rates(0) + J[1] * s.body_rates(1) * s.body_rates(1) +
                      J[2] * s.body_rates(2) * s.body_rates(2));
    };
    const auto momentum = [&](const QuadrotorState<double>& s) {
        const auto h = vec3(J[0] * s.body_rates(0), J[1] * s.body_rates(1), J[2] * s.body_rates(2));
        return norm(h);
    };
    const double e0 = energy(x), h0 = momentum(x);
    x = run(m, x, all(0.0), 1e-3, 2000);  // 2 s of tumbling
    REQUIRE_THAT(energy(x), WithinRel(e0, 1e-8));
    REQUIRE_THAT(momentum(x), WithinRel(h0, 1e-8));
    REQUIRE(is_unit(x.attitude, 1e-12));
}

TEST_CASE("torque signs: left rotors roll right-side down, CCW rotors yaw CW", "[model]") {
    const QuadrotorModel<double> m(rp1<double>());
    const double w = m.hover_rotor_speed();

    // More thrust on the left rotors (1 rear-left, 2 front-left): positive roll about +x.
    auto x = airborne(2.0, w);
    x.rotor_speeds = {w, 1.1 * w, 1.1 * w, w};
    REQUIRE(m.derivative(x, all(w)).body_rates(0) > 0.0);

    // More speed on the CCW rotors (0, 1): the reaction yaws the body CW (negative about +z).
    x.rotor_speeds = {1.1 * w, 1.1 * w, w, w};
    const auto d = m.derivative(x, all(w));
    REQUIRE(d.body_rates(2) < 0.0);
    REQUIRE_THAT(d.body_rates(0), WithinAbs(0.0, 1e-12));  // diagonal pair: no roll
    REQUIRE_THAT(d.body_rates(1), WithinAbs(0.0, 1e-12));  // ... and no pitch
}

TEMPLATE_TEST_CASE("motor lag: a step reaches 1 - 1/e of its target after one time constant",
                   "[model]",
                   double,
                   float) {
    using T = TestType;
    const QuadrotorModel<T> m(rp1<T>());
    const T target = m.max_rotor_speed() / T(2);
    const T tau = m.params().motor_time_constant;
    const int steps = static_cast<int>(std::lround(double(tau) / 1e-4));
    const auto x = run(m, airborne(T(1000), T(0)), all(target), T(1e-4), steps);
    REQUIRE_THAT(double(x.rotor_speeds[0] / target), WithinAbs(1.0 - std::exp(-1.0), 1e-5));
}

TEST_CASE("rotor commands are clamped to [0, omega_max]", "[model]") {
    const QuadrotorModel<double> m(rp1<double>());
    const auto x = run(m, airborne(1000.0, 0.0), all(10.0 * m.max_rotor_speed()), 1e-3, 1000);
    REQUIRE(x.rotor_speeds[0] <= m.max_rotor_speed());
    const auto y = run(m, x, all(-5.0), 1e-3, 1000);
    REQUIRE(y.rotor_speeds[0] >= 0.0);
}

TEST_CASE("battery sag scales thrust, and energy only grows", "[model]") {
    auto p = rp1<double>();
    p.battery_sag = 0.2;
    p.power_coefficient = 1e-9;
    const QuadrotorModel<double> m(p);
    REQUIRE(m.thrust_scale(0.0) == 1.0);
    REQUIRE_THAT(m.thrust_scale(p.battery_capacity), WithinAbs(0.8, 1e-15));
    REQUIRE_THAT(m.thrust_scale(2 * p.battery_capacity), WithinAbs(0.8, 1e-15));  // clamped at empty

    auto x = airborne(2.0, m.hover_rotor_speed());
    x.energy = p.battery_capacity;  // empty: hover speed no longer carries the weight
    REQUIRE(m.derivative(x, all(m.hover_rotor_speed())).velocity(2) < 0.0);
    const auto y = m.step(x, all(m.hover_rotor_speed()), 1e-3);
    REQUIRE(y.energy > x.energy);
}

TEST_CASE("ground contact: a resting vehicle stays put, full thrust lifts off", "[model]") {
    const QuadrotorModel<double> m(rp1<double>());
    QuadrotorState<double> pad;
    pad.on_ground = true;
    const auto resting = run(m, pad, all(0.0), 1e-3, 500);
    REQUIRE(resting.on_ground);
    REQUIRE(resting.position(2) == 0.0);
    REQUIRE(resting.velocity(2) == 0.0);

    const auto flying = run(m, pad, all(m.max_rotor_speed()), 1e-3, 500);
    REQUIRE_FALSE(flying.on_ground);
    REQUIRE(flying.position(2) > 0.1);

    // Falling onto the ground stops there.
    const auto landed = run(m, airborne(0.5, 0.0), all(0.0), 1e-3, 1000);
    REQUIRE(landed.on_ground);
    REQUIRE(landed.position(2) == 0.0);
    REQUIRE(m.quadrotor_state_valid(landed));
}

TEST_CASE("drag opposes motion in the body frame", "[model]") {
    auto p = rp1<double>();
    p.drag_linear = {0.3, 0.3, 0.5};
    p.drag_quadratic = {0.03, 0.03, 0.05};
    const QuadrotorModel<double> m(p);
    auto x = airborne(2.0, m.hover_rotor_speed());
    x.velocity = vec3(10.0, 0.0, 0.0);
    const auto d = m.derivative(x, all(m.hover_rotor_speed()));
    REQUIRE_THAT(d.velocity(0), WithinAbs(-(0.3 * 10.0 + 0.03 * 100.0) / p.mass, 1e-12));
}

TEST_CASE("determinism: identical inputs give bit-identical trajectories", "[model]") {
    const QuadrotorModel<double> m(rp1<double>());
    // High enough to stay airborne for the whole run: every step exercises free flight.
    auto a = airborne(1000.0, m.hover_rotor_speed());
    a.body_rates = vec3(0.1, -0.2, 0.3);
    auto b = a;
    const RotorCommand<double> u{{900.0, 1100.0, 1000.0, 950.0}};
    for (int i = 0; i < 2000; ++i) {
        a = m.step(a, u, 1e-3);
        b = m.step(b, u, 1e-3);
    }
    REQUIRE_FALSE(a.on_ground);
    for (std::size_t k = 0; k < 3; ++k) {
        REQUIRE(a.position(k) == b.position(k));
        REQUIRE(a.velocity(k) == b.velocity(k));
        REQUIRE(a.body_rates(k) == b.body_rates(k));
    }
    REQUIRE(a.attitude == b.attitude);
    REQUIRE(a.rotor_speeds == b.rotor_speeds);
    REQUIRE(a.energy == b.energy);
    REQUIRE(a.on_ground == b.on_ground);
}
