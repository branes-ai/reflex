// Closed-loop acceptance tests for the cascade (reflex#20). These are design
// verification on a nominal plant, not validation: the plant is
// QuadrotorModel, which has no independent reference yet (see the quadrotor
// V&V plan). Three things keep the evidence honest:
//
//  1. The acceptance criteria were written into reflex#20 before any of these
//     tests ran (BB-L5), and are checked as stated.
//  2. Small-signal responses on QuadrotorModel are compared with hand-derived
//     linear plants that share no code with it (rate axes: inertia behind a
//     first-order motor lag; vertical: mass behind the same lag).
//  3. Stability and bounded overshoot are required across a perturbation set
//     around the nominal vehicle, so the result does not rest on the model
//     being exactly right.
//
// The loops run at 1 kHz (rate), 250 Hz (attitude), and 50 Hz (position).

#include <branes/reflex/control/cascade.hpp>
#include <branes/reflex/control/mixer.hpp>
#include <branes/reflex/control/rate.hpp>
#include <branes/reflex/math/quat.hpp>
#include <branes/reflex/math/vec3.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include "closed_loop.hpp"

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <numbers>
#include <vector>

using namespace branes::reflex;
using namespace closed_loop;

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kDeg = kPi / 180.0;

template <typename T>
model::QuadrotorParams<T> rp1() {
    return model::QuadrotorParams<T>{};
}

/// Roll angle of an attitude that is a pure roll [rad].
template <typename T>
double roll_of(const Quat<T>& q) {
    return 2.0 * std::atan2(double(q.x), double(q.w));
}

/// Angle between two attitudes [rad].
template <typename T>
double angle_between(const Quat<T>& a, const Quat<T>& b) {
    const Quat<T> e = conjugate(a) * b;
    const double w = std::abs(double(e.w));
    return 2.0 * std::acos(w > 1.0 ? 1.0 : w);
}

/// Angle between the body z axes of two attitudes [rad].
template <typename T>
double tilt_between(const Quat<T>& a, const Quat<T>& b) {
    const Vec3<T> e3 = vec3(T(0), T(0), T(1));
    const double c = double(dot(rotate(a, e3), rotate(b, e3)));
    return std::acos(c > 1.0 ? 1.0 : (c < -1.0 ? -1.0 : c));
}

/// Run a position step on `axis` by `size` metres from hover at (0, 0, 10).
template <typename T>
StepMetrics position_step(Loop<T>& loop, std::size_t axis, double size, int ticks) {
    const Vec3<T> start = vec3(T(0), T(0), T(10));
    loop.hover_at(start);
    control::PositionSetpoint<T> sp;
    sp.position = start;
    sp.position(axis) = sp.position(axis) + T(size);
    std::vector<double> t, y;
    for (int i = 0; i < ticks; ++i) {
        loop.step(sp);
        t.push_back(double(loop.time()));
        y.push_back(double(loop.state().position(axis)));
    }
    return step_metrics(t, y, double(start(axis)), double(start(axis)) + size, 0.02);
}

/// A 30° roll step in attitude mode at hover thrust.
template <typename T>
StepMetrics roll_step(Loop<T>& loop, const model::QuadrotorParams<T>& nominal, double band, int ticks) {
    loop.hover_at(vec3(T(0), T(0), T(10)));
    const control::AttitudeTarget<T> target{from_rotation_vector(vec3(T(30 * kDeg), T(0), T(0))),
                                            nominal.mass * nominal.gravity};
    std::vector<double> t, y;
    for (int i = 0; i < ticks; ++i) {
        loop.step(target);
        t.push_back(double(loop.time()));
        y.push_back(roll_of(loop.state().attitude));
    }
    return step_metrics(t, y, 0.0, 30 * kDeg, band);
}

}  // namespace

// ── Acceptance criteria from reflex#20 ────────────────────────────────────

TEMPLATE_TEST_CASE("hover hold: error below 1 cm after 2 s, with no drift over 10 s",
                   "[cascade][closed-loop]",
                   double,
                   float) {
    using T = TestType;
    const auto p = rp1<T>();
    Loop<T> loop(p, p);
    const Vec3<T> home = vec3(T(1), T(2), T(3));
    // Start disturbed: 5 cm off on every axis and tumbling slowly. An exact
    // hover start would hold trivially and prove nothing.
    loop.hover_at(add(home, vec3(T(0.05), T(-0.05), T(0.05))));
    loop.state().body_rates = vec3(T(0.3), T(-0.3), T(0.2));
    control::PositionSetpoint<T> sp;
    sp.position = home;
    double worst = 0;
    for (int i = 0; i < 12000; ++i) {
        loop.step(sp);
        if (loop.time() > T(2)) {
            worst = std::max(worst, double(norm(sub(loop.state().position, home))));
        }
    }
    INFO("worst error after 2 s: " << worst << " m");
    REQUIRE(worst < 0.01);
}

TEMPLATE_TEST_CASE("30 deg roll step: 90 % in 0.25 s, overshoot <= 10 %, within 2 deg by 0.6 s",
                   "[cascade][closed-loop]",
                   double,
                   float) {
    using T = TestType;
    const auto p = rp1<T>();
    Loop<T> loop(p, p);
    const StepMetrics m = roll_step(loop, p, 2.0 / 30.0, 1500);
    INFO("rise90 " << m.rise90 << " s, overshoot " << m.overshoot << ", settle " << m.settle << " s");
    REQUIRE(m.rise90 > 0);
    REQUIRE(m.rise90 <= 0.25);
    REQUIRE(m.overshoot <= 0.10);
    REQUIRE(m.settle <= 0.6);
}

TEST_CASE("180 deg roll step: error below 2 deg by 1.5 s; -q flies identically", "[cascade][closed-loop]") {
    const auto p = rp1<double>();
    const control::AttitudeTarget<double> flip{from_rotation_vector(vec3(kPi, 0.0, 0.0)), p.mass * p.gravity};
    Loop<double> loop(p, p);
    Loop<double> negated(p, p);
    loop.hover_at(vec3(0.0, 0.0, 50.0));
    negated.hover_at(vec3(0.0, 0.0, 50.0));
    negated.negate_attitude(true);
    double last_outside = 0;
    for (int i = 0; i < 3000; ++i) {
        loop.step(flip);
        negated.step(flip);
        if (angle_between(loop.state().attitude, flip.attitude) > 2 * kDeg) {
            last_outside = loop.time();
        }
        // q and −q are the same attitude: the controller must not see a difference.
        REQUIRE(negated.state().attitude == loop.state().attitude);
        REQUIRE(negated.state().body_rates(0) == loop.state().body_rates(0));
    }
    INFO("last time beyond 2 deg: " << last_outside << " s");
    REQUIRE(last_outside <= 1.5);
}

TEST_CASE("tilt and yaw step: the tilt converges as fast as with no yaw step", "[cascade][closed-loop]") {
    const auto p = rp1<double>();
    const Quat<double> tilt = from_rotation_vector(vec3(30 * kDeg, 0.0, 0.0));
    const Quat<double> both = from_rotation_vector(vec3(0.0, 0.0, kPi / 2)) * tilt;
    const auto tilt_time = [&](const Quat<double>& target) {
        Loop<double> loop(p, p);
        loop.hover_at(vec3(0.0, 0.0, 20.0));
        const control::AttitudeTarget<double> t{target, p.mass * p.gravity};
        double last = 0;
        for (int i = 0; i < 2000; ++i) {
            loop.step(t);
            if (tilt_between(loop.state().attitude, target) > 2 * kDeg) {
                last = loop.time();
            }
        }
        return last;
    };
    const double alone = tilt_time(tilt);
    const double with_yaw = tilt_time(both);
    INFO("tilt within 2 deg: alone " << alone << " s, with a 90 deg yaw step " << with_yaw << " s");
    // Times are read on the 1 kHz grid while the attitude loop acts every 4 ms,
    // so "as fast" holds to one attitude-loop period.
    REQUIRE(with_yaw <= alone + 4e-3);
}

TEMPLATE_TEST_CASE("1 m step in x and in z: overshoot <= 10 %, within 2 % by 3 s",
                   "[cascade][closed-loop]",
                   double,
                   float) {
    using T = TestType;
    const auto p = rp1<T>();
    for (const std::size_t axis : {std::size_t{0}, std::size_t{2}}) {
        Loop<T> loop(p, p);
        const StepMetrics m = position_step(loop, axis, 1.0, 6000);
        INFO("axis " << axis << ": overshoot " << m.overshoot << ", settle " << m.settle << " s");
        REQUIRE(m.overshoot <= 0.10);
        REQUIRE(m.settle <= 3.0);
    }
}

TEST_CASE("waypoint square, 10 m sides, commanded peak 5 m/s: cross-track RMS <= 0.3 m, max <= 0.6 m",
          "[cascade][closed-loop]") {
    // Each leg is a triangle speed profile: 2.5 m/s² to 5 m/s at mid-leg and
    // back to rest at the corner, with velocity and acceleration feed-forward.
    const double corners[5][2] = {{0, 0}, {10, 0}, {10, 10}, {0, 10}, {0, 0}};
    const double leg = 4.0;
    const double accel = 2.5;
    const auto setpoint = [&](double t) {
        const int k = std::min(3, static_cast<int>(t / leg));
        const double s = std::min(t - k * leg, leg);
        double d = 0, v = 0, a = 0;
        if (s < leg / 2) {
            d = 0.5 * accel * s * s;
            v = accel * s;
            a = accel;
        } else if (s < leg) {
            const double r = s - leg / 2;
            d = 5.0 + 5.0 * r - 0.5 * accel * r * r;
            v = 5.0 - accel * r;
            a = -accel;
        } else {
            d = 10.0;
        }
        const double ux = (corners[k + 1][0] - corners[k][0]) / 10.0;
        const double uy = (corners[k + 1][1] - corners[k][1]) / 10.0;
        control::PositionSetpoint<double> sp;
        sp.position = vec3(corners[k][0] + ux * d, corners[k][1] + uy * d, 5.0);
        sp.velocity = vec3(ux * v, uy * v, 0.0);
        sp.acceleration = vec3(ux * a, uy * a, 0.0);
        return sp;
    };
    const auto cross_track = [&](const Vec3<double>& p) {
        double best = 1e9;
        for (int k = 0; k < 4; ++k) {
            const double ax = corners[k][0], ay = corners[k][1];
            const double vx = corners[k + 1][0] - ax, vy = corners[k + 1][1] - ay;
            const double u = std::clamp(((p(0) - ax) * vx + (p(1) - ay) * vy) / (vx * vx + vy * vy), 0.0, 1.0);
            best = std::min(best, std::hypot(ax + u * vx - p(0), ay + u * vy - p(1)));
        }
        return best;
    };

    const auto p = rp1<double>();
    Loop<double> loop(p, p);
    loop.hover_at(vec3(0.0, 0.0, 5.0));
    double sum2 = 0, worst = 0;
    int n = 0;
    double top_speed = 0;
    for (int i = 0; i < 18000; ++i) {
        loop.step(setpoint(loop.time()));
        const double c = cross_track(loop.state().position);
        sum2 += c * c;
        worst = std::max(worst, c);
        top_speed = std::max(top_speed, norm(loop.state().velocity));
        ++n;
    }
    const double rms = std::sqrt(sum2 / n);
    INFO("cross-track RMS " << rms << " m, max " << worst << " m, top speed " << top_speed << " m/s");
    REQUIRE(top_speed > 4.5);  // the commanded peak is 5 m/s; the flown top speed is within 10 %
    REQUIRE(top_speed < 5.5);
    REQUIRE(rms <= 0.3);
    REQUIRE(worst <= 0.6);
}

TEST_CASE("2 s of thrust saturation: overshoot after release within 5 points of an unsaturated step",
          "[cascade][closed-loop]") {
    const auto p = rp1<double>();
    // Unsaturated reference: the 1 m vertical step.
    Loop<double> reference(p, p);
    const StepMetrics unsaturated = position_step(reference, 2, 1.0, 6000);

    // Saturated: a 10 m climb with the collective capped at 1.25 m·g and a
    // high climb-rate limit, so the vertical force stays clipped for > 2 s.
    auto cfg = control::cascade_config_from(p);
    cfg.position.max_thrust = 1.25 * p.mass * p.gravity;
    cfg.position.max_speed_up = 10.0;
    Loop<double> loop(p, p, cfg);
    loop.hover_at(vec3(0.0, 0.0, 5.0));
    control::PositionSetpoint<double> sp;
    sp.position = vec3(0.0, 0.0, 15.0);
    std::vector<double> t, z;
    double clipped = 0;
    for (int i = 0; i < 15000; ++i) {
        loop.step(sp);
        t.push_back(loop.time());
        z.push_back(loop.state().position(2));
        if (loop.cascade().position().cut()[2] != 0) {
            clipped += loop.dt();
        }
    }
    const StepMetrics saturated = step_metrics(t, z, 5.0, 15.0, 0.02);
    INFO("clipped " << clipped << " s; overshoot saturated " << saturated.overshoot << ", unsaturated "
                    << unsaturated.overshoot);
    REQUIRE(clipped >= 2.0);
    REQUIRE(saturated.overshoot <= unsaturated.overshoot + 0.05);
}

// ── Margins over the model's uncertainty ──────────────────────────────────

TEST_CASE("perturbation set: stable with overshoot <= 25 % at every vertex", "[cascade][closed-loop][robust]") {
    // Vertices: inertia ±30 %, mass ±20 %, motor τ ×0.5 / ×2, thrust coefficient
    // (and with it the rotor thrust limit) ±10 %, command delay 0 / 4 ms, and
    // the scenario: a 1 m x step (position mode) or a 30° roll step (attitude
    // mode). The cascade and mixer always use the nominal vehicle.
    const auto p = rp1<double>();
    for (unsigned v = 0; v < 64; ++v) {
        auto actual = p;
        for (double& j : actual.inertia) {
            j *= (v & 1u) ? 1.3 : 0.7;
        }
        actual.mass *= (v & 2u) ? 1.2 : 0.8;
        actual.motor_time_constant *= (v & 4u) ? 2.0 : 0.5;
        actual.thrust_coefficient *= (v & 8u) ? 1.1 : 0.9;
        actual.max_thrust *= (v & 8u) ? 1.1 : 0.9;
        const std::size_t delay = (v & 16u) ? 4 : 0;
        Loop<double> loop(p, actual, delay);
        INFO("vertex " << v << ": J x" << ((v & 1u) ? 1.3 : 0.7) << ", m x" << ((v & 2u) ? 1.2 : 0.8) << ", tau x"
                       << ((v & 4u) ? 2.0 : 0.5) << ", k_f x" << ((v & 8u) ? 1.1 : 0.9) << ", delay " << delay
                       << " ms");
        if (v & 32u) {
            const StepMetrics m = roll_step(loop, p, 0.05, 1500);
            const double final_error = std::abs(roll_of(loop.state().attitude) - 30 * kDeg);
            INFO("roll: overshoot " << m.overshoot << ", final error " << final_error / kDeg << " deg");
            REQUIRE(m.overshoot <= 0.25);
            REQUIRE(final_error < 1 * kDeg);
        } else {
            const StepMetrics m = position_step(loop, 0, 1.0, 15000);
            const double final_error = norm(sub(loop.state().position, vec3(1.0, 0.0, 10.0)));
            INFO("x step: overshoot " << m.overshoot << ", final error " << final_error << " m");
            REQUIRE(m.overshoot <= 0.25);
            REQUIRE(final_error < 0.02);
        }
    }
}

TEST_CASE("rate loop: analytic phase margin >= 45 deg nominal, >= 30 deg at the slow corner",
          "[cascade][closed-loop][robust]") {
    // Loop transfer of one rate axis, from the gains (D on the measurement
    // still sits in the feedback loop):
    //   L(s) = g · (k_p + k_i/s + k_d·s) / (s (τ s + 1)) · e^(−s T_d)
    // with g = J_nominal / J_actual and T_d the sample and command delay.
    const auto p = rp1<double>();
    const auto c = control::cascade_config_from(p).rate.axis[0];
    const auto margin = [&](double tau, double g, double delay) {
        const auto loop_gain = [&](double w) {
            const std::complex<double> s(0.0, w);
            return g * (c.kp + c.ki / s + c.kd * s) / (s * (tau * s + 1.0)) * std::exp(-s * delay);
        };
        double lo = 1e-2, hi = 1e4;  // |L| falls through 1 once in this range
        for (int i = 0; i < 200; ++i) {
            const double mid = std::sqrt(lo * hi);
            (std::abs(loop_gain(mid)) > 1.0 ? lo : hi) = mid;
        }
        return 180.0 + std::arg(loop_gain(lo)) / kDeg;
    };
    const double tau = p.motor_time_constant;
    const double nominal = margin(tau, 1.0, 1e-3);
    const double slow = margin(2 * tau, 1 / 1.3, 5e-3);  // τ ×2, J ×1.3, 4 ms more delay
    INFO("phase margin nominal " << nominal << " deg, slow corner " << slow << " deg");
    REQUIRE(nominal >= 45.0);
    REQUIRE(slow >= 30.0);
}

// ── Hand-derived linear plants ────────────────────────────────────────────

TEST_CASE("small rate steps on QuadrotorModel match a hand-derived inertia-and-lag plant",
          "[cascade][closed-loop][reference]") {
    // Linear plant per axis: α̇ = (α_cmd − α)/τ, ω̇ = α, with α_cmd = τ_cmd / J,
    // discretized exactly for a command held over each tick. It shares no code
    // with QuadrotorModel. Step sizes keep each rotor's thrust change near 4 %
    // of hover, so the model's k_f·ω² curvature contributes about 2 % of the
    // thrust change during the motor transient; the agreement bound is 1 % of
    // the step on the rate, which the transient's short share of the run keeps
    // below.
    const auto p = rp1<double>();
    const auto cfg = control::cascade_config_from(p);
    const control::Mixer<double> mixer(control::mixer_params_from(p));
    const model::QuadrotorModel<double> plant(p);
    const double dt = 1e-3;
    const double tau = p.motor_time_constant;
    const double decay = std::exp(-dt / tau);
    const double steps[3] = {0.2, 0.2, 0.025};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        control::RateController<double> rc_model(cfg.rate);
        control::RateController<double> rc_linear(cfg.rate);
        model::QuadrotorState<double> x;
        x.position = vec3(0.0, 0.0, 50.0);
        const double w_h = plant.hover_rotor_speed();
        x.rotor_speeds = {w_h, w_h, w_h, w_h};
        double w = 0, alpha = 0;  // linear plant state
        Vec3<double> sp = zero3<double>();
        sp(axis) = steps[axis];
        double worst = 0;
        for (int i = 0; i < 400; ++i) {
            const Vec3<double> tq = rc_model.update(sp, x.body_rates, zero3<double>(), dt);
            const auto out = mixer.mix(control::Wrench<double>{p.mass * p.gravity, tq});
            x = plant.step(x, mixer.to_rotor_command(out.thrusts), dt);

            Vec3<double> rates = zero3<double>();
            rates(axis) = w;
            const double cmd = rc_linear.update(sp, rates, zero3<double>(), dt)(axis) / p.inertia[axis];
            w = w + cmd * dt + (alpha - cmd) * tau * (1.0 - decay);
            alpha = cmd + (alpha - cmd) * decay;

            worst = std::max(worst, std::abs(x.body_rates(axis) - w));
        }
        INFO("axis " << axis << ": worst |omega_model - omega_linear| = " << worst << " rad/s");
        REQUIRE(worst <= 0.01 * steps[axis]);
    }
}

TEST_CASE("a small vertical step on QuadrotorModel matches a hand-derived mass-and-lag plant",
          "[cascade][closed-loop][reference]") {
    // Linear vertical plant: Ṫ = (T_cmd − T)/τ, z̈ = (T − m g)/m, discretized
    // exactly for a held command. The position controller is the one under
    // test in both loops; the model loop runs the full cascade and mixer.
    const auto p = rp1<double>();
    const auto cfg = control::cascade_config_from(p);
    const double dt = 1e-3;
    const double tau = p.motor_time_constant;
    const double decay = std::exp(-dt / tau);
    const double step = 0.05;

    Loop<double> loop(p, p);
    loop.hover_at(vec3(0.0, 0.0, 10.0));
    control::PositionSetpoint<double> sp;
    sp.position = vec3(0.0, 0.0, 10.0 + step);

    control::PositionController<double> pc(cfg.position);
    double z = 10.0, vz = 0.0, thrust = p.mass * p.gravity, cmd = thrust;
    double worst = 0;
    for (int i = 0; i < 4000; ++i) {
        loop.step(sp);
        if (i % static_cast<int>(cfg.position_divider) == 0) {
            cmd =
                pc.update(vec3(0.0, 0.0, z), vec3(0.0, 0.0, vz), Quat<double>{}, sp, dt * cfg.position_divider).thrust;
        }
        // Exact solution of the lag and the double integrator over one tick.
        const double a0 = (thrust - p.mass * p.gravity) / p.mass;
        const double ac = (cmd - p.mass * p.gravity) / p.mass;
        const double d1 = tau * (1.0 - decay);
        z = z + vz * dt + ac * dt * dt / 2 + (a0 - ac) * tau * (dt - d1);
        vz = vz + ac * dt + (a0 - ac) * d1;
        thrust = cmd + (thrust - cmd) * decay;
        worst = std::max(worst, std::abs(loop.state().position(2) - z));
    }
    INFO("worst |z_model - z_linear| = " << worst << " m");
    REQUIRE(worst <= 0.01 * step);
}
