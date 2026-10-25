// Unit and contract tests for the cascade's blocks: Pid's external
// saturation, the mixer's saturation report, from_matrix, and the rate,
// attitude, and position controllers and the Cascade itself. Each case checks
// an invariant, a contract, or a hand-derivable answer; nothing here depends
// on the plant model. Closed-loop behaviour is in cascade_closed_loop.cpp.

#include <branes/reflex/control/attitude.hpp>
#include <branes/reflex/control/cascade.hpp>
#include <branes/reflex/control/mixer.hpp>
#include <branes/reflex/control/pid.hpp>
#include <branes/reflex/control/position.hpp>
#include <branes/reflex/control/rate.hpp>
#include <branes/reflex/math/quat.hpp>
#include <branes/reflex/math/vec3.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <random>

using namespace branes::reflex;
using namespace branes::reflex::control;
using Catch::Matchers::WithinAbs;

namespace {

constexpr double kPi = std::numbers::pi;

template <typename T>
model::QuadrotorParams<T> rp1() {
    return model::QuadrotorParams<T>{};
}

template <typename T>
Quat<T> random_attitude(std::mt19937& rng) {
    std::normal_distribution<double> n(0.0, 1.0);
    return normalized(Quat<T>{T(n(rng)), T(n(rng)), T(n(rng)), T(n(rng))});
}

template <typename T>
Quat<T> negated(const Quat<T>& q) {
    return Quat<T>{-q.w, -q.x, -q.y, -q.z};
}

/// Angle between the body z axes of two attitudes [rad].
template <typename T>
double tilt_between(const Quat<T>& a, const Quat<T>& b) {
    const Vec3<T> e3 = vec3(T(0), T(0), T(1));
    const double c = double(dot(rotate(a, e3), rotate(b, e3)));
    return std::acos(c > 1.0 ? 1.0 : (c < -1.0 ? -1.0 : c));
}

}  // namespace

// ── Pid external saturation ───────────────────────────────────────────────

TEST_CASE("pid: external saturation freezes the integral only while the error pushes into it", "[cascade][pid]") {
    PidConfig<double> cfg;
    cfg.ki = 10.0;
    cfg.output_min = -100.0;
    cfg.output_max = 100.0;
    cfg.integral_min = -100.0;
    cfg.integral_max = 100.0;

    Pid<double> plain(cfg), none(cfg), high(cfg), low(cfg);
    for (int i = 0; i < 10; ++i) {
        (void)plain.update(1.0, 0.0, 0.01);
        (void)none.update(1.0, 0.0, 0.01, 0);
        (void)high.update(1.0, 0.0, 0.01, 1);  // error > 0 pushes into a high cut: frozen
        (void)low.update(1.0, 0.0, 0.01, -1);  // error > 0 pulls out of a low cut: integrates
    }
    REQUIRE(none.integral() == plain.integral());
    REQUIRE(high.integral() == 0.0);
    REQUIRE_THAT(low.integral(), WithinAbs(1.0, 1e-12));  // 10 · 1 · 0.1 s

    Pid<double> high_out(cfg), low_in(cfg);
    for (int i = 0; i < 10; ++i) {
        (void)high_out.update(-1.0, 0.0, 0.01, 1);  // error < 0 pulls out of a high cut: integrates
        (void)low_in.update(-1.0, 0.0, 0.01, -1);   // error < 0 pushes into a low cut: frozen
    }
    REQUIRE_THAT(high_out.integral(), WithinAbs(-1.0, 1e-12));
    REQUIRE(low_in.integral() == 0.0);
}

// ── Mixer saturation report ───────────────────────────────────────────────

TEST_CASE("mixer: saturation_from reports each cut channel in the demand's direction", "[cascade][mixer]") {
    const auto p = rp1<double>();
    const Mixer<double> mixer(mixer_params_from(p));
    const double hover = p.mass * p.gravity;

    const Wrench<double> fits{hover, vec3(0.01, -0.01, 0.001)};
    const Saturation none = saturation_from(fits, mixer.mix(fits));
    REQUIRE(none.torque == std::array<int, 3>{0, 0, 0});
    REQUIRE(none.thrust == 0);

    // Roll and pitch beyond the envelope: σ < 1, cut in the demand's sign.
    const Wrench<double> rp{hover, vec3(-50.0, 30.0, 0.0)};
    const auto out_rp = mixer.mix(rp);
    REQUIRE(out_rp.rp_scale < 1.0);
    const Saturation s_rp = saturation_from(rp, out_rp);
    REQUIRE(s_rp.torque[0] == -1);
    REQUIRE(s_rp.torque[1] == 1);

    // Yaw yields first.
    const Wrench<double> yaw{hover, vec3(0.0, 0.0, -5.0)};
    const auto out_yaw = mixer.mix(yaw);
    REQUIRE(out_yaw.yaw_scale < 1.0);
    REQUIRE(saturation_from(yaw, out_yaw).torque[2] == -1);

    // Full throttle with roll: the collective drops below the demand (cut high).
    const Wrench<double> punch{4.0 * p.max_thrust, vec3(1.0, 0.0, 0.0)};
    const auto out_punch = mixer.mix(punch);
    REQUIRE(out_punch.thrust_shifted);
    REQUIRE(saturation_from(punch, out_punch).thrust == 1);

    // Zero throttle with roll: airmode raises the collective (cut low).
    const Wrench<double> idle{0.0, vec3(1.0, 0.0, 0.0)};
    const auto out_idle = mixer.mix(idle);
    REQUIRE(out_idle.thrust_shifted);
    REQUIRE(saturation_from(idle, out_idle).thrust == -1);
}

// ── from_matrix ───────────────────────────────────────────────────────────

TEMPLATE_TEST_CASE("quat: from_matrix inverts to_matrix in every Shepperd branch", "[cascade][quat]", double, float) {
    using T = TestType;
    const T tol = T(32) * std::numeric_limits<T>::epsilon();
    const auto check = [&](const Quat<T>& q) {
        const Quat<T> r = from_matrix(to_matrix(q));
        const T sign = (r.w * q.w + r.x * q.x + r.y * q.y + r.z * q.z) < T(0) ? T(-1) : T(1);  // q and −q
        REQUIRE(is_unit(r, tol));
        REQUIRE_THAT(double(sign * r.w), WithinAbs(double(q.w), double(tol)));
        REQUIRE_THAT(double(sign * r.x), WithinAbs(double(q.x), double(tol)));
        REQUIRE_THAT(double(sign * r.y), WithinAbs(double(q.y), double(tol)));
        REQUIRE_THAT(double(sign * r.z), WithinAbs(double(q.z), double(tol)));
    };
    // Trace > 0, and 180° about x, y, z: each of the four branches.
    check(Quat<T>{});
    check(from_rotation_vector(vec3(T(kPi), T(0), T(0))));
    check(from_rotation_vector(vec3(T(0), T(kPi), T(0))));
    check(from_rotation_vector(vec3(T(0), T(0), T(kPi))));
    std::mt19937 rng(20261025);
    for (int i = 0; i < 2000; ++i) {
        check(random_attitude<T>(rng));
    }
}

// ── RateController ────────────────────────────────────────────────────────

TEST_CASE("rate: validator rejects each violation", "[cascade][rate]") {
    const auto good = cascade_config_from(rp1<double>()).rate;
    REQUIRE(rate_config_valid(good));
    auto c = good;
    c.inertia[1] = 0.0;
    REQUIRE_FALSE(rate_config_valid(c));
    c = good;
    c.inertia[2] = std::numeric_limits<double>::infinity();
    REQUIRE_FALSE(rate_config_valid(c));
    c = good;
    c.axis[0].kp = -1.0;
    REQUIRE_FALSE(rate_config_valid(c));
}

TEST_CASE("rate: torque is J alpha plus omega x J omega", "[cascade][rate]") {
    RateConfig<double> cfg;
    cfg.inertia = {0.003, 0.004, 0.005};
    for (auto& a : cfg.axis) {
        a.kp = 10.0;
        a.output_min = -1000.0;
        a.output_max = 1000.0;
    }
    RateController<double> rc(cfg);
    const Vec3<double> w = vec3(1.0, -2.0, 0.5);
    const Vec3<double> sp = vec3(2.0, 0.0, 0.0);
    const Vec3<double> ff = vec3(0.0, 3.0, 0.0);
    const Vec3<double> tau = rc.update(sp, w, ff, 1e-3);

    const Vec3<double> alpha = vec3(10.0 * (2.0 - 1.0), 10.0 * (0.0 + 2.0) + 3.0, 10.0 * (0.0 - 0.5));
    const Vec3<double> jw = vec3(0.003 * 1.0, 0.004 * -2.0, 0.005 * 0.5);
    const Vec3<double> expected = add(vec3(0.003 * alpha(0), 0.004 * alpha(1), 0.005 * alpha(2)), cross(w, jw));
    for (std::size_t i = 0; i < 3; ++i) {
        REQUIRE_THAT(tau(i), WithinAbs(expected(i), 1e-15));
    }
    REQUIRE(rc.torque()(0) == tau(0));
}

TEST_CASE("rate: angular acceleration stays within limits, feed-forward included", "[cascade][rate]") {
    const auto cfg = cascade_config_from(rp1<double>()).rate;
    RateController<double> rc(cfg);
    const Vec3<double> tau = rc.update(vec3(1e3, -1e3, 1e3), zero3<double>(), vec3(1e5, 0.0, 0.0), 1e-3);
    for (std::size_t i = 0; i < 3; ++i) {
        const double alpha = tau(i) / cfg.inertia[i];  // ω = 0: no gyroscopic term
        REQUIRE(alpha <= cfg.axis[i].output_max * (1 + 1e-12));
        REQUIRE(alpha >= cfg.axis[i].output_min * (1 + 1e-12));
    }
    REQUIRE_THAT(tau(0), WithinAbs(cfg.inertia[0] * cfg.axis[0].output_max, 1e-12));
    REQUIRE_THAT(tau(1), WithinAbs(cfg.inertia[1] * cfg.axis[1].output_min, 1e-12));
}

TEST_CASE("rate: dt <= 0 holds the last torque; saturation freezes one axis; reset clears", "[cascade][rate]") {
    const auto cfg = cascade_config_from(rp1<double>()).rate;
    RateController<double> rc(cfg);
    const Vec3<double> t1 = rc.update(vec3(0.5, 0.5, 0.5), zero3<double>(), zero3<double>(), 1e-3);
    const double i0 = rc.axis(0).integral();
    const Vec3<double> held = rc.update(vec3(9.0, 9.0, 9.0), zero3<double>(), zero3<double>(), 0.0);
    REQUIRE(held(0) == t1(0));
    REQUIRE(rc.axis(0).integral() == i0);

    for (int i = 0; i < 10; ++i) {
        (void)rc.update(vec3(0.5, 0.5, 0.5), zero3<double>(), zero3<double>(), 1e-3, {1, 0, 0});
    }
    REQUIRE(rc.axis(0).integral() == i0);  // roll cut high, positive error: frozen
    REQUIRE(rc.axis(1).integral() > i0);   // pitch not cut: integrates
    rc.reset();
    REQUIRE(rc.axis(1).integral() == 0.0);
    REQUIRE(rc.torque()(1) == 0.0);
}

// ── AttitudeController ────────────────────────────────────────────────────

TEST_CASE("attitude: validator rejects each violation", "[cascade][attitude]") {
    AttitudeConfig<double> c;
    REQUIRE(attitude_config_valid(c));
    c.gain[2] = -1.0;
    REQUIRE_FALSE(attitude_config_valid(c));
    c = AttitudeConfig<double>{};
    c.max_rate[0] = 0.0;
    REQUIRE_FALSE(attitude_config_valid(c));
    c = AttitudeConfig<double>{};
    c.gain[1] = std::numeric_limits<double>::quiet_NaN();
    REQUIRE_FALSE(attitude_config_valid(c));
}

TEMPLATE_TEST_CASE("attitude: zero at the setpoint, sign-invariant, bounded", "[cascade][attitude]", double, float) {
    using T = TestType;
    const AttitudeController<T> ac(AttitudeConfig<T>{});
    std::mt19937 rng(20261025);
    const T tol = T(64) * std::numeric_limits<T>::epsilon();
    for (int i = 0; i < 2000; ++i) {
        const Quat<T> q = random_attitude<T>(rng);
        const Quat<T> sp = random_attitude<T>(rng);
        const Vec3<T> zero = ac.update(q, q);
        const Vec3<T> a = ac.update(q, sp);
        const Vec3<T> b = ac.update(negated(q), sp);
        const Vec3<T> c = ac.update(q, negated(sp));
        for (std::size_t k = 0; k < 3; ++k) {
            REQUIRE_THAT(double(zero(k)), WithinAbs(0.0, double(tol)));
            REQUIRE_THAT(double(b(k)), WithinAbs(double(a(k)), double(tol)));
            REQUIRE_THAT(double(c(k)), WithinAbs(double(a(k)), double(tol)));
            REQUIRE(std::abs(a(k)) <= ac.config().max_rate[k]);
        }
    }
}

TEST_CASE("attitude: the roll-pitch error depends on the tilt only, the yaw error on the heading only",
          "[cascade][attitude]") {
    const AttitudeController<double> ac(AttitudeConfig<double>{});
    std::mt19937 rng(20261025);
    for (int i = 0; i < 2000; ++i) {
        const Quat<double> q = random_attitude<double>(rng);
        const Quat<double> sp = random_attitude<double>(rng);
        const Vec3<double> e = ac.error(q, sp);
        // |e_xy| = 2 sin(tilt / 2), whatever the yaw part of the error.
        const double exy = std::sqrt(e(0) * e(0) + e(1) * e(1));
        REQUIRE_THAT(exy, WithinAbs(2.0 * std::sin(tilt_between(q, sp) / 2.0), 1e-9));
        // A pure yaw of the setpoint about its own z axis leaves e_xy unchanged.
        const Quat<double> spun = sp * from_rotation_vector(vec3(0.0, 0.0, 1.3));
        const Vec3<double> e2 = ac.error(q, spun);
        REQUIRE_THAT(std::sqrt(e2(0) * e2(0) + e2(1) * e2(1)), WithinAbs(exy, 1e-9));
    }
    // Pure roll: all tilt, no yaw; pure yaw: all yaw, 2 sin(ψ/2).
    const Vec3<double> roll = ac.error(Quat<double>{}, from_rotation_vector(vec3(0.4, 0.0, 0.0)));
    REQUIRE_THAT(roll(0), WithinAbs(2.0 * std::sin(0.2), 1e-12));
    REQUIRE_THAT(roll(2), WithinAbs(0.0, 1e-12));
    const Vec3<double> yaw = ac.error(Quat<double>{}, from_rotation_vector(vec3(0.0, 0.0, -2.0)));
    REQUIRE_THAT(yaw(2), WithinAbs(2.0 * std::sin(-1.0), 1e-12));
    REQUIRE_THAT(yaw(0), WithinAbs(0.0, 1e-12));
    // A 180° tilt drives at full error (no degenerate zero), all as tilt.
    const Vec3<double> flip = ac.error(Quat<double>{}, from_rotation_vector(vec3(kPi, 0.0, 0.0)));
    REQUIRE_THAT(flip(0), WithinAbs(2.0, 1e-12));
    REQUIRE_THAT(flip(2), WithinAbs(0.0, 1e-12));
}

// ── PositionController ────────────────────────────────────────────────────

TEST_CASE("position: validator rejects each violation", "[cascade][position]") {
    const auto good = cascade_config_from(rp1<double>()).position;
    REQUIRE(position_config_valid(good));
    auto c = good;
    c.max_tilt = 1.6;
    REQUIRE_FALSE(position_config_valid(c));
    c = good;
    c.min_thrust = 0.0;  // T_min > 0 keeps the thrust axis defined
    REQUIRE_FALSE(position_config_valid(c));
    c = good;
    c.max_thrust = 0.9 * c.mass * c.gravity;  // cannot hover
    REQUIRE_FALSE(position_config_valid(c));
    c = good;
    c.max_speed_xy = 0.0;
    REQUIRE_FALSE(position_config_valid(c));
    c = good;
    c.position_gain[2] = -1.0;
    REQUIRE_FALSE(position_config_valid(c));
}

TEST_CASE("position: at the setpoint it commands level hover thrust and the heading", "[cascade][position]") {
    const auto cfg = cascade_config_from(rp1<double>()).position;
    PositionController<double> pc(cfg);
    PositionSetpoint<double> sp;
    sp.position = vec3(1.0, 2.0, 3.0);
    sp.yaw = 0.7;
    const AttitudeTarget<double> t = pc.update(sp.position, zero3<double>(), Quat<double>{}, sp, 0.02);
    REQUIRE_THAT(t.thrust, WithinAbs(cfg.mass * cfg.gravity, 1e-12));
    REQUIRE_THAT(tilt_between(t.attitude, Quat<double>{}), WithinAbs(0.0, 1e-7));
    const Vec3<double> bx = rotate(t.attitude, vec3(1.0, 0.0, 0.0));
    REQUIRE_THAT(bx(0), WithinAbs(std::cos(0.7), 1e-12));
    REQUIRE_THAT(bx(1), WithinAbs(std::sin(0.7), 1e-12));
}

TEST_CASE("position: tilt, thrust, and velocity limits hold for any demand", "[cascade][position]") {
    const auto cfg = cascade_config_from(rp1<double>()).position;
    PositionController<double> pc(cfg);
    std::mt19937 rng(20261025);
    std::uniform_real_distribution<double> big(-200.0, 200.0);
    std::uniform_real_distribution<double> yaw(-4.0, 4.0);
    for (int i = 0; i < 4000; ++i) {
        PositionSetpoint<double> sp;
        sp.position = vec3(big(rng), big(rng), big(rng));
        sp.velocity = vec3(big(rng), big(rng), big(rng));
        sp.acceleration = vec3(big(rng), big(rng), big(rng));
        sp.yaw = yaw(rng);
        const Quat<double> q = random_attitude<double>(rng);
        const AttitudeTarget<double> t =
            pc.update(vec3(big(rng), big(rng), big(rng)), vec3(big(rng), big(rng), big(rng)), q, sp, 0.02);
        REQUIRE(tilt_between(t.attitude, Quat<double>{}) <= cfg.max_tilt + 1e-9);
        REQUIRE(t.thrust >= cfg.min_thrust);
        REQUIRE(t.thrust <= cfg.max_thrust);
        const Vec3<double>& v = pc.velocity_setpoint();
        REQUIRE(std::sqrt(v(0) * v(0) + v(1) * v(1)) <= cfg.max_speed_xy * (1 + 1e-12));
        REQUIRE(v(2) <= cfg.max_speed_up);
        REQUIRE(v(2) >= -cfg.max_speed_down);
        for (const int c : pc.cut()) {
            REQUIRE((c >= -1 && c <= 1));
        }
    }
}

TEST_CASE("position: cuts are reported in the demand's direction; dt <= 0 holds", "[cascade][position]") {
    const auto cfg = cascade_config_from(rp1<double>()).position;
    PositionController<double> pc(cfg);
    PositionSetpoint<double> sp;
    sp.acceleration = vec3(-100.0, 0.0, 100.0);  // beyond both the tilt and the thrust limit
    const AttitudeTarget<double> t = pc.update(zero3<double>(), zero3<double>(), Quat<double>{}, sp, 0.02);
    REQUIRE(pc.cut()[0] == -1);
    REQUIRE(pc.cut()[1] == 0);
    REQUIRE(pc.cut()[2] == 1);
    REQUIRE(t.thrust == cfg.max_thrust);

    const AttitudeTarget<double> held = pc.update(vec3(5.0, 5.0, 5.0), zero3<double>(), Quat<double>{}, sp, -1.0);
    REQUIRE(held.thrust == t.thrust);
    REQUIRE(held.attitude == t.attitude);

    sp.acceleration = vec3(0.0, 0.0, -100.0);  // free fall demanded: f_z clamps at T_min / m
    const AttitudeTarget<double> low = pc.update(zero3<double>(), zero3<double>(), Quat<double>{}, sp, 0.02);
    REQUIRE(pc.cut()[2] == -1);
    REQUIRE(low.thrust == cfg.min_thrust);
}

// ── Cascade ───────────────────────────────────────────────────────────────

TEST_CASE("cascade: defaults are valid and traceable to the vehicle; validator rejects bad dividers", "[cascade]") {
    const auto p = rp1<double>();
    const auto c = cascade_config_from(p);
    REQUIRE(cascade_config_valid(c));
    const double wn = 1.0 / p.motor_time_constant;
    REQUIRE_THAT(c.rate.axis[0].kp, WithinAbs(wn, 1e-12));
    REQUIRE_THAT(c.rate.axis[0].kd, WithinAbs(0.4, 1e-12));
    REQUIRE_THAT(c.attitude.gain[0], WithinAbs(wn / 5.0, 1e-12));
    REQUIRE_THAT(c.position.velocity[0].kp, WithinAbs(wn / 14.0, 1e-12));
    REQUIRE_THAT(c.position.position_gain[0], WithinAbs(wn / 32.0, 1e-12));
    REQUIRE(c.rate.inertia == p.inertia);

    auto bad = c;
    bad.attitude_divider = 0;
    REQUIRE_FALSE(cascade_config_valid(bad));
    bad = c;
    bad.position_divider = 10;  // not a multiple of 4
    REQUIRE_FALSE(cascade_config_valid(bad));
    bad = c;
    bad.position_divider = 2;  // faster than the attitude loop
    REQUIRE_FALSE(cascade_config_valid(bad));
}

TEST_CASE("cascade: outer loops run on their dividers and hold in between", "[cascade]") {
    const auto p = rp1<double>();
    Cascade<double> cascade(cascade_config_from(p));
    VehicleState<double> x;
    PositionSetpoint<double> sp;
    sp.position = vec3(0.3, 0.0, 0.0);
    std::size_t target_changes = 0;
    std::size_t rate_changes = 0;
    AttitudeTarget<double> last_target = cascade.target();
    Vec3<double> last_rate = cascade.rate_setpoint();
    for (int i = 0; i < 400; ++i) {
        x.velocity(0) = 0.001 * i;  // a moving state, so every outer update changes its output
        x.attitude = from_rotation_vector(vec3(0.0, 0.0001 * i, 0.0));
        const Wrench<double> w = cascade.update(x, sp, 1e-3);
        REQUIRE(w.thrust == cascade.target().thrust);  // I4
        if (!(cascade.target().attitude == last_target.attitude)) {
            ++target_changes;
            REQUIRE(i % 20 == 0);
        }
        if (!(cascade.rate_setpoint()(1) == last_rate(1))) {
            ++rate_changes;
            REQUIRE(i % 4 == 0);
        }
        last_target = cascade.target();
        last_rate = cascade.rate_setpoint();
    }
    REQUIRE(target_changes == 20);
    REQUIRE(rate_changes == 100);
}

TEST_CASE("cascade: attitude mode bypasses the position loop; feedback stores the mixer's report", "[cascade]") {
    const auto p = rp1<double>();
    const Mixer<double> mixer(mixer_params_from(p));
    Cascade<double> cascade(cascade_config_from(p));
    const AttitudeTarget<double> target{from_rotation_vector(vec3(1.0, 0.0, 0.0)), 4.0 * p.max_thrust};
    const Wrench<double> w = cascade.update(VehicleState<double>{}, target, 1e-3);
    REQUIRE(w.thrust == target.thrust);
    REQUIRE(cascade.target().attitude == target.attitude);
    REQUIRE(cascade.rate_setpoint()(0) > 0.0);
    cascade.feedback(mixer.mix(w));
    REQUIRE(cascade.saturation().thrust == 1);  // full throttle with roll: the collective was cut

    const Wrench<double> held = cascade.update(VehicleState<double>{}, target, 0.0);
    REQUIRE(held.thrust == w.thrust);
    REQUIRE(held.torque(0) == w.torque(0));
    cascade.reset();
    REQUIRE(cascade.saturation().thrust == 0);
    REQUIRE(cascade.rate_setpoint()(0) == 0.0);
}

TEST_CASE("cascade: switching from attitude to position mode drops the attitude-mode target at once", "[cascade]") {
    const auto p = rp1<double>();
    Cascade<double> cascade(cascade_config_from(p));
    const AttitudeTarget<double> manual{from_rotation_vector(vec3(1.0, 0.0, 0.0)), 0.5 * p.mass * p.gravity};
    for (int i = 0; i < 7; ++i) {  // leave the schedule mid-period
        (void)cascade.update(VehicleState<double>{}, manual, 1e-3);
    }
    PositionSetpoint<double> sp;  // hold the origin, where the vehicle is
    const Wrench<double> w = cascade.update(VehicleState<double>{}, sp, 1e-3);
    REQUIRE_THAT(w.thrust, WithinAbs(p.mass * p.gravity, 1e-12));  // the position loop ran on this tick
    REQUIRE_THAT(tilt_between(cascade.target().attitude, Quat<double>{}), WithinAbs(0.0, 1e-7));
    REQUIRE_THAT(cascade.rate_setpoint()(0), WithinAbs(0.0, 1e-12));  // and the attitude loop with it
    REQUIRE(cascade.position().axis(2).integral() == 0.0);
}

TEST_CASE("cascade: derived gains are valid iff thrust-to-weight exceeds 1.25", "[cascade]") {
    auto p = rp1<double>();
    p.max_thrust = 1.3 * p.mass * p.gravity / 4.0;
    REQUIRE(cascade_config_valid(cascade_config_from(p)));
    p.max_thrust = 1.2 * p.mass * p.gravity / 4.0;  // 0.8 · 4 · T_max = 0.96 m g: cannot hover in the limit
    REQUIRE_FALSE(cascade_config_valid(cascade_config_from(p)));
}
