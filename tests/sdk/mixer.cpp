// Unit tests for branes::reflex::control::Mixer. They check the allocation
// round trip, the identity inside the limits, the saturation priority at the
// corners of the thrust envelope (roll/pitch before collective before yaw), the
// mix() postconditions over a sweep, and the torque signs against
// QuadrotorModel. Run in double and float.

#include <branes/reflex/control/mixer.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <type_traits>

using namespace branes::reflex;
using namespace branes::reflex::control;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

template <typename T>
Wrench<T> wrench_of(T thrust, T tx, T ty, T tz) {
    return Wrench<T>{thrust, vec3(tx, ty, tz)};
}

/// Absolute tolerance for a quantity of magnitude ~scale.
template <typename T>
double tol(double scale) {
    return (std::is_same_v<T, float> ? 1e-5 : 1e-12) * scale;
}

/// The RP-1 mixer: L = 0.11 m, k_m = 0.016 m, T in [0, 12] N per rotor.
template <typename T>
Mixer<T> rp1() {
    return Mixer<T>(mixer_params_from(model::QuadrotorParams<T>{}));
}

template <typename T>
bool within_limits(const Mixer<T>& m, const MixerOutput<T>& out) {
    for (const T t : out.thrusts) {
        if (t < m.params().min_thrust || t > m.params().max_thrust) {
            return false;
        }
    }
    return true;
}

}  // namespace

TEST_CASE("mixer: parameter validity", "[mixer]") {
    REQUIRE(mixer_params_valid(MixerParams<double>{}));
    MixerParams<double> p;
    p.min_thrust = p.max_thrust;
    REQUIRE_FALSE(mixer_params_valid(p));
    p = MixerParams<double>{};
    p.arm_length = 0.0;
    REQUIRE_FALSE(mixer_params_valid(p));
    p = MixerParams<double>{};
    p.torque_coefficient = std::nan("");
    REQUIRE_FALSE(mixer_params_valid(p));
    p = MixerParams<double>{};
    p.min_thrust = -1.0;
    REQUIRE_FALSE(mixer_params_valid(p));
}

TEMPLATE_TEST_CASE("mixer: allocate and wrench are inverses", "[mixer]", double, float) {
    using T = TestType;
    const auto m = rp1<T>();
    const Wrench<T> demands[] = {wrench_of(T(20), T(0.3), T(-0.2), T(0.05)),
                                 wrench_of(T(-4), T(-1), T(2), T(-0.4)),
                                 wrench_of(T(0), T(0), T(0), T(0)),
                                 wrench_of(T(48), T(0), T(0), T(0.7))};
    for (const auto& w : demands) {
        const auto back = m.wrench(m.allocate(w));
        REQUIRE_THAT(double(back.thrust), WithinAbs(double(w.thrust), tol<T>(50)));
        for (std::size_t k = 0; k < 3; ++k) {
            REQUIRE_THAT(double(back.torque(k)), WithinAbs(double(w.torque(k)), tol<T>(5)));
        }
    }
    const std::array<T, 4> t{T(1), T(7), T(3.5), T(11)};
    const auto again = m.allocate(m.wrench(t));
    for (std::size_t i = 0; i < 4; ++i) {
        REQUIRE_THAT(double(again[i]), WithinAbs(double(t[i]), tol<T>(20)));
    }
}

TEMPLATE_TEST_CASE("mixer: the allocation matches the rotor geometry", "[mixer]", double, float) {
    using T = TestType;
    const auto m = rp1<T>();
    // Positive roll (about +x) needs more thrust on the left rotors (1 rear-left, 2 front-left).
    const auto roll = m.allocate(wrench_of(T(20), T(0.2), T(0), T(0)));
    REQUIRE(roll[1] > roll[0]);
    REQUIRE(roll[2] > roll[3]);
    // Positive pitch (about +y, nose down) needs more thrust on the rear rotors (1, 3).
    const auto pitch = m.allocate(wrench_of(T(20), T(0), T(0.2), T(0)));
    REQUIRE(pitch[1] > pitch[2]);
    REQUIRE(pitch[3] > pitch[0]);
    // Positive yaw (CCW from above) needs more thrust on the CW rotors (2, 3).
    const auto yaw = m.allocate(wrench_of(T(20), T(0), T(0), T(0.05)));
    REQUIRE(yaw[2] > yaw[0]);
    REQUIRE(yaw[3] > yaw[1]);
}

TEMPLATE_TEST_CASE("mixer: a demand inside the limits comes back unchanged", "[mixer]", double, float) {
    using T = TestType;
    const auto m = rp1<T>();
    const Wrench<T> demands[] = {wrench_of(T(9.80665), T(0), T(0), T(0)),
                                 wrench_of(T(20), T(0.4), T(-0.3), T(0.1)),
                                 wrench_of(T(40), T(-0.05), T(0.02), T(-0.05))};
    for (const auto& w : demands) {
        const auto out = m.mix(w);
        const auto ref = m.allocate(w);
        for (std::size_t i = 0; i < 4; ++i) {
            REQUIRE(ref[i] >= T(0));  // the demand really is inside the limits
            REQUIRE(ref[i] <= T(12));
            REQUIRE(out.thrusts[i] == ref[i]);
        }
        REQUIRE(out.rp_scale == T(1));
        REQUIRE(out.yaw_scale == T(1));
        REQUIRE_FALSE(out.thrust_shifted);
    }
}

TEMPLATE_TEST_CASE("mixer: yaw that relieves a roll/pitch saturation is not cut", "[mixer]", double, float) {
    using T = TestType;
    // Per-rotor targets (8.9, 0.1, 7.5, 7.5) in a [0, 10] N range: rotor 0 holds
    // c + d_a = 10.4 > 10 without yaw, but yaw brings it to 8.9. A mixer that fit
    // roll/pitch alone first would shift the collective and cut yaw here.
    MixerParams<T> p;
    p.max_thrust = T(10);
    const Mixer<T> m(p);
    const std::array<T, 4> target{T(8.9), T(0.1), T(7.5), T(7.5)};
    const auto out = m.mix(m.wrench(target));
    for (std::size_t i = 0; i < 4; ++i) {
        REQUIRE_THAT(double(out.thrusts[i]), WithinAbs(double(target[i]), tol<T>(20)));
    }
    REQUIRE(out.rp_scale == T(1));
    REQUIRE_THAT(double(out.yaw_scale), WithinAbs(1.0, tol<T>(1)));
}

TEMPLATE_TEST_CASE("mixer: at full throttle the collective drops to keep roll", "[mixer]", double, float) {
    using T = TestType;
    const auto m = rp1<T>();
    const auto w = wrench_of(T(46), T(0.5), T(0), T(0));  // 11.5 N per rotor plus a roll
    const auto out = m.mix(w);
    REQUIRE(within_limits(m, out));
    REQUIRE(out.thrust_shifted);
    REQUIRE(out.rp_scale == T(1));
    REQUIRE_THAT(double(out.achieved.torque(0)), WithinAbs(0.5, tol<T>(1)));
    REQUIRE(out.achieved.thrust < w.thrust);
    REQUIRE_THAT(double(*std::max_element(out.thrusts.begin(), out.thrusts.end())), WithinAbs(12.0, tol<T>(12)));
}

TEMPLATE_TEST_CASE("mixer: at zero throttle the collective rises to keep pitch", "[mixer]", double, float) {
    using T = TestType;
    const auto m = rp1<T>();
    const auto w = wrench_of(T(0), T(0), T(-0.3), T(0));
    const auto out = m.mix(w);
    REQUIRE(within_limits(m, out));
    REQUIRE(out.thrust_shifted);
    REQUIRE(out.rp_scale == T(1));
    REQUIRE_THAT(double(out.achieved.torque(1)), WithinAbs(-0.3, tol<T>(1)));
    REQUIRE(out.achieved.thrust > T(0));
    REQUIRE_THAT(double(*std::min_element(out.thrusts.begin(), out.thrusts.end())), WithinAbs(0.0, tol<T>(12)));
}

TEMPLATE_TEST_CASE("mixer: roll/pitch beyond the envelope scale down with their direction kept",
                   "[mixer]",
                   double,
                   float) {
    using T = TestType;
    const auto m = rp1<T>();
    // a·(T_max − T_min)·2 ≈ 1.87 N·m is the most one axis can get; ask for twice that.
    const auto w = wrench_of(T(24), T(3), T(-1.5), T(0.1));
    const auto out = m.mix(w);
    REQUIRE(within_limits(m, out));
    REQUIRE(out.rp_scale < T(1));
    REQUIRE(out.rp_scale > T(0));
    REQUIRE_THAT(double(out.achieved.torque(0)), WithinAbs(3.0 * double(out.rp_scale), tol<T>(3)));
    REQUIRE_THAT(double(out.achieved.torque(1)), WithinAbs(-1.5 * double(out.rp_scale), tol<T>(3)));
    // Saturating both extremes leaves no room for yaw.
    REQUIRE_THAT(double(out.yaw_scale), WithinAbs(0.0, tol<T>(1)));
}

TEMPLATE_TEST_CASE("mixer: near full throttle yaw yields before thrust and roll", "[mixer]", double, float) {
    using T = TestType;
    const auto m = rp1<T>();
    const auto w = wrench_of(T(40), T(0.2), T(0), T(0.5));  // 10 N per rotor, a large yaw
    const auto out = m.mix(w);
    REQUIRE(within_limits(m, out));
    REQUIRE(out.yaw_scale < T(1));
    REQUIRE(out.yaw_scale > T(0));
    REQUIRE_FALSE(out.thrust_shifted);
    REQUIRE(out.rp_scale == T(1));
    REQUIRE_THAT(double(out.achieved.thrust), WithinAbs(40.0, tol<T>(40)));
    REQUIRE_THAT(double(out.achieved.torque(0)), WithinAbs(0.2, tol<T>(1)));
    REQUIRE_THAT(double(out.achieved.torque(2)), WithinAbs(0.5 * double(out.yaw_scale), tol<T>(1)));
}

TEMPLATE_TEST_CASE("mixer: postconditions hold over a sweep of the demand space", "[mixer]", double, float) {
    using T = TestType;
    MixerParams<T> p;
    p.min_thrust = T(0.5);  // a nonzero idle thrust exercises the lower bound
    const Mixer<T> m(p);
    const T thrusts[] = {T(-10), T(0), T(2), T(10), T(24), T(46), T(48), T(80)};
    const T rolls[] = {T(-4), T(-0.6), T(0), T(0.3), T(2)};
    const T pitches[] = {T(-2), T(0), T(0.5), T(5)};
    const T yaws[] = {T(-1), T(-0.1), T(0), T(0.2), T(3)};
    for (const T f : thrusts) {
        for (const T tx : rolls) {
            for (const T ty : pitches) {
                for (const T tz : yaws) {
                    const auto w = wrench_of(f, tx, ty, tz);
                    const auto out = m.mix(w);
                    REQUIRE(within_limits(m, out));
                    REQUIRE(out.rp_scale > T(0));
                    REQUIRE(out.rp_scale <= T(1));
                    REQUIRE(out.yaw_scale >= T(0));
                    REQUIRE(out.yaw_scale <= T(1));
                    const auto made = m.wrench(out.thrusts);
                    REQUIRE(made.thrust == out.achieved.thrust);
                    const double s = double(out.rp_scale);
                    REQUIRE_THAT(double(out.achieved.torque(0)), WithinAbs(s * double(tx), tol<T>(10)));
                    REQUIRE_THAT(double(out.achieved.torque(1)), WithinAbs(s * double(ty), tol<T>(10)));
                    REQUIRE_THAT(double(out.achieved.torque(2)),
                                 WithinAbs(double(out.yaw_scale) * double(tz), tol<T>(10)));
                }
            }
        }
    }
}

TEMPLATE_TEST_CASE("mixer: the output is continuous across the saturation boundary", "[mixer]", double, float) {
    using T = TestType;
    const auto m = rp1<T>();
    // Walk the collective through the point where the top rotor reaches T_max.
    T prev_max{0};
    bool first = true;
    for (int i = 0; i <= 400; ++i) {
        const T f = T(40) + T(i) * T(0.02);
        const auto out = m.mix(wrench_of(f, T(0.3), T(-0.1), T(0.2)));
        const T top = *std::max_element(out.thrusts.begin(), out.thrusts.end());
        if (!first) {
            REQUIRE(double(top - prev_max) <= 0.02);  // steps no larger than the input step
            REQUIRE(double(prev_max - top) <= 0.02);
        }
        prev_max = top;
        first = false;
    }
}

TEMPLATE_TEST_CASE("mixer: rotor commands reproduce the thrusts", "[mixer]", double, float) {
    using T = TestType;
    const auto m = rp1<T>();
    const std::array<T, 4> t{T(0), T(2.5), T(6), T(12)};
    const auto u = m.to_rotor_command(t);
    for (std::size_t i = 0; i < 4; ++i) {
        const double kf = double(m.params().thrust_coefficient);
        REQUIRE_THAT(kf * double(u.speeds[i]) * double(u.speeds[i]), WithinAbs(double(t[i]), tol<T>(12)));
    }
}

TEMPLATE_TEST_CASE("mixer: the model produces the wrench the mixer was asked for", "[mixer]", double, float) {
    using T = TestType;
    const model::QuadrotorParams<T> qp{};
    const model::QuadrotorModel<T> plant(qp);
    const Mixer<T> m(mixer_params_from(qp));
    const auto w = wrench_of(T(14), T(0.08), T(-0.05), T(0.02));
    const auto out = m.mix(w);
    REQUIRE_FALSE(out.thrust_shifted);
    const auto u = m.to_rotor_command(out.thrusts);

    // Rotors already at the commanded speed, body at rest: ω̇ = J⁻¹τ and v̇_z = F/m − g.
    model::QuadrotorState<T> x;
    x.position = vec3(T(0), T(0), T(10));
    x.rotor_speeds = u.speeds;
    const auto d = plant.derivative(x, u);
    for (std::size_t k = 0; k < 3; ++k) {
        REQUIRE_THAT(double(d.body_rates(k)), WithinAbs(double(w.torque(k)) / double(qp.inertia[k]), tol<T>(500)));
    }
    REQUIRE_THAT(double(d.velocity(2)), WithinAbs(double(w.thrust) / double(qp.mass) - double(qp.gravity), tol<T>(50)));
}
