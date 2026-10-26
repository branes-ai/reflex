// BB-S5 for branes::reflex::model::QuadrotorModel: step() runs a fixed
// operation sequence. An instrumented scalar counts the arithmetic operations
// and square roots of one step; over generated valid states and commands the
// count must be identical within each arm of the ground-contact branch, the
// only data-dependent branch the header admits.

#include <branes/reflex/model/quadrotor.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <compare>
#include <cstdint>
#include <limits>
#include <random>

namespace {

/// A double that counts +, −, ×, ÷ (including compound forms) and sqrt.
/// Negation, comparison, abs, and isfinite are not counted: they are not
/// arithmetic in the sense of BB-S5.
struct Counted {
    static inline std::uint64_t ops = 0;
    static inline std::uint64_t roots = 0;

    double v{0};

    constexpr Counted() = default;
    constexpr Counted(double x) : v(x) {}  // NOLINT(google-explicit-constructor): a scalar stand-in

    friend Counted operator+(Counted a, Counted b) {
        ++ops;
        return a.v + b.v;
    }
    friend Counted operator-(Counted a, Counted b) {
        ++ops;
        return a.v - b.v;
    }
    friend Counted operator*(Counted a, Counted b) {
        ++ops;
        return a.v * b.v;
    }
    friend Counted operator/(Counted a, Counted b) {
        ++ops;
        return a.v / b.v;
    }
    friend Counted operator-(Counted a) {
        return -a.v;
    }
    Counted& operator+=(Counted b) {
        return *this = *this + b;
    }
    Counted& operator-=(Counted b) {
        return *this = *this - b;
    }
    Counted& operator*=(Counted b) {
        return *this = *this * b;
    }
    Counted& operator/=(Counted b) {
        return *this = *this / b;
    }
    friend constexpr std::partial_ordering operator<=>(Counted a, Counted b) {
        return a.v <=> b.v;
    }
    friend Counted sqrt(Counted a) {
        ++roots;
        return std::sqrt(a.v);
    }
    friend Counted abs(Counted a) {
        return std::abs(a.v);
    }
    friend bool isfinite(Counted a) {
        return std::isfinite(a.v);
    }
};

}  // namespace

template <>
struct std::numeric_limits<Counted> : std::numeric_limits<double> {
    static constexpr Counted epsilon() noexcept {
        return std::numeric_limits<double>::epsilon();
    }
};

using namespace branes::reflex;
using namespace branes::reflex::model;

namespace {

struct Count {
    std::uint64_t ops;
    std::uint64_t roots;
};

Count count_step(const QuadrotorModel<Counted>& m,
                 const QuadrotorState<Counted>& x,
                 const RotorCommand<Counted>& u,
                 Counted dt,
                 QuadrotorState<Counted>& y) {
    Counted::ops = 0;
    Counted::roots = 0;
    y = m.step(x, u, dt);
    return Count{Counted::ops, Counted::roots};
}

}  // namespace

TEST_CASE("S5: step() has a fixed operation count per ground-contact arm", "[model][resource]") {
    QuadrotorParams<Counted> p;
    p.drag_linear = {0.3, 0.3, 0.5};
    p.drag_quadratic = {0.03, 0.03, 0.05};
    p.battery_capacity = 50.0;  // small, so generated energies cover a full and an empty battery
    p.battery_sag = 0.2;
    p.power_coefficient = 1e-9;
    const QuadrotorModel<Counted> m(p);
    const double w_max = m.max_rotor_speed().v;

    const std::uint32_t seed = 20261025;
    INFO("seed " << seed);
    std::mt19937 rng(seed);
    const auto uniform = [&rng](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); };

    // Arms: airborne and staying up, airborne and touching down, grounded and
    // staying, grounded and lifting off. The two airborne arms do the same
    // arithmetic, and so do the two grounded arms (plus vertical_thrust).
    bool seen[4] = {false, false, false, false};
    Count airborne{0, 0};
    Count grounded{0, 0};
    for (int i = 0; i < 4000; ++i) {
        QuadrotorState<Counted> x;
        const bool on_ground = i % 2 == 1;
        x.on_ground = on_ground;
        // Airborne states start low half the time, so some of them touch down.
        x.position = vec3(Counted(uniform(-5, 5)),
                          Counted(uniform(-5, 5)),
                          Counted(on_ground ? 0.0 : (i % 4 == 0 ? uniform(0, 2e-3) : uniform(1, 10))));
        x.velocity = on_ground ? zero3<Counted>()
                               : vec3(Counted(uniform(-5, 5)), Counted(uniform(-5, 5)), Counted(uniform(-5, 1)));
        const double tilt = on_ground ? 0.3 : 3.0;
        const Quat<double> q =
            from_rotation_vector(vec3(uniform(-tilt, tilt), uniform(-tilt, tilt), uniform(-3.0, 3.0)));
        x.attitude = Quat<Counted>{q.w, q.x, q.y, q.z};
        x.body_rates = on_ground
                           ? zero3<Counted>()
                           : vec3(Counted(uniform(-10, 10)), Counted(uniform(-10, 10)), Counted(uniform(-10, 10)));
        RotorCommand<Counted> u;
        for (std::size_t r = 0; r < kRotors; ++r) {
            x.rotor_speeds[r] = Counted(uniform(0, w_max));
            u.speeds[r] = Counted(uniform(-0.2 * w_max, 1.2 * w_max));  // includes out-of-range commands
        }
        x.energy = Counted(uniform(0, 100));
        REQUIRE(m.quadrotor_state_valid(x));

        QuadrotorState<Counted> y;
        const Count c = count_step(m, x, u, Counted(uniform(1e-4, 1e-2)), y);
        INFO("state " << i << (on_ground ? " grounded" : " airborne") << ", ops " << c.ops << ", sqrt " << c.roots);
        Count& expected = on_ground ? grounded : airborne;
        if (expected.ops == 0) {
            expected = c;
        }
        REQUIRE(c.ops == expected.ops);
        REQUIRE(c.roots == expected.roots);
        seen[(on_ground ? 2 : 0) + (y.on_ground == on_ground ? 0 : 1)] = true;
    }
    // Every arm was exercised, and the grounded arm costs strictly more.
    REQUIRE(seen[0]);
    REQUIRE(seen[1]);
    REQUIRE(seen[2]);
    REQUIRE(seen[3]);
    REQUIRE(grounded.ops > airborne.ops);
    REQUIRE(grounded.roots == airborne.roots);
}
