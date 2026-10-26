// BB-S5 for branes::reflex::dsp: a bounded operation count per call.
//  - Biquad::update runs 9 operations (5 ×, 4 ±) on every finite input and
//    none on a faulted one (plus one for the debug-only invariant check).
//  - RpmNotch::apply runs 9 operations per section and 3 more per blended
//    (w ≠ 0) section, on each of 3 axes: exactly that count on finite input,
//    so at most Rotors · harmonics · 3 · (9 + 3).
//  - RpmNotch::update_rotor_speeds has one arm per notch (in range with a
//    fade, or parked above f_max) and a hold arm per rotor: every call is
//    bounded by the all-in-range call, and every all-in-range call equals it.

#include <branes/reflex/dsp/biquad.hpp>
#include <branes/reflex/dsp/rpm_notch.hpp>
#include <branes/reflex/math/vec3.hpp>

#include "counted.hpp"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>

using namespace branes::reflex;
using namespace branes::reflex::dsp;
using counted::Counted;

namespace {

struct Cost {
    std::uint64_t ops;
    std::uint64_t calls;
};

template <typename F>
Cost cost(F&& f) {
    Counted::reset();
    f();
    return Cost{Counted::ops, Counted::calls};
}

constexpr double kTwoPi = 6.283185307179586;

// Debug builds also run the invariant checks: Biquad's stability test adds one
// operation per update (1 + a2), and RpmNotch's three per apply (the
// configuration's min_hz + fade_hz and two 0.45·fs limits). Release builds
// run none.
#ifdef NDEBUG
constexpr std::uint64_t kCheckOps = 0;
constexpr std::uint64_t kApplyCheckOps = 0;
#else
constexpr std::uint64_t kCheckOps = 1;
constexpr std::uint64_t kApplyCheckOps = 3;
#endif

}  // namespace

TEST_CASE("S5: Biquad::update runs 9 operations per finite sample, 0 on a fault", "[dsp][resource]") {
    Biquad<Counted> b(notch(Counted(264), Counted(5), Counted(8000)));
    const std::uint32_t seed = 20261026;
    INFO("seed " << seed);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> u(-10, 10);
    for (int k = 0; k < 2000; ++k) {
        const Cost c = cost([&] { (void)b.update(Counted(u(rng))); });
        REQUIRE(c.ops == 9 + kCheckOps);
        REQUIRE(c.calls == 0);
    }
    const Cost fault = cost([&] { (void)b.update(Counted(std::numeric_limits<double>::quiet_NaN())); });
    REQUIRE(fault.ops == 0);
}

TEST_CASE("S5: RpmNotch::apply's operation count follows the configuration and the weights", "[dsp][resource]") {
    const std::uint32_t seed = 20261026;
    INFO("seed " << seed);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> u(-10, 10);
    std::uniform_real_distribution<double> w(0, 4000);
    for (unsigned harmonics = 1; harmonics <= 3; ++harmonics) {
        RpmNotchConfig<Counted> cfg;
        cfg.sample_rate = Counted(8000);
        cfg.harmonics = harmonics;
        RpmNotch<Counted> n(cfg);
        const std::uint64_t bound = 4 * harmonics * 3 * (9 + kCheckOps + 3) + kApplyCheckOps;
        std::uint64_t expected = 0;
        for (int k = 0; k < 500; ++k) {
            if (k % 10 == 0) {
                n.update_rotor_speeds({Counted(w(rng)), Counted(w(rng)), Counted(w(rng)), Counted(w(rng))});
                std::uint64_t blended = 0;
                for (std::size_t r = 0; r < 4; ++r) {
                    for (std::size_t h = 0; h < harmonics; ++h) {
                        blended += n.weight(r, h) == Counted(0) ? 0 : 1;
                    }
                }
                expected = 4 * harmonics * 3 * (9 + kCheckOps) + blended * 3 * 3 + kApplyCheckOps;
            }
            const Cost c = cost([&] { (void)n.apply(vec3(Counted(u(rng)), Counted(u(rng)), Counted(u(rng)))); });
            INFO("harmonics " << harmonics << ", sample " << k);
            REQUIRE(c.ops == expected);
            REQUIRE(c.ops <= bound);
            REQUIRE(c.calls == 0);
        }
    }
}

TEST_CASE("S5: RpmNotch::update_rotor_speeds is bounded by its all-in-range call", "[dsp][resource]") {
    RpmNotchConfig<Counted> cfg;
    cfg.sample_rate = Counted(8000);  // f_max 3600 Hz
    RpmNotch<Counted> n(cfg);
    // All twelve notches in range: 1000 Hz fundamental, harmonics to 3000 Hz.
    const Counted in_range(kTwoPi * 1000);
    const Cost worst = cost([&] { n.update_rotor_speeds({in_range, in_range, in_range, in_range}); });
    INFO("all-in-range call: " << worst.ops << " ops, " << worst.calls << " calls");
    REQUIRE(worst.calls == 4 * 3 * 2);  // sin and cos per notch design

    const std::uint32_t seed = 20261026;
    INFO("seed " << seed);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> any(-100, kTwoPi * 3000);  // some faults, some parked
    std::uniform_real_distribution<double> ok(0, kTwoPi * 1200);      // every harmonic ≤ 3600 Hz
    for (int k = 0; k < 2000; ++k) {
        std::array<Counted, 4> omega{};
        for (Counted& v : omega) {
            v = Counted(k % 2 == 0 ? any(rng) : ok(rng));
        }
        if (k % 13 == 0) {
            omega[1] = Counted(std::numeric_limits<double>::quiet_NaN());
        }
        const Cost c = cost([&] { n.update_rotor_speeds(omega); });
        INFO("call " << k << ": " << c.ops << " ops, " << c.calls << " calls");
        REQUIRE(c.ops <= worst.ops);
        REQUIRE(c.calls <= worst.calls);
        if (k % 2 == 1 && k % 13 != 0) {
            REQUIRE(c.ops == worst.ops);
            REQUIRE(c.calls == worst.calls);
        }
    }
}
