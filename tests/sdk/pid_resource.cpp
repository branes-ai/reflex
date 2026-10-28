// BB-S5 for branes::reflex::Pid: update() runs a fixed operation count for
// each combination of the derivative filter (derivative_tau > 0 or not) and
// the primed flag. Saturation, the integral clamp, anti-windup, and an
// external saturation report select by comparison only, so they leave the
// count unchanged:
//  - 10 always: the error 1, the P term 1, the D term 1, the integration 3
//    (k_i·e·dt and its sum), and the two three-term sums 4 (the candidate
//    output that anti-windup tests, and the output);
//  - the measured rate (a difference and a division) once primed: +2;
//  - the filter's blend (2 ×, 2 +, 1 ÷): +5.
// That is 10 / 12 unfiltered and 15 / 17 filtered, unprimed / primed, and no
// calls. A rejected dt (≤ 0) runs none. The debug-only invariant check
// compares only, so the counts hold in Debug and Release alike.

#include <branes/reflex/control/pid.hpp>

#include "counted.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdint>
#include <random>

using branes::reflex::Pid;
using branes::reflex::PidConfig;
using counted::Counted;

namespace {

struct Cost {
    std::uint64_t ops;
    std::uint64_t calls;
};

Cost update(Pid<Counted>& pid, double r, double y, double dt, int external_saturation) {
    Counted::reset();
    (void)pid.update(Counted(r), Counted(y), Counted(dt), external_saturation);
    return Cost{Counted::ops, Counted::calls};
}

PidConfig<Counted> config(double tau) {
    PidConfig<Counted> cfg;
    cfg.kp = Counted(2);
    cfg.ki = Counted(5);
    cfg.kd = Counted(0.1);
    cfg.output_min = Counted(-2);
    cfg.output_max = Counted(2);
    cfg.integral_min = Counted(-0.5);
    cfg.integral_max = Counted(0.5);
    cfg.derivative_tau = Counted(tau);
    return cfg;
}

}  // namespace

TEST_CASE("S5: a pid update has a fixed operation count per filter and primed state", "[pid][resource]") {
    for (const double tau : {0.0, 0.005}) {
        const bool filtered = tau > 0;
        const std::uint64_t unprimed = filtered ? 15 : 10;
        const std::uint64_t primed = filtered ? 17 : 12;
        INFO("derivative_tau " << tau);

        Pid<Counted> pid(config(tau));
        Cost c = update(pid, 0.2, 0.0, 1e-3, 0);
        REQUIRE(c.ops == unprimed);
        REQUIRE(c.calls == 0);
        c = update(pid, 0.2, 0.01, 1e-3, 0);
        REQUIRE(c.ops == primed);
        REQUIRE(c.calls == 0);

        // A rejected dt holds the output and runs nothing.
        REQUIRE(update(pid, 0.2, 0.01, 0.0, 0).ops == 0);
        REQUIRE(update(pid, 0.2, 0.01, -1e-3, 0).ops == 0);

        pid.reset();
        REQUIRE(update(pid, 0.2, 0.0, 1e-3, 0).ops == unprimed);

        // Generated inputs, with external reports of each sign, reach both
        // output limits, both integral bounds, and an anti-windup freeze in
        // each direction: an integral left unchanged, off its bound, while
        // the error pushes it that way.
        const std::uint32_t seed = 20261027;
        INFO("seed " << seed);
        std::mt19937 rng(seed);
        const auto u = [&rng](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); };
        bool output_high = false;
        bool output_low = false;
        bool integral_high = false;
        bool integral_low = false;
        bool frozen_high = false;
        bool frozen_low = false;
        double r = 0;
        double y = 0;
        for (int i = 0; i < 2000; ++i) {
            // A setpoint held for 50 updates, long enough for the integral to
            // reach a bound, and a slow walk for the measurement, so the D
            // term does not pin the output at a limit.
            if (i % 50 == 0) {
                r = u(-2, 2);
            }
            y = std::clamp(y + u(-5e-3, 5e-3), -2.0, 2.0);
            const int ext = static_cast<int>(rng() % 3) - 1;
            const double before = pid.integral().v;
            c = update(pid, r, y, u(1e-3, 2e-2), ext);
            INFO("update " << i);
            REQUIRE(c.ops == primed);
            REQUIRE(c.calls == 0);
            const double after = pid.integral().v;
            output_high = output_high || pid.output() == Counted(2);
            output_low = output_low || pid.output() == Counted(-2);
            integral_high = integral_high || after == 0.5;
            integral_low = integral_low || after == -0.5;
            frozen_high = frozen_high || (r > y && after == before && before < 0.5);
            frozen_low = frozen_low || (r < y && after == before && before > -0.5);
        }
        REQUIRE(output_high);
        REQUIRE(output_low);
        REQUIRE(integral_high);
        REQUIRE(integral_low);
        REQUIRE(frozen_high);
        REQUIRE(frozen_low);
    }
}
