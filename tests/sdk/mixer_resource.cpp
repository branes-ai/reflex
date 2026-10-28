// BB-S5 for branes::reflex::control::Mixer: every operation has a bounded
// operation count, and mix() a fixed one apart from two branches the demand
// selects. The priority's clamps and the per-rotor bound select by comparison
// only, so they leave the count unchanged:
//  - mix() 47 always: the range and F/4 2, the roll/pitch and yaw terms 5,
//    the span 1, σ applied 2, the four pair-mean bounds 4, the collective's
//    interval 8, the yaw's interval 4, the pair means 2, the thrusts 4, and
//    wrench() 15;
//  - σ < 1, the division range/span: +1;
//  - τz ≠ 0, the division t/Y for the yaw scale: +1;
//  - the clamp's rounding guard, when rounding leaves the collective's or the
//    yaw's interval empty: +2 each, so mix() never exceeds 53;
//  - allocate() 14, wrench() 15, to_rotor_command() 4 divisions and 4 square
//    roots, and saturation_from() 1 if the collective moved, else 0.
// No other calls. The debug-only precondition checks compare only, so the
// counts hold in Debug and Release alike.

#include <branes/reflex/control/mixer.hpp>
#include <branes/reflex/math/vec3.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include "counted.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <random>

using namespace branes::reflex;
using counted::Counted;

namespace {

struct Cost {
    std::uint64_t ops;
    std::uint64_t calls;
};

control::Wrench<Counted> wrench_of(double f, double tx, double ty, double tz) {
    return {Counted(f), vec3(Counted(tx), Counted(ty), Counted(tz))};
}

}  // namespace

TEST_CASE("S5: mix() has a fixed operation count per branch, bounded by 53", "[mixer][resource]") {
    const control::Mixer<Counted> mixer(control::mixer_params_from(model::QuadrotorParams<Counted>{}));

    const auto mix = [&mixer](const control::Wrench<Counted>& w, control::MixerOutput<Counted>& out) {
        Counted::reset();
        out = mixer.mix(w);
        return Cost{Counted::ops, Counted::calls};
    };

    // One demand per branch: inside the limits, yaw at hover, roll beyond any collective.
    control::MixerOutput<Counted> out;
    Cost c = mix(wrench_of(20, 0.1, -0.05, 0), out);
    REQUIRE(out.rp_scale == Counted(1));
    REQUIRE(c.ops == 47);
    REQUIRE(c.calls == 0);
    REQUIRE(mix(wrench_of(9.81, 0.1, -0.05, 0.05), out).ops == 48);
    c = mix(wrench_of(24, 2.5, 0, 0), out);
    REQUIRE(out.rp_scale < Counted(1));
    REQUIRE(c.ops == 48);
    c = mix(wrench_of(24, 2.5, 0, 0.1), out);
    REQUIRE(out.rp_scale < Counted(1));
    REQUIRE(c.ops == 49);

    // Generated demands from below zero to beyond full thrust and torque, a
    // tenth with no yaw, reach every stage of the priority (σ < 1, yaw cut
    // and kept whole, the collective moved up and down) and the rounding
    // guard, which a saturated demand often meets at the edge of an interval
    // that has shrunk to a point, once or twice in a call.
    const std::uint32_t seed = 20261028;
    INFO("seed " << seed);
    std::mt19937 rng(seed);
    const auto u = [&rng](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); };
    bool rp_cut = false;
    bool yaw_cut = false;
    bool yaw_whole = false;
    bool raised = false;
    bool lowered = false;
    bool guarded = false;
    std::uint64_t worst = 0;
    for (int i = 0; i < 4000; ++i) {
        const double tz = i % 10 == 0 ? 0.0 : u(-0.5, 0.5);
        const auto w = wrench_of(u(-5, 60), u(-2.5, 2.5), u(-2.5, 2.5), tz);
        c = mix(w, out);
        const std::uint64_t base = 47 + (out.rp_scale < Counted(1) ? 1 : 0) + (tz != 0 ? 1 : 0);
        INFO("demand " << i << ": " << c.ops << " ops, branch count " << base);
        REQUIRE((c.ops == base || c.ops == base + 2 || c.ops == base + 4));
        REQUIRE(c.ops <= 53);
        REQUIRE(c.calls == 0);
        guarded = guarded || c.ops != base;
        worst = c.ops > worst ? c.ops : worst;
        rp_cut = rp_cut || out.rp_scale < Counted(1);
        yaw_cut = yaw_cut || (tz != 0 && out.yaw_scale < Counted(1));
        yaw_whole = yaw_whole || (tz != 0 && out.yaw_scale == Counted(1));
        raised = raised || out.achieved.thrust.v > w.thrust.v + 1e-9;
        lowered = lowered || out.achieved.thrust.v < w.thrust.v - 1e-9;
    }
    REQUIRE(rp_cut);
    REQUIRE(yaw_cut);
    REQUIRE(yaw_whole);
    REQUIRE(raised);
    REQUIRE(lowered);
    REQUIRE(guarded);
    REQUIRE(worst >= 51);
}

TEST_CASE("S5: allocate(), wrench(), to_rotor_command(), and saturation_from() have fixed counts",
          "[mixer][resource]") {
    const control::Mixer<Counted> mixer(control::mixer_params_from(model::QuadrotorParams<Counted>{}));
    const auto w = wrench_of(20, 0.1, -0.05, 0.02);

    Counted::reset();
    const auto t = mixer.allocate(w);
    REQUIRE(Counted::ops == 14);
    REQUIRE(Counted::calls == 0);

    Counted::reset();
    (void)mixer.wrench(t);
    REQUIRE(Counted::ops == 15);
    REQUIRE(Counted::calls == 0);

    Counted::reset();
    (void)mixer.to_rotor_command(t);
    REQUIRE(Counted::ops == 4);
    REQUIRE(Counted::calls == 4);

    // The report's only arithmetic is demand − achieved, for a moved collective.
    const auto inside = mixer.mix(w);
    REQUIRE(!inside.thrust_shifted);
    Counted::reset();
    (void)control::saturation_from(w, inside);
    REQUIRE(Counted::ops == 0);
    const auto airmode = wrench_of(0, 0.5, 0, 0);
    const auto lifted = mixer.mix(airmode);
    REQUIRE(lifted.thrust_shifted);
    Counted::reset();
    (void)control::saturation_from(airmode, lifted);
    REQUIRE(Counted::ops == 1);
    REQUIRE(Counted::calls == 0);
}
