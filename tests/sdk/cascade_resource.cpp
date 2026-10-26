// BB-S5 for branes::reflex::control::Cascade: a tick has a bounded operation
// count. Its branches (velocity and force limits, the 180° tilt fallback, the
// four Shepperd branches of from_matrix) select between fixed sequences, and
// there are no data-dependent loops. Over generated states and setpoints:
//  - a rate-only tick has one fixed count once the PIDs are primed;
//  - an attitude tick never exceeds a constructed tick whose tilt split runs;
//  - a position tick never exceeds a constructed tick where every limit cuts.

#include <branes/reflex/control/cascade.hpp>
#include <branes/reflex/math/quat.hpp>
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

Cost tick(control::Cascade<Counted>& c,
          const control::VehicleState<Counted>& x,
          const control::PositionSetpoint<Counted>& sp) {
    Counted::reset();
    (void)c.update(x, sp, Counted(1e-3));
    return Cost{Counted::ops, Counted::calls};
}

}  // namespace

TEST_CASE("S5: a cascade tick has a bounded operation count per tick kind", "[cascade][resource]") {
    const auto cfg = control::cascade_config_from(model::QuadrotorParams<Counted>{});
    const unsigned n_att = cfg.attitude_divider;
    const unsigned n_pos = cfg.position_divider;

    // The constructed worst case: a position tick where the horizontal speed
    // is scaled, the vertical force and the horizontal force are both cut,
    // and yaw = π puts from_matrix in a non-trace branch (one more operation
    // than the trace branch). Its attitude tick takes the tilt split.
    control::VehicleState<Counted> worst_x;
    worst_x.attitude = from_rotation_vector(vec3(Counted(0.3), Counted(-0.2), Counted(0.1)));
    worst_x.body_rates = vec3(Counted(1), Counted(-1), Counted(0.5));
    control::PositionSetpoint<Counted> worst_sp;
    worst_sp.position = vec3(Counted(100), Counted(-100), Counted(100));
    worst_sp.acceleration = vec3(Counted(50), Counted(50), Counted(100));
    worst_sp.yaw = Counted(3.141592653589793);
    // Measured over the second position period, so every PID is primed (a
    // PID's first sample skips the derivative's difference and division).
    control::Cascade<Counted> probe(cfg);
    Cost worst_position{0, 0};
    Cost worst_attitude{0, 0};
    Cost rate_only{0, 0};
    for (unsigned k = 0; k < 2 * n_pos; ++k) {
        const Cost c = tick(probe, worst_x, worst_sp);
        if (k < n_pos) {
            continue;
        }
        if (k % n_pos == 0) {
            worst_position = c;
            REQUIRE(probe.position().cut()[0] != 0);
            REQUIRE(probe.position().cut()[2] == 1);
        } else if (k % n_att == 0) {
            worst_attitude = c;
        } else {
            rate_only = c;
        }
    }
    INFO("worst position tick " << worst_position.ops << " ops / " << worst_position.calls << " calls; attitude tick "
                                << worst_attitude.ops << " / " << worst_attitude.calls << "; rate tick "
                                << rate_only.ops << " / " << rate_only.calls);
    REQUIRE(worst_attitude.ops > rate_only.ops);
    REQUIRE(worst_position.ops > worst_attitude.ops);

    const std::uint32_t seed = 20261025;
    INFO("seed " << seed);
    std::mt19937 rng(seed);
    const auto u = [&rng](double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); };
    control::Cascade<Counted> cascade(cfg);
    for (unsigned i = 0; i < 200 * n_pos; ++i) {
        control::VehicleState<Counted> x;
        x.position = vec3(Counted(u(-50, 50)), Counted(u(-50, 50)), Counted(u(0, 50)));
        x.velocity = vec3(Counted(u(-20, 20)), Counted(u(-20, 20)), Counted(u(-10, 10)));
        const double flip = i % 7 == 0 ? 3.141592653589793 : u(-3, 3);  // some 180° tilts
        const Quat<double> q = from_rotation_vector(vec3(flip, u(-1, 1), u(-3, 3)));
        x.attitude = Quat<Counted>{q.w, q.x, q.y, q.z};
        x.body_rates = vec3(Counted(u(-10, 10)), Counted(u(-10, 10)), Counted(u(-5, 5)));
        control::PositionSetpoint<Counted> sp;
        sp.position = vec3(Counted(u(-60, 60)), Counted(u(-60, 60)), Counted(u(0, 60)));
        sp.velocity = vec3(Counted(u(-10, 10)), Counted(u(-10, 10)), Counted(u(-5, 5)));
        sp.acceleration = vec3(Counted(u(-30, 30)), Counted(u(-30, 30)), Counted(u(-30, 30)));
        sp.yaw = Counted(u(-4, 4));

        const Cost c = tick(cascade, x, sp);
        INFO("tick " << i << ": " << c.ops << " ops, " << c.calls << " calls");
        if (i < n_pos) {
            continue;  // the PIDs prime during the first position period
        }
        if (i % n_pos == 0) {
            REQUIRE(c.ops <= worst_position.ops);
            REQUIRE(c.calls <= worst_position.calls);
        } else if (i % n_att == 0) {
            REQUIRE(c.ops <= worst_attitude.ops);
            REQUIRE(c.calls <= worst_attitude.calls);
        } else {
            REQUIRE(c.ops == rate_only.ops);
            REQUIRE(c.calls == rate_only.calls);
        }
    }
}
