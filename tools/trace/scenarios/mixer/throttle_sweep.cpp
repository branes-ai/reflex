// The collective through the whole throttle range, with a fixed torque
// demand: the three regimes of mix().
//
// The RP-1 mixer is asked for τ = (0.5, 0.2, 0.1) N·m, a moderate roll, a
// small pitch, and some yaw, while the collective demand F sweeps from 0 to
// 52 N, past 4·T_max = 48 N, over 6.5 s.
//  - Low throttle: the lowest rotor would need a negative thrust, so the
//    collective rises above F, as far as roll and pitch need and no
//    further (airmode). Collective ranks above yaw, so yaw gives way rather
//    than lift the collective more: at F = 0 the rotors make 9 N and no yaw.
//    A disarmed vehicle must not reach the mixer: motor idle and arming are
//    the state machine's business.
//  - Mid throttle: every rotor fits, and the demand comes back unchanged.
//  - High throttle: the top rotor reaches T_max. Yaw gives way first, then
//    the collective is held below F; roll and pitch stay whole, because they
//    fit the thrust range at any collective.
// The saturation report (saturation_from()) shows which channel each regime
// cuts, as the control loops receive it.
//
// Check (BB-T1): the output is continuous in the demand. Between samples the
// collective demand steps by ΔF = 0.2 N; no rotor's thrust may step by more
// than ΔF, the bound of the unit test "the output is continuous across the
// saturation boundary".
//
// Writes $REFLEX_TRACE_DIR/mixer/throttle_sweep.json.

#include "mixer_trace.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace {

namespace trace = branes::reflex::trace;
namespace mt = branes::reflex::trace::mixer;

constexpr double kDtTrace = 0.025;
constexpr int kSamples = 261;  // 0 … 6.5 s
constexpr double kThrustMax = 52;

trace::Trace run() {
    const auto m = mt::rp1();
    trace::Trace tr({.commit = {},
                     .block = "control/mixer",
                     .scenario = "tools/trace/scenarios/mixer/throttle_sweep.cpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = kDtTrace,
                     .dt_trace = kDtTrace});
    mt::declare_mix(tr);

    const double t_end = (kSamples - 1) * kDtTrace;
    const double step = kThrustMax / (kSamples - 1);
    std::array<double, 4> prev{};
    double worst = 0;
    bool was_shifted = false;
    bool was_cut = false;
    for (int k = 0; k < kSamples; ++k) {
        const double t = k * kDtTrace;
        const mt::Wrench w = mt::wrench_of(kThrustMax * t / t_end, 0.5, 0.2, 0.1);
        const mt::Output out = m.mix(w);
        tr.sample(t);
        mt::record_mix(tr, m, w, out);
        for (std::size_t i = 0; i < 4 && k > 0; ++i) {
            worst = std::max(worst, std::abs(out.thrusts[i] - prev[i]) / step);
        }
        prev = out.thrusts;
        // Events at each change of regime.
        const bool cut = out.yaw_scale < 1;
        if (k == 0 || out.thrust_shifted != was_shifted || cut != was_cut) {
            const char* collective = !out.thrust_shifted              ? "as demanded"
                                     : out.achieved.thrust > w.thrust ? "raised above F"
                                                                      : "held below F";
            char label[96];
            std::snprintf(
                label, sizeof label, "F = %.1f N: collective %s, yaw %s", w.thrust, collective, cut ? "cut" : "whole");
            tr.event(t, label);
        }
        was_shifted = out.thrust_shifted;
        was_cut = cut;
    }

    tr.check({.id = "BB-T1",
              .metric = "rotor_thrust_step_per_collective_step",
              .unit = "1",
              .value = worst,
              .limit = 1,
              .comparison = "<="});
    return tr;
}

}  // namespace

int main() {
    namespace qt = branes::reflex::trace::quadrotor;
    return qt::guarded("throttle_sweep", [] { return qt::write(run(), "mixer/throttle_sweep.json"); });
}
