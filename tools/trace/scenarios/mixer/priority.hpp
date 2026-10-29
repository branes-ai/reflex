#pragma once

// Saturation priority: what mix() gives up, and in what order, as a roll
// torque demand grows past what the rotors can deliver.
//
// The RP-1 mixer (T_max = 12 N per rotor) is asked for a fixed collective
// thrust, a fixed yaw torque, and a roll torque that ramps from 0 to
// 2.5 N·m over 5 s. Roll alone raises rotors 1 and 2 and lowers 0 and 3 by
// τx/(4a) = 3.2 N per N·m; yaw adds τz/(4k_m) = 1.6 N per 0.1 N·m to rotors
// 2 and 3 and takes it from 0 and 1. At the defaults (throttle 75 %, so
// F/4 = 9 N per rotor, and τz = 0.1 N·m) rotor 2 reaches T_max first, and
// mix() gives way in its fixed order:
//  1. yaw shrinks, from τx = 0.45 N·m (0.9 s), and is gone at 0.94 N·m;
//  2. the collective drops from there (1.9 s), keeping roll whole;
//  3. roll scales down only once 2·τx/(4a) exceeds the thrust range
//     T_max − T_min, at τx = 1.87 N·m (3.7 s).
// The allocation without limits (allocate()) is recorded beside the mixed
// thrusts, so each rotor shows how far past its limit the demand reaches.
//
// Check (BB-T1): mix()'s postcondition, that the achieved torque is the
// demand scaled by (σ, σ, s), to within 1e-12 N·m at every sample.
//
// The parameters are the throttle (F as a fraction of 4·T_max) and the yaw
// torque. priority.cpp writes the trace at the defaults; the explainer's live
// widget runs this function, compiled to WebAssembly, at the reader's values.

#include "mixer_trace.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace branes::reflex::trace::scenarios::mixer_priority {

/// The widget's parameters, with the ranges the sliders offer.
struct Params {
    double throttle = 0.75;  ///< F / (4·T_max), in [0.05, 1]
    double yaw = 0.1;        ///< demanded yaw torque [N·m], in [0, 0.3]
};

namespace trace = branes::reflex::trace;
namespace mt = branes::reflex::trace::mixer;

inline constexpr double kDtTrace = 0.02;
inline constexpr int kSamples = 251;  // 0 … 5 s
inline constexpr double kRollMax = 2.5;

inline trace::Trace run(const Params& p) {
    if (!(p.throttle >= 0.05 && p.throttle <= 1) || !(p.yaw >= 0 && p.yaw <= 0.3)) {
        throw std::invalid_argument("mixer_priority: parameters outside the widget's ranges");
    }
    const auto m = mt::rp1();
    const double thrust = p.throttle * 4 * m.params().max_thrust;
    const double t_end = (kSamples - 1) * kDtTrace;

    trace::Trace tr({.commit = {},
                     .block = "control/mixer",
                     .scenario = "tools/trace/scenarios/mixer/priority.hpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = kDtTrace,
                     .dt_trace = kDtTrace});
    mt::declare_mix(tr);

    double first_yaw = -1;
    double first_shift = -1;
    double first_roll = -1;
    double worst = 0;
    for (int k = 0; k < kSamples; ++k) {
        const double t = k * kDtTrace;
        const mt::Wrench w = mt::wrench_of(thrust, kRollMax * t / t_end, 0, p.yaw);
        const mt::Output out = m.mix(w);
        tr.sample(t);
        mt::record_mix(tr, m, w, out);
        worst = std::max(worst, mt::scaled_torque_error(w, out));
        if (first_yaw < 0 && out.yaw_scale < 1) {
            first_yaw = t;
        }
        if (first_shift < 0 && out.thrust_shifted) {
            first_shift = t;
        }
        if (first_roll < 0 && out.rp_scale < 1) {
            first_roll = t;
        }
    }

    tr.event(0, "roll torque ramps up from 0");
    const auto at = [&](double t, const char* what) {
        if (t >= 0) {
            char label[96];
            std::snprintf(label, sizeof label, "%s, at τx = %.2f N·m", what, kRollMax * t / t_end);
            tr.event(t, label);
        }
    };
    at(first_yaw, "yaw starts to give way");
    at(first_shift, "the collective moves");
    at(first_roll, "roll and pitch scale down");
    tr.check({.id = "BB-T1",
              .metric = "achieved_torque_vs_scaled_demand",
              .unit = "N·m",
              .value = worst,
              .limit = 1e-12,
              .comparison = "<="});
    return tr;
}

}  // namespace branes::reflex::trace::scenarios::mixer_priority
