#pragma once

// Integrator windup, and the anti-windup that prevents it.
//
// The plant is first order, T·ẋ = −x + u with T = 0.1 s (a body rate behind a
// motor lag), and its actuator saturates at |u| ≤ 1, so the plant can hold
// at most x = 1. Two PI controllers (k_p = 1, k_i = 10) fly two copies of it
// at 1 kHz:
//  - anti-windup: the Pid's output limits are the actuator's, ±1, so its
//    conditional integration engages while the output is saturated;
//  - no anti-windup: a Pid with its limits wide open (±1000), ahead of the
//    same ±1 actuator. Its integral keeps growing while the actuator is
//    pinned, which is what a controller without anti-windup does.
// The setpoint is 1.5, beyond the plant's reach, for the hold time (2 s),
// and then 0.5, which it can reach. The wound-up integral must unwind before
// the output leaves the limit, so the second loop takes about 2.5 s to
// reach 0.5; the first parks its integral at the edge of saturation and
// reaches 0.5 in under 0.1 s. (It then dips to 0.43 and settles within 2 %
// by about 0.5 s: the PI loop's own response at k_p = 1.)
//
// Check (BB-T1): the time from the setpoint drop until the anti-windup run
// first enters a 2 % band around 0.5, at most 0.2 s. It is the metric and
// limit of the closed-loop test "anti-windup recovers quickly from an
// unreachable setpoint", which flies this loop.
//
// The parameters are k_i of both controllers and the hold time. windup.cpp
// writes the trace at the defaults; the explainer's live widget runs this
// function, compiled to WebAssembly, at the reader's values.

#include "pid_trace.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

namespace branes::reflex::trace::scenarios::pid_windup {

/// The widget's parameters, with the ranges the sliders offer.
struct Params {
    double ki = 10;   ///< integral gain [1/s], in [1, 30]
    double hold = 2;  ///< time at the unreachable setpoint [s], in [0.5, 3]
};

namespace trace = branes::reflex::trace;
namespace pt = branes::reflex::trace::pid;

inline constexpr int kStepsPerSample = 10;
inline constexpr double kDtTrace = pt::kDt * kStepsPerSample;
inline constexpr int kSamples = 801;  // 0 … 8 s
inline constexpr double kUnreachable = 1.5;
inline constexpr double kReachable = 0.5;
inline constexpr double kBand = 0.02 * kReachable;

inline trace::Trace run(const Params& p) {
    if (!(p.ki >= 1 && p.ki <= 30) || !(p.hold >= 0.5 && p.hold <= 3)) {
        throw std::invalid_argument("pid_windup: parameters outside the widget's ranges");
    }
    PidConfig<double> guarded;  // output and integral range [-1, 1]: the actuator's
    guarded.kp = 1;
    guarded.ki = p.ki;
    PidConfig<double> open = guarded;
    pt::open_limits(open);
    Pid<double> ctl[2] = {Pid<double>{guarded}, Pid<double>{open}};
    pt::FirstOrderPlant plant[2];
    const long drop = std::lround(p.hold / pt::kDt);

    trace::Trace tr({.commit = {},
                     .block = "control/pid",
                     .scenario = "tools/trace/scenarios/pid/windup.hpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = pt::kDt,
                     .dt_trace = kDtTrace});
    tr.channel("response", 2, "1");
    tr.channel("setpoint", 1, "1");
    tr.channel("command", 2, "1");
    tr.channel("actuator_limit", 1, "1");
    tr.channel("integral", 2, "1");

    double u[2] = {0, 0};
    long recovered[2] = {-1, -1};  // first step in the band after the drop
    for (long n = 0;; ++n) {
        const double r = n < drop ? kUnreachable : kReachable;
        for (int i = 0; i < 2; ++i) {
            if (n >= drop && recovered[i] < 0 && std::abs(plant[i].x - kReachable) < kBand) {
                recovered[i] = n;
            }
            u[i] = ctl[i].update(r, plant[i].x, pt::kDt);
        }
        if (n % kStepsPerSample == 0) {
            tr.sample(static_cast<double>(n) * pt::kDt);
            tr.set("response", {plant[0].x, plant[1].x});
            tr.set("setpoint", r);
            tr.set("command", {u[0], u[1]});
            tr.set("actuator_limit", 1.0);
            tr.set("integral", {ctl[0].integral(), ctl[1].integral()});
            if (n / kStepsPerSample + 1 == kSamples) {
                break;
            }
        }
        for (int i = 0; i < 2; ++i) {
            plant[i].step(std::clamp(u[i], -1.0, 1.0), pt::kDt);
        }
    }

    const double t_drop = static_cast<double>(drop) * pt::kDt;
    const double t_end = (kSamples - 1) * kDtTrace;
    const auto recovery = [&](int i) {
        return recovered[i] < 0 ? t_end - t_drop : static_cast<double>(recovered[i] - drop) * pt::kDt;
    };
    const auto label = [&](const char* run, int i) {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%s reaches 0.5 ± 2 %%, %.2f s after the drop", run, recovery(i));
        return std::string(buf);
    };
    tr.event(0, "setpoint 1.5, beyond the actuator's reach");
    tr.event(t_drop, "setpoint 0.5, within reach");
    for (int i = 0; i < 2; ++i) {
        if (recovered[i] >= 0) {
            tr.event(static_cast<double>(recovered[i]) * pt::kDt,
                     label(i == 0 ? "anti-windup:" : "no anti-windup:", i));
        }
    }
    tr.check({.id = "BB-T1",
              .metric = "recovery_time_to_2pct_band",
              .unit = "s",
              .value = recovery(0),
              .limit = 0.2,
              .comparison = "<="});
    return tr;
}

}  // namespace branes::reflex::trace::scenarios::pid_windup
