// External saturation: report a downstream cut, or the integral winds up
// behind it.
//
// The windup loop of windup.hpp (T·ẋ = −x + u, T = 0.1 s; PI with k_p = 1,
// k_i = 10; output and integral ±1, at 1 kHz), but a stage after the
// controller delivers at most ±0.2: a mixer giving up authority, or a
// thrust limiter. The controller's own limits are five times wider, so they
// never engage. Two copies fly side by side:
//  - reported: each tick passes the stage's last cut to the four-argument
//    update(), as the cascade passes the mixer's report. The integral
//    freezes while the error pushes into the cut;
//  - not reported: the three-argument update(). The integral winds up
//    behind the cut, toward its clamp of 1 (0.9 by 2 s).
// The setpoint is 0.3, beyond the delivered range, for 2 s, and then 0.1.
// The reported run settles within 2 % in about 0.5 s; the other one has to
// unwind first, and takes more than twice as long.
//
// Check (BB-V1): the reported run's integral at the drop, against where it
// parks: the plant holds x = 0.2, and the integral freezes as soon as the
// command reaches the cut, at I = cut − k_p·(r − cut) = 0.1. It parks within
// one tick of integration of that, k_i·e·dt = 1e-3, the limit.
//
// Writes $REFLEX_TRACE_DIR/pid/external_saturation.json.

#include "pid_trace.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

namespace trace = branes::reflex::trace;
namespace pt = branes::reflex::trace::pid;
using branes::reflex::Pid;
using branes::reflex::PidConfig;

constexpr int kStepsPerSample = 10;
constexpr int kSamples = 501;  // 0 … 5 s
constexpr long kDrop = 2000;
constexpr double kCut = 0.2;
constexpr double kUnreachable = 0.3;
constexpr double kReachable = 0.1;

trace::Trace run() {
    PidConfig<double> cfg;  // output and integral range [-1, 1]
    cfg.kp = 1;
    cfg.ki = 10;
    Pid<double> ctl[2] = {Pid<double>{cfg}, Pid<double>{cfg}};
    pt::FirstOrderPlant plant[2];

    trace::Trace tr({.commit = {},
                     .block = "control/pid",
                     .scenario = "tools/trace/scenarios/pid/external_saturation.cpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = pt::kDt,
                     .dt_trace = pt::kDt * kStepsPerSample});
    tr.channel("response", 2, "1");
    tr.channel("setpoint", 1, "1");
    tr.channel("command", 2, "1");
    tr.channel("delivered", 2, "1");
    tr.channel("delivery_limit", 1, "1");
    tr.channel("integral", 2, "1");

    double u[2] = {0, 0};
    double delivered[2] = {0, 0};
    int cut = 0;               // the stage's last report to the reported run: +1 high, −1 low
    long settled[2] = {0, 0};  // first step after the last one outside the band
    double parked = 0;         // the reported run's integral at the drop
    for (long n = 0;; ++n) {
        const double r = n < kDrop ? kUnreachable : kReachable;
        if (n == kDrop) {
            parked = ctl[0].integral();
        }
        for (int i = 0; i < 2; ++i) {
            if (n >= kDrop && std::abs(plant[i].x - kReachable) >= 0.02 * kReachable) {
                settled[i] = n + 1;
            }
        }
        u[0] = ctl[0].update(r, plant[0].x, pt::kDt, cut);
        u[1] = ctl[1].update(r, plant[1].x, pt::kDt);
        for (int i = 0; i < 2; ++i) {
            delivered[i] = std::clamp(u[i], -kCut, kCut);
        }
        cut = delivered[0] < u[0] ? 1 : delivered[0] > u[0] ? -1 : 0;
        if (n % kStepsPerSample == 0) {
            tr.sample(static_cast<double>(n) * pt::kDt);
            tr.set("response", {plant[0].x, plant[1].x});
            tr.set("setpoint", r);
            tr.set("command", {u[0], u[1]});
            tr.set("delivered", {delivered[0], delivered[1]});
            tr.set("delivery_limit", kCut);
            tr.set("integral", {ctl[0].integral(), ctl[1].integral()});
            if (n / kStepsPerSample + 1 == kSamples) {
                break;
            }
        }
        for (int i = 0; i < 2; ++i) {
            plant[i].step(delivered[i], pt::kDt);
        }
    }

    const double t_drop = static_cast<double>(kDrop) * pt::kDt;
    tr.event(0, "setpoint 0.3, beyond the 0.2 the stage delivers");
    tr.event(t_drop, "setpoint 0.1");
    for (int i = 0; i < 2; ++i) {
        char label[96];
        std::snprintf(label,
                      sizeof label,
                      "%s within 2 %% from here, %.2f s after the drop",
                      i == 0 ? "reported:" : "not reported:",
                      static_cast<double>(settled[i] - kDrop) * pt::kDt);
        tr.event(static_cast<double>(settled[i]) * pt::kDt, label);
    }
    tr.check({.id = "BB-V1",
              .metric = "reported_integral_vs_park_value",
              .unit = "1",
              .value = std::abs(parked - (kCut - cfg.kp * (kUnreachable - kCut))),
              .limit = 1e-3,
              .comparison = "<="});
    return tr;
}

}  // namespace

int main() {
    namespace qt = branes::reflex::trace::quadrotor;
    return qt::guarded("external_saturation", [] { return qt::write(run(), "pid/external_saturation.json"); });
}
