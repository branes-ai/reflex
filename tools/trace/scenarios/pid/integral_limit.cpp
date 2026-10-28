// Integral limit: the integral range is the disturbance the loop can cancel.
//
// The height loop of terms.hpp (z̈ = u + d; k_p = 16, k_i = 8, k_d = 8,
// derivative_tau = 5 ms, output ±20 m/s², at 1 kHz) holds 1 m, with the
// integral limited to ±2 m/s². The offset d steps from 0 to −1.5 m/s² at 1 s
// and to −3 m/s² at 6 s. The first step is inside the integral's authority:
// the integral grows to 1.5 and the height returns to 1 m. The second is
// not: the integral stops at its limit of 2, and the P term carries the
// rest, so the height settles low, at r + (d + I_max)/k_p = 0.9375 m.
//
// Check (BB-V1): the height at 12 s against that closed-form offset, within
// 1 mm (what is left of the transient).
//
// Writes $REFLEX_TRACE_DIR/pid/integral_limit.json.

#include "pid_trace.hpp"

#include <cmath>

namespace {

namespace trace = branes::reflex::trace;
namespace pt = branes::reflex::trace::pid;
using branes::reflex::Pid;
using branes::reflex::PidConfig;

constexpr int kStepsPerSample = 20;
constexpr int kSamples = 601;  // 0 … 12 s
constexpr double kSetpoint = 1;
constexpr double kIntegralMax = 2;

double disturbance(long n) {
    return n < 1000 ? 0.0 : n < 6000 ? -1.5 : -3.0;
}

trace::Trace run() {
    PidConfig<double> cfg;
    cfg.kp = 16;
    cfg.ki = 8;
    cfg.kd = 8;
    cfg.output_min = -20;
    cfg.output_max = 20;
    cfg.integral_min = -kIntegralMax;
    cfg.integral_max = kIntegralMax;
    cfg.derivative_tau = 0.005;
    Pid<double> ctl{cfg};
    pt::DoubleIntegrator plant{.z = kSetpoint};

    trace::Trace tr({.commit = {},
                     .block = "control/pid",
                     .scenario = "tools/trace/scenarios/pid/integral_limit.cpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = pt::kDt,
                     .dt_trace = pt::kDt * kStepsPerSample});
    tr.channel("height", 1, "m");
    tr.channel("setpoint", 1, "m");
    tr.channel("offset_prediction", 1, "m");
    tr.channel("integral", 1, "m/s²");
    tr.channel("integral_limit", 1, "m/s²");
    tr.channel("needed", 1, "m/s²");

    // The steady height: r while the integral can supply −d, and r plus the
    // shortfall over k_p once it is at its limit.
    const auto prediction = [&](double d) { return kSetpoint + std::fmin(0.0, d + kIntegralMax) / cfg.kp; };
    for (long n = 0;; ++n) {
        const double u = ctl.update(kSetpoint, plant.z, pt::kDt);
        if (n % kStepsPerSample == 0) {
            tr.sample(static_cast<double>(n) * pt::kDt);
            tr.set("height", plant.z);
            tr.set("setpoint", kSetpoint);
            tr.set("offset_prediction", prediction(disturbance(n)));
            tr.set("integral", ctl.integral());
            tr.set("integral_limit", kIntegralMax);
            tr.set("needed", -disturbance(n));
            if (n / kStepsPerSample + 1 == kSamples) {
                break;
            }
        }
        plant.step(u, disturbance(n), pt::kDt);
    }

    tr.event(1, "offset −1.5 m/s², within the integral's ±2");
    tr.event(6, "offset −3 m/s², beyond it");
    tr.check({.id = "BB-V1",
              .metric = "height_error_vs_closed_form_offset",
              .unit = "m",
              .value = std::abs(plant.z - prediction(-3.0)),
              .limit = 1e-3,
              .comparison = "<="});
    return tr;
}

}  // namespace

int main() {
    namespace qt = branes::reflex::trace::quadrotor;
    return qt::guarded("integral_limit", [] { return qt::write(run(), "pid/integral_limit.json"); });
}
