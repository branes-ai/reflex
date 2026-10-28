#pragma once

// P, then PD, then PID: what each term adds, on a height loop.
//
// The plant is a double integrator, z̈ = u + d: a height under an
// acceleration command u, with d = −3 m/s² an unmodelled offset (the vehicle
// is heavier than its controller believes). Three Pid controllers fly three
// copies of it from z = 0 to a 1 m setpoint, at 1 kHz for 10 s:
//  - P   (k_p only): undamped. The height oscillates about r + d/k_p and never
//    settles; the semi-implicit Euler step neither adds nor removes energy.
//  - PD  (k_p, k_d): damped. It settles, below the setpoint, at the offset
//    r + d/k_p where the P term alone holds up the extra weight.
//  - PID (k_p, k_i, k_d): the integral grows until it supplies −d, and the
//    offset goes to zero. The loop is stable while k_d·k_p > k_i.
// Limits: output ±20 m/s², integral ±20 m/s², derivative filter τ = 5 ms.
//
// Checks: BB-V1, the PD height at 10 s against its closed-form offset
// r + d/k_p; BB-T1, the PID's remaining error at 10 s (the closed-loop test
// "PID holds a double integrator against a constant disturbance" uses the
// same plant, gains, and 1 mm tolerance). Both allow 1 mm: what is left of
// the transient at 10 s, which a slow choice of gains can exceed.
//
// The parameters are k_p, k_i, k_d of the PID run (the P and PD runs use the
// same k_p and k_d). terms.cpp writes the trace at the defaults; the
// explainer's live widget runs this function, compiled to WebAssembly, at
// the reader's gains.

#include "pid_trace.hpp"

#include <cmath>
#include <stdexcept>

namespace branes::reflex::trace::scenarios::pid_terms {

/// The widget's parameters, with the ranges the sliders offer.
struct Params {
    double kp = 16;  ///< [1/s²], in [2, 40]
    double ki = 8;   ///< [1/s³], in [0, 30]
    double kd = 8;   ///< [1/s], in [0, 20]
};

namespace trace = branes::reflex::trace;
namespace pt = branes::reflex::trace::pid;

inline constexpr int kStepsPerSample = 20;
inline constexpr double kDtTrace = pt::kDt * kStepsPerSample;
inline constexpr int kSamples = 501;  // 0 … 10 s
inline constexpr double kSetpoint = 1;
inline constexpr double kDisturbance = -3;

inline trace::Trace run(const Params& p) {
    if (!(p.kp >= 2 && p.kp <= 40) || !(p.ki >= 0 && p.ki <= 30) || !(p.kd >= 0 && p.kd <= 20)) {
        throw std::invalid_argument("pid_terms: gains outside the widget's ranges");
    }
    PidConfig<double> cfg;
    cfg.kp = p.kp;
    cfg.ki = p.ki;
    cfg.kd = p.kd;
    cfg.output_min = cfg.integral_min = -20;
    cfg.output_max = cfg.integral_max = 20;
    cfg.derivative_tau = 0.005;
    PidConfig<double> p_only = cfg;
    p_only.ki = p_only.kd = 0;
    PidConfig<double> pd = cfg;
    pd.ki = 0;
    Pid<double> ctl[3] = {Pid<double>{p_only}, Pid<double>{pd}, Pid<double>{cfg}};
    pt::DoubleIntegrator plant[3];

    trace::Trace tr({.commit = {},
                     .block = "control/pid",
                     .scenario = "tools/trace/scenarios/pid/terms.hpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = pt::kDt,
                     .dt_trace = kDtTrace});
    tr.channel("height", 3, "m");
    tr.channel("setpoint", 1, "m");
    tr.channel("pd_offset_prediction", 1, "m");
    tr.channel("command", 3, "m/s²");
    tr.channel("integral", 1, "m/s²");
    tr.channel("weight_offset", 1, "m/s²");

    const double offset = kSetpoint + kDisturbance / p.kp;
    double u[3] = {0, 0, 0};
    for (int n = 0;; ++n) {
        for (int i = 0; i < 3; ++i) {
            u[i] = ctl[i].update(kSetpoint, plant[i].z, pt::kDt);
        }
        if (n % kStepsPerSample == 0) {
            tr.sample(n * pt::kDt);
            tr.set("height", {plant[0].z, plant[1].z, plant[2].z});
            tr.set("setpoint", kSetpoint);
            tr.set("pd_offset_prediction", offset);
            tr.set("command", {u[0], u[1], u[2]});
            tr.set("integral", ctl[2].integral());
            tr.set("weight_offset", -kDisturbance);
            if (n / kStepsPerSample + 1 == kSamples) {
                break;
            }
        }
        for (int i = 0; i < 3; ++i) {
            plant[i].step(u[i], kDisturbance, pt::kDt);
        }
    }

    tr.event(0, "setpoint 1 m; the vehicle is 3 m/s² heavier than the controller believes");
    tr.check({.id = "BB-V1",
              .metric = "pd_height_error_vs_closed_form_offset",
              .unit = "m",
              .value = std::abs(plant[1].z - offset),
              .limit = 1e-3,
              .comparison = "<="});
    tr.check({.id = "BB-T1",
              .metric = "pid_steady_state_error",
              .unit = "m",
              .value = std::abs(plant[2].z - kSetpoint),
              .limit = 1e-3,
              .comparison = "<="});
    return tr;
}

}  // namespace branes::reflex::trace::scenarios::pid_terms
