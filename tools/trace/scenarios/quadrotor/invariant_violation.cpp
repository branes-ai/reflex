// An invariant violation, found by checking the state before every step.
//
// The model is correct; the caller has a bug. The vehicle hovers at 0.3 m
// and sinks slowly on a command 3 % below the hover speed. Every 0.1 s the
// caller applies an altimeter correction of −2 cm, meant to be applied once,
// as an increment to the state's height. The state steps down 2 cm at each
// correction, reaches the ground sooner than it would, and the correction
// after touch-down puts it below ground_z.
//
// Before each step() the harness calls quadrotor_state_violation(), which
// names the first violated invariant, as the invariant monitor of the V&V
// plan (WS6, BB-D2) will. At the first violation it stops without stepping:
// step() requires a valid state, and a Debug build would assert. The trace
// ends at the invalid state, and the last event is the monitor's report.
// Writes $REFLEX_TRACE_DIR/quadrotor/invariant_violation.json.

#include "quadrotor_trace.hpp"

#include <cstdio>
#include <string>

namespace {

namespace model = branes::reflex::model;
namespace trace = branes::reflex::trace;
namespace qt = branes::reflex::trace::quadrotor;

constexpr double kDtSim = 0.001;
constexpr int kStepsPerSample = 10;
constexpr double kDtTrace = kDtSim * kStepsPerSample;
constexpr int kMaxSteps = 3000;
constexpr int kCorrectionPeriod = 100;  // steps
constexpr double kCorrection = -0.02;   // m

const char* violation_name(model::StateViolation v) {
    switch (v) {
    case model::StateViolation::non_finite:
        return "non_finite";
    case model::StateViolation::attitude_not_unit:
        return "attitude_not_unit";
    case model::StateViolation::rotor_speed:
        return "rotor_speed";
    case model::StateViolation::energy:
        return "energy";
    case model::StateViolation::below_ground:
        return "below_ground";
    case model::StateViolation::none:
        break;
    }
    return "none";
}

int run() {
    const model::QuadrotorParams<double> params{};
    const qt::Model quad(params);
    const double w = 0.97 * quad.hover_rotor_speed();
    const qt::Command u{{w, w, w, w}};
    qt::State x = qt::hover_state(quad, 0.3);

    trace::Trace tr({.commit = {},
                     .block = "model/quadrotor",
                     .scenario = "tools/trace/scenarios/quadrotor/invariant_violation.cpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = kDtSim,
                     .dt_trace = kDtTrace});
    tr.vehicle_arm_length(params.arm_length);
    qt::declare_state(tr);
    tr.channel("height_above_ground", 1, "m");
    tr.channel("on_ground", 1, "1");

    bool touched = false;
    for (int n = 0; n <= kMaxSteps; ++n) {
        const bool correct = n > 0 && n % kCorrectionPeriod == 0;
        if (correct) {
            x.position(2) += kCorrection;  // the caller's bug: an increment, every time
        }
        const model::StateViolation v = quad.quadrotor_state_violation(x);
        const bool last = v != model::StateViolation::none || n == kMaxSteps;
        if (n % kStepsPerSample == 0 || last) {
            // A violation falls on a correction step, a multiple of the sample
            // interval, so the trace stays uniform.
            tr.sample(n * kDtSim);
            qt::record_state(tr, quad, x, u);
            tr.set("height_above_ground", x.position(2) - params.ground_z);
            tr.set("on_ground", x.on_ground ? 1.0 : 0.0);
        }
        if (correct && n == kCorrectionPeriod) {
            tr.event(n * kDtSim, "the caller applies the -2 cm altimeter correction, and again every 0.1 s");
        }
        if (v != model::StateViolation::none) {
            char report[200];
            std::snprintf(report,
                          sizeof report,
                          "monitor: %s (%s) before step %d, position.z = %.6g m < ground_z = %g m",
                          model::invariant_id(v),
                          violation_name(v),
                          n,
                          x.position(2),
                          params.ground_z);
            tr.event(n * kDtSim, report);
            break;
        }
        if (last) {
            break;
        }
        x = quad.step(x, u, kDtSim);
        if (x.on_ground && !touched) {
            touched = true;
            tr.event((n + 1) * kDtSim, "touch-down");
        }
    }
    return qt::write(tr, "quadrotor/invariant_violation.json");
}

}  // namespace

int main() {
    return qt::guarded("invariant_violation", run);
}
