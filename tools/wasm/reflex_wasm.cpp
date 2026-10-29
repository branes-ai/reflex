// reflex_wasm: the trace scenarios behind the docs-site's live widgets,
// compiled to WebAssembly (Tier 2 of the block-page animation standard,
// docs/standards/animation).
//
// The module runs the same scenario functions as the native trace
// executables (tools/trace/scenarios/<block>/<scenario>.hpp), so a widget
// shows the real headers at the reader's parameters, and CI checks that the
// module and the native build agree at the defaults (the `wasm` equivalence
// profile).
//
// C ABI
// -----
//   const char* reflex_run(const char* scenario, double p0, double p1, double p2)
//       Runs `scenario` (one of those listed below) with up to three
//       parameters, in the order listed below, and returns its reflex-trace/1
//       JSON. The string stays valid until the next call. On an unknown
//       scenario or a parameter outside its range it returns null, and
//       reflex_error() says why.
//   const char* reflex_error(void)
//       The error of the last failed call, or "".
//   const char* reflex_version(void)
//       The commit the module was built from.
//
//   motor_lag   p0 = τ [s]
//   tumble      p0, p1, p2 = J_x, J_y, J_z [kg·m²]
//   roll_step   p0 = attitude gain K [1/s], p1 = rate gain k_p [1/s]
//   pid_terms   p0, p1, p2 = k_p [1/s²], k_i [1/s³], k_d [1/s]
//   pid_derivative_filter  p0 = derivative_tau [s]
//   pid_windup  p0 = k_i [1/s], p1 = hold time [s]
//   mixer_priority  p0 = throttle [1], p1 = yaw torque [N·m]
//   mixer_clip_vs_priority  p0 = throttle [1]

#include "cascade/roll_step.hpp"
#include "mixer/clip_vs_priority.hpp"
#include "mixer/priority.hpp"
#include "pid/derivative_filter.hpp"
#include "pid/terms.hpp"
#include "pid/windup.hpp"
#include "quadrotor/motor_lag.hpp"
#include "quadrotor/tumble.hpp"

#include <cstdlib>
#include <exception>
#include <string>
#include <string_view>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define REFLEX_EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define REFLEX_EXPORT
#endif

#ifndef REFLEX_WASM_COMMIT
#define REFLEX_WASM_COMMIT "unknown"
#endif

namespace {

namespace sc = branes::reflex::trace::scenarios;

std::string g_result;
std::string g_error;

branes::reflex::trace::Trace dispatch(std::string_view scenario, double p0, double p1, double p2) {
    if (scenario == "motor_lag") {
        return sc::motor_lag::run({.tau = p0});
    }
    if (scenario == "tumble") {
        return sc::tumble::run({.inertia = {p0, p1, p2}});
    }
    if (scenario == "roll_step") {
        return sc::roll_step::run({.attitude_gain = p0, .rate_gain = p1});
    }
    if (scenario == "pid_terms") {
        return sc::pid_terms::run({.kp = p0, .ki = p1, .kd = p2});
    }
    if (scenario == "pid_derivative_filter") {
        return sc::pid_derivative_filter::run({.tau = p0});
    }
    if (scenario == "pid_windup") {
        return sc::pid_windup::run({.ki = p0, .hold = p1});
    }
    if (scenario == "mixer_priority") {
        return sc::mixer_priority::run({.throttle = p0, .yaw = p1});
    }
    if (scenario == "mixer_clip_vs_priority") {
        return sc::mixer_clip_vs_priority::run({.throttle = p0});
    }
    throw std::invalid_argument("unknown scenario " + std::string(scenario));
}

}  // namespace

extern "C" {

REFLEX_EXPORT const char* reflex_run(const char* scenario, double p0, double p1, double p2) {
    // The trace records the commit from the environment (provenance, ANI-2).
    setenv("REFLEX_TRACE_COMMIT", REFLEX_WASM_COMMIT, 0);
    try {
        g_result = dispatch(scenario ? scenario : "", p0, p1, p2).to_json();
        g_error.clear();
        return g_result.c_str();
    } catch (const std::exception& e) {
        g_error = e.what();
        return nullptr;
    }
}

REFLEX_EXPORT const char* reflex_error(void) {
    return g_error.c_str();
}

REFLEX_EXPORT const char* reflex_version(void) {
    return REFLEX_WASM_COMMIT;
}

}  // extern "C"
