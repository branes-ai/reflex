// Observed order of accuracy (BB-V2): the same flight at five step sizes.
//
// The set-up follows the V&V plan's convergence harness: non-symmetric
// inertia, linear and quadratic drag, initial body rates (2, −1, 3) rad/s,
// and a constant differential command, for 0.64 s. The flight is integrated
// at dt = h₀, h₀/2, …, h₀/16 with h₀ = min(10 ms, τ) = 10 ms, and at h₀/1024
// as the reference. The reserved channels hold the reference run; the
// position error of each step size against it is recorded as log10 of the
// largest component, floored at 1e-16 m (the start, where every run is
// exact). RK4 is fourth order, so each halving of dt lowers the error by
// 2⁴ = 16, about 1.2 decades.
//
// Check (BB-V2): the smallest observed order p = log2(e(dt) / e(dt/2)) over
// the four pairs, from the position error at the end, fitted only on pairs
// whose errors are above 1e3 × the roundoff floor: ε × the position scale,
// about 1e-15 m, so the cutoff is 1e-12 m. All four pairs qualify (the finest
// error is about 8e-12 m); with none, the check would report 0. Pass: p ≥ 3.7.
// Writes $REFLEX_TRACE_DIR/quadrotor/dt_convergence.json.

#include "quadrotor_trace.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace {

namespace model = branes::reflex::model;
namespace trace = branes::reflex::trace;
namespace qt = branes::reflex::trace::quadrotor;
using branes::reflex::vec3;

constexpr double kDtTrace = 0.01;
constexpr int kSamples = 65;  // 0 … 0.64 s
constexpr std::size_t kRuns = 5;
constexpr int kReferenceSteps = 1024;  // per trace sample
constexpr double kFloor = 1e-16;
constexpr double kRoundoff = 1e-15;  // ε × the position scale (about 5 m)

int run() {
    model::QuadrotorParams<double> params{};
    params.inertia = {0.003, 0.0035, 0.005};
    params.drag_linear = {0.1, 0.1, 0.2};
    params.drag_quadratic = {0.02, 0.02, 0.04};
    const qt::Model quad(params);
    const double hover = quad.hover_rotor_speed();
    const qt::Command u{{hover * 1.05, hover * 0.97, hover * 1.02, hover * 0.96}};

    qt::State x0;
    x0.position = vec3(0.0, 0.0, 5.0);
    x0.velocity = vec3(1.0, -0.5, 0.2);
    x0.body_rates = vec3(2.0, -1.0, 3.0);
    x0.rotor_speeds = {hover, hover, hover, hover};

    // Steps per sample of each run: 1, 2, 4, 8, 16 (dt = 10 ms … 0.625 ms);
    // dt divides kDtTrace by a power of two, so it is exact.
    std::array<qt::State, kRuns> runs;
    runs.fill(x0);
    qt::State ref = x0;

    trace::Trace tr({.commit = {},
                     .block = "model/quadrotor",
                     .scenario = "tools/trace/scenarios/quadrotor/dt_convergence.cpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = kDtTrace / kReferenceSteps,
                     .dt_trace = kDtTrace});
    tr.vehicle_arm_length(params.arm_length);
    qt::declare_state(tr);
    tr.channel("position_error_log10", kRuns, "log10 m");

    const auto error = [](const qt::State& a, const qt::State& b) {
        double e = 0;
        for (std::size_t i = 0; i < 3; ++i) {
            e = std::max(e, std::abs(a.position(i) - b.position(i)));
        }
        return e;
    };

    std::array<double, kRuns> final_error{};
    for (int k = 0; k < kSamples; ++k) {
        tr.sample(k * kDtTrace);
        qt::record_state(tr, quad, ref, u);
        std::vector<double> logs;
        for (std::size_t r = 0; r < kRuns; ++r) {
            const double e = error(runs[r], ref);
            final_error[r] = e;
            logs.push_back(std::log10(std::max(e, kFloor)));
        }
        tr.set("position_error_log10", logs);
        if (k + 1 == kSamples) {
            break;
        }
        for (std::size_t r = 0; r < kRuns; ++r) {
            const int steps = 1 << r;
            for (int i = 0; i < steps; ++i) {
                runs[r] = quad.step(runs[r], u, kDtTrace / steps);
            }
        }
        for (int i = 0; i < kReferenceSteps; ++i) {
            ref = quad.step(ref, u, kDtTrace / kReferenceSteps);
        }
    }

    // No pair above the floor would make the check vacuous: report 0.
    double order_min = 0;
    int pairs = 0;
    for (std::size_t r = 0; r + 1 < kRuns; ++r) {
        if (final_error[r + 1] > 1e3 * kRoundoff) {
            const double p = std::log2(final_error[r] / final_error[r + 1]);
            order_min = pairs++ == 0 ? p : std::min(order_min, p);
        }
    }

    tr.check({.id = "BB-V2",
              .metric = "position_order_min",
              .unit = "1",
              .value = order_min,
              .limit = 3.7,
              .comparison = ">="});
    return qt::write(tr, "quadrotor/dt_convergence.json");
}

}  // namespace

int main() {
    return qt::guarded("dt_convergence", run);
}
