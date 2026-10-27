// The mirror relation (BB-V4): a run and its reflection through the x–z
// plane, side by side.
//
// The original run starts airborne, tilted, moving, and rotating, and flies
// 1 s of piecewise-constant commands drawn from a seeded generator. The
// mirrored run starts from the reflected state and flies the reflected
// commands. Reflecting y → −y maps the Quad X onto itself with the rotors
// swapped 0 ↔ 2 and 1 ↔ 3, and reverses each rotor's spin, which is the swap's
// own spin direction, so the reflected vehicle is the same vehicle:
//
//   position, velocity   (x, y, z) → (x, −y, z)
//   attitude             (w, x, y, z) → (w, −x, y, −z)
//   body rates           (p, q, r) → (−p, q, −r)
//   rotor speeds, cmds   [0, 1, 2, 3] → [2, 3, 0, 1]
//
// Check (BB-V4): the largest difference between the mirrored run and the
// reflected original, over every state component and sample, relative to the
// largest magnitude of that component in the run. The two runs sum the rotor
// terms in a different order, so they agree to roundoff, not bit for bit; the
// limit is 512 ε. Writes $REFLEX_TRACE_DIR/quadrotor/mirror.json.

#include "quadrotor_trace.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>

namespace {

namespace model = branes::reflex::model;
namespace trace = branes::reflex::trace;
namespace qt = branes::reflex::trace::quadrotor;
using branes::reflex::vec3;

constexpr double kDtSim = 0.001;
constexpr int kStepsPerSample = 10;
constexpr double kDtTrace = kDtSim * kStepsPerSample;
constexpr int kSamples = 101;  // 0 … 1 s
constexpr int kSegment = 100;  // steps per command segment
constexpr int kSegments = 10;
constexpr std::uint64_t kSeed = 7;
constexpr std::array<std::size_t, 4> kSwap = {2, 3, 0, 1};

qt::State reflect(const qt::State& x) {
    qt::State y = x;
    y.position(1) = -x.position(1);
    y.velocity(1) = -x.velocity(1);
    y.attitude = {x.attitude.w, -x.attitude.x, x.attitude.y, -x.attitude.z};
    y.body_rates(0) = -x.body_rates(0);
    y.body_rates(2) = -x.body_rates(2);
    for (std::size_t i = 0; i < model::kRotors; ++i) {
        y.rotor_speeds[i] = x.rotor_speeds[kSwap[i]];
    }
    return y;
}

qt::Command reflect(const qt::Command& u) {
    qt::Command v{};
    for (std::size_t i = 0; i < model::kRotors; ++i) {
        v.speeds[i] = u.speeds[kSwap[i]];
    }
    return v;
}

/// The state as one flat vector, for the comparison.
std::array<double, 17> flat(const qt::State& x) {
    return {x.position(0),
            x.position(1),
            x.position(2),
            x.velocity(0),
            x.velocity(1),
            x.velocity(2),
            x.attitude.w,
            x.attitude.x,
            x.attitude.y,
            x.attitude.z,
            x.body_rates(0),
            x.body_rates(1),
            x.body_rates(2),
            x.rotor_speeds[0],
            x.rotor_speeds[1],
            x.rotor_speeds[2],
            x.rotor_speeds[3]};
}

int run() {
    const model::QuadrotorParams<double> params{};
    const qt::Model quad(params);
    const double hover = quad.hover_rotor_speed();

    // Commands: hover ± up to 8 %, per rotor and per segment. Raw mt19937_64
    // output, which the standard fixes, mapped to [0, 1) by hand: the standard
    // distributions differ between library implementations.
    std::mt19937_64 gen(kSeed);
    std::array<qt::Command, kSegments> segments{};
    for (auto& u : segments) {
        for (double& w : u.speeds) {
            const double r = static_cast<double>(gen() >> 11) * 0x1p-53;
            w = hover * (0.92 + 0.16 * r);
        }
    }
    const auto command = [&](int step) { return segments[static_cast<std::size_t>(step / kSegment)]; };

    qt::State x;
    x.position = vec3(0.0, 0.3, 2.0);
    x.velocity = vec3(0.6, 0.4, 0.1);
    x.attitude = branes::reflex::normalized(branes::reflex::Quat<double>{0.98, 0.08, -0.05, 0.15});
    x.body_rates = vec3(0.8, -0.4, 0.5);
    x.rotor_speeds = {hover * 1.02, hover * 0.97, hover * 1.01, hover * 0.99};
    qt::State m = reflect(x);

    trace::Trace tr({.commit = {},
                     .block = "model/quadrotor",
                     .scenario = "tools/trace/scenarios/quadrotor/mirror.cpp",
                     .seed = kSeed,
                     .compiler = {},
                     .dt_sim = kDtSim,
                     .dt_trace = kDtTrace});
    tr.vehicle_arm_length(params.arm_length);
    qt::declare_state(tr);
    tr.channel("mirror_position", 3, "m");
    tr.channel("mirror_attitude", 4, "1");
    tr.channel("mirror_body_rates", 3, "rad/s");
    tr.channel("mirror_rotor_speeds", 4, "rad/s");
    tr.channel("mirror_rotor_thrusts", 4, "N");

    // Run both, keeping every step for the comparison.
    std::array<double, 17> scale{};
    std::array<double, 17> diff{};
    const auto compare = [&](const qt::State& a, const qt::State& b) {
        const auto fa = flat(reflect(a));
        const auto fb = flat(b);
        for (std::size_t i = 0; i < fa.size(); ++i) {
            scale[i] = std::max(scale[i], std::abs(fa[i]));
            diff[i] = std::max(diff[i], std::abs(fa[i] - fb[i]));
        }
    };
    compare(x, m);
    for (int k = 0; k < kSamples; ++k) {
        const int n = k * kStepsPerSample;
        tr.sample(k * kDtTrace);
        qt::record_state(tr, quad, x, command(std::min(n, kSegment * kSegments - 1)));
        tr.set("mirror_position", {m.position(0), m.position(1), m.position(2)});
        tr.set("mirror_attitude", {m.attitude.w, m.attitude.x, m.attitude.y, m.attitude.z});
        tr.set("mirror_body_rates", {m.body_rates(0), m.body_rates(1), m.body_rates(2)});
        tr.set("mirror_rotor_speeds", m.rotor_speeds);
        tr.set("mirror_rotor_thrusts", quad.rotor_thrusts(m));
        if (k + 1 == kSamples) {
            break;
        }
        for (int i = 0; i < kStepsPerSample; ++i) {
            const qt::Command u = command(n + i);
            x = quad.step(x, u, kDtSim);
            m = quad.step(m, reflect(u), kDtSim);
            compare(x, m);
        }
    }
    double relative_max = 0;
    for (std::size_t i = 0; i < diff.size(); ++i) {
        relative_max = std::max(relative_max, scale[i] > 0 ? diff[i] / scale[i] : diff[i]);
    }

    tr.check({.id = "BB-V4",
              .metric = "mirror_relative_error_max",
              .unit = "1",
              .value = relative_max,
              .limit = 512 * std::numeric_limits<double>::epsilon(),
              .comparison = "<="});
    return qt::write(tr, "quadrotor/mirror.json");
}

}  // namespace

int main() {
    return qt::guarded("mirror", run);
}
