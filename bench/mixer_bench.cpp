// mixer_bench — per-call latency of branes::reflex::control::Mixer.
//
// Feeds mix() and allocate() a stream of demands that sweeps the collective
// across the whole throttle range with a rotating roll/pitch torque and a yaw
// torque, so mix() takes every stage of its priority: inside the limits, yaw
// cut, collective moved, and roll/pitch scaled. One thrust of each result
// feeds the next demand, so the compiler cannot hoist the call out of the
// loop, and every field goes into a checksum tested at the end, so it cannot
// drop the work behind a field nobody reads. The checksum is a separate
// chain, so its additions overlap the next call. mix() and allocate()
// alternate over rounds, so neither gets a warmer cache or clock, and the
// median round is reported, for float and double. A flight-controller rate loop at
// 8 kHz has a 125 us budget per tick for everything; one mix() should be a
// tiny fraction of that.
//
// Usage: mixer_bench [iterations]   (default 10,000,000, over 9 rounds)

#include <branes/reflex/control/mixer.hpp>
#include <branes/reflex/math/vec3.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>

namespace {

using namespace branes::reflex;

/// 64 demands: F from 0 to 52 N, τx and τy on a circle of 0.3 to 2.2 N·m, τz ±0.2 N·m.
template <typename Real>
std::array<control::Wrench<Real>, 64> demands() {
    std::array<control::Wrench<Real>, 64> w{};
    const std::array<double, 8> c{1, 0.7071, 0, -0.7071, -1, -0.7071, 0, 0.7071};
    for (std::size_t i = 0; i < w.size(); ++i) {
        const double r = 0.3 + 1.9 * static_cast<double>(i % 5) / 4.0;
        w[i].thrust = Real(52.0 * static_cast<double>(i) / 63.0);
        w[i].torque = vec3(Real(r * c[i % 8]), Real(r * c[(i + 2) % 8]), Real(i % 2 ? 0.2 : -0.2));
    }
    return w;
}

template <typename Real, typename Call>
double ns_per_call(long iterations, Call call) {
    const auto w = demands<Real>();
    Real sink{0};
    Real checksum{0};
    const auto start = std::chrono::steady_clock::now();
    for (long i = 0; i < iterations; ++i) {
        control::Wrench<Real> d = w[static_cast<std::size_t>(i) & 63];
        d.thrust += sink * Real(1e-30);         // a data dependence on the last result
        const std::array<Real, 2> r = call(d);  // {a thrust, the digest of every field}
        sink = r[0];
        checksum += r[1];
    }
    const auto stop = std::chrono::steady_clock::now();

    // Keep the results observable.
    if (sink != sink || checksum != checksum) {
        std::puts("unexpected NaN");
    }
    const auto ns = std::chrono::duration<double, std::nano>(stop - start).count();
    return ns / static_cast<double>(iterations);
}

template <typename Real>
Real sum(const std::array<Real, control::kMixerRotors>& t) {
    return t[0] + t[1] + t[2] + t[3];
}

/// Every field of a mix() result, so none of its work is dead.
template <typename Real>
Real digest(const control::MixerOutput<Real>& out) {
    const auto& w = out.achieved;
    return sum(out.thrusts) + w.thrust + w.torque(0) + w.torque(1) + w.torque(2) + out.rp_scale + out.yaw_scale +
           Real(out.thrust_shifted ? 1 : 0);
}

double median(std::array<double, 9> v) {
    std::nth_element(v.begin(), v.begin() + 4, v.end());
    return v[4];
}

template <typename Real>
void run(long iterations, const char* type) {
    const control::Mixer<Real> mixer(control::mixer_params_from(model::QuadrotorParams<Real>{}));
    const long per_round = std::max(1L, iterations / 9);
    std::array<double, 9> mix_ns{};
    std::array<double, 9> allocate_ns{};
    for (std::size_t r = 0; r < 9; ++r) {
        const auto time_mix = [&] {
            mix_ns[r] = ns_per_call<Real>(per_round, [&mixer](const control::Wrench<Real>& d) {
                const auto out = mixer.mix(d);
                return std::array<Real, 2>{out.thrusts[0], digest(out)};
            });
        };
        const auto time_allocate = [&] {
            allocate_ns[r] = ns_per_call<Real>(per_round, [&mixer](const control::Wrench<Real>& d) {
                const auto t = mixer.allocate(d);
                return std::array<Real, 2>{t[0], sum(t)};
            });
        };
        if (r % 2 == 0) {
            time_mix();
            time_allocate();
        } else {
            time_allocate();
            time_mix();
        }
    }
    const double mix = median(mix_ns);
    const double allocate = median(allocate_ns);
    std::printf("Mixer<%s>::mix       %8.2f ns/call\n", type, mix);
    std::printf("Mixer<%s>::allocate  %8.2f ns/call\n", type, allocate);
}

}  // namespace

int main(int argc, char** argv) {
    const long iterations = argc > 1 ? std::strtol(argv[1], nullptr, 10) : 10'000'000L;
    if (iterations <= 0) {
        std::fprintf(stderr, "iterations must be positive\n");
        return 1;
    }
    run<float>(iterations, "float");
    run<double>(iterations, "double");
    return 0;
}
