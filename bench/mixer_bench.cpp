// mixer_bench — per-call latency of branes::reflex::control::Mixer.
//
// Feeds mix() and allocate() a stream of demands that sweeps the collective
// across the whole throttle range with a rotating roll/pitch torque and a yaw
// torque, so mix() takes every stage of its priority: inside the limits, yaw
// cut, collective moved, and roll/pitch scaled. Each result feeds the next
// demand, so the compiler cannot hoist the call out of the loop. Reports the
// mean time per call for float and double. A flight-controller rate loop at
// 8 kHz has a 125 us budget per tick for everything; one mix() should be a
// tiny fraction of that.
//
// Usage: mixer_bench [iterations]   (default 10,000,000)

#include <branes/reflex/control/mixer.hpp>
#include <branes/reflex/math/vec3.hpp>
#include <branes/reflex/model/quadrotor.hpp>

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
    const auto start = std::chrono::steady_clock::now();
    for (long i = 0; i < iterations; ++i) {
        control::Wrench<Real> d = w[static_cast<std::size_t>(i) & 63];
        d.thrust += sink * Real(1e-30);  // a data dependence on the last result
        sink = call(d);
    }
    const auto stop = std::chrono::steady_clock::now();

    // Keep the result observable.
    if (sink != sink) {
        std::puts("unexpected NaN");
    }
    const auto ns = std::chrono::duration<double, std::nano>(stop - start).count();
    return ns / static_cast<double>(iterations);
}

template <typename Real>
void run(long iterations, const char* type) {
    const control::Mixer<Real> mixer(control::mixer_params_from(model::QuadrotorParams<Real>{}));
    const double mix =
        ns_per_call<Real>(iterations, [&mixer](const control::Wrench<Real>& d) { return mixer.mix(d).thrusts[0]; });
    const double allocate =
        ns_per_call<Real>(iterations, [&mixer](const control::Wrench<Real>& d) { return mixer.allocate(d)[0]; });
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
