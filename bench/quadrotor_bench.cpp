// quadrotor_bench — per-step latency of branes::reflex::model::QuadrotorModel.
//
// Steps the model (RK4, four derivative evaluations per step) with a varying
// rotor command so the compiler cannot hoist the work out of the loop, and
// reports the mean time per step for float and double. SITL steps the model at
// 1-2 kHz and an MPC rollout calls it once per horizon node, so one step should
// stay a few microseconds at most. The vehicle is far above the ground so the
// contact branch never short-circuits the integration.
//
// Usage: quadrotor_bench [iterations]   (default 2,000,000)

#include <branes/reflex/model/quadrotor.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>

namespace {

template <typename Real>
double ns_per_step(long iterations) {
    using namespace branes::reflex;
    const model::QuadrotorModel<Real> m(model::QuadrotorParams<Real>{});
    const Real w = m.hover_rotor_speed();
    model::QuadrotorState<Real> x;
    x.position = vec3(Real(0), Real(0), Real(1e6));
    x.rotor_speeds = {w, w, w, w};
    const Real dt = Real(1.0 / 2000.0);
    const auto start = std::chrono::steady_clock::now();
    for (long i = 0; i < iterations; ++i) {
        const Real wiggle = (i & 0x100) ? Real(1.01) : Real(0.99);
        x = m.step(x, model::RotorCommand<Real>{{w * wiggle, w, w / wiggle, w}}, dt);
        if (x.position(2) < Real(1e5)) {  // keep it airborne
            x.position(2) = Real(1e6);
            x.velocity = zero3<Real>();
        }
    }
    const auto stop = std::chrono::steady_clock::now();
    if (x.attitude.w != x.attitude.w) {
        std::puts("unexpected NaN");
    }
    const auto ns = std::chrono::duration<double, std::nano>(stop - start).count();
    return ns / static_cast<double>(iterations);
}

}  // namespace

int main(int argc, char** argv) {
    const long iterations = argc > 1 ? std::strtol(argv[1], nullptr, 10) : 2'000'000L;
    if (iterations <= 0) {
        std::fprintf(stderr, "iterations must be positive\n");
        return 1;
    }
    std::printf("QuadrotorModel<float>::step   %8.2f ns/step\n", ns_per_step<float>(iterations));
    std::printf("QuadrotorModel<double>::step  %8.2f ns/step\n", ns_per_step<double>(iterations));
    return 0;
}
