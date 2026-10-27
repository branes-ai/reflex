#pragma once

// Torque-free tumbling: a spin about the intermediate principal axis flips.
//
// Rotors off, so there is no torque. The inertia is changed from RP-1's
// (3, 3, 5)·10⁻³ kg·m², which has no intermediate axis, to (2, 4, 6)·10⁻³ by
// default, and the vehicle is spun at 6 rad/s about body y, the intermediate
// axis, with a 0.1 rad/s disturbance about x and z. The disturbance grows at
// about 6·√((J_y − J_x)(J_z − J_y)/(J_x J_z)) ≈ 3.5 /s, and the vehicle flips
// end over end and back, about every 3 s (the Dzhanibekov effect). Kinetic
// energy and the world-frame angular momentum stay constant throughout.
//
// Gravity is set to 1e-6 m/s² so the vehicle stays in view. Gravity acts at
// the centre of mass, so it exerts no torque; the rotation is the same as in
// free fall.
//
// Check (BB-V3): the largest relative drift of kinetic energy and of the
// angular momentum vector.
//
// Sensitivity: near the separatrix the motion amplifies roundoff, but only
// modestly over 5 s. A 1e-15 rad/s change in the initial x rate moves the body
// rates by at most 1e-11 rad/s, so the default regen tolerances (1e-9) hold
// and the trace declares no equivalence override.
//
// The parameters are the three moments of inertia. tumble.cpp writes the
// trace at (2, 4, 6)·10⁻³; the explainer's live widget runs this function,
// compiled to WebAssembly, at the reader's values. With y no longer the
// middle axis, the spin is stable and does not flip.

#include "quadrotor_trace.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace branes::reflex::trace::scenarios::tumble {

/// The widget's parameters: the principal moments of inertia [kg·m²], each in
/// [0.5, 10]·10⁻³.
struct Params {
    std::array<double, 3> inertia{0.002, 0.004, 0.006};
};

namespace model = branes::reflex::model;
namespace trace = branes::reflex::trace;
namespace qt = branes::reflex::trace::quadrotor;
using branes::reflex::Vec3;

inline constexpr double kDtSim = 0.001;
inline constexpr int kStepsPerSample = 10;
inline constexpr double kDtTrace = kDtSim * kStepsPerSample;
inline constexpr int kSamples = 501;  // 0 … 5 s

inline trace::Trace run(const Params& p) {
    for (const double j : p.inertia) {
        if (!(j >= 0.5e-3 && j <= 10e-3)) {
            throw std::invalid_argument("tumble: inertia outside [0.5, 10]e-3 kg·m²");
        }
    }
    model::QuadrotorParams<double> params{};
    params.inertia = p.inertia;
    params.gravity = 1e-6;
    const qt::Model quad(params);
    const auto& j = params.inertia;

    qt::State x;
    x.position = branes::reflex::vec3(0.0, 0.0, 1.0);
    x.body_rates = branes::reflex::vec3(0.1, 6.0, 0.1);

    trace::Trace tr({.commit = {},
                     .block = "model/quadrotor",
                     .scenario = "tools/trace/scenarios/quadrotor/tumble.hpp",
                     .seed = 0,
                     .compiler = {},
                     .dt_sim = kDtSim,
                     .dt_trace = kDtTrace});
    tr.vehicle_arm_length(params.arm_length);
    qt::declare_state(tr);
    tr.channel("kinetic_energy", 1, "J");
    tr.channel("angular_momentum", 3, "N·m·s");

    const auto energy = [&](const qt::State& s) {
        const Vec3<double>& w = s.body_rates;
        return 0.5 * (j[0] * w(0) * w(0) + j[1] * w(1) * w(1) + j[2] * w(2) * w(2));
    };
    // World frame: L = R(q) J ω.
    const auto momentum = [&](const qt::State& s) {
        const Vec3<double>& w = s.body_rates;
        return branes::reflex::mul(branes::reflex::to_matrix(s.attitude),
                                   branes::reflex::vec3(j[0] * w(0), j[1] * w(1), j[2] * w(2)));
    };
    const double e0 = energy(x);
    const Vec3<double> l0 = momentum(x);
    const double l0_norm = branes::reflex::norm(l0);

    double drift_max = 0;
    const qt::Command off{};
    for (int k = 0; k < kSamples; ++k) {
        tr.sample(k * kDtTrace);
        qt::record_state(tr, quad, x, off);
        const Vec3<double> l = momentum(x);
        tr.set("kinetic_energy", energy(x));
        tr.set("angular_momentum", {l(0), l(1), l(2)});
        if (k + 1 == kSamples) {
            break;
        }
        for (int i = 0; i < kStepsPerSample; ++i) {
            x = quad.step(x, off, kDtSim);
            const double de = std::abs(energy(x) - e0) / e0;
            const double dl = branes::reflex::norm(branes::reflex::sub(momentum(x), l0)) / l0_norm;
            drift_max = std::max({drift_max, de, dl});
        }
    }

    tr.check({.id = "BB-V3",
              .metric = "energy_and_momentum_relative_drift_max",
              .unit = "1",
              .value = drift_max,
              .limit = 1e-6,
              .comparison = "<="});
    return tr;
}

}  // namespace branes::reflex::trace::scenarios::tumble
