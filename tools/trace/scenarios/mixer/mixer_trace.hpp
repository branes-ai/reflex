#pragma once

// Shared by the mixer trace scenarios: the RP-1 mixer, and the channels that
// record one mix() call.
//
// Channels (record_mix):
//   thrusts        4  N    mix()'s rotor thrusts, within [T_min, T_max]
//   allocation     4  N    allocate()'s, the same demand with no limits
//   thrust_limits  2  N    T_min, T_max
//   collective     2  N    demanded F, achieved
//   roll_torque    2  N·m  demanded τx, achieved (likewise pitch and yaw)
//   scales         2  1    rp_scale σ, yaw_scale s
//   report         4  1    saturation_from(): roll, pitch, yaw, collective

#include <branes/reflex/control/mixer.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include "../quadrotor/quadrotor_trace.hpp"

namespace branes::reflex::trace::mixer {

namespace control = branes::reflex::control;

using Wrench = control::Wrench<double>;
using Output = control::MixerOutput<double>;

/// The mixer for RP-1, the vehicle the model simulates: T_min = 0, T_max = 12 N.
inline control::Mixer<double> rp1() {
    return control::Mixer<double>(control::mixer_params_from(model::QuadrotorParams<double>{}));
}

inline Wrench wrench_of(double thrust, double tx, double ty, double tz) {
    return Wrench{thrust, vec3(tx, ty, tz)};
}

inline void declare_mix(Trace& tr) {
    tr.channel("thrusts", 4, "N");
    tr.channel("allocation", 4, "N");
    tr.channel("thrust_limits", 2, "N");
    tr.channel("collective", 2, "N");
    tr.channel("roll_torque", 2, "N·m");
    tr.channel("pitch_torque", 2, "N·m");
    tr.channel("yaw_torque", 2, "N·m");
    tr.channel("scales", 2, "1");
    tr.channel("report", 4, "1");
}

inline void record_mix(Trace& tr, const control::Mixer<double>& m, const Wrench& w, const Output& out) {
    const control::Saturation s = control::saturation_from(w, out);
    tr.set("thrusts", out.thrusts);
    tr.set("allocation", m.allocate(w));
    tr.set("thrust_limits", {m.params().min_thrust, m.params().max_thrust});
    tr.set("collective", {w.thrust, out.achieved.thrust});
    tr.set("roll_torque", {w.torque(0), out.achieved.torque(0)});
    tr.set("pitch_torque", {w.torque(1), out.achieved.torque(1)});
    tr.set("yaw_torque", {w.torque(2), out.achieved.torque(2)});
    tr.set("scales", {out.rp_scale, out.yaw_scale});
    tr.set("report",
           {static_cast<double>(s.torque[0]),
            static_cast<double>(s.torque[1]),
            static_cast<double>(s.torque[2]),
            static_cast<double>(s.thrust)});
}

/// The postcondition's error: |achieved − (σ·τx, σ·τy, s·τz)|, largest component [N·m].
inline double scaled_torque_error(const Wrench& w, const Output& out) {
    const double e[3] = {out.achieved.torque(0) - out.rp_scale * w.torque(0),
                         out.achieved.torque(1) - out.rp_scale * w.torque(1),
                         out.achieved.torque(2) - out.yaw_scale * w.torque(2)};
    double worst = 0;
    for (const double v : e) {
        worst = std::max(worst, std::abs(v));
    }
    return worst;
}

}  // namespace branes::reflex::trace::mixer
