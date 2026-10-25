#pragma once
/// @file attitude.hpp
/// @brief Quaternion attitude loop with tilt priority: attitude setpoint to body-rate setpoint.
///
/// Law (tilt-prioritized attitude control, Brescianini and D'Andrea 2018)
/// ---
///   1. Error. q_e = q⁻¹ ⊗ q_sp, the rotation from the current body frame to
///      the desired one, with its scalar part made non-negative so it takes
///      the shorter way round (q and −q give the same result).
///   2. Split. q_e = q_red ⊗ q_yaw: q_red = (w_r, x_r, y_r, 0) tilts the
///      current thrust axis onto the desired one, and q_yaw = (c, 0, 0, s) then
///      turns about it. With n = √(w² + z²):
///        q_red = (n, (w·x − y·z)/n, (w·y + x·z)/n, 0),   q_yaw = (w/n, 0, 0, z/n).
///   3. Rate setpoint. Roll and pitch rates come from the tilt part only, yaw
///      from the yaw part only:
///        ω_sp = clamp((k_x·2x_r, k_y·2y_r, k_z·2z/n) + ω_ff, −ω_max, ω_max).
///      Each error magnitude, 2·sin(angle/2), grows monotonically up to 180°,
///      so even a 180° error drives the vehicle.
///
/// The tilt error therefore does not depend on the yaw error: the thrust axis,
/// which carries the vehicle, is corrected at its own gain whatever the
/// heading does, and yaw, the lowest-priority axis in the mixer too, follows.
/// When the tilt error is 180° (n = 0) the split is undefined; the controller
/// then commands the full error as tilt and no yaw. Euler angles appear
/// nowhere.
///
/// Model
/// -----
///   AttitudeController<T>
///     cfg_ : AttitudeConfig<T> [owned, immutable] — gains and rate limits
///
/// Invariants
/// ----------
///   I1: attitude_config_valid(cfg_)  — constrains cfg_
///
/// The constructor establishes I1 and nothing mutates the controller: it is a
/// pure function of its inputs. A bounded operation count (the 180° fallback
/// is the cheaper branch; see cascade.hpp, BB-S5), no allocation, no exceptions;
/// generic in the scalar type.

#include <branes/reflex/math/quat.hpp>
#include <branes/reflex/math/vec3.hpp>

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>

namespace branes::reflex::control {

template <typename T>
struct AttitudeConfig {
    std::array<T, 3> gain{T(6), T(6), T(3)};        ///< K: rate per error [1/s], roll, pitch, yaw
    std::array<T, 3> max_rate{T(12), T(12), T(4)};  ///< ω_max [rad/s]
};

/// True if `c` satisfies I1: gains finite and ≥ 0, rate limits finite and > 0.
template <typename T>
[[nodiscard]] bool attitude_config_valid(const AttitudeConfig<T>& c) noexcept {
    using std::isfinite;
    bool ok = true;
    for (std::size_t i = 0; i < 3; ++i) {
        ok = ok && isfinite(c.gain[i]) && c.gain[i] >= T(0) && isfinite(c.max_rate[i]) && c.max_rate[i] > T(0);
    }
    return ok;
}

template <typename T>
class AttitudeController {
public:
    /// Pre: attitude_config_valid(cfg).  Post: I1.
    explicit AttitudeController(const AttitudeConfig<T>& cfg) : cfg_(cfg) {
        assert(attitude_config_valid(cfg_));  // Pre / I1
    }

    /// The body-rate setpoint that drives attitude q toward q_sp.
    ///
    /// Pre:  q and q_sp are unit quaternions; rate_ff finite.
    /// Post: |ω_sp,i| ≤ max_rate_i; the result is unchanged by q → −q or
    ///       q_sp → −q_sp; q == q_sp and rate_ff == 0 give ω_sp == 0.
    ///
    /// @param q        current attitude, body → world
    /// @param q_sp     attitude setpoint, body → world
    /// @param rate_ff  body-rate feed-forward [rad/s]
    [[nodiscard]] Vec3<T> update(const Quat<T>& q, const Quat<T>& q_sp, const Vec3<T>& rate_ff = zero3<T>()) const {
        const Vec3<T> e = error(q, q_sp);
        Vec3<T> rate = zero3<T>();
        for (std::size_t i = 0; i < 3; ++i) {
            const T r = cfg_.gain[i] * e(i) + rate_ff(i);
            const T m = cfg_.max_rate[i];
            rate(i) = r < -m ? -m : (m < r ? m : r);
        }
        return rate;
    }

    /// The attitude error of steps 1–2 in the body frame: (2x_r, 2y_r, 2z/n).
    /// Pre: q and q_sp are unit quaternions.  Post: each component in [−2, 2].
    [[nodiscard]] Vec3<T> error(const Quat<T>& q, const Quat<T>& q_sp) const {
        using std::sqrt;
        Quat<T> e = conjugate(q) * q_sp;
        if (e.w < T(0)) {
            e = Quat<T>{-e.w, -e.x, -e.y, -e.z};
        }
        const T n2 = e.w * e.w + e.z * e.z;
        if (!(T(16) * std::numeric_limits<T>::epsilon() < n2)) {
            return vec3(T(2) * e.x, T(2) * e.y, T(0));  // tilt of 180°: all tilt, no yaw
        }
        const T n = sqrt(n2);
        return vec3(T(2) * (e.w * e.x - e.y * e.z) / n, T(2) * (e.w * e.y + e.x * e.z) / n, T(2) * e.z / n);
    }

    [[nodiscard]] const AttitudeConfig<T>& config() const noexcept {
        return cfg_;
    }

private:
    AttitudeConfig<T> cfg_;
};

}  // namespace branes::reflex::control
