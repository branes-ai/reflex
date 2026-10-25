#pragma once
/// @file position.hpp
/// @brief Position and velocity loops: position setpoint to attitude setpoint and collective thrust.
///
/// Law (world frame, z up)
/// ---
///   1. Velocity setpoint. v_sp = K_p ⊙ (p_sp − p) + v_ff, its horizontal part
///      scaled to ‖v_xy‖ ≤ v_max,xy and its vertical part clamped to
///      [−v_max,down, v_max,up].
///   2. Acceleration. a_i = PID_i(v_sp,i, v_i) + a_ff,i per axis.
///   3. Specific force f = a + g·e₃, saturated vertical first: f_z is clamped
///      to [T_min/m, T_max/m], then ‖f_xy‖ to min(f_z·tan θ_max, √(f_max² − f_z²)).
///   4. Attitude setpoint. b_z = f/‖f‖; with y_c = (−sin ψ, cos ψ, 0) for the
///      yaw setpoint ψ, b_x = (y_c × b_z)/‖y_c × b_z‖ and b_y = b_z × b_x;
///      q_sp = from_matrix([b_x b_y b_z]). θ_max < 90° keeps b_z off the
///      horizontal plane, so y_c × b_z never vanishes.
///   5. Collective thrust T = clamp(m · f·R(q)e₃, T_min, T_max): the force
///      projected on the current thrust axis, so altitude holds while the tilt
///      is still converging.
///
/// Anti-windup: a velocity integrator freezes while the error pushes into a
/// cut of its axis. The cuts are those of step 3 on the previous tick, and on
/// z also the mixer's collective report.
///
/// Model
/// -----
///   PositionController<T>
///     cfg_      : PositionConfig<T> [owned, immutable]
///     pid_      : std::array<Pid<T>, 3> [owned] — velocity loops x, y, z; output acceleration [m/s²]
///     target_   : AttitudeTarget<T> — the last output
///     v_sp_     : Vec3<T> — the last velocity setpoint
///     cut_      : std::array<int, 3> — direction step 3 cut each axis on the last tick
///     tan_tilt_ : T [derived] — tan θ_max
///
/// Invariants
/// ----------
///   I1: position_config_valid(cfg_), tan_tilt_ = tan(θ_max)               — constrains cfg_, tan_tilt_
///   I2: T_min ≤ target_.thrust ≤ T_max; target_.attitude is a unit quaternion — constrains target_
///   I3: ‖v_sp,xy‖ ≤ v_max,xy (to rounding); −v_max,down ≤ v_sp,z ≤ v_max,up — constrains v_sp_
///   I4: each cut_[i] ∈ {−1, 0, 1}                                       — constrains cut_
///
/// A bounded operation count (each limit selects between fixed sequences; see
/// cascade.hpp, BB-S5), no allocation, no exceptions; generic in the scalar type.

#include <branes/reflex/control/pid.hpp>
#include <branes/reflex/math/quat.hpp>
#include <branes/reflex/math/vec3.hpp>

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>

namespace branes::reflex::control {

/// Position-loop configuration. The velocity PIDs' output limits are the
/// acceleration they may command [m/s²].
template <typename T>
struct PositionConfig {
    T mass{1};                                         ///< [kg]
    T gravity{T(9.80665)};                             ///< [m/s²]
    std::array<T, 3> position_gain{T(1), T(1), T(1)};  ///< K_p [1/s]
    std::array<PidConfig<T>, 3> velocity{};            ///< x, y, z
    T max_speed_xy{5};                                 ///< [m/s]
    T max_speed_up{3};                                 ///< [m/s]
    T max_speed_down{2};                               ///< [m/s]
    T max_tilt{T(0.6)};                                ///< θ_max [rad], in (0, π/2)
    T min_thrust{1};                                   ///< collective T_min [N], > 0
    T max_thrust{30};                                  ///< collective T_max [N]
};

/// True if `c` satisfies I1: every value finite; mass, gravity, speeds > 0;
/// gains ≥ 0; valid PID configs; 0 < θ_max < π/2; 0 < T_min < m·g < T_max
/// (the vehicle can hover with margin both ways).
template <typename T>
[[nodiscard]] bool position_config_valid(const PositionConfig<T>& c) noexcept {
    using std::isfinite;
    bool ok = isfinite(c.mass) && isfinite(c.gravity) && isfinite(c.max_speed_xy) && isfinite(c.max_speed_up) &&
              isfinite(c.max_speed_down) && isfinite(c.max_tilt) && isfinite(c.min_thrust) && isfinite(c.max_thrust);
    ok = ok && c.mass > T(0) && c.gravity > T(0) && c.max_speed_xy > T(0) && c.max_speed_up > T(0) &&
         c.max_speed_down > T(0) && c.max_tilt > T(0) && c.max_tilt < T(1.5707963267948966);
    ok = ok && c.min_thrust > T(0) && c.min_thrust < c.mass * c.gravity && c.mass * c.gravity < c.max_thrust;
    for (std::size_t i = 0; i < 3; ++i) {
        ok = ok && isfinite(c.position_gain[i]) && c.position_gain[i] >= T(0) && pid_config_valid(c.velocity[i]);
    }
    return ok;
}

/// What the position loop tracks: position, with velocity and acceleration
/// feed-forward (world frame), and a yaw angle about world z.
template <typename T>
struct PositionSetpoint {
    Vec3<T> position = zero3<T>();      ///< [m]
    Vec3<T> velocity = zero3<T>();      ///< feed-forward [m/s]
    Vec3<T> acceleration = zero3<T>();  ///< feed-forward [m/s²]
    T yaw{0};                           ///< ψ [rad]
};

/// An attitude setpoint and the collective thrust to fly it with.
template <typename T>
struct AttitudeTarget {
    Quat<T> attitude{};  ///< body → world
    T thrust{0};         ///< collective [N]
};

template <typename T>
class PositionController {
public:
    /// Pre: position_config_valid(cfg).
    /// Post: I1–I4; PIDs reset; target level at hover thrust m·g; v_sp = 0; no cuts.
    explicit PositionController(const PositionConfig<T>& cfg)
        : cfg_(cfg), pid_{Pid<T>(cfg.velocity[0]), Pid<T>(cfg.velocity[1]), Pid<T>(cfg.velocity[2])},
          tan_tilt_(tan_of(cfg.max_tilt)) {
        assert(position_config_valid(cfg_));  // Pre / I1
        reset();
    }

    /// One position-loop tick.
    ///
    /// Pre:  every argument finite; q a unit quaternion. A non-positive dt is
    ///       a timing fault in the caller: the last target is returned and no
    ///       state changes.
    /// Post: I1–I4; the tilt of the returned attitude (the angle of its body z
    ///       axis from world z) is ≤ θ_max to rounding.
    ///
    /// @param p, v               measured position [m] and velocity [m/s], world frame
    /// @param q                  measured attitude, body → world
    /// @param sp                 the setpoint
    /// @param dt                 time since the previous tick [s]
    /// @param thrust_saturation  the mixer's collective report (Saturation::thrust)
    [[nodiscard]] AttitudeTarget<T> update(const Vec3<T>& p,
                                           const Vec3<T>& v,
                                           const Quat<T>& q,
                                           const PositionSetpoint<T>& sp,
                                           T dt,
                                           int thrust_saturation = 0) {
        using std::cos;
        using std::sin;
        using std::sqrt;
        if (!(T(0) < dt)) {
            return target_;  // Pre violated: hold (I1–I4 untouched)
        }

        // 1. Velocity setpoint, horizontal norm and vertical range limited (I3).
        Vec3<T> v_sp = add(
            hadamard(vec3(cfg_.position_gain[0], cfg_.position_gain[1], cfg_.position_gain[2]), sub(sp.position, p)),
            sp.velocity);
        const T speed_xy = sqrt(v_sp(0) * v_sp(0) + v_sp(1) * v_sp(1));
        if (cfg_.max_speed_xy < speed_xy) {
            const T s = cfg_.max_speed_xy / speed_xy;
            v_sp(0) = s * v_sp(0);
            v_sp(1) = s * v_sp(1);
        }
        v_sp(2) = clamp(v_sp(2), -cfg_.max_speed_down, cfg_.max_speed_up);
        v_sp_ = v_sp;

        // 2. Acceleration, the integrators held against last tick's cuts.
        Vec3<T> f = zero3<T>();
        for (std::size_t i = 0; i < 3; ++i) {
            const int held = i == 2 && cut_[2] == 0 ? thrust_saturation : cut_[i];
            f(i) = pid_[i].update(v_sp(i), v(i), dt, held) + sp.acceleration(i);
        }

        // 3. Specific force, vertical first (I4: cuts are −1, 0, +1).
        f(2) = f(2) + cfg_.gravity;
        const T f_max = cfg_.max_thrust / cfg_.mass;
        const T f_min = cfg_.min_thrust / cfg_.mass;
        cut_ = {0, 0, 0};
        if (f_max < f(2)) {
            f(2) = f_max;
            cut_[2] = 1;
        } else if (f(2) < f_min) {
            f(2) = f_min;
            cut_[2] = -1;
        }
        const T room = sqrt(f_max * f_max - f(2) * f(2));
        const T tilt_room = f(2) * tan_tilt_;
        const T limit_xy = room < tilt_room ? room : tilt_room;
        const T norm_xy = sqrt(f(0) * f(0) + f(1) * f(1));
        if (limit_xy < norm_xy) {
            const T s = limit_xy / norm_xy;
            cut_[0] = sign(f(0));
            cut_[1] = sign(f(1));
            f(0) = s * f(0);
            f(1) = s * f(1);
        }

        // 4. Attitude: thrust axis along f, heading from the yaw setpoint.
        const Vec3<T> b_z = scale(T(1) / norm(f), f);
        const Vec3<T> y_c = vec3(-sin(sp.yaw), cos(sp.yaw), T(0));
        const Vec3<T> x_raw = cross(y_c, b_z);
        const Vec3<T> b_x = scale(T(1) / norm(x_raw), x_raw);
        const Vec3<T> b_y = cross(b_z, b_x);
        Mat3<T> r;
        for (std::size_t k = 0; k < 3; ++k) {
            r(k, 0) = b_x(k);
            r(k, 1) = b_y(k);
            r(k, 2) = b_z(k);
        }
        target_.attitude = from_matrix(r);

        // 5. Collective thrust along the current thrust axis (I2).
        const T along = cfg_.mass * dot(f, rotate(q, vec3(T(0), T(0), T(1))));
        target_.thrust = clamp(along, cfg_.min_thrust, cfg_.max_thrust);

        assert(check_invariant());
        return target_;
    }

    /// Pre: none.  Post: as after construction.
    void reset() noexcept {
        for (Pid<T>& p : pid_) {
            p.reset();
        }
        target_ = AttitudeTarget<T>{Quat<T>{}, clamp(cfg_.mass * cfg_.gravity, cfg_.min_thrust, cfg_.max_thrust)};
        v_sp_ = zero3<T>();
        cut_ = {0, 0, 0};
        assert(check_invariant());
    }

    [[nodiscard]] const PositionConfig<T>& config() const noexcept {
        return cfg_;
    }

    /// The velocity PID of axis i (0 x, 1 y, 2 z).
    [[nodiscard]] const Pid<T>& axis(std::size_t i) const noexcept {
        return pid_[i];
    }

    [[nodiscard]] const AttitudeTarget<T>& target() const noexcept {
        return target_;
    }

    [[nodiscard]] const Vec3<T>& velocity_setpoint() const noexcept {
        return v_sp_;
    }

    /// The direction step 3 cut each axis on the last tick: +1 the commanded
    /// force is below the demand, −1 above it, 0 not cut.
    [[nodiscard]] const std::array<int, 3>& cut() const noexcept {
        return cut_;
    }

private:
    [[nodiscard]] static T clamp(T x, T lo, T hi) noexcept {
        return x < lo ? lo : (hi < x ? hi : x);
    }

    [[nodiscard]] static int sign(T x) noexcept {
        return x > T(0) ? 1 : (x < T(0) ? -1 : 0);
    }

    [[nodiscard]] static T tan_of(T angle) {
        using std::cos;
        using std::sin;
        return sin(angle) / cos(angle);
    }

    [[nodiscard]] bool check_invariant() const noexcept {
        const T slack = T(1) + T(16) * std::numeric_limits<T>::epsilon();
        bool ok = position_config_valid(cfg_);                                                                    // I1
        ok = ok && !(target_.thrust < cfg_.min_thrust) && !(cfg_.max_thrust < target_.thrust);                    // I2
        ok = ok && is_unit(target_.attitude, T(16) * std::numeric_limits<T>::epsilon());                          // I2
        ok = ok && !(cfg_.max_speed_xy * cfg_.max_speed_xy * slack < v_sp_(0) * v_sp_(0) + v_sp_(1) * v_sp_(1));  // I3
        ok = ok && !(v_sp_(2) < -cfg_.max_speed_down) && !(cfg_.max_speed_up < v_sp_(2));                         // I3
        for (const int c : cut_) {
            ok = ok && -1 <= c && c <= 1;  // I4
        }
        return ok;
    }

    PositionConfig<T> cfg_;
    std::array<Pid<T>, 3> pid_;
    T tan_tilt_;
    AttitudeTarget<T> target_{};
    Vec3<T> v_sp_ = zero3<T>();
    std::array<int, 3> cut_{};
};

}  // namespace branes::reflex::control
