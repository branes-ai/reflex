#pragma once
/// @file quadrotor.hpp
/// @brief Rigid-body quadrotor dynamics: the model behind SITL and, later, the MPC.
///
/// Conventions
/// -----------
///  - World frame: z up. Body frame: x forward, y left, z up.
///  - Rotor order (PX4 Quad X): 0 front-right CCW, 1 rear-left CCW,
///    2 front-left CW, 3 rear-right CW (spin as seen from above).
///  - A rotor spinning CCW pushes the body CW: its yaw torque is −k_m·T.
///
/// Dynamics
/// --------
///   rotors   ω̇_i = (clamp(u_i, 0, ω_max) − ω_i) / τ
///   thrust   T_i = s · k_f · ω_i²,    s = 1 − sag · min(E / capacity, 1)
///   position ṗ = v
///   velocity v̇ = R(q) F_b / m − g e_z,
///            F_b = (0, 0, ΣT_i) − D_lin ⊙ v_b − D_quad ⊙ v_b ⊙ |v_b|,  v_b = Rᵀ v
///   attitude q̇ = ½ q ⊗ (0, ω)
///   rates    ω̇ = J⁻¹ (τ − ω × J ω),  τ = Σ r_i × (0, 0, T_i) + (0, 0, −spin_i k_m T_i)
///   energy   Ė = Σ k_p ω_i³
///
/// step() integrates with classical RK4, renormalizes the attitude, and applies
/// the ground contact: a vehicle on the ground stays put (v = ω = 0) while its
/// world-vertical thrust does not exceed its weight, and a vehicle that reaches
/// the ground stops there.
///
/// Model
/// -----
///   QuadrotorModel<T>
///     p_ : QuadrotorParams<T> [owned, immutable] — the vehicle
///
/// Invariants
/// ----------
///   I1: quadrotor_params_valid(p_)                               — constrains p_
///   State invariants (checked by quadrotor_state_valid, kept by step()):
///   I2: the attitude is a unit quaternion (to 1e-6 relative; renormalized every step)
///   I3: 0 ≤ ω_rotor_i ≤ ω_max, and energy ≥ 0
///   I4: every value is finite
///   I5: position.z ≥ ground_z
///
/// The model is stateless apart from its parameters: step() is a pure function
/// of (state, command, dt), so the same inputs give bit-identical outputs. It
/// never allocates and never throws.

#include <branes/reflex/math/quat.hpp>
#include <branes/reflex/math/vec3.hpp>

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>

namespace branes::reflex::model {

inline constexpr std::size_t kRotors = 4;

/// Vehicle parameters. Plain aggregate, passed by value. They map one-to-one
/// onto racing-drone's VehicleConfig.
template <typename T>
struct QuadrotorParams {
    T mass{1};                                               ///< [kg]
    std::array<T, 3> inertia{T(0.003), T(0.003), T(0.005)};  ///< diagonal J [kg·m²]
    T arm_length{T(0.11)};                                   ///< center to rotor axis [m]
    T thrust_coefficient{T(8.9e-7)};                         ///< k_f: T = k_f·ω² [N / (rad/s)²]
    T torque_coefficient{T(0.016)};                          ///< k_m: yaw torque = k_m·T [m]
    T max_thrust{12};                                        ///< per rotor [N]
    T motor_time_constant{T(0.02)};                          ///< τ [s]
    std::array<T, 3> drag_linear{};                          ///< body frame [N / (m/s)]
    std::array<T, 3> drag_quadratic{};                       ///< body frame [N / (m/s)²]
    T gravity{T(9.80665)};                                   ///< [m/s²]
    T ground_z{0};                                           ///< ground height, world frame [m]
    T battery_capacity{T(1e5)};                              ///< usable energy [J]
    T battery_sag{0};                                        ///< thrust fraction lost when empty, ∈ [0, 1)
    T power_coefficient{0};                                  ///< k_p: rotor power = k_p·ω³ [W / (rad/s)³]
};

/// True if `p` satisfies I1.
template <typename T>
[[nodiscard]] constexpr bool quadrotor_params_valid(const QuadrotorParams<T>& p) noexcept {
    const T zero{0};
    bool ok = p.mass > zero && p.arm_length > zero && p.thrust_coefficient > zero && p.torque_coefficient > zero &&
              p.max_thrust > zero && p.motor_time_constant > zero && p.gravity > zero && p.battery_capacity > zero &&
              p.battery_sag >= zero && p.battery_sag < T(1) && p.power_coefficient >= zero;
    for (std::size_t i = 0; i < 3; ++i) {
        ok = ok && p.inertia[i] > zero && p.drag_linear[i] >= zero && p.drag_quadratic[i] >= zero;
    }
    return ok;
}

/// The simulated vehicle state.
template <typename T>
struct QuadrotorState {
    Vec3<T> position = zero3<T>();          ///< world [m]
    Vec3<T> velocity = zero3<T>();          ///< world [m/s]
    Quat<T> attitude{};                     ///< body → world
    Vec3<T> body_rates = zero3<T>();        ///< body frame [rad/s]
    std::array<T, kRotors> rotor_speeds{};  ///< [rad/s]
    T energy{0};                            ///< energy drawn from the battery [J]
    bool on_ground{false};
};

/// Commanded rotor speeds [rad/s], clamped to [0, ω_max] by the model.
template <typename T>
struct RotorCommand {
    std::array<T, kRotors> speeds{};
};

/// The time derivative of a QuadrotorState (on_ground excluded).
template <typename T>
struct QuadrotorDerivative {
    Vec3<T> position = zero3<T>();
    Vec3<T> velocity = zero3<T>();
    Quat<T> attitude{T(0), T(0), T(0), T(0)};
    Vec3<T> body_rates = zero3<T>();
    std::array<T, kRotors> rotor_speeds{};
    T energy{0};
};

template <typename T>
class QuadrotorModel {
public:
    /// Pre: quadrotor_params_valid(p) (I1).
    explicit QuadrotorModel(const QuadrotorParams<T>& p) : p_(p) {
        assert(quadrotor_params_valid(p_));  // Pre / I1
        using std::sqrt;
        omega_max_ = sqrt(p_.max_thrust / p_.thrust_coefficient);
        const T a = p_.arm_length / sqrt(T(2));
        rotor_position_ = {vec3(a, -a, T(0)), vec3(-a, a, T(0)), vec3(a, a, T(0)), vec3(-a, -a, T(0))};
    }

    [[nodiscard]] const QuadrotorParams<T>& params() const noexcept {
        return p_;
    }

    /// Rotor speed at which one rotor makes max_thrust.
    [[nodiscard]] T max_rotor_speed() const noexcept {
        return omega_max_;
    }

    /// Rotor speed at which four rotors carry the vehicle's weight (full battery).
    [[nodiscard]] T hover_rotor_speed() const {
        return rotor_speed_for_thrust(p_.mass * p_.gravity / T(kRotors));
    }

    /// ω with k_f·ω² = thrust (full battery). Pre: thrust ≥ 0.
    [[nodiscard]] T rotor_speed_for_thrust(T thrust) const {
        using std::sqrt;
        return sqrt(thrust / p_.thrust_coefficient);
    }

    /// Position of rotor i in the body frame.
    [[nodiscard]] const Vec3<T>& rotor_position(std::size_t i) const noexcept {
        return rotor_position_[i];
    }

    /// +1 for a CCW rotor, −1 for a CW rotor.
    [[nodiscard]] static constexpr T rotor_spin(std::size_t i) noexcept {
        return i < 2 ? T(1) : T(-1);
    }

    /// The thrust scale from battery sag at energy E.
    [[nodiscard]] T thrust_scale(T energy) const {
        const T used = energy < p_.battery_capacity ? energy / p_.battery_capacity : T(1);
        return T(1) - p_.battery_sag * used;
    }

    /// Thrust of each rotor in state x [N].
    [[nodiscard]] std::array<T, kRotors> rotor_thrusts(const QuadrotorState<T>& x) const {
        const T s = thrust_scale(x.energy);
        std::array<T, kRotors> t{};
        for (std::size_t i = 0; i < kRotors; ++i) {
            t[i] = s * p_.thrust_coefficient * x.rotor_speeds[i] * x.rotor_speeds[i];
        }
        return t;
    }

    /// The free-flight time derivative (no ground contact). A pure function.
    [[nodiscard]] QuadrotorDerivative<T> derivative(const QuadrotorState<T>& x, const RotorCommand<T>& u) const {
        QuadrotorDerivative<T> d;
        const auto thrust = rotor_thrusts(x);

        T total{0};
        Vec3<T> torque = zero3<T>();
        for (std::size_t i = 0; i < kRotors; ++i) {
            const T target = clamp_speed(u.speeds[i]);
            d.rotor_speeds[i] = (target - x.rotor_speeds[i]) / p_.motor_time_constant;
            d.energy += p_.power_coefficient * x.rotor_speeds[i] * x.rotor_speeds[i] * x.rotor_speeds[i];
            total += thrust[i];
            // r × (0, 0, T) = (r_y T, −r_x T, 0), plus the reaction yaw torque.
            const Vec3<T>& r = rotor_position_[i];
            torque = add(torque,
                         vec3(r(1) * thrust[i], -r(0) * thrust[i], -rotor_spin(i) * p_.torque_coefficient * thrust[i]));
        }

        const Mat3<T> rot = to_matrix(x.attitude);
        const Vec3<T> v_body = mul_transpose(rot, x.velocity);
        Vec3<T> force = vec3(T(0), T(0), total);
        for (std::size_t k = 0; k < 3; ++k) {
            using std::abs;
            force(k) = force(k) - p_.drag_linear[k] * v_body(k) - p_.drag_quadratic[k] * v_body(k) * abs(v_body(k));
        }

        d.position = x.velocity;
        d.velocity = axpy(scale(T(1) / p_.mass, mul(rot, force)), T(-1), vec3(T(0), T(0), p_.gravity));

        const Quat<T> omega{T(0), x.body_rates(0), x.body_rates(1), x.body_rates(2)};
        const Quat<T> qdot = x.attitude * omega;
        d.attitude = Quat<T>{qdot.w / T(2), qdot.x / T(2), qdot.y / T(2), qdot.z / T(2)};

        const Vec3<T> j_omega =
            vec3(p_.inertia[0] * x.body_rates(0), p_.inertia[1] * x.body_rates(1), p_.inertia[2] * x.body_rates(2));
        const Vec3<T> net = sub(torque, cross(x.body_rates, j_omega));
        d.body_rates = vec3(net(0) / p_.inertia[0], net(1) / p_.inertia[1], net(2) / p_.inertia[2]);
        return d;
    }

    /// Advance the state by dt with RK4, renormalize the attitude, and apply the
    /// ground contact.
    /// Pre: quadrotor_state_valid(x) (I2–I5); every command is finite; 0 < dt ≤ 0.01 s.
    /// Post: quadrotor_state_valid(result).
    [[nodiscard]] QuadrotorState<T> step(const QuadrotorState<T>& x, const RotorCommand<T>& u, T dt) const {
        assert(quadrotor_state_valid(x));    // Pre / I2–I5
        assert(dt > T(0) && dt <= T(0.01));  // Pre

        const QuadrotorDerivative<T> k1 = derivative(x, u);
        const QuadrotorDerivative<T> k2 = derivative(advance(x, k1, dt / T(2)), u);
        const QuadrotorDerivative<T> k3 = derivative(advance(x, k2, dt / T(2)), u);
        const QuadrotorDerivative<T> k4 = derivative(advance(x, k3, dt), u);

        QuadrotorState<T> y = x;
        const T h6 = dt / T(6);
        const auto combine =
            [&](const Vec3<T>& a, const Vec3<T>& b1, const Vec3<T>& b2, const Vec3<T>& b3, const Vec3<T>& b4) {
                return vec3(a(0) + h6 * (b1(0) + T(2) * b2(0) + T(2) * b3(0) + b4(0)),
                            a(1) + h6 * (b1(1) + T(2) * b2(1) + T(2) * b3(1) + b4(1)),
                            a(2) + h6 * (b1(2) + T(2) * b2(2) + T(2) * b3(2) + b4(2)));
            };
        y.position = combine(x.position, k1.position, k2.position, k3.position, k4.position);
        y.velocity = combine(x.velocity, k1.velocity, k2.velocity, k3.velocity, k4.velocity);
        y.body_rates = combine(x.body_rates, k1.body_rates, k2.body_rates, k3.body_rates, k4.body_rates);
        y.attitude =
            Quat<T>{x.attitude.w + h6 * (k1.attitude.w + T(2) * k2.attitude.w + T(2) * k3.attitude.w + k4.attitude.w),
                    x.attitude.x + h6 * (k1.attitude.x + T(2) * k2.attitude.x + T(2) * k3.attitude.x + k4.attitude.x),
                    x.attitude.y + h6 * (k1.attitude.y + T(2) * k2.attitude.y + T(2) * k3.attitude.y + k4.attitude.y),
                    x.attitude.z + h6 * (k1.attitude.z + T(2) * k2.attitude.z + T(2) * k3.attitude.z + k4.attitude.z)};
        y.attitude = normalized(y.attitude);  // I2
        for (std::size_t i = 0; i < kRotors; ++i) {
            const T w = x.rotor_speeds[i] + h6 * (k1.rotor_speeds[i] + T(2) * k2.rotor_speeds[i] +
                                                  T(2) * k3.rotor_speeds[i] + k4.rotor_speeds[i]);
            y.rotor_speeds[i] = clamp_speed(w);  // I3
        }
        const T e = x.energy + h6 * (k1.energy + T(2) * k2.energy + T(2) * k3.energy + k4.energy);
        y.energy = e > x.energy ? e : x.energy;  // I3: energy never decreases

        // Ground contact (I5). A grounded vehicle whose world-vertical thrust does
        // not exceed its weight stays put; one that reaches the ground stops.
        if (x.on_ground && vertical_thrust(x) <= p_.mass * p_.gravity) {
            y.position = x.position;
            y.velocity = zero3<T>();
            y.attitude = x.attitude;
            y.body_rates = zero3<T>();
            y.on_ground = true;
        } else if (y.position(2) <= p_.ground_z) {
            y.position(2) = p_.ground_z;
            y.velocity = zero3<T>();
            y.body_rates = zero3<T>();
            y.on_ground = true;
        } else {
            y.on_ground = false;
        }
        assert(quadrotor_state_valid(y));  // Post
        return y;
    }

    /// True if x satisfies I2–I5 for this vehicle.
    [[nodiscard]] bool quadrotor_state_valid(const QuadrotorState<T>& x) const {
        using std::isfinite;
        const auto finite3 = [](const Vec3<T>& v) { return isfinite(v(0)) && isfinite(v(1)) && isfinite(v(2)); };
        bool ok = finite3(x.position) && finite3(x.velocity) && finite3(x.body_rates) && isfinite(x.energy) &&
                  isfinite(x.attitude.w) && isfinite(x.attitude.x) && isfinite(x.attitude.y) &&
                  isfinite(x.attitude.z);         // I4
        ok = ok && is_unit(x.attitude, T(1e-6));  // I2
        ok = ok && x.energy >= T(0);              // I3
        for (const T w : x.rotor_speeds) {
            ok = ok && isfinite(w) && w >= T(0) && w <= omega_max_;  // I3, I4
        }
        return ok && x.position(2) >= p_.ground_z;  // I5
    }

private:
    [[nodiscard]] T clamp_speed(T w) const noexcept {
        return w < T(0) ? T(0) : (w > omega_max_ ? omega_max_ : w);
    }

    /// World-vertical component of the total thrust.
    [[nodiscard]] T vertical_thrust(const QuadrotorState<T>& x) const {
        T total{0};
        for (const T t : rotor_thrusts(x)) {
            total += t;
        }
        const Quat<T>& q = x.attitude;
        return (T(1) - T(2) * (q.x * q.x + q.y * q.y)) * total;  // R(2, 2) · ΣT
    }

    /// x + h·d, for the RK4 stages (no renormalization, clamping, or contact).
    [[nodiscard]] static QuadrotorState<T> advance(const QuadrotorState<T>& x, const QuadrotorDerivative<T>& d, T h) {
        QuadrotorState<T> y = x;
        y.position = axpy(x.position, h, d.position);
        y.velocity = axpy(x.velocity, h, d.velocity);
        y.body_rates = axpy(x.body_rates, h, d.body_rates);
        y.attitude = Quat<T>{x.attitude.w + h * d.attitude.w,
                             x.attitude.x + h * d.attitude.x,
                             x.attitude.y + h * d.attitude.y,
                             x.attitude.z + h * d.attitude.z};
        for (std::size_t i = 0; i < kRotors; ++i) {
            y.rotor_speeds[i] = x.rotor_speeds[i] + h * d.rotor_speeds[i];
        }
        y.energy = x.energy + h * d.energy;
        return y;
    }

    QuadrotorParams<T> p_;
    T omega_max_{0};
    std::array<Vec3<T>, kRotors> rotor_position_{};
};

}  // namespace branes::reflex::model
