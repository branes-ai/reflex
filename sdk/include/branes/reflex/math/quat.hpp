#pragma once
/// @file quat.hpp
/// @brief Quat: a rotation quaternion (Hamilton convention, scalar first).
///
/// MTL5 has no quaternion type, so reflex carries this small one. As an
/// attitude it rotates body-frame vectors into the world frame:
/// v_world = q ⊗ (0, v_body) ⊗ q*.

#include <branes/reflex/math/vec3.hpp>

#include <cmath>

namespace branes::reflex {

template <typename T>
struct Quat {
    T w{1};
    T x{0};
    T y{0};
    T z{0};

    friend constexpr bool operator==(const Quat&, const Quat&) = default;
};

/// Hamilton product a ⊗ b.
template <typename T>
[[nodiscard]] constexpr Quat<T> operator*(const Quat<T>& a, const Quat<T>& b) noexcept {
    return Quat<T>{a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                   a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                   a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                   a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

template <typename T>
[[nodiscard]] constexpr Quat<T> conjugate(const Quat<T>& q) noexcept {
    return Quat<T>{q.w, -q.x, -q.y, -q.z};
}

template <typename T>
[[nodiscard]] constexpr T norm_squared(const Quat<T>& q) noexcept {
    return q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
}

/// |‖q‖² − 1| ≤ tol.
template <typename T>
[[nodiscard]] constexpr bool is_unit(const Quat<T>& q, T tol) noexcept {
    const T d = norm_squared(q) - T(1);
    return d <= tol && -d <= tol;
}

/// q / ‖q‖. Pre: q ≠ 0.
template <typename T>
[[nodiscard]] Quat<T> normalized(const Quat<T>& q) {
    using std::sqrt;
    const T n = sqrt(norm_squared(q));
    return Quat<T>{q.w / n, q.x / n, q.y / n, q.z / n};
}

/// The rotation matrix R(q), so that R · v == rotate(q, v). Pre: q is unit.
template <typename T>
[[nodiscard]] Mat3<T> to_matrix(const Quat<T>& q) {
    const T ww = q.w * q.w, xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    const T wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    const T xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    Mat3<T> r;
    r(0, 0) = ww + xx - yy - zz;
    r(0, 1) = T(2) * (xy - wz);
    r(0, 2) = T(2) * (xz + wy);
    r(1, 0) = T(2) * (xy + wz);
    r(1, 1) = ww - xx + yy - zz;
    r(1, 2) = T(2) * (yz - wx);
    r(2, 0) = T(2) * (xz - wy);
    r(2, 1) = T(2) * (yz + wx);
    r(2, 2) = ww - xx - yy + zz;
    return r;
}

/// q ⊗ (0, v) ⊗ q*. Pre: q is unit.
template <typename T>
[[nodiscard]] Vec3<T> rotate(const Quat<T>& q, const Vec3<T>& v) {
    return mul(to_matrix(q), v);
}

/// The unit quaternion of a rotation by ‖θ‖ about θ/‖θ‖ (the exponential map).
template <typename T>
[[nodiscard]] Quat<T> from_rotation_vector(const Vec3<T>& theta) {
    using std::cos;
    using std::sin;
    const T angle = norm(theta);
    if (angle == T(0)) {
        return Quat<T>{};
    }
    const T s = sin(angle / T(2)) / angle;
    return Quat<T>{cos(angle / T(2)), s * theta(0), s * theta(1), s * theta(2)};
}

}  // namespace branes::reflex
