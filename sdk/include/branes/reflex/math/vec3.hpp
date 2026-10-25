#pragma once
/// @file vec3.hpp
/// @brief Vec3 / Mat3: fixed-size, stack-allocated MTL5 types, and the 3-D kernels the models use.
///
/// The storage is MTL5 (`dense_vector` / `dense2D` with fixed dimensions and
/// `tag::on_stack`), so these types never allocate and share one linear-algebra
/// layer with the MPC components that follow. The handful of 3-D kernels below
/// (dot, cross, 3×3 · vector) are written out element by element: they are
/// branch-free, allocation-free, and never reach MTL5's threaded paths, which
/// a kHz control loop must not touch.
///
/// Every kernel is generic in the scalar type (double, float, Universal posits).

#include <mtl/mat/dense2D.hpp>
#include <mtl/mat/parameter.hpp>
#include <mtl/vec/dense_vector.hpp>
#include <mtl/vec/parameter.hpp>

#include <cmath>
#include <cstddef>

namespace branes::reflex {

/// A 3-vector on the stack.
template <typename T>
using Vec3 = mtl::vec::
    dense_vector<T, mtl::vec::parameters<mtl::tag::col_major, mtl::vec::fixed::dimension<3>, mtl::tag::on_stack>>;

/// A 3×3 matrix on the stack, row-major.
template <typename T>
using Mat3 = mtl::mat::dense2D<
    T,
    mtl::mat::
        parameters<mtl::tag::row_major, mtl::detail::c_index, mtl::mat::fixed::dimensions<3, 3>, mtl::tag::on_stack>>;

template <typename T>
[[nodiscard]] Vec3<T> vec3(T x, T y, T z) {
    Vec3<T> v;
    v(0) = x;
    v(1) = y;
    v(2) = z;
    return v;
}

template <typename T>
[[nodiscard]] Vec3<T> zero3() {
    return vec3(T(0), T(0), T(0));
}

template <typename T>
[[nodiscard]] Vec3<T> add(const Vec3<T>& a, const Vec3<T>& b) {
    return vec3(a(0) + b(0), a(1) + b(1), a(2) + b(2));
}

template <typename T>
[[nodiscard]] Vec3<T> sub(const Vec3<T>& a, const Vec3<T>& b) {
    return vec3(a(0) - b(0), a(1) - b(1), a(2) - b(2));
}

template <typename T>
[[nodiscard]] Vec3<T> scale(T s, const Vec3<T>& a) {
    return vec3(s * a(0), s * a(1), s * a(2));
}

/// a + s·b
template <typename T>
[[nodiscard]] Vec3<T> axpy(const Vec3<T>& a, T s, const Vec3<T>& b) {
    return vec3(a(0) + s * b(0), a(1) + s * b(1), a(2) + s * b(2));
}

/// Element-wise product.
template <typename T>
[[nodiscard]] Vec3<T> hadamard(const Vec3<T>& a, const Vec3<T>& b) {
    return vec3(a(0) * b(0), a(1) * b(1), a(2) * b(2));
}

template <typename T>
[[nodiscard]] T dot(const Vec3<T>& a, const Vec3<T>& b) {
    return a(0) * b(0) + a(1) * b(1) + a(2) * b(2);
}

template <typename T>
[[nodiscard]] Vec3<T> cross(const Vec3<T>& a, const Vec3<T>& b) {
    return vec3(a(1) * b(2) - a(2) * b(1), a(2) * b(0) - a(0) * b(2), a(0) * b(1) - a(1) * b(0));
}

template <typename T>
[[nodiscard]] T norm(const Vec3<T>& a) {
    using std::sqrt;
    return sqrt(dot(a, a));
}

/// M · v
template <typename T>
[[nodiscard]] Vec3<T> mul(const Mat3<T>& m, const Vec3<T>& v) {
    return vec3(m(0, 0) * v(0) + m(0, 1) * v(1) + m(0, 2) * v(2),
                m(1, 0) * v(0) + m(1, 1) * v(1) + m(1, 2) * v(2),
                m(2, 0) * v(0) + m(2, 1) * v(1) + m(2, 2) * v(2));
}

/// Mᵀ · v
template <typename T>
[[nodiscard]] Vec3<T> mul_transpose(const Mat3<T>& m, const Vec3<T>& v) {
    return vec3(m(0, 0) * v(0) + m(1, 0) * v(1) + m(2, 0) * v(2),
                m(0, 1) * v(0) + m(1, 1) * v(1) + m(2, 1) * v(2),
                m(0, 2) * v(0) + m(1, 2) * v(1) + m(2, 2) * v(2));
}

}  // namespace branes::reflex
