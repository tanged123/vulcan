// Vulcan Rotation Interpolation
// Slerp re-export and Squad interpolation for smooth quaternion curves
#pragma once

#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/rotations/AxisAngle.hpp>

#include <janus/math/Quaternion.hpp>

namespace vulcan {

// =============================================================================
// Basic Interpolation (Slerp)
// =============================================================================

/// Spherical linear interpolation between quaternions (re-export from Janus)
///
/// Computes smooth interpolation between two rotations.
/// Handles shortest path automatically.
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param q0 Start quaternion (t=0)
/// @param q1 End quaternion (t=1)
/// @param t Interpolation parameter [0, 1]
/// @return Interpolated quaternion
template <typename Scalar>
janus::Quaternion<Scalar> slerp(const janus::Quaternion<Scalar> &q0,
                                const janus::Quaternion<Scalar> &q1, Scalar t) {
    return janus::slerp(q0, q1, t);
}

// =============================================================================
// Quaternion Logarithm and Exponential
// =============================================================================

/// Quaternion exponential: exp(v) where v is a pure quaternion (0, v_xyz)
///
/// exp((0, v)) = (cos(||v||), sin(||v||) * v / ||v||)
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param v Pure quaternion vector part (rotation vector / 2)
/// @return Unit quaternion
template <typename Scalar>
janus::Quaternion<Scalar> quat_exp(const Vec3<Scalar> &v) {
    Scalar angle = janus::norm(v);
    Scalar eps = Scalar(1e-12);
    Scalar safe_angle = angle + eps;

    Scalar s = janus::sin(angle) / safe_angle;
    Scalar c = janus::cos(angle);

    // For small angles, use Taylor expansion: sin(x)/x ≈ 1
    Scalar is_small = angle < eps;
    s = janus::where(is_small, Scalar(1), s);
    c = janus::where(is_small, Scalar(1), c);

    return janus::Quaternion<Scalar>(c, s * v(0), s * v(1), s * v(2));
}

/// Quaternion logarithm: log(q) returns pure quaternion (0, v_xyz)
///
/// For unit quaternion q = (w, x, y, z):
///   log(q) = (0, acos(w) * [x,y,z] / ||[x,y,z]||)
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param q Unit quaternion
/// @return Pure quaternion vector part (rotation vector / 2)
template <typename Scalar>
Vec3<Scalar> quat_log(const janus::Quaternion<Scalar> &q) {
    Scalar w = q.w;
    Vec3<Scalar> v;
    v(0) = q.x;
    v(1) = q.y;
    v(2) = q.z;

    Scalar v_norm = janus::norm(v);
    Scalar eps = Scalar(1e-12);
    Scalar safe_norm = v_norm + eps;

    Scalar angle = janus::acos(w);
    Scalar scale = angle / safe_norm;

    // For small angles, scale ≈ 1
    Scalar is_small = v_norm < eps;
    scale = janus::where(is_small, Scalar(1), scale);

    v(0) = scale * v(0);
    v(1) = scale * v(1);
    v(2) = scale * v(2);

    // For identity quaternion, return zero
    v(0) = janus::where(is_small, Scalar(0), v(0));
    v(1) = janus::where(is_small, Scalar(0), v(1));
    v(2) = janus::where(is_small, Scalar(0), v(2));

    return v;
}

// =============================================================================
// Squad (Spherical Quadrangle Interpolation)
// =============================================================================

/// Compute Squad control point for smooth interpolation through waypoints
///
/// Given quaternions q_{i-1}, q_i, q_{i+1}, computes the control point s_i
/// that produces C1 continuity in orientation.
///
/// s_i = q_i * exp(-0.25 * (log(q_i^(-1) * q_{i+1}) + log(q_i^(-1) * q_{i-1})))
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param q_prev Previous quaternion (q_{i-1})
/// @param q_curr Current quaternion (q_i)
/// @param q_next Next quaternion (q_{i+1})
/// @return Control point s_i
template <typename Scalar>
janus::Quaternion<Scalar>
squad_control_point(const janus::Quaternion<Scalar> &q_prev,
                    const janus::Quaternion<Scalar> &q_curr,
                    const janus::Quaternion<Scalar> &q_next) {
    auto q_curr_inv = q_curr.conjugate();

    // Compute log(q_i^(-1) * q_{i+1}) and log(q_i^(-1) * q_{i-1})
    auto log_next = quat_log(q_curr_inv * q_next);
    auto log_prev = quat_log(q_curr_inv * q_prev);

    // Average and negate
    Vec3<Scalar> avg;
    avg(0) = -Scalar(0.25) * (log_next(0) + log_prev(0));
    avg(1) = -Scalar(0.25) * (log_next(1) + log_prev(1));
    avg(2) = -Scalar(0.25) * (log_next(2) + log_prev(2));

    return q_curr * quat_exp(avg);
}

/// Squad (Spherical Quadrangle) interpolation for smooth quaternion curves
///
/// Provides C1 continuous interpolation between quaternion waypoints.
/// Uses control points computed by squad_control_point.
///
/// squad(q_i, q_{i+1}, s_i, s_{i+1}, t) =
///     slerp(slerp(q_i, q_{i+1}, t), slerp(s_i, s_{i+1}, t), 2t(1-t))
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param q0 Start quaternion
/// @param q1 End quaternion
/// @param s0 Start control point (from squad_control_point)
/// @param s1 End control point (from squad_control_point)
/// @param t Interpolation parameter [0, 1]
/// @return Interpolated quaternion with C1 continuity
template <typename Scalar>
janus::Quaternion<Scalar> squad(const janus::Quaternion<Scalar> &q0,
                                const janus::Quaternion<Scalar> &q1,
                                const janus::Quaternion<Scalar> &s0,
                                const janus::Quaternion<Scalar> &s1, Scalar t) {
    auto q_slerp = slerp(q0, q1, t);
    auto s_slerp = slerp(s0, s1, t);

    // Blend parameter: 2t(1-t) peaks at t=0.5
    Scalar h = Scalar(2) * t * (Scalar(1) - t);

    return slerp(q_slerp, s_slerp, h);
}

} // namespace vulcan
