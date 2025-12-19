// Vulcan Quaternion Utilities
// Convenience functions for quaternion-based coordinate transformations
#pragma once

#include <vulcan/core/VulcanTypes.hpp>

#include <janus/math/Quaternion.hpp>

namespace vulcan {

// =============================================================================
// Quaternion Composition
// =============================================================================

/// Compose two rotations: result = q2 * q1 (apply q1 first, then q2)
///
/// @tparam Scalar Scalar type
/// @param q1 First rotation (applied first)
/// @param q2 Second rotation (applied second)
/// @return Composed rotation quaternion
template <typename Scalar>
janus::Quaternion<Scalar>
compose_rotations(const janus::Quaternion<Scalar> &q1,
                  const janus::Quaternion<Scalar> &q2) {
    return q2 * q1;
}

/// Compute relative rotation from frame A to frame B
///
/// Given two quaternions representing rotations from a common reference frame,
/// computes the rotation needed to go from orientation A to orientation B:
///   q_rel = q_to * q_from.conjugate()
///
/// @tparam Scalar Scalar type
/// @param q_from Source orientation quaternion
/// @param q_to Target orientation quaternion
/// @return Relative rotation from q_from to q_to
template <typename Scalar>
janus::Quaternion<Scalar>
relative_rotation(const janus::Quaternion<Scalar> &q_from,
                  const janus::Quaternion<Scalar> &q_to) {
    return q_to * q_from.conjugate();
}

// =============================================================================
// Interpolation (Re-export)
// =============================================================================

/// Spherical linear interpolation between quaternions
///
/// Computes smooth interpolation between two rotations.
/// Handles shortest path automatically.
///
/// @tparam Scalar Scalar type
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
// Angular Velocity from Quaternion Rate
// =============================================================================

/// Compute angular velocity from quaternion and its time derivative
///
/// Given quaternion q and its time derivative q_dot, computes:
///   omega = 2 * q.conjugate() * q_dot
///
/// The result is the angular velocity in the local (body) frame.
///
/// @tparam Scalar Scalar type
/// @param q Current orientation quaternion
/// @param q_dot Time derivative of quaternion
/// @return Angular velocity vector in body frame [rad/s]
template <typename Scalar>
Vec3<Scalar>
omega_from_quaternion_rate(const janus::Quaternion<Scalar> &q,
                           const janus::Quaternion<Scalar> &q_dot) {
    // omega_body = 2 * q* * q_dot
    auto omega_quat = q.conjugate() * q_dot;
    Scalar two = Scalar(2);

    Vec3<Scalar> omega;
    omega << two * omega_quat.x, two * omega_quat.y, two * omega_quat.z;
    return omega;
}

/// Compute quaternion rate from angular velocity
///
/// Given quaternion q and angular velocity omega (in body frame), computes:
///   q_dot = 0.5 * q * omega_quat
///
/// where omega_quat = (0, omega_x, omega_y, omega_z)
///
/// @tparam Scalar Scalar type
/// @param q Current orientation quaternion
/// @param omega Angular velocity in body frame [rad/s]
/// @return Time derivative of quaternion
template <typename Scalar>
janus::Quaternion<Scalar>
quaternion_rate_from_omega(const janus::Quaternion<Scalar> &q,
                           const Vec3<Scalar> &omega) {
    // q_dot = 0.5 * q * (0, omega)
    janus::Quaternion<Scalar> omega_quat(Scalar(0), omega(0), omega(1),
                                         omega(2));
    Scalar half = Scalar(0.5);
    auto q_dot = q * omega_quat;
    return janus::Quaternion<Scalar>(half * q_dot.w, half * q_dot.x,
                                     half * q_dot.y, half * q_dot.z);
}

// =============================================================================
// Convenience Constructors
// =============================================================================

/// Create quaternion from axis-angle representation
///
/// @tparam Scalar Scalar type
/// @param axis Rotation axis (should be unit length)
/// @param angle Rotation angle [rad]
/// @return Rotation quaternion
template <typename Scalar>
janus::Quaternion<Scalar> quaternion_from_axis_angle(const Vec3<Scalar> &axis,
                                                     Scalar angle) {
    return janus::Quaternion<Scalar>::from_axis_angle(axis, angle);
}

/// Create quaternion from rotation vector (axis * angle)
///
/// @tparam Scalar Scalar type
/// @param rot_vec Rotation vector [rad]
/// @return Rotation quaternion
template <typename Scalar>
janus::Quaternion<Scalar>
quaternion_from_rotation_vector(const Vec3<Scalar> &rot_vec) {
    return janus::Quaternion<Scalar>::from_rotation_vector(rot_vec);
}

/// Extract rotation vector from quaternion
///
/// @tparam Scalar Scalar type
/// @param q Rotation quaternion
/// @return Rotation vector (axis * angle) [rad]
template <typename Scalar>
Vec3<Scalar>
rotation_vector_from_quaternion(const janus::Quaternion<Scalar> &q) {
    // angle = 2 * acos(w), axis = [x,y,z] / sin(angle/2)
    Scalar w = q.w;
    Scalar angle = Scalar(2) * janus::acos(w);

    // Handle identity quaternion
    Scalar eps = Scalar(1e-10);
    Scalar sin_half = janus::sqrt(Scalar(1) - w * w);
    Scalar is_small = sin_half < eps;

    Vec3<Scalar> axis;
    axis << q.x / sin_half, q.y / sin_half, q.z / sin_half;

    // For small angles, return [0,0,0]
    axis(0) = janus::where(is_small, Scalar(0), axis(0) * angle);
    axis(1) = janus::where(is_small, Scalar(0), axis(1) * angle);
    axis(2) = janus::where(is_small, Scalar(0), axis(2) * angle);

    return axis;
}

} // namespace vulcan
