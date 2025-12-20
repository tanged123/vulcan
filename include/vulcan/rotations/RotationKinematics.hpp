// Vulcan Rotation Kinematics
// Angular velocity and rotation rate relationships
#pragma once

#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/rotations/AxisAngle.hpp>
#include <vulcan/rotations/DCMUtils.hpp>

#include <janus/math/Quaternion.hpp>

namespace vulcan {

// =============================================================================
// Quaternion Rate <-> Angular Velocity
// =============================================================================

/// Compute angular velocity from quaternion and its time derivative
///
/// Given quaternion q and its time derivative q_dot, computes:
///   omega_body = 2 * q.conjugate() * q_dot
///
/// The result is the angular velocity in the local (body) frame.
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
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
    omega(0) = two * omega_quat.x;
    omega(1) = two * omega_quat.y;
    omega(2) = two * omega_quat.z;
    return omega;
}

/// Compute quaternion rate from angular velocity
///
/// Given quaternion q and angular velocity omega (in body frame), computes:
///   q_dot = 0.5 * q * omega_quat
///
/// where omega_quat = (0, omega_x, omega_y, omega_z)
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param q Current orientation quaternion
/// @param omega_body Angular velocity in body frame [rad/s]
/// @return Time derivative of quaternion
template <typename Scalar>
janus::Quaternion<Scalar>
quaternion_rate_from_omega(const janus::Quaternion<Scalar> &q,
                           const Vec3<Scalar> &omega_body) {
    // q_dot = 0.5 * q * (0, omega)
    janus::Quaternion<Scalar> omega_quat(Scalar(0), omega_body(0),
                                         omega_body(1), omega_body(2));
    Scalar half = Scalar(0.5);
    auto q_dot = q * omega_quat;
    return janus::Quaternion<Scalar>(half * q_dot.w, half * q_dot.x,
                                     half * q_dot.y, half * q_dot.z);
}

// =============================================================================
// DCM Rate <-> Angular Velocity
// =============================================================================

/// Compute angular velocity from DCM rate
///
/// Given DCM R and its time derivative R_dot:
///   [omega]× = R^T * R_dot
///   omega = unskew(R^T * R_dot)
///
/// The result is the angular velocity in the body frame.
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param R Current rotation matrix
/// @param R_dot Time derivative of rotation matrix
/// @return Angular velocity vector in body frame [rad/s]
template <typename Scalar>
Vec3<Scalar> omega_from_dcm_rate(const Mat3<Scalar> &R,
                                 const Mat3<Scalar> &R_dot) {
    Mat3<Scalar> omega_skew = R.transpose() * R_dot;
    return unskew(omega_skew);
}

/// Compute DCM rate from angular velocity
///
/// Given DCM R and body-frame angular velocity omega:
///   R_dot = R * [omega]×
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param R Current rotation matrix
/// @param omega_body Angular velocity in body frame [rad/s]
/// @return Time derivative of rotation matrix
template <typename Scalar>
Mat3<Scalar> dcm_rate_from_omega(const Mat3<Scalar> &R,
                                 const Vec3<Scalar> &omega_body) {
    return R * skew(omega_body);
}

// =============================================================================
// Rotation Composition
// =============================================================================

/// Compose quaternion rotations: result = q2 * q1
///
/// Applies q1 first, then q2:
///   v_final = q2.rotate(q1.rotate(v_initial))
///           = (q2 * q1).rotate(v_initial)
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
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
/// Given quaternions representing rotations from a common reference frame,
/// computes the rotation needed to go from orientation A to orientation B:
///   q_rel = q_to * q_from.conjugate()
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param q_from Source orientation quaternion
/// @param q_to Target orientation quaternion
/// @return Relative rotation from q_from to q_to
template <typename Scalar>
janus::Quaternion<Scalar>
relative_rotation(const janus::Quaternion<Scalar> &q_from,
                  const janus::Quaternion<Scalar> &q_to) {
    return q_to * q_from.conjugate();
}

/// Compute rotation error as a rotation vector
///
/// Returns the rotation vector (axis × angle) representing the rotation
/// from q_actual to q_desired. Useful for attitude control:
///   error = rotation_error(q_actual, q_desired)
///   torque = K * error  (for P-control)
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param q_actual Current orientation
/// @param q_desired Desired orientation
/// @return Rotation vector representing error [rad]
template <typename Scalar>
Vec3<Scalar> rotation_error(const janus::Quaternion<Scalar> &q_actual,
                            const janus::Quaternion<Scalar> &q_desired) {
    // q_error = q_desired * q_actual^(-1)
    auto q_error = q_desired * q_actual.conjugate();
    return rotation_vector_from_quaternion(q_error);
}

} // namespace vulcan
