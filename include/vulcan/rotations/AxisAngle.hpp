// Vulcan Axis-Angle Utilities
// Axis-angle and rotation vector conversions using Rodrigues' formula
#pragma once

#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/rotations/DCMUtils.hpp>

#include <janus/math/Arithmetic.hpp>
#include <janus/math/Linalg.hpp>
#include <janus/math/Logic.hpp>
#include <janus/math/Quaternion.hpp>
#include <janus/math/Trig.hpp>

#include <utility>

using vulcan::units::rad;

namespace vulcan {

// =============================================================================
// Quaternion from Axis-Angle
// =============================================================================

/// Create quaternion from axis-angle representation
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param axis Rotation axis (should be unit length, will be normalized)
/// @param angle Rotation angle [rad]
/// @return Unit quaternion
template <typename Scalar>
janus::Quaternion<Scalar>
quaternion_from_axis_angle(const Vec3<Scalar> &axis,
                           Quantity<rad, Scalar> angle) {
    return janus::Quaternion<Scalar>::from_axis_angle(axis, angle.value());
}

/// Create quaternion from rotation vector (axis × angle)
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param rot_vec Rotation vector [rad]
/// @return Unit quaternion
template <typename Scalar>
janus::Quaternion<Scalar>
quaternion_from_rotation_vector(const Vec3<Quantity<rad, Scalar>> &rot_vec) {
    // Unwrap Quantity components to raw Scalar
    Vec3<Scalar> raw;
    raw(0) = rot_vec(0).value();
    raw(1) = rot_vec(1).value();
    raw(2) = rot_vec(2).value();
    return janus::Quaternion<Scalar>::from_rotation_vector(raw);
}

// =============================================================================
// DCM from Axis-Angle (Rodrigues' Formula)
// =============================================================================

/// Create DCM from axis-angle using Rodrigues' rotation formula
///
/// R = I + sin(θ)[k]× + (1 - cos(θ))[k]×²
///
/// where k is the unit rotation axis and θ is the angle.
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param axis Rotation axis (will be normalized), dimensionless unit vector
/// @param angle Rotation angle [rad]
/// @return 3x3 rotation matrix
template <typename Scalar>
Mat3<Scalar> dcm_from_axis_angle(const Vec3<Scalar> &axis,
                                 Quantity<rad, Scalar> angle) {
    // Normalize axis (with small-angle protection)
    Scalar axis_norm = janus::norm(axis);
    Scalar eps = Scalar(1e-12);
    Scalar safe_norm = axis_norm + eps;

    Vec3<Scalar> k;
    k(0) = axis(0) / safe_norm;
    k(1) = axis(1) / safe_norm;
    k(2) = axis(2) / safe_norm;

    // Skew matrix of unit axis
    Mat3<Scalar> K = skew(k);
    Mat3<Scalar> K2 = K * K;

    // Rodrigues' formula
    Scalar a = angle.value();
    Scalar s = janus::sin(a);
    Scalar c = janus::cos(a);

    return Mat3<Scalar>::Identity() + s * K + (Scalar(1) - c) * K2;
}

/// Create DCM from rotation vector (axis × angle)
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param rot_vec Rotation vector [rad]
/// @return 3x3 rotation matrix
template <typename Scalar>
Mat3<Scalar>
dcm_from_rotation_vector(const Vec3<Quantity<rad, Scalar>> &rot_vec) {
    // Unwrap to raw Scalar for norm computation
    Vec3<Scalar> raw;
    raw(0) = rot_vec(0).value();
    raw(1) = rot_vec(1).value();
    raw(2) = rot_vec(2).value();

    Scalar angle = janus::norm(raw);
    Scalar eps = Scalar(1e-12);
    Scalar safe_angle = angle + eps;

    Vec3<Scalar> axis;
    axis(0) = raw(0) / safe_angle;
    axis(1) = raw(1) / safe_angle;
    axis(2) = raw(2) / safe_angle;

    return dcm_from_axis_angle(axis, Quantity<rad, Scalar>{angle});
}

// =============================================================================
// Axis-Angle from Quaternion
// =============================================================================

/// Extract axis and angle from quaternion
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param q Unit quaternion
/// @return Pair of (axis, angle) where axis is dimensionless unit vector
template <typename Scalar>
std::pair<Vec3<Scalar>, Quantity<rad, Scalar>>
axis_angle_from_quaternion(const janus::Quaternion<Scalar> &q) {
    // angle = 2 * acos(w)
    Scalar w = q.w;
    Scalar angle = Scalar(2) * janus::acos(w);

    // axis = [x, y, z] / sin(angle/2)
    Scalar sin_half = janus::sqrt(Scalar(1) - w * w);
    Scalar eps = Scalar(1e-12);
    Scalar safe_sin_half = sin_half + eps;

    Vec3<Scalar> axis;
    axis(0) = q.x / safe_sin_half;
    axis(1) = q.y / safe_sin_half;
    axis(2) = q.z / safe_sin_half;

    // For small angles, default to Z-axis (arbitrary but consistent)
    Scalar is_small = sin_half < eps;
    axis(0) = janus::where(is_small, Scalar(0), axis(0));
    axis(1) = janus::where(is_small, Scalar(0), axis(1));
    axis(2) = janus::where(is_small, Scalar(1), axis(2));

    return {axis, Quantity<rad, Scalar>{angle}};
}

/// Extract rotation vector from quaternion
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param q Unit quaternion
/// @return Rotation vector (axis × angle) [rad]
template <typename Scalar>
Vec3<Quantity<rad, Scalar>>
rotation_vector_from_quaternion(const janus::Quaternion<Scalar> &q) {
    auto [axis, angle] = axis_angle_from_quaternion(q);
    Scalar a = angle.value();
    Vec3<Quantity<rad, Scalar>> rot_vec;
    rot_vec(0) = Quantity<rad, Scalar>{axis(0) * a};
    rot_vec(1) = Quantity<rad, Scalar>{axis(1) * a};
    rot_vec(2) = Quantity<rad, Scalar>{axis(2) * a};
    return rot_vec;
}

// =============================================================================
// Axis-Angle from DCM
// =============================================================================

/// Extract axis and angle from DCM
///
/// Uses the trace relation: trace(R) = 1 + 2*cos(θ)
/// And the anti-symmetric part for axis: k = unskew(R - R^T) / (2*sin(θ))
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param R 3x3 rotation matrix
/// @return Pair of (axis, angle) where axis is dimensionless unit vector
template <typename Scalar>
std::pair<Vec3<Scalar>, Quantity<rad, Scalar>>
axis_angle_from_dcm(const Mat3<Scalar> &R) {
    // angle from trace: trace(R) = 1 + 2*cos(θ) => cos(θ) = (trace - 1) / 2
    Scalar trace = R.trace();
    Scalar cos_angle = (trace - Scalar(1)) * Scalar(0.5);

    // Clamp to [-1, 1] for numerical stability
    cos_angle = janus::where(cos_angle > Scalar(1), Scalar(1), cos_angle);
    cos_angle = janus::where(cos_angle < Scalar(-1), Scalar(-1), cos_angle);

    Scalar angle = janus::acos(cos_angle);

    // Axis from anti-symmetric part
    Vec3<Scalar> raw_axis;
    raw_axis(0) = R(2, 1) - R(1, 2);
    raw_axis(1) = R(0, 2) - R(2, 0);
    raw_axis(2) = R(1, 0) - R(0, 1);

    Scalar sin_angle = janus::sin(angle);
    Scalar eps = Scalar(1e-12);
    Scalar safe_sin = sin_angle + eps;

    Vec3<Scalar> axis;
    axis(0) = raw_axis(0) / (Scalar(2) * safe_sin);
    axis(1) = raw_axis(1) / (Scalar(2) * safe_sin);
    axis(2) = raw_axis(2) / (Scalar(2) * safe_sin);

    // Normalize axis
    Scalar axis_norm = janus::norm(axis);
    Scalar safe_norm = axis_norm + eps;
    axis(0) = axis(0) / safe_norm;
    axis(1) = axis(1) / safe_norm;
    axis(2) = axis(2) / safe_norm;

    // For small angles (angle ≈ 0), use small-angle approximation
    Scalar is_small = janus::abs(sin_angle) < eps;
    axis(0) = janus::where(is_small, Scalar(0), axis(0));
    axis(1) = janus::where(is_small, Scalar(0), axis(1));
    axis(2) = janus::where(is_small, Scalar(1), axis(2));

    return {axis, Quantity<rad, Scalar>{angle}};
}

/// Extract rotation vector from DCM
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param R 3x3 rotation matrix
/// @return Rotation vector (axis × angle) [rad]
template <typename Scalar>
Vec3<Quantity<rad, Scalar>> rotation_vector_from_dcm(const Mat3<Scalar> &R) {
    auto [axis, angle] = axis_angle_from_dcm(R);
    Scalar a = angle.value();
    Vec3<Quantity<rad, Scalar>> rot_vec;
    rot_vec(0) = Quantity<rad, Scalar>{axis(0) * a};
    rot_vec(1) = Quantity<rad, Scalar>{axis(1) * a};
    rot_vec(2) = Quantity<rad, Scalar>{axis(2) * a};
    return rot_vec;
}

} // namespace vulcan
