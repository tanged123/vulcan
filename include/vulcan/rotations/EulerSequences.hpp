// Vulcan Euler Angle Sequences
// All 12 Euler angle sequences with DCM and quaternion conversions
#pragma once

#include <vulcan/core/VulcanTypes.hpp>

#include <janus/math/Quaternion.hpp>
#include <janus/math/Rotations.hpp>

#include <array>

namespace vulcan {

// =============================================================================
// Euler Sequence Enumeration
// =============================================================================

/// All 12 Euler angle sequences
///
/// Naming convention: Axis labels in rotation order (intrinsic rotations)
/// - First letter: first rotation axis
/// - Second letter: second rotation axis (about rotated frame)
/// - Third letter: third rotation axis (about twice-rotated frame)
///
/// For example, ZYX means:
/// 1. Rotate about Z (yaw)
/// 2. Rotate about new Y (pitch)
/// 3. Rotate about new X (roll)
///
/// Tait-Bryan sequences use 3 distinct axes.
/// Proper Euler sequences reuse the first axis as the third.
enum class EulerSequence {
    // Tait-Bryan (3 distinct axes)
    XYZ, ///< Roll-Pitch-Yaw (robotics convention)
    XZY,
    YXZ,
    YZX,
    ZXY,
    ZYX, ///< Yaw-Pitch-Roll (aerospace standard)

    // Proper Euler (symmetric axes)
    XYX,
    XZX,
    YXY,
    YZY,
    ZXZ, ///< Classical mechanics (precession-nutation-spin)
    ZYZ
};

// =============================================================================
// Sequence Utilities
// =============================================================================

/// Get axis indices for a sequence [0=X, 1=Y, 2=Z]
///
/// @param seq Euler sequence
/// @return Array of 3 axis indices
constexpr std::array<int, 3> euler_axes(EulerSequence seq) {
    switch (seq) {
    case EulerSequence::XYZ:
        return {0, 1, 2};
    case EulerSequence::XZY:
        return {0, 2, 1};
    case EulerSequence::YXZ:
        return {1, 0, 2};
    case EulerSequence::YZX:
        return {1, 2, 0};
    case EulerSequence::ZXY:
        return {2, 0, 1};
    case EulerSequence::ZYX:
        return {2, 1, 0};
    case EulerSequence::XYX:
        return {0, 1, 0};
    case EulerSequence::XZX:
        return {0, 2, 0};
    case EulerSequence::YXY:
        return {1, 0, 1};
    case EulerSequence::YZY:
        return {1, 2, 1};
    case EulerSequence::ZXZ:
        return {2, 0, 2};
    case EulerSequence::ZYZ:
        return {2, 1, 2};
    default:
        return {2, 1, 0}; // Default to ZYX
    }
}

/// Check if sequence is proper Euler (symmetric: first axis == third axis)
constexpr bool is_proper_euler(EulerSequence seq) {
    auto axes = euler_axes(seq);
    return axes[0] == axes[2];
}

/// Get sequence name as string (for debugging/logging)
constexpr const char *euler_sequence_name(EulerSequence seq) {
    switch (seq) {
    case EulerSequence::XYZ:
        return "XYZ";
    case EulerSequence::XZY:
        return "XZY";
    case EulerSequence::YXZ:
        return "YXZ";
    case EulerSequence::YZX:
        return "YZX";
    case EulerSequence::ZXY:
        return "ZXY";
    case EulerSequence::ZYX:
        return "ZYX";
    case EulerSequence::XYX:
        return "XYX";
    case EulerSequence::XZX:
        return "XZX";
    case EulerSequence::YXY:
        return "YXY";
    case EulerSequence::YZY:
        return "YZY";
    case EulerSequence::ZXZ:
        return "ZXZ";
    case EulerSequence::ZYZ:
        return "ZYZ";
    default:
        return "Unknown";
    }
}

// =============================================================================
// DCM from Euler Angles
// =============================================================================

/// Create DCM from Euler angles with specified sequence
///
/// The DCM transforms vectors from the rotated frame to the reference frame:
///   v_ref = DCM * v_rotated
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param e1 First rotation angle [rad]
/// @param e2 Second rotation angle [rad]
/// @param e3 Third rotation angle [rad]
/// @param seq Euler sequence
/// @return 3x3 rotation matrix (DCM)
template <typename Scalar>
Mat3<Scalar> dcm_from_euler(Scalar e1, Scalar e2, Scalar e3,
                            EulerSequence seq) {
    auto axes = euler_axes(seq);

    // R = R_axis[0](e1) * R_axis[1](e2) * R_axis[2](e3)
    // For intrinsic rotations, compose left-to-right
    // janus::rotation_matrix_3d gives R such that v' = R * v
    Mat3<Scalar> R1 = janus::rotation_matrix_3d(e1, axes[0]);
    Mat3<Scalar> R2 = janus::rotation_matrix_3d(e2, axes[1]);
    Mat3<Scalar> R3 = janus::rotation_matrix_3d(e3, axes[2]);

    return R1 * R2 * R3;
}

// =============================================================================
// Quaternion from Euler Angles
// =============================================================================

/// Create quaternion from Euler angles with specified sequence
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param e1 First rotation angle [rad]
/// @param e2 Second rotation angle [rad]
/// @param e3 Third rotation angle [rad]
/// @param seq Euler sequence
/// @return Unit quaternion representing the rotation
template <typename Scalar>
janus::Quaternion<Scalar> quaternion_from_euler(Scalar e1, Scalar e2, Scalar e3,
                                                EulerSequence seq) {
    // For ZYX, we can use the optimized Janus implementation
    if (seq == EulerSequence::ZYX) {
        // Janus from_euler takes (roll, pitch, yaw) for ZYX intrinsic
        return janus::Quaternion<Scalar>::from_euler(e3, e2, e1);
    }

    // For other sequences, build via DCM and convert
    Mat3<Scalar> R = dcm_from_euler(e1, e2, e3, seq);
    return janus::Quaternion<Scalar>::from_rotation_matrix(R);
}

// =============================================================================
// Euler Angles from DCM - Tait-Bryan Sequences
// =============================================================================

namespace detail {

/// Extract Euler angles from DCM for ZYX sequence (aerospace standard)
///
/// R = Rz(e1) * Ry(e2) * Rx(e3)  -- intrinsic ZYX
/// DCM elements:
///   R[2,0] = -sin(e2)
///   R[0,0] = cos(e1)*cos(e2), R[1,0] = sin(e1)*cos(e2)
///   R[2,1] = cos(e2)*sin(e3), R[2,2] = cos(e2)*cos(e3)
template <typename Scalar>
Vec3<Scalar> euler_from_dcm_zyx(const Mat3<Scalar> &R) {
    Scalar sin_e2 = -R(2, 0);
    Scalar e2 = janus::asin(sin_e2);

    Scalar eps = Scalar(1e-6);
    Scalar cos_e2 = janus::cos(e2);
    Scalar is_gimbal = janus::abs(cos_e2) < eps;

    // Normal case
    Scalar e1_normal = janus::atan2(R(1, 0), R(0, 0));
    Scalar e3_normal = janus::atan2(R(2, 1), R(2, 2));

    // Gimbal lock: set e3 = 0, compute e1 from other elements
    Scalar e3_gimbal = Scalar(0);
    Scalar e1_gimbal = janus::atan2(-R(0, 1), R(1, 1));

    Vec3<Scalar> euler;
    euler(0) = janus::where(is_gimbal, e1_gimbal, e1_normal);
    euler(1) = e2;
    euler(2) = janus::where(is_gimbal, e3_gimbal, e3_normal);

    return euler;
}

/// Extract Euler angles from DCM for XYZ sequence
///
/// R = Rx(e1) * Ry(e2) * Rz(e3)
/// DCM elements:
///   R[0,2] = sin(e2)
///   R[0,0] = cos(e2)*cos(e3), R[0,1] = -cos(e2)*sin(e3)
///   R[1,2] = -cos(e2)*sin(e1), R[2,2] = cos(e2)*cos(e1)
template <typename Scalar>
Vec3<Scalar> euler_from_dcm_xyz(const Mat3<Scalar> &R) {
    Scalar sin_e2 = R(0, 2);
    Scalar e2 = janus::asin(sin_e2);

    Scalar eps = Scalar(1e-6);
    Scalar cos_e2 = janus::cos(e2);
    Scalar is_gimbal = janus::abs(cos_e2) < eps;

    // Normal case
    Scalar e1_normal = janus::atan2(-R(1, 2), R(2, 2));
    Scalar e3_normal = janus::atan2(-R(0, 1), R(0, 0));

    // Gimbal lock: set e3 = 0
    Scalar e3_gimbal = Scalar(0);
    Scalar e1_gimbal = janus::atan2(R(2, 1), R(1, 1));

    Vec3<Scalar> euler;
    euler(0) = janus::where(is_gimbal, e1_gimbal, e1_normal);
    euler(1) = e2;
    euler(2) = janus::where(is_gimbal, e3_gimbal, e3_normal);

    return euler;
}

/// Extract Euler angles from DCM for XZY sequence
template <typename Scalar>
Vec3<Scalar> euler_from_dcm_xzy(const Mat3<Scalar> &R) {
    Scalar sin_e2 = -R(0, 1);
    Scalar e2 = janus::asin(sin_e2);

    Scalar eps = Scalar(1e-6);
    Scalar cos_e2 = janus::cos(e2);
    Scalar is_gimbal = janus::abs(cos_e2) < eps;

    Scalar e1_normal = janus::atan2(R(2, 1), R(1, 1));
    Scalar e3_normal = janus::atan2(R(0, 2), R(0, 0));

    Scalar e3_gimbal = Scalar(0);
    Scalar e1_gimbal = janus::atan2(-R(1, 2), R(2, 2));

    Vec3<Scalar> euler;
    euler(0) = janus::where(is_gimbal, e1_gimbal, e1_normal);
    euler(1) = e2;
    euler(2) = janus::where(is_gimbal, e3_gimbal, e3_normal);

    return euler;
}

/// Extract Euler angles from DCM for YXZ sequence
template <typename Scalar>
Vec3<Scalar> euler_from_dcm_yxz(const Mat3<Scalar> &R) {
    Scalar sin_e2 = -R(1, 2);
    Scalar e2 = janus::asin(sin_e2);

    Scalar eps = Scalar(1e-6);
    Scalar cos_e2 = janus::cos(e2);
    Scalar is_gimbal = janus::abs(cos_e2) < eps;

    Scalar e1_normal = janus::atan2(R(0, 2), R(2, 2));
    Scalar e3_normal = janus::atan2(R(1, 0), R(1, 1));

    Scalar e3_gimbal = Scalar(0);
    Scalar e1_gimbal = janus::atan2(-R(2, 0), R(0, 0));

    Vec3<Scalar> euler;
    euler(0) = janus::where(is_gimbal, e1_gimbal, e1_normal);
    euler(1) = e2;
    euler(2) = janus::where(is_gimbal, e3_gimbal, e3_normal);

    return euler;
}

/// Extract Euler angles from DCM for YZX sequence
template <typename Scalar>
Vec3<Scalar> euler_from_dcm_yzx(const Mat3<Scalar> &R) {
    Scalar sin_e2 = R(1, 0);
    Scalar e2 = janus::asin(sin_e2);

    Scalar eps = Scalar(1e-6);
    Scalar cos_e2 = janus::cos(e2);
    Scalar is_gimbal = janus::abs(cos_e2) < eps;

    Scalar e1_normal = janus::atan2(-R(2, 0), R(0, 0));
    Scalar e3_normal = janus::atan2(-R(1, 2), R(1, 1));

    Scalar e3_gimbal = Scalar(0);
    Scalar e1_gimbal = janus::atan2(R(0, 2), R(2, 2));

    Vec3<Scalar> euler;
    euler(0) = janus::where(is_gimbal, e1_gimbal, e1_normal);
    euler(1) = e2;
    euler(2) = janus::where(is_gimbal, e3_gimbal, e3_normal);

    return euler;
}

/// Extract Euler angles from DCM for ZXY sequence
template <typename Scalar>
Vec3<Scalar> euler_from_dcm_zxy(const Mat3<Scalar> &R) {
    Scalar sin_e2 = R(2, 1);
    Scalar e2 = janus::asin(sin_e2);

    Scalar eps = Scalar(1e-6);
    Scalar cos_e2 = janus::cos(e2);
    Scalar is_gimbal = janus::abs(cos_e2) < eps;

    Scalar e1_normal = janus::atan2(-R(0, 1), R(1, 1));
    Scalar e3_normal = janus::atan2(-R(2, 0), R(2, 2));

    Scalar e3_gimbal = Scalar(0);
    Scalar e1_gimbal = janus::atan2(R(1, 0), R(0, 0));

    Vec3<Scalar> euler;
    euler(0) = janus::where(is_gimbal, e1_gimbal, e1_normal);
    euler(1) = e2;
    euler(2) = janus::where(is_gimbal, e3_gimbal, e3_normal);

    return euler;
}

// =============================================================================
// Euler Angles from DCM - Proper Euler Sequences
// =============================================================================

/// Extract Euler angles from DCM for ZXZ sequence (classical mechanics)
///
/// R = Rz(e1) * Rx(e2) * Rz(e3)
/// Singularity at e2 = 0 or π (sin(e2) = 0)
template <typename Scalar>
Vec3<Scalar> euler_from_dcm_zxz(const Mat3<Scalar> &R) {
    Scalar cos_e2 = R(2, 2);
    Scalar e2 = janus::acos(cos_e2);

    Scalar eps = Scalar(1e-6);
    Scalar sin_e2 = janus::sin(e2);
    Scalar is_gimbal = janus::abs(sin_e2) < eps;

    // Normal case
    Scalar e1_normal = janus::atan2(R(0, 2), -R(1, 2));
    Scalar e3_normal = janus::atan2(R(2, 0), R(2, 1));

    // Gimbal lock: set e3 = 0
    Scalar e3_gimbal = Scalar(0);
    // At e2 = 0: e1 + e3 = atan2(-R[0,1], R[0,0])
    // At e2 = π: e1 - e3 = atan2(R[0,1], R[0,0])
    Scalar e1_gimbal = janus::atan2(-R(0, 1), R(0, 0));

    Vec3<Scalar> euler;
    euler(0) = janus::where(is_gimbal, e1_gimbal, e1_normal);
    euler(1) = e2;
    euler(2) = janus::where(is_gimbal, e3_gimbal, e3_normal);

    return euler;
}

/// Extract Euler angles from DCM for ZYZ sequence
template <typename Scalar>
Vec3<Scalar> euler_from_dcm_zyz(const Mat3<Scalar> &R) {
    Scalar cos_e2 = R(2, 2);
    Scalar e2 = janus::acos(cos_e2);

    Scalar eps = Scalar(1e-6);
    Scalar sin_e2 = janus::sin(e2);
    Scalar is_gimbal = janus::abs(sin_e2) < eps;

    Scalar e1_normal = janus::atan2(R(1, 2), R(0, 2));
    Scalar e3_normal = janus::atan2(R(2, 1), -R(2, 0));

    Scalar e3_gimbal = Scalar(0);
    Scalar e1_gimbal = janus::atan2(R(1, 0), R(0, 0));

    Vec3<Scalar> euler;
    euler(0) = janus::where(is_gimbal, e1_gimbal, e1_normal);
    euler(1) = e2;
    euler(2) = janus::where(is_gimbal, e3_gimbal, e3_normal);

    return euler;
}

/// Extract Euler angles from DCM for XYX sequence
template <typename Scalar>
Vec3<Scalar> euler_from_dcm_xyx(const Mat3<Scalar> &R) {
    Scalar cos_e2 = R(0, 0);
    Scalar e2 = janus::acos(cos_e2);

    Scalar eps = Scalar(1e-6);
    Scalar sin_e2 = janus::sin(e2);
    Scalar is_gimbal = janus::abs(sin_e2) < eps;

    Scalar e1_normal = janus::atan2(R(1, 0), -R(2, 0));
    Scalar e3_normal = janus::atan2(R(0, 1), R(0, 2));

    Scalar e3_gimbal = Scalar(0);
    Scalar e1_gimbal = janus::atan2(-R(1, 2), R(1, 1));

    Vec3<Scalar> euler;
    euler(0) = janus::where(is_gimbal, e1_gimbal, e1_normal);
    euler(1) = e2;
    euler(2) = janus::where(is_gimbal, e3_gimbal, e3_normal);

    return euler;
}

/// Extract Euler angles from DCM for XZX sequence
template <typename Scalar>
Vec3<Scalar> euler_from_dcm_xzx(const Mat3<Scalar> &R) {
    Scalar cos_e2 = R(0, 0);
    Scalar e2 = janus::acos(cos_e2);

    Scalar eps = Scalar(1e-6);
    Scalar sin_e2 = janus::sin(e2);
    Scalar is_gimbal = janus::abs(sin_e2) < eps;

    Scalar e1_normal = janus::atan2(R(2, 0), R(1, 0));
    Scalar e3_normal = janus::atan2(R(0, 2), -R(0, 1));

    Scalar e3_gimbal = Scalar(0);
    Scalar e1_gimbal = janus::atan2(R(2, 1), R(2, 2));

    Vec3<Scalar> euler;
    euler(0) = janus::where(is_gimbal, e1_gimbal, e1_normal);
    euler(1) = e2;
    euler(2) = janus::where(is_gimbal, e3_gimbal, e3_normal);

    return euler;
}

/// Extract Euler angles from DCM for YXY sequence
template <typename Scalar>
Vec3<Scalar> euler_from_dcm_yxy(const Mat3<Scalar> &R) {
    Scalar cos_e2 = R(1, 1);
    Scalar e2 = janus::acos(cos_e2);

    Scalar eps = Scalar(1e-6);
    Scalar sin_e2 = janus::sin(e2);
    Scalar is_gimbal = janus::abs(sin_e2) < eps;

    Scalar e1_normal = janus::atan2(R(0, 1), R(2, 1));
    Scalar e3_normal = janus::atan2(R(1, 0), -R(1, 2));

    Scalar e3_gimbal = Scalar(0);
    Scalar e1_gimbal = janus::atan2(R(0, 2), R(0, 0));

    Vec3<Scalar> euler;
    euler(0) = janus::where(is_gimbal, e1_gimbal, e1_normal);
    euler(1) = e2;
    euler(2) = janus::where(is_gimbal, e3_gimbal, e3_normal);

    return euler;
}

/// Extract Euler angles from DCM for YZY sequence
template <typename Scalar>
Vec3<Scalar> euler_from_dcm_yzy(const Mat3<Scalar> &R) {
    Scalar cos_e2 = R(1, 1);
    Scalar e2 = janus::acos(cos_e2);

    Scalar eps = Scalar(1e-6);
    Scalar sin_e2 = janus::sin(e2);
    Scalar is_gimbal = janus::abs(sin_e2) < eps;

    Scalar e1_normal = janus::atan2(R(2, 1), -R(0, 1));
    Scalar e3_normal = janus::atan2(R(1, 2), R(1, 0));

    Scalar e3_gimbal = Scalar(0);
    Scalar e1_gimbal = janus::atan2(-R(2, 0), R(2, 2));

    Vec3<Scalar> euler;
    euler(0) = janus::where(is_gimbal, e1_gimbal, e1_normal);
    euler(1) = e2;
    euler(2) = janus::where(is_gimbal, e3_gimbal, e3_normal);

    return euler;
}

} // namespace detail

// =============================================================================
// Euler Angles from DCM - Public Interface
// =============================================================================

/// Extract Euler angles from DCM for specified sequence
///
/// Handles gimbal lock for all sequences via janus::where for symbolic
/// compatibility. At singularities, the third angle is set to zero.
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param R 3x3 rotation matrix (DCM)
/// @param seq Euler sequence
/// @return [e1, e2, e3] angles [rad]
template <typename Scalar>
Vec3<Scalar> euler_from_dcm(const Mat3<Scalar> &R, EulerSequence seq) {
    switch (seq) {
    case EulerSequence::ZYX:
        return detail::euler_from_dcm_zyx(R);
    case EulerSequence::XYZ:
        return detail::euler_from_dcm_xyz(R);
    case EulerSequence::XZY:
        return detail::euler_from_dcm_xzy(R);
    case EulerSequence::YXZ:
        return detail::euler_from_dcm_yxz(R);
    case EulerSequence::YZX:
        return detail::euler_from_dcm_yzx(R);
    case EulerSequence::ZXY:
        return detail::euler_from_dcm_zxy(R);
    case EulerSequence::ZXZ:
        return detail::euler_from_dcm_zxz(R);
    case EulerSequence::ZYZ:
        return detail::euler_from_dcm_zyz(R);
    case EulerSequence::XYX:
        return detail::euler_from_dcm_xyx(R);
    case EulerSequence::XZX:
        return detail::euler_from_dcm_xzx(R);
    case EulerSequence::YXY:
        return detail::euler_from_dcm_yxy(R);
    case EulerSequence::YZY:
        return detail::euler_from_dcm_yzy(R);
    default:
        return detail::euler_from_dcm_zyx(R);
    }
}

/// Extract Euler angles from quaternion for specified sequence
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param q Unit quaternion
/// @param seq Euler sequence
/// @return [e1, e2, e3] angles [rad]
template <typename Scalar>
Vec3<Scalar> euler_from_quaternion(const janus::Quaternion<Scalar> &q,
                                   EulerSequence seq) {
    // For ZYX, we can use the optimized Janus implementation
    if (seq == EulerSequence::ZYX) {
        auto euler_rpy = q.to_euler(); // Returns [roll, pitch, yaw]
        Vec3<Scalar> euler;
        euler(0) = euler_rpy(2); // yaw (e1)
        euler(1) = euler_rpy(1); // pitch (e2)
        euler(2) = euler_rpy(0); // roll (e3)
        return euler;
    }

    // For other sequences, convert via DCM
    Mat3<Scalar> R = q.to_rotation_matrix();
    return euler_from_dcm(R, seq);
}

} // namespace vulcan
