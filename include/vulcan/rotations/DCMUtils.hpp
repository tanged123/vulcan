// Vulcan DCM Utilities
// Direction Cosine Matrix utilities: skew, composition, small-angle
// approximation
#pragma once

#include <vulcan/core/VulcanTypes.hpp>

#include <janus/math/Arithmetic.hpp>
#include <janus/math/Linalg.hpp>
#include <janus/math/Logic.hpp>
#include <janus/math/Rotations.hpp>
#include <janus/math/Trig.hpp>

namespace vulcan {

// =============================================================================
// Skew-Symmetric Matrix
// =============================================================================

/// Create skew-symmetric matrix from vector
///
/// The skew-symmetric matrix [v]× satisfies: [v]× * u = v × u
///
/// For v = [x, y, z]:
///         [  0  -z   y ]
/// [v]× =  [  z   0  -x ]
///         [ -y   x   0 ]
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param v 3D vector
/// @return 3x3 skew-symmetric matrix
template <typename Scalar> Mat3<Scalar> skew(const Vec3<Scalar> &v) {
    Mat3<Scalar> S;
    S(0, 0) = Scalar(0);
    S(0, 1) = -v(2);
    S(0, 2) = v(1);
    S(1, 0) = v(2);
    S(1, 1) = Scalar(0);
    S(1, 2) = -v(0);
    S(2, 0) = -v(1);
    S(2, 1) = v(0);
    S(2, 2) = Scalar(0);
    return S;
}

/// Extract vector from skew-symmetric matrix
///
/// Given [v]×, returns v = [S[2,1], S[0,2], S[1,0]]
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param S 3x3 skew-symmetric matrix
/// @return 3D vector
template <typename Scalar> Vec3<Scalar> unskew(const Mat3<Scalar> &S) {
    Vec3<Scalar> v;
    v(0) = S(2, 1);
    v(1) = S(0, 2);
    v(2) = S(1, 0);
    return v;
}

// =============================================================================
// DCM Composition
// =============================================================================

/// Compose two DCMs: R_combined = R2 * R1
///
/// Applies R1 first, then R2:
///   v_final = R2 * (R1 * v_initial) = (R2 * R1) * v_initial
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param R1 First rotation (applied first)
/// @param R2 Second rotation (applied second)
/// @return Combined rotation matrix
template <typename Scalar>
Mat3<Scalar> compose_dcm(const Mat3<Scalar> &R1, const Mat3<Scalar> &R2) {
    return R2 * R1;
}

/// Compute relative rotation DCM from frame A to frame B
///
/// R_rel transforms vectors from frame A to frame B:
///   v_B = R_rel * v_A
///
/// Given R_A (transforms from A to reference) and R_B (transforms from B to
/// reference):
///   R_rel = R_B^T * R_A
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param R_A DCM transforming from frame A to reference
/// @param R_B DCM transforming from frame B to reference
/// @return DCM transforming from frame A to frame B
template <typename Scalar>
Mat3<Scalar> relative_dcm(const Mat3<Scalar> &R_A, const Mat3<Scalar> &R_B) {
    return R_B.transpose() * R_A;
}

// =============================================================================
// Small-Angle Approximation
// =============================================================================

/// Create DCM from small-angle vector using first-order approximation
///
/// For small rotation angles θ = [θx, θy, θz], the DCM is approximately:
///   R ≈ I + [θ]×
///
/// This is valid when ||θ|| << 1 rad (typically < 0.1 rad for ~1% error).
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param theta Small rotation angles [rad]
/// @return Approximate DCM (not exactly orthonormal)
template <typename Scalar>
Mat3<Scalar> dcm_from_small_angle(const Vec3<Scalar> &theta) {
    return Mat3<Scalar>::Identity() + skew(theta);
}

/// Extract small-angle vector from near-identity DCM
///
/// For DCM R ≈ I + [θ]×:
///   θ ≈ unskew(R - I) = unskew(R - R^T) / 2
///
/// More robustly:
///   θ = [R[2,1] - R[1,2], R[0,2] - R[2,0], R[1,0] - R[0,1]] / 2
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param R Near-identity rotation matrix
/// @return Small rotation angles [rad]
template <typename Scalar>
Vec3<Scalar> small_angle_from_dcm(const Mat3<Scalar> &R) {
    Scalar half = Scalar(0.5);
    Vec3<Scalar> theta;
    theta(0) = half * (R(2, 1) - R(1, 2));
    theta(1) = half * (R(0, 2) - R(2, 0));
    theta(2) = half * (R(1, 0) - R(0, 1));
    return theta;
}

// =============================================================================
// DCM Validation
// =============================================================================

/// Check if matrix is a valid rotation matrix (re-export from Janus)
///
/// Verifies:
/// - Determinant ≈ +1
/// - R^T * R ≈ I (orthonormality)
///
/// @tparam Derived Eigen matrix type
/// @param R Input matrix
/// @param tol Tolerance for numerical checks
/// @return True if valid rotation matrix
template <typename Derived>
auto is_valid_dcm(const Eigen::MatrixBase<Derived> &R, double tol = 1e-9) {
    return janus::is_valid_rotation_matrix(R, tol);
}

// =============================================================================
// DCM from Principal Axis
// =============================================================================

/// Create DCM for rotation about a principal axis (re-export from Janus)
///
/// @tparam Scalar Scalar type
/// @param theta Rotation angle [rad]
/// @param axis Axis index: 0=X, 1=Y, 2=Z
/// @return 3x3 rotation matrix
template <typename Scalar>
Mat3<Scalar> dcm_principal_axis(Scalar theta, int axis) {
    return janus::rotation_matrix_3d(theta, axis);
}

} // namespace vulcan
