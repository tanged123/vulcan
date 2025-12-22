// Estimation Utilities - Helper functions for Kalman filtering
// Part of Vulcan - Aerospace Engineering Utilities
#pragma once

#include <Eigen/Dense>
#include <janus/janus.hpp>

namespace vulcan::estimation {

// =============================================================================
// Innovation Statistics
// =============================================================================

/**
 * @brief Compute normalized innovation squared (NIS)
 *
 * NIS = yᵀ * S⁻¹ * y
 *
 * Used for filter consistency checking. For a properly tuned filter,
 * NIS should follow a chi-squared distribution with M degrees of freedom.
 *
 * @tparam Scalar Numeric type
 * @tparam M Innovation (measurement residual) dimension
 * @param innovation Measurement residual y = z - z_pred [M x 1]
 * @param S Innovation covariance [M x M]
 * @return Normalized innovation squared (scalar)
 */
template <typename Scalar, int M>
Scalar
normalized_innovation_squared(const Eigen::Matrix<Scalar, M, 1> &innovation,
                              const Eigen::Matrix<Scalar, M, M> &S) {
    return (innovation.transpose() * S.inverse() * innovation)(0, 0);
}

// =============================================================================
// Measurement Gating
// =============================================================================

/**
 * @brief Chi-squared gating for outlier rejection
 *
 * Returns true if the innovation passes the gate (is consistent with filter).
 * A measurement fails the gate if NIS > threshold.
 *
 * Common thresholds (from chi-squared distribution):
 *   M=1: 3.84 (95%), 6.63 (99%)
 *   M=2: 5.99 (95%), 9.21 (99%)
 *   M=3: 7.81 (95%), 11.34 (99%)
 *
 * @tparam Scalar Numeric type
 * @tparam M Innovation dimension
 * @param innovation Measurement residual [M x 1]
 * @param S Innovation covariance [M x M]
 * @param threshold Chi-squared threshold for gating
 * @return true if NIS <= threshold (measurement passes gate)
 */
template <typename Scalar, int M>
bool passes_gate(const Eigen::Matrix<Scalar, M, 1> &innovation,
                 const Eigen::Matrix<Scalar, M, M> &S, double threshold) {
    Scalar nis = normalized_innovation_squared<Scalar, M>(innovation, S);
    return static_cast<double>(nis) <= threshold;
}

/**
 * @brief Compute innovation and check gate in one call
 *
 * Convenience function that computes innovation from predicted and actual
 * measurements, then checks the gate.
 *
 * @tparam Scalar Numeric type
 * @tparam M Measurement dimension
 * @param z Actual measurement [M x 1]
 * @param z_pred Predicted measurement [M x 1]
 * @param S Innovation covariance [M x M]
 * @param threshold Chi-squared threshold
 * @return true if measurement passes gate
 */
template <typename Scalar, int M>
bool measurement_passes_gate(const Eigen::Matrix<Scalar, M, 1> &z,
                             const Eigen::Matrix<Scalar, M, 1> &z_pred,
                             const Eigen::Matrix<Scalar, M, M> &S,
                             double threshold) {
    Eigen::Matrix<Scalar, M, 1> innovation = z - z_pred;
    return passes_gate<Scalar, M>(innovation, S, threshold);
}

// =============================================================================
// Joseph Form Covariance Update
// =============================================================================

/**
 * @brief Joseph form covariance update
 *
 * Numerically stable covariance update that guarantees positive definiteness:
 *   P = (I - K*H)*P⁻*(I - K*H)ᵀ + K*R*Kᵀ
 *
 * This is equivalent to the standard update P = (I - K*H)*P⁻ but is
 * more robust to numerical errors and always produces a symmetric,
 * positive semi-definite result.
 *
 * @tparam N State dimension
 * @tparam M Measurement dimension
 * @param P_prior Prior covariance P⁻ [N x N]
 * @param K Kalman gain [N x M]
 * @param H Measurement matrix [M x N]
 * @param R Measurement noise covariance [M x M]
 * @return Updated covariance P [N x N]
 */
template <int N, int M>
Eigen::Matrix<double, N, N>
joseph_update(const Eigen::Matrix<double, N, N> &P_prior,
              const Eigen::Matrix<double, N, M> &K,
              const Eigen::Matrix<double, M, N> &H,
              const Eigen::Matrix<double, M, M> &R) {
    Eigen::Matrix<double, N, N> I = Eigen::Matrix<double, N, N>::Identity();
    Eigen::Matrix<double, N, N> IKH = I - K * H;

    // Joseph form: P = (I-KH)*P*(I-KH)ᵀ + K*R*Kᵀ
    return IKH * P_prior * IKH.transpose() + K * R * K.transpose();
}

// =============================================================================
// Common Chi-Squared Thresholds
// =============================================================================

/**
 * @brief Chi-squared threshold lookup
 *
 * Returns the chi-squared critical value for the given degrees of freedom
 * and confidence level.
 *
 * @param dof Degrees of freedom (measurement dimension)
 * @param confidence Confidence level (0.95 or 0.99)
 * @return Chi-squared threshold
 */
inline double chi_squared_threshold(int dof, double confidence = 0.95) {
    // Pre-computed values for common cases
    if (confidence >= 0.99) {
        switch (dof) {
        case 1:
            return 6.635;
        case 2:
            return 9.210;
        case 3:
            return 11.345;
        case 4:
            return 13.277;
        case 5:
            return 15.086;
        case 6:
            return 16.812;
        default:
            // Approximation for larger dof at 99%
            return dof + 2.33 * std::sqrt(2.0 * dof);
        }
    } else {
        // 95% confidence
        switch (dof) {
        case 1:
            return 3.841;
        case 2:
            return 5.991;
        case 3:
            return 7.815;
        case 4:
            return 9.488;
        case 5:
            return 11.070;
        case 6:
            return 12.592;
        default:
            // Approximation for larger dof at 95%
            return dof + 1.645 * std::sqrt(2.0 * dof);
        }
    }
}

// =============================================================================
// Covariance Symmetry Enforcement
// =============================================================================

/**
 * @brief Enforce covariance matrix symmetry
 *
 * Forces a covariance matrix to be exactly symmetric by averaging
 * with its transpose. Useful after numerical operations that may
 * introduce small asymmetries.
 *
 * @tparam N Matrix dimension
 * @param P Covariance matrix [N x N]
 * @return Symmetrized covariance (P + Pᵀ)/2
 */
template <int N>
Eigen::Matrix<double, N, N> symmetrize(const Eigen::Matrix<double, N, N> &P) {
    return 0.5 * (P + P.transpose());
}

} // namespace vulcan::estimation
