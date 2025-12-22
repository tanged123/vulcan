// Kalman Filter - Linear Kalman filter predict/update utilities
// Part of Vulcan - Aerospace Engineering Utilities
#pragma once

#include <vulcan/estimation/EstimationTypes.hpp>

namespace vulcan::estimation {

// =============================================================================
// Linear Kalman Filter Predict
// =============================================================================

/**
 * @brief Linear Kalman filter predict step
 *
 * Propagates state and covariance through linear dynamics:
 *   x̂⁻ = F*x̂ + B*u
 *   P⁻ = F*P*Fᵀ + Q
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 * @tparam N State dimension
 * @tparam U Control input dimension
 * @param state Current filter state (x, P)
 * @param F State transition matrix [N x N]
 * @param B Control input matrix [N x U]
 * @param u Control input vector [U x 1]
 * @param noise Process noise specification (Q)
 * @return Predicted filter state (x⁻, P⁻)
 */
template <typename Scalar, int N, int U>
FilterState<Scalar, N> kf_predict(const FilterState<Scalar, N> &state,
                                  const Eigen::Matrix<double, N, N> &F,
                                  const Eigen::Matrix<double, N, U> &B,
                                  const Eigen::Matrix<Scalar, U, 1> &u,
                                  const ProcessNoise<N> &noise) {
    FilterState<Scalar, N> predicted;

    // State prediction: x⁻ = F*x + B*u
    predicted.x =
        F.template cast<Scalar>() * state.x + B.template cast<Scalar>() * u;

    // Covariance prediction: P⁻ = F*P*Fᵀ + Q
    auto F_scalar = F.template cast<Scalar>();
    predicted.P = F_scalar * state.P * F_scalar.transpose() +
                  noise.Q.template cast<Scalar>();

    return predicted;
}

/**
 * @brief Linear Kalman filter predict (no control input)
 *
 * Simplified predict for autonomous systems:
 *   x̂⁻ = F*x̂
 *   P⁻ = F*P*Fᵀ + Q
 *
 * @tparam Scalar Numeric type
 * @tparam N State dimension
 */
template <typename Scalar, int N>
FilterState<Scalar, N> kf_predict(const FilterState<Scalar, N> &state,
                                  const Eigen::Matrix<double, N, N> &F,
                                  const ProcessNoise<N> &noise) {
    FilterState<Scalar, N> predicted;

    // State prediction: x⁻ = F*x
    predicted.x = F.template cast<Scalar>() * state.x;

    // Covariance prediction: P⁻ = F*P*Fᵀ + Q
    auto F_scalar = F.template cast<Scalar>();
    predicted.P = F_scalar * state.P * F_scalar.transpose() +
                  noise.Q.template cast<Scalar>();

    return predicted;
}

// =============================================================================
// Linear Kalman Filter Update
// =============================================================================

/**
 * @brief Linear Kalman filter update step
 *
 * Incorporates measurement into state estimate:
 *   S = H*P⁻*Hᵀ + R           (innovation covariance)
 *   K = P⁻*Hᵀ*S⁻¹             (Kalman gain)
 *   x̂ = x̂⁻ + K*(z - H*x̂⁻)     (state update)
 *   P = (I - K*H)*P⁻          (covariance update)
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 * @tparam N State dimension
 * @tparam M Measurement dimension
 * @param state Predicted filter state (x⁻, P⁻)
 * @param z Measurement vector [M x 1]
 * @param H Measurement matrix [M x N]
 * @param noise Measurement noise specification (R)
 * @return Updated filter state (x̂, P)
 */
template <typename Scalar, int N, int M>
FilterState<Scalar, N> kf_update(const FilterState<Scalar, N> &state,
                                 const Eigen::Matrix<Scalar, M, 1> &z,
                                 const Eigen::Matrix<double, M, N> &H,
                                 const MeasurementNoise<M> &noise) {
    // Cast matrices to Scalar type for mixed operations
    auto H_scalar = H.template cast<Scalar>();
    auto R_scalar = noise.R.template cast<Scalar>();

    // Innovation covariance: S = H*P*Hᵀ + R
    Eigen::Matrix<Scalar, M, M> S =
        H_scalar * state.P * H_scalar.transpose() + R_scalar;

    // Kalman gain: K = P*Hᵀ*S⁻¹
    Eigen::Matrix<Scalar, N, M> K =
        state.P * H_scalar.transpose() * S.inverse();

    // Innovation (measurement residual): y = z - H*x
    Eigen::Matrix<Scalar, M, 1> innovation = z - H_scalar * state.x;

    // State update: x̂ = x⁻ + K*y
    FilterState<Scalar, N> updated;
    updated.x = state.x + K * innovation;

    // Covariance update: P = (I - K*H)*P⁻
    Eigen::Matrix<Scalar, N, N> I = Eigen::Matrix<Scalar, N, N>::Identity();
    updated.P = (I - K * H_scalar) * state.P;

    return updated;
}

// =============================================================================
// Combined Predict + Update
// =============================================================================

/**
 * @brief Combined Kalman filter step (predict + update)
 *
 * Convenience function that performs both predict and update in one call.
 *
 * @tparam Scalar Numeric type
 * @tparam N State dimension
 * @tparam U Control input dimension
 * @tparam M Measurement dimension
 */
template <typename Scalar, int N, int U, int M>
FilterState<Scalar, N> kf_step(
    const FilterState<Scalar, N> &state, const Eigen::Matrix<Scalar, U, 1> &u,
    const Eigen::Matrix<Scalar, M, 1> &z, const Eigen::Matrix<double, N, N> &F,
    const Eigen::Matrix<double, N, U> &B, const Eigen::Matrix<double, M, N> &H,
    const ProcessNoise<N> &Q, const MeasurementNoise<M> &R) {
    // Predict
    auto predicted = kf_predict<Scalar, N, U>(state, F, B, u, Q);

    // Update
    return kf_update<Scalar, N, M>(predicted, z, H, R);
}

/**
 * @brief Combined Kalman filter step (no control input)
 */
template <typename Scalar, int N, int M>
FilterState<Scalar, N> kf_step(const FilterState<Scalar, N> &state,
                               const Eigen::Matrix<Scalar, M, 1> &z,
                               const Eigen::Matrix<double, N, N> &F,
                               const Eigen::Matrix<double, M, N> &H,
                               const ProcessNoise<N> &Q,
                               const MeasurementNoise<M> &R) {
    // Predict
    auto predicted = kf_predict<Scalar, N>(state, F, Q);

    // Update
    return kf_update<Scalar, N, M>(predicted, z, H, R);
}

} // namespace vulcan::estimation
