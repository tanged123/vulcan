// Extended Kalman Filter - EKF predict/update for nonlinear systems
// Part of Vulcan - Aerospace Engineering Utilities
#pragma once

#include <vulcan/estimation/EstimationTypes.hpp>

namespace vulcan::estimation {

// =============================================================================
// Extended Kalman Filter Predict
// =============================================================================

/**
 * @brief Extended Kalman filter predict step
 *
 * Propagates state through nonlinear dynamics with linear covariance update:
 *   x̂⁻ = f(x̂, u)              (nonlinear state propagation)
 *   P⁻ = F*P*Fᵀ + Q           (linearized covariance update)
 *
 * The user provides both the nonlinear dynamics function and its Jacobian.
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 * @tparam N State dimension
 * @tparam U Control input dimension
 * @tparam DynamicsFunc Callable: x_next = f(x, u)
 * @param state Current filter state (x, P)
 * @param u Control input vector [U x 1]
 * @param f Nonlinear dynamics function
 * @param F Jacobian of dynamics ∂f/∂x evaluated at current state [N x N]
 * @param noise Process noise specification (Q)
 * @return Predicted filter state (x⁻, P⁻)
 */
template <typename Scalar, int N, int U, typename DynamicsFunc>
FilterState<Scalar, N> ekf_predict(const FilterState<Scalar, N> &state,
                                   const Eigen::Matrix<Scalar, U, 1> &u,
                                   DynamicsFunc f,
                                   const Eigen::Matrix<double, N, N> &F,
                                   const ProcessNoise<N> &noise) {
    FilterState<Scalar, N> predicted;

    // Nonlinear state prediction: x⁻ = f(x, u)
    predicted.x = f(state.x, u);

    // Linearized covariance prediction: P⁻ = F*P*Fᵀ + Q
    auto F_scalar = F.template cast<Scalar>();
    predicted.P = F_scalar * state.P * F_scalar.transpose() +
                  noise.Q.template cast<Scalar>();

    return predicted;
}

/**
 * @brief Extended Kalman filter predict (no control input)
 *
 * For autonomous nonlinear systems.
 *
 * @tparam DynamicsFunc Callable: x_next = f(x)
 */
template <typename Scalar, int N, typename DynamicsFunc>
FilterState<Scalar, N> ekf_predict(const FilterState<Scalar, N> &state,
                                   DynamicsFunc f,
                                   const Eigen::Matrix<double, N, N> &F,
                                   const ProcessNoise<N> &noise) {
    FilterState<Scalar, N> predicted;

    // Nonlinear state prediction: x⁻ = f(x)
    predicted.x = f(state.x);

    // Linearized covariance prediction: P⁻ = F*P*Fᵀ + Q
    auto F_scalar = F.template cast<Scalar>();
    predicted.P = F_scalar * state.P * F_scalar.transpose() +
                  noise.Q.template cast<Scalar>();

    return predicted;
}

// =============================================================================
// Extended Kalman Filter Update
// =============================================================================

/**
 * @brief Extended Kalman filter update step
 *
 * Incorporates measurement using nonlinear observation model:
 *   z_pred = h(x̂⁻)             (predicted measurement)
 *   S = H*P⁻*Hᵀ + R           (innovation covariance)
 *   K = P⁻*Hᵀ*S⁻¹             (Kalman gain)
 *   x̂ = x̂⁻ + K*(z - z_pred)   (state update)
 *   P = (I - K*H)*P⁻          (covariance update)
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 * @tparam N State dimension
 * @tparam M Measurement dimension
 * @tparam MeasurementFunc Callable: z_pred = h(x)
 * @param state Predicted filter state (x⁻, P⁻)
 * @param z Measurement vector [M x 1]
 * @param h Nonlinear measurement function
 * @param H Jacobian of measurement model ∂h/∂x [M x N]
 * @param noise Measurement noise specification (R)
 * @return Updated filter state (x̂, P)
 */
template <typename Scalar, int N, int M, typename MeasurementFunc>
FilterState<Scalar, N> ekf_update(const FilterState<Scalar, N> &state,
                                  const Eigen::Matrix<Scalar, M, 1> &z,
                                  MeasurementFunc h,
                                  const Eigen::Matrix<double, M, N> &H,
                                  const MeasurementNoise<M> &noise) {
    // Cast matrices to Scalar type
    auto H_scalar = H.template cast<Scalar>();
    auto R_scalar = noise.R.template cast<Scalar>();

    // Predicted measurement: z_pred = h(x)
    Eigen::Matrix<Scalar, M, 1> z_pred = h(state.x);

    // Innovation covariance: S = H*P*Hᵀ + R
    Eigen::Matrix<Scalar, M, M> S =
        H_scalar * state.P * H_scalar.transpose() + R_scalar;

    // Kalman gain: K = P*Hᵀ*S⁻¹
    Eigen::Matrix<Scalar, N, M> K =
        state.P * H_scalar.transpose() * S.inverse();

    // Innovation (measurement residual): y = z - h(x)
    Eigen::Matrix<Scalar, M, 1> innovation = z - z_pred;

    // State update: x̂ = x⁻ + K*y
    FilterState<Scalar, N> updated;
    updated.x = state.x + K * innovation;

    // Covariance update: P = (I - K*H)*P⁻
    Eigen::Matrix<Scalar, N, N> I = Eigen::Matrix<Scalar, N, N>::Identity();
    updated.P = (I - K * H_scalar) * state.P;

    return updated;
}

// =============================================================================
// Combined EKF Step
// =============================================================================

/**
 * @brief Combined EKF step (predict + update)
 *
 * @tparam DynamicsFunc Callable: x_next = f(x, u)
 * @tparam MeasurementFunc Callable: z_pred = h(x)
 */
template <typename Scalar, int N, int U, int M, typename DynamicsFunc,
          typename MeasurementFunc>
FilterState<Scalar, N> ekf_step(
    const FilterState<Scalar, N> &state, const Eigen::Matrix<Scalar, U, 1> &u,
    const Eigen::Matrix<Scalar, M, 1> &z, DynamicsFunc f, MeasurementFunc h,
    const Eigen::Matrix<double, N, N> &F, const Eigen::Matrix<double, M, N> &H,
    const ProcessNoise<N> &Q, const MeasurementNoise<M> &R) {
    // Predict
    auto predicted = ekf_predict<Scalar, N, U>(state, u, f, F, Q);

    // Update
    return ekf_update<Scalar, N, M>(predicted, z, h, H, R);
}

} // namespace vulcan::estimation
