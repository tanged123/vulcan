// Unscented Kalman Filter - UKF predict/update using sigma points
// Part of Vulcan - Aerospace Engineering Utilities
#pragma once

#include <vulcan/estimation/EstimationTypes.hpp>

namespace vulcan::estimation {

// =============================================================================
// UKF Parameters
// =============================================================================

/**
 * @brief UKF tuning parameters
 *
 * Controls the spread and weighting of sigma points.
 */
struct UKFParams {
    double alpha = 1e-3; ///< Spread of sigma points around mean (1e-4 to 1)
    double beta = 2.0;   ///< Prior knowledge about distribution (2 = Gaussian)
    double kappa = 0.0;  ///< Secondary scaling parameter

    /// Compute lambda scaling factor
    double lambda(int n) const { return alpha * alpha * (n + kappa) - n; }

    /// Compute weight for mean of zeroth sigma point
    double Wm0(int n) const { return lambda(n) / (n + lambda(n)); }

    /// Compute weight for covariance of zeroth sigma point
    double Wc0(int n) const { return Wm0(n) + (1 - alpha * alpha + beta); }

    /// Compute weight for other sigma points (same for mean and covariance)
    double Wi(int n) const { return 1.0 / (2 * (n + lambda(n))); }
};

// =============================================================================
// Sigma Point Generation
// =============================================================================

/**
 * @brief Generate sigma points for UKF
 *
 * Generates 2N+1 sigma points using the matrix square root (Cholesky).
 * Points are arranged as columns: [x, x + √((n+λ)P), x - √((n+λ)P)]
 *
 * @tparam N State dimension
 * @param x Mean state vector [N x 1]
 * @param P State covariance [N x N]
 * @param params UKF tuning parameters
 * @return Sigma points matrix [N x 2N+1]
 *
 * @note Numeric-only (Cholesky decomposition not available for symbolic)
 */
template <int N>
Eigen::Matrix<double, N, 2 * N + 1>
sigma_points(const Eigen::Matrix<double, N, 1> &x,
             const Eigen::Matrix<double, N, N> &P, const UKFParams &params) {
    constexpr int num_sigma = 2 * N + 1;
    Eigen::Matrix<double, N, num_sigma> sigma;

    // Scaling factor
    double scale = std::sqrt(N + params.lambda(N));

    // Cholesky decomposition of P: P = L*Lᵀ
    Eigen::LLT<Eigen::Matrix<double, N, N>> llt(P);
    Eigen::Matrix<double, N, N> L = llt.matrixL();
    Eigen::Matrix<double, N, N> sqrtP = scale * L;

    // Zeroth sigma point: mean
    sigma.col(0) = x;

    // Sigma points from positive and negative perturbations
    for (int i = 0; i < N; ++i) {
        sigma.col(1 + i) = x + sqrtP.col(i);
        sigma.col(1 + N + i) = x - sqrtP.col(i);
    }

    return sigma;
}

// =============================================================================
// UKF Predict
// =============================================================================

/**
 * @brief Unscented Kalman filter predict step
 *
 * Propagates sigma points through nonlinear dynamics:
 *   1. Generate sigma points from current state
 *   2. Propagate each sigma point through f(x, u)
 *   3. Compute weighted mean and covariance of propagated points
 *
 * @tparam N State dimension
 * @tparam U Control input dimension
 * @tparam DynamicsFunc Callable: x_next = f(x, u)
 * @param state Current filter state (x, P) - must be double for Cholesky
 * @param u Control input vector [U x 1]
 * @param f Nonlinear dynamics function
 * @param noise Process noise specification (Q)
 * @param params UKF tuning parameters
 * @return Predicted filter state (x⁻, P⁻)
 *
 * @note Numeric-only due to Cholesky decomposition requirement
 */
template <int N, int U, typename DynamicsFunc>
FilterState<double, N> ukf_predict(const FilterState<double, N> &state,
                                   const Eigen::Matrix<double, U, 1> &u,
                                   DynamicsFunc f, const ProcessNoise<N> &noise,
                                   const UKFParams &params = {}) {
    constexpr int num_sigma = 2 * N + 1;

    // Generate sigma points
    auto sigma = sigma_points<N>(state.x, state.P, params);

    // Propagate sigma points through dynamics
    Eigen::Matrix<double, N, num_sigma> sigma_prop;
    for (int i = 0; i < num_sigma; ++i) {
        Eigen::Matrix<double, N, 1> xi = sigma.col(i);
        sigma_prop.col(i) = f(xi, u);
    }

    // Compute weights
    double Wm0 = params.Wm0(N);
    double Wc0 = params.Wc0(N);
    double Wi = params.Wi(N);

    // Weighted mean
    FilterState<double, N> predicted;
    predicted.x = Wm0 * sigma_prop.col(0);
    for (int i = 1; i < num_sigma; ++i) {
        predicted.x += Wi * sigma_prop.col(i);
    }

    // Weighted covariance
    Eigen::Matrix<double, N, 1> diff0 = sigma_prop.col(0) - predicted.x;
    predicted.P = Wc0 * diff0 * diff0.transpose();
    for (int i = 1; i < num_sigma; ++i) {
        Eigen::Matrix<double, N, 1> diff = sigma_prop.col(i) - predicted.x;
        predicted.P += Wi * diff * diff.transpose();
    }
    predicted.P += noise.Q;

    return predicted;
}

/**
 * @brief UKF predict (no control input)
 */
template <int N, typename DynamicsFunc>
FilterState<double, N> ukf_predict(const FilterState<double, N> &state,
                                   DynamicsFunc f, const ProcessNoise<N> &noise,
                                   const UKFParams &params = {}) {
    // Wrap dynamics to accept dummy control
    auto f_wrapped = [&f](const Eigen::Matrix<double, N, 1> &x,
                          const Eigen::Matrix<double, 1, 1> & /*u*/) {
        return f(x);
    };
    Eigen::Matrix<double, 1, 1> dummy_u = Eigen::Matrix<double, 1, 1>::Zero();
    return ukf_predict<N, 1>(state, dummy_u, f_wrapped, noise, params);
}

// =============================================================================
// UKF Update
// =============================================================================

/**
 * @brief Unscented Kalman filter update step
 *
 * Incorporates measurement using sigma points:
 *   1. Generate sigma points from predicted state
 *   2. Propagate each through measurement model h(x)
 *   3. Compute innovation covariance and cross-covariance
 *   4. Compute Kalman gain and update state
 *
 * @tparam N State dimension
 * @tparam M Measurement dimension
 * @tparam MeasurementFunc Callable: z_pred = h(x)
 * @param state Predicted filter state (x⁻, P⁻)
 * @param z Measurement vector [M x 1]
 * @param h Nonlinear measurement function
 * @param noise Measurement noise specification (R)
 * @param params UKF tuning parameters
 * @return Updated filter state (x̂, P)
 */
template <int N, int M, typename MeasurementFunc>
FilterState<double, N>
ukf_update(const FilterState<double, N> &state,
           const Eigen::Matrix<double, M, 1> &z, MeasurementFunc h,
           const MeasurementNoise<M> &noise, const UKFParams &params = {}) {
    constexpr int num_sigma = 2 * N + 1;

    // Generate sigma points from predicted state
    auto sigma = sigma_points<N>(state.x, state.P, params);

    // Propagate sigma points through measurement model
    Eigen::Matrix<double, M, num_sigma> gamma;
    for (int i = 0; i < num_sigma; ++i) {
        Eigen::Matrix<double, N, 1> xi = sigma.col(i);
        gamma.col(i) = h(xi);
    }

    // Compute weights
    double Wm0 = params.Wm0(N);
    double Wc0 = params.Wc0(N);
    double Wi = params.Wi(N);

    // Predicted measurement mean
    Eigen::Matrix<double, M, 1> z_pred = Wm0 * gamma.col(0);
    for (int i = 1; i < num_sigma; ++i) {
        z_pred += Wi * gamma.col(i);
    }

    // Innovation covariance S = Pzz + R
    Eigen::Matrix<double, M, 1> dz0 = gamma.col(0) - z_pred;
    Eigen::Matrix<double, M, M> Pzz = Wc0 * dz0 * dz0.transpose();
    for (int i = 1; i < num_sigma; ++i) {
        Eigen::Matrix<double, M, 1> dz = gamma.col(i) - z_pred;
        Pzz += Wi * dz * dz.transpose();
    }
    Eigen::Matrix<double, M, M> S = Pzz + noise.R;

    // Cross-covariance Pxz
    Eigen::Matrix<double, N, 1> dx0 = sigma.col(0) - state.x;
    Eigen::Matrix<double, N, M> Pxz = Wc0 * dx0 * dz0.transpose();
    for (int i = 1; i < num_sigma; ++i) {
        Eigen::Matrix<double, N, 1> dx = sigma.col(i) - state.x;
        Eigen::Matrix<double, M, 1> dz = gamma.col(i) - z_pred;
        Pxz += Wi * dx * dz.transpose();
    }

    // Kalman gain
    Eigen::Matrix<double, N, M> K = Pxz * S.inverse();

    // Update state
    FilterState<double, N> updated;
    updated.x = state.x + K * (z - z_pred);
    updated.P = state.P - K * S * K.transpose();

    return updated;
}

// =============================================================================
// Combined UKF Step
// =============================================================================

/**
 * @brief Combined UKF step (predict + update)
 */
template <int N, int U, int M, typename DynamicsFunc, typename MeasurementFunc>
FilterState<double, N>
ukf_step(const FilterState<double, N> &state,
         const Eigen::Matrix<double, U, 1> &u,
         const Eigen::Matrix<double, M, 1> &z, DynamicsFunc f,
         MeasurementFunc h, const ProcessNoise<N> &Q,
         const MeasurementNoise<M> &R, const UKFParams &params = {}) {
    // Predict
    auto predicted = ukf_predict<N, U>(state, u, f, Q, params);

    // Update
    return ukf_update<N, M>(predicted, z, h, R, params);
}

} // namespace vulcan::estimation
