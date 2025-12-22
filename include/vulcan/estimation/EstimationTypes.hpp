// Estimation Types - Common types for Kalman filter implementations
// Part of Vulcan - Aerospace Engineering Utilities
#pragma once

#include <Eigen/Dense>
#include <janus/janus.hpp>

namespace vulcan::estimation {

// =============================================================================
// Filter State Container
// =============================================================================

/**
 * @brief Filter state container holding estimate and covariance
 *
 * @tparam Scalar Numeric type (double or casadi::MX)
 * @tparam N State dimension
 */
template <typename Scalar, int N> struct FilterState {
    Eigen::Matrix<Scalar, N, 1> x; ///< State estimate
    Eigen::Matrix<Scalar, N, N> P; ///< State covariance

    /// Default constructor - zero-initializes state, identity covariance
    FilterState()
        : x(Eigen::Matrix<Scalar, N, 1>::Zero()),
          P(Eigen::Matrix<Scalar, N, N>::Identity()) {}

    /// Constructor with state and covariance
    FilterState(const Eigen::Matrix<Scalar, N, 1> &state,
                const Eigen::Matrix<Scalar, N, N> &covariance)
        : x(state), P(covariance) {}
};

// =============================================================================
// Noise Specifications
// =============================================================================

/**
 * @brief Process noise specification
 *
 * Wraps process noise covariance matrix Q for clarity in API.
 *
 * @tparam N State dimension
 */
template <int N> struct ProcessNoise {
    Eigen::Matrix<double, N, N> Q; ///< Process noise covariance

    /// Default constructor - zero noise
    ProcessNoise() : Q(Eigen::Matrix<double, N, N>::Zero()) {}

    /// Constructor with covariance matrix
    explicit ProcessNoise(const Eigen::Matrix<double, N, N> &covariance)
        : Q(covariance) {}

    /// Create diagonal process noise from variance vector
    static ProcessNoise<N>
    diagonal(const Eigen::Matrix<double, N, 1> &variances) {
        ProcessNoise<N> noise;
        noise.Q = variances.asDiagonal();
        return noise;
    }

    /// Create scalar (isotropic) process noise
    static ProcessNoise<N> scalar(double variance) {
        ProcessNoise<N> noise;
        noise.Q = Eigen::Matrix<double, N, N>::Identity() * variance;
        return noise;
    }
};

/**
 * @brief Measurement noise specification
 *
 * Wraps measurement noise covariance matrix R for clarity in API.
 *
 * @tparam M Measurement dimension
 */
template <int M> struct MeasurementNoise {
    Eigen::Matrix<double, M, M> R; ///< Measurement noise covariance

    /// Default constructor - zero noise
    MeasurementNoise() : R(Eigen::Matrix<double, M, M>::Zero()) {}

    /// Constructor with covariance matrix
    explicit MeasurementNoise(const Eigen::Matrix<double, M, M> &covariance)
        : R(covariance) {}

    /// Create diagonal measurement noise from variance vector
    static MeasurementNoise<M>
    diagonal(const Eigen::Matrix<double, M, 1> &variances) {
        MeasurementNoise<M> noise;
        noise.R = variances.asDiagonal();
        return noise;
    }

    /// Create scalar (isotropic) measurement noise
    static MeasurementNoise<M> scalar(double variance) {
        MeasurementNoise<M> noise;
        noise.R = Eigen::Matrix<double, M, M>::Identity() * variance;
        return noise;
    }
};

} // namespace vulcan::estimation
