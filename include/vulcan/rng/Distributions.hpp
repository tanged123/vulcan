/**
 * @file Distributions.hpp
 * @brief Higher-level noise generators for sensor simulation
 *
 * Provides convenience functions for generating noise samples
 * compatible with Vulcan sensor models.
 */

#pragma once

#include <Eigen/Core>
#include <vulcan/sensors/AllanVarianceNoise.hpp>

namespace vulcan::rng {

/**
 * @brief Generate IMU noise input for Allan variance model
 *
 * Fills all 18 noise channels (3 axes × 2 sensors × 3 noise types)
 * with independent N(0,1) samples.
 *
 * Example:
 * @code
 * vulcan::rng::RNG rng(42);
 * auto state = vulcan::allan::init_state<double>();
 * auto coeffs = vulcan::allan::consumer_grade_coeffs(0.01);
 *
 * auto input = vulcan::rng::generate_imu_noise(rng);
 * auto noise = vulcan::allan::step(state, coeffs, input);
 * @endcode
 *
 * @tparam RNGType Any type with gaussian() method returning double
 * @param rng Random number generator
 * @return IMUNoiseInput with all channels filled
 */
template <typename RNGType>
allan::IMUNoiseInput<double> generate_imu_noise(RNGType &rng) {
    allan::IMUNoiseInput<double> input;
    for (int i = 0; i < 3; ++i) {
        input.gyro_arw(i) = rng.gaussian();
        input.gyro_bias(i) = rng.gaussian();
        input.gyro_rrw(i) = rng.gaussian();
        input.accel_arw(i) = rng.gaussian();
        input.accel_bias(i) = rng.gaussian();
        input.accel_rrw(i) = rng.gaussian();
    }
    return input;
}

/**
 * @brief Generate 3-axis white noise
 *
 * @tparam RNGType Any type with gaussian() method
 * @param rng Random number generator
 * @return 3D vector of independent N(0,1) samples
 */
template <typename RNGType> Eigen::Vector3d generate_noise3(RNGType &rng) {
    return Eigen::Vector3d(rng.gaussian(), rng.gaussian(), rng.gaussian());
}

/**
 * @brief Generate N-dimensional white noise
 *
 * @tparam N Vector dimension
 * @tparam RNGType Any type with gaussian() method
 * @param rng Random number generator
 * @return N-dimensional vector of independent N(0,1) samples
 */
template <int N, typename RNGType>
Eigen::Vector<double, N> generate_noiseN(RNGType &rng) {
    Eigen::Vector<double, N> v;
    for (int i = 0; i < N; ++i) {
        v(i) = rng.gaussian();
    }
    return v;
}

/**
 * @brief Generate correlated noise with given Cholesky factor
 *
 * For covariance matrix P, compute L = chol(P) and pass L.
 * Returns L * w where w ~ N(0, I).
 *
 * @tparam N Vector dimension
 * @tparam RNGType Any type with gaussian() method
 * @param rng Random number generator
 * @param L Lower-triangular Cholesky factor of covariance
 * @return Correlated noise vector ~ N(0, P)
 */
template <int N, typename RNGType>
Eigen::Vector<double, N>
generate_correlated_noise(RNGType &rng, const Eigen::Matrix<double, N, N> &L) {
    Eigen::Vector<double, N> w;
    for (int i = 0; i < N; ++i) {
        w(i) = rng.gaussian();
    }
    return L * w;
}

/**
 * @brief Generate correlated noise for dynamic-size matrices
 *
 * @tparam RNGType Any type with gaussian() method
 * @param rng Random number generator
 * @param L Lower-triangular Cholesky factor of covariance (n×n)
 * @return Correlated noise vector ~ N(0, P) (n×1)
 */
template <typename RNGType>
Eigen::VectorXd generate_correlated_noise(RNGType &rng,
                                          const Eigen::MatrixXd &L) {
    Eigen::VectorXd w(L.rows());
    for (int i = 0; i < L.rows(); ++i) {
        w(i) = rng.gaussian();
    }
    return L * w;
}

/**
 * @brief Generate turbulence forcing noise (3 independent channels)
 *
 * For Dryden/von Kármán turbulence filters.
 *
 * @tparam RNGType Any type with gaussian() method
 * @param rng Random number generator
 * @param noise_u Output: longitudinal noise
 * @param noise_v Output: lateral noise
 * @param noise_w Output: vertical noise
 */
template <typename RNGType>
void generate_turbulence_noise(RNGType &rng, double &noise_u, double &noise_v,
                               double &noise_w) {
    noise_u = rng.gaussian();
    noise_v = rng.gaussian();
    noise_w = rng.gaussian();
}

} // namespace vulcan::rng
