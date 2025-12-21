// Gaussian White Noise
// White noise scaling and variance utilities for sensor simulation
#pragma once

#include <cmath>
#include <janus/janus.hpp>

namespace vulcan::gaussian {

// =============================================================================
// White Noise Scaling
// =============================================================================

/**
 * @brief Apply Gaussian white noise scaling
 *
 * Scales a unit-variance white noise sample by the specified standard
 * deviation. For discrete-time simulation with a given PSD, use the
 * appropriate scaling: σ_discrete = σ_continuous / √(Δt)
 *
 * @tparam Scalar double or casadi::MX
 * @param noise_input White noise sample with unit variance (N(0,1))
 * @param sigma Standard deviation of the output noise
 * @return Scaled noise value with variance σ²
 */
template <typename Scalar>
Scalar apply(const Scalar &noise_input, double sigma) {
    return sigma * noise_input;
}

/**
 * @brief Apply white noise with PSD-based scaling
 *
 * Converts continuous-time power spectral density to discrete-time
 * variance and applies appropriate scaling. This is the correct way
 * to generate discrete noise samples from a continuous PSD specification.
 *
 * For one-sided PSD S(f) [units²/Hz]:
 *   σ²_discrete = S(f) / Δt
 *
 * @tparam Scalar double or casadi::MX
 * @param noise_input White noise sample with unit variance
 * @param psd_density One-sided power spectral density [units²/Hz]
 * @param dt Time step [s]
 * @return Scaled noise value
 */
template <typename Scalar>
Scalar apply_psd(const Scalar &noise_input, double psd_density, double dt) {
    double sigma = std::sqrt(psd_density / dt);
    return sigma * noise_input;
}

// =============================================================================
// Variance Calculations
// =============================================================================

/**
 * @brief Compute discrete variance from continuous PSD
 *
 * Given a one-sided power spectral density S(f) [units²/Hz] and sample
 * period Δt, computes the variance of discrete-time samples.
 *
 * @param psd_density One-sided power spectral density [units²/Hz]
 * @param dt Time step [s]
 * @return Variance of discrete samples [units²]
 */
inline double variance_from_psd(double psd_density, double dt) {
    return psd_density / dt;
}

/**
 * @brief Compute continuous PSD from discrete variance
 *
 * Inverse of variance_from_psd.
 *
 * @param variance Discrete-time variance [units²]
 * @param dt Time step [s]
 * @return One-sided power spectral density [units²/Hz]
 */
inline double psd_from_variance(double variance, double dt) {
    return variance * dt;
}

/**
 * @brief Compute white noise gain from Allan variance N parameter
 *
 * The angle/velocity random walk parameter N from Allan variance analysis
 * is related to the white noise PSD by: PSD = N²
 *
 * For discrete-time: σ = N / √(Δt)
 *
 * @param N Angle or velocity random walk [rad/s/√Hz] or [m/s²/√Hz]
 * @param dt Time step [s]
 * @return Standard deviation for discrete noise samples
 */
inline double sigma_from_arw(double N, double dt) { return N / std::sqrt(dt); }

} // namespace vulcan::gaussian
