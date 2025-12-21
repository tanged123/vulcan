// Bias Instability Noise Model
// Long-correlation-time Markov process for sensor bias drift
#pragma once

#include <cmath>
#include <janus/janus.hpp>
#include <vulcan/sensors/MarkovProcess.hpp>

namespace vulcan::bias_instability {

// =============================================================================
// Bias Instability State
// =============================================================================

/**
 * @brief Bias instability state
 *
 * Bias instability is modeled as a first-order Gauss-Markov process
 * with a very long correlation time (typically 100s to 3600s).
 * This represents the slow wandering of the sensor bias.
 *
 * @tparam Scalar double or casadi::MX
 */
template <typename Scalar> struct State {
    Scalar bias; ///< Current bias value
};

/**
 * @brief Initialize state to zero bias
 *
 * @tparam Scalar double or casadi::MX
 * @return State initialized to zero
 */
template <typename Scalar> State<Scalar> init_state() {
    return State<Scalar>{.bias = Scalar(0)};
}

/**
 * @brief Initialize state with given initial bias
 *
 * @tparam Scalar double or casadi::MX
 * @param initial_bias Initial bias value
 * @return State initialized to given bias
 */
template <typename Scalar>
State<Scalar> init_state(const Scalar &initial_bias) {
    return State<Scalar>{.bias = initial_bias};
}

// =============================================================================
// Discretized Coefficients
// =============================================================================

/**
 * @brief Discretized bias instability coefficients
 *
 * Same structure as first-order Markov process, but named
 * for clarity in the bias context.
 */
struct Coeffs {
    double a; ///< State transition coefficient: exp(-Δt/τ)
    double b; ///< Input gain: σ_b·√(1-a²)
};

/**
 * @brief Compute bias instability coefficients
 *
 * @param sigma_b Bias instability magnitude (1σ) [units]
 * @param tau Correlation time [s] (typically 100-3600s)
 * @param dt Time step [s]
 * @return Discretized coefficients
 */
inline Coeffs compute_coeffs(double sigma_b, double tau, double dt) {
    double a = std::exp(-dt / tau);
    double b = sigma_b * std::sqrt(1.0 - a * a);
    return Coeffs{.a = a, .b = b};
}

// =============================================================================
// Simulation Step
// =============================================================================

/**
 * @brief Step the bias instability process
 *
 * Updates bias:
 *   bias[k+1] = a·bias[k] + b·w[k]
 *
 * @tparam Scalar double or casadi::MX
 * @param state Current state (updated in-place)
 * @param coeffs Pre-computed coefficients
 * @param noise_input White noise sample with unit variance
 * @return Current bias value after update
 */
template <typename Scalar>
Scalar step(State<Scalar> &state, const Coeffs &coeffs,
            const Scalar &noise_input) {
    state.bias = coeffs.a * state.bias + coeffs.b * noise_input;
    return state.bias;
}

/**
 * @brief Step with inline coefficient computation
 *
 * Convenience function for simpler usage.
 *
 * @tparam Scalar double or casadi::MX
 * @param state Current state (updated in-place)
 * @param sigma_b Bias instability magnitude [units]
 * @param tau Correlation time [s]
 * @param dt Time step [s]
 * @param noise_input White noise sample with unit variance
 * @return Current bias value after update
 */
template <typename Scalar>
Scalar step(State<Scalar> &state, double sigma_b, double tau, double dt,
            const Scalar &noise_input) {
    Coeffs coeffs = compute_coeffs(sigma_b, tau, dt);
    return step(state, coeffs, noise_input);
}

// =============================================================================
// Analysis Utilities
// =============================================================================

/**
 * @brief Steady-state variance of bias instability
 *
 * At steady state: Var(bias) = σ_b²
 *
 * @param sigma_b Bias instability magnitude
 * @return Steady-state variance
 */
inline double steady_state_variance(double sigma_b) {
    return sigma_b * sigma_b;
}

/**
 * @brief Time to reach fraction of steady-state variance
 *
 * Time for variance to reach (1 - exp(-2t/τ)) of steady-state:
 *   t = -τ/2 · ln(1 - fraction)
 *
 * @param tau Correlation time [s]
 * @param fraction Fraction of steady-state (e.g., 0.632 for 1-time constant)
 * @return Time [s]
 */
inline double time_to_fraction(double tau, double fraction) {
    return -tau / 2.0 * std::log(1.0 - fraction);
}

} // namespace vulcan::bias_instability
