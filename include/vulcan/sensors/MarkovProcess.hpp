// First-Order Gauss-Markov Process
// Exponentially correlated noise model
#pragma once

#include <cmath>
#include <janus/janus.hpp>

namespace vulcan::markov {

// =============================================================================
// Process State
// =============================================================================

/**
 * @brief First-order Gauss-Markov process state
 *
 * The continuous-time process is:
 *   dx/dt = -x/τ + w(t)
 *
 * where τ is the correlation time and w(t) is white noise chosen
 * such that the steady-state variance is σ².
 *
 * @tparam Scalar double or casadi::MX
 */
template <typename Scalar> struct State {
    Scalar x; ///< Process state
};

/**
 * @brief Initialize state to zero
 *
 * @tparam Scalar double or casadi::MX
 * @return State initialized to zero
 */
template <typename Scalar> State<Scalar> init_state() {
    return State<Scalar>{.x = Scalar(0)};
}

/**
 * @brief Initialize state to specific value
 *
 * Useful for starting with a non-zero initial bias.
 *
 * @tparam Scalar double or casadi::MX
 * @param initial_value Initial process value
 * @return State initialized to given value
 */
template <typename Scalar>
State<Scalar> init_state(const Scalar &initial_value) {
    return State<Scalar>{.x = initial_value};
}

// =============================================================================
// Discretized Process Parameters
// =============================================================================

/**
 * @brief Discretized first-order Markov process coefficients
 *
 * For discrete-time step:
 *   x[k+1] = φ·x[k] + q·w[k]
 *
 * where:
 *   φ = exp(-Δt/τ) is the state transition
 *   q = σ·√(1-φ²) preserves steady-state variance
 *   w[k] ~ N(0,1)
 */
struct Coeffs {
    double phi; ///< State transition: exp(-Δt/τ)
    double q;   ///< Process noise gain: σ·√(1-φ²)
};

/**
 * @brief Discretize first-order Markov process
 *
 * Uses exact discretization that preserves the steady-state variance
 * regardless of the time step.
 *
 * @param tau Correlation time [s]
 * @param sigma Steady-state standard deviation
 * @param dt Time step [s]
 * @return Discretized coefficients
 */
inline Coeffs discretize(double tau, double sigma, double dt) {
    double phi = std::exp(-dt / tau);
    double q = sigma * std::sqrt(1.0 - phi * phi);
    return Coeffs{.phi = phi, .q = q};
}

// =============================================================================
// Continuous-Time Parameters
// =============================================================================

/**
 * @brief Compute continuous-time process noise PSD
 *
 * For a first-order Markov process with correlation time τ and
 * steady-state variance σ², the continuous-time process noise
 * PSD is: S_w = 2σ²/τ
 *
 * @param tau Correlation time [s]
 * @param sigma Steady-state standard deviation
 * @return Process noise power spectral density
 */
inline double process_noise_psd(double tau, double sigma) {
    return 2.0 * sigma * sigma / tau;
}

/**
 * @brief Compute continuous-time process noise intensity
 *
 * The process noise intensity q_c relates to PSD by: q_c² = S_w
 *
 * @param tau Correlation time [s]
 * @param sigma Steady-state standard deviation
 * @return Process noise intensity q_c
 */
inline double process_noise_intensity(double tau, double sigma) {
    return std::sqrt(2.0 / tau) * sigma;
}

// =============================================================================
// Simulation Step
// =============================================================================

/**
 * @brief Step the first-order Markov process
 *
 * Updates the process state:
 *   x[k+1] = φ·x[k] + q·w[k]
 *
 * @tparam Scalar double or casadi::MX
 * @param state Current state (updated in-place)
 * @param coeffs Discretized coefficients
 * @param noise_input White noise sample with unit variance
 * @return Current process output after update
 */
template <typename Scalar>
Scalar step(State<Scalar> &state, const Coeffs &coeffs,
            const Scalar &noise_input) {
    state.x = coeffs.phi * state.x + coeffs.q * noise_input;
    return state.x;
}

/**
 * @brief Step with inline discretization
 *
 * Convenience function when coefficients don't need to be pre-computed.
 *
 * @tparam Scalar double or casadi::MX
 * @param state Current state (updated in-place)
 * @param tau Correlation time [s]
 * @param sigma Steady-state standard deviation
 * @param dt Time step [s]
 * @param noise_input White noise sample with unit variance
 * @return Current process output after update
 */
template <typename Scalar>
Scalar step(State<Scalar> &state, double tau, double sigma, double dt,
            const Scalar &noise_input) {
    Coeffs coeffs = discretize(tau, sigma, dt);
    return step(state, coeffs, noise_input);
}

// =============================================================================
// Analysis Utilities
// =============================================================================

/**
 * @brief Compute autocorrelation at lag τ_lag
 *
 * For first-order Markov process:
 *   R(τ_lag) = σ² · exp(-|τ_lag|/τ)
 *
 * @param tau Correlation time [s]
 * @param sigma Steady-state standard deviation
 * @param tau_lag Time lag [s]
 * @return Autocorrelation value
 */
inline double autocorrelation(double tau, double sigma, double tau_lag) {
    return sigma * sigma * std::exp(-std::abs(tau_lag) / tau);
}

/**
 * @brief Compute power spectral density at frequency f
 *
 * For first-order Markov:
 *   S(f) = σ²·(2τ)/(1 + (2πfτ)²)
 *
 * This is a Lorentzian spectrum.
 *
 * @param tau Correlation time [s]
 * @param sigma Steady-state standard deviation
 * @param f Frequency [Hz]
 * @return Power spectral density at frequency f
 */
inline double psd_at_frequency(double tau, double sigma, double f) {
    double omega_tau = 2.0 * M_PI * f * tau;
    return sigma * sigma * 2.0 * tau / (1.0 + omega_tau * omega_tau);
}

} // namespace vulcan::markov
