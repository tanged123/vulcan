// Random Walk Noise Model
// Integrated white noise (rate random walk / drift)
#pragma once

#include <cmath>
#include <janus/janus.hpp>

namespace vulcan::random_walk {

// =============================================================================
// Random Walk State
// =============================================================================

/**
 * @brief Random walk process state
 *
 * Tracks the accumulated value of integrated white noise.
 * This models sensor drift / rate random walk.
 *
 * @tparam Scalar double or casadi::MX
 */
template <typename Scalar> struct State {
    Scalar value; ///< Accumulated random walk value
};

/**
 * @brief Initialize random walk state to zero
 *
 * @tparam Scalar double or casadi::MX
 * @return State initialized to zero
 */
template <typename Scalar> State<Scalar> init_state() {
    return State<Scalar>{.value = Scalar(0)};
}

// =============================================================================
// Discretized Coefficients
// =============================================================================

/**
 * @brief Pre-computed random walk coefficients
 *
 * For rate random walk with parameter K [units/s/√Hz]:
 *   x[k+1] = x[k] + gain * w[k]
 * where w[k] ~ N(0,1) and gain = K * √(Δt)
 *
 * The variance grows linearly with time: Var(x) = K² * t
 */
struct Coeffs {
    double gain; ///< Noise input gain: K * √(Δt)
};

/**
 * @brief Compute random walk coefficients
 *
 * @param K Rate random walk coefficient [units/s/√Hz]
 * @param dt Time step [s]
 * @return Coeffs structure ready for simulation
 */
inline Coeffs compute_coeffs(double K, double dt) {
    return Coeffs{.gain = K * std::sqrt(dt)};
}

// =============================================================================
// Simulation Step
// =============================================================================

/**
 * @brief Step the random walk process
 *
 * Updates the random walk state by integrating white noise.
 *
 * @tparam Scalar double or casadi::MX
 * @param state Current state (updated in-place)
 * @param coeffs Pre-computed coefficients
 * @param noise_input White noise sample with unit variance
 * @return Current random walk value after update
 */
template <typename Scalar>
Scalar step(State<Scalar> &state, const Coeffs &coeffs,
            const Scalar &noise_input) {
    state.value = state.value + coeffs.gain * noise_input;
    return state.value;
}

/**
 * @brief Step random walk with inline coefficient computation
 *
 * Convenience function for simpler usage when coefficients
 * don't need to be pre-computed.
 *
 * @tparam Scalar double or casadi::MX
 * @param state Current state (updated in-place)
 * @param K Rate random walk coefficient [units/s/√Hz]
 * @param dt Time step [s]
 * @param noise_input White noise sample with unit variance
 * @return Current random walk value after update
 */
template <typename Scalar>
Scalar step(State<Scalar> &state, double K, double dt,
            const Scalar &noise_input) {
    Coeffs coeffs = compute_coeffs(K, dt);
    return step(state, coeffs, noise_input);
}

// =============================================================================
// Analysis Utilities
// =============================================================================

/**
 * @brief Compute expected variance at given time
 *
 * For a random walk with parameter K, starting from zero:
 *   Var(t) = K² * t
 *
 * @param K Rate random walk coefficient [units/s/√Hz]
 * @param t Time since start [s]
 * @return Expected variance at time t
 */
inline double expected_variance(double K, double t) { return K * K * t; }

/**
 * @brief Compute expected standard deviation at given time
 *
 * @param K Rate random walk coefficient [units/s/√Hz]
 * @param t Time since start [s]
 * @return Expected standard deviation at time t
 */
inline double expected_stddev(double K, double t) { return K * std::sqrt(t); }

} // namespace vulcan::random_walk
