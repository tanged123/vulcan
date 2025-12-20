// Dryden Turbulence Model
// Forming filters with rational transfer functions (MIL-F-8785C)
#pragma once

#include <cmath>
#include <vulcan/wind/WindTypes.hpp>

namespace vulcan::dryden {

// ============================================================================
// Power Spectral Density Functions
// ============================================================================

/**
 * @brief Dryden longitudinal PSD
 *
 * Φ_u(Ω) = σ_u² * (2L_u/π) * [1 / (1 + (L_u·Ω)²)]
 *
 * @tparam Scalar double or casadi::MX
 * @param omega Spatial frequency Ω [rad/m]
 * @param sigma_u RMS turbulence intensity [m/s]
 * @param L_u Scale length [m]
 * @return PSD value [m²/s²/(rad/m)]
 */
template <typename Scalar>
Scalar psd_longitudinal(const Scalar &omega, double sigma_u, double L_u) {
    Scalar L_omega = L_u * omega;
    return sigma_u * sigma_u * (2.0 * L_u / M_PI) /
           (Scalar(1) + L_omega * L_omega);
}

/**
 * @brief Dryden lateral/vertical PSD
 *
 * Φ_v(Ω) = σ_v² * (L_v/π) * [(1 + 3(L_v·Ω)²) / (1 + (L_v·Ω)²)²]
 *
 * Same form for vertical component with w subscripts.
 *
 * @tparam Scalar double or casadi::MX
 * @param omega Spatial frequency Ω [rad/m]
 * @param sigma RMS turbulence intensity [m/s]
 * @param L Scale length [m]
 * @return PSD value [m²/s²/(rad/m)]
 */
template <typename Scalar>
Scalar psd_lateral(const Scalar &omega, double sigma, double L) {
    Scalar L_omega = L * omega;
    Scalar L_omega_sq = L_omega * L_omega;
    Scalar denom = Scalar(1) + L_omega_sq;
    return sigma * sigma * (L / M_PI) * (Scalar(1) + Scalar(3) * L_omega_sq) /
           (denom * denom);
}

// ============================================================================
// Forming Filter State
// ============================================================================

/**
 * @brief Dryden forming filter state
 *
 * Tracks internal filter states for time-domain simulation.
 * - Longitudinal: 1st order filter (1 state)
 * - Lateral: 2nd order filter (2 states)
 * - Vertical: 2nd order filter (2 states)
 */
template <typename Scalar> struct FilterState {
    Scalar x_u;  ///< Longitudinal filter state
    Scalar x_v1; ///< Lateral filter state 1
    Scalar x_v2; ///< Lateral filter state 2
    Scalar x_w1; ///< Vertical filter state 1
    Scalar x_w2; ///< Vertical filter state 2
};

/**
 * @brief Initialize filter state to zero
 *
 * @tparam Scalar double or casadi::MX
 * @return FilterState with all zeros
 */
template <typename Scalar> FilterState<Scalar> init_state() {
    return FilterState<Scalar>{.x_u = Scalar(0),
                               .x_v1 = Scalar(0),
                               .x_v2 = Scalar(0),
                               .x_w1 = Scalar(0),
                               .x_w2 = Scalar(0)};
}

// ============================================================================
// Filter Coefficients
// ============================================================================

/**
 * @brief Discretized Dryden forming filter coefficients
 *
 * Coefficients for the discrete-time state-space representation:
 * - Longitudinal: x[k+1] = a_u * x[k] + b_u * noise, output = c_u * x[k]
 * - Lateral/Vertical: 2nd order system with similar structure
 */
struct FilterCoeffs {
    // Longitudinal (first-order filter)
    double a_u; ///< State transition coefficient
    double b_u; ///< Input coefficient
    double c_u; ///< Output coefficient

    // Lateral (second-order filter)
    double a_v11, a_v12, a_v21, a_v22; ///< State transition matrix
    double b_v1, b_v2;                 ///< Input vector
    double c_v1, c_v2;                 ///< Output vector

    // Vertical (second-order filter, same structure as lateral)
    double a_w11, a_w12, a_w21, a_w22;
    double b_w1, b_w2;
    double c_w1, c_w2;
};

/**
 * @brief Compute forming filter coefficients
 *
 * Discretizes the continuous-time Dryden forming filters for given
 * airspeed and time step using the bilinear (Tustin) transform.
 *
 * Continuous-time transfer functions:
 * - Longitudinal: H_u(s) = σ_u * sqrt(2V/πL_u) / (1 + L_u/V * s)
 * - Lateral:      H_v(s) = σ_v * sqrt(V/πL_v) * (1 + sqrt(3)*L_v/V*s) / (1 +
 * L_v/V*s)^2
 * - Vertical:     H_w(s) = same form as lateral with w subscripts
 *
 * @param params Turbulence parameters (σ and L values)
 * @param airspeed True airspeed V [m/s]
 * @param dt Time step [s]
 * @return Discretized filter coefficients
 */
inline FilterCoeffs
compute_filter_coeffs(const wind::TurbulenceParams<double> &params,
                      double airspeed, double dt) {
    FilterCoeffs coeffs;

    // Longitudinal filter (first-order)
    // Time constant: τ_u = L_u / V
    double tau_u = params.L_u / airspeed;
    double alpha_u = dt / tau_u;
    coeffs.a_u = std::exp(-alpha_u);
    coeffs.b_u = std::sqrt(1.0 - coeffs.a_u * coeffs.a_u);
    coeffs.c_u = params.sigma_u;

    // Lateral filter (second-order)
    // Time constant: τ_v = L_v / V
    double tau_v = params.L_v / airspeed;
    double alpha_v = dt / tau_v;
    double exp_v = std::exp(-alpha_v);

    // State-space representation for second-order system
    // Using diagonal approximation for the 2x2 system
    coeffs.a_v11 = exp_v;
    coeffs.a_v12 = alpha_v * exp_v;
    coeffs.a_v21 = 0.0;
    coeffs.a_v22 = exp_v;

    // Input and output scaling
    double gain_v = params.sigma_v * std::sqrt(3.0 * params.L_v / airspeed);
    coeffs.b_v1 = std::sqrt(1.0 - exp_v * exp_v);
    coeffs.b_v2 = coeffs.b_v1 * std::sqrt(3.0) / 2.0;
    coeffs.c_v1 = gain_v;
    coeffs.c_v2 = gain_v * std::sqrt(3.0);

    // Vertical filter (same structure as lateral)
    double tau_w = params.L_w / airspeed;
    double alpha_w = dt / tau_w;
    double exp_w = std::exp(-alpha_w);

    coeffs.a_w11 = exp_w;
    coeffs.a_w12 = alpha_w * exp_w;
    coeffs.a_w21 = 0.0;
    coeffs.a_w22 = exp_w;

    double gain_w = params.sigma_w * std::sqrt(3.0 * params.L_w / airspeed);
    coeffs.b_w1 = std::sqrt(1.0 - exp_w * exp_w);
    coeffs.b_w2 = coeffs.b_w1 * std::sqrt(3.0) / 2.0;
    coeffs.c_w1 = gain_w;
    coeffs.c_w2 = gain_w * std::sqrt(3.0);

    return coeffs;
}

// ============================================================================
// Filter Step Function
// ============================================================================

/**
 * @brief Step the Dryden forming filter
 *
 * Updates filter state and computes gust velocities given white noise input.
 * The noise inputs should be independent standard normal random variables.
 *
 * @tparam Scalar double or casadi::MX
 * @param state Current filter state (updated in-place)
 * @param coeffs Pre-computed filter coefficients
 * @param noise_u White noise input for longitudinal channel
 * @param noise_v White noise input for lateral channel
 * @param noise_w White noise input for vertical channel
 * @return Gust velocity (u_g, v_g, w_g)
 */
template <typename Scalar>
wind::GustVelocity<Scalar>
step(FilterState<Scalar> &state, const FilterCoeffs &coeffs,
     const Scalar &noise_u, const Scalar &noise_v, const Scalar &noise_w) {
    // Longitudinal (1st order)
    Scalar new_x_u = coeffs.a_u * state.x_u + coeffs.b_u * noise_u;
    Scalar u_g = coeffs.c_u * new_x_u;
    state.x_u = new_x_u;

    // Lateral (2nd order)
    Scalar new_x_v1 = coeffs.a_v11 * state.x_v1 + coeffs.a_v12 * state.x_v2 +
                      coeffs.b_v1 * noise_v;
    Scalar new_x_v2 = coeffs.a_v21 * state.x_v1 + coeffs.a_v22 * state.x_v2 +
                      coeffs.b_v2 * noise_v;
    Scalar v_g = coeffs.c_v1 * new_x_v1 + coeffs.c_v2 * new_x_v2;
    state.x_v1 = new_x_v1;
    state.x_v2 = new_x_v2;

    // Vertical (2nd order)
    Scalar new_x_w1 = coeffs.a_w11 * state.x_w1 + coeffs.a_w12 * state.x_w2 +
                      coeffs.b_w1 * noise_w;
    Scalar new_x_w2 = coeffs.a_w21 * state.x_w1 + coeffs.a_w22 * state.x_w2 +
                      coeffs.b_w2 * noise_w;
    Scalar w_g = coeffs.c_w1 * new_x_w1 + coeffs.c_w2 * new_x_w2;
    state.x_w1 = new_x_w1;
    state.x_w2 = new_x_w2;

    return wind::GustVelocity<Scalar>{.u_g = u_g, .v_g = v_g, .w_g = w_g};
}

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * @brief Compute all filter coefficients for MIL-spec conditions
 *
 * Convenience function that combines mil_spec_params and compute_filter_coeffs.
 *
 * @param altitude Altitude above ground level [m]
 * @param severity Turbulence severity level
 * @param airspeed True airspeed [m/s]
 * @param dt Time step [s]
 * @return FilterCoeffs ready for simulation
 */
inline FilterCoeffs mil_spec_coeffs(double altitude,
                                    wind::TurbulenceSeverity severity,
                                    double airspeed, double dt) {
    wind::TurbulenceParams params = wind::mil_spec_params(altitude, severity);
    return compute_filter_coeffs(params, airspeed, dt);
}

} // namespace vulcan::dryden
