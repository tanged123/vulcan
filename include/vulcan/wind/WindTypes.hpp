// Wind Model Types and Utilities
// Common types for all wind models - MIL-F-8785C / MIL-HDBK-1797 compliant
#pragma once

#include <janus/janus.hpp>

namespace vulcan::wind {

// ============================================================================
// Wind Vector Representation
// ============================================================================

/**
 * @brief 3D wind velocity in NED frame
 *
 * Wind components represent the velocity of the air mass relative to the Earth.
 * Positive North = wind blowing from South to North
 * Positive East = wind blowing from West to East
 * Positive Down = wind blowing upward (rare, usually near-zero)
 */
template <typename Scalar> struct WindVector {
    Scalar north; ///< North component [m/s]
    Scalar east;  ///< East component [m/s]
    Scalar down;  ///< Down component [m/s]

    /// Magnitude in horizontal plane
    Scalar horizontal_speed() const {
        return janus::sqrt(north * north + east * east);
    }

    /// Total magnitude
    Scalar speed() const {
        return janus::sqrt(north * north + east * east + down * down);
    }

    /// Direction wind is coming FROM (meteorological convention)
    /// Returns angle in radians from North, clockwise positive
    Scalar direction_from() const { return janus::atan2(east, north); }
};

/**
 * @brief Turbulent gust velocities in body frame
 *
 * Linear gust components in aircraft body-fixed frame (uvw convention).
 */
template <typename Scalar> struct GustVelocity {
    Scalar u_g; ///< Longitudinal gust [m/s] (along body x-axis)
    Scalar v_g; ///< Lateral gust [m/s] (along body y-axis)
    Scalar w_g; ///< Vertical gust [m/s] (along body z-axis)
};

/**
 * @brief Angular gust rates in body frame
 *
 * Angular velocity perturbations caused by spatial gradients of linear gusts.
 */
template <typename Scalar> struct GustAngularRate {
    Scalar p_g; ///< Roll gust rate [rad/s]
    Scalar q_g; ///< Pitch gust rate [rad/s]
    Scalar r_g; ///< Yaw gust rate [rad/s]
};

// ============================================================================
// Turbulence Parameters
// ============================================================================

/**
 * @brief Turbulence intensity and scale parameters (MIL-F-8785C)
 *
 * @tparam Scalar double for numeric evaluation, or casadi::MX for symbolic
 *         optimization of turbulence parameters
 */
template <typename Scalar = double> struct TurbulenceParams {
    Scalar sigma_u; ///< Longitudinal RMS intensity [m/s]
    Scalar sigma_v; ///< Lateral RMS intensity [m/s]
    Scalar sigma_w; ///< Vertical RMS intensity [m/s]
    Scalar L_u;     ///< Longitudinal scale length [m]
    Scalar L_v;     ///< Lateral scale length [m]
    Scalar L_w;     ///< Vertical scale length [m]
};

/**
 * @brief Turbulence severity levels per MIL-HDBK-1797
 */
enum class TurbulenceSeverity {
    Light,    ///< σ_w ≈ 1 m/s at low altitude
    Moderate, ///< σ_w ≈ 3 m/s at low altitude
    Severe    ///< σ_w ≈ 7 m/s at low altitude
};

// ============================================================================
// MIL-Spec Parameter Calculation
// ============================================================================

namespace detail {

/// Wind speed at 20 ft for different turbulence severities (MIL-HDBK-1797)
inline constexpr double W20_LIGHT = 7.5;     // [m/s] (~15 knots)
inline constexpr double W20_MODERATE = 15.0; // [m/s] (~30 knots)
inline constexpr double W20_SEVERE = 30.0;   // [m/s] (~60 knots)

} // namespace detail

/**
 * @brief Compute MIL-spec turbulence parameters for given conditions
 *
 * Implements the turbulence intensity and scale length equations from
 * MIL-F-8785C and MIL-HDBK-1797.
 *
 * For low altitude (h < 1000 ft / 304.8 m):
 * - L_w = h
 * - L_u = L_v = h / (0.177 + 0.000823*h)^1.2
 * - σ_w = 0.1 * W_20
 * - σ_u = σ_w / (0.177 + 0.000823*h)^0.4
 * - σ_v = σ_u
 *
 * For medium/high altitude (h >= 1000 ft / 304.8 m):
 * - L_u = L_v = L_w = 533.4 m (1750 ft)
 * - σ_u = σ_v = σ_w (from probability of exceedance)
 *
 * @param altitude Altitude above ground level [m]
 * @param severity Turbulence severity level
 * @return TurbulenceParams with appropriate σ and L values
 */
inline TurbulenceParams<double> mil_spec_params(double altitude,
                                                TurbulenceSeverity severity) {
    // Convert altitude to feet for MIL-spec equations
    constexpr double M_TO_FT = 3.28084;
    constexpr double FT_TO_M = 0.3048;
    double h_ft = altitude * M_TO_FT;

    // Wind speed at 20 ft based on severity
    double W20 = 0.0;
    switch (severity) {
    case TurbulenceSeverity::Light:
        W20 = detail::W20_LIGHT;
        break;
    case TurbulenceSeverity::Moderate:
        W20 = detail::W20_MODERATE;
        break;
    case TurbulenceSeverity::Severe:
        W20 = detail::W20_SEVERE;
        break;
    }

    TurbulenceParams<double> params;

    if (h_ft < 1000.0) {
        // Low altitude model
        // Ensure minimum altitude of 10 ft for numerical stability
        h_ft = std::max(h_ft, 10.0);

        // Scale lengths (in feet, then convert)
        double denom = std::pow(0.177 + 0.000823 * h_ft, 1.2);
        params.L_w = h_ft * FT_TO_M;
        params.L_u = (h_ft / denom) * FT_TO_M;
        params.L_v = params.L_u;

        // Turbulence intensities
        params.sigma_w = 0.1 * W20;
        double denom2 = std::pow(0.177 + 0.000823 * h_ft, 0.4);
        params.sigma_u = params.sigma_w / denom2;
        params.sigma_v = params.sigma_u;
    } else {
        // Medium/high altitude model
        // Scale lengths are constant at 1750 ft = 533.4 m
        params.L_u = 533.4;
        params.L_v = 533.4;
        params.L_w = 533.4;

        // Intensities based on probability of exceedance
        // Using simplified model: σ increases with severity
        switch (severity) {
        case TurbulenceSeverity::Light:
            params.sigma_u = params.sigma_v = params.sigma_w = 1.5;
            break;
        case TurbulenceSeverity::Moderate:
            params.sigma_u = params.sigma_v = params.sigma_w = 3.0;
            break;
        case TurbulenceSeverity::Severe:
            params.sigma_u = params.sigma_v = params.sigma_w = 7.0;
            break;
        }
    }

    return params;
}

} // namespace vulcan::wind
