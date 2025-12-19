// Exponential Atmosphere Model
// Simple exponential atmosphere for quick estimates and validation
#pragma once

#include <janus/janus.hpp>

namespace vulcan::exponential_atmosphere {

// ============================================================================
// Physical Constants
// ============================================================================

/// Sea-level reference density [kg/m³]
inline constexpr double RHO_0 = 1.225;

/// Sea-level reference pressure [Pa]
inline constexpr double P_0 = 101325.0;

/// Sea-level reference temperature [K]
inline constexpr double T_0 = 288.15;

/// Default atmospheric scale height [m]
/// Approximately 8.5 km based on RT/g at sea level
inline constexpr double DEFAULT_SCALE_HEIGHT = 8500.0;

/// Sea-level gravitational acceleration [m/s²]
inline constexpr double G_0 = 9.80665;

/// Specific gas constant for air [J/(kg·K)]
inline constexpr double R_AIR = 287.058;

/// Ratio of specific heats for air (γ = Cp/Cv)
inline constexpr double GAMMA = 1.4;

// ============================================================================
// Atmospheric State Struct
// ============================================================================

/**
 * @brief Complete atmospheric state at a given altitude
 *
 * Contains all atmospheric properties computed in a single evaluation.
 * Efficient for trajectory optimization where multiple properties are needed.
 *
 * @tparam Scalar double or casadi::MX for symbolic computation
 */
template <typename Scalar> struct AtmosphericState {
    Scalar temperature;    ///< Temperature [K]
    Scalar pressure;       ///< Pressure [Pa]
    Scalar density;        ///< Air density [kg/m³]
    Scalar speed_of_sound; ///< Speed of sound [m/s]
};

// ============================================================================
// Public API - Individual Property Accessors
// ============================================================================

/**
 * @brief Exponential atmosphere - Density
 *
 * Computes air density using the simple exponential model:
 * ρ = ρ₀ * exp(-h / H)
 *
 * This model is suitable for quick estimates and trajectory optimization
 * where a simple, smooth, differentiable model is preferred.
 *
 * @tparam Scalar double or casadi::MX for symbolic computation
 * @param altitude Geometric altitude above sea level [m]
 * @param scale_height Atmospheric scale height [m] (default: 8500 m)
 * @return Density [kg/m³]
 *
 * @note For altitudes below 0, density will be greater than ρ₀.
 *       For very high altitudes, density approaches 0.
 */
template <typename Scalar>
Scalar density(const Scalar &altitude,
               double scale_height = DEFAULT_SCALE_HEIGHT) {
    return RHO_0 * janus::exp(-altitude / scale_height);
}

/**
 * @brief Exponential atmosphere - Pressure
 *
 * Computes pressure using the simple exponential model:
 * P = P₀ * exp(-h / H)
 *
 * Uses the same scale height as density for consistency with
 * an isothermal atmosphere assumption.
 *
 * @tparam Scalar double or casadi::MX for symbolic computation
 * @param altitude Geometric altitude above sea level [m]
 * @param scale_height Atmospheric scale height [m] (default: 8500 m)
 * @return Pressure [Pa]
 */
template <typename Scalar>
Scalar pressure(const Scalar &altitude,
                double scale_height = DEFAULT_SCALE_HEIGHT) {
    return P_0 * janus::exp(-altitude / scale_height);
}

/**
 * @brief Exponential atmosphere - Temperature
 *
 * Returns the reference temperature (isothermal assumption).
 * The exponential atmosphere model assumes constant temperature,
 * which is why both density and pressure decay with the same scale height.
 *
 * @tparam Scalar double or casadi::MX for symbolic computation
 * @param altitude Geometric altitude above sea level [m] (unused)
 * @param scale_height Atmospheric scale height [m] (unused)
 * @return Temperature [K] - always returns T_0 = 288.15 K
 */
template <typename Scalar>
Scalar temperature(const Scalar &altitude,
                   double scale_height = DEFAULT_SCALE_HEIGHT) {
    // Isothermal assumption - return constant temperature
    // Use janus multiplication to ensure proper type for symbolic inputs
    (void)scale_height; // Unused
    return altitude * 0.0 + T_0;
}

/**
 * @brief Exponential atmosphere - Speed of Sound
 *
 * Computes speed of sound from temperature:
 * a = sqrt(γ * R * T)
 *
 * Since temperature is constant in this model, speed of sound is also constant.
 *
 * @tparam Scalar double or casadi::MX for symbolic computation
 * @param altitude Geometric altitude above sea level [m] (unused)
 * @param scale_height Atmospheric scale height [m] (unused)
 * @return Speed of sound [m/s] - constant at ~340.3 m/s
 */
template <typename Scalar>
Scalar speed_of_sound(const Scalar &altitude,
                      double scale_height = DEFAULT_SCALE_HEIGHT) {
    // a = sqrt(gamma * R * T)
    // For isothermal atmosphere, this is constant
    Scalar T = temperature(altitude, scale_height);
    return janus::sqrt(GAMMA * R_AIR * T);
}

// ============================================================================
// Public API - Combined State Accessor
// ============================================================================

/**
 * @brief Exponential atmosphere - Complete atmospheric state
 *
 * Returns all atmospheric properties in a single evaluation.
 * Efficient for trajectory optimization where multiple properties are needed.
 *
 * @tparam Scalar double or casadi::MX for symbolic computation
 * @param altitude Geometric altitude above sea level [m]
 * @param scale_height Atmospheric scale height [m] (default: 8500 m)
 * @return AtmosphericState containing T, P, ρ, a
 */
template <typename Scalar>
AtmosphericState<Scalar> state(const Scalar &altitude,
                               double scale_height = DEFAULT_SCALE_HEIGHT) {
    Scalar T = temperature(altitude, scale_height);
    Scalar exp_factor = janus::exp(-altitude / scale_height);

    return AtmosphericState<Scalar>{.temperature = T,
                                    .pressure = P_0 * exp_factor,
                                    .density = RHO_0 * exp_factor,
                                    .speed_of_sound =
                                        janus::sqrt(GAMMA * R_AIR * T)};
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Compute scale height from reference conditions
 *
 * The scale height H = R*T / g relates temperature, gravity,
 * and the rate of pressure decay.
 *
 * @param temperature Reference temperature [K]
 * @param gravity Gravitational acceleration [m/s²]
 * @return Scale height [m]
 */
inline double compute_scale_height(double temperature = T_0,
                                   double gravity = G_0) {
    return R_AIR * temperature / gravity;
}

/**
 * @brief Compute altitude from density
 *
 * Inverse of the density function:
 * h = -H * ln(ρ / ρ₀)
 *
 * @tparam Scalar double or casadi::MX for symbolic computation
 * @param rho Density [kg/m³]
 * @param scale_height Atmospheric scale height [m] (default: 8500 m)
 * @return Altitude [m]
 */
template <typename Scalar>
Scalar altitude_from_density(const Scalar &rho,
                             double scale_height = DEFAULT_SCALE_HEIGHT) {
    return -scale_height * janus::log(rho / RHO_0);
}

/**
 * @brief Compute altitude from pressure
 *
 * Inverse of the pressure function:
 * h = -H * ln(P / P₀)
 *
 * @tparam Scalar double or casadi::MX for symbolic computation
 * @param P Pressure [Pa]
 * @param scale_height Atmospheric scale height [m] (default: 8500 m)
 * @return Altitude [m]
 */
template <typename Scalar>
Scalar altitude_from_pressure(const Scalar &P,
                              double scale_height = DEFAULT_SCALE_HEIGHT) {
    return -scale_height * janus::log(P / P_0);
}

} // namespace vulcan::exponential_atmosphere
