// Wind Shear Profiles
// Altitude-dependent wind profiles for boundary layer and low-level wind shear
#pragma once

#include <vulcan/wind/WindTypes.hpp>

namespace vulcan::wind_shear {

// ============================================================================
// Physical Constants
// ============================================================================

/// von Kármán constant for atmospheric boundary layer
inline constexpr double VON_KARMAN_CONSTANT = 0.41;

// ============================================================================
// Linear Wind Shear
// ============================================================================

/**
 * @brief Linear wind shear profile
 *
 * Wind = W_0 + (dW/dh) * (h - h_0)
 *
 * @tparam Scalar double or casadi::MX
 * @param altitude Altitude [m]
 * @param base_wind Wind at reference altitude [m/s]
 * @param base_altitude Reference altitude [m]
 * @param shear_rate Wind change per unit altitude [1/s], i.e., dV/dh
 * @return Wind speed at given altitude [m/s]
 */
template <typename Scalar>
Scalar linear(const Scalar &altitude, double base_wind, double base_altitude,
              double shear_rate) {
    return base_wind + shear_rate * (altitude - base_altitude);
}

/**
 * @brief Linear wind shear returning full wind vector
 *
 * Scales the base wind vector by the altitude-dependent factor.
 * Wind direction is preserved while magnitude changes linearly.
 *
 * @tparam Scalar double or casadi::MX
 * @param altitude Altitude [m]
 * @param base_wind Wind vector at reference altitude
 * @param base_altitude Reference altitude [m]
 * @param shear_rate Wind change per unit altitude [1/s]
 * @return WindVector scaled by altitude factor
 */
template <typename Scalar>
wind::WindVector<Scalar>
linear_vector(const Scalar &altitude, const wind::WindVector<double> &base_wind,
              double base_altitude, double shear_rate) {
    // Compute base wind speed (numeric)
    double base_speed = std::sqrt(base_wind.north * base_wind.north +
                                  base_wind.east * base_wind.east +
                                  base_wind.down * base_wind.down);

    // Avoid division by zero for calm conditions
    if (base_speed < 1e-10) {
        return wind::WindVector<Scalar>{.north = altitude * Scalar(0),
                                        .east = altitude * Scalar(0),
                                        .down = altitude * Scalar(0)};
    }

    // Scale factor based on altitude
    Scalar new_speed = linear(altitude, base_speed, base_altitude, shear_rate);
    Scalar scale = new_speed / base_speed;

    return wind::WindVector<Scalar>{.north = scale * base_wind.north,
                                    .east = scale * base_wind.east,
                                    .down = scale * base_wind.down};
}

// ============================================================================
// Power Law Wind Shear (Atmospheric Boundary Layer)
// ============================================================================

/**
 * @brief Power-law wind profile (typical for atmospheric boundary layer)
 *
 * V(h) = V_ref * (h / h_ref)^alpha
 *
 * Common exponents:
 * - α = 1/7 (0.143) - unstable/daytime conditions
 * - α = 1/4 (0.25) - neutral conditions
 * - α = 1/3 (0.33) - stable/nighttime conditions
 *
 * @tparam Scalar double or casadi::MX
 * @param altitude Altitude above ground [m]
 * @param ref_wind Wind speed at reference height [m/s]
 * @param ref_altitude Reference height [m] (typically 10m)
 * @param exponent Power law exponent α (default: 1/7 for unstable)
 * @return Wind speed at altitude [m/s]
 */
template <typename Scalar>
Scalar power_law(const Scalar &altitude, double ref_wind,
                 double ref_altitude = 10.0, double exponent = 1.0 / 7.0) {
    // V = V_ref * (h / h_ref)^alpha
    // Use janus::pow for symbolic compatibility
    return ref_wind * janus::pow(altitude / ref_altitude, exponent);
}

/**
 * @brief Power-law wind profile returning full wind vector
 *
 * @tparam Scalar double or casadi::MX
 * @param altitude Altitude above ground [m]
 * @param base_wind Wind vector at reference height
 * @param ref_altitude Reference height [m] (typically 10m)
 * @param exponent Power law exponent α
 * @return WindVector scaled by power law
 */
template <typename Scalar>
wind::WindVector<Scalar>
power_law_vector(const Scalar &altitude,
                 const wind::WindVector<double> &base_wind,
                 double ref_altitude = 10.0, double exponent = 1.0 / 7.0) {
    // Compute base wind speed (numeric)
    double base_speed = std::sqrt(base_wind.north * base_wind.north +
                                  base_wind.east * base_wind.east +
                                  base_wind.down * base_wind.down);

    if (base_speed < 1e-10) {
        return wind::WindVector<Scalar>{.north = altitude * Scalar(0),
                                        .east = altitude * Scalar(0),
                                        .down = altitude * Scalar(0)};
    }

    // Scale factor from power law
    Scalar scale = janus::pow(altitude / ref_altitude, exponent);

    return wind::WindVector<Scalar>{.north = scale * base_wind.north,
                                    .east = scale * base_wind.east,
                                    .down = scale * base_wind.down};
}

// ============================================================================
// Logarithmic Wind Profile
// ============================================================================

/**
 * @brief Logarithmic wind profile (neutral atmospheric boundary layer)
 *
 * V(h) = (u* / κ) * ln((h - d) / z_0)
 *
 * Where:
 * - u* = friction velocity [m/s]
 * - κ = von Kármán constant (≈ 0.41)
 * - d = zero-plane displacement [m] (for vegetation/urban areas)
 * - z_0 = roughness length [m]
 *
 * @tparam Scalar double or casadi::MX
 * @param altitude Altitude above ground [m]
 * @param friction_velocity Surface friction velocity u* [m/s]
 * @param roughness_length Surface roughness z_0 [m]
 * @param displacement Zero-plane displacement d [m] (default: 0)
 * @return Wind speed at altitude [m/s]
 */
template <typename Scalar>
Scalar logarithmic(const Scalar &altitude, double friction_velocity,
                   double roughness_length, double displacement = 0.0) {
    // V = (u* / κ) * ln((h - d) / z_0)
    Scalar effective_height = altitude - displacement;
    return (friction_velocity / VON_KARMAN_CONSTANT) *
           janus::log(effective_height / roughness_length);
}

/**
 * @brief Compute friction velocity from reference wind
 *
 * Given wind speed at a reference height, compute the friction velocity
 * for use in logarithmic profile calculations.
 *
 * u* = κ * V_ref / ln((h_ref - d) / z_0)
 *
 * @param ref_wind Wind speed at reference height [m/s]
 * @param ref_altitude Reference height [m]
 * @param roughness_length Surface roughness z_0 [m]
 * @param displacement Zero-plane displacement d [m]
 * @return Friction velocity u* [m/s]
 */
inline double friction_velocity_from_ref(double ref_wind, double ref_altitude,
                                         double roughness_length,
                                         double displacement = 0.0) {
    double effective_height = ref_altitude - displacement;
    return VON_KARMAN_CONSTANT * ref_wind /
           std::log(effective_height / roughness_length);
}

// ============================================================================
// Common Roughness Lengths
// ============================================================================

namespace roughness {
inline constexpr double OPEN_WATER = 0.0002; // [m]
inline constexpr double OPEN_TERRAIN = 0.03; // [m] - grass, few trees
inline constexpr double RURAL = 0.1;         // [m] - scattered buildings
inline constexpr double SUBURBAN = 0.5;      // [m] - low-rise urban
inline constexpr double URBAN = 1.0;         // [m] - city center
} // namespace roughness

// ============================================================================
// Common Power Law Exponents
// ============================================================================

namespace exponent {
inline constexpr double UNSTABLE =
    1.0 / 7.0;                               // ~0.143 - daytime, strong heating
inline constexpr double NEUTRAL = 1.0 / 4.0; // 0.25 - overcast, moderate wind
inline constexpr double STABLE = 1.0 / 3.0;  // ~0.333 - nighttime, light wind
} // namespace exponent

} // namespace vulcan::wind_shear
