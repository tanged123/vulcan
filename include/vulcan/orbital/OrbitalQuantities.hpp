// Vulcan Orbital Quantities
// Two-body orbital mechanics computations
#pragma once

#include <cmath>
#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::orbital::quantities {

/**
 * @brief Orbital period
 *
 * @tparam Scalar double or casadi::MX
 * @param a Semi-major axis [m]
 * @param mu Gravitational parameter [m³/s²] (default: Earth)
 * @return Period [s]
 */
template <typename Scalar>
Scalar period(const Scalar &a, double mu = constants::earth::mu) {
    return 2.0 * M_PI * janus::sqrt(a * a * a / mu);
}

/**
 * @brief Orbital velocity (vis-viva equation)
 *
 * @tparam Scalar double or casadi::MX
 * @param r Current radius [m]
 * @param a Semi-major axis [m]
 * @param mu Gravitational parameter [m³/s²] (default: Earth)
 * @return Velocity magnitude [m/s]
 */
template <typename Scalar>
Scalar velocity(const Scalar &r, const Scalar &a,
                double mu = constants::earth::mu) {
    return janus::sqrt(mu * (2.0 / r - 1.0 / a));
}

/**
 * @brief Specific orbital energy
 *
 * @tparam Scalar double or casadi::MX
 * @param a Semi-major axis [m]
 * @param mu Gravitational parameter [m³/s²] (default: Earth)
 * @return Specific energy [J/kg]
 */
template <typename Scalar>
Scalar energy(const Scalar &a, double mu = constants::earth::mu) {
    return -mu / (2.0 * a);
}

/**
 * @brief Escape velocity at given radius
 *
 * @tparam Scalar double or casadi::MX
 * @param r Radius [m]
 * @param mu Gravitational parameter [m³/s²] (default: Earth)
 * @return Escape velocity [m/s]
 */
template <typename Scalar>
Scalar escape_velocity(const Scalar &r, double mu = constants::earth::mu) {
    return janus::sqrt(2.0 * mu / r);
}

/**
 * @brief Circular orbit velocity at given radius
 *
 * @tparam Scalar double or casadi::MX
 * @param r Radius [m]
 * @param mu Gravitational parameter [m³/s²] (default: Earth)
 * @return Circular velocity [m/s]
 */
template <typename Scalar>
Scalar circular_velocity(const Scalar &r, double mu = constants::earth::mu) {
    return janus::sqrt(mu / r);
}

/**
 * @brief Mean motion
 *
 * @tparam Scalar double or casadi::MX
 * @param a Semi-major axis [m]
 * @param mu Gravitational parameter [m³/s²] (default: Earth)
 * @return Mean motion [rad/s]
 */
template <typename Scalar>
Scalar mean_motion(const Scalar &a, double mu = constants::earth::mu) {
    return janus::sqrt(mu / (a * a * a));
}

/**
 * @brief Semi-latus rectum
 *
 * @tparam Scalar double or casadi::MX
 * @param a Semi-major axis [m]
 * @param e Eccentricity [-]
 * @return Semi-latus rectum [m]
 */
template <typename Scalar>
Scalar semi_latus_rectum(const Scalar &a, const Scalar &e) {
    return a * (1.0 - e * e);
}

/**
 * @brief Radius at given true anomaly
 *
 * @tparam Scalar double or casadi::MX
 * @param a Semi-major axis [m]
 * @param e Eccentricity [-]
 * @param nu True anomaly [rad]
 * @return Radius [m]
 */
template <typename Scalar>
Scalar radius_at_anomaly(const Scalar &a, const Scalar &e, const Scalar &nu) {
    return a * (1.0 - e * e) / (1.0 + e * janus::cos(nu));
}

} // namespace vulcan::orbital::quantities
