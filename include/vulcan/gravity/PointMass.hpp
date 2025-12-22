// Vulcan Point Mass Gravity Model
// Simple inverse-square gravitational acceleration
#pragma once

#include <type_traits>
#include <vulcan/core/VulcanError.hpp>

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::gravity::point_mass {

/**
 * @brief Point mass gravitational acceleration
 *
 * The simplest gravity model treating Earth as a uniform sphere.
 *
 * g = -μ/r³ · r_vec
 *
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF frame [m]
 * @param mu Gravitational parameter [m³/s²] (default: Earth)
 * @return Gravitational acceleration in ECEF [m/s²]
 * @throws GravityError if r_mag < 1.0 (numeric mode only)
 */
template <typename Scalar>
Vec3<Scalar> acceleration(const Vec3<Scalar> &r_ecef,
                          double mu = constants::earth::mu) {
    const Scalar r_mag = janus::norm(r_ecef);

    // Singularity check: position inside Earth's center
    if constexpr (std::is_same_v<Scalar, double>) {
        if (r_mag < 1.0) {
            throw GravityError("point_mass::acceleration: position magnitude < "
                               "1m (singularity)");
        }
    }

    const Scalar r_cubed = r_mag * r_mag * r_mag;

    // g = -μ/r³ · r
    return -mu / r_cubed * r_ecef;
}

/**
 * @brief Point mass gravitational potential
 *
 * U = -μ/r
 *
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF frame [m]
 * @param mu Gravitational parameter [m³/s²]
 * @return Gravitational potential [m²/s²]
 * @throws GravityError if r_mag < 1.0 (numeric mode only)
 */
template <typename Scalar>
Scalar potential(const Vec3<Scalar> &r_ecef, double mu = constants::earth::mu) {
    const Scalar r_mag = janus::norm(r_ecef);

    if constexpr (std::is_same_v<Scalar, double>) {
        if (r_mag < 1.0) {
            throw GravityError(
                "point_mass::potential: position magnitude < 1m (singularity)");
        }
    }

    return -mu / r_mag;
}

/**
 * @brief Gravitational acceleration magnitude at distance r
 *
 * |g| = μ/r²
 *
 * Convenience function when only magnitude is needed.
 *
 * @tparam Scalar double or casadi::MX
 * @param r_mag Radial distance from center [m]
 * @param mu Gravitational parameter [m³/s²]
 * @return Acceleration magnitude [m/s²]
 * @throws GravityError if r_mag < 1.0 (numeric mode only)
 */
template <typename Scalar>
Scalar acceleration_magnitude(const Scalar &r_mag,
                              double mu = constants::earth::mu) {
    if constexpr (std::is_same_v<Scalar, double>) {
        if (r_mag < 1.0) {
            throw GravityError("point_mass::acceleration_magnitude: distance < "
                               "1m (singularity)");
        }
    }

    return mu / (r_mag * r_mag);
}

} // namespace vulcan::gravity::point_mass
