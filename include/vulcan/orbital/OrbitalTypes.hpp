// Vulcan Orbital Mechanics Types
// Core types and constants for orbital mechanics computations
#pragma once

#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::orbital {

// =============================================================================
// Orbital Elements Structure
// =============================================================================

/**
 * @brief Classical Keplerian orbital elements
 *
 * Standard six-element representation of an orbit in the two-body problem.
 * All angles are in radians.
 */
template <typename Scalar> struct OrbitalElements {
    Scalar a;     ///< Semi-major axis [m]
    Scalar e;     ///< Eccentricity [-]
    Scalar i;     ///< Inclination [rad]
    Scalar Omega; ///< Right ascension of ascending node (RAAN) [rad]
    Scalar omega; ///< Argument of periapsis [rad]
    Scalar nu;    ///< True anomaly [rad]
};

} // namespace vulcan::orbital
