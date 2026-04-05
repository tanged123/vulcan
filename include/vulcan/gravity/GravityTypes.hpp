// Vulcan Gravity Types
// Shared types for gravity models
#pragma once

#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/QuantityEigen.hpp>
#include <vulcan/quantity/Units.hpp>

namespace vulcan::gravity {

/**
 * @brief Gravitational state vector
 *
 * Contains both acceleration and potential for efficiency when both are needed.
 * Units are enforced at the type level via Quantity.
 */
template <typename Scalar> struct GravityState {
    Vec3<Quantity<units::m / (units::s * units::s), Scalar>>
        acceleration; ///< Gravitational acceleration [m/s²]
    Quantity<units::m * units::m / (units::s * units::s), Scalar>
        potential; ///< Gravitational potential [m²/s²]
};

} // namespace vulcan::gravity
