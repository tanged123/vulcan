// Vulcan Gravity Types
// Shared types for gravity models
#pragma once

#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::gravity {

/**
 * @brief Gravitational state vector
 *
 * Contains both acceleration and potential for efficiency when both are needed.
 */
template <typename Scalar> struct GravityState {
    Vec3<Scalar> acceleration; ///< Gravitational acceleration [m/s²]
    Scalar potential;          ///< Gravitational potential [m²/s²]
};

} // namespace vulcan::gravity
