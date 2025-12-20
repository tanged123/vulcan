// Constant Wind Field Model
// Simple constant wind vector in NED frame
#pragma once

#include <vulcan/wind/WindTypes.hpp>

namespace vulcan::constant_wind {

// ============================================================================
// Constant Wind from NED Components
// ============================================================================

/**
 * @brief Create constant wind from NED components
 *
 * @tparam Scalar double or casadi::MX
 * @param north North wind component [m/s]
 * @param east East wind component [m/s]
 * @param down Down wind component [m/s] (usually 0)
 * @return WindVector with constant components
 */
template <typename Scalar>
wind::WindVector<Scalar> from_ned(const Scalar &north, const Scalar &east,
                                  const Scalar &down = Scalar(0)) {
    return wind::WindVector<Scalar>{.north = north, .east = east, .down = down};
}

// ============================================================================
// Constant Wind from Speed and Direction
// ============================================================================

/**
 * @brief Create constant wind from speed and direction
 *
 * Uses meteorological convention: direction is where wind comes FROM.
 * For example, a "northerly wind" blows FROM the north TO the south,
 * so direction_from = 0 and the resulting north component is negative.
 *
 * @tparam Scalar double or casadi::MX
 * @param speed Wind speed [m/s]
 * @param direction_from Direction wind is blowing FROM [rad], clockwise from
 * North
 * @return WindVector in NED frame
 */
template <typename Scalar>
wind::WindVector<Scalar> from_speed_direction(const Scalar &speed,
                                              const Scalar &direction_from) {
    // Wind blows FROM direction_from, so velocity is in opposite direction
    // North component: -speed * cos(direction_from)
    // East component: -speed * sin(direction_from)
    return wind::WindVector<Scalar>{
        .north = -speed * janus::cos(direction_from),
        .east = -speed * janus::sin(direction_from),
        .down = speed * Scalar(0) // Zero down component, maintain scalar type
    };
}

} // namespace vulcan::constant_wind
