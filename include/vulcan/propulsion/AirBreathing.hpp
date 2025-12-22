#pragma once

#include <janus/janus.hpp>

namespace vulcan::propulsion::air_breathing {

/**
 * @brief Calculates fuel flow rate from thrust and TSFC.
 *
 * mdot = TSFC * F
 *
 * Note: Check unit consistency. If TSFC is [1/s] (weight specific),
 * the result is weight flow [N/s]. If TSFC is [kg/(N*s)] (mass specific),
 * the result is mass flow [kg/s].
 *
 * @tparam Scalar Variable type
 * @param thrust Thrust force [N]
 * @param TSFC Thrust Specific Fuel Consumption
 * @return Fuel flow rate
 */
template <typename Scalar>
Scalar fuel_flow_rate(const Scalar &thrust, const Scalar &TSFC) {
    return TSFC * thrust;
}

/**
 * @brief Calculates Breguet Range for jet aircraft.
 *
 * R = (V / TSFC) * (L / D) * ln(W0 / W1)
 *
 * Note: For result in distance units (e.g. meters), TSFC must be in [1/s]
 * (weight flow per thrust), or V/(TSFC*g) must be used if TSFC is
 * mass-specific. This implementation follows the plan: (V/TSFC).
 *
 * @tparam Scalar Variable type
 * @param velocity Flight velocity [m/s]
 * @param TSFC Thrust Specific Fuel Consumption [1/s]
 * @param L_D Lift-to-Drag ratio
 * @param W0 Initial weight
 * @param W1 Final weight
 * @return Range distance [m] (if TSFC is [1/s])
 */
template <typename Scalar>
Scalar breguet_range(const Scalar &velocity, const Scalar &TSFC,
                     const Scalar &L_D, const Scalar &W0, const Scalar &W1) {
    return (velocity / TSFC) * L_D * janus::log(W0 / W1);
}

/**
 * @brief Calculates Breguet Endurance for jet aircraft.
 *
 * E = (1 / TSFC) * (L / D) * ln(W0 / W1)
 *
 * @tparam Scalar Variable type
 * @param TSFC Thrust Specific Fuel Consumption [1/s]
 * @param L_D Lift-to-Drag ratio
 * @param W0 Initial weight
 * @param W1 Final weight
 * @return Endurance time [s] (if TSFC is [1/s])
 */
template <typename Scalar>
Scalar breguet_endurance(const Scalar &TSFC, const Scalar &L_D,
                         const Scalar &W0, const Scalar &W1) {
    return (1.0 / TSFC) * L_D * janus::log(W0 / W1);
}

} // namespace vulcan::propulsion::air_breathing
