#pragma once

#include <janus/janus.hpp>
#include <vulcan/quantity/Quantity.hpp>

namespace vulcan::propulsion::air_breathing {

using namespace vulcan::units;

/**
 * @brief Calculates fuel flow rate from thrust and TSFC.
 *
 * mdot = TSFC * F
 *
 * Note: If TSFC is [kg/(N*s)] (mass specific), the result is mass flow [kg/s].
 * Here we model TSFC as [1/s] (weight specific) and the result as [kg/s]
 * for dimensional consistency with downstream usage.
 *
 * @tparam Scalar Variable type
 * @param thrust Thrust force [N]
 * @param TSFC Thrust Specific Fuel Consumption [1/s]
 * @return Fuel flow rate [kg/s]
 */
template <typename Scalar>
Quantity<kg_per_s, Scalar> fuel_flow_rate(Quantity<N, Scalar> thrust,
                                          Quantity<per_s, Scalar> TSFC) {
    return Quantity<kg_per_s, Scalar>{TSFC.value() * thrust.value()};
}

/**
 * @brief Calculates Breguet Range for jet aircraft.
 *
 * R = (V / TSFC) * (L / D) * ln(W0 / W1)
 *
 * @tparam Scalar Variable type
 * @param velocity Flight velocity [m/s]
 * @param TSFC Thrust Specific Fuel Consumption [1/s]
 * @param L_D Lift-to-Drag ratio [dimensionless]
 * @param W0 Initial weight [N]
 * @param W1 Final weight [N]
 * @return Range distance [m]
 */
template <typename Scalar>
Quantity<m, Scalar>
breguet_range(Quantity<mps, Scalar> velocity, Quantity<per_s, Scalar> TSFC,
              Quantity<dimensionless, Scalar> L_D, Quantity<N, Scalar> W0,
              Quantity<N, Scalar> W1) {
    return Quantity<m, Scalar>{(velocity.value() / TSFC.value()) * L_D.value() *
                               janus::log(W0.value() / W1.value())};
}

/**
 * @brief Calculates Breguet Endurance for jet aircraft.
 *
 * E = (1 / TSFC) * (L / D) * ln(W0 / W1)
 *
 * @tparam Scalar Variable type
 * @param TSFC Thrust Specific Fuel Consumption [1/s]
 * @param L_D Lift-to-Drag ratio [dimensionless]
 * @param W0 Initial weight [N]
 * @param W1 Final weight [N]
 * @return Endurance time [s]
 */
template <typename Scalar>
Quantity<s, Scalar> breguet_endurance(Quantity<per_s, Scalar> TSFC,
                                      Quantity<dimensionless, Scalar> L_D,
                                      Quantity<N, Scalar> W0,
                                      Quantity<N, Scalar> W1) {
    return Quantity<s, Scalar>{(1.0 / TSFC.value()) * L_D.value() *
                               janus::log(W0.value() / W1.value())};
}

} // namespace vulcan::propulsion::air_breathing
