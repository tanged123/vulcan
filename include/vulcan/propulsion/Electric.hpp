#pragma once

#include <janus/janus.hpp>
#include <vulcan/quantity/Quantity.hpp>

namespace vulcan::propulsion::electric {

using namespace vulcan::units;

/**
 * @brief Calculates thrust for power-limited propulsion.
 *
 * Derived from P = F * Ve / (2 * efficiency)
 * => F = 2 * efficiency * P / Ve
 *
 * @tparam Scalar Variable type
 * @param power Input power [W]
 * @param Ve Effective exhaust velocity [m/s]
 * @param efficiency Thruster efficiency (0 to 1) [dimensionless]
 * @return Thrust [N]
 */
template <typename Scalar>
Quantity<N, Scalar>
thrust_from_power(Quantity<W, Scalar> power, Quantity<mps, Scalar> Ve,
                  Quantity<dimensionless, Scalar> efficiency) {
    return Quantity<N, Scalar>{2.0 * efficiency.value() * power.value() /
                               Ve.value()};
}

/**
 * @brief Calculates mass flow rate for power-limited propulsion.
 *
 * mdot = 2 * P * efficiency / Ve^2
 *
 * @tparam Scalar Variable type
 * @param power Input power [W]
 * @param Ve Effective exhaust velocity [m/s]
 * @param efficiency Thruster efficiency (0 to 1) [dimensionless]
 * @return Mass flow rate [kg/s]
 */
template <typename Scalar>
Quantity<kg_per_s, Scalar>
mass_flow_from_power(Quantity<W, Scalar> power, Quantity<mps, Scalar> Ve,
                     Quantity<dimensionless, Scalar> efficiency) {
    return Quantity<kg_per_s, Scalar>{
        (2.0 * power.value() * efficiency.value()) / (Ve.value() * Ve.value())};
}

/**
 * @brief Calculates characteristic velocity (exhaust velocity) from power and
 * mass flow.
 *
 * c* = sqrt(2 * efficiency * P / mdot)
 *
 * @tparam Scalar Variable type
 * @param power Input power [W]
 * @param efficiency Thruster efficiency (0 to 1) [dimensionless]
 * @param mdot Mass flow rate [kg/s]
 * @return Characteristic velocity (Exhaust Velocity) [m/s]
 */
template <typename Scalar>
Quantity<mps, Scalar>
characteristic_velocity(Quantity<W, Scalar> power,
                        Quantity<dimensionless, Scalar> efficiency,
                        Quantity<kg_per_s, Scalar> mdot) {
    return Quantity<mps, Scalar>{
        janus::sqrt(2.0 * efficiency.value() * power.value() / mdot.value())};
}

} // namespace vulcan::propulsion::electric
