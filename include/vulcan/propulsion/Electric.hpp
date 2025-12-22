#pragma once

#include <janus/janus.hpp>

namespace vulcan::propulsion::electric {

/**
 * @brief Calculates thrust for power-limited propulsion.
 *
 * Derived from P = F * Ve / (2 * efficiency)
 * => F = 2 * efficiency * P / Ve
 *
 * @tparam Scalar Variable type
 * @param power Input power [W]
 * @param Ve Effective exhaust velocity [m/s]
 * @param efficiency Thruster efficiency (0 to 1)
 * @return Thrust [N]
 */
template <typename Scalar>
Scalar thrust_from_power(const Scalar &power, const Scalar &Ve,
                         const Scalar &efficiency) {
    return 2.0 * efficiency * power / Ve;
}

/**
 * @brief Calculates mass flow rate for power-limited propulsion.
 *
 * mdot = 2 * P * efficiency / Ve^2
 *
 * @tparam Scalar Variable type
 * @param power Input power [W]
 * @param Ve Effective exhaust velocity [m/s]
 * @param efficiency Thruster efficiency (0 to 1)
 * @return Mass flow rate [kg/s]
 */
template <typename Scalar>
Scalar mass_flow_from_power(const Scalar &power, const Scalar &Ve,
                            const Scalar &efficiency) {
    return (2.0 * power * efficiency) / (Ve * Ve);
}

/**
 * @brief Calculates characteristic velocity (exhaust velocity) from power and
 * mass flow.
 *
 * c* = sqrt(2 * efficiency * P / mdot)
 *
 * @tparam Scalar Variable type
 * @param power Input power [W]
 * @param efficiency Thruster efficiency (0 to 1)
 * @param mdot Mass flow rate [kg/s]
 * @return Characteristic velocity (Exhaust Velocity) [m/s]
 */
template <typename Scalar>
Scalar characteristic_velocity(const Scalar &power, const Scalar &efficiency,
                               const Scalar &mdot) {
    return janus::sqrt(2.0 * efficiency * power / mdot);
}

} // namespace vulcan::propulsion::electric
