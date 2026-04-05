#pragma once

#include <janus/janus.hpp>
#include <vulcan/quantity/Quantity.hpp>

namespace vulcan::propulsion::rocket {

using namespace vulcan::units;

/**
 * @brief Calculates thrust from mass flow rate and effective exhaust velocity.
 *
 * F = mdot * Ve
 *
 * @tparam Scalar Variable type (e.g., double or casadi::MX)
 * @param mdot Mass flow rate [kg/s]
 * @param Ve Effective exhaust velocity [m/s]
 * @return Thrust [N]
 */
template <typename Scalar>
Quantity<N, Scalar> thrust_from_mdot(Quantity<kg_per_s, Scalar> mdot,
                                     Quantity<mps, Scalar> Ve) {
    return Quantity<N, Scalar>{mdot.value() * Ve.value()};
}

/**
 * @brief Calculates effective exhaust velocity from specific impulse.
 *
 * Ve = Isp * g0
 *
 * @tparam Scalar Variable type
 * @param Isp Specific impulse [s]
 * @param g0 Standard gravity [m/s^2] (default: 9.80665)
 * @return Effective exhaust velocity [m/s]
 */
template <typename Scalar>
Quantity<mps, Scalar> exhaust_velocity(Quantity<s, Scalar> Isp,
                                       double g0 = 9.80665) {
    return Quantity<mps, Scalar>{Isp.value() * g0};
}

/**
 * @brief Calculates specific impulse from thrust, mass flow rate, and gravity.
 *
 * Isp = F / (mdot * g0)
 *
 * @tparam Scalar Variable type
 * @param thrust Thrust force [N]
 * @param mdot Mass flow rate [kg/s]
 * @param g0 Standard gravity [m/s^2] (default: 9.80665)
 * @return Specific impulse [s]
 */
template <typename Scalar>
Quantity<s, Scalar> specific_impulse(Quantity<N, Scalar> thrust,
                                     Quantity<kg_per_s, Scalar> mdot,
                                     double g0 = 9.80665) {
    return Quantity<s, Scalar>{thrust.value() / (mdot.value() * g0)};
}

/**
 * @brief Calculates Delta-V using the Tsiolkovsky rocket equation.
 *
 * dV = Ve * ln(m0 / mf)
 *
 * @tparam Scalar Variable type
 * @param Ve Effective exhaust velocity [m/s]
 * @param m0 Initial mass [kg]
 * @param mf Final mass [kg]
 * @return Delta-V [m/s]
 */
template <typename Scalar>
Quantity<mps, Scalar> delta_v(Quantity<mps, Scalar> Ve, Quantity<kg, Scalar> m0,
                              Quantity<kg, Scalar> mf) {
    return Quantity<mps, Scalar>{Ve.value() *
                                 janus::log(m0.value() / mf.value())};
}

/**
 * @brief Calculates required propellant mass for a given Delta-V.
 *
 * mp = m0 * (1 - exp(-dV / Ve))
 *
 * @tparam Scalar Variable type
 * @param delta_v Target Delta-V [m/s]
 * @param m0 Initial mass [kg]
 * @param Ve Effective exhaust velocity [m/s]
 * @return Propellant mass required [kg]
 */
template <typename Scalar>
Quantity<kg, Scalar> propellant_mass(Quantity<mps, Scalar> delta_v,
                                     Quantity<kg, Scalar> m0,
                                     Quantity<mps, Scalar> Ve) {
    return Quantity<kg, Scalar>{
        m0.value() * (1.0 - janus::exp(-delta_v.value() / Ve.value()))};
}

/**
 * @brief Calculates mass flow rate from thrust and exhaust velocity.
 *
 * mdot = F / Ve
 *
 * @tparam Scalar Variable type
 * @param thrust Thrust force [N]
 * @param Ve Effective exhaust velocity [m/s]
 * @return Mass flow rate [kg/s]
 */
template <typename Scalar>
Quantity<kg_per_s, Scalar> mass_flow_rate(Quantity<N, Scalar> thrust,
                                          Quantity<mps, Scalar> Ve) {
    return Quantity<kg_per_s, Scalar>{thrust.value() / Ve.value()};
}

/**
 * @brief Calculates burn time for a given propellant mass and flow rate.
 *
 * t = mp / mdot
 *
 * @tparam Scalar Variable type
 * @param propellant_mass Mass of propellant to burn [kg]
 * @param mdot Mass flow rate [kg/s]
 * @return Burn time [s]
 */
template <typename Scalar>
Quantity<s, Scalar> burn_time(Quantity<kg, Scalar> propellant_mass,
                              Quantity<kg_per_s, Scalar> mdot) {
    return Quantity<s, Scalar>{propellant_mass.value() / mdot.value()};
}

} // namespace vulcan::propulsion::rocket
