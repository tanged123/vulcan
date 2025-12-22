#pragma once

#include <janus/janus.hpp>

namespace vulcan::propulsion::rocket {

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
Scalar thrust_from_mdot(const Scalar &mdot, const Scalar &Ve) {
    return mdot * Ve;
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
Scalar exhaust_velocity(const Scalar &Isp, double g0 = 9.80665) {
    return Isp * g0;
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
Scalar specific_impulse(const Scalar &thrust, const Scalar &mdot,
                        double g0 = 9.80665) {
    return thrust / (mdot * g0);
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
Scalar delta_v(const Scalar &Ve, const Scalar &m0, const Scalar &mf) {
    return Ve * janus::log(m0 / mf);
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
Scalar propellant_mass(const Scalar &delta_v, const Scalar &m0,
                       const Scalar &Ve) {
    return m0 * (1.0 - janus::exp(-delta_v / Ve));
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
Scalar mass_flow_rate(const Scalar &thrust, const Scalar &Ve) {
    return thrust / Ve;
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
Scalar burn_time(const Scalar &propellant_mass, const Scalar &mdot) {
    return propellant_mass / mdot;
}

} // namespace vulcan::propulsion::rocket
