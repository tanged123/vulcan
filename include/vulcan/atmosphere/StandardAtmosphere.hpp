// Vulcan Standard Atmosphere
// US Standard Atmosphere 1976 implementation
//
// NOTE: This file contains the LEGACY analytical model
// (vulcan::standard_atmosphere). For vetted table-based data, use vulcan::us76
// from <vulcan/atmosphere/US76Atmosphere.hpp>
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>

namespace vulcan::standard_atmosphere {

/**
 * @brief US Standard Atmosphere 1976 - Temperature
 *
 * @tparam Scalar Type for scalar operations (double or casadi::MX)
 * @param altitude Geometric altitude [m]
 * @return Temperature [K]
 */
template <typename Scalar> Scalar temperature(const Scalar &altitude) {
    // Constants (structural - OK to use as-is)
    constexpr double T0 = constants::atmosphere::T0;
    constexpr double L = constants::atmosphere::L;
    constexpr double h_tropopause = constants::atmosphere::h_tropopause;

    // Use janus::where for branching (MANDATORY for symbolic compatibility)
    auto in_troposphere = (altitude < h_tropopause);

    // Troposphere: linear decrease
    Scalar T_troposphere = T0 - L * altitude;

    // Stratosphere: isothermal at tropopause temperature
    Scalar T_stratosphere = T0 - L * h_tropopause;

    return janus::where(in_troposphere, T_troposphere, T_stratosphere);
}

/**
 * @brief US Standard Atmosphere 1976 - Pressure
 *
 * @tparam Scalar Type for scalar operations (double or casadi::MX)
 * @param altitude Geometric altitude [m]
 * @return Pressure [Pa]
 */
template <typename Scalar> Scalar pressure(const Scalar &altitude) {
    constexpr double P0 = constants::atmosphere::P0;
    constexpr double g0 = constants::physics::g0;
    constexpr double M = constants::atmosphere::M;
    constexpr double R = constants::atmosphere::R_universal;
    constexpr double L = constants::atmosphere::L;
    constexpr double T0 = constants::atmosphere::T0;
    constexpr double h_tropopause = constants::atmosphere::h_tropopause;

    Scalar T = temperature(altitude);

    // Troposphere: barometric formula with lapse rate
    Scalar exponent = g0 * M / (R * L);
    Scalar P_troposphere = P0 * janus::pow(T / T0, exponent);

    // Stratosphere: isothermal barometric formula
    Scalar T_tropopause = T0 - L * h_tropopause;
    Scalar P_tropopause = P0 * janus::pow(T_tropopause / T0, exponent);
    Scalar P_stratosphere =
        P_tropopause *
        janus::exp(-g0 * M * (altitude - h_tropopause) / (R * T_tropopause));

    auto in_troposphere = (altitude < h_tropopause);
    return janus::where(in_troposphere, P_troposphere, P_stratosphere);
}

/**
 * @brief US Standard Atmosphere 1976 - Density
 *
 * @tparam Scalar Type for scalar operations (double or casadi::MX)
 * @param altitude Geometric altitude [m]
 * @return Density [kg/m^3]
 */
template <typename Scalar> Scalar density(const Scalar &altitude) {
    constexpr double R = constants::atmosphere::R; // Specific gas constant

    Scalar P = pressure(altitude);
    Scalar T = temperature(altitude);

    // Ideal gas law: rho = P / (R * T)
    return P / (R * T);
}

/**
 * @brief US Standard Atmosphere 1976 - Speed of Sound
 *
 * @tparam Scalar Type for scalar operations (double or casadi::MX)
 * @param altitude Geometric altitude [m]
 * @return Speed of sound [m/s]
 */
template <typename Scalar> Scalar speed_of_sound(const Scalar &altitude) {
    constexpr double gamma = constants::atmosphere::gamma;
    constexpr double R = constants::atmosphere::R;

    Scalar T = temperature(altitude);

    // a = sqrt(gamma * R * T)
    return janus::sqrt(gamma * R * T);
}

} // namespace vulcan::standard_atmosphere
