// Vulcan Aerodynamics Module
// Core aerodynamic utility functions for trajectory optimization
#pragma once

#include <vulcan/core/VulcanTypes.hpp>

#include <janus/math/Linalg.hpp>
#include <janus/math/Logic.hpp>
#include <janus/math/Trig.hpp>

namespace vulcan::aero {

// =============================================================================
// Fundamental Aerodynamic Quantities
// =============================================================================

/**
 * @brief Dynamic pressure
 *
 * q = 0.5 * rho * V²
 *
 * @tparam Scalar Scalar type (double or casadi::MX)
 * @param density Air density [kg/m³]
 * @param velocity Airspeed magnitude [m/s]
 * @return Dynamic pressure [Pa]
 */
template <typename Scalar>
Scalar dynamic_pressure(const Scalar &density, const Scalar &velocity) {
    return Scalar(0.5) * density * velocity * velocity;
}

/**
 * @brief Mach number
 *
 * M = V / a
 *
 * @tparam Scalar Scalar type (double or casadi::MX)
 * @param velocity Airspeed magnitude [m/s]
 * @param speed_of_sound Local speed of sound [m/s]
 * @return Mach number [-]
 */
template <typename Scalar>
Scalar mach_number(const Scalar &velocity, const Scalar &speed_of_sound) {
    return velocity / speed_of_sound;
}

/**
 * @brief Reynolds number
 *
 * Re = rho * V * L / mu
 *
 * @tparam Scalar Scalar type (double or casadi::MX)
 * @param density Air density [kg/m³]
 * @param velocity Airspeed magnitude [m/s]
 * @param length Characteristic length [m]
 * @param viscosity Dynamic viscosity [Pa·s]
 * @return Reynolds number [-]
 */
template <typename Scalar>
Scalar reynolds_number(const Scalar &density, const Scalar &velocity,
                       const Scalar &length, const Scalar &viscosity) {
    return density * velocity * length / viscosity;
}

/**
 * @brief Compute airspeed from ground velocity and wind
 *
 * V_air = ||V_ground - V_wind||
 *
 * @tparam Scalar Scalar type (double or casadi::MX)
 * @param velocity_ground Ground-relative velocity vector [m/s]
 * @param velocity_wind Wind velocity vector [m/s]
 * @return Airspeed magnitude [m/s]
 */
template <typename Scalar>
Scalar airspeed(const Vec3<Scalar> &velocity_ground,
                const Vec3<Scalar> &velocity_wind) {
    Vec3<Scalar> v_air = velocity_ground - velocity_wind;
    return janus::norm(v_air);
}

/**
 * @brief Compute airspeed magnitude from velocity vector
 *
 * @tparam Scalar Scalar type (double or casadi::MX)
 * @param velocity Velocity vector [m/s]
 * @return Airspeed magnitude [m/s]
 */
template <typename Scalar> Scalar airspeed(const Vec3<Scalar> &velocity) {
    return janus::norm(velocity);
}

// =============================================================================
// Aerodynamic Angles
// =============================================================================

/**
 * @brief Compute aerodynamic angles from velocity in body frame
 *
 * These are the fundamental aerodynamic angles used for force calculations:
 * - Alpha (angle of attack): rotation about body Y-axis
 * - Beta (sideslip angle): rotation about body Z-axis
 *
 * Sign conventions:
 * - Alpha positive: nose up relative to velocity
 * - Beta positive: wind from right (velocity has positive Y component)
 *
 * @tparam Scalar Scalar type (double or casadi::MX)
 * @param velocity_body Velocity vector in body frame [m/s]
 * @return [alpha, beta] aerodynamic angles [rad]
 */
template <typename Scalar>
Vec2<Scalar> aero_angles(const Vec3<Scalar> &velocity_body) {
    Scalar vx = velocity_body(0); // Forward component
    Scalar vy = velocity_body(1); // Right component
    Scalar vz = velocity_body(2); // Down component

    // Total speed
    Scalar v_total = janus::norm(velocity_body);
    Scalar eps = Scalar(1e-10);
    Scalar is_zero = v_total < eps;

    // Angle of attack: alpha = atan2(vz, vx)
    // Positive when flow comes from below (nose up relative to velocity)
    Scalar alpha = janus::atan2(vz, vx);
    alpha = janus::where(is_zero, Scalar(0), alpha);

    // Sideslip angle: beta = asin(vy / v_total)
    // Positive when flow comes from right
    Scalar beta = janus::asin(vy / v_total);
    beta = janus::where(is_zero, Scalar(0), beta);

    Vec2<Scalar> angles;
    angles << alpha, beta;
    return angles;
}

// =============================================================================
// Combined Aerodynamic State
// =============================================================================

/**
 * @brief Complete aerodynamic state at a flight condition
 *
 * Contains all aerodynamic quantities computed in a single evaluation.
 * Efficient for trajectory optimization where multiple properties are needed.
 *
 * @tparam Scalar Scalar type (double or casadi::MX)
 */
template <typename Scalar> struct AeroState {
    Scalar dynamic_pressure; ///< Dynamic pressure [Pa]
    Scalar mach;             ///< Mach number [-]
    Scalar reynolds;         ///< Reynolds number [-]
    Scalar airspeed;         ///< Airspeed magnitude [m/s]
    Scalar alpha;            ///< Angle of attack [rad]
    Scalar beta;             ///< Sideslip angle [rad]
};

/**
 * @brief Compute complete aerodynamic state
 *
 * @tparam Scalar Scalar type (double or casadi::MX)
 * @param density Air density [kg/m³]
 * @param speed_of_sound Speed of sound [m/s]
 * @param viscosity Dynamic viscosity [Pa·s]
 * @param velocity_body Velocity in body frame [m/s]
 * @param char_length Characteristic length for Reynolds number [m]
 * @return Complete aerodynamic state
 */
template <typename Scalar>
AeroState<Scalar>
aero_state(const Scalar &density, const Scalar &speed_of_sound,
           const Scalar &viscosity, const Vec3<Scalar> &velocity_body,
           const Scalar &char_length) {
    Scalar V = airspeed(velocity_body);
    Vec2<Scalar> angles = aero_angles(velocity_body);

    return AeroState<Scalar>{
        .dynamic_pressure = dynamic_pressure(density, V),
        .mach = mach_number(V, speed_of_sound),
        .reynolds = reynolds_number(density, V, char_length, viscosity),
        .airspeed = V,
        .alpha = angles(0),
        .beta = angles(1)};
}

} // namespace vulcan::aero
