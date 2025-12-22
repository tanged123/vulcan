// Vulcan 3-DOF Point Mass Dynamics
// Pure stateless functions for point mass trajectory computation
#pragma once

#include <vulcan/core/VulcanTypes.hpp>

#include <janus/janus.hpp>

namespace vulcan::dynamics {

// =============================================================================
// 3-DOF Point Mass Dynamics (Inertial Frame)
// =============================================================================

/// Compute 3-DOF point mass acceleration in inertial frame
///
/// Simple Newton's second law in 3D:
/// $$\ddot{\mathbf{r}} = \mathbf{F}/m$$
///
/// @tparam Scalar Scalar type (double or casadi::MX)
/// @param force Force vector in inertial frame [N]
/// @param mass Point mass [kg]
/// @return Acceleration vector [m/s²]
template <typename Scalar>
Vec3<Scalar> point_mass_acceleration(const Vec3<Scalar> &force,
                                     const Scalar &mass) {
    return force / mass;
}

/// Compute velocity magnitude
///
/// @param velocity Velocity vector [m/s]
/// @return Speed [m/s]
template <typename Scalar> Scalar speed(const Vec3<Scalar> &velocity) {
    return janus::norm(velocity);
}

/// Compute unit velocity direction
///
/// @param velocity Velocity vector [m/s]
/// @return Unit velocity direction (normalized)
template <typename Scalar>
Vec3<Scalar> velocity_direction(const Vec3<Scalar> &velocity) {
    Scalar v_mag = janus::norm(velocity);
    return velocity / (v_mag + Scalar(1e-12)); // Regularized for zero velocity
}

// =============================================================================
// 3-DOF Point Mass Dynamics (ECEF Frame)
// =============================================================================

/// Compute 3-DOF point mass acceleration in ECEF frame
///
/// Includes Coriolis and centrifugal terms for rotating reference frame:
/// $$\ddot{\mathbf{r}} = \mathbf{F}/m - 2\boldsymbol{\omega}_e \times
/// \mathbf{v} - \boldsymbol{\omega}_e \times (\boldsymbol{\omega}_e \times
/// \mathbf{r})$$
///
/// @tparam Scalar Scalar type (double or casadi::MX)
/// @param position Position in ECEF [m]
/// @param velocity Velocity in ECEF [m/s]
/// @param force Force vector in ECEF [N]
/// @param mass Point mass [kg]
/// @param omega_earth Earth rotation vector [rad/s] (typically {0,
/// 0, 7.2921159e-5})
/// @return Acceleration vector in ECEF [m/s²]
template <typename Scalar>
Vec3<Scalar> point_mass_acceleration_ecef(const Vec3<Scalar> &position,
                                          const Vec3<Scalar> &velocity,
                                          const Vec3<Scalar> &force,
                                          const Scalar &mass,
                                          const Vec3<Scalar> &omega_earth) {
    // Force-based acceleration
    Vec3<Scalar> a_force = force / mass;

    // Coriolis: -2 * ω × v
    Vec3<Scalar> coriolis = Scalar(-2) * janus::cross(omega_earth, velocity);

    // Centrifugal: -ω × (ω × r)
    Vec3<Scalar> centrifugal =
        -janus::cross(omega_earth, janus::cross(omega_earth, position));

    return a_force + coriolis + centrifugal;
}

// =============================================================================
// Specific Force Components
// =============================================================================

/// Compute specific force from acceleration (a = F/m)
///
/// @param acceleration Acceleration vector [m/s²]
/// @return Specific force [m/s²] = [N/kg]
template <typename Scalar>
Vec3<Scalar> specific_force(const Vec3<Scalar> &acceleration) {
    return acceleration; // They're the same thing
}

/// Compute g-load from acceleration
///
/// @param acceleration Acceleration vector [m/s²]
/// @param g0 Standard gravity [m/s²] (default 9.80665)
/// @return Load factor [g's]
template <typename Scalar>
Vec3<Scalar> g_load(const Vec3<Scalar> &acceleration,
                    const Scalar &g0 = Scalar(9.80665)) {
    return acceleration / g0;
}

/// Compute total g-load magnitude
///
/// @param acceleration Acceleration vector [m/s²]
/// @param g0 Standard gravity [m/s²]
/// @return Total load factor magnitude [g's]
template <typename Scalar>
Scalar g_load_magnitude(const Vec3<Scalar> &acceleration,
                        const Scalar &g0 = Scalar(9.80665)) {
    return janus::norm(acceleration) / g0;
}

// =============================================================================
// Flight Path Angles
// =============================================================================

/// Compute flight path angle (gamma) from velocity
///
/// Flight path angle is the angle between the velocity vector and the
/// local horizontal plane, positive upward.
///
/// $$\gamma = \arcsin(-v_z / |v|)$$  (for Z-down convention)
///
/// @param velocity Velocity vector [m/s] (Z-down)
/// @return Flight path angle [rad]
template <typename Scalar>
Scalar flight_path_angle(const Vec3<Scalar> &velocity) {
    Scalar v_mag = janus::norm(velocity);
    // Z-down: negative vz is upward, positive gamma
    return janus::asin(-velocity(2) / (v_mag + Scalar(1e-12)));
}

/// Compute heading angle (chi) from velocity
///
/// Heading angle is the angle from North to the horizontal projection
/// of velocity, measured clockwise (for NED convention).
///
/// $$\chi = \arctan2(v_y, v_x)$$
///
/// @param velocity Velocity vector [m/s] (in NED frame)
/// @return Heading angle [rad]
template <typename Scalar> Scalar heading_angle(const Vec3<Scalar> &velocity) {
    return janus::atan2(velocity(1), velocity(0));
}

/// Compute velocity from speed and flight path angles
///
/// @param speed Speed magnitude [m/s]
/// @param gamma Flight path angle [rad]
/// @param chi Heading angle from North [rad]
/// @return Velocity vector in NED [m/s]
template <typename Scalar>
Vec3<Scalar> velocity_from_angles(const Scalar &speed, const Scalar &gamma,
                                  const Scalar &chi) {
    Scalar cos_gamma = janus::cos(gamma);
    Scalar sin_gamma = janus::sin(gamma);
    Scalar cos_chi = janus::cos(chi);
    Scalar sin_chi = janus::sin(chi);

    return Vec3<Scalar>{
        speed * cos_gamma * cos_chi, // North (vx)
        speed * cos_gamma * sin_chi, // East (vy)
        -speed * sin_gamma           // Down (vz), negative for positive gamma
    };
}

// =============================================================================
// Energy & Orbital Quantities
// =============================================================================

/// Compute specific kinetic energy
///
/// @param velocity Velocity vector [m/s]
/// @return Specific kinetic energy [J/kg] = [m²/s²]
template <typename Scalar>
Scalar specific_kinetic_energy(const Vec3<Scalar> &velocity) {
    return Scalar(0.5) * janus::dot(velocity, velocity);
}

/// Compute specific potential energy (gravitation)
///
/// @param position Position vector from center of attracting body [m]
/// @param mu Gravitational parameter [m³/s²]
/// @return Specific potential energy [J/kg] (negative for bound orbits)
template <typename Scalar>
Scalar specific_potential_energy(const Vec3<Scalar> &position,
                                 const Scalar &mu) {
    return -mu / janus::norm(position);
}

/// Compute total specific mechanical energy
///
/// @param position Position vector [m]
/// @param velocity Velocity vector [m/s]
/// @param mu Gravitational parameter [m³/s²]
/// @return Specific mechanical energy [J/kg]
template <typename Scalar>
Scalar specific_energy(const Vec3<Scalar> &position,
                       const Vec3<Scalar> &velocity, const Scalar &mu) {
    return specific_kinetic_energy(velocity) +
           specific_potential_energy(position, mu);
}

} // namespace vulcan::dynamics
