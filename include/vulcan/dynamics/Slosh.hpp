// Vulcan Fuel Slosh Dynamics
// Pure stateless functions for slosh modeling per NASA SP-8009
#pragma once

#include <vulcan/core/VulcanTypes.hpp>

#include <janus/janus.hpp>

namespace vulcan::dynamics {

// =============================================================================
// Pendulum Slosh Model (NASA SP-8009)
// =============================================================================

/// Compute pendulum slosh angular acceleration
///
/// Models fuel as equivalent pendulum attached to tank wall.
/// The pendulum responds to vehicle acceleration (including gravity).
///
/// Equation of motion:
/// $$\ddot{\theta} = -\omega_n^2\sin\theta - 2\zeta\omega_n\dot{\theta} +
/// \frac{a_t}{L}\cos\theta$$
///
/// Reference: NASA SP-8009, Section 3
///
/// @tparam Scalar Scalar type (double or casadi::MX)
/// @param theta Pendulum angle from vertical [rad]
/// @param theta_dot Angular rate [rad/s]
/// @param length Pendulum arm length [m]
/// @param zeta Damping ratio (typically 0.005-0.02)
/// @param accel_transverse Transverse acceleration forcing [m/s²]
/// @param gravity Gravitational acceleration [m/s²]
/// @return Angular acceleration [rad/s²]
template <typename Scalar>
Scalar pendulum_slosh_acceleration(const Scalar &theta, const Scalar &theta_dot,
                                   const Scalar &length, const Scalar &zeta,
                                   const Scalar &accel_transverse,
                                   const Scalar &gravity) {
    Scalar omega_n = janus::sqrt(gravity / length);
    Scalar sin_theta = janus::sin(theta);
    Scalar cos_theta = janus::cos(theta);

    // θ̈ = -(g/L)*sin(θ) - 2ζω_n*θ̇ + (a_t/L)*cos(θ)
    return -omega_n * omega_n * sin_theta -
           Scalar(2) * zeta * omega_n * theta_dot +
           (accel_transverse / length) * cos_theta;
}

/// Compute transverse acceleration for pendulum in X-Z plane
///
/// @param accel_body Vehicle acceleration in body frame [m/s²]
/// @param theta Pendulum angle from vertical [rad]
/// @return Transverse acceleration component [m/s²]
template <typename Scalar>
Scalar slosh_transverse_accel(const Vec3<Scalar> &accel_body,
                              const Scalar &theta) {
    Scalar cos_theta = janus::cos(theta);
    Scalar sin_theta = janus::sin(theta);
    // a_t = a_x * cos(θ) - a_z * sin(θ) for pendulum in X-Z plane
    return accel_body(0) * cos_theta - accel_body(2) * sin_theta;
}

/// Compute pendulum slosh mass position relative to hinge
///
/// @param theta Pendulum angle from vertical [rad]
/// @param length Pendulum arm length [m]
/// @return Position vector [x, y, z] where z is down
template <typename Scalar>
Vec3<Scalar> pendulum_slosh_position(const Scalar &theta,
                                     const Scalar &length) {
    return Vec3<Scalar>{length * janus::sin(theta), Scalar(0),
                        length * janus::cos(theta)};
}

/// Compute pendulum slosh reaction force on vehicle
///
/// @param theta Pendulum angle [rad]
/// @param theta_dot Angular rate [rad/s]
/// @param theta_ddot Angular acceleration [rad/s²]
/// @param length Pendulum arm length [m]
/// @param m_slosh Slosh mass [kg]
/// @return Reaction force in body frame [N]
template <typename Scalar>
Vec3<Scalar> pendulum_slosh_force(const Scalar &theta, const Scalar &theta_dot,
                                  const Scalar &theta_ddot,
                                  const Scalar &length, const Scalar &m_slosh) {
    Scalar sin_theta = janus::sin(theta);
    Scalar cos_theta = janus::cos(theta);
    Scalar theta_dot_sq = theta_dot * theta_dot;

    // Slosh mass acceleration (relative to vehicle)
    Vec3<Scalar> a_slosh{
        length * (cos_theta * theta_ddot - sin_theta * theta_dot_sq), Scalar(0),
        length * (-sin_theta * theta_ddot - cos_theta * theta_dot_sq)};

    // Reaction force (Newton's 3rd law)
    return -m_slosh * a_slosh;
}

// =============================================================================
// Spring-Mass Slosh Model (3-DOF per tank)
// =============================================================================

/// Compute spring-mass slosh acceleration (per axis)
///
/// Equation of motion per axis:
/// $$\ddot{x} = (-c\dot{x} - kx - m a_{vehicle}) / m$$
///
/// @tparam Scalar Scalar type (double or casadi::MX)
/// @param displacement Displacement from equilibrium [m]
/// @param velocity Velocity [m/s]
/// @param stiffness Spring stiffness [N/m]
/// @param damping Damping coefficient [N·s/m]
/// @param mass Slosh mass [kg]
/// @param vehicle_accel Vehicle acceleration [m/s²]
/// @return Acceleration [m/s²]
template <typename Scalar>
Scalar
spring_slosh_acceleration(const Scalar &displacement, const Scalar &velocity,
                          const Scalar &stiffness, const Scalar &damping,
                          const Scalar &mass, const Scalar &vehicle_accel) {
    return (-damping * velocity - stiffness * displacement -
            mass * vehicle_accel) /
           mass;
}

/// Compute spring-mass slosh acceleration (3D vector form)
///
/// @param displacement Displacement vector [m]
/// @param velocity Velocity vector [m/s]
/// @param stiffness Stiffness vector [N/m] (per axis)
/// @param damping Damping vector [N·s/m] (per axis)
/// @param mass Slosh mass [kg]
/// @param accel_body Vehicle acceleration [m/s²]
/// @return Acceleration vector [m/s²]
template <typename Scalar>
Vec3<Scalar> spring_slosh_acceleration_3d(const Vec3<Scalar> &displacement,
                                          const Vec3<Scalar> &velocity,
                                          const Vec3<Scalar> &stiffness,
                                          const Vec3<Scalar> &damping,
                                          const Scalar &mass,
                                          const Vec3<Scalar> &accel_body) {
    Vec3<Scalar> accel;
    for (int i = 0; i < 3; ++i) {
        accel(i) = (-damping(i) * velocity(i) - stiffness(i) * displacement(i) -
                    mass * accel_body(i)) /
                   mass;
    }
    return accel;
}

/// Compute spring-mass slosh reaction force
///
/// @param displacement Displacement vector [m]
/// @param velocity Velocity vector [m/s]
/// @param stiffness Stiffness vector [N/m]
/// @param damping Damping vector [N·s/m]
/// @return Reaction force [N]
template <typename Scalar>
Vec3<Scalar> spring_slosh_force(const Vec3<Scalar> &displacement,
                                const Vec3<Scalar> &velocity,
                                const Vec3<Scalar> &stiffness,
                                const Vec3<Scalar> &damping) {
    Vec3<Scalar> force;
    for (int i = 0; i < 3; ++i) {
        force(i) = stiffness(i) * displacement(i) + damping(i) * velocity(i);
    }
    return force;
}

// =============================================================================
// NASA SP-8009 Parameter Estimation
// =============================================================================

/// Estimate slosh frequency for cylindrical tank (NASA SP-8009)
///
/// @param tank_radius Tank inner radius [m]
/// @param fill_level Fractional fill level (0 to 1)
/// @param gravity Gravitational acceleration [m/s²]
/// @return First mode slosh frequency [rad/s]
template <typename Scalar>
Scalar slosh_frequency_cylindrical(const Scalar &tank_radius,
                                   const Scalar &fill_level,
                                   const Scalar &gravity) {
    // ω_s² ≈ 1.84 * g / R for deep fill (h/R > 1)
    Scalar depth_ratio = fill_level * Scalar(2);
    Scalar freq_factor =
        janus::where(depth_ratio > Scalar(1), Scalar(1.84),
                     Scalar(1.84) * janus::tanh(Scalar(1.84) * depth_ratio));

    return janus::sqrt(freq_factor * gravity / tank_radius);
}

/// Estimate slosh frequency for spherical tank
///
/// @param tank_radius Tank inner radius [m]
/// @param fill_level Fractional fill level (0 to 1)
/// @param gravity Gravitational acceleration [m/s²]
/// @return First mode slosh frequency [rad/s]
template <typename Scalar>
Scalar slosh_frequency_spherical(const Scalar &tank_radius,
                                 const Scalar &fill_level,
                                 const Scalar &gravity) {
    // ω_s² ≈ 1.56 * g / R at 50% fill
    Scalar fill_factor = Scalar(4) * fill_level * (Scalar(1) - fill_level);
    Scalar omega_sq = Scalar(1.56) * gravity / tank_radius * fill_factor;
    return janus::sqrt(omega_sq + Scalar(1e-6)); // Avoid zero at empty/full
}

/// Compute equivalent pendulum length from slosh frequency
///
/// @param omega_slosh Slosh frequency [rad/s]
/// @param gravity Gravitational acceleration [m/s²]
/// @return Equivalent pendulum length [m]
template <typename Scalar>
Scalar slosh_pendulum_length(const Scalar &omega_slosh, const Scalar &gravity) {
    return gravity / (omega_slosh * omega_slosh);
}

/// Estimate slosh mass fraction for cylindrical tank (NASA SP-8009)
///
/// @param fill_level Fractional fill level (0 to 1)
/// @return Slosh mass fraction (0 to 1)
template <typename Scalar>
Scalar slosh_mass_fraction_cylindrical(const Scalar &fill_level) {
    Scalar depth_ratio = fill_level * Scalar(2);
    return janus::where(depth_ratio > Scalar(1), Scalar(0.70),
                        Scalar(0.35) + Scalar(0.35) * depth_ratio);
}

/// Compute slosh mass from total fuel mass and fraction
///
/// @param total_fuel_mass Total fuel mass [kg]
/// @param fill_level Fractional fill level (0 to 1)
/// @return Slosh mass [kg]
template <typename Scalar>
Scalar slosh_mass_cylindrical(const Scalar &total_fuel_mass,
                              const Scalar &fill_level) {
    return slosh_mass_fraction_cylindrical(fill_level) * total_fuel_mass;
}

} // namespace vulcan::dynamics
