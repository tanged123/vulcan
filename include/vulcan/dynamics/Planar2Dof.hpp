// Vulcan 2-DOF Planar Dynamics
// Pure stateless functions for planar point mass and pendulum computations
#pragma once

#include <vulcan/core/VulcanTypes.hpp>

#include <janus/janus.hpp>

namespace vulcan::dynamics {

// =============================================================================
// 2-DOF Planar Point Mass
// =============================================================================

/// Compute 2-DOF planar point mass acceleration
///
/// Simple Newton's second law in 2D:
/// $$\ddot{x} = F_x/m, \quad \ddot{y} = F_y/m$$
///
/// @tparam Scalar Scalar type (double or casadi::MX)
/// @param force_x X-direction force [N]
/// @param force_y Y-direction force [N]
/// @param mass Point mass [kg]
/// @return Acceleration vector [ax, ay] [m/s²]
template <typename Scalar>
Vec2<Scalar> planar_acceleration(const Scalar &force_x, const Scalar &force_y,
                                 const Scalar &mass) {
    return Vec2<Scalar>{force_x / mass, force_y / mass};
}

/// Compute 2-DOF planar acceleration from force vector
///
/// @tparam Scalar Scalar type (double or casadi::MX)
/// @param force Force vector [Fx, Fy] [N]
/// @param mass Point mass [kg]
/// @return Acceleration vector [ax, ay] [m/s²]
template <typename Scalar>
Vec2<Scalar> planar_acceleration(const Vec2<Scalar> &force,
                                 const Scalar &mass) {
    return force / mass;
}

/// Compute 2-DOF planar acceleration with gravity (Y-down convention)
///
/// @param force_x X-direction force [N] (not including gravity)
/// @param force_y Y-direction force [N] (not including gravity)
/// @param mass Point mass [kg]
/// @param gravity Gravitational acceleration [m/s²] (positive down)
/// @return Acceleration vector [ax, ay] [m/s²]
template <typename Scalar>
Vec2<Scalar>
planar_acceleration_gravity(const Scalar &force_x, const Scalar &force_y,
                            const Scalar &mass, const Scalar &gravity) {
    return Vec2<Scalar>{force_x / mass, force_y / mass + gravity};
}

// =============================================================================
// Spherical Pendulum (2-DOF on sphere)
// =============================================================================

/// Compute spherical pendulum polar angle acceleration (θ̈)
///
/// For a spherical pendulum using spherical coordinates:
/// - θ (theta): polar angle from vertical (0 = straight down)
/// - φ (phi): azimuthal angle in horizontal plane
///
/// $$\ddot{\theta} = \sin\theta\cos\theta \dot{\phi}^2 - \frac{g}{L}\sin\theta
/// - 2\zeta\omega_n\dot{\theta}$$
///
/// @tparam Scalar Scalar type (double or casadi::MX)
/// @param theta Polar angle from vertical [rad]
/// @param theta_dot Polar angular rate [rad/s]
/// @param phi_dot Azimuthal angular rate [rad/s]
/// @param length Pendulum length [m]
/// @param gravity Gravitational acceleration [m/s²]
/// @param zeta Damping ratio (dimensionless)
/// @return Polar angular acceleration [rad/s²]
template <typename Scalar>
Scalar
spherical_pendulum_theta_ddot(const Scalar &theta, const Scalar &theta_dot,
                              const Scalar &phi_dot, const Scalar &length,
                              const Scalar &gravity, const Scalar &zeta) {
    Scalar sin_theta = janus::sin(theta);
    Scalar cos_theta = janus::cos(theta);
    Scalar omega_n = janus::sqrt(gravity / length);

    // θ̈ = sin(θ)cos(θ)φ̇² - (g/L)sin(θ) - 2ζω_n θ̇
    return sin_theta * cos_theta * phi_dot * phi_dot -
           (gravity / length) * sin_theta -
           Scalar(2) * zeta * omega_n * theta_dot;
}

/// Compute spherical pendulum azimuthal angle acceleration (φ̈)
///
/// $$\ddot{\phi} = -2\cot\theta \dot{\theta}\dot{\phi} -
/// 2\zeta\omega_n\dot{\phi}$$
///
/// @tparam Scalar Scalar type (double or casadi::MX)
/// @param theta Polar angle from vertical [rad]
/// @param theta_dot Polar angular rate [rad/s]
/// @param phi_dot Azimuthal angular rate [rad/s]
/// @param length Pendulum length [m]
/// @param gravity Gravitational acceleration [m/s²]
/// @param zeta Damping ratio (dimensionless)
/// @return Azimuthal angular acceleration [rad/s²]
template <typename Scalar>
Scalar spherical_pendulum_phi_ddot(const Scalar &theta, const Scalar &theta_dot,
                                   const Scalar &phi_dot, const Scalar &length,
                                   const Scalar &gravity, const Scalar &zeta) {
    Scalar sin_theta = janus::sin(theta);
    Scalar cos_theta = janus::cos(theta);
    Scalar omega_n = janus::sqrt(gravity / length);

    // Regularized cot(θ) to avoid division by zero
    Scalar cot_theta = cos_theta / (sin_theta + Scalar(1e-10));

    // φ̈ = -2cot(θ)θ̇φ̇ - 2ζω_n φ̇
    return Scalar(-2) * cot_theta * theta_dot * phi_dot -
           Scalar(2) * zeta * omega_n * phi_dot;
}

/// Convert spherical pendulum angles to Cartesian position
///
/// @param theta Polar angle from vertical [rad]
/// @param phi Azimuthal angle [rad]
/// @param length Pendulum length [m]
/// @return Position [x, y, z] where z is down (positive = below pivot)
template <typename Scalar>
Vec3<Scalar> spherical_pendulum_position(const Scalar &theta, const Scalar &phi,
                                         const Scalar &length) {
    Scalar sin_theta = janus::sin(theta);
    Scalar cos_theta = janus::cos(theta);
    Scalar sin_phi = janus::sin(phi);
    Scalar cos_phi = janus::cos(phi);

    return Vec3<Scalar>{
        length * sin_theta * cos_phi, // x
        length * sin_theta * sin_phi, // y
        length * cos_theta            // z (down from pivot)
    };
}

/// Spherical pendulum total mechanical energy
///
/// @param theta Polar angle [rad]
/// @param theta_dot Polar angular rate [rad/s]
/// @param phi_dot Azimuthal angular rate [rad/s]
/// @param length Pendulum length [m]
/// @param mass Pendulum mass [kg]
/// @param gravity Gravitational acceleration [m/s²]
/// @return Total mechanical energy [J] (KE + PE, with PE=0 at bottom)
template <typename Scalar>
Scalar spherical_pendulum_energy(const Scalar &theta, const Scalar &theta_dot,
                                 const Scalar &phi_dot, const Scalar &length,
                                 const Scalar &mass, const Scalar &gravity) {
    Scalar sin_theta = janus::sin(theta);
    Scalar cos_theta = janus::cos(theta);

    // Kinetic energy: T = 0.5 * m * L² * (θ̇² + sin²(θ)φ̇²)
    Scalar KE =
        Scalar(0.5) * mass * length * length *
        (theta_dot * theta_dot + sin_theta * sin_theta * phi_dot * phi_dot);

    // Potential energy: V = m*g*L*(1 - cos(θ)) [zero at bottom]
    Scalar PE = mass * gravity * length * (Scalar(1) - cos_theta);

    return KE + PE;
}

// =============================================================================
// Simple Pendulum (1-DOF special case: phi = 0)
// =============================================================================

/// Compute simple pendulum angular acceleration
///
/// $$\ddot{\theta} = -\frac{g}{L}\sin\theta - 2\zeta\omega_n\dot{\theta}$$
///
/// @param theta Angle from vertical [rad]
/// @param theta_dot Angular rate [rad/s]
/// @param length Pendulum length [m]
/// @param gravity Gravitational acceleration [m/s²]
/// @param zeta Damping ratio
/// @return Angular acceleration [rad/s²]
template <typename Scalar>
Scalar simple_pendulum_acceleration(const Scalar &theta,
                                    const Scalar &theta_dot,
                                    const Scalar &length, const Scalar &gravity,
                                    const Scalar &zeta) {
    Scalar omega_n = janus::sqrt(gravity / length);
    return -(gravity / length) * janus::sin(theta) -
           Scalar(2) * zeta * omega_n * theta_dot;
}

/// Simple pendulum period (small angle approximation)
///
/// $$T = 2\pi\sqrt{L/g}$$
template <typename Scalar>
Scalar simple_pendulum_period(const Scalar &length, const Scalar &gravity) {
    return Scalar(2) * Scalar(M_PI) * janus::sqrt(length / gravity);
}

/// Simple pendulum natural frequency
///
/// $$\omega_n = \sqrt{g/L}$$
template <typename Scalar>
Scalar simple_pendulum_omega(const Scalar &length, const Scalar &gravity) {
    return janus::sqrt(gravity / length);
}

} // namespace vulcan::dynamics
