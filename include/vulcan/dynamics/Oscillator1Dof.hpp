// Vulcan 1-DOF Oscillator Dynamics
// Pure stateless functions for harmonic oscillator computations
#pragma once

#include <vulcan/core/VulcanTypes.hpp>

#include <janus/janus.hpp>

namespace vulcan::dynamics {

// =============================================================================
// 1-DOF Harmonic Oscillator Dynamics
// =============================================================================

/// Compute 1-DOF harmonic oscillator acceleration
///
/// Implements the standard second-order oscillator equation:
/// $$\ddot{x} = F/m - 2\zeta\omega_n\dot{x} - \omega_n^2 x$$
///
/// This model is used for:
/// - Fuel slosh (simplified pendulum equivalent)
/// - Torsional springs
/// - Linear rail motion with spring return
/// - Any single-degree-of-freedom vibration
///
/// @tparam Scalar Scalar type (double or casadi::MX)
/// @param x Displacement [m] or angle [rad]
/// @param x_dot Velocity [m/s] or angular rate [rad/s]
/// @param omega_n Natural frequency [rad/s]
/// @param zeta Damping ratio (0=undamped, 1=critical, >1=overdamped)
/// @param force External force [N] or torque [N·m]
/// @param mass Mass [kg] or moment of inertia [kg·m²]
/// @return Acceleration [m/s²] or angular acceleration [rad/s²]
template <typename Scalar>
Scalar oscillator_acceleration(const Scalar &x, const Scalar &x_dot,
                               const Scalar &omega_n, const Scalar &zeta,
                               const Scalar &force, const Scalar &mass) {
    // ẍ = F/m - 2ζωẋ - ω²x
    return force / mass - Scalar(2) * zeta * omega_n * x_dot -
           omega_n * omega_n * x;
}

/// Compute 1-DOF spring-damper acceleration
///
/// Alternative formulation using physical spring-damper parameters:
/// $$\ddot{x} = (F - c\dot{x} - kx) / m$$
///
/// @tparam Scalar Scalar type (double or casadi::MX)
/// @param x Displacement [m]
/// @param x_dot Velocity [m/s]
/// @param stiffness Spring stiffness k [N/m] or [N·m/rad]
/// @param damping Damping coefficient c [N·s/m] or [N·m·s/rad]
/// @param force External force [N] or torque [N·m]
/// @param mass Mass [kg] or moment of inertia [kg·m²]
/// @return Acceleration [m/s²]
template <typename Scalar>
Scalar spring_damper_acceleration(const Scalar &x, const Scalar &x_dot,
                                  const Scalar &stiffness,
                                  const Scalar &damping, const Scalar &force,
                                  const Scalar &mass) {
    // ẍ = (F - c·ẋ - k·x) / m
    return (force - damping * x_dot - stiffness * x) / mass;
}

// =============================================================================
// Parameter Conversions
// =============================================================================

/// Convert spring-damper parameters to natural frequency
///
/// @param stiffness Spring stiffness k [N/m]
/// @param mass Mass [kg]
/// @return Natural frequency [rad/s]
template <typename Scalar>
Scalar spring_to_omega(const Scalar &stiffness, const Scalar &mass) {
    return janus::sqrt(stiffness / mass);
}

/// Convert spring-damper parameters to damping ratio
///
/// @param stiffness Spring stiffness k [N/m]
/// @param damping Damping coefficient c [N·s/m]
/// @param mass Mass [kg]
/// @return Damping ratio (dimensionless)
template <typename Scalar>
Scalar spring_to_zeta(const Scalar &stiffness, const Scalar &damping,
                      const Scalar &mass) {
    return damping / (Scalar(2) * janus::sqrt(stiffness * mass));
}

/// Compute natural period from frequency
///
/// @param omega_n Natural frequency [rad/s]
/// @return Period [s]
template <typename Scalar> Scalar oscillator_period(const Scalar &omega_n) {
    return Scalar(2) * Scalar(M_PI) / omega_n;
}

// =============================================================================
// Energy Calculations
// =============================================================================

/// Oscillator total mechanical energy (kinetic + potential)
///
/// @param x Displacement [m]
/// @param x_dot Velocity [m/s]
/// @param omega_n Natural frequency [rad/s]
/// @param mass Mass [kg]
/// @return Total mechanical energy [J]
template <typename Scalar>
Scalar oscillator_energy(const Scalar &x, const Scalar &x_dot,
                         const Scalar &omega_n, const Scalar &mass) {
    // E = 0.5*m*ẋ² + 0.5*k*x² = 0.5*m*(ẋ² + ω²x²)
    return Scalar(0.5) * mass * (x_dot * x_dot + omega_n * omega_n * x * x);
}

/// Oscillator kinetic energy
///
/// @param x_dot Velocity [m/s]
/// @param mass Mass [kg]
/// @return Kinetic energy [J]
template <typename Scalar>
Scalar oscillator_kinetic_energy(const Scalar &x_dot, const Scalar &mass) {
    return Scalar(0.5) * mass * x_dot * x_dot;
}

/// Oscillator potential energy
///
/// @param x Displacement [m]
/// @param omega_n Natural frequency [rad/s]
/// @param mass Mass [kg]
/// @return Potential energy [J]
template <typename Scalar>
Scalar oscillator_potential_energy(const Scalar &x, const Scalar &omega_n,
                                   const Scalar &mass) {
    // U = 0.5*k*x² = 0.5*m*ω²*x²
    return Scalar(0.5) * mass * omega_n * omega_n * x * x;
}

} // namespace vulcan::dynamics
