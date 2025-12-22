// Vulcan Rigid Body Dynamics
// 6DOF equations of motion for trajectory optimization and simulation
#pragma once

#include <vulcan/dynamics/RigidBodyTypes.hpp>
#include <vulcan/rotations/RotationKinematics.hpp>

#include <janus/math/Linalg.hpp>
#include <janus/math/Quaternion.hpp>

namespace vulcan::dynamics {

// =============================================================================
// Translational Dynamics
// =============================================================================

/// Translational dynamics in body frame (F = ma with transport term)
///
/// $$\dot{v}_B = F_B/m - \omega_B \times v_B$$
///
/// The transport term (ω × v) accounts for the rate of change of the
/// body-fixed velocity representation due to body rotation.
///
/// Reference: TAOS Methods Eq 2-105, body-frame form
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param velocity_body Current velocity in body frame [m/s]
/// @param omega_body Angular velocity in body frame [rad/s]
/// @param force_body Sum of forces in body frame [N]
/// @param mass Current body mass [kg]
/// @return Acceleration in body frame [m/s²]
template <typename Scalar>
Vec3<Scalar> translational_dynamics(const Vec3<Scalar> &velocity_body,
                                    const Vec3<Scalar> &omega_body,
                                    const Vec3<Scalar> &force_body,
                                    const Scalar &mass) {
    // v_dot = F/m - ω × v
    Vec3<Scalar> transport = janus::cross(omega_body, velocity_body);
    return force_body / mass - transport;
}

// =============================================================================
// Rotational Dynamics (Euler's Equations)
// =============================================================================

/// Rotational dynamics (Euler's equations)
///
/// $$I \dot{\omega}_B = M_B - \omega_B \times (I \omega_B)$$
///
/// This is the general form of Euler's equations for a rigid body with
/// arbitrary (non-diagonal) inertia tensor.
///
/// Reference: Goldstein, Classical Mechanics, 3rd Ed., Ch.5
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param omega_body Angular velocity in body frame [rad/s]
/// @param moment_body Sum of moments in body frame [N·m]
/// @param inertia Inertia tensor in body frame [kg·m²]
/// @return Angular acceleration in body frame [rad/s²]
template <typename Scalar>
Vec3<Scalar> rotational_dynamics(const Vec3<Scalar> &omega_body,
                                 const Vec3<Scalar> &moment_body,
                                 const Mat3<Scalar> &inertia) {
    // I * ω_dot = M - ω × (I * ω)
    Vec3<Scalar> H = inertia * omega_body; // Angular momentum
    Vec3<Scalar> gyroscopic = janus::cross(omega_body, H);
    Vec3<Scalar> rhs = moment_body - gyroscopic;

    // Solve I * ω_dot = rhs using janus::solve for symbolic compatibility
    return janus::solve(inertia, rhs);
}

// =============================================================================
// Frame Transformations
// =============================================================================

/// Transform velocity from body frame to reference frame
///
/// v_ref = R_body_to_ref * v_body
///
/// Uses the quaternion to perform the rotation.
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param velocity_body Velocity in body frame [m/s]
/// @param attitude Body-to-reference quaternion
/// @return Velocity in reference frame [m/s]
template <typename Scalar>
Vec3<Scalar>
velocity_to_reference_frame(const Vec3<Scalar> &velocity_body,
                            const janus::Quaternion<Scalar> &attitude) {
    return attitude.rotate(velocity_body);
}

/// Transform velocity from reference frame to body frame
///
/// v_body = R_ref_to_body * v_ref
///
/// Uses the conjugate of the attitude quaternion.
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param velocity_ref Velocity in reference frame [m/s]
/// @param attitude Body-to-reference quaternion
/// @return Velocity in body frame [m/s]
template <typename Scalar>
Vec3<Scalar> velocity_to_body_frame(const Vec3<Scalar> &velocity_ref,
                                    const janus::Quaternion<Scalar> &attitude) {
    return attitude.conjugate().rotate(velocity_ref);
}

// =============================================================================
// Full 6DOF Derivatives
// =============================================================================

/// Compute full 6DOF rigid body derivatives
///
/// Implements the standard 6DOF equations of motion:
/// - **Translational (TAOS Eq 2-105 in body frame):**
///   $$\dot{v}_B = F_B/m - \omega_B \times v_B$$
/// - **Rotational (Euler's equations, Goldstein Ch.5):**
///   $$I \dot{\omega}_B = M_B - \omega_B \times (I \omega_B)$$
/// - **Kinematics (Shuster 1993):**
///   $$\dot{q} = \frac{1}{2} q \otimes (0, \omega_B)$$
///
/// This is a PURE FUNCTION with no internal state.
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param state Current rigid body state
/// @param force_body Sum of forces in body frame [N]
/// @param moment_body Sum of moments about CG in body frame [N·m]
/// @param mass_props Current mass and inertia properties
/// @return State derivatives
template <typename Scalar>
RigidBodyDerivatives<Scalar> compute_6dof_derivatives(
    const RigidBodyState<Scalar> &state, const Vec3<Scalar> &force_body,
    const Vec3<Scalar> &moment_body, const MassProperties<Scalar> &mass_props) {
    RigidBodyDerivatives<Scalar> derivs;

    // Position rate = velocity in reference frame
    derivs.position_dot =
        velocity_to_reference_frame(state.velocity_body, state.attitude);

    // Velocity rate = translational dynamics
    derivs.velocity_dot = translational_dynamics(
        state.velocity_body, state.omega_body, force_body, mass_props.mass);

    // Attitude rate = quaternion kinematics
    derivs.attitude_dot =
        quaternion_rate_from_omega(state.attitude, state.omega_body);

    // Angular velocity rate = rotational dynamics
    derivs.omega_dot =
        rotational_dynamics(state.omega_body, moment_body, mass_props.inertia);

    return derivs;
}

// =============================================================================
// ECEF Dynamics with Earth Rotation Effects
// =============================================================================

/// Compute Earth-relative acceleration with Coriolis and centrifugal terms
///
/// For ECEF integration, includes fictitious forces:
/// $$\dot{v}_{\oplus} = F/m - 2(\omega_{\oplus} \times v_{\oplus})
///                    - \omega_{\oplus} \times (\omega_{\oplus} \times r)$$
///
/// Reference: TAOS Methods Eq 2-105
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param position Position in ECEF [m]
/// @param velocity_ecef Velocity in ECEF [m/s]
/// @param force_ecef Force in ECEF [N]
/// @param mass Mass [kg]
/// @param omega_earth Earth rotation vector [rad/s] (typically 7.2921159e-5 *
/// z_hat)
/// @return Acceleration in ECEF [m/s²]
template <typename Scalar>
Vec3<Scalar> translational_dynamics_ecef(const Vec3<Scalar> &position,
                                         const Vec3<Scalar> &velocity_ecef,
                                         const Vec3<Scalar> &force_ecef,
                                         const Scalar &mass,
                                         const Vec3<Scalar> &omega_earth) {
    // Coriolis: 2 * (ω_earth × v)
    Vec3<Scalar> coriolis =
        Scalar(2) * janus::cross(omega_earth, velocity_ecef);

    // Centrifugal: ω × (ω × r)
    Vec3<Scalar> centrifugal =
        janus::cross(omega_earth, janus::cross(omega_earth, position));

    // Total acceleration: F/m - Coriolis - Centrifugal
    return force_ecef / mass - coriolis - centrifugal;
}

} // namespace vulcan::dynamics
