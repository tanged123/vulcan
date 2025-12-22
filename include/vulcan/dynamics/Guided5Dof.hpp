// Vulcan Pseudo-5DOF Dynamics
// Pure stateless dynamics functions for guided vehicle simulation
//
// Pseudo-5DOF: 3-DOF translation + 2-DOF attitude (pitch/yaw) with roll
// commanded. Attitude responds to commands via second-order transfer function
// dynamics. Supports bank-to-turn and skid-to-turn steering laws.
#pragma once

#include <vulcan/core/VulcanTypes.hpp>

#include <janus/janus.hpp>

namespace vulcan::dynamics {

// =============================================================================
// Position Kinematics
// =============================================================================

/// Compute position derivative (trivial kinematic relationship)
///
/// For pseudo-5DOF in NED frame: ṙ = v
///
/// @param velocity Velocity vector in NED [m/s]
/// @return Position derivative in NED [m/s]
template <typename Scalar>
Vec3<Scalar> position_dot(const Vec3<Scalar> &velocity) {
    return velocity;
}

// =============================================================================
// Attitude Dynamics via Transfer Function
// =============================================================================

/// Compute attitude angle acceleration via 2nd-order transfer function
///
/// Implements: α̈ = ω²(α_cmd - α) - 2ζωα̇
///
/// This models the vehicle's attitude response to commands (e.g., from
/// autopilot). The ω_n and ζ parameters characterize the closed-loop response.
///
/// @tparam Scalar Scalar type (double or casadi::MX)
/// @param angle Current angle [rad]
/// @param angle_dot Current angle rate [rad/s]
/// @param angle_cmd Commanded angle [rad]
/// @param omega_n Natural frequency of response [rad/s]
/// @param zeta Damping ratio of response
/// @return Angular acceleration [rad/s²]
template <typename Scalar>
Scalar attitude_response_accel(const Scalar &angle, const Scalar &angle_dot,
                               const Scalar &angle_cmd, const Scalar &omega_n,
                               const Scalar &zeta) {
    Scalar omega_sq = omega_n * omega_n;
    return omega_sq * (angle_cmd - angle) -
           Scalar(2) * zeta * omega_n * angle_dot;
}

// =============================================================================
// Force Direction for Different Steering Laws
// =============================================================================

/// Compute thrust direction for bank-to-turn vehicle
///
/// Bank-to-turn: Vehicle banks (rolls) to turn. Thrust is along body X-axis.
/// Lift acts perpendicular to velocity, rotated by bank angle.
///
/// @param gamma Flight path angle [rad] (positive = climbing)
/// @param chi Heading angle [rad] (from North)
/// @return Thrust unit direction in NED frame
template <typename Scalar>
Vec3<Scalar> thrust_direction_btt(const Scalar &gamma, const Scalar &chi) {
    Scalar cos_gamma = janus::cos(gamma);
    Scalar sin_gamma = janus::sin(gamma);
    Scalar cos_chi = janus::cos(chi);
    Scalar sin_chi = janus::sin(chi);

    // Thrust aligned with velocity direction
    return Vec3<Scalar>{
        cos_gamma * cos_chi, // North
        cos_gamma * sin_chi, // East
        -sin_gamma           // Down (negative for nose up)
    };
}

/// Compute lift direction for bank-to-turn vehicle
///
/// Lift acts perpendicular to velocity, in the plane defined by bank angle.
/// For zero bank, lift is "up" (opposite gravity).
/// For non-zero bank, lift tilts toward the turn.
///
/// @param gamma Flight path angle [rad]
/// @param chi Heading angle [rad]
/// @param phi Bank angle [rad] (positive = right wing down)
/// @return Lift unit direction in NED frame
template <typename Scalar>
Vec3<Scalar> lift_direction_btt(const Scalar &gamma, const Scalar &chi,
                                const Scalar &phi) {
    Scalar cos_gamma = janus::cos(gamma);
    Scalar sin_gamma = janus::sin(gamma);
    Scalar cos_chi = janus::cos(chi);
    Scalar sin_chi = janus::sin(chi);
    Scalar cos_phi = janus::cos(phi);
    Scalar sin_phi = janus::sin(phi);

    // "Up" direction in velocity frame (perpendicular to velocity, in vertical
    // plane)
    Vec3<Scalar> up_v{sin_gamma * cos_chi, sin_gamma * sin_chi, cos_gamma};

    // "Right" direction (perpendicular to velocity, horizontal)
    Vec3<Scalar> right_v{sin_chi, -cos_chi, Scalar(0)};

    // Lift direction rotated by bank angle
    return cos_phi * up_v + sin_phi * right_v;
}

/// Compute thrust direction for skid-to-turn vehicle
///
/// Skid-to-turn: Vehicle yaws to turn (like a missile with cruciform fins).
/// Thrust is along body X-axis, defined by pitch (theta) and yaw (psi).
///
/// @param theta Pitch angle [rad] (positive = nose up)
/// @param psi Yaw angle [rad] (positive = nose right)
/// @return Thrust unit direction in NED frame
template <typename Scalar>
Vec3<Scalar> thrust_direction_stt(const Scalar &theta, const Scalar &psi) {
    Scalar cos_theta = janus::cos(theta);
    Scalar sin_theta = janus::sin(theta);
    Scalar cos_psi = janus::cos(psi);
    Scalar sin_psi = janus::sin(psi);

    return Vec3<Scalar>{
        cos_theta * cos_psi, // North
        cos_theta * sin_psi, // East
        -sin_theta           // Down
    };
}

/// Compute side force direction for skid-to-turn vehicle
///
/// Side force acts perpendicular to body X-axis in the horizontal body plane.
/// This is the force generated by sideslip (beta).
///
/// @param theta Pitch angle [rad]
/// @param psi Yaw angle [rad]
/// @return Side force unit direction in NED frame (body Y direction)
template <typename Scalar>
Vec3<Scalar> side_force_direction_stt(const Scalar &theta, const Scalar &psi) {
    Scalar cos_psi = janus::cos(psi);
    Scalar sin_psi = janus::sin(psi);

    // Body Y is perpendicular to body X, in horizontal plane
    return Vec3<Scalar>{-sin_psi, cos_psi, Scalar(0)};
}

/// Compute normal force direction for skid-to-turn vehicle
///
/// Normal force acts perpendicular to body X in the vertical body plane.
/// This is the lift-like force generated by angle of attack (alpha).
///
/// @param theta Pitch angle [rad]
/// @param psi Yaw angle [rad]
/// @return Normal force unit direction in NED frame (body -Z direction)
template <typename Scalar>
Vec3<Scalar> normal_force_direction_stt(const Scalar &theta,
                                        const Scalar &psi) {
    Scalar cos_theta = janus::cos(theta);
    Scalar sin_theta = janus::sin(theta);
    Scalar cos_psi = janus::cos(psi);
    Scalar sin_psi = janus::sin(psi);

    // Body -Z (up in body frame)
    return Vec3<Scalar>{sin_theta * cos_psi, sin_theta * sin_psi, cos_theta};
}

// =============================================================================
// Velocity Derivatives
// =============================================================================

/// Compute velocity derivative for bank-to-turn vehicle
///
/// @param thrust Thrust magnitude [N]
/// @param drag Drag magnitude [N]
/// @param lift Lift magnitude [N]
/// @param mass Vehicle mass [kg]
/// @param gravity Gravitational acceleration [m/s²]
/// @param gamma Flight path angle [rad]
/// @param chi Heading angle [rad]
/// @param phi Bank angle [rad]
/// @return Velocity derivative in NED [m/s²]
template <typename Scalar>
Vec3<Scalar> velocity_dot_btt(const Scalar &thrust, const Scalar &drag,
                              const Scalar &lift, const Scalar &mass,
                              const Scalar &gravity, const Scalar &gamma,
                              const Scalar &chi, const Scalar &phi) {
    Vec3<Scalar> T_dir = thrust_direction_btt(gamma, chi);
    Vec3<Scalar> L_dir = lift_direction_btt(gamma, chi, phi);
    Vec3<Scalar> D_dir = -T_dir; // Drag opposes thrust direction

    Vec3<Scalar> grav{Scalar(0), Scalar(0), gravity}; // Z-down

    return (thrust / mass) * T_dir + (drag / mass) * D_dir +
           (lift / mass) * L_dir + grav;
}

/// Compute velocity derivative for skid-to-turn vehicle
///
/// @param thrust Thrust magnitude [N]
/// @param drag Drag magnitude [N]
/// @param normal_force Normal force magnitude [N] (from alpha)
/// @param side_force Side force magnitude [N] (from beta)
/// @param mass Vehicle mass [kg]
/// @param gravity Gravitational acceleration [m/s²]
/// @param theta Pitch angle [rad]
/// @param psi Yaw angle [rad]
/// @return Velocity derivative in NED [m/s²]
template <typename Scalar>
Vec3<Scalar> velocity_dot_stt(const Scalar &thrust, const Scalar &drag,
                              const Scalar &normal_force,
                              const Scalar &side_force, const Scalar &mass,
                              const Scalar &gravity, const Scalar &theta,
                              const Scalar &psi) {
    Vec3<Scalar> T_dir = thrust_direction_stt(theta, psi);
    Vec3<Scalar> N_dir = normal_force_direction_stt(theta, psi);
    Vec3<Scalar> S_dir = side_force_direction_stt(theta, psi);
    Vec3<Scalar> D_dir = -T_dir; // Drag opposes thrust

    Vec3<Scalar> grav{Scalar(0), Scalar(0), gravity}; // Z-down

    return (thrust / mass) * T_dir + (drag / mass) * D_dir +
           (normal_force / mass) * N_dir + (side_force / mass) * S_dir + grav;
}

// =============================================================================
// Flight Path Angle Derivatives
// =============================================================================

/// Compute flight path angle rate from vertical acceleration
///
/// γ̇ = (L·cos(φ) - W·cos(γ)) / (m·V)
///
/// For pseudo-5DOF, given the normal load factor, compute gamma_dot.
///
/// @param lift Lift magnitude [N]
/// @param weight Weight (m*g) [N]
/// @param mass Vehicle mass [kg]
/// @param velocity Speed [m/s]
/// @param gamma Current flight path angle [rad]
/// @param phi Bank angle [rad]
/// @return Flight path angle rate [rad/s]
template <typename Scalar>
Scalar gamma_dot(const Scalar &lift, const Scalar &weight, const Scalar &mass,
                 const Scalar &velocity, const Scalar &gamma,
                 const Scalar &phi) {
    Scalar cos_phi = janus::cos(phi);
    Scalar cos_gamma = janus::cos(gamma);
    return (lift * cos_phi - weight * cos_gamma) /
           (mass * velocity + Scalar(1e-12));
}

/// Compute heading rate from lateral acceleration (bank-to-turn)
///
/// χ̇ = (L·sin(φ)) / (m·V·cos(γ))
///
/// @param lift Lift magnitude [N]
/// @param mass Vehicle mass [kg]
/// @param velocity Speed [m/s]
/// @param gamma Flight path angle [rad]
/// @param phi Bank angle [rad]
/// @return Heading rate [rad/s]
template <typename Scalar>
Scalar chi_dot_btt(const Scalar &lift, const Scalar &mass,
                   const Scalar &velocity, const Scalar &gamma,
                   const Scalar &phi) {
    Scalar sin_phi = janus::sin(phi);
    Scalar cos_gamma = janus::cos(gamma);
    return (lift * sin_phi) /
           (mass * velocity * (cos_gamma + Scalar(1e-12)) + Scalar(1e-12));
}

/// Compute load factor from lift
///
/// n = L / W
///
/// @param lift Lift magnitude [N]
/// @param weight Weight [N]
/// @return Load factor [g's]
template <typename Scalar>
Scalar load_factor_from_lift(const Scalar &lift, const Scalar &weight) {
    return lift / (weight + Scalar(1e-12));
}

/// Compute required bank angle for coordinated turn
///
/// For a given turn rate and velocity, the required bank angle is:
/// tan(φ) = V·χ̇ / g
///
/// @param velocity Speed [m/s]
/// @param chi_dot Desired heading rate [rad/s]
/// @param gravity Gravitational acceleration [m/s²]
/// @return Required bank angle [rad]
template <typename Scalar>
Scalar bank_for_turn_rate(const Scalar &velocity, const Scalar &chi_dot,
                          const Scalar &gravity) {
    return janus::atan(velocity * chi_dot / gravity);
}

} // namespace vulcan::dynamics
