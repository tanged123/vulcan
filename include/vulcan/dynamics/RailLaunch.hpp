// Vulcan Rail Launch Dynamics
// Pure stateless functions for rail-constrained motion
#pragma once

#include <vulcan/core/VulcanTypes.hpp>

#include <janus/janus.hpp>

namespace vulcan::dynamics {

// =============================================================================
// Rail Direction Utilities
// =============================================================================

/// Compute rail direction unit vector from elevation and azimuth
///
/// @param azimuth Rail azimuth from North [rad]
/// @param elevation Rail elevation from horizontal [rad]
/// @return Unit direction vector in NED frame
template <typename Scalar>
Vec3<Scalar> rail_direction_ned(const Scalar &azimuth,
                                const Scalar &elevation) {
    // In NED: North=+X, East=+Y, Down=+Z
    // Azimuth from North, elevation from horizontal
    Scalar cos_elev = janus::cos(elevation);
    Scalar sin_elev = janus::sin(elevation);
    Scalar cos_az = janus::cos(azimuth);
    Scalar sin_az = janus::sin(azimuth);

    return Vec3<Scalar>{
        cos_elev * cos_az, // North component
        cos_elev * sin_az, // East component
        -sin_elev          // Down component (negative because up is -Z)
    };
}

// =============================================================================
// Rail Launch Dynamics (1-DOF)
// =============================================================================

/// Compute rail launch acceleration
///
/// Constrains vehicle to 1-DOF motion along rail axis.
/// Forces perpendicular to rail are reacted by the rail (not integrated).
///
/// Equation of motion along rail:
/// $$\ddot{s} = (F_{along} - \mu |F_\perp| \cdot \text{sign}(\dot{s})) / m - g
/// \sin(\theta)$$
///
/// @tparam Scalar Scalar type (double or casadi::MX)
/// @param s_dot Velocity along rail [m/s]
/// @param force_along Force component along rail axis [N]
/// @param force_perp_mag Magnitude of perpendicular force [N]
/// @param mass Vehicle mass [kg]
/// @param gravity Gravitational acceleration [m/s²]
/// @param elevation Rail elevation angle from horizontal [rad]
/// @param friction_coeff Rail friction coefficient (Coulomb)
/// @return Acceleration along rail [m/s²]
template <typename Scalar>
Scalar rail_acceleration(const Scalar &s_dot, const Scalar &force_along,
                         const Scalar &force_perp_mag, const Scalar &mass,
                         const Scalar &gravity, const Scalar &elevation,
                         const Scalar &friction_coeff) {
    // Gravity component along rail (positive opposes motion for uphill)
    Scalar gravity_along = gravity * janus::sin(elevation);

    // Friction force (Coulomb friction opposing motion direction)
    Scalar friction_sign =
        janus::where(s_dot > Scalar(0), Scalar(1),
                     janus::where(s_dot < Scalar(0), Scalar(-1), Scalar(0)));
    Scalar friction_force = friction_coeff * force_perp_mag * friction_sign;

    // Net acceleration along rail
    return (force_along - friction_force) / mass - gravity_along;
}

/// Compute rail acceleration from body-frame forces
///
/// Assumes body X-axis is aligned with rail direction.
///
/// @param s_dot Velocity along rail [m/s]
/// @param force_body Sum of forces in body frame [N]
/// @param mass Vehicle mass [kg]
/// @param gravity Gravitational acceleration [m/s²]
/// @param elevation Rail elevation angle from horizontal [rad]
/// @param friction_coeff Rail friction coefficient
/// @return Acceleration along rail [m/s²]
template <typename Scalar>
Scalar
rail_acceleration_body(const Scalar &s_dot, const Vec3<Scalar> &force_body,
                       const Scalar &mass, const Scalar &gravity,
                       const Scalar &elevation, const Scalar &friction_coeff) {
    // Force along rail (body X)
    Scalar force_along = force_body(0);

    // Perpendicular force magnitude (body Y and Z)
    Scalar force_perp_mag = janus::sqrt(force_body(1) * force_body(1) +
                                        force_body(2) * force_body(2));

    return rail_acceleration(s_dot, force_along, force_perp_mag, mass, gravity,
                             elevation, friction_coeff);
}

/// Check if vehicle is on rail
///
/// @param s Position along rail [m]
/// @param rail_length Total rail length [m]
/// @return 1.0 if on rail, 0.0 if departed (symbolic compatible)
template <typename Scalar>
Scalar on_rail(const Scalar &s, const Scalar &rail_length) {
    return janus::where(s < rail_length,
                        janus::where(s >= Scalar(0), Scalar(1), Scalar(0)),
                        Scalar(0));
}

// =============================================================================
// Position/Velocity from Rail State
// =============================================================================

/// Compute position in reference frame from rail position
///
/// @param s Position along rail [m]
/// @param origin Rail start point in reference frame [m]
/// @param direction Unit direction vector along rail
/// @return Position in reference frame [m]
template <typename Scalar>
Vec3<Scalar> rail_position(const Scalar &s, const Vec3<Scalar> &origin,
                           const Vec3<Scalar> &direction) {
    return origin + s * direction;
}

/// Compute velocity in reference frame from rail velocity
///
/// @param s_dot Velocity along rail [m/s]
/// @param direction Unit direction vector along rail
/// @return Velocity in reference frame [m/s]
template <typename Scalar>
Vec3<Scalar> rail_velocity(const Scalar &s_dot, const Vec3<Scalar> &direction) {
    return s_dot * direction;
}

// =============================================================================
// Attitude for Rail-Aligned Body
// =============================================================================

/// Create attitude quaternion for rail-aligned body
///
/// Body X-axis aligned with rail direction, Z-axis roughly "up" (opposite
/// gravity)
///
/// @param azimuth Rail azimuth from North [rad]
/// @param elevation Rail elevation from horizontal [rad]
/// @return Quaternion rotating from body frame to NED reference
template <typename Scalar>
janus::Quaternion<Scalar> rail_aligned_attitude(const Scalar &azimuth,
                                                const Scalar &elevation) {
    // Construct rotation: first rotate about Z by azimuth, then about Y by
    // pitch This aligns body +X with rail direction in NED frame

    // Yaw (azimuth rotation about NED Z-axis)
    Scalar half_az = azimuth / Scalar(2);
    janus::Quaternion<Scalar> q_yaw(janus::cos(half_az), Scalar(0), Scalar(0),
                                    janus::sin(half_az));

    // Pitch (elevation rotation about body Y-axis)
    // Positive elevation = pitch up = body X toward NED -Z
    Scalar half_pitch = elevation / Scalar(2);
    janus::Quaternion<Scalar> q_pitch(janus::cos(half_pitch), Scalar(0),
                                      janus::sin(half_pitch), Scalar(0));

    // Combined rotation: yaw then pitch
    return q_yaw * q_pitch;
}

/// Compute rail reaction force (perpendicular force reacted by rail)
///
/// @param force_body Sum of forces in body frame [N]
/// @return Reaction force from rail on vehicle [N]
template <typename Scalar>
Vec3<Scalar> rail_reaction_force(const Vec3<Scalar> &force_body) {
    // Rail reacts perpendicular components (Y and Z in body frame)
    return Vec3<Scalar>{Scalar(0), -force_body(1), -force_body(2)};
}

} // namespace vulcan::dynamics
