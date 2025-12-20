// Vulcan Body Frames
// Body-fixed frames, Euler angles, and velocity frame utilities
#pragma once

#include <vulcan/aerodynamics/Aerodynamics.hpp>
#include <vulcan/coordinates/CoordinateFrame.hpp>
#include <vulcan/coordinates/Geodetic.hpp>
#include <vulcan/coordinates/LocalFrames.hpp>
#include <vulcan/rotations/EulerSequences.hpp>

#include <janus/math/Linalg.hpp>
#include <janus/math/Quaternion.hpp>
#include <janus/math/Trig.hpp>

namespace vulcan {

// =============================================================================
// Quaternion-Based Body Frame Construction
// =============================================================================

/// Create body-fixed frame from quaternion relative to NED
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param ned Reference NED frame (origin for body frame)
/// @param q_body_ned Quaternion rotating from NED to body frame
/// @return Body-fixed frame expressed in ECEF
template <typename Scalar>
CoordinateFrame<Scalar>
body_from_quaternion(const CoordinateFrame<Scalar> &ned,
                     const janus::Quaternion<Scalar> &q_body_to_ned) {
    // q_body_to_ned transforms body vectors to NED vectors: v_ned =
    // q.rotate(v_body) The body axes (unit vectors in body coords) rotated to
    // NED give body axes in NED

    // Body axes in body coordinates
    Vec3<Scalar> x_body = Vec3<Scalar>::UnitX();
    Vec3<Scalar> y_body = Vec3<Scalar>::UnitY();
    Vec3<Scalar> z_body = Vec3<Scalar>::UnitZ();

    // Body axes expressed in NED (rotate body unit vectors to NED)
    Vec3<Scalar> x_body_ned = q_body_to_ned.rotate(x_body);
    Vec3<Scalar> y_body_ned = q_body_to_ned.rotate(y_body);
    Vec3<Scalar> z_body_ned = q_body_to_ned.rotate(z_body);

    // Transform body axes from NED to ECEF
    Vec3<Scalar> x_body_ecef = ned.to_ecef(x_body_ned);
    Vec3<Scalar> y_body_ecef = ned.to_ecef(y_body_ned);
    Vec3<Scalar> z_body_ecef = ned.to_ecef(z_body_ned);

    return CoordinateFrame<Scalar>(x_body_ecef, y_body_ecef, z_body_ecef,
                                   ned.origin);
}

/// Extract quaternion representing body orientation relative to NED
///
/// @tparam Scalar Scalar type
/// @param body Body-fixed frame
/// @param ned Reference NED frame
/// @return Quaternion rotating from body to NED (v_ned = q.rotate(v_body))
template <typename Scalar>
janus::Quaternion<Scalar>
quaternion_from_body(const CoordinateFrame<Scalar> &body,
                     const CoordinateFrame<Scalar> &ned) {
    // Get body axes in NED coordinates
    Vec3<Scalar> x_body_ned = ned.from_ecef(body.x_axis);
    Vec3<Scalar> y_body_ned = ned.from_ecef(body.y_axis);
    Vec3<Scalar> z_body_ned = ned.from_ecef(body.z_axis);

    // Build rotation matrix (body axes as columns)
    // R transforms body vectors to NED: v_ned = R * v_body
    Mat3<Scalar> R;
    R.col(0) = x_body_ned;
    R.col(1) = y_body_ned;
    R.col(2) = z_body_ned;

    // Return quaternion that rotates body to NED (same as from_euler
    // convention)
    return janus::Quaternion<Scalar>::from_rotation_matrix(R);
}

// =============================================================================
// Euler Angle Construction (Using Quaternions Internally)
// =============================================================================

/// Create body-fixed frame from Euler angles relative to NED
///
/// Rotation sequence: Yaw (Z) -> Pitch (Y') -> Roll (X'')
/// This is the standard aerospace Tait-Bryan rotation sequence.
///
/// Internally uses quaternions to avoid gimbal lock issues during composition.
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param ned Reference NED frame (origin for body frame)
/// @param yaw Heading angle from North, positive clockwise [rad]
/// @param pitch Pitch angle, positive nose up [rad]
/// @param roll Roll angle, positive right wing down [rad]
/// @return Body-fixed frame expressed in ECEF
template <typename Scalar>
CoordinateFrame<Scalar> body_from_euler(const CoordinateFrame<Scalar> &ned,
                                        Scalar yaw, Scalar pitch, Scalar roll) {
    // Janus from_euler uses (roll, pitch, yaw) for XYZ intrinsic sequence
    // which corresponds to ZYX extrinsic (yaw-pitch-roll aerospace convention)
    auto q = janus::Quaternion<Scalar>::from_euler(roll, pitch, yaw);
    return body_from_quaternion(ned, q);
}

// =============================================================================
// Euler Angle Extraction (Using Rotations Library)
// =============================================================================

/// Extract Euler angles from body frame relative to NED
///
/// Returns [yaw, pitch, roll] in the Yaw-Pitch-Roll (Z-Y'-X'') sequence.
///
/// Uses the unified rotations library for DCM-to-Euler conversion with
/// proper gimbal lock handling. This is compatible with symbolic mode.
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param body Body-fixed frame
/// @param ned Reference NED frame
/// @return [yaw, pitch, roll] angles [rad]
template <typename Scalar>
Vec3<Scalar> euler_from_body(const CoordinateFrame<Scalar> &body,
                             const CoordinateFrame<Scalar> &ned) {
    // Build DCM: columns are body axes in NED coordinates
    // v_ned = DCM * v_body
    Mat3<Scalar> dcm;
    dcm.col(0) = ned.from_ecef(body.x_axis);
    dcm.col(1) = ned.from_ecef(body.y_axis);
    dcm.col(2) = ned.from_ecef(body.z_axis);

    // Use unified Euler extraction with gimbal lock handling
    return euler_from_dcm(dcm, EulerSequence::ZYX);
}

// =============================================================================
// Velocity Frame
// =============================================================================

/// Create velocity frame (wind axes) from Earth-relative velocity
///
/// The velocity frame has:
/// - X-axis: Aligned with velocity vector (forward)
/// - Y-axis: Perpendicular to velocity, in local horizontal plane (right)
/// - Z-axis: Completes right-handed system
///
/// This is commonly used for aerodynamic analysis.
///
/// @tparam Scalar Scalar type
/// @param velocity_ecef Velocity vector in ECEF [m/s]
/// @param ned Reference NED frame at current position
/// @return Velocity frame expressed in ECEF
template <typename Scalar>
CoordinateFrame<Scalar> velocity_frame(const Vec3<Scalar> &velocity_ecef,
                                       const CoordinateFrame<Scalar> &ned) {
    // Get velocity in NED
    Vec3<Scalar> v_ned = ned.from_ecef(velocity_ecef);

    // Velocity magnitude
    Scalar v_mag = janus::norm(velocity_ecef);
    Scalar eps = Scalar(1e-10);

    // Handle zero velocity case - use NED as default
    Scalar is_zero = v_mag < eps;

    // X-axis: velocity direction
    Vec3<Scalar> x_vel_ned;
    x_vel_ned << v_ned(0) / v_mag, v_ned(1) / v_mag, v_ned(2) / v_mag;

    // For near-zero velocity, default to North
    x_vel_ned(0) = janus::where(is_zero, Scalar(1), x_vel_ned(0));
    x_vel_ned(1) = janus::where(is_zero, Scalar(0), x_vel_ned(1));
    x_vel_ned(2) = janus::where(is_zero, Scalar(0), x_vel_ned(2));

    // Horizontal velocity magnitude
    Scalar v_horiz = janus::sqrt(v_ned(0) * v_ned(0) + v_ned(1) * v_ned(1));
    Scalar is_vertical = v_horiz < eps;

    // Y-axis: Horizontal, perpendicular to velocity (to the right)
    Vec3<Scalar> y_vel_ned;
    y_vel_ned << v_ned(1) / v_horiz, -v_ned(0) / v_horiz, Scalar(0);

    // For vertical flight, Y defaults to East
    y_vel_ned(0) = janus::where(is_vertical, Scalar(0), y_vel_ned(0));
    y_vel_ned(1) = janus::where(is_vertical, Scalar(1), y_vel_ned(1));
    y_vel_ned(2) = janus::where(is_vertical, Scalar(0), y_vel_ned(2));

    // For zero velocity, Y defaults to East
    y_vel_ned(0) = janus::where(is_zero, Scalar(0), y_vel_ned(0));
    y_vel_ned(1) = janus::where(is_zero, Scalar(1), y_vel_ned(1));

    // Z-axis: Complete right-handed system: z = x cross y
    Vec3<Scalar> z_vel_ned = janus::cross(x_vel_ned, y_vel_ned);

    // Transform to ECEF
    Vec3<Scalar> x_vel = ned.to_ecef(x_vel_ned);
    Vec3<Scalar> y_vel = ned.to_ecef(y_vel_ned);
    Vec3<Scalar> z_vel = ned.to_ecef(z_vel_ned);

    return CoordinateFrame<Scalar>(x_vel, y_vel, z_vel, ned.origin);
}

// =============================================================================
// Flight Path Angles
// =============================================================================

/// Compute flight path angles from velocity in NED frame
///
/// @tparam Scalar Scalar type
/// @param velocity_ned Velocity vector in NED [m/s]
/// @return [gamma, psi] where:
///         gamma = flight path angle (vertical), positive up [rad]
///         psi = heading angle from North, positive clockwise [rad]
template <typename Scalar>
Vec2<Scalar> flight_path_angles(const Vec3<Scalar> &velocity_ned) {
    Scalar vn = velocity_ned(0);
    Scalar ve = velocity_ned(1);
    Scalar vd = velocity_ned(2);

    // Horizontal speed
    Scalar v_horiz = janus::sqrt(vn * vn + ve * ve);

    // Total speed
    Scalar v_total = janus::norm(velocity_ned);
    Scalar eps = Scalar(1e-10);
    Scalar is_zero = v_total < eps;

    // Flight path angle: gamma = atan2(-vd, v_horiz)
    // For climbing, vd < 0, so gamma > 0
    Scalar gamma = janus::atan2(-vd, v_horiz);
    gamma = janus::where(is_zero, Scalar(0), gamma);

    // Heading: psi = atan2(ve, vn)
    Scalar psi = janus::atan2(ve, vn);
    psi = janus::where(is_zero, Scalar(0), psi);

    Vec2<Scalar> angles;
    angles << gamma, psi;
    return angles;
}

/// Compute flight path angles from ECEF velocity and position
///
/// @tparam Scalar Scalar type
/// @param velocity_ecef Velocity vector in ECEF [m/s]
/// @param ned NED frame at current position
/// @return [gamma, psi] flight path angles [rad]
template <typename Scalar>
Vec2<Scalar> flight_path_angles(const Vec3<Scalar> &velocity_ecef,
                                const CoordinateFrame<Scalar> &ned) {
    Vec3<Scalar> v_ned = ned.from_ecef(velocity_ecef);
    return flight_path_angles(v_ned);
}

// =============================================================================
// Aerodynamic Angles
// =============================================================================

// Core aero_angles implementation is in vulcan::aero namespace
// Re-export for backwards compatibility
using aero::aero_angles;

/// Compute aerodynamic angles from ECEF velocity and body frame
///
/// @tparam Scalar Scalar type
/// @param velocity_ecef Velocity vector in ECEF [m/s]
/// @param body Body-fixed frame
/// @return [alpha, beta] aerodynamic angles [rad]
template <typename Scalar>
Vec2<Scalar> aero_angles(const Vec3<Scalar> &velocity_ecef,
                         const CoordinateFrame<Scalar> &body) {
    Vec3<Scalar> v_body = body.from_ecef(velocity_ecef);
    return aero_angles(v_body);
}

} // namespace vulcan
