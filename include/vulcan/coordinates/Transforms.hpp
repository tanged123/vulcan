// Vulcan Coordinate Transforms
// Velocity transforms and non-inertial accelerations for ECEF/ECI
#pragma once

#include <vulcan/coordinates/CoordinateFrame.hpp>
#include <vulcan/coordinates/EarthModel.hpp>
#include <vulcan/core/VulcanTypes.hpp>

#include <janus/math/Linalg.hpp>

namespace vulcan {

// =============================================================================
// Velocity Transforms
// =============================================================================

/// Transform velocity from ECEF to ECI
///
/// The velocity transformation from ECEF to ECI accounts for Earth's rotation:
///   v_eci = v_ecef + omega x r_ecef
///
/// where omega is Earth's angular velocity vector [0, 0, omega].
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param v_ecef Velocity in ECEF [m/s]
/// @param r_ecef Position in ECEF [m]
/// @param eci ECI frame at current epoch
/// @param m Earth model (for angular velocity)
/// @return Velocity in ECI [m/s]
template <typename Scalar>
Vec3<Scalar> velocity_ecef_to_eci(const Vec3<Scalar> &v_ecef,
                                  const Vec3<Scalar> &r_ecef,
                                  const CoordinateFrame<Scalar> &eci,
                                  const EarthModel &m = EarthModel::WGS84()) {
    // Earth's angular velocity vector in ECEF: [0, 0, omega]
    Vec3<Scalar> omega_ecef;
    omega_ecef << Scalar(0), Scalar(0), Scalar(m.omega);

    // omega x r gives velocity contribution from rotation
    Vec3<Scalar> omega_cross_r = janus::cross(omega_ecef, r_ecef);

    // v_ecef_inertial = v_ecef + omega x r (velocity of ECEF point in inertial
    // frame)
    Vec3<Scalar> v_ecef_inertial = v_ecef + omega_cross_r;

    // Transform to ECI
    return eci.from_ecef(v_ecef_inertial);
}

/// Transform velocity from ECI to ECEF
///
/// The inverse of velocity_ecef_to_eci:
///   v_ecef = v_eci - omega x r_ecef
///
/// @tparam Scalar Scalar type
/// @param v_eci Velocity in ECI [m/s]
/// @param r_ecef Position in ECEF [m]
/// @param eci ECI frame at current epoch
/// @param m Earth model
/// @return Velocity in ECEF [m/s]
template <typename Scalar>
Vec3<Scalar> velocity_eci_to_ecef(const Vec3<Scalar> &v_eci,
                                  const Vec3<Scalar> &r_ecef,
                                  const CoordinateFrame<Scalar> &eci,
                                  const EarthModel &m = EarthModel::WGS84()) {
    // Transform v_eci to ECEF
    Vec3<Scalar> v_ecef_inertial = eci.to_ecef(v_eci);

    // Earth's angular velocity in ECEF
    Vec3<Scalar> omega_ecef;
    omega_ecef << Scalar(0), Scalar(0), Scalar(m.omega);

    // Subtract omega x r
    Vec3<Scalar> omega_cross_r = janus::cross(omega_ecef, r_ecef);

    return v_ecef_inertial - omega_cross_r;
}

// =============================================================================
// Non-Inertial Accelerations
// =============================================================================

/// Compute fictitious accelerations for ECEF equations of motion
///
/// When working in ECEF (rotating frame), the equations of motion include
/// fictitious forces. The acceleration transformation is:
///
///   a_ecef = a_inertial - 2(omega x v_ecef) - omega x (omega x r_ecef)
///
/// This function returns the fictitious acceleration term:
///   a_fictitious = -2(omega x v_ecef) - omega x (omega x r_ecef)
///
/// which contains:
/// - Coriolis acceleration: -2(omega x v_ecef)
/// - Centrifugal acceleration: -omega x (omega x r_ecef)
///
/// To use in ECEF EOMs:
///   a_ecef = a_gravity + a_drag + ... + coriolis_centrifugal(r, v)
///
/// @tparam Scalar Scalar type
/// @param r_ecef Position in ECEF [m]
/// @param v_ecef Velocity in ECEF [m/s]
/// @param m Earth model
/// @return Fictitious acceleration in ECEF [m/s^2]
template <typename Scalar>
Vec3<Scalar> coriolis_centrifugal(const Vec3<Scalar> &r_ecef,
                                  const Vec3<Scalar> &v_ecef,
                                  const EarthModel &m = EarthModel::WGS84()) {
    // Earth's angular velocity vector in ECEF: [0, 0, omega]
    Vec3<Scalar> omega;
    omega << Scalar(0), Scalar(0), Scalar(m.omega);

    // Coriolis: -2(omega x v)
    Vec3<Scalar> coriolis = Scalar(-2) * janus::cross(omega, v_ecef);

    // Centrifugal: -omega x (omega x r)
    Vec3<Scalar> omega_cross_r = janus::cross(omega, r_ecef);
    Vec3<Scalar> centrifugal = -janus::cross(omega, omega_cross_r);

    return coriolis + centrifugal;
}

/// Compute only Coriolis acceleration
///
/// @tparam Scalar Scalar type
/// @param v_ecef Velocity in ECEF [m/s]
/// @param m Earth model
/// @return Coriolis acceleration in ECEF [m/s^2]
template <typename Scalar>
Vec3<Scalar> coriolis_acceleration(const Vec3<Scalar> &v_ecef,
                                   const EarthModel &m = EarthModel::WGS84()) {
    Vec3<Scalar> omega;
    omega << Scalar(0), Scalar(0), Scalar(m.omega);

    return Scalar(-2) * janus::cross(omega, v_ecef);
}

/// Compute only centrifugal acceleration
///
/// @tparam Scalar Scalar type
/// @param r_ecef Position in ECEF [m]
/// @param m Earth model
/// @return Centrifugal acceleration in ECEF [m/s^2]
template <typename Scalar>
Vec3<Scalar>
centrifugal_acceleration(const Vec3<Scalar> &r_ecef,
                         const EarthModel &m = EarthModel::WGS84()) {
    Vec3<Scalar> omega;
    omega << Scalar(0), Scalar(0), Scalar(m.omega);

    Vec3<Scalar> omega_cross_r = janus::cross(omega, r_ecef);
    return -janus::cross(omega, omega_cross_r);
}

// =============================================================================
// Relative Position/Velocity
// =============================================================================

/// Compute relative position from observer to target
///
/// @tparam Scalar Scalar type
/// @param r_target Target position in frame [m]
/// @param r_observer Observer position in frame [m]
/// @return Relative position (target - observer) [m]
template <typename Scalar>
Vec3<Scalar> relative_position(const Vec3<Scalar> &r_target,
                               const Vec3<Scalar> &r_observer) {
    return r_target - r_observer;
}

/// Compute range (distance) between two positions
///
/// @tparam Scalar Scalar type
/// @param r1 First position [m]
/// @param r2 Second position [m]
/// @return Distance [m]
template <typename Scalar>
Scalar range(const Vec3<Scalar> &r1, const Vec3<Scalar> &r2) {
    return janus::norm(r1 - r2);
}

/// Compute range rate (velocity along line-of-sight)
///
/// @tparam Scalar Scalar type
/// @param r_target Target position [m]
/// @param v_target Target velocity [m/s]
/// @param r_observer Observer position [m]
/// @param v_observer Observer velocity [m/s]
/// @return Range rate (positive = increasing range) [m/s]
template <typename Scalar>
Scalar range_rate(const Vec3<Scalar> &r_target, const Vec3<Scalar> &v_target,
                  const Vec3<Scalar> &r_observer,
                  const Vec3<Scalar> &v_observer) {
    Vec3<Scalar> r_rel = r_target - r_observer;
    Vec3<Scalar> v_rel = v_target - v_observer;

    Scalar r_mag = janus::norm(r_rel);
    Scalar eps = Scalar(1e-10);
    Scalar is_zero = r_mag < eps;

    // Range rate = relative velocity dot unit line-of-sight
    Scalar rdot = janus::dot(r_rel, v_rel) / r_mag;

    // Handle zero range case
    return janus::where(is_zero, Scalar(0), rdot);
}

// =============================================================================
// State Propagation Helpers
// =============================================================================

/// Angular velocity of NED frame in ECEF (for propagating attitude)
///
/// As a vehicle moves over Earth's surface, the local NED frame rotates
/// relative to ECEF. This function computes the angular velocity of NED
/// wrt ECEF, expressed in NED coordinates.
///
/// omega_ned = [-v_n/R_n, v_e/R_e, v_e*tan(lat)/R_e]
///
/// where R_n = radius of curvature in meridian
///       R_e = radius of curvature in prime vertical
///
/// @tparam Scalar Scalar type
/// @param v_ned Velocity in NED [m/s]
/// @param lat Geodetic latitude [rad]
/// @param alt Altitude above ellipsoid [m]
/// @param m Earth model
/// @return Angular velocity of NED frame wrt ECEF, expressed in NED [rad/s]
template <typename Scalar>
Vec3<Scalar> omega_ned_wrt_ecef(const Vec3<Scalar> &v_ned, Scalar lat,
                                Scalar alt,
                                const EarthModel &m = EarthModel::WGS84()) {
    Scalar sin_lat = janus::sin(lat);
    Scalar cos_lat = janus::cos(lat);

    // Radius of curvature in meridian
    Scalar denom = janus::sqrt(Scalar(1) - Scalar(m.e2) * sin_lat * sin_lat);
    Scalar R_n =
        Scalar(m.a) * (Scalar(1) - Scalar(m.e2)) / (denom * denom * denom);
    R_n = R_n + alt;

    // Radius of curvature in prime vertical
    Scalar R_e = Scalar(m.a) / denom + alt;

    // Angular velocity of NED wrt ECEF in NED coordinates
    Vec3<Scalar> omega;
    omega << -v_ned(0) / R_n, v_ned(1) / R_e, v_ned(1) * janus::tan(lat) / R_e;

    return omega;
}

/// Angular velocity of NED frame wrt ECI (includes Earth rotation)
///
/// omega_ned_eci = omega_ned_ecef + omega_earth
///
/// where omega_earth is Earth's rotation expressed in NED.
///
/// @tparam Scalar Scalar type
/// @param v_ned Velocity in NED [m/s]
/// @param lat Geodetic latitude [rad]
/// @param alt Altitude above ellipsoid [m]
/// @param m Earth model
/// @return Angular velocity of NED frame wrt ECI, expressed in NED [rad/s]
template <typename Scalar>
Vec3<Scalar> omega_ned_wrt_eci(const Vec3<Scalar> &v_ned, Scalar lat,
                               Scalar alt,
                               const EarthModel &m = EarthModel::WGS84()) {
    // Angular velocity of NED wrt ECEF
    Vec3<Scalar> omega_ned_ecef = omega_ned_wrt_ecef(v_ned, lat, alt, m);

    // Earth's angular velocity in NED: [omega*cos(lat), 0, -omega*sin(lat)]
    Scalar sin_lat = janus::sin(lat);
    Scalar cos_lat = janus::cos(lat);

    Vec3<Scalar> omega_earth_ned;
    omega_earth_ned << Scalar(m.omega) * cos_lat, Scalar(0),
        -Scalar(m.omega) * sin_lat;

    return omega_ned_ecef + omega_earth_ned;
}

} // namespace vulcan
