// Vulcan Local Frames
// Convenience functions for local tangent plane reference frames
#pragma once

#include <vulcan/coordinates/CoordinateFrame.hpp>
#include <vulcan/coordinates/Geodetic.hpp>

#include <janus/math/Trig.hpp>

namespace vulcan {

// =============================================================================
// Local Geodetic Horizon Frames
// =============================================================================

/// Create Local Geodetic Horizon frame (NED — North, East, Down)
///
/// The NED frame is the standard aerospace local horizon frame:
/// - X-axis: Points North (tangent to meridian, toward North Pole)
/// - Y-axis: Points East (tangent to parallel of latitude)
/// - Z-axis: Points Down (opposite to ellipsoid normal)
///
/// This is a convenience wrapper around CoordinateFrame::ned().
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param lon Longitude [rad]
/// @param lat_gd Geodetic latitude [rad]
/// @return NED frame expressed in ECEF
template <typename Scalar>
CoordinateFrame<Scalar> local_ned(Scalar lon, Scalar lat_gd) {
    return CoordinateFrame<Scalar>::ned(lon, lat_gd);
}

/// Create Local Geodetic Horizon frame (ENU — East, North, Up)
///
/// The ENU frame is common in surveying and geodesy:
/// - X-axis: Points East (tangent to parallel of latitude)
/// - Y-axis: Points North (tangent to meridian, toward North Pole)
/// - Z-axis: Points Up (ellipsoid normal)
///
/// This is a convenience wrapper around CoordinateFrame::enu().
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param lon Longitude [rad]
/// @param lat_gd Geodetic latitude [rad]
/// @return ENU frame expressed in ECEF
template <typename Scalar>
CoordinateFrame<Scalar> local_enu(Scalar lon, Scalar lat_gd) {
    return CoordinateFrame<Scalar>::enu(lon, lat_gd);
}

// =============================================================================
// Local Geocentric Horizon Frame
// =============================================================================

/// Create Local Geocentric Horizon frame
///
/// Similar to NED but uses geocentric latitude instead of geodetic.
/// The "down" direction points toward Earth's center rather than
/// perpendicular to the ellipsoid surface.
///
/// For a spherical Earth, geocentric and geodetic frames are identical.
/// For an oblate Earth, the difference is most pronounced at mid-latitudes
/// (up to ~12 arcminutes for WGS84).
///
/// - X-axis: Points North (in the meridian plane, perpendicular to radius)
/// - Y-axis: Points East (tangent to parallel of latitude)
/// - Z-axis: Points toward Earth center (radially inward)
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param lon Longitude [rad]
/// @param lat_gc Geocentric latitude [rad]
/// @return Geocentric horizon frame expressed in ECEF
template <typename Scalar>
CoordinateFrame<Scalar> local_geocentric(Scalar lon, Scalar lat_gc) {
    Scalar sin_lat = janus::sin(lat_gc);
    Scalar cos_lat = janus::cos(lat_gc);
    Scalar sin_lon = janus::sin(lon);
    Scalar cos_lon = janus::cos(lon);

    // North: perpendicular to radial direction, in meridian plane, pointing
    // north
    //   = -sin(lat_gc)*cos(lon), -sin(lat_gc)*sin(lon), cos(lat_gc)
    Vec3<Scalar> north;
    north << -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat;

    // East: tangent to parallel of latitude
    //   = -sin(lon), cos(lon), 0
    Vec3<Scalar> east;
    east << -sin_lon, cos_lon, Scalar(0);

    // Down: toward Earth center (negative of radial unit vector)
    //   = -cos(lat_gc)*cos(lon), -cos(lat_gc)*sin(lon), -sin(lat_gc)
    Vec3<Scalar> down;
    down << -cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat;

    return CoordinateFrame<Scalar>(north, east, down, Vec3<Scalar>::Zero());
}

/// Create Local Geocentric Horizon frame at a given ECEF position
///
/// Computes the geocentric latitude and longitude from the position
/// and returns the corresponding geocentric horizon frame.
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param r_ecef Position in ECEF [m]
/// @return Geocentric horizon frame expressed in ECEF
template <typename Scalar>
CoordinateFrame<Scalar> local_geocentric_at(const Vec3<Scalar> &r_ecef) {
    Spherical<Scalar> geo = ecef_to_spherical(r_ecef);
    return local_geocentric(geo.lon, geo.lat_gc);
}

/// Create Local Geodetic Horizon frame (NED) at a given ECEF position
///
/// Computes the geodetic latitude and longitude from the position
/// using the WGS84 ellipsoid and returns the NED frame.
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param r_ecef Position in ECEF [m]
/// @param m Earth model (default: WGS84)
/// @return NED frame expressed in ECEF
template <typename Scalar>
CoordinateFrame<Scalar>
local_ned_at(const Vec3<Scalar> &r_ecef,
             const EarthModel &m = EarthModel::WGS84()) {
    LLA<Scalar> lla = ecef_to_lla(r_ecef, m);
    return local_ned(lla.lon, lla.lat);
}

/// Create Local Geodetic Horizon frame (ENU) at a given ECEF position
///
/// Computes the geodetic latitude and longitude from the position
/// using the WGS84 ellipsoid and returns the ENU frame.
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param r_ecef Position in ECEF [m]
/// @param m Earth model (default: WGS84)
/// @return ENU frame expressed in ECEF
template <typename Scalar>
CoordinateFrame<Scalar>
local_enu_at(const Vec3<Scalar> &r_ecef,
             const EarthModel &m = EarthModel::WGS84()) {
    LLA<Scalar> lla = ecef_to_lla(r_ecef, m);
    return local_enu(lla.lon, lla.lat);
}

// =============================================================================
// CDA Frame (Cross-range, Down-range, Altitude) — Trajectory-Relative
// =============================================================================

/// Create CDA frame at a given position with specified bearing
///
/// The CDA frame is a trajectory-relative local tangent plane:
/// - X-axis (D): Down-range, along great-circle bearing
/// - Y-axis (C): Cross-range, perpendicular to down-range (right-hand)
/// - Z-axis (A): Altitude, up from ellipsoid surface
///
/// This is commonly used for impact prediction and range safety.
/// The frame is constructed by rotating the local ENU frame about
/// the Up axis by (90° - bearing) to align X with the bearing direction.
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param lla_origin Origin position
/// @param bearing Down-range bearing [rad], clockwise from North
/// @param m Earth model (default: WGS84)
/// @return CDA frame expressed in ECEF
template <typename Scalar>
CoordinateFrame<Scalar>
local_cda(const LLA<Scalar> &lla_origin, const Scalar &bearing,
          [[maybe_unused]] const EarthModel &m = EarthModel::WGS84()) {
    // Start with local ENU frame at origin
    const Scalar sin_lat = janus::sin(lla_origin.lat);
    const Scalar cos_lat = janus::cos(lla_origin.lat);
    const Scalar sin_lon = janus::sin(lla_origin.lon);
    const Scalar cos_lon = janus::cos(lla_origin.lon);

    // ENU basis vectors in ECEF
    Vec3<Scalar> east;
    east << -sin_lon, cos_lon, Scalar(0);

    Vec3<Scalar> north;
    north << -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat;

    Vec3<Scalar> up;
    up << cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;

    // Rotate about Up axis by bearing angle
    // Down-range = cos(bearing) * North + sin(bearing) * East
    // Cross-range = -sin(bearing) * North + cos(bearing) * East (right-hand)
    const Scalar sin_b = janus::sin(bearing);
    const Scalar cos_b = janus::cos(bearing);

    Vec3<Scalar> downrange = cos_b * north + sin_b * east;
    Vec3<Scalar> crossrange = -sin_b * north + cos_b * east;

    // CDA frame: D (downrange) as X, C (crossrange) as Y, A (altitude/up) as Z
    return CoordinateFrame<Scalar>(downrange, crossrange, up,
                                   Vec3<Scalar>::Zero());
}

/// Create CDA frame at a given ECEF position with bearing
///
/// Convenience overload that takes ECEF position.
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param r_ecef Position in ECEF [m]
/// @param bearing Down-range bearing [rad], clockwise from North
/// @param m Earth model (default: WGS84)
/// @return CDA frame expressed in ECEF
template <typename Scalar>
CoordinateFrame<Scalar>
local_cda_at(const Vec3<Scalar> &r_ecef, const Scalar &bearing,
             const EarthModel &m = EarthModel::WGS84()) {
    LLA<Scalar> lla = ecef_to_lla(r_ecef, m);
    return local_cda(lla, bearing, m);
}

// =============================================================================
// CDA Coordinate Conversions
// =============================================================================

/// Convert ECEF position to CDA coordinates relative to reference
///
/// Computes the cross-range, down-range, and altitude of an ECEF point
/// relative to a reference point with a given bearing direction.
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param r_ecef Position to convert (ECEF) [m]
/// @param lla_ref Reference point (origin of CDA frame)
/// @param bearing Down-range bearing [rad]
/// @param m Earth model (default: WGS84)
/// @return Vec3 with (down-range, cross-range, altitude) [m]
template <typename Scalar>
Vec3<Scalar> ecef_to_cda(const Vec3<Scalar> &r_ecef, const LLA<Scalar> &lla_ref,
                         const Scalar &bearing,
                         const EarthModel &m = EarthModel::WGS84()) {
    // Get CDA frame at reference point
    CoordinateFrame<Scalar> cda_frame = local_cda(lla_ref, bearing, m);

    // Get reference point in ECEF
    Vec3<Scalar> r_ref = lla_to_ecef(lla_ref, m);

    // Compute offset vector
    Vec3<Scalar> delta = r_ecef - r_ref;

    // Project onto CDA axes (frame axes are columns of rotation matrix)
    // CDA = R^T * delta where R = [downrange | crossrange | up]
    Vec3<Scalar> cda;
    cda(0) = delta.dot(cda_frame.x_axis); // Down-range
    cda(1) = delta.dot(cda_frame.y_axis); // Cross-range
    cda(2) = delta.dot(cda_frame.z_axis); // Altitude

    return cda;
}

/// Convert CDA coordinates to ECEF position
///
/// Converts cross-range, down-range, and altitude back to ECEF.
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param cda CDA coordinates (down-range, cross-range, altitude) [m]
/// @param lla_ref Reference point (origin of CDA frame)
/// @param bearing Down-range bearing [rad]
/// @param m Earth model (default: WGS84)
/// @return Position in ECEF [m]
template <typename Scalar>
Vec3<Scalar> cda_to_ecef(const Vec3<Scalar> &cda, const LLA<Scalar> &lla_ref,
                         const Scalar &bearing,
                         const EarthModel &m = EarthModel::WGS84()) {
    // Get CDA frame at reference point
    CoordinateFrame<Scalar> cda_frame = local_cda(lla_ref, bearing, m);

    // Get reference point in ECEF
    Vec3<Scalar> r_ref = lla_to_ecef(lla_ref, m);

    // Convert CDA to ECEF offset
    // delta = R * cda where R = [downrange | crossrange | up]
    Vec3<Scalar> delta = cda(0) * cda_frame.x_axis + cda(1) * cda_frame.y_axis +
                         cda(2) * cda_frame.z_axis;

    return r_ref + delta;
}

} // namespace vulcan
