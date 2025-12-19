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

} // namespace vulcan
