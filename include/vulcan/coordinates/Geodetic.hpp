// Vulcan Geodetic Utilities
// ECEF <-> Geodetic conversions using Vermeille (2004) closed-form algorithm
#pragma once

#include <vulcan/coordinates/EarthModel.hpp>
#include <vulcan/core/VulcanTypes.hpp>

#include <janus/math/Arithmetic.hpp>
#include <janus/math/Logic.hpp>
#include <janus/math/Trig.hpp>

namespace vulcan {

// =============================================================================
// LLA - Geodetic Coordinates
// =============================================================================

/// Geodetic coordinates (Longitude, Latitude, Altitude)
///
/// Represents a position on or above an ellipsoidal Earth model.
/// - Longitude is measured positive East from Greenwich meridian
/// - Latitude is geodetic (angle from equatorial plane to ellipsoid normal)
/// - Altitude is height above the reference ellipsoid surface
///
/// @tparam Scalar Scalar type (double for numeric, janus::SymbolicScalar for
/// symbolic)
template <typename Scalar> struct LLA {
    Scalar lon; ///< Longitude [rad], positive East, range [-π, π]
    Scalar lat; ///< Geodetic latitude [rad], range [-π/2, π/2]
    Scalar alt; ///< Altitude above ellipsoid [m]

    LLA() : lon(Scalar(0)), lat(Scalar(0)), alt(Scalar(0)) {}
    LLA(Scalar lon_, Scalar lat_, Scalar alt_)
        : lon(lon_), lat(lat_), alt(alt_) {}
};

// =============================================================================
// Spherical - Geocentric Coordinates
// =============================================================================

/// Geocentric (spherical) coordinates
///
/// Represents a position in spherical coordinates centered at Earth's center.
/// Unlike geodetic coordinates, geocentric latitude is measured as the angle
/// from the equatorial plane to the position vector (not the ellipsoid normal).
///
/// @tparam Scalar Scalar type (double for numeric, janus::SymbolicScalar for
/// symbolic)
template <typename Scalar> struct Spherical {
    Scalar lon;    ///< Longitude [rad], positive East, range [-π, π]
    Scalar lat_gc; ///< Geocentric latitude [rad], range [-π/2, π/2]
    Scalar radius; ///< Distance from Earth center [m]

    Spherical() : lon(Scalar(0)), lat_gc(Scalar(0)), radius(Scalar(0)) {}
    Spherical(Scalar lon_, Scalar lat_gc_, Scalar radius_)
        : lon(lon_), lat_gc(lat_gc_), radius(radius_) {}
};

// =============================================================================
// ECEF to LLA - Vermeille (2004) Closed-Form Algorithm
// =============================================================================

/// Convert ECEF position to geodetic LLA using Vermeille (2004) algorithm
///
/// This is a closed-form, non-iterative algorithm that handles all edge cases
/// including poles, equator, and points at/near Earth's center. The algorithm
/// is fully symbolic-compatible with no conditional branches on Scalar values.
///
/// Reference: Vermeille, H. (2004). "Computing geodetic coordinates from
/// geocentric coordinates." Journal of Geodesy, 78, 94-95.
///
/// Accuracy: Sub-millimeter for all altitudes from -20km to geostationary orbit
///
/// @param r Position in ECEF [m]
/// @param m Earth model (default: WGS84)
/// @return LLA structure with (lon, lat, alt)
template <typename Scalar>
LLA<Scalar> ecef_to_lla(const Vec3<Scalar> &r,
                        const EarthModel &m = EarthModel::WGS84()) {
    const Scalar x = r(0);
    const Scalar y = r(1);
    const Scalar z = r(2);

    const double a = m.a;
    const double e2 = m.e2;
    const double e4 = e2 * e2;

    // Compute intermediate values
    // p = (x² + y²) / a²
    // q = (1 - e²) z² / a²
    const Scalar p = (x * x + y * y) / (a * a);
    const Scalar q = (1.0 - e2) * z * z / (a * a);

    // r = (p + q - e⁴) / 6
    const Scalar r_val = (p + q - e4) / 6.0;

    // Evolute parameters
    // s = e⁴ p q / (4 r³)
    const Scalar r_cubed = r_val * r_val * r_val;
    const Scalar s = e4 * p * q / (4.0 * r_cubed);

    // t = ∛(1 + s + √(s(2+s)))
    // Note: For numerical stability when s is small, this still works
    const Scalar s_term = s * (2.0 + s);
    const Scalar t = janus::pow(1.0 + s + janus::sqrt(s_term), 1.0 / 3.0);

    // u = r (1 + t + 1/t)
    const Scalar u = r_val * (1.0 + t + 1.0 / t);

    // v = √(u² + e⁴ q)
    const Scalar v = janus::sqrt(u * u + e4 * q);

    // w = e² (u + v - q) / (2v)
    const Scalar w = e2 * (u + v - q) / (2.0 * v);

    // k = √(u + v + w²) - w
    const Scalar k = janus::sqrt(u + v + w * w) - w;

    // D = k √(x² + y²) / (k + e²)
    const Scalar xy_dist = janus::sqrt(x * x + y * y);
    const Scalar D = k * xy_dist / (k + e2);

    // Compute geodetic latitude
    // φ = 2 atan2(z, D + √(D² + z²))
    const Scalar lat = 2.0 * janus::atan2(z, D + janus::sqrt(D * D + z * z));

    // Compute altitude
    // h = (k + e² - 1) / k · √(D² + z²)
    const Scalar alt = (k + e2 - 1.0) / k * janus::sqrt(D * D + z * z);

    // Compute longitude with pole handling
    // At poles (xy_dist ≈ 0), longitude is undefined; we set it to 0
    constexpr double eps = 1e-15;
    const Scalar is_pole = xy_dist < eps;
    const Scalar lon = janus::where(is_pole, Scalar(0.0), janus::atan2(y, x));

    return LLA<Scalar>(lon, lat, alt);
}

// =============================================================================
// LLA to ECEF - Closed-Form Conversion
// =============================================================================

/// Convert geodetic LLA to ECEF position (closed-form)
///
/// This is the standard geodetic to ECEF conversion formula.
///
/// @param lla Geodetic coordinates (lon, lat, alt)
/// @param m Earth model (default: WGS84)
/// @return Position in ECEF [m]
template <typename Scalar>
Vec3<Scalar> lla_to_ecef(const LLA<Scalar> &lla,
                         const EarthModel &m = EarthModel::WGS84()) {
    const Scalar sin_lat = janus::sin(lla.lat);
    const Scalar cos_lat = janus::cos(lla.lat);
    const Scalar sin_lon = janus::sin(lla.lon);
    const Scalar cos_lon = janus::cos(lla.lon);

    const double a = m.a;
    const double e2 = m.e2;

    // Radius of curvature in the prime vertical
    // N = a / √(1 - e² sin²φ)
    const Scalar N = a / janus::sqrt(1.0 - e2 * sin_lat * sin_lat);

    // ECEF coordinates
    // x = (N + h) cos(φ) cos(λ)
    // y = (N + h) cos(φ) sin(λ)
    // z = (N(1 - e²) + h) sin(φ)
    Vec3<Scalar> r;
    r(0) = (N + lla.alt) * cos_lat * cos_lon;
    r(1) = (N + lla.alt) * cos_lat * sin_lon;
    r(2) = (N * (1.0 - e2) + lla.alt) * sin_lat;

    return r;
}

// =============================================================================
// ECEF to Spherical (Geocentric) Coordinates
// =============================================================================

/// Convert ECEF position to geocentric spherical coordinates
///
/// Geocentric latitude differs from geodetic latitude for non-spherical
/// Earth models. This conversion is exact and non-iterative.
///
/// @param r Position in ECEF [m]
/// @return Spherical coordinates (lon, lat_gc, radius)
template <typename Scalar>
Spherical<Scalar> ecef_to_spherical(const Vec3<Scalar> &r) {
    const Scalar x = r(0);
    const Scalar y = r(1);
    const Scalar z = r(2);

    // Radius (distance from Earth center)
    const Scalar radius = janus::sqrt(x * x + y * y + z * z);

    // Geocentric latitude (angle from equatorial plane to position vector)
    const Scalar lat_gc = janus::asin(z / radius);

    // Longitude with pole handling
    const Scalar xy_dist = janus::sqrt(x * x + y * y);
    constexpr double eps = 1e-15;
    const Scalar is_pole = xy_dist < eps;
    const Scalar lon = janus::where(is_pole, Scalar(0.0), janus::atan2(y, x));

    return Spherical<Scalar>(lon, lat_gc, radius);
}

// =============================================================================
// Spherical (Geocentric) to ECEF Conversion
// =============================================================================

/// Convert geocentric spherical coordinates to ECEF position
///
/// This is the standard spherical to Cartesian conversion.
///
/// @param geo Geocentric spherical coordinates (lon, lat_gc, radius)
/// @return Position in ECEF [m]
template <typename Scalar>
Vec3<Scalar> spherical_to_ecef(const Spherical<Scalar> &geo) {
    const Scalar sin_lat = janus::sin(geo.lat_gc);
    const Scalar cos_lat = janus::cos(geo.lat_gc);
    const Scalar sin_lon = janus::sin(geo.lon);
    const Scalar cos_lon = janus::cos(geo.lon);

    Vec3<Scalar> r;
    r(0) = geo.radius * cos_lat * cos_lon;
    r(1) = geo.radius * cos_lat * sin_lon;
    r(2) = geo.radius * sin_lat;

    return r;
}

// =============================================================================
// Convenience Functions
// =============================================================================

/// Convert geodetic latitude to geocentric latitude
///
/// For an ellipsoidal Earth, the geocentric latitude is always smaller
/// in magnitude than the geodetic latitude (except at equator and poles).
///
/// @param lat_gd Geodetic latitude [rad]
/// @param m Earth model (default: WGS84)
/// @return Geocentric latitude [rad]
template <typename Scalar>
Scalar geodetic_to_geocentric_lat(Scalar lat_gd,
                                  const EarthModel &m = EarthModel::WGS84()) {
    // tan(φ_gc) = (1 - e²) tan(φ_gd)
    return janus::atan((1.0 - m.e2) * janus::tan(lat_gd));
}

/// Convert geocentric latitude to geodetic latitude
///
/// @param lat_gc Geocentric latitude [rad]
/// @param m Earth model (default: WGS84)
/// @return Geodetic latitude [rad]
template <typename Scalar>
Scalar geocentric_to_geodetic_lat(Scalar lat_gc,
                                  const EarthModel &m = EarthModel::WGS84()) {
    // tan(φ_gd) = tan(φ_gc) / (1 - e²)
    return janus::atan(janus::tan(lat_gc) / (1.0 - m.e2));
}

/// Compute the radius of curvature in the prime vertical (N)
///
/// This is the distance from the surface to the Z-axis along the
/// ellipsoid normal.
///
/// @param lat Geodetic latitude [rad]
/// @param m Earth model (default: WGS84)
/// @return Radius of curvature N [m]
template <typename Scalar>
Scalar radius_of_curvature_N(Scalar lat,
                             const EarthModel &m = EarthModel::WGS84()) {
    const Scalar sin_lat = janus::sin(lat);
    return m.a / janus::sqrt(1.0 - m.e2 * sin_lat * sin_lat);
}

/// Compute the radius of curvature in the meridian (M)
///
/// This is the radius of curvature of the meridian ellipse at the
/// given latitude.
///
/// @param lat Geodetic latitude [rad]
/// @param m Earth model (default: WGS84)
/// @return Radius of curvature M [m]
template <typename Scalar>
Scalar radius_of_curvature_M(Scalar lat,
                             const EarthModel &m = EarthModel::WGS84()) {
    const Scalar sin_lat = janus::sin(lat);
    const Scalar denom = 1.0 - m.e2 * sin_lat * sin_lat;
    return m.a * (1.0 - m.e2) / janus::pow(denom, 1.5);
}

} // namespace vulcan
