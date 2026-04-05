// Vulcan Geodetic Utilities
// ECEF <-> Geodetic conversions using Vermeille (2004) closed-form algorithm
#pragma once

#include <vulcan/coordinates/EarthModel.hpp>
#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/Units.hpp>

#include <janus/math/Arithmetic.hpp>
#include <janus/math/Logic.hpp>
#include <janus/math/Trig.hpp>

namespace vulcan {

using namespace vulcan::units;

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
    Quantity<rad, Scalar>
        lon; ///< Longitude [rad], positive East, range [-π, π]
    Quantity<rad, Scalar> lat; ///< Geodetic latitude [rad], range [-π/2, π/2]
    Quantity<m, Scalar> alt;   ///< Altitude above ellipsoid [m]

    LLA()
        : lon(Quantity<rad, Scalar>(Scalar(0))),
          lat(Quantity<rad, Scalar>(Scalar(0))),
          alt(Quantity<m, Scalar>(Scalar(0))) {}
    LLA(Quantity<rad, Scalar> lon_, Quantity<rad, Scalar> lat_,
        Quantity<m, Scalar> alt_)
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
    Quantity<rad, Scalar>
        lon; ///< Longitude [rad], positive East, range [-π, π]
    Quantity<rad, Scalar>
        lat_gc; ///< Geocentric latitude [rad], range [-π/2, π/2]
    Quantity<m, Scalar> radius; ///< Distance from Earth center [m]

    Spherical()
        : lon(Quantity<rad, Scalar>(Scalar(0))),
          lat_gc(Quantity<rad, Scalar>(Scalar(0))),
          radius(Quantity<m, Scalar>(Scalar(0))) {}
    Spherical(Quantity<rad, Scalar> lon_, Quantity<rad, Scalar> lat_gc_,
              Quantity<m, Scalar> radius_)
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
LLA<Scalar> ecef_to_lla(const Vec3<Quantity<m, Scalar>> &r,
                        const EarthModel &em = EarthModel::WGS84()) {
    // Unwrap Quantity inputs to raw Scalar
    const Scalar x = r(0).value();
    const Scalar y = r(1).value();
    const Scalar z = r(2).value();

    const double a = em.a;
    const double e2 = em.e2;
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
    const Scalar lat_raw =
        2.0 * janus::atan2(z, D + janus::sqrt(D * D + z * z));

    // Compute altitude
    // h = (k + e² - 1) / k · √(D² + z²)
    const Scalar alt_raw = (k + e2 - 1.0) / k * janus::sqrt(D * D + z * z);

    // Compute longitude with pole handling
    // At poles (xy_dist ≈ 0), longitude is undefined; we set it to 0
    constexpr double eps = 1e-15;
    const Scalar is_pole = xy_dist < eps;
    const Scalar lon_raw =
        janus::where(is_pole, Scalar(0.0), janus::atan2(y, x));

    return LLA<Scalar>(Quantity<rad, Scalar>(lon_raw),
                       Quantity<rad, Scalar>(lat_raw),
                       Quantity<m, Scalar>(alt_raw));
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
Vec3<Quantity<m, Scalar>>
lla_to_ecef(const LLA<Scalar> &lla,
            const EarthModel &em = EarthModel::WGS84()) {
    // Unwrap Quantity fields to raw Scalar
    const Scalar lat_raw = lla.lat.value();
    const Scalar lon_raw = lla.lon.value();
    const Scalar alt_raw = lla.alt.value();

    const Scalar sin_lat = janus::sin(lat_raw);
    const Scalar cos_lat = janus::cos(lat_raw);
    const Scalar sin_lon = janus::sin(lon_raw);
    const Scalar cos_lon = janus::cos(lon_raw);

    const double a = em.a;
    const double e2 = em.e2;

    // Radius of curvature in the prime vertical
    // N = a / √(1 - e² sin²φ)
    const Scalar N = a / janus::sqrt(1.0 - e2 * sin_lat * sin_lat);

    // ECEF coordinates
    // x = (N + h) cos(φ) cos(λ)
    // y = (N + h) cos(φ) sin(λ)
    // z = (N(1 - e²) + h) sin(φ)
    Vec3<Quantity<m, Scalar>> r;
    r(0) = Quantity<m, Scalar>((N + alt_raw) * cos_lat * cos_lon);
    r(1) = Quantity<m, Scalar>((N + alt_raw) * cos_lat * sin_lon);
    r(2) = Quantity<m, Scalar>((N * (1.0 - e2) + alt_raw) * sin_lat);

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
    const Scalar radius_raw = janus::sqrt(x * x + y * y + z * z);

    // Geocentric latitude (angle from equatorial plane to position vector)
    const Scalar lat_gc_raw = janus::asin(z / radius_raw);

    // Longitude with pole handling
    const Scalar xy_dist = janus::sqrt(x * x + y * y);
    constexpr double eps = 1e-15;
    const Scalar is_pole = xy_dist < eps;
    const Scalar lon_raw =
        janus::where(is_pole, Scalar(0.0), janus::atan2(y, x));

    return Spherical<Scalar>(Quantity<rad, Scalar>(lon_raw),
                             Quantity<rad, Scalar>(lat_gc_raw),
                             Quantity<m, Scalar>(radius_raw));
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
    // Unwrap Quantity fields to raw Scalar
    const Scalar lat_gc_raw = geo.lat_gc.value();
    const Scalar lon_raw = geo.lon.value();
    const Scalar radius_raw = geo.radius.value();

    const Scalar sin_lat = janus::sin(lat_gc_raw);
    const Scalar cos_lat = janus::cos(lat_gc_raw);
    const Scalar sin_lon = janus::sin(lon_raw);
    const Scalar cos_lon = janus::cos(lon_raw);

    Vec3<Scalar> r;
    r(0) = radius_raw * cos_lat * cos_lon;
    r(1) = radius_raw * cos_lat * sin_lon;
    r(2) = radius_raw * sin_lat;

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
Quantity<rad, Scalar>
geodetic_to_geocentric_lat(Quantity<rad, Scalar> lat_gd,
                           const EarthModel &em = EarthModel::WGS84()) {
    // Unwrap, compute, wrap
    const Scalar raw = lat_gd.value();
    return Quantity<rad, Scalar>(janus::atan((1.0 - em.e2) * janus::tan(raw)));
}

/// Convert geocentric latitude to geodetic latitude
///
/// @param lat_gc Geocentric latitude [rad]
/// @param m Earth model (default: WGS84)
/// @return Geodetic latitude [rad]
template <typename Scalar>
Quantity<rad, Scalar>
geocentric_to_geodetic_lat(Quantity<rad, Scalar> lat_gc,
                           const EarthModel &em = EarthModel::WGS84()) {
    const Scalar raw = lat_gc.value();
    return Quantity<rad, Scalar>(janus::atan(janus::tan(raw) / (1.0 - em.e2)));
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
Quantity<m, Scalar>
radius_of_curvature_N(Quantity<rad, Scalar> lat,
                      const EarthModel &em = EarthModel::WGS84()) {
    const Scalar sin_lat = janus::sin(lat.value());
    return Quantity<m, Scalar>(em.a /
                               janus::sqrt(1.0 - em.e2 * sin_lat * sin_lat));
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
Quantity<m, Scalar>
radius_of_curvature_M(Quantity<rad, Scalar> lat,
                      const EarthModel &em = EarthModel::WGS84()) {
    const Scalar sin_lat = janus::sin(lat.value());
    const Scalar denom = 1.0 - em.e2 * sin_lat * sin_lat;
    return Quantity<m, Scalar>(em.a * (1.0 - em.e2) / janus::pow(denom, 1.5));
}

} // namespace vulcan
