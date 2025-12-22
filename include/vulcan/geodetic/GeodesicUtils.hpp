// Vulcan Geodesic Utilities
// Pure geometric computations on the WGS84 ellipsoid
#pragma once

#include <vulcan/coordinates/EarthModel.hpp>
#include <vulcan/coordinates/Geodetic.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>

#include <janus/math/Arithmetic.hpp>
#include <janus/math/Logic.hpp>
#include <janus/math/Trig.hpp>

namespace vulcan::geodetic {

// =============================================================================
// Distance Calculations
// =============================================================================

/// Haversine distance (fast spherical approximation)
///
/// Uses the haversine formula for great-circle distance on a sphere.
/// Good for short distances (<100km), error <0.3% compared to Vincenty.
///
/// Formula:
///   a = sin²(Δφ/2) + cos(φ₁)cos(φ₂)sin²(Δλ/2)
///   c = 2·atan2(√a, √(1−a))
///   d = R·c
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param lla1 First point (altitude ignored)
/// @param lla2 Second point (altitude ignored)
/// @param radius Sphere radius [m] (default: WGS84 mean radius)
/// @return Distance along sphere surface [m]
template <typename Scalar>
Scalar haversine_distance(const LLA<Scalar> &lla1, const LLA<Scalar> &lla2,
                          double radius = constants::earth::R_mean) {
    const Scalar dlat = lla2.lat - lla1.lat;
    const Scalar dlon = lla2.lon - lla1.lon;

    const Scalar sin_dlat_2 = janus::sin(dlat / 2.0);
    const Scalar sin_dlon_2 = janus::sin(dlon / 2.0);

    const Scalar a = sin_dlat_2 * sin_dlat_2 + janus::cos(lla1.lat) *
                                                   janus::cos(lla2.lat) *
                                                   sin_dlon_2 * sin_dlon_2;

    const Scalar c = 2.0 * janus::atan2(janus::sqrt(a), janus::sqrt(1.0 - a));

    return radius * c;
}

/// Great-circle distance using Vincenty formula (accurate to 0.5mm)
///
/// Uses Vincenty's inverse formula for geodesic distance on an ellipsoid.
/// This is an iterative algorithm that converges for most point pairs.
///
/// Reference: Vincenty, T. (1975). "Direct and inverse solutions of geodesics
/// on the ellipsoid with application of nested equations"
///
/// Note: For symbolic mode, uses a fixed iteration count (20 iterations)
/// rather than convergence checking.
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param lla1 First point (altitude ignored)
/// @param lla2 Second point (altitude ignored)
/// @param m Earth model (default: WGS84)
/// @return Distance along ellipsoid surface [m]
template <typename Scalar>
Scalar great_circle_distance(const LLA<Scalar> &lla1, const LLA<Scalar> &lla2,
                             const EarthModel &m = EarthModel::WGS84()) {
    const double a = m.a;
    const double b = m.b;
    const double f = m.f;

    // Reduced latitudes
    const Scalar U1 = janus::atan((1.0 - f) * janus::tan(lla1.lat));
    const Scalar U2 = janus::atan((1.0 - f) * janus::tan(lla2.lat));

    const Scalar sin_U1 = janus::sin(U1);
    const Scalar cos_U1 = janus::cos(U1);
    const Scalar sin_U2 = janus::sin(U2);
    const Scalar cos_U2 = janus::cos(U2);

    const Scalar L = lla2.lon - lla1.lon;

    // Iterative solution
    Scalar lambda = L;
    Scalar sin_sigma, cos_sigma, sigma;
    Scalar sin_alpha, cos2_alpha, cos_2sigma_m;

    // Fixed iteration count for symbolic compatibility
    constexpr int max_iterations = 20;

    for (int i = 0; i < max_iterations; ++i) {
        const Scalar sin_lambda = janus::sin(lambda);
        const Scalar cos_lambda = janus::cos(lambda);

        const Scalar term1 = cos_U2 * sin_lambda;
        const Scalar term2 = cos_U1 * sin_U2 - sin_U1 * cos_U2 * cos_lambda;
        sin_sigma = janus::sqrt(term1 * term1 + term2 * term2);

        cos_sigma = sin_U1 * sin_U2 + cos_U1 * cos_U2 * cos_lambda;
        sigma = janus::atan2(sin_sigma, cos_sigma);

        // sin(α) = cos(U₁)cos(U₂)sin(λ) / sin(σ)
        // Handle sin_sigma ≈ 0 (coincident points)
        sin_alpha = cos_U1 * cos_U2 * sin_lambda / (sin_sigma + 1e-15);
        cos2_alpha = 1.0 - sin_alpha * sin_alpha;

        // cos(2σₘ) = cos(σ) - 2sin(U₁)sin(U₂) / cos²(α)
        // Handle cos2_alpha ≈ 0 (equatorial line)
        cos_2sigma_m = cos_sigma - 2.0 * sin_U1 * sin_U2 / (cos2_alpha + 1e-15);

        const Scalar C =
            f / 16.0 * cos2_alpha * (4.0 + f * (4.0 - 3.0 * cos2_alpha));

        const Scalar lambda_prev = lambda;
        lambda = L + (1.0 - C) * f * sin_alpha *
                         (sigma +
                          C * sin_sigma *
                              (cos_2sigma_m +
                               C * cos_sigma *
                                   (-1.0 + 2.0 * cos_2sigma_m * cos_2sigma_m)));

        // Note: In symbolic mode, we don't break early
        // For numeric mode, could add convergence check here
        (void)lambda_prev;
    }

    // Calculate distance
    const Scalar u2 = cos2_alpha * (a * a - b * b) / (b * b);
    const Scalar A =
        1.0 +
        u2 / 16384.0 * (4096.0 + u2 * (-768.0 + u2 * (320.0 - 175.0 * u2)));
    const Scalar B =
        u2 / 1024.0 * (256.0 + u2 * (-128.0 + u2 * (74.0 - 47.0 * u2)));

    const Scalar cos2_2sigma_m = cos_2sigma_m * cos_2sigma_m;
    const Scalar delta_sigma =
        B * sin_sigma *
        (cos_2sigma_m +
         B / 4.0 *
             (cos_sigma * (-1.0 + 2.0 * cos2_2sigma_m) -
              B / 6.0 * cos_2sigma_m * (-3.0 + 4.0 * sin_sigma * sin_sigma) *
                  (-3.0 + 4.0 * cos2_2sigma_m)));

    return b * A * (sigma - delta_sigma);
}

// =============================================================================
// Bearing Calculations
// =============================================================================

/// Initial bearing (azimuth) from point 1 to point 2
///
/// Computes the initial bearing (forward azimuth) for the geodesic from
/// point 1 to point 2. This is the direction to head from point 1.
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param lla1 First point (start)
/// @param lla2 Second point (destination)
/// @param m Earth model (default: WGS84) -- used only for flattening
/// @return Azimuth in radians [0, 2π), measured clockwise from North
template <typename Scalar>
Scalar
initial_bearing(const LLA<Scalar> &lla1, const LLA<Scalar> &lla2,
                [[maybe_unused]] const EarthModel &m = EarthModel::WGS84()) {
    // For spherical approximation (sufficient for bearing):
    // θ = atan2(sin(Δλ)cos(φ₂), cos(φ₁)sin(φ₂) - sin(φ₁)cos(φ₂)cos(Δλ))

    const Scalar dlon = lla2.lon - lla1.lon;

    const Scalar sin_dlon = janus::sin(dlon);
    const Scalar cos_dlon = janus::cos(dlon);
    const Scalar sin_lat1 = janus::sin(lla1.lat);
    const Scalar cos_lat1 = janus::cos(lla1.lat);
    const Scalar sin_lat2 = janus::sin(lla2.lat);
    const Scalar cos_lat2 = janus::cos(lla2.lat);

    const Scalar x = sin_dlon * cos_lat2;
    const Scalar y = cos_lat1 * sin_lat2 - sin_lat1 * cos_lat2 * cos_dlon;

    Scalar bearing = janus::atan2(x, y);

    // Normalize to [0, 2π) using: bearing = bearing - floor(bearing / 2π) * 2π
    constexpr double two_pi = 2.0 * constants::angle::pi;
    // Add 2π first to handle negative bearings, then use atan2's periodicity
    // Since atan2 returns [-π, π], adding 2π and taking modulo gives [0, 2π)
    bearing = bearing + Scalar(two_pi);
    // For symbolic compatibility, use: fmod(x, y) ≈ x - floor(x/y) * y
    // But simpler: just use conditional approach with where
    bearing = janus::where(bearing >= Scalar(two_pi), bearing - Scalar(two_pi),
                           bearing);
    bearing =
        janus::where(bearing < Scalar(0.0), bearing + Scalar(two_pi), bearing);

    return bearing;
}

/// Final bearing arriving at point 2 from point 1
///
/// Computes the final bearing (back azimuth + π) arriving at point 2.
/// This is the reverse of initial_bearing(lla2, lla1) + π.
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param lla1 First point (start)
/// @param lla2 Second point (destination)
/// @param m Earth model (default: WGS84)
/// @return Azimuth in radians [0, 2π), measured clockwise from North
template <typename Scalar>
Scalar final_bearing(const LLA<Scalar> &lla1, const LLA<Scalar> &lla2,
                     const EarthModel &m = EarthModel::WGS84()) {
    // Final bearing is the reverse of initial bearing from lla2 to lla1, + 180°
    Scalar bearing =
        initial_bearing(lla2, lla1, m) + Scalar(constants::angle::pi);

    // Normalize to [0, 2π)
    constexpr double two_pi = 2.0 * constants::angle::pi;
    bearing = janus::where(bearing >= Scalar(two_pi), bearing - Scalar(two_pi),
                           bearing);

    return bearing;
}

// =============================================================================
// Direct Geodesic Problem
// =============================================================================

/// Compute destination point given start, bearing, and distance
///
/// Uses the Vincenty direct formula to compute the endpoint of a geodesic
/// given start point, initial bearing, and distance along the ellipsoid.
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param lla Starting point
/// @param bearing Initial azimuth [rad], clockwise from North
/// @param distance Distance along ellipsoid [m]
/// @param m Earth model (default: WGS84)
/// @return Destination LLA (altitude set to source altitude)
template <typename Scalar>
LLA<Scalar> destination_point(const LLA<Scalar> &lla, const Scalar &bearing,
                              const Scalar &distance,
                              const EarthModel &m = EarthModel::WGS84()) {
    const double a = m.a;
    const double b = m.b;
    const double f = m.f;

    const Scalar sin_bearing = janus::sin(bearing);
    const Scalar cos_bearing = janus::cos(bearing);

    // Reduced latitude
    const Scalar U1 = janus::atan((1.0 - f) * janus::tan(lla.lat));
    const Scalar sin_U1 = janus::sin(U1);
    const Scalar cos_U1 = janus::cos(U1);

    // σ₁ = atan2(tan(U₁), cos(α₁))
    const Scalar sigma1 = janus::atan2(janus::tan(U1), cos_bearing);

    // sin(α) = cos(U₁)sin(α₁)
    const Scalar sin_alpha = cos_U1 * sin_bearing;
    const Scalar cos2_alpha = 1.0 - sin_alpha * sin_alpha;

    const Scalar u2 = cos2_alpha * (a * a - b * b) / (b * b);
    const Scalar A =
        1.0 +
        u2 / 16384.0 * (4096.0 + u2 * (-768.0 + u2 * (320.0 - 175.0 * u2)));
    const Scalar B =
        u2 / 1024.0 * (256.0 + u2 * (-128.0 + u2 * (74.0 - 47.0 * u2)));

    // Initial guess for σ
    Scalar sigma = distance / (b * A);

    // Iterate to find σ
    Scalar cos_2sigma_m, sin_sigma, cos_sigma;

    constexpr int max_iterations = 20;
    for (int i = 0; i < max_iterations; ++i) {
        cos_2sigma_m = janus::cos(2.0 * sigma1 + sigma);
        sin_sigma = janus::sin(sigma);
        cos_sigma = janus::cos(sigma);

        const Scalar cos2_2sigma_m = cos_2sigma_m * cos_2sigma_m;
        const Scalar delta_sigma =
            B * sin_sigma *
            (cos_2sigma_m + B / 4.0 *
                                (cos_sigma * (-1.0 + 2.0 * cos2_2sigma_m) -
                                 B / 6.0 * cos_2sigma_m *
                                     (-3.0 + 4.0 * sin_sigma * sin_sigma) *
                                     (-3.0 + 4.0 * cos2_2sigma_m)));

        const Scalar sigma_prev = sigma;
        sigma = distance / (b * A) + delta_sigma;
        (void)sigma_prev;
    }

    // Compute final values
    sin_sigma = janus::sin(sigma);
    cos_sigma = janus::cos(sigma);
    cos_2sigma_m = janus::cos(2.0 * sigma1 + sigma);

    // Latitude - use x*x instead of pow(x, 2) for type compatibility
    const Scalar temp = sin_U1 * sin_sigma - cos_U1 * cos_sigma * cos_bearing;
    const Scalar lat2 = janus::atan2(
        sin_U1 * cos_sigma + cos_U1 * sin_sigma * cos_bearing,
        (1.0 - f) * janus::sqrt(sin_alpha * sin_alpha + temp * temp));

    // Longitude
    const Scalar lambda =
        janus::atan2(sin_sigma * sin_bearing,
                     cos_U1 * cos_sigma - sin_U1 * sin_sigma * cos_bearing);

    const Scalar C =
        f / 16.0 * cos2_alpha * (4.0 + f * (4.0 - 3.0 * cos2_alpha));
    const Scalar L =
        lambda -
        (1.0 - C) * f * sin_alpha *
            (sigma +
             C * sin_sigma *
                 (cos_2sigma_m +
                  C * cos_sigma * (-1.0 + 2.0 * cos_2sigma_m * cos_2sigma_m)));

    const Scalar lon2 = lla.lon + L;

    return LLA<Scalar>(lon2, lat2, lla.alt);
}

// =============================================================================
// Horizon & Visibility
// =============================================================================

/// Distance to geometric horizon from given altitude
///
/// Computes the distance along the Earth's surface from a point at
/// altitude h to the geometric horizon (line-of-sight tangent to surface).
///
/// For a sphere: d = √(2Rh + h²)
/// For ellipsoid: uses mean radius at given position.
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param altitude Height above ellipsoid [m]
/// @param m Earth model (default: WGS84)
/// @return Distance to horizon along surface [m]
template <typename Scalar>
Scalar horizon_distance(const Scalar &altitude,
                        const EarthModel &m = EarthModel::WGS84()) {
    // Use mean radius for simplicity
    const double R = (2.0 * m.a + m.b) / 3.0;

    // d = √(2Rh + h²) = √(h(2R + h))
    return janus::sqrt(altitude * (2.0 * R + altitude));
}

/// Check if target is visible from observer (no terrain, pure geometry)
///
/// Uses geometric horizon distance and great-circle arc comparison.
/// Returns a value > 0 if visible, ≤ 0 if not visible.
///
/// The visibility check accounts for both observer and target altitudes:
/// visible if horizon_distance(alt1) + horizon_distance(alt2) ≥ ground_distance
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param lla_observer Observer position
/// @param lla_target Target position
/// @param m Earth model (default: WGS84)
/// @return Positive if visible, zero/negative if occluded by Earth
template <typename Scalar>
Scalar is_visible(const LLA<Scalar> &lla_observer,
                  const LLA<Scalar> &lla_target,
                  const EarthModel &m = EarthModel::WGS84()) {
    // Horizon distance from each point
    const Scalar h1 = horizon_distance(lla_observer.alt, m);
    const Scalar h2 = horizon_distance(lla_target.alt, m);

    // Ground distance between points
    const Scalar ground_dist =
        haversine_distance(lla_observer, lla_target, (2.0 * m.a + m.b) / 3.0);

    // Visible if combined horizon distances exceed ground distance
    return (h1 + h2) - ground_dist;
}

// =============================================================================
// Ray-Ellipsoid Intersection
// =============================================================================

/// Result of ray-ellipsoid intersection
template <typename Scalar> struct RayIntersection {
    Scalar hit;              ///< > 0 if ray intersects ellipsoid, ≤ 0 otherwise
    Scalar t_near;           ///< Parameter for near intersection point
    Scalar t_far;            ///< Parameter for far intersection point
    Vec3<Scalar> point_near; ///< Near intersection point (ECEF)
    Vec3<Scalar> point_far;  ///< Far intersection point (ECEF)
};

/// Ray-ellipsoid intersection for terrain/visibility calculations
///
/// Computes intersection of ray with WGS84 ellipsoid.
/// Ray: P(t) = origin + t * direction
///
/// Ellipsoid equation: (x/a)² + (y/a)² + (z/b)² = 1
/// Substituting ray equation gives quadratic: At² + Bt + C = 0
///
/// @tparam Scalar Scalar type (double for numeric, SymbolicScalar for symbolic)
/// @param origin Ray origin in ECEF [m]
/// @param direction Ray direction (will be normalized internally)
/// @param m Earth model (default: WGS84)
/// @return RayIntersection with intersection parameters and points
template <typename Scalar>
RayIntersection<Scalar>
ray_ellipsoid_intersection(const Vec3<Scalar> &origin,
                           const Vec3<Scalar> &direction,
                           const EarthModel &m = EarthModel::WGS84()) {

    const double a = m.a;
    const double b = m.b;
    const double a2 = a * a;
    const double b2 = b * b;

    // Normalize direction
    const Scalar dir_norm =
        janus::sqrt(direction(0) * direction(0) + direction(1) * direction(1) +
                    direction(2) * direction(2));
    const Vec3<Scalar> d = direction / dir_norm;

    // Ray origin components
    const Scalar ox = origin(0);
    const Scalar oy = origin(1);
    const Scalar oz = origin(2);

    // Ray direction components
    const Scalar dx = d(0);
    const Scalar dy = d(1);
    const Scalar dz = d(2);

    // Quadratic coefficients for ellipsoid intersection
    // (x/a)² + (y/a)² + (z/b)² = 1
    // Substitute x = ox + t*dx, etc.
    const Scalar A = dx * dx / a2 + dy * dy / a2 + dz * dz / b2;
    const Scalar B = 2.0 * (ox * dx / a2 + oy * dy / a2 + oz * dz / b2);
    const Scalar C = ox * ox / a2 + oy * oy / a2 + oz * oz / b2 - 1.0;

    // Discriminant
    const Scalar discriminant = B * B - 4.0 * A * C;

    // Compute intersection parameters
    const Scalar sqrt_disc = janus::sqrt(janus::abs(discriminant) + 1e-15);
    const Scalar t_near = (-B - sqrt_disc) / (2.0 * A);
    const Scalar t_far = (-B + sqrt_disc) / (2.0 * A);

    // Compute intersection points
    Vec3<Scalar> point_near;
    point_near(0) = origin(0) + t_near * d(0);
    point_near(1) = origin(1) + t_near * d(1);
    point_near(2) = origin(2) + t_near * d(2);

    Vec3<Scalar> point_far;
    point_far(0) = origin(0) + t_far * d(0);
    point_far(1) = origin(1) + t_far * d(1);
    point_far(2) = origin(2) + t_far * d(2);

    RayIntersection<Scalar> result;
    result.hit = discriminant; // > 0 means hit
    result.t_near = t_near;
    result.t_far = t_far;
    result.point_near = point_near;
    result.point_far = point_far;

    return result;
}

} // namespace vulcan::geodetic
