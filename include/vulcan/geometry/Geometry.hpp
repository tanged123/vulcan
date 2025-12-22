// Vulcan Geometry Primitives Module
// Spatial computation utilities for guidance, visibility, and sensor FOV
#pragma once

#include <vulcan/coordinates/EarthModel.hpp>
#include <vulcan/coordinates/Geodetic.hpp>
#include <vulcan/core/VulcanTypes.hpp>

#include <janus/math/Arithmetic.hpp>
#include <janus/math/Linalg.hpp>
#include <janus/math/Logic.hpp>
#include <janus/math/Trig.hpp>

namespace vulcan::geometry {

// =============================================================================
// Line-of-Sight Computations
// =============================================================================

/**
 * @brief Compute slant range (distance) between two points
 *
 * @tparam Scalar Scalar type (double or casadi::MX)
 * @param r_observer Observer position [m]
 * @param r_target Target position [m]
 * @return Distance between points [m]
 */
template <typename Scalar>
Scalar slant_range(const Vec3<Scalar> &r_observer,
                   const Vec3<Scalar> &r_target) {
    Vec3<Scalar> delta = r_target - r_observer;
    return janus::norm(delta);
}

/**
 * @brief Compute line-of-sight azimuth and elevation angles
 *
 * Computes the look angles from observer to target in the observer's
 * local coordinate frame (typically NED or ENU).
 *
 * Convention:
 * - Azimuth: angle from +X axis, positive clockwise when viewed from above
 * - Elevation: angle from XY plane, positive upward
 *
 * @tparam Scalar Scalar type (double or casadi::MX)
 * @param r_observer Observer position [m]
 * @param r_target Target position [m]
 * @return [azimuth, elevation] in radians
 */
template <typename Scalar>
Vec2<Scalar> los_angles(const Vec3<Scalar> &r_observer,
                        const Vec3<Scalar> &r_target) {
    Vec3<Scalar> delta = r_target - r_observer;

    Scalar dx = delta(0);
    Scalar dy = delta(1);
    Scalar dz = delta(2);

    // Horizontal distance
    Scalar horiz_dist = janus::sqrt(dx * dx + dy * dy);
    Scalar range = janus::norm(delta);

    // Avoid division by zero
    Scalar eps = Scalar(1e-12);
    Scalar is_zero = range < eps;

    // Azimuth: atan2(y, x)
    Scalar azimuth = janus::atan2(dy, dx);
    azimuth = janus::where(is_zero, Scalar(0), azimuth);

    // Elevation: atan2(z, horizontal_distance)
    // Negative z means target is above in NED frame
    Scalar elevation = janus::atan2(-dz, horiz_dist);
    elevation = janus::where(is_zero, Scalar(0), elevation);

    Vec2<Scalar> angles;
    angles << azimuth, elevation;
    return angles;
}

/**
 * @brief Compute line-of-sight angular rates
 *
 * Computes the time derivative of LOS angles given relative positions
 * and velocities of observer and target.
 *
 * @tparam Scalar Scalar type (double or casadi::MX)
 * @param r_obs Observer position [m]
 * @param v_obs Observer velocity [m/s]
 * @param r_tgt Target position [m]
 * @param v_tgt Target velocity [m/s]
 * @return [azimuth_rate, elevation_rate] in rad/s
 */
template <typename Scalar>
Vec2<Scalar> los_rate(const Vec3<Scalar> &r_obs, const Vec3<Scalar> &v_obs,
                      const Vec3<Scalar> &r_tgt, const Vec3<Scalar> &v_tgt) {
    // Relative position and velocity
    Vec3<Scalar> r_rel = r_tgt - r_obs;
    Vec3<Scalar> v_rel = v_tgt - v_obs;

    Scalar dx = r_rel(0);
    Scalar dy = r_rel(1);
    Scalar dz = r_rel(2);
    Scalar vx = v_rel(0);
    Scalar vy = v_rel(1);
    Scalar vz = v_rel(2);

    // Range and range rate
    Scalar range_sq = dx * dx + dy * dy + dz * dz;
    Scalar range = janus::sqrt(range_sq);
    Scalar horiz_sq = dx * dx + dy * dy;
    Scalar horiz = janus::sqrt(horiz_sq);

    // Avoid division by zero
    Scalar eps = Scalar(1e-12);
    Scalar is_range_zero = range < eps;
    Scalar is_horiz_zero = horiz < eps;

    // Azimuth rate: d/dt[atan2(y,x)] = (x*vy - y*vx) / (x^2 + y^2)
    Scalar az_rate = (dx * vy - dy * vx) / horiz_sq;
    az_rate = janus::where(is_horiz_zero, Scalar(0), az_rate);

    // Elevation rate: d/dt[atan2(-z, horiz)]
    // = d/dt[atan2(-z, sqrt(x^2+y^2))]
    // = (horiz * (-vz) - (-z) * d_horiz/dt) / (horiz^2 + z^2)
    // where d_horiz/dt = (x*vx + y*vy) / horiz
    Scalar d_horiz_dt = (dx * vx + dy * vy) / horiz;
    d_horiz_dt = janus::where(is_horiz_zero, Scalar(0), d_horiz_dt);

    Scalar el_rate = (-horiz * vz + dz * d_horiz_dt) / range_sq;
    el_rate = janus::where(is_range_zero, Scalar(0), el_rate);

    Vec2<Scalar> rates;
    rates << az_rate, el_rate;
    return rates;
}

// =============================================================================
// Ray Intersections
// =============================================================================

/**
 * @brief Ray-sphere intersection
 *
 * Computes the distance along a ray to the nearest intersection with a sphere.
 * Uses the geometric solution for numerical stability.
 *
 * @tparam Scalar Scalar type (double or casadi::MX)
 * @param origin Ray origin point
 * @param direction Ray direction (should be normalized)
 * @param center Sphere center
 * @param radius Sphere radius
 * @return Distance to nearest intersection, or negative if no intersection
 */
template <typename Scalar>
Scalar ray_sphere_intersection(const Vec3<Scalar> &origin,
                               const Vec3<Scalar> &direction,
                               const Vec3<Scalar> &center,
                               const Scalar &radius) {
    // Vector from ray origin to sphere center
    Vec3<Scalar> oc = center - origin;

    // Project oc onto ray direction
    Scalar tca = janus::dot(oc, direction);

    // Distance squared from sphere center to ray
    Scalar oc_sq = janus::dot(oc, oc);
    Scalar d_sq = oc_sq - tca * tca;

    // Check if ray misses sphere
    Scalar r_sq = radius * radius;
    Scalar misses = d_sq > r_sq;

    // Distance from closest approach to intersection points
    Scalar thc = janus::sqrt(janus::abs(r_sq - d_sq));

    // Two intersection points at t = tca +/- thc
    // We want the nearest positive intersection
    Scalar t0 = tca - thc;
    Scalar t1 = tca + thc;

    // If t0 is positive, use it; otherwise use t1 if positive
    Scalar t0_positive = t0 > Scalar(0);
    Scalar t1_positive = t1 > Scalar(0);
    Scalar t = janus::where(t0_positive, t0, t1);

    // Return -1 if no valid intersection
    Scalar no_intersection = misses + (!t0_positive * !t1_positive) > Scalar(0);
    return janus::where(no_intersection, Scalar(-1), t);
}

/**
 * @brief Ray-plane intersection
 *
 * Computes the distance along a ray to intersection with an infinite plane.
 *
 * @tparam Scalar Scalar type (double or casadi::MX)
 * @param origin Ray origin point
 * @param direction Ray direction (should be normalized)
 * @param plane_normal Plane normal vector (should be normalized)
 * @param plane_point A point on the plane
 * @return Distance to intersection, or negative if parallel or behind
 */
template <typename Scalar>
Scalar ray_plane_intersection(const Vec3<Scalar> &origin,
                              const Vec3<Scalar> &direction,
                              const Vec3<Scalar> &plane_normal,
                              const Vec3<Scalar> &plane_point) {
    // Compute denominator (dot of direction and normal)
    Scalar denom = janus::dot(direction, plane_normal);

    // Check if ray is parallel to plane
    Scalar eps = Scalar(1e-12);
    Scalar is_parallel = janus::abs(denom) < eps;

    // Distance from origin to plane along normal
    Vec3<Scalar> diff = plane_point - origin;
    Scalar t = janus::dot(diff, plane_normal) / denom;

    // Return -1 if parallel or intersection is behind ray origin
    Scalar invalid = is_parallel + (t < Scalar(0)) > Scalar(0);
    return janus::where(invalid, Scalar(-1), t);
}

/**
 * @brief Check if a point lies within a cone (e.g., sensor FOV)
 *
 * Determines if a point is inside a cone defined by its apex, axis direction,
 * and half-angle. Useful for sensor field-of-view checks.
 *
 * @tparam Scalar Scalar type (double or casadi::MX)
 * @param point Point to test
 * @param apex Cone apex (e.g., sensor location)
 * @param axis Cone axis direction (should be normalized)
 * @param half_angle Cone half-angle in radians
 * @return 1.0 if point is inside cone, 0.0 otherwise (Scalar for symbolic
 * compat)
 */
template <typename Scalar>
Scalar point_in_cone(const Vec3<Scalar> &point, const Vec3<Scalar> &apex,
                     const Vec3<Scalar> &axis, const Scalar &half_angle) {
    // Vector from apex to point
    Vec3<Scalar> to_point = point - apex;
    Scalar dist = janus::norm(to_point);

    // Avoid division by zero
    Scalar eps = Scalar(1e-12);
    Scalar is_at_apex = dist < eps;

    // Normalize direction to point
    Vec3<Scalar> to_point_norm = to_point / dist;

    // Angle between axis and direction to point
    Scalar cos_angle = janus::dot(axis, to_point_norm);
    Scalar cos_half_angle = janus::cos(half_angle);

    // Point is inside if angle is less than half_angle
    // (cos_angle > cos_half_angle since cos is decreasing)
    Scalar inside = cos_angle > cos_half_angle;

    // Point at apex is considered inside
    return janus::where(is_at_apex, Scalar(1), inside);
}

// =============================================================================
// Projections
// =============================================================================

/**
 * @brief Project a point onto a plane
 *
 * @tparam Scalar Scalar type (double or casadi::MX)
 * @param point Point to project
 * @param plane_normal Plane normal (should be normalized)
 * @param plane_point A point on the plane
 * @return Projected point on the plane
 */
template <typename Scalar>
Vec3<Scalar> project_to_plane(const Vec3<Scalar> &point,
                              const Vec3<Scalar> &plane_normal,
                              const Vec3<Scalar> &plane_point) {
    // Distance from point to plane along normal
    Vec3<Scalar> diff = point - plane_point;
    Scalar dist = janus::dot(diff, plane_normal);

    // Subtract the normal component
    return point - dist * plane_normal;
}

/**
 * @brief Project ECEF position to ellipsoid surface (ground track point)
 *
 * Computes the point on the Earth's surface directly below (or above)
 * the given ECEF position. Returns LLA with altitude set to zero.
 *
 * @tparam Scalar Scalar type (double or casadi::MX)
 * @param r_ecef Position in ECEF [m]
 * @param model Earth model (default: WGS84)
 * @return LLA coordinates of ground track point (altitude = 0)
 */
template <typename Scalar>
LLA<Scalar> ground_track_point(const Vec3<Scalar> &r_ecef,
                               const EarthModel &model = EarthModel::WGS84()) {
    // Convert to LLA and set altitude to zero
    LLA<Scalar> lla = ecef_to_lla<Scalar>(r_ecef, model);
    return LLA<Scalar>(lla.lon, lla.lat, Scalar(0));
}

/**
 * @brief Compute ground track point and return as ECEF
 *
 * Projects the position to the ellipsoid surface and returns the
 * ECEF coordinates of that point.
 *
 * @tparam Scalar Scalar type (double or casadi::MX)
 * @param r_ecef Position in ECEF [m]
 * @param model Earth model (default: WGS84)
 * @return ECEF position of ground track point [m]
 */
template <typename Scalar>
Vec3<Scalar> ground_track_ecef(const Vec3<Scalar> &r_ecef,
                               const EarthModel &model = EarthModel::WGS84()) {
    LLA<Scalar> ground = ground_track_point<Scalar>(r_ecef, model);
    return lla_to_ecef<Scalar>(ground, model);
}

} // namespace vulcan::geometry
