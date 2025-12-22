// Vulcan Geometry Primitives Tests
#include <gtest/gtest.h>
#include <vulcan/geometry/Geometry.hpp>

#include <janus/janus.hpp>

#include <cmath>

using namespace vulcan;
using namespace vulcan::geometry;

// =============================================================================
// Slant Range Tests
// =============================================================================

TEST(SlantRange, BasicDistance) {
    Vec3<double> p1(0, 0, 0);
    Vec3<double> p2(3, 4, 0);

    double range = slant_range(p1, p2);
    EXPECT_NEAR(range, 5.0, 1e-12); // 3-4-5 triangle
}

TEST(SlantRange, ThreeDimensional) {
    Vec3<double> p1(1, 2, 3);
    Vec3<double> p2(4, 6, 15);

    double range = slant_range(p1, p2);
    // Expected: sqrt(3^2 + 4^2 + 12^2) = sqrt(9 + 16 + 144) = sqrt(169) = 13
    EXPECT_NEAR(range, 13.0, 1e-12);
}

TEST(SlantRange, ZeroDistance) {
    Vec3<double> p(1, 2, 3);
    double range = slant_range(p, p);
    EXPECT_NEAR(range, 0.0, 1e-12);
}

// =============================================================================
// LOS Angles Tests
// =============================================================================

TEST(LOSAngles, AlongXAxis) {
    Vec3<double> obs(0, 0, 0);
    Vec3<double> tgt(10, 0, 0);

    Vec2<double> angles = los_angles(obs, tgt);
    EXPECT_NEAR(angles(0), 0.0, 1e-12); // Azimuth = 0 (along X)
    EXPECT_NEAR(angles(1), 0.0, 1e-12); // Elevation = 0 (horizontal)
}

TEST(LOSAngles, AlongYAxis) {
    Vec3<double> obs(0, 0, 0);
    Vec3<double> tgt(0, 10, 0);

    Vec2<double> angles = los_angles(obs, tgt);
    EXPECT_NEAR(angles(0), M_PI / 2, 1e-12); // Azimuth = 90 deg
    EXPECT_NEAR(angles(1), 0.0, 1e-12);      // Elevation = 0
}

TEST(LOSAngles, NegativeY) {
    Vec3<double> obs(0, 0, 0);
    Vec3<double> tgt(0, -10, 0);

    Vec2<double> angles = los_angles(obs, tgt);
    EXPECT_NEAR(angles(0), -M_PI / 2, 1e-12); // Azimuth = -90 deg
    EXPECT_NEAR(angles(1), 0.0, 1e-12);
}

TEST(LOSAngles, Elevation45Up) {
    Vec3<double> obs(0, 0, 0);
    Vec3<double> tgt(10, 0, -10); // NED: negative Z is up

    Vec2<double> angles = los_angles(obs, tgt);
    EXPECT_NEAR(angles(0), 0.0, 1e-12);
    EXPECT_NEAR(angles(1), M_PI / 4, 1e-10); // 45 deg elevation
}

TEST(LOSAngles, Elevation45Down) {
    Vec3<double> obs(0, 0, 0);
    Vec3<double> tgt(10, 0, 10); // NED: positive Z is down

    Vec2<double> angles = los_angles(obs, tgt);
    EXPECT_NEAR(angles(0), 0.0, 1e-12);
    EXPECT_NEAR(angles(1), -M_PI / 4, 1e-10); // -45 deg elevation
}

TEST(LOSAngles, DiagonalCase) {
    Vec3<double> obs(0, 0, 0);
    Vec3<double> tgt(10, 10, 0);

    Vec2<double> angles = los_angles(obs, tgt);
    EXPECT_NEAR(angles(0), M_PI / 4, 1e-12); // 45 deg azimuth
    EXPECT_NEAR(angles(1), 0.0, 1e-12);      // Horizontal
}

TEST(LOSAngles, ZeroRange) {
    Vec3<double> p(1, 2, 3);
    Vec2<double> angles = los_angles(p, p);
    EXPECT_NEAR(angles(0), 0.0, 1e-12);
    EXPECT_NEAR(angles(1), 0.0, 1e-12);
}

// =============================================================================
// LOS Rate Tests
// =============================================================================

TEST(LOSRate, StationaryObserverMovingTarget) {
    Vec3<double> r_obs(0, 0, 0);
    Vec3<double> v_obs(0, 0, 0);
    Vec3<double> r_tgt(10, 0, 0);
    Vec3<double> v_tgt(0, 1, 0); // Moving in Y direction

    Vec2<double> rates = los_rate(r_obs, v_obs, r_tgt, v_tgt);
    // Target at 10m range moving 1 m/s laterally -> 0.1 rad/s azimuth rate
    EXPECT_NEAR(rates(0), 0.1, 1e-10);
    EXPECT_NEAR(rates(1), 0.0, 1e-10);
}

TEST(LOSRate, VerticalMotion) {
    Vec3<double> r_obs(0, 0, 0);
    Vec3<double> v_obs(0, 0, 0);
    Vec3<double> r_tgt(10, 0, 0);
    Vec3<double> v_tgt(0, 0, -1); // Moving up (NED)

    Vec2<double> rates = los_rate(r_obs, v_obs, r_tgt, v_tgt);
    // Elevation rate should be positive (target going up)
    EXPECT_NEAR(rates(0), 0.0, 1e-10);
    EXPECT_NEAR(rates(1), 0.1, 1e-10); // 1 m/s at 10m -> 0.1 rad/s
}

// =============================================================================
// Ray-Sphere Intersection Tests
// =============================================================================

TEST(RaySphereIntersection, DirectHit) {
    Vec3<double> origin(0, 0, 0);
    Vec3<double> direction(1, 0, 0);
    Vec3<double> center(10, 0, 0);
    double radius = 2.0;

    double t = ray_sphere_intersection(origin, direction, center, radius);
    EXPECT_NEAR(t, 8.0, 1e-10); // Hit at t = 10 - 2 = 8
}

TEST(RaySphereIntersection, Miss) {
    Vec3<double> origin(0, 0, 0);
    Vec3<double> direction(1, 0, 0);
    Vec3<double> center(10, 10, 0); // Offset in Y
    double radius = 2.0;

    double t = ray_sphere_intersection(origin, direction, center, radius);
    EXPECT_LT(t, 0.0); // No intersection
}

TEST(RaySphereIntersection, InsideSphere) {
    Vec3<double> origin(0, 0, 0);
    Vec3<double> direction(1, 0, 0);
    Vec3<double> center(0, 0, 0); // Origin inside sphere
    double radius = 10.0;

    double t = ray_sphere_intersection(origin, direction, center, radius);
    EXPECT_NEAR(t, 10.0, 1e-10); // Exit at radius
}

TEST(RaySphereIntersection, GrazingHit) {
    Vec3<double> origin(0, 0, 0);
    Vec3<double> direction(1, 0, 0);
    Vec3<double> center(10, 2, 0); // Offset by exactly radius
    double radius = 2.0;

    double t = ray_sphere_intersection(origin, direction, center, radius);
    // Tangent hit at t = 10
    EXPECT_NEAR(t, 10.0, 1e-10);
}

// =============================================================================
// Ray-Plane Intersection Tests
// =============================================================================

TEST(RayPlaneIntersection, PerpendicularHit) {
    Vec3<double> origin(0, 0, 0);
    Vec3<double> direction(1, 0, 0);
    Vec3<double> plane_normal(-1, 0, 0); // Facing back toward origin
    Vec3<double> plane_point(5, 0, 0);

    double t =
        ray_plane_intersection(origin, direction, plane_normal, plane_point);
    EXPECT_NEAR(t, 5.0, 1e-10);
}

TEST(RayPlaneIntersection, AngledHit) {
    Vec3<double> origin(0, 0, 0);
    Vec3<double> direction(1, 0, 0);
    Vec3<double> plane_normal(1, 0, 0); // Facing away
    Vec3<double> plane_point(5, 0, 0);

    double t =
        ray_plane_intersection(origin, direction, plane_normal, plane_point);
    EXPECT_NEAR(t, 5.0, 1e-10); // Should still work
}

TEST(RayPlaneIntersection, ParallelRay) {
    Vec3<double> origin(0, 0, 0);
    Vec3<double> direction(1, 0, 0);
    Vec3<double> plane_normal(0, 1, 0); // Plane parallel to ray
    Vec3<double> plane_point(5, 5, 0);

    double t =
        ray_plane_intersection(origin, direction, plane_normal, plane_point);
    EXPECT_LT(t, 0.0); // No intersection
}

TEST(RayPlaneIntersection, BehindOrigin) {
    Vec3<double> origin(0, 0, 0);
    Vec3<double> direction(1, 0, 0);
    Vec3<double> plane_normal(-1, 0, 0);
    Vec3<double> plane_point(-5, 0, 0); // Behind ray origin

    double t =
        ray_plane_intersection(origin, direction, plane_normal, plane_point);
    EXPECT_LT(t, 0.0); // Behind origin
}

// =============================================================================
// Point in Cone Tests
// =============================================================================

TEST(PointInCone, InsideCone) {
    Vec3<double> apex(0, 0, 0);
    Vec3<double> axis(1, 0, 0);
    double half_angle = M_PI / 6; // 30 degrees

    Vec3<double> point(10, 1, 0); // Small offset in Y

    double inside = point_in_cone(point, apex, axis, half_angle);
    EXPECT_NEAR(inside, 1.0, 1e-10);
}

TEST(PointInCone, OutsideCone) {
    Vec3<double> apex(0, 0, 0);
    Vec3<double> axis(1, 0, 0);
    double half_angle = M_PI / 6; // 30 degrees

    Vec3<double> point(1, 10, 0); // Large offset - outside cone

    double inside = point_in_cone(point, apex, axis, half_angle);
    EXPECT_NEAR(inside, 0.0, 1e-10);
}

TEST(PointInCone, OnBoundary) {
    Vec3<double> apex(0, 0, 0);
    Vec3<double> axis(1, 0, 0);
    double half_angle = M_PI / 4; // 45 degrees

    // Point exactly on cone surface: angle from axis = 45 deg
    Vec3<double> point(10, 10, 0);

    double inside = point_in_cone(point, apex, axis, half_angle);
    // On boundary, cos_angle ≈ cos_half_angle, so marginally inside or outside
    // Due to numerical precision, could be either
    EXPECT_LE(inside, 1.0);
    EXPECT_GE(inside, 0.0);
}

TEST(PointInCone, AtApex) {
    Vec3<double> apex(0, 0, 0);
    Vec3<double> axis(1, 0, 0);
    double half_angle = M_PI / 6;

    double inside = point_in_cone(apex, apex, axis, half_angle);
    EXPECT_NEAR(inside, 1.0, 1e-10); // Point at apex is inside
}

// =============================================================================
// Project to Plane Tests
// =============================================================================

TEST(ProjectToPlane, SimpleProjection) {
    Vec3<double> plane_normal(0, 0, 1);
    Vec3<double> plane_point(0, 0, 0);
    Vec3<double> point(3, 4, 10);

    Vec3<double> projected = project_to_plane(point, plane_normal, plane_point);
    EXPECT_NEAR(projected(0), 3.0, 1e-12);
    EXPECT_NEAR(projected(1), 4.0, 1e-12);
    EXPECT_NEAR(projected(2), 0.0, 1e-12);
}

TEST(ProjectToPlane, OffsetPlane) {
    Vec3<double> plane_normal(0, 0, 1);
    Vec3<double> plane_point(0, 0, 5);
    Vec3<double> point(3, 4, 10);

    Vec3<double> projected = project_to_plane(point, plane_normal, plane_point);
    EXPECT_NEAR(projected(0), 3.0, 1e-12);
    EXPECT_NEAR(projected(1), 4.0, 1e-12);
    EXPECT_NEAR(projected(2), 5.0, 1e-12);
}

TEST(ProjectToPlane, AngledPlane) {
    // 45-degree plane
    Vec3<double> plane_normal(1.0 / std::sqrt(2.0), 0, 1.0 / std::sqrt(2.0));
    Vec3<double> plane_point(0, 0, 0);
    Vec3<double> point(2, 0, 2);

    Vec3<double> projected = project_to_plane(point, plane_normal, plane_point);
    // Point is on the plane (dot with normal = 0)
    double dist = (projected - plane_point).dot(plane_normal);
    EXPECT_NEAR(dist, 0.0, 1e-12);
}

// =============================================================================
// Ground Track Tests
// =============================================================================

TEST(GroundTrack, BasicGroundTrack) {
    // Position above Equator at prime meridian
    Vec3<double> r_ecef(7000000, 0, 0); // ~622 km altitude

    LLA<double> ground = ground_track_point(r_ecef);

    EXPECT_NEAR(ground.lon, 0.0, 1e-10);
    EXPECT_NEAR(ground.lat, 0.0, 1e-10);
    EXPECT_NEAR(ground.alt, 0.0, 1e-10);
}

TEST(GroundTrack, NonEquatorial) {
    // First convert a known LLA to ECEF at altitude, then verify ground track
    double lat = 45.0 * M_PI / 180.0;
    double lon = 30.0 * M_PI / 180.0;
    double alt = 500000.0; // 500 km

    LLA<double> lla_in(lon, lat, alt);
    Vec3<double> r_ecef = lla_to_ecef(lla_in);

    LLA<double> ground = ground_track_point(r_ecef);

    EXPECT_NEAR(ground.lon, lon, 1e-8);
    EXPECT_NEAR(ground.lat, lat, 1e-8);
    EXPECT_NEAR(ground.alt, 0.0, 1e-10);
}

TEST(GroundTrack, ECEFOutput) {
    double lat = 45.0 * M_PI / 180.0;
    double lon = 30.0 * M_PI / 180.0;
    double alt = 500000.0;

    LLA<double> lla_in(lon, lat, alt);
    Vec3<double> r_ecef = lla_to_ecef(lla_in);

    Vec3<double> ground_ecef = ground_track_ecef(r_ecef);

    // Verify this is on the surface
    LLA<double> ground_lla = ecef_to_lla(ground_ecef);
    EXPECT_NEAR(ground_lla.alt, 0.0, 1e-6);
}

// =============================================================================
// Symbolic Compatibility Tests
// =============================================================================

TEST(GeometrySymbolic, SlantRangeSymbolic) {
    using MX = casadi::MX;

    MX p1 = MX::sym("p1", 3);
    MX p2 = MX::sym("p2", 3);

    Vec3<MX> r1, r2;
    r1 << p1(0), p1(1), p1(2);
    r2 << p2(0), p2(1), p2(2);

    MX range = slant_range(r1, r2);

    casadi::Function f("slant_range", {p1, p2}, {range});

    // Evaluate numerically
    std::vector<casadi::DM> args = {casadi::DM::vertcat({0, 0, 0}),
                                    casadi::DM::vertcat({3, 4, 0})};
    casadi::DM result = f(args).at(0);

    EXPECT_NEAR(static_cast<double>(result), 5.0, 1e-10);
}

TEST(GeometrySymbolic, LOSAnglesSymbolic) {
    using MX = casadi::MX;

    MX p1 = MX::sym("p1", 3);
    MX p2 = MX::sym("p2", 3);

    Vec3<MX> r1, r2;
    r1 << p1(0), p1(1), p1(2);
    r2 << p2(0), p2(1), p2(2);

    Vec2<MX> angles = los_angles(r1, r2);

    casadi::Function f("los_angles", {p1, p2}, {angles(0), angles(1)});

    std::vector<casadi::DM> args = {casadi::DM::vertcat({0, 0, 0}),
                                    casadi::DM::vertcat({10, 0, 0})};
    std::vector<casadi::DM> result = f(args);

    EXPECT_NEAR(static_cast<double>(result[0]), 0.0, 1e-10);
    EXPECT_NEAR(static_cast<double>(result[1]), 0.0, 1e-10);
}

TEST(GeometrySymbolic, RaySphereSymbolic) {
    using MX = casadi::MX;

    MX origin = MX::sym("origin", 3);
    MX direction = MX::sym("dir", 3);
    MX center = MX::sym("center", 3);
    MX radius = MX::sym("r");

    Vec3<MX> o, d, c;
    o << origin(0), origin(1), origin(2);
    d << direction(0), direction(1), direction(2);
    c << center(0), center(1), center(2);

    MX t = ray_sphere_intersection(o, d, c, radius);

    casadi::Function f("ray_sphere", {origin, direction, center, radius}, {t});

    std::vector<casadi::DM> args = {casadi::DM::vertcat({0, 0, 0}),
                                    casadi::DM::vertcat({1, 0, 0}),
                                    casadi::DM::vertcat({10, 0, 0}), 2.0};
    casadi::DM result = f(args).at(0);

    EXPECT_NEAR(static_cast<double>(result), 8.0, 1e-10);
}

TEST(GeometrySymbolic, ProjectToPlaneSymbolic) {
    using MX = casadi::MX;

    MX point = MX::sym("point", 3);
    MX normal = MX::sym("normal", 3);
    MX plane_pt = MX::sym("plane_pt", 3);

    Vec3<MX> pt, n, pp;
    pt << point(0), point(1), point(2);
    n << normal(0), normal(1), normal(2);
    pp << plane_pt(0), plane_pt(1), plane_pt(2);

    Vec3<MX> projected = project_to_plane(pt, n, pp);

    casadi::Function f("project", {point, normal, plane_pt},
                       {projected(0), projected(1), projected(2)});

    std::vector<casadi::DM> args = {casadi::DM::vertcat({3, 4, 10}),
                                    casadi::DM::vertcat({0, 0, 1}),
                                    casadi::DM::vertcat({0, 0, 0})};
    std::vector<casadi::DM> result = f(args);

    EXPECT_NEAR(static_cast<double>(result[0]), 3.0, 1e-10);
    EXPECT_NEAR(static_cast<double>(result[1]), 4.0, 1e-10);
    EXPECT_NEAR(static_cast<double>(result[2]), 0.0, 1e-10);
}
