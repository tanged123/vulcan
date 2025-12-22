// Test suite for Geodetic Utilities (Phase 12)
// Tests for distance calculations, bearings, horizon, visibility, and CDA frame

#include <gtest/gtest.h>
#include <vulcan/coordinates/LocalFrames.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/geodetic/GeodesicUtils.hpp>

#include <janus/janus.hpp>

using namespace vulcan;
using namespace vulcan::geodetic;

// =============================================================================
// Haversine Distance Tests
// =============================================================================

TEST(GeodesicUtils, Haversine_SamePoint) {
    LLA<double> p1(0.0, 0.0, 0.0);
    auto d = haversine_distance(p1, p1);
    EXPECT_NEAR(d, 0.0, 1e-10);
}

TEST(GeodesicUtils, Haversine_Equator_90Degrees) {
    // Quarter of Earth circumference along equator
    LLA<double> p1(0.0, 0.0, 0.0);
    LLA<double> p2(constants::angle::pi / 2.0, 0.0, 0.0);

    auto d = haversine_distance(p1, p2);
    // Expected: ~10,007 km (quarter circumference using mean radius 6371km)
    EXPECT_NEAR(d, 10007557.0, 10000.0); // Within 10km
}

TEST(GeodesicUtils, Haversine_Poles) {
    // North to South pole
    LLA<double> north(0.0, constants::angle::pi / 2.0, 0.0);
    LLA<double> south(0.0, -constants::angle::pi / 2.0, 0.0);

    auto d = haversine_distance(north, south);
    // Expected: half circumference using mean radius ~20,015 km
    EXPECT_NEAR(d, 20015115.0, 10000.0);
}

TEST(GeodesicUtils, Haversine_ShortDistance) {
    // Two points 1 km apart approximately
    double lat = 45.0 * constants::angle::deg2rad;
    double lon1 = 0.0;
    // ~0.013 degrees longitude at 45N is about 1 km
    double lon2 = 0.013 * constants::angle::deg2rad;

    LLA<double> p1(lon1, lat, 0.0);
    LLA<double> p2(lon2, lat, 0.0);

    auto d = haversine_distance(p1, p2);
    EXPECT_NEAR(d, 1000.0, 50.0); // Within 50m
}

// =============================================================================
// Vincenty Distance Tests
// =============================================================================

TEST(GeodesicUtils, Vincenty_SamePoint) {
    LLA<double> p1(0.0, 0.0, 0.0);
    auto d = great_circle_distance(p1, p1);
    EXPECT_NEAR(d, 0.0, 1.0); // Within 1m
}

TEST(GeodesicUtils, Vincenty_LondonToNewYork) {
    // London: 51.5074° N, 0.1278° W
    // New York: 40.7128° N, 74.0060° W
    LLA<double> london(-0.1278 * constants::angle::deg2rad,
                       51.5074 * constants::angle::deg2rad, 0.0);
    LLA<double> nyc(-74.0060 * constants::angle::deg2rad,
                    40.7128 * constants::angle::deg2rad, 0.0);

    auto d = great_circle_distance(london, nyc);
    // Expected: ~5,585 km (Vincenty gives slightly different result than
    // GeographicLib)
    EXPECT_NEAR(d, 5585000.0, 1000.0); // Within 1km
}

TEST(GeodesicUtils, Vincenty_SydneyToSantiago) {
    // Sydney: 33.8688° S, 151.2093° E
    // Santiago: 33.4489° S, 70.6693° W
    LLA<double> sydney(151.2093 * constants::angle::deg2rad,
                       -33.8688 * constants::angle::deg2rad, 0.0);
    LLA<double> santiago(-70.6693 * constants::angle::deg2rad,
                         -33.4489 * constants::angle::deg2rad, 0.0);

    auto d = great_circle_distance(sydney, santiago);
    // Expected: ~11,369 km
    EXPECT_NEAR(d, 11369000.0, 50000.0); // Within 50km
}

TEST(GeodesicUtils, Vincenty_ShortDistance) {
    // 1 km test
    double lat = 45.0 * constants::angle::deg2rad;
    double lon1 = 0.0;
    double lon2 = 0.013 * constants::angle::deg2rad;

    LLA<double> p1(lon1, lat, 0.0);
    LLA<double> p2(lon2, lat, 0.0);

    auto d = great_circle_distance(p1, p2);
    EXPECT_NEAR(d, 1025.0, 50.0); // Within 50m
}

// =============================================================================
// Bearing Tests
// =============================================================================

TEST(GeodesicUtils, InitialBearing_DueNorth) {
    LLA<double> p1(0.0, 0.0, 0.0);
    LLA<double> p2(0.0, 10.0 * constants::angle::deg2rad, 0.0);

    auto bearing = initial_bearing(p1, p2);
    EXPECT_NEAR(bearing, 0.0, 0.001); // Due North = 0°
}

TEST(GeodesicUtils, InitialBearing_DueEast) {
    LLA<double> p1(0.0, 0.0, 0.0);
    LLA<double> p2(10.0 * constants::angle::deg2rad, 0.0, 0.0);

    auto bearing = initial_bearing(p1, p2);
    EXPECT_NEAR(bearing, constants::angle::pi / 2.0, 0.001); // Due East = 90°
}

TEST(GeodesicUtils, InitialBearing_DueSouth) {
    LLA<double> p1(0.0, 10.0 * constants::angle::deg2rad, 0.0);
    LLA<double> p2(0.0, 0.0, 0.0);

    auto bearing = initial_bearing(p1, p2);
    EXPECT_NEAR(bearing, constants::angle::pi, 0.001); // Due South = 180°
}

TEST(GeodesicUtils, InitialBearing_DueWest) {
    LLA<double> p1(10.0 * constants::angle::deg2rad, 0.0, 0.0);
    LLA<double> p2(0.0, 0.0, 0.0);

    auto bearing = initial_bearing(p1, p2);
    EXPECT_NEAR(bearing, 3.0 * constants::angle::pi / 2.0,
                0.001); // Due West = 270°
}

TEST(GeodesicUtils, FinalBearing_GreatCircle) {
    // Final bearing differs from initial for great circle routes
    LLA<double> london(-0.1278 * constants::angle::deg2rad,
                       51.5074 * constants::angle::deg2rad, 0.0);
    LLA<double> nyc(-74.0060 * constants::angle::deg2rad,
                    40.7128 * constants::angle::deg2rad, 0.0);

    auto init = initial_bearing(london, nyc);
    auto final_b = final_bearing(london, nyc);

    // Should be different (not a rhumb line)
    EXPECT_NE(init, final_b);

    // Both bearings should be in the western hemisphere for London->NYC
    // Initial: heading northwest (between 270° and 360° or ~288°)
    // Final: heading southwest (could be > 180° depending on normalization)
    EXPECT_GT(init, constants::angle::pi); // Northwest > 180°
}

// =============================================================================
// Destination Point Tests
// =============================================================================

TEST(GeodesicUtils, DestinationPoint_Roundtrip) {
    LLA<double> start(0.0, 45.0 * constants::angle::deg2rad, 1000.0);
    double bearing = 45.0 * constants::angle::deg2rad; // Northeast
    double distance = 100000.0;                        // 100 km

    // Go to destination
    auto dest = destination_point(start, bearing, distance);

    // Check distance to destination
    auto calc_dist = great_circle_distance(start, dest);
    EXPECT_NEAR(calc_dist, distance, 1.0); // Within 1m

    // Altitude should be preserved
    EXPECT_NEAR(dest.alt, start.alt, 1e-6);
}

TEST(GeodesicUtils, DestinationPoint_Cardinal) {
    LLA<double> start(0.0, 0.0, 0.0); // Equator, prime meridian
    double distance = 111000.0;       // ~1 degree at equator

    // Go due North
    auto dest_n = destination_point(start, 0.0, distance);
    EXPECT_NEAR(dest_n.lat, 1.0 * constants::angle::deg2rad, 0.01);
    EXPECT_NEAR(dest_n.lon, 0.0, 0.01);

    // Go due East
    auto dest_e =
        destination_point(start, constants::angle::pi / 2.0, distance);
    EXPECT_NEAR(dest_e.lat, 0.0, 0.01);
    EXPECT_NEAR(dest_e.lon, 1.0 * constants::angle::deg2rad, 0.01);
}

// =============================================================================
// Horizon Distance Tests
// =============================================================================

TEST(GeodesicUtils, HorizonDistance_SeaLevel) {
    auto d = horizon_distance(0.0);
    EXPECT_NEAR(d, 0.0, 1.0);
}

TEST(GeodesicUtils, HorizonDistance_Aircraft) {
    // 10 km altitude (cruising aircraft)
    auto d = horizon_distance(10000.0);
    // Expected: ~357 km
    EXPECT_NEAR(d, 357000.0, 5000.0); // Within 5 km
}

TEST(GeodesicUtils, HorizonDistance_ISS) {
    // ISS at ~400 km
    auto d = horizon_distance(400000.0);
    // Expected: ~2,294 km
    EXPECT_NEAR(d, 2294000.0, 50000.0); // Within 50 km
}

// =============================================================================
// Visibility Tests
// =============================================================================

TEST(GeodesicUtils, IsVisible_ClosePoints) {
    // Two points 10 km apart, both at 100m altitude
    LLA<double> p1(0.0, 0.0, 100.0);
    LLA<double> p2(0.001 * constants::angle::deg2rad, 0.0, 100.0);

    auto vis = is_visible(p1, p2);
    EXPECT_GT(vis, 0.0); // Should be visible
}

TEST(GeodesicUtils, IsVisible_FarPoints_SeaLevel) {
    // Two points 500 km apart at sea level
    LLA<double> p1(0.0, 0.0, 0.0);
    LLA<double> p2(5.0 * constants::angle::deg2rad, 0.0, 0.0);

    auto vis = is_visible(p1, p2);
    EXPECT_LT(vis, 0.0); // Should NOT be visible (below horizon)
}

TEST(GeodesicUtils, IsVisible_HighAltitude) {
    // Observer at 30km (high altitude balloon), target 500km away at sea level
    LLA<double> observer(0.0, 0.0, 30000.0);
    LLA<double> target(5.0 * constants::angle::deg2rad, 0.0, 0.0);

    auto vis = is_visible(observer, target);
    EXPECT_GT(vis, 0.0); // Should be visible from high altitude
}

// =============================================================================
// Ray-Ellipsoid Intersection Tests
// =============================================================================

TEST(GeodesicUtils, RayEllipsoid_VerticalDown) {
    // Origin above surface, pointing down
    Vec3<double> origin;
    origin << constants::wgs84::a + 100000.0, 0.0, 0.0; // 100km above equator

    Vec3<double> direction;
    direction << -1.0, 0.0, 0.0; // Pointing toward Earth center

    auto result = ray_ellipsoid_intersection(origin, direction);

    // Discriminant should be positive (ray intersects ellipsoid)
    EXPECT_GT(result.hit, 0.0);

    // t_far should give the entry point when looking inward from outside
    // (t_near may be negative for outer intersection behind us)
    EXPECT_GT(result.t_far, 0.0); // Forward intersection exists
}

TEST(GeodesicUtils, RayEllipsoid_Miss) {
    // Origin above surface, pointing away
    Vec3<double> origin;
    origin << constants::wgs84::a + 100000.0, 0.0, 0.0;

    Vec3<double> direction;
    direction << 1.0, 0.0, 0.0; // Pointing away from Earth

    auto result = ray_ellipsoid_intersection(origin, direction);
    // t_near should be negative (intersection behind ray origin)
    EXPECT_LT(result.t_near, 0.0);
}

TEST(GeodesicUtils, RayEllipsoid_Tangent) {
    // Ray tangent to surface (roughly)
    Vec3<double> origin;
    origin << constants::wgs84::a + 1000.0, 0.0, 0.0;

    // Direction tangent to sphere at equator
    Vec3<double> direction;
    direction << 0.0, 1.0, 0.0; // Pointing East

    auto result = ray_ellipsoid_intersection(origin, direction);
    // Discriminant should be small but positive for near-tangent
    // This is a grazing case
}

// =============================================================================
// CDA Frame Tests
// =============================================================================

TEST(CDAFrame, FromBearing_North) {
    // CDA with bearing = 0 (North) should have D=North, C=East
    LLA<double> origin(0.0, 45.0 * constants::angle::deg2rad, 0.0);
    double bearing = 0.0; // Due North

    auto frame = local_cda(origin, bearing);

    // X-axis (downrange) should align with North at this latitude
    // At 45N, North in ECEF is roughly (-sin(45), 0, cos(45))
    Vec3<double> expected_north;
    expected_north << -std::sin(45.0 * constants::angle::deg2rad), 0.0,
        std::cos(45.0 * constants::angle::deg2rad);

    EXPECT_NEAR(frame.x_axis.dot(expected_north), 1.0, 0.01);
}

TEST(CDAFrame, FromBearing_East) {
    // CDA with bearing = 90° (East)
    LLA<double> origin(0.0, 0.0, 0.0);           // Equator, prime meridian
    double bearing = constants::angle::pi / 2.0; // Due East

    auto frame = local_cda(origin, bearing);

    // At equator/prime meridian: East in ECEF is (0, 1, 0)
    Vec3<double> expected_east;
    expected_east << 0.0, 1.0, 0.0;

    EXPECT_NEAR(frame.x_axis.dot(expected_east), 1.0, 0.01);
}

TEST(CDAFrame, Roundtrip_ECEF) {
    // Convert point to CDA and back
    LLA<double> ref(10.0 * constants::angle::deg2rad,
                    45.0 * constants::angle::deg2rad, 0.0);
    double bearing = 30.0 * constants::angle::deg2rad;

    // Target point 10km downrange, 5km crossrange, 1km up
    LLA<double> target_lla(10.1 * constants::angle::deg2rad,
                           45.05 * constants::angle::deg2rad, 1000.0);
    Vec3<double> target_ecef = lla_to_ecef(target_lla);

    // Convert to CDA
    Vec3<double> cda = ecef_to_cda(target_ecef, ref, bearing);

    // Convert back to ECEF
    Vec3<double> recovered = cda_to_ecef(cda, ref, bearing);

    // Should match original
    EXPECT_NEAR((recovered - target_ecef).norm(), 0.0, 0.1); // Within 10cm
}

TEST(CDAFrame, DownRange_Distance) {
    // Point along bearing should have positive downrange, zero crossrange
    LLA<double> origin(0.0, 0.0, 0.0);
    double bearing = 0.0;      // Due North
    double distance = 10000.0; // 10 km

    // Destination point along bearing
    auto dest = destination_point(origin, bearing, distance);
    Vec3<double> dest_ecef = lla_to_ecef(dest);

    // Convert to CDA
    Vec3<double> cda = ecef_to_cda(dest_ecef, origin, bearing);

    // Downrange should be ~10km, crossrange ~0
    EXPECT_NEAR(cda(0), distance, 100.0); // Within 100m
    EXPECT_NEAR(cda(1), 0.0, 100.0);      // Crossrange near zero
}

TEST(CDAFrame, CrossRange_Offset) {
    // Point perpendicular to bearing should have zero downrange
    LLA<double> origin(0.0, 0.0, 0.0);
    double bearing = 0.0;      // Due North
    double distance = 10000.0; // 10 km

    // Point due East (90° from North)
    double cross_bearing = constants::angle::pi / 2.0;
    auto dest = destination_point(origin, cross_bearing, distance);
    Vec3<double> dest_ecef = lla_to_ecef(dest);

    // Convert to CDA
    Vec3<double> cda = ecef_to_cda(dest_ecef, origin, bearing);

    // Crossrange should be ~10km, downrange ~0
    EXPECT_NEAR(cda(0), 0.0, 100.0);      // Downrange near zero
    EXPECT_NEAR(cda(1), distance, 100.0); // Crossrange ~10km
}

// =============================================================================
// Symbolic Mode Tests
// =============================================================================

TEST(GeodesicSymbolic, Haversine_GraphBuilds) {
    auto lon1 = casadi::MX::sym("lon1");
    auto lat1 = casadi::MX::sym("lat1");
    auto lon2 = casadi::MX::sym("lon2");
    auto lat2 = casadi::MX::sym("lat2");

    LLA<casadi::MX> p1(lon1, lat1, casadi::MX(0));
    LLA<casadi::MX> p2(lon2, lat2, casadi::MX(0));

    auto d = haversine_distance(p1, p2);
    EXPECT_FALSE(d.is_empty());
}

TEST(GeodesicSymbolic, InitialBearing_GraphBuilds) {
    auto lon1 = casadi::MX::sym("lon1");
    auto lat1 = casadi::MX::sym("lat1");
    auto lon2 = casadi::MX::sym("lon2");
    auto lat2 = casadi::MX::sym("lat2");

    LLA<casadi::MX> p1(lon1, lat1, casadi::MX(0));
    LLA<casadi::MX> p2(lon2, lat2, casadi::MX(0));

    auto b = initial_bearing(p1, p2);
    EXPECT_FALSE(b.is_empty());
}

TEST(GeodesicSymbolic, DestinationPoint_GraphBuilds) {
    auto lon = casadi::MX::sym("lon");
    auto lat = casadi::MX::sym("lat");
    auto bearing = casadi::MX::sym("bearing");
    auto dist = casadi::MX::sym("dist");

    LLA<casadi::MX> start(lon, lat, casadi::MX(0));

    auto dest = destination_point(start, bearing, dist);
    EXPECT_FALSE(dest.lon.is_empty());
    EXPECT_FALSE(dest.lat.is_empty());
}

TEST(GeodesicSymbolic, HorizonDistance_GraphBuilds) {
    auto alt = casadi::MX::sym("alt");
    auto d = horizon_distance(alt);
    EXPECT_FALSE(d.is_empty());
}

TEST(GeodesicSymbolic, CDA_GraphBuilds) {
    auto lon = casadi::MX::sym("lon");
    auto lat = casadi::MX::sym("lat");
    auto bearing = casadi::MX::sym("bearing");

    LLA<casadi::MX> origin(lon, lat, casadi::MX(0));

    auto frame = local_cda(origin, bearing);

    // Frame should have valid axes
    EXPECT_FALSE(frame.x_axis(0).is_empty());
}
