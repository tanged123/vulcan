#include <gtest/gtest.h>
#include <vulcan/coordinates/Geodetic.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/Units.hpp>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/Units.hpp>

#include <janus/janus.hpp>

using namespace vulcan::units;

// ============================================
// LLA and Spherical Struct Tests
// ============================================
TEST(Geodetic, LLA_DefaultConstruction) {
    vulcan::LLA<double> lla;
    EXPECT_DOUBLE_EQ(lla.lon.value(), 0.0);
    EXPECT_DOUBLE_EQ(lla.lat.value(), 0.0);
    EXPECT_DOUBLE_EQ(lla.alt.value(), 0.0);
}

TEST(Geodetic, LLA_ValueConstruction) {
    double lon = 0.5;
    double lat = 0.3;
    double alt = 1000.0;
    vulcan::LLA<double> lla{vulcan::Quantity<rad>(lon),
                            vulcan::Quantity<rad>(lat),
                            vulcan::Quantity<m>(alt)};
    EXPECT_DOUBLE_EQ(lla.lon.value(), lon);
    EXPECT_DOUBLE_EQ(lla.lat.value(), lat);
    EXPECT_DOUBLE_EQ(lla.alt.value(), alt);
}

TEST(Geodetic, Spherical_DefaultConstruction) {
    vulcan::Spherical<double> geo;
    EXPECT_DOUBLE_EQ(geo.lon.value(), 0.0);
    EXPECT_DOUBLE_EQ(geo.lat_gc.value(), 0.0);
    EXPECT_DOUBLE_EQ(geo.radius.value(), 0.0);
}

// ============================================
// LLA to ECEF Conversion Tests
// ============================================
TEST(Geodetic, LLA_to_ECEF_Equator) {
    // Point on equator at prime meridian, sea level
    vulcan::LLA<double> lla(vulcan::Quantity<rad>(0.0),
                            vulcan::Quantity<rad>(0.0),
                            vulcan::Quantity<m>(0.0));
    auto r = vulcan::lla_to_ecef(lla);

    // Should be at X = semi-major axis, Y = 0, Z = 0
    const double a = vulcan::constants::wgs84::a.value();
    EXPECT_NEAR(r(0).value(), a, 0.001);
    EXPECT_NEAR(r(1).value(), 0.0, 0.001);
    EXPECT_NEAR(r(2).value(), 0.0, 0.001);
}

TEST(Geodetic, LLA_to_ECEF_NorthPole) {
    // North Pole at sea level
    vulcan::LLA<double> lla(
        vulcan::Quantity<rad>(0.0),
        vulcan::Quantity<rad>(vulcan::constants::angle::pi.value() / 2.0),
        vulcan::Quantity<m>(0.0));
    auto r = vulcan::lla_to_ecef(lla);

    // Should be at X = 0, Y = 0, Z = semi-minor axis
    const double b = vulcan::constants::wgs84::b.value();
    EXPECT_NEAR(r(0).value(), 0.0, 0.001);
    EXPECT_NEAR(r(1).value(), 0.0, 0.001);
    EXPECT_NEAR(r(2).value(), b, 0.001);
}

TEST(Geodetic, LLA_to_ECEF_SouthPole) {
    // South Pole at sea level
    vulcan::LLA<double> lla(
        vulcan::Quantity<rad>(0.0),
        vulcan::Quantity<rad>(-vulcan::constants::angle::pi.value() / 2.0),
        vulcan::Quantity<m>(0.0));
    auto r = vulcan::lla_to_ecef(lla);

    // Should be at X = 0, Y = 0, Z = -semi-minor axis
    const double b = vulcan::constants::wgs84::b.value();
    EXPECT_NEAR(r(0).value(), 0.0, 0.001);
    EXPECT_NEAR(r(1).value(), 0.0, 0.001);
    EXPECT_NEAR(r(2).value(), -b, 0.001);
}

TEST(Geodetic, LLA_to_ECEF_90East) {
    // Equator at 90 degrees East, sea level
    vulcan::LLA<double> lla(
        vulcan::Quantity<rad>(vulcan::constants::angle::pi.value() / 2.0),
        vulcan::Quantity<rad>(0.0), vulcan::Quantity<m>(0.0));
    auto r = vulcan::lla_to_ecef(lla);

    // Should be at X = 0, Y = semi-major axis, Z = 0
    const double a = vulcan::constants::wgs84::a.value();
    EXPECT_NEAR(r(0).value(), 0.0, 0.001);
    EXPECT_NEAR(r(1).value(), a, 0.001);
    EXPECT_NEAR(r(2).value(), 0.0, 0.001);
}

TEST(Geodetic, LLA_to_ECEF_WithAltitude) {
    // Equator at prime meridian, 1000 km altitude
    double alt = 1000000.0; // 1000 km
    vulcan::LLA<double> lla{vulcan::Quantity<rad>(0.0),
                            vulcan::Quantity<rad>(0.0),
                            vulcan::Quantity<m>(alt)};
    auto r = vulcan::lla_to_ecef(lla);

    // Should be at X = a + alt, Y = 0, Z = 0
    const double a = vulcan::constants::wgs84::a.value();
    EXPECT_NEAR(r(0).value(), a + alt, 0.001);
    EXPECT_NEAR(r(1).value(), 0.0, 0.001);
    EXPECT_NEAR(r(2).value(), 0.0, 0.001);
}

// ============================================
// ECEF to LLA Conversion Tests (Vermeille Algorithm)
// ============================================
TEST(Geodetic, ECEF_to_LLA_Equator) {
    // ECEF on equator at prime meridian
    const double a = vulcan::constants::wgs84::a.value();
    vulcan::Vec3<vulcan::Quantity<m>> r;
    r << vulcan::Quantity<m>(a), vulcan::Quantity<m>(0.0),
        vulcan::Quantity<m>(0.0);

    auto lla = vulcan::ecef_to_lla(r);

    EXPECT_NEAR(lla.lon.value(), 0.0, 1e-10);
    EXPECT_NEAR(lla.lat.value(), 0.0, 1e-10);
    EXPECT_NEAR(lla.alt.value(), 0.0, 0.001);
}

TEST(Geodetic, ECEF_to_LLA_NorthPole) {
    // ECEF at North Pole
    const double b = vulcan::constants::wgs84::b.value();
    vulcan::Vec3<vulcan::Quantity<m>> r;
    r << vulcan::Quantity<m>(0.0), vulcan::Quantity<m>(0.0),
        vulcan::Quantity<m>(b);

    auto lla = vulcan::ecef_to_lla(r);

    EXPECT_NEAR(lla.lat.value(), vulcan::constants::angle::pi.value() / 2.0,
                1e-10);
    EXPECT_NEAR(lla.alt.value(), 0.0, 0.001);
    // Longitude is undefined at poles, but we set it to 0
    EXPECT_NEAR(lla.lon.value(), 0.0, 1e-10);
}

TEST(Geodetic, ECEF_to_LLA_SouthPole) {
    // ECEF at South Pole
    const double b = vulcan::constants::wgs84::b.value();
    vulcan::Vec3<vulcan::Quantity<m>> r;
    r << vulcan::Quantity<m>(0.0), vulcan::Quantity<m>(0.0),
        vulcan::Quantity<m>(-b);

    auto lla = vulcan::ecef_to_lla(r);

    EXPECT_NEAR(lla.lat.value(), -vulcan::constants::angle::pi.value() / 2.0,
                1e-10);
    EXPECT_NEAR(lla.alt.value(), 0.0, 0.001);
}

TEST(Geodetic, ECEF_to_LLA_90East) {
    // ECEF on equator at 90E
    const double a = vulcan::constants::wgs84::a.value();
    vulcan::Vec3<vulcan::Quantity<m>> r;
    r << vulcan::Quantity<m>(0.0), vulcan::Quantity<m>(a),
        vulcan::Quantity<m>(0.0);

    auto lla = vulcan::ecef_to_lla(r);

    EXPECT_NEAR(lla.lon.value(), vulcan::constants::angle::pi.value() / 2.0,
                1e-10);
    EXPECT_NEAR(lla.lat.value(), 0.0, 1e-10);
    EXPECT_NEAR(lla.alt.value(), 0.0, 0.001);
}

TEST(Geodetic, ECEF_to_LLA_Geostationary) {
    // Geostationary orbit (altitude ~35786 km)
    double r_geo = 42164e3; // ~42164 km from Earth center
    vulcan::Vec3<vulcan::Quantity<m>> r;
    r << vulcan::Quantity<m>(r_geo), vulcan::Quantity<m>(0.0),
        vulcan::Quantity<m>(0.0);

    auto lla = vulcan::ecef_to_lla(r);

    EXPECT_NEAR(lla.lon.value(), 0.0, 1e-10);
    EXPECT_NEAR(lla.lat.value(), 0.0, 1e-10);
    EXPECT_NEAR(lla.alt.value(), r_geo - vulcan::constants::wgs84::a.value(),
                100.0);
}

// ============================================
// Roundtrip Tests (LLA -> ECEF -> LLA)
// ============================================
TEST(Geodetic, Roundtrip_Equator) {
    vulcan::LLA<double> lla_orig(vulcan::Quantity<rad>(0.0),
                                 vulcan::Quantity<rad>(0.0),
                                 vulcan::Quantity<m>(0.0));
    auto r = vulcan::lla_to_ecef(lla_orig);
    auto lla = vulcan::ecef_to_lla(r);

    EXPECT_NEAR(lla.lon.value(), lla_orig.lon.value(), 1e-10);
    EXPECT_NEAR(lla.lat.value(), lla_orig.lat.value(), 1e-10);
    EXPECT_NEAR(lla.alt.value(), lla_orig.alt.value(), 1e-6);
}

TEST(Geodetic, Roundtrip_MidLatitude) {
    // Washington DC ~38.9N, 77W, 100m altitude
    double lon = -77.0 * vulcan::constants::angle::deg2rad;
    double lat = 38.9 * vulcan::constants::angle::deg2rad;
    double alt = 100.0;
    vulcan::LLA<double> lla_orig{vulcan::Quantity<rad>(lon),
                                 vulcan::Quantity<rad>(lat),
                                 vulcan::Quantity<m>(alt)};

    auto r = vulcan::lla_to_ecef(lla_orig);
    auto lla = vulcan::ecef_to_lla(r);

    EXPECT_NEAR(lla.lon.value(), lla_orig.lon.value(), 1e-10);
    EXPECT_NEAR(lla.lat.value(), lla_orig.lat.value(), 1e-10);
    EXPECT_NEAR(lla.alt.value(), lla_orig.alt.value(), 1e-6);
}

TEST(Geodetic, Roundtrip_HighAltitude) {
    // High altitude (500 km)
    double lon = 45.0 * vulcan::constants::angle::deg2rad;
    double lat = -30.0 * vulcan::constants::angle::deg2rad;
    double alt = 500000.0; // 500 km
    vulcan::LLA<double> lla_orig{vulcan::Quantity<rad>(lon),
                                 vulcan::Quantity<rad>(lat),
                                 vulcan::Quantity<m>(alt)};

    auto r = vulcan::lla_to_ecef(lla_orig);
    auto lla = vulcan::ecef_to_lla(r);

    EXPECT_NEAR(lla.lon.value(), lla_orig.lon.value(), 1e-10);
    EXPECT_NEAR(lla.lat.value(), lla_orig.lat.value(), 1e-10);
    EXPECT_NEAR(lla.alt.value(), lla_orig.alt.value(), 1e-4);
}

TEST(Geodetic, Roundtrip_Dateline) {
    // Near dateline (+/-180)
    double lon = 179.5 * vulcan::constants::angle::deg2rad;
    double lat = 10.0 * vulcan::constants::angle::deg2rad;
    double alt = 0.0;
    vulcan::LLA<double> lla_orig{vulcan::Quantity<rad>(lon),
                                 vulcan::Quantity<rad>(lat),
                                 vulcan::Quantity<m>(alt)};

    auto r = vulcan::lla_to_ecef(lla_orig);
    auto lla = vulcan::ecef_to_lla(r);

    EXPECT_NEAR(lla.lon.value(), lla_orig.lon.value(), 1e-10);
    EXPECT_NEAR(lla.lat.value(), lla_orig.lat.value(), 1e-10);
    EXPECT_NEAR(lla.alt.value(), lla_orig.alt.value(), 1e-6);
}

// ============================================
// Spherical Coordinate Tests
// ============================================
TEST(Geodetic, ECEF_to_Spherical_Equator) {
    vulcan::Vec3<double> r;
    r << 6371000.0, 0.0, 0.0;

    auto geo = vulcan::ecef_to_spherical(r);

    EXPECT_NEAR(geo.lon.value(), 0.0, 1e-10);
    EXPECT_NEAR(geo.lat_gc.value(), 0.0, 1e-10);
    EXPECT_NEAR(geo.radius.value(), 6371000.0, 0.001);
}

TEST(Geodetic, Spherical_to_ECEF_Roundtrip) {
    vulcan::Spherical<double> geo_orig(vulcan::Quantity<rad>(0.5),
                                       vulcan::Quantity<rad>(0.3),
                                       vulcan::Quantity<m>(6500000.0));

    auto r = vulcan::spherical_to_ecef(geo_orig);
    auto geo = vulcan::ecef_to_spherical(r);

    EXPECT_NEAR(geo.lon.value(), geo_orig.lon.value(), 1e-10);
    EXPECT_NEAR(geo.lat_gc.value(), geo_orig.lat_gc.value(), 1e-10);
    EXPECT_NEAR(geo.radius.value(), geo_orig.radius.value(), 1e-6);
}

// ============================================
// Latitude Conversion Tests
// ============================================
TEST(Geodetic, GeocentricToGeodetic_Equator) {
    // At equator, geodetic = geocentric
    auto lat_gd =
        vulcan::geocentric_to_geodetic_lat(vulcan::Quantity<rad>(0.0));
    EXPECT_NEAR(lat_gd.value(), 0.0, 1e-10);
}

TEST(Geodetic, GeocentricToGeodetic_Pole) {
    // At poles, geodetic ~ geocentric (singularity)
    double lat_gc_val = vulcan::constants::angle::pi.value() / 2.0;
    auto lat_gd =
        vulcan::geocentric_to_geodetic_lat(vulcan::Quantity<rad>(lat_gc_val));
    EXPECT_NEAR(lat_gd.value(), lat_gc_val, 1e-10);
}

TEST(Geodetic, GeodeticToGeocentric_MidLatitude) {
    // At mid-latitude, geocentric < geodetic
    double lat_gd_val = 45.0 * vulcan::constants::angle::deg2rad;
    auto lat_gc =
        vulcan::geodetic_to_geocentric_lat(vulcan::Quantity<rad>(lat_gd_val));

    // Geocentric should be smaller in magnitude
    EXPECT_LT(std::abs(lat_gc.value()), std::abs(lat_gd_val));
}

TEST(Geodetic, LatitudeConversion_Roundtrip) {
    double lat_gd_orig = 38.9 * vulcan::constants::angle::deg2rad;
    auto lat_gc =
        vulcan::geodetic_to_geocentric_lat(vulcan::Quantity<rad>(lat_gd_orig));
    auto lat_gd = vulcan::geocentric_to_geodetic_lat(lat_gc);

    EXPECT_NEAR(lat_gd.value(), lat_gd_orig, 1e-10);
}

// ============================================
// Radius of Curvature Tests
// ============================================
TEST(Geodetic, RadiusN_Equator) {
    // At equator, N = a
    auto N = vulcan::radius_of_curvature_N(vulcan::Quantity<rad>(0.0));
    EXPECT_NEAR(N.value(), vulcan::constants::wgs84::a.value(), 0.001);
}

TEST(Geodetic, RadiusN_Pole) {
    // At poles, N = a / sqrt(1 - e^2) = a^2 / b
    auto N = vulcan::radius_of_curvature_N(
        vulcan::Quantity<rad>(vulcan::constants::angle::pi.value() / 2.0));
    double expected = vulcan::constants::wgs84::a.value() *
                      vulcan::constants::wgs84::a.value() /
                      vulcan::constants::wgs84::b.value();
    EXPECT_NEAR(N.value(), expected, 0.001);
}

TEST(Geodetic, RadiusM_Equator) {
    // At equator, M = a(1-e^2)
    auto M = vulcan::radius_of_curvature_M(vulcan::Quantity<rad>(0.0));
    double expected = vulcan::constants::wgs84::a.value() *
                      (1.0 - vulcan::constants::wgs84::e2.value());
    EXPECT_NEAR(M.value(), expected, 0.001);
}

// ============================================
// Symbolic Graph Tests
// ============================================
TEST(Geodetic, Symbolic_LLA_to_ECEF) {
    using Scalar = janus::SymbolicScalar;

    // Create symbolic LLA
    Scalar lon = janus::sym("lon");
    Scalar lat = janus::sym("lat");
    Scalar alt = janus::sym("alt");
    vulcan::LLA<Scalar> lla{vulcan::Quantity<rad, Scalar>(lon),
                            vulcan::Quantity<rad, Scalar>(lat),
                            vulcan::Quantity<m, Scalar>(alt)};

    // Convert to ECEF
    auto r = vulcan::lla_to_ecef(lla);

    // Verify symbolic expressions were created
    EXPECT_FALSE(r(0).value().is_constant());
    EXPECT_FALSE(r(1).value().is_constant());
    EXPECT_FALSE(r(2).value().is_constant());

    // Create function and test
    janus::Function f("lla_to_ecef", {lon, lat, alt},
                      {r(0).value(), r(1).value(), r(2).value()});

    // Test with equator values
    auto result = f({0.0, 0.0, 0.0});

    const double a = vulcan::constants::wgs84::a.value();
    EXPECT_NEAR(result[0](0, 0), a, 0.001);
    EXPECT_NEAR(result[1](0, 0), 0.0, 0.001);
    EXPECT_NEAR(result[2](0, 0), 0.0, 0.001);
}

TEST(Geodetic, Symbolic_ECEF_to_LLA) {
    using Scalar = janus::SymbolicScalar;

    // Create symbolic ECEF
    Scalar x = janus::sym("x");
    Scalar y = janus::sym("y");
    Scalar z = janus::sym("z");
    vulcan::Vec3<vulcan::Quantity<m, Scalar>> r;
    r << vulcan::Quantity<m, Scalar>(x), vulcan::Quantity<m, Scalar>(y),
        vulcan::Quantity<m, Scalar>(z);

    // Convert to LLA
    auto lla = vulcan::ecef_to_lla(r);

    // Verify symbolic expressions were created
    EXPECT_FALSE(lla.lon.value().is_constant());
    EXPECT_FALSE(lla.lat.value().is_constant());
    EXPECT_FALSE(lla.alt.value().is_constant());

    // Create function and test
    janus::Function f("ecef_to_lla", {x, y, z},
                      {lla.lon.value(), lla.lat.value(), lla.alt.value()});

    // Test with equator values
    const double a = vulcan::constants::wgs84::a.value();
    auto result = f({a, 0.0, 0.0});

    EXPECT_NEAR(result[0](0, 0), 0.0, 1e-10); // lon
    EXPECT_NEAR(result[1](0, 0), 0.0, 1e-10); // lat
    EXPECT_NEAR(result[2](0, 0), 0.0, 0.001); // alt
}

TEST(Geodetic, Symbolic_Roundtrip) {
    using Scalar = janus::SymbolicScalar;

    // Create symbolic LLA
    Scalar lon = janus::sym("lon");
    Scalar lat = janus::sym("lat");
    Scalar alt = janus::sym("alt");
    vulcan::LLA<Scalar> lla_orig{vulcan::Quantity<rad, Scalar>(lon),
                                 vulcan::Quantity<rad, Scalar>(lat),
                                 vulcan::Quantity<m, Scalar>(alt)};

    // Roundtrip: LLA -> ECEF -> LLA
    auto r = vulcan::lla_to_ecef(lla_orig);
    auto lla = vulcan::ecef_to_lla(r);

    // Create function
    janus::Function f("lla_roundtrip", {lon, lat, alt},
                      {lla.lon.value(), lla.lat.value(), lla.alt.value()});

    // Test with Washington DC
    double test_lon = -77.0 * vulcan::constants::angle::deg2rad;
    double test_lat = 38.9 * vulcan::constants::angle::deg2rad;
    double test_alt = 100.0;

    auto result = f({test_lon, test_lat, test_alt});

    EXPECT_NEAR(result[0](0, 0), test_lon, 1e-10);
    EXPECT_NEAR(result[1](0, 0), test_lat, 1e-10);
    EXPECT_NEAR(result[2](0, 0), test_alt, 1e-4);
}

// ============================================
// Spherical Earth Model Tests
// ============================================
TEST(Geodetic, Roundtrip_SphericalEarth) {
    // With a spherical Earth, geodetic = geocentric
    auto model = vulcan::EarthModel::Spherical();

    double lon = 45.0 * vulcan::constants::angle::deg2rad;
    double lat = 30.0 * vulcan::constants::angle::deg2rad;
    double alt = 1000.0;
    vulcan::LLA<double> lla_orig{vulcan::Quantity<rad>(lon),
                                 vulcan::Quantity<rad>(lat),
                                 vulcan::Quantity<m>(alt)};

    auto r = vulcan::lla_to_ecef(lla_orig, model);
    auto lla = vulcan::ecef_to_lla(r, model);

    EXPECT_NEAR(lla.lon.value(), lla_orig.lon.value(), 1e-10);
    EXPECT_NEAR(lla.lat.value(), lla_orig.lat.value(), 1e-10);
    EXPECT_NEAR(lla.alt.value(), lla_orig.alt.value(), 1e-6);
}
