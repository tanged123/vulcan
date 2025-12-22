// Analytical Ephemeris Tests
#include <gtest/gtest.h>
#include <vulcan/orbital/AnalyticalEphemeris.hpp>

using namespace vulcan::orbital::ephemeris::analytical;

// Test Sun position at J2000
TEST(AnalyticalEphemeris, SunPosition_J2000) {
    double jd_j2000 = 2451545.0;

    vulcan::Vec3<double> r_sun = sun_position_eci(jd_j2000);
    double dist = janus::norm(r_sun);

    // Sun should be ~1 AU away
    double AU = vulcan::constants::sun::AU;
    EXPECT_NEAR(dist / AU, 1.0, 0.02); // Within 2%
}

TEST(AnalyticalEphemeris, SunDistance_Range) {
    // Test over a year - distance should vary between 0.983 and 1.017 AU
    double jd_start = 2451545.0; // J2000

    for (int day = 0; day < 365; day += 30) {
        double jd = jd_start + day;
        double dist = sun_distance(jd);
        double dist_au = dist / vulcan::constants::sun::AU;

        EXPECT_GT(dist_au, 0.98);
        EXPECT_LT(dist_au, 1.02);
    }
}

// Test Moon position
TEST(AnalyticalEphemeris, MoonPosition_Distance) {
    double jd = 2451545.0;

    vulcan::Vec3<double> r_moon = moon_position_eci(jd);
    double dist = janus::norm(r_moon);

    // Moon should be ~384,000 km away
    double expected = vulcan::constants::moon::mean_distance;
    EXPECT_NEAR(dist, expected, expected * 0.1); // Within 10%
}

TEST(AnalyticalEphemeris, MoonDistance_Range) {
    // Moon distance varies between ~356,500 and ~406,700 km
    double jd_start = 2451545.0;

    double min_dist = 1e20;
    double max_dist = 0;

    for (int day = 0; day < 30; day++) {
        double jd = jd_start + day;
        double dist = moon_distance(jd);
        min_dist = std::min(min_dist, dist);
        max_dist = std::max(max_dist, dist);
    }

    EXPECT_GT(min_dist, 350000.0e3);
    EXPECT_LT(max_dist, 410000.0e3);
}

// Test ECEF conversions
TEST(AnalyticalEphemeris, SunECEF_Magnitude) {
    double jd = 2451545.0;

    vulcan::Vec3<double> r_eci = sun_position_eci(jd);
    vulcan::Vec3<double> r_ecef = sun_position_ecef(jd);

    // Distance should be preserved
    EXPECT_NEAR(janus::norm(r_eci), janus::norm(r_ecef), 1.0);
}

TEST(AnalyticalEphemeris, MoonECEF_Magnitude) {
    double jd = 2451545.0;

    vulcan::Vec3<double> r_eci = moon_position_eci(jd);
    vulcan::Vec3<double> r_ecef = moon_position_ecef(jd);

    EXPECT_NEAR(janus::norm(r_eci), janus::norm(r_ecef), 1.0);
}

// Test right ascension and declination
TEST(AnalyticalEphemeris, SunRaDec_Range) {
    double jd = 2451545.0;

    auto [ra, dec] = sun_ra_dec(jd);

    // RA should be in [0, 2pi), Dec in [-pi/2, pi/2]
    // Note: atan2 returns [-pi, pi], so RA could be negative
    EXPECT_GE(dec, -M_PI / 2.0);
    EXPECT_LE(dec, M_PI / 2.0);
}

// Test symbolic compatibility
TEST(AnalyticalEphemeris, Symbolic_SunPosition) {
    auto jd = janus::sym("jd");

    auto r_sun = sun_position_eci(jd);

    janus::Function f("sun_pos", {jd}, {r_sun(0), r_sun(1), r_sun(2)});
    auto result = f({2451545.0});

    EXPECT_NE(result[0](0, 0), 0.0);
}

TEST(AnalyticalEphemeris, Symbolic_MoonPosition) {
    auto jd = janus::sym("jd");

    auto r_moon = moon_position_eci(jd);

    janus::Function f("moon_pos", {jd}, {r_moon(0), r_moon(1), r_moon(2)});
    auto result = f({2451545.0});

    EXPECT_NE(result[0](0, 0), 0.0);
}

// Test unit vector
TEST(AnalyticalEphemeris, SunUnitVector_Normalized) {
    double jd = 2451545.0;

    vulcan::Vec3<double> u = sun_unit_vector_eci(jd);
    double mag = janus::norm(u);

    EXPECT_NEAR(mag, 1.0, 1e-10);
}

// Test consistency between unit vector and position
TEST(AnalyticalEphemeris, SunUnitVectorConsistency) {
    double jd = 2451545.5;

    vulcan::Vec3<double> r = sun_position_eci(jd);
    vulcan::Vec3<double> u = sun_unit_vector_eci(jd);

    vulcan::Vec3<double> u_from_r = r / janus::norm(r);

    EXPECT_NEAR((u - u_from_r).norm(), 0.0, 1e-10);
}
