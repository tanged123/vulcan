// Solar Position Tests
// Verify solar ephemeris accuracy against reference values

#include <cmath>
#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/environment/SolarPosition.hpp>
#include <vulcan/time/JulianDate.hpp>

using namespace vulcan::environment;

// =============================================================================
// Numeric Tests
// =============================================================================

// Test at J2000.0 epoch - well-known reference point
TEST(SolarPosition, J2000Reference) {
    // J2000.0 = 2000-01-01 12:00:00 TT
    double jd = vulcan::time::calendar_to_jd(2000, 1, 1, 12, 0, 0.0);

    auto [ra, dec] = solar::ra_dec(jd);

    // At J2000.0 (early January), Sun is in Capricorn/Sagittarius
    // RA ≈ 281° (18h 45m), Dec ≈ -23° (near winter solstice)
    double ra_deg = ra * vulcan::constants::angle::rad2deg;
    double dec_deg = dec * vulcan::constants::angle::rad2deg;

    // Normalize RA to [0, 360)
    if (ra_deg < 0)
        ra_deg += 360.0;

    EXPECT_NEAR(ra_deg, 281.3, 1.0);  // Within 1 degree
    EXPECT_NEAR(dec_deg, -23.0, 1.0); // Near winter solstice
}

// March equinox - declination should be near zero
TEST(SolarPosition, VernalEquinox2024) {
    // March 20, 2024 ~03:06 UTC (vernal equinox)
    double jd = vulcan::time::calendar_to_jd(2024, 3, 20, 3, 6, 0.0);

    auto [ra, dec] = solar::ra_dec(jd);
    double dec_deg = dec * vulcan::constants::angle::rad2deg;

    // At equinox, declination should be very close to zero
    EXPECT_NEAR(dec_deg, 0.0, 1.0);
}

// Summer solstice - maximum declination
TEST(SolarPosition, SummerSolstice2024) {
    // June 20, 2024 ~20:50 UTC (summer solstice)
    double jd = vulcan::time::calendar_to_jd(2024, 6, 20, 20, 50, 0.0);

    auto [ra, dec] = solar::ra_dec(jd);
    double dec_deg = dec * vulcan::constants::angle::rad2deg;

    // Maximum declination ~23.44°
    EXPECT_NEAR(dec_deg, 23.44, 1.0);
}

// Winter solstice - minimum declination
TEST(SolarPosition, WinterSolstice2024) {
    // December 21, 2024 ~09:21 UTC (winter solstice)
    double jd = vulcan::time::calendar_to_jd(2024, 12, 21, 9, 21, 0.0);

    auto [ra, dec] = solar::ra_dec(jd);
    double dec_deg = dec * vulcan::constants::angle::rad2deg;

    // Minimum declination ~-23.44°
    EXPECT_NEAR(dec_deg, -23.44, 1.0);
}

// Distance sanity check over a year
TEST(SolarPosition, DistanceRange) {
    constexpr double AU = 149597870700.0;

    // Perihelion (early January) - closest
    double jd_perihelion = vulcan::time::calendar_to_jd(2024, 1, 3, 0, 0, 0.0);
    double dist_peri = solar::distance(jd_perihelion);
    EXPECT_GT(dist_peri, 0.98 * AU);
    EXPECT_LT(dist_peri, 0.985 * AU);

    // Aphelion (early July) - farthest
    double jd_aphelion = vulcan::time::calendar_to_jd(2024, 7, 5, 0, 0, 0.0);
    double dist_aph = solar::distance(jd_aphelion);
    EXPECT_GT(dist_aph, 1.015 * AU);
    EXPECT_LT(dist_aph, 1.02 * AU);
}

// Position vector magnitude should match distance
TEST(SolarPosition, PositionMagnitude) {
    double jd = vulcan::time::calendar_to_jd(2024, 6, 15, 12, 0, 0.0);

    auto pos = solar::position_eci(jd);
    double computed_distance =
        std::sqrt(pos(0) * pos(0) + pos(1) * pos(1) + pos(2) * pos(2));
    double direct_distance = solar::distance(jd);

    EXPECT_NEAR(computed_distance, direct_distance, 1e6); // Within 1km
}

// Unit vector should have magnitude 1
TEST(SolarPosition, UnitVectorMagnitude) {
    double jd = vulcan::time::calendar_to_jd(2024, 9, 22, 12, 0, 0.0);

    auto u = solar::unit_vector_eci(jd);
    double mag = std::sqrt(u(0) * u(0) + u(1) * u(1) + u(2) * u(2));

    EXPECT_NEAR(mag, 1.0, 1e-10);
}

// =============================================================================
// Symbolic Tests
// =============================================================================

TEST(SolarPosition, SymbolicEvaluation) {
    auto jd = janus::sym("jd");

    // Should be able to create symbolic expressions
    auto pos = solar::position_eci(jd);
    auto dist = solar::distance(jd);

    // Create function for evaluation
    janus::Function f("solar_pos", {jd}, {pos(0), pos(1), pos(2), dist});

    // Evaluate at J2000
    double jd_val = vulcan::time::calendar_to_jd(2000, 1, 1, 12, 0, 0.0);
    auto result = f({jd_val});

    double x = result[0](0, 0);
    double y = result[1](0, 0);
    double z = result[2](0, 0);
    double r = result[3](0, 0);

    double computed_r = std::sqrt(x * x + y * y + z * z);
    EXPECT_NEAR(computed_r, r, 1e6);

    // Early January - distance should be near perihelion (~147 million km)
    EXPECT_NEAR(r, 147.1e9, 1e9);
}

TEST(SolarPosition, SymbolicGradient) {
    auto jd = janus::sym("jd");

    auto dec = solar::declination(jd);

    // Compute derivative of declination w.r.t. Julian date
    auto ddec_djd = janus::jacobian(dec, jd);

    // Create function for evaluation
    janus::Function f("dec_gradient", {jd}, {ddec_djd});

    // Evaluate at J2000
    double jd_val = vulcan::time::calendar_to_jd(2000, 1, 1, 12, 0, 0.0);
    auto result = f({jd_val});
    double grad = result[0](0, 0);

    // In early January, declination is increasing (moving toward spring)
    EXPECT_GT(grad, 0.0);
}
