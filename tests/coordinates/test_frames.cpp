#include <gtest/gtest.h>
#include <vulcan/coordinates/EarthModel.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/Units.hpp>

// ============================================
// Coordinate Frame Placeholder Tests
// ============================================
TEST(CoordinateFrames, WGS84Constants) {
    // Verify WGS84 constants are defined correctly
    EXPECT_NEAR(vulcan::constants::wgs84::a, 6378137.0, 1.0);
    EXPECT_NEAR(vulcan::constants::wgs84::f, 1.0 / 298.257223563, 1e-12);

    // Verify semi-minor axis calculation
    double b_expected =
        vulcan::constants::wgs84::a * (1.0 - vulcan::constants::wgs84::f);
    EXPECT_NEAR(vulcan::constants::wgs84::b, b_expected, 1.0);
}

TEST(CoordinateFrames, EarthConstants) {
    // Verify Earth gravitational parameter
    EXPECT_NEAR(vulcan::constants::earth::mu, 3.986004418e14, 1e8);

    // Verify Earth angular velocity
    EXPECT_NEAR(vulcan::constants::earth::omega, 7.292115e-5, 1e-10);
}

TEST(UnitConversions, AngularConversions) {
    // Degrees to radians
    double rad = vulcan::units::deg_to_rad(180.0);
    EXPECT_NEAR(rad, vulcan::constants::angle::pi, 1e-10);

    // Radians to degrees
    double deg = vulcan::units::rad_to_deg(vulcan::constants::angle::pi);
    EXPECT_NEAR(deg, 180.0, 1e-10);
}

TEST(UnitConversions, LengthConversions) {
    // Feet to meters
    double m = vulcan::units::ft_to_m(1.0);
    EXPECT_NEAR(m, 0.3048, 1e-6);

    // Nautical miles to meters
    double nm_m = vulcan::units::nm_to_m(1.0);
    EXPECT_NEAR(nm_m, 1852.0, 0.1);
}

// ============================================
// EarthModel Tests
// ============================================
TEST(EarthModel, WGS84_Constants) {
    auto model = vulcan::EarthModel::WGS84();

    // Primary parameters
    EXPECT_DOUBLE_EQ(model.a, vulcan::constants::wgs84::a);
    EXPECT_DOUBLE_EQ(model.f, vulcan::constants::wgs84::f);
    EXPECT_DOUBLE_EQ(model.omega, vulcan::constants::wgs84::omega);
    EXPECT_DOUBLE_EQ(model.mu, vulcan::constants::wgs84::mu);

    // Derived parameters
    EXPECT_NEAR(model.b, vulcan::constants::wgs84::b, 0.001);
    EXPECT_NEAR(model.e2, vulcan::constants::wgs84::e2, 1e-15);
    EXPECT_NEAR(model.e_prime2, vulcan::constants::wgs84::e_prime2, 1e-15);
}

TEST(EarthModel, Spherical_NoFlattening) {
    auto model = vulcan::EarthModel::Spherical();

    // Spherical means no flattening
    EXPECT_DOUBLE_EQ(model.f, 0.0);

    // Semi-major = semi-minor for a sphere
    EXPECT_DOUBLE_EQ(model.a, model.b);

    // Zero eccentricity for a sphere
    EXPECT_DOUBLE_EQ(model.e2, 0.0);
    EXPECT_DOUBLE_EQ(model.e_prime2, 0.0);

    // Uses mean Earth radius
    EXPECT_DOUBLE_EQ(model.a, vulcan::constants::earth::R_mean);
}

TEST(EarthModel, DerivedQuantities) {
    // Test derived quantity formulas with a custom model
    double a = 6378137.0;
    double f = 1.0 / 298.257223563;

    vulcan::EarthModel model(a, f, 7.292115e-5, 3.986004418e14);

    // b = a * (1 - f)
    double b_expected = a * (1.0 - f);
    EXPECT_NEAR(model.b, b_expected, 1e-10);

    // e² = 2f - f²
    double e2_expected = 2.0 * f - f * f;
    EXPECT_NEAR(model.e2, e2_expected, 1e-15);

    // e'² = e² / (1 - e²)
    double e_prime2_expected = e2_expected / (1.0 - e2_expected);
    EXPECT_NEAR(model.e_prime2, e_prime2_expected, 1e-15);
}

// ============================================
// Earth Rotation Model Tests
// ============================================
TEST(EarthRotation, ConstantOmega_ZeroTime) {
    auto rotation = vulcan::ConstantOmegaRotation::from_wgs84();

    // At t=0, angle should be theta0 (default 0)
    EXPECT_DOUBLE_EQ(rotation.gmst(0.0), 0.0);
}

TEST(EarthRotation, ConstantOmega_OneDay) {
    auto rotation = vulcan::ConstantOmegaRotation::from_wgs84();

    // After one sidereal day (~86164.1 seconds), should complete one rotation
    // Sidereal day = 2π / ω
    double sidereal_day =
        2.0 * vulcan::constants::angle::pi / vulcan::constants::wgs84::omega;
    double angle = rotation.gmst(sidereal_day);

    EXPECT_NEAR(angle, 2.0 * vulcan::constants::angle::pi, 1e-6);
}

TEST(EarthRotation, ConstantOmega_QuarterDay) {
    auto rotation = vulcan::ConstantOmegaRotation::from_wgs84();

    // After ~6 hours, should rotate ~90 degrees
    double quarter_sidereal_day =
        0.5 * vulcan::constants::angle::pi / vulcan::constants::wgs84::omega;
    double angle = rotation.gmst(quarter_sidereal_day);

    EXPECT_NEAR(angle, vulcan::constants::angle::pi / 2.0, 1e-6);
}

TEST(EarthRotation, ConstantOmega_WithInitialAngle) {
    double theta0 = vulcan::constants::angle::pi / 4.0; // 45 degrees
    vulcan::ConstantOmegaRotation rotation(vulcan::constants::wgs84::omega,
                                           theta0);

    // At t=0, angle should be theta0
    EXPECT_DOUBLE_EQ(rotation.gmst(0.0), theta0);

    // After some time, angle should be theta0 + omega*t
    double t = 3600.0; // 1 hour
    double expected = theta0 + vulcan::constants::wgs84::omega * t;
    EXPECT_NEAR(rotation.gmst(t), expected, 1e-10);
}

TEST(EarthRotation, GMSTRotation_Basic) {
    vulcan::GMSTRotation rotation;

    // GMST at J2000.0 (t=0) should be ~280 degrees (known value)
    // The formula gives GMST in radians, normalized to [0, 2π)
    double gmst_0 = rotation.gmst(0.0);

    // Just verify it returns a reasonable angle (0 to 2π)
    EXPECT_GE(gmst_0, 0.0);
    EXPECT_LT(gmst_0, 2.0 * vulcan::constants::angle::pi);
}

TEST(EarthRotation, PolymorphicInterface) {
    // Test that we can use EarthRotationModel polymorphically
    std::unique_ptr<vulcan::EarthRotationModel> rotation =
        std::make_unique<vulcan::ConstantOmegaRotation>(
            vulcan::constants::wgs84::omega);

    double angle = rotation->gmst(3600.0);
    double expected = vulcan::constants::wgs84::omega * 3600.0;
    EXPECT_NEAR(angle, expected, 1e-10);
}
