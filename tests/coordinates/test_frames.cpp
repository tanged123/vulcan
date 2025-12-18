#include <gtest/gtest.h>
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
