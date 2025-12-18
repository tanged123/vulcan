#include <gtest/gtest.h>
#include <vulcan/core/Constants.hpp>

// ============================================
// Gravity Placeholder Tests
// ============================================
TEST(Gravity, PointMassConstants) {
    // Verify gravitational constant
    EXPECT_NEAR(vulcan::constants::physics::G, 6.67430e-11, 1e-14);

    // Verify standard gravity
    EXPECT_NEAR(vulcan::constants::physics::g0, 9.80665, 1e-5);
}

TEST(Gravity, EarthGravityConstants) {
    // Verify Earth mu
    EXPECT_NEAR(vulcan::constants::earth::mu, 3.986004418e14, 1e8);

    // Verify J2 coefficient
    EXPECT_NEAR(vulcan::constants::earth::J2, 1.08263e-3, 1e-8);
}
