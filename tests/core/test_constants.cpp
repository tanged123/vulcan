#include <cmath>
#include <gtest/gtest.h>
#include <vulcan/core/Constants.hpp>

namespace vulcan::tests {

TEST(ConstantsTest, UniversalConstants) {
    // CODATA 2018 values
    EXPECT_DOUBLE_EQ(constants::physics::c, 299792458.0);
    EXPECT_NEAR(constants::physics::G, 6.67430e-11, 1e-16);
    EXPECT_NEAR(constants::physics::k_B, 1.380649e-23, 1e-28);
    EXPECT_NEAR(constants::physics::sigma, 5.670374e-8, 1e-13);
}

TEST(ConstantsTest, EarthConstants) {
    EXPECT_NEAR(constants::earth::mu, 3.986004418e14, 1e-1);
    EXPECT_DOUBLE_EQ(constants::earth::R_eq, 6378137.0);
    EXPECT_NEAR(constants::earth::R_pol, 6356752.3142, 1e-4);
    EXPECT_NEAR(constants::earth::f, 1.0 / 298.257223563, 1e-12);
    EXPECT_NEAR(constants::earth::J2, 1.08263e-3, 1e-8);
    EXPECT_NEAR(constants::earth::omega, 7.2921159e-5, 1e-10);
}

TEST(ConstantsTest, AtmosphereConstants) {
    EXPECT_DOUBLE_EQ(constants::atmosphere::P0, 101325.0);
    EXPECT_DOUBLE_EQ(constants::atmosphere::T0, 288.15);
    EXPECT_DOUBLE_EQ(constants::atmosphere::rho0, 1.225);
    EXPECT_NEAR(constants::atmosphere::R_air, 287.05287, 1e-5);
    EXPECT_DOUBLE_EQ(constants::atmosphere::gamma, 1.4);
}

} // namespace vulcan::tests
