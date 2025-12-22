#include <cmath>
#include <gtest/gtest.h>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/Units.hpp>

namespace vulcan::tests {

using namespace vulcan::units;
using namespace vulcan::constants;

TEST(UnitsTest, AngleConversions) {
    EXPECT_NEAR(deg_to_rad(180.0), angle::pi, 1e-15);
    EXPECT_NEAR(rad_to_deg(angle::pi), 180.0, 1e-13);
    EXPECT_NEAR(deg_to_rad(45.0), angle::pi / 4.0, 1e-15);
}

TEST(UnitsTest, AngleWrapping) {
    // wrap_to_2pi
    EXPECT_NEAR(wrap_to_2pi(0.0), 0.0, 1e-15);
    EXPECT_NEAR(wrap_to_2pi(angle::pi), angle::pi, 1e-15);
    EXPECT_NEAR(wrap_to_2pi(2.0 * angle::pi), 0.0, 1e-15);
    EXPECT_NEAR(wrap_to_2pi(2.0 * angle::pi + 0.1), 0.1, 1e-15);
    EXPECT_NEAR(wrap_to_2pi(-0.1), 2.0 * angle::pi - 0.1, 1e-15);
    EXPECT_NEAR(wrap_to_2pi(-2.0 * angle::pi), 0.0, 1e-15);

    // wrap_to_pi
    EXPECT_NEAR(wrap_to_pi(0.0), 0.0, 1e-15);
    EXPECT_NEAR(wrap_to_pi(0.1), 0.1, 1e-15);
    EXPECT_NEAR(wrap_to_pi(angle::pi - 0.1), angle::pi - 0.1, 1e-15);
    // Boundary behavior depends on implementation, usually [-pi, pi) or similar
    // implementation: wrap_to_2pi(x + pi) - pi
    // wrap_to_2pi(pi + pi) = wrap_to_2pi(2pi) = 0. -> -pi.
    // So usually returns -pi for pi.
    EXPECT_NEAR(wrap_to_pi(angle::pi), -angle::pi, 1e-15);
    EXPECT_NEAR(wrap_to_pi(-angle::pi), -angle::pi, 1e-15);
    EXPECT_NEAR(wrap_to_pi(angle::pi + 0.1), -angle::pi + 0.1, 1e-15);
    EXPECT_NEAR(wrap_to_pi(3.0 * angle::pi), -angle::pi, 1e-15);

    // wrap_to_180
    EXPECT_NEAR(wrap_to_180(0.0), 0.0, 1e-15);
    EXPECT_NEAR(wrap_to_180(180.0), -180.0, 1e-15);
    EXPECT_NEAR(wrap_to_180(-180.0), -180.0, 1e-15);
    EXPECT_NEAR(wrap_to_180(190.0), -170.0, 1e-15);
    EXPECT_NEAR(wrap_to_180(-190.0), 170.0, 1e-15);
}

TEST(UnitsTest, LengthConversions) {
    EXPECT_NEAR(ft_to_m(1.0), 0.3048, 1e-15);
    EXPECT_NEAR(m_to_ft(0.3048), 1.0, 1e-15);
    EXPECT_NEAR(nm_to_m(1.0), 1852.0, 1e-15);
    EXPECT_NEAR(km_to_m(1.0), 1000.0, 1e-15);
}

TEST(UnitsTest, SpeedConversions) {
    EXPECT_NEAR(kts_to_mps(1.0), 0.514444, 1e-6);
    EXPECT_NEAR(fps_to_mps(1.0), 0.3048, 1e-15);
}

TEST(UnitsTest, PressureConversions) {
    EXPECT_NEAR(psi_to_Pa(1.0), 6894.76, 1e-1);
    EXPECT_NEAR(atm_to_Pa(1.0), 101325.0, 1e-15);
}

TEST(UnitsTest, TemperatureConversions) {
    EXPECT_NEAR(C_to_K(0.0), 273.15, 1e-15);
    EXPECT_NEAR(F_to_K(32.0), 273.15, 1e-15);
    EXPECT_NEAR(F_to_K(212.0), 373.15, 1e-15);
}

} // namespace vulcan::tests
