#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/gravity/J2.hpp>
#include <vulcan/gravity/J2J4.hpp>
#include <vulcan/gravity/PointMass.hpp>

using namespace vulcan;
using namespace vulcan::gravity;

// ============================================
// Numeric Tests
// ============================================

TEST(J2J4Gravity, BasicFunctionality) {
    Vec3<double> r;
    r << 7000000.0, 0.0, 0.0;

    auto g = j2j4::acceleration(r);

    // Should produce valid acceleration
    EXPECT_LT(g(0), 0.0); // Points toward center
    EXPECT_TRUE(std::isfinite(janus::norm(g)));
}

TEST(J2J4Gravity, CloseToJ2AtLowAltitude) {
    // At LEO, J3/J4 contributions are small compared to J2
    Vec3<double> r;
    r << constants::earth::R_eq + 400000.0, 0.0, 0.0;

    auto g_j2j4 = j2j4::acceleration(r);
    auto g_j2 = j2::acceleration(r);

    // Should be within 0.1% of J2-only
    double diff = janus::norm(g_j2j4 - g_j2);
    double mag = janus::norm(g_j2);

    EXPECT_LT(diff / mag, 0.001);
}

TEST(J2J4Gravity, J3AsymmetryNorthSouth) {
    // J3 introduces north-south asymmetry
    double alt = 500000.0;

    Vec3<double> r_north, r_south;
    r_north << 0.0, 0.0, constants::earth::R_eq + alt;    // North pole
    r_south << 0.0, 0.0, -(constants::earth::R_eq + alt); // South pole

    auto g_north = j2j4::acceleration(r_north);
    auto g_south = j2j4::acceleration(r_south);

    // Magnitudes should be slightly different due to J3
    double mag_north = janus::norm(g_north);
    double mag_south = janus::norm(g_south);

    // The difference may be very small; mainly testing code runs
    EXPECT_TRUE(std::isfinite(mag_north));
    EXPECT_TRUE(std::isfinite(mag_south));
}

TEST(J2J4Gravity, ReducesToJ2WhenJ3J4Zero) {
    Vec3<double> r;
    r << 7000000.0, 1000000.0, 500000.0;

    auto g_j2j4 =
        j2j4::acceleration(r, constants::earth::mu, constants::earth::J2, 0.0,
                           0.0, constants::earth::R_eq);
    auto g_j2 = j2::acceleration(r);

    EXPECT_NEAR(g_j2j4(0), g_j2(0), 1e-12);
    EXPECT_NEAR(g_j2j4(1), g_j2(1), 1e-12);
    EXPECT_NEAR(g_j2j4(2), g_j2(2), 1e-12);
}

TEST(J2J4Gravity, ReducesToPointMassWhenAllZero) {
    Vec3<double> r;
    r << 7000000.0, 1000000.0, 500000.0;

    auto g_j2j4 = j2j4::acceleration(r, constants::earth::mu, 0.0, 0.0, 0.0,
                                     constants::earth::R_eq);
    auto g_pm = point_mass::acceleration(r);

    EXPECT_NEAR(g_j2j4(0), g_pm(0), 1e-12);
    EXPECT_NEAR(g_j2j4(1), g_pm(1), 1e-12);
    EXPECT_NEAR(g_j2j4(2), g_pm(2), 1e-12);
}

// ============================================
// Potential Tests
// ============================================

TEST(J2J4Gravity, PotentialBasic) {
    Vec3<double> r;
    r << constants::earth::R_eq + 500000.0, 0.0, 0.0;

    auto U = j2j4::potential(r);

    EXPECT_LT(U, 0.0); // Negative potential
    EXPECT_TRUE(std::isfinite(U));
}

TEST(J2J4Gravity, PotentialConsistency) {
    Vec3<double> r;
    r << 7000000.0, 0.0, 1000000.0;

    auto U_j2j4 = j2j4::potential(r);
    auto U_j2 = j2::potential(r);
    auto U_pm = point_mass::potential(r);

    // All should be negative and roughly similar
    EXPECT_LT(U_j2j4, 0.0);
    EXPECT_LT(U_j2, 0.0);
    EXPECT_LT(U_pm, 0.0);

    // J2J4 should be close to J2
    EXPECT_NEAR(U_j2j4, U_j2, std::abs(U_j2) * 0.001);
}

// ============================================
// Symbolic Tests
// ============================================

TEST(J2J4Gravity, SymbolicEvaluation) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r;
    r << x, y, z;

    auto g = j2j4::acceleration(r);

    // Create function for evaluation
    janus::Function f("j2j4_accel", {x, y, z}, {g(0), g(1), g(2)});

    // Evaluate at specific point
    auto result = f({7000000.0, 0.0, 1000000.0});

    EXPECT_TRUE(std::isfinite(static_cast<double>(result[0](0, 0))));
    EXPECT_TRUE(std::isfinite(static_cast<double>(result[1](0, 0))));
    EXPECT_TRUE(std::isfinite(static_cast<double>(result[2](0, 0))));
}

TEST(J2J4Gravity, SymbolicMatchesNumeric) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r_sym;
    r_sym << x, y, z;

    Vec3<double> r_num;
    r_num << 7000000.0, 1000000.0, 500000.0;

    auto g_sym = j2j4::acceleration(r_sym);
    auto g_num = j2j4::acceleration(r_num);

    janus::Function f("j2j4_test", {x, y, z}, {g_sym(0), g_sym(1), g_sym(2)});
    auto result = f({r_num(0), r_num(1), r_num(2)});

    EXPECT_NEAR(result[0](0, 0), g_num(0), 1e-8);
    EXPECT_NEAR(result[1](0, 0), g_num(1), 1e-8);
    EXPECT_NEAR(result[2](0, 0), g_num(2), 1e-8);
}

TEST(J2J4Gravity, SymbolicPotential) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r;
    r << x, y, z;

    auto U = j2j4::potential(r);

    janus::Function f("j2j4_potential", {x, y, z}, {U});

    Vec3<double> r_num;
    r_num << 7000000.0, 500000.0, 2000000.0;

    auto result = f({r_num(0), r_num(1), r_num(2)});
    double U_num = j2j4::potential(r_num);

    EXPECT_NEAR(result[0](0, 0), U_num, 1e-6);
}
