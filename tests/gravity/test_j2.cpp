#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/gravity/J2.hpp>
#include <vulcan/gravity/PointMass.hpp>

using namespace vulcan;
using namespace vulcan::gravity;

// ============================================
// Numeric Tests
// ============================================

TEST(J2Gravity, EquatorialSurfaceJ2Effect) {
    // At equator, J2 modifies gravity compared to point mass
    // The J2 term accounts for Earth's oblateness
    Vec3<double> r_eq;
    r_eq << constants::earth::R_eq, 0.0, 0.0;

    auto g_j2 = j2::acceleration(r_eq);
    auto g_pm = point_mass::acceleration(r_eq);

    // Both should point toward center and have similar magnitude
    EXPECT_LT(g_j2(0), 0.0);
    EXPECT_LT(g_pm(0), 0.0);
    // Difference should be small (within a few percent)
    EXPECT_NEAR(janus::norm(g_j2), janus::norm(g_pm), janus::norm(g_pm) * 0.01);
}

TEST(J2Gravity, PolarSurfaceJ2Effect) {
    // At poles, J2 modifies gravity compared to point mass
    Vec3<double> r_pole;
    r_pole << 0.0, 0.0, constants::earth::R_pol;

    auto g_j2 = j2::acceleration(r_pole);
    auto g_pm = point_mass::acceleration(r_pole);

    // Both should point toward center and have similar magnitude
    EXPECT_LT(g_j2(2), 0.0);
    EXPECT_LT(g_pm(2), 0.0);
    // Difference should be small
    EXPECT_NEAR(janus::norm(g_j2), janus::norm(g_pm), janus::norm(g_pm) * 0.02);
}

TEST(J2Gravity, ReferenceValueLEO) {
    // At 400 km altitude on equator
    Vec3<double> r;
    r << constants::earth::R_eq + 400000.0, 0.0, 0.0;

    auto g = j2::acceleration(r);
    double g_mag = janus::norm(g);

    // Expected ~8.7 m/s² at 400 km
    EXPECT_NEAR(g_mag, 8.7, 0.1);
}

TEST(J2Gravity, DirectionTowardCenter) {
    Vec3<double> r;
    r << constants::earth::R_eq + 500000.0, 0.0, 0.0;

    auto g = j2::acceleration(r);

    // On equator, gravity should still point mainly toward center
    EXPECT_LT(g(0), 0.0); // Negative x component
    EXPECT_NEAR(g(1), 0.0, 1e-10);
    EXPECT_NEAR(g(2), 0.0, 1e-10);
}

TEST(J2Gravity, EquatorPoleAsymmetry) {
    double alt = 500000.0; // 500 km altitude

    Vec3<double> r_eq, r_pole;
    r_eq << constants::earth::R_eq + alt, 0.0, 0.0;
    r_pole << 0.0, 0.0, constants::earth::R_pol + alt;

    auto g_eq = j2::acceleration(r_eq);
    auto g_pole = j2::acceleration(r_pole);

    // Polar gravity should be stronger due to closer distance and J2
    EXPECT_GT(janus::norm(g_pole), janus::norm(g_eq));
}

// ============================================
// Consistency Tests
// ============================================

TEST(J2Gravity, ReducesToPointMassWhenJ2Zero) {
    // With J2=0, J2 model should equal point mass
    Vec3<double> r;
    r << 7000000.0, 1000000.0, 500000.0;

    auto g_j2 =
        j2::acceleration(r, constants::earth::mu, 0.0, constants::earth::R_eq);
    auto g_pm = point_mass::acceleration(r);

    EXPECT_NEAR(g_j2(0), g_pm(0), 1e-12);
    EXPECT_NEAR(g_j2(1), g_pm(1), 1e-12);
    EXPECT_NEAR(g_j2(2), g_pm(2), 1e-12);
}

TEST(J2Gravity, Potential) {
    Vec3<double> r;
    r << constants::earth::R_eq, 0.0, 0.0;

    auto U_j2 = j2::potential(r);
    auto U_pm = point_mass::potential(r);

    // Both should be negative
    EXPECT_LT(U_j2, 0.0);
    EXPECT_LT(U_pm, 0.0);

    // J2 correction is relatively small
    EXPECT_NEAR(U_j2, U_pm, std::abs(U_pm) * 0.01); // Within 1%
}

// ============================================
// Symbolic Tests
// ============================================

TEST(J2Gravity, SymbolicEvaluation) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r;
    r << x, y, z;

    auto g = j2::acceleration(r);

    // Create function for evaluation
    janus::Function f("j2_accel", {x, y, z}, {g(0), g(1), g(2)});

    // Evaluate at specific point
    double R = constants::earth::R_eq;
    auto result = f({R, 0.0, 0.0});

    // Should point toward Earth center
    EXPECT_LT(result[0](0, 0), 0.0);
    EXPECT_NEAR(result[1](0, 0), 0.0, 1e-10);
    EXPECT_NEAR(result[2](0, 0), 0.0, 1e-10);
}

TEST(J2Gravity, SymbolicMatchesNumeric) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r_sym;
    r_sym << x, y, z;

    Vec3<double> r_num;
    r_num << 7000000.0, 1000000.0, 500000.0;

    auto g_sym = j2::acceleration(r_sym);
    auto g_num = j2::acceleration(r_num);

    janus::Function f("j2_accel_test", {x, y, z},
                      {g_sym(0), g_sym(1), g_sym(2)});
    auto result = f({r_num(0), r_num(1), r_num(2)});

    EXPECT_NEAR(result[0](0, 0), g_num(0), 1e-10);
    EXPECT_NEAR(result[1](0, 0), g_num(1), 1e-10);
    EXPECT_NEAR(result[2](0, 0), g_num(2), 1e-10);
}

TEST(J2Gravity, SymbolicPotential) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r;
    r << x, y, z;

    auto U = j2::potential(r);

    janus::Function f("j2_potential", {x, y, z}, {U});

    Vec3<double> r_num;
    r_num << 7000000.0, 0.0, 1000000.0;

    auto result = f({r_num(0), r_num(1), r_num(2)});
    double U_num = j2::potential(r_num);

    EXPECT_NEAR(result[0](0, 0), U_num, 1e-6);
}
