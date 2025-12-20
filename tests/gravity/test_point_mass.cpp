#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/gravity/PointMass.hpp>

using namespace vulcan;
using namespace vulcan::gravity;

// ============================================
// Numeric Tests
// ============================================

TEST(PointMassGravity, SurfaceGravity) {
    // At Earth's surface, gravity should be approximately 9.8 m/s²
    Vec3<double> r_surface;
    r_surface << constants::earth::R_eq, 0.0, 0.0;

    auto g = point_mass::acceleration(r_surface);
    double g_mag = janus::norm(g);

    EXPECT_NEAR(g_mag, 9.8, 0.1); // ~9.8 m/s² at surface
}

TEST(PointMassGravity, DirectionTowardCenter) {
    // Gravity should point toward Earth's center
    Vec3<double> r;
    r << 7000000.0, 1000000.0, 500000.0;

    auto g = point_mass::acceleration(r);

    // g should be anti-parallel to r
    double dot = r(0) * g(0) + r(1) * g(1) + r(2) * g(2);
    EXPECT_LT(dot, 0.0); // Opposite direction
}

TEST(PointMassGravity, InverseSquareLaw) {
    // Double the distance -> quarter the gravity
    Vec3<double> r1, r2;
    r1 << constants::earth::R_eq, 0.0, 0.0;
    r2 << 2.0 * constants::earth::R_eq, 0.0, 0.0;

    auto g1 = point_mass::acceleration(r1);
    auto g2 = point_mass::acceleration(r2);

    double g1_mag = janus::norm(g1);
    double g2_mag = janus::norm(g2);

    EXPECT_NEAR(g1_mag / g2_mag, 4.0, 1e-10);
}

TEST(PointMassGravity, LEOGravity) {
    // At 400 km altitude (ISS), gravity should be ~8.7 m/s²
    Vec3<double> r;
    r << constants::earth::R_eq + 400000.0, 0.0, 0.0;

    auto g = point_mass::acceleration(r);
    EXPECT_NEAR(janus::norm(g), 8.7, 0.1);
}

TEST(PointMassGravity, GEOGravity) {
    // At GEO (35786 km), gravity should be ~0.224 m/s²
    Vec3<double> r;
    r << constants::earth::R_eq + 35786000.0, 0.0, 0.0;

    auto g = point_mass::acceleration(r);
    EXPECT_NEAR(janus::norm(g), 0.224, 0.01);
}

TEST(PointMassGravity, Potential) {
    Vec3<double> r;
    r << constants::earth::R_eq, 0.0, 0.0;

    auto U = point_mass::potential(r);

    // U = -μ/r, should be negative
    EXPECT_LT(U, 0.0);
    EXPECT_NEAR(U, -constants::earth::mu / constants::earth::R_eq, 1e6);
}

TEST(PointMassGravity, AccelerationMagnitude) {
    double r = constants::earth::R_eq;
    double g = point_mass::acceleration_magnitude(r);

    EXPECT_NEAR(g, 9.8, 0.1);
}

// ============================================
// Symbolic Tests
// ============================================

TEST(PointMassGravity, SymbolicEvaluation) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r;
    r << x, y, z;

    auto g = point_mass::acceleration(r);

    // Create function to evaluate
    janus::Function f("point_mass_accel", {x, y, z}, {g(0), g(1), g(2)});

    // Evaluate at Earth's equatorial surface
    double R = constants::earth::R_eq;
    auto result = f({R, 0.0, 0.0});

    // Should point toward center (negative x direction)
    EXPECT_LT(result[0](0, 0), 0.0);
    EXPECT_NEAR(result[1](0, 0), 0.0, 1e-10);
    EXPECT_NEAR(result[2](0, 0), 0.0, 1e-10);
}

TEST(PointMassGravity, SymbolicMatchesNumeric) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r_sym;
    r_sym << x, y, z;

    Vec3<double> r_num;
    r_num << 7000000.0, 1000000.0, 500000.0;

    auto g_sym = point_mass::acceleration(r_sym);
    auto g_num = point_mass::acceleration(r_num);

    janus::Function f("pm_accel", {x, y, z}, {g_sym(0), g_sym(1), g_sym(2)});
    auto result = f({r_num(0), r_num(1), r_num(2)});

    EXPECT_NEAR(result[0](0, 0), g_num(0), 1e-10);
    EXPECT_NEAR(result[1](0, 0), g_num(1), 1e-10);
    EXPECT_NEAR(result[2](0, 0), g_num(2), 1e-10);
}

TEST(PointMassGravity, SymbolicPotential) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r;
    r << x, y, z;

    auto U = point_mass::potential(r);

    janus::Function f("pm_potential", {x, y, z}, {U});

    Vec3<double> r_num;
    r_num << 7000000.0, 1000000.0, 500000.0;

    auto result = f({r_num(0), r_num(1), r_num(2)});
    double U_num = point_mass::potential(r_num);

    EXPECT_NEAR(result[0](0, 0), U_num, 1e-6);
}

// ============================================
// Constants Verification
// ============================================

TEST(GravityConstants, PhysicsConstants) {
    EXPECT_NEAR(constants::physics::G, 6.67430e-11, 1e-14);
    EXPECT_NEAR(constants::physics::g0, 9.80665, 1e-5);
}

TEST(GravityConstants, EarthConstants) {
    EXPECT_NEAR(constants::earth::mu, 3.986004418e14, 1e8);
    EXPECT_NEAR(constants::earth::R_eq, 6378137.0, 1.0);
    EXPECT_NEAR(constants::earth::J2, 1.08263e-3, 1e-8);
    EXPECT_NEAR(constants::earth::J3, -2.54e-6, 1e-10);
    EXPECT_NEAR(constants::earth::J4, -1.61e-6, 1e-10);
}
