#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/gravity/PointMass.hpp>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/QuantityEigen.hpp>
#include <vulcan/quantity/Units.hpp>

using namespace vulcan;
using namespace vulcan::gravity;
using namespace vulcan::units;

// Helper: create a Vec3<Quantity<m, Scalar>> from three raw values
template <typename Scalar>
Vec3<Quantity<m, Scalar>> make_pos(Scalar x, Scalar y, Scalar z) {
    Vec3<Quantity<m, Scalar>> v;
    v(0) = Quantity<m, Scalar>{x};
    v(1) = Quantity<m, Scalar>{y};
    v(2) = Quantity<m, Scalar>{z};
    return v;
}

// ============================================
// Numeric Tests
// ============================================

TEST(PointMassGravity, SurfaceGravity) {
    // At Earth's surface, gravity should be approximately 9.8 m/s^2
    auto r_surface = make_pos(constants::earth::R_eq.value(), 0.0, 0.0);

    auto g = point_mass::acceleration(r_surface);
    double g_mag =
        std::sqrt(g(0).value() * g(0).value() + g(1).value() * g(1).value() +
                  g(2).value() * g(2).value());

    EXPECT_NEAR(g_mag, 9.8, 0.1); // ~9.8 m/s^2 at surface
}

TEST(PointMassGravity, DirectionTowardCenter) {
    // Gravity should point toward Earth's center
    auto r = make_pos(7000000.0, 1000000.0, 500000.0);

    auto g = point_mass::acceleration(r);

    // g should be anti-parallel to r
    double dot = r(0).value() * g(0).value() + r(1).value() * g(1).value() +
                 r(2).value() * g(2).value();
    EXPECT_LT(dot, 0.0); // Opposite direction
}

TEST(PointMassGravity, InverseSquareLaw) {
    // Double the distance -> quarter the gravity
    auto r1 = make_pos(constants::earth::R_eq.value(), 0.0, 0.0);
    auto r2 = make_pos(2.0 * constants::earth::R_eq.value(), 0.0, 0.0);

    auto g1 = point_mass::acceleration(r1);
    auto g2 = point_mass::acceleration(r2);

    double g1_mag = std::sqrt(g1(0).value() * g1(0).value() +
                              g1(1).value() * g1(1).value() +
                              g1(2).value() * g1(2).value());
    double g2_mag = std::sqrt(g2(0).value() * g2(0).value() +
                              g2(1).value() * g2(1).value() +
                              g2(2).value() * g2(2).value());

    EXPECT_NEAR(g1_mag / g2_mag, 4.0, 1e-10);
}

TEST(PointMassGravity, LEOGravity) {
    // At 400 km altitude (ISS), gravity should be ~8.7 m/s^2
    auto r = make_pos(constants::earth::R_eq.value() + 400000.0, 0.0, 0.0);

    auto g = point_mass::acceleration(r);
    double g_mag =
        std::sqrt(g(0).value() * g(0).value() + g(1).value() * g(1).value() +
                  g(2).value() * g(2).value());
    EXPECT_NEAR(g_mag, 8.7, 0.1);
}

TEST(PointMassGravity, GEOGravity) {
    // At GEO (35786 km), gravity should be ~0.224 m/s^2
    auto r = make_pos(constants::earth::R_eq.value() + 35786000.0, 0.0, 0.0);

    auto g = point_mass::acceleration(r);
    double g_mag =
        std::sqrt(g(0).value() * g(0).value() + g(1).value() * g(1).value() +
                  g(2).value() * g(2).value());
    EXPECT_NEAR(g_mag, 0.224, 0.01);
}

TEST(PointMassGravity, Potential) {
    auto r = make_pos(constants::earth::R_eq.value(), 0.0, 0.0);

    auto U = point_mass::potential(r);

    // U = -mu/r, should be negative
    EXPECT_LT(U.value(), 0.0);
    EXPECT_NEAR(U.value(),
                -constants::earth::mu.value() / constants::earth::R_eq.value(),
                1e6);
}

TEST(PointMassGravity, AccelerationMagnitude) {
    Quantity<m, double> r_mag{constants::earth::R_eq.value()};
    auto g = point_mass::acceleration_magnitude(r_mag);

    EXPECT_NEAR(g.value(), 9.8, 0.1);
}

// ============================================
// Symbolic Tests
// ============================================

TEST(PointMassGravity, SymbolicEvaluation) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    auto r = make_pos(x, y, z);

    auto g = point_mass::acceleration(r);

    // Create function to evaluate
    janus::Function f("point_mass_accel", {x, y, z},
                      {g(0).value(), g(1).value(), g(2).value()});

    // Evaluate at Earth's equatorial surface
    double R = constants::earth::R_eq.value();
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

    auto r_sym = make_pos(x, y, z);
    auto r_num = make_pos(7000000.0, 1000000.0, 500000.0);

    auto g_sym = point_mass::acceleration(r_sym);
    auto g_num = point_mass::acceleration(r_num);

    janus::Function f("pm_accel", {x, y, z},
                      {g_sym(0).value(), g_sym(1).value(), g_sym(2).value()});
    auto result = f({7000000.0, 1000000.0, 500000.0});

    EXPECT_NEAR(result[0](0, 0), g_num(0).value(), 1e-10);
    EXPECT_NEAR(result[1](0, 0), g_num(1).value(), 1e-10);
    EXPECT_NEAR(result[2](0, 0), g_num(2).value(), 1e-10);
}

TEST(PointMassGravity, SymbolicPotential) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    auto r = make_pos(x, y, z);

    auto U = point_mass::potential(r);

    janus::Function f("pm_potential", {x, y, z}, {U.value()});

    auto r_num = make_pos(7000000.0, 1000000.0, 500000.0);

    auto result = f({7000000.0, 1000000.0, 500000.0});
    double U_num = point_mass::potential(r_num).value();

    EXPECT_NEAR(result[0](0, 0), U_num, 1e-6);
}

// ============================================
// Constants Verification
// ============================================

TEST(GravityConstants, PhysicsConstants) {
    EXPECT_NEAR(constants::physics::G, 6.67430e-11, 1e-14);
    EXPECT_NEAR(constants::physics::g0.value(), 9.80665, 1e-5);
}

TEST(GravityConstants, EarthConstants) {
    EXPECT_NEAR(constants::earth::mu.value(), 3.986004418e14, 1e8);
    EXPECT_NEAR(constants::earth::R_eq.value(), 6378137.0, 1.0);
    EXPECT_NEAR(constants::earth::J2.value(), 1.08263e-3, 1e-8);
    EXPECT_NEAR(constants::earth::J3.value(), -2.54e-6, 1e-10);
    EXPECT_NEAR(constants::earth::J4.value(), -1.61e-6, 1e-10);
}
