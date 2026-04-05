#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/gravity/J2.hpp>
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

// Helper: compute magnitude from Vec3<Quantity<accel_unit>>
template <auto U> double qvec_mag(const Vec3<Quantity<U, double>> &v) {
    double x = v(0).value(), y = v(1).value(), z = v(2).value();
    return std::sqrt(x * x + y * y + z * z);
}

// ============================================
// Numeric Tests
// ============================================

TEST(J2Gravity, EquatorialSurfaceJ2Effect) {
    // At equator, J2 modifies gravity compared to point mass
    auto r_eq = make_pos(constants::earth::R_eq.value(), 0.0, 0.0);

    auto g_j2 = j2::acceleration(r_eq);
    auto g_pm = point_mass::acceleration(r_eq);

    // Both should point toward center and have similar magnitude
    EXPECT_LT(g_j2(0).value(), 0.0);
    EXPECT_LT(g_pm(0).value(), 0.0);
    // Difference should be small (within a few percent)
    EXPECT_NEAR(qvec_mag(g_j2), qvec_mag(g_pm), qvec_mag(g_pm) * 0.01);
}

TEST(J2Gravity, PolarSurfaceJ2Effect) {
    // At poles, J2 modifies gravity compared to point mass
    auto r_pole = make_pos(0.0, 0.0, constants::earth::R_pol.value());

    auto g_j2 = j2::acceleration(r_pole);
    auto g_pm = point_mass::acceleration(r_pole);

    // Both should point toward center and have similar magnitude
    EXPECT_LT(g_j2(2).value(), 0.0);
    EXPECT_LT(g_pm(2).value(), 0.0);
    // Difference should be small
    EXPECT_NEAR(qvec_mag(g_j2), qvec_mag(g_pm), qvec_mag(g_pm) * 0.02);
}

TEST(J2Gravity, ReferenceValueLEO) {
    // At 400 km altitude on equator
    auto r = make_pos(constants::earth::R_eq.value() + 400000.0, 0.0, 0.0);

    auto g = j2::acceleration(r);
    double g_mag = qvec_mag(g);

    // Expected ~8.7 m/s^2 at 400 km
    EXPECT_NEAR(g_mag, 8.7, 0.1);
}

TEST(J2Gravity, DirectionTowardCenter) {
    auto r = make_pos(constants::earth::R_eq.value() + 500000.0, 0.0, 0.0);

    auto g = j2::acceleration(r);

    // On equator, gravity should still point mainly toward center
    EXPECT_LT(g(0).value(), 0.0); // Negative x component
    EXPECT_NEAR(g(1).value(), 0.0, 1e-10);
    EXPECT_NEAR(g(2).value(), 0.0, 1e-10);
}

TEST(J2Gravity, EquatorPoleAsymmetry) {
    double alt = 500000.0; // 500 km altitude

    auto r_eq = make_pos(constants::earth::R_eq.value() + alt, 0.0, 0.0);
    auto r_pole = make_pos(0.0, 0.0, constants::earth::R_pol.value() + alt);

    auto g_eq = j2::acceleration(r_eq);
    auto g_pole = j2::acceleration(r_pole);

    // Polar gravity should be stronger due to closer distance and J2
    EXPECT_GT(qvec_mag(g_pole), qvec_mag(g_eq));
}

// ============================================
// Consistency Tests
// ============================================

TEST(J2Gravity, ReducesToPointMassWhenJ2Zero) {
    // With J2=0, J2 model should equal point mass
    auto r = make_pos(7000000.0, 1000000.0, 500000.0);

    auto g_j2 = j2::acceleration(r, constants::earth::mu.value(), 0.0,
                                 constants::earth::R_eq.value());
    auto g_pm = point_mass::acceleration(r);

    EXPECT_NEAR(g_j2(0).value(), g_pm(0).value(), 1e-12);
    EXPECT_NEAR(g_j2(1).value(), g_pm(1).value(), 1e-12);
    EXPECT_NEAR(g_j2(2).value(), g_pm(2).value(), 1e-12);
}

TEST(J2Gravity, Potential) {
    auto r = make_pos(constants::earth::R_eq.value(), 0.0, 0.0);

    auto U_j2 = j2::potential(r);
    auto U_pm = point_mass::potential(r);

    // Both should be negative
    EXPECT_LT(U_j2.value(), 0.0);
    EXPECT_LT(U_pm.value(), 0.0);

    // J2 correction is relatively small
    EXPECT_NEAR(U_j2.value(), U_pm.value(),
                std::abs(U_pm.value()) * 0.01); // Within 1%
}

// ============================================
// Symbolic Tests
// ============================================

TEST(J2Gravity, SymbolicEvaluation) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    auto r = make_pos(x, y, z);

    auto g = j2::acceleration(r);

    // Create function for evaluation
    janus::Function f("j2_accel", {x, y, z},
                      {g(0).value(), g(1).value(), g(2).value()});

    // Evaluate at specific point
    double R = constants::earth::R_eq.value();
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

    auto r_sym = make_pos(x, y, z);
    auto r_num = make_pos(7000000.0, 1000000.0, 500000.0);

    auto g_sym = j2::acceleration(r_sym);
    auto g_num = j2::acceleration(r_num);

    janus::Function f("j2_accel_test", {x, y, z},
                      {g_sym(0).value(), g_sym(1).value(), g_sym(2).value()});
    auto result = f({7000000.0, 1000000.0, 500000.0});

    EXPECT_NEAR(result[0](0, 0), g_num(0).value(), 1e-10);
    EXPECT_NEAR(result[1](0, 0), g_num(1).value(), 1e-10);
    EXPECT_NEAR(result[2](0, 0), g_num(2).value(), 1e-10);
}

TEST(J2Gravity, SymbolicPotential) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    auto r = make_pos(x, y, z);

    auto U = j2::potential(r);

    janus::Function f("j2_potential", {x, y, z}, {U.value()});

    auto r_num = make_pos(7000000.0, 0.0, 1000000.0);

    auto result = f({7000000.0, 0.0, 1000000.0});
    double U_num = j2::potential(r_num).value();

    EXPECT_NEAR(result[0](0, 0), U_num, 1e-6);
}
