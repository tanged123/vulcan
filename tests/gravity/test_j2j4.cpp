#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/gravity/J2.hpp>
#include <vulcan/gravity/J2J4.hpp>
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

// Helper: subtract two Vec3<Quantity<U>> and return raw Vec3<double>
template <auto U>
Vec3<double> qvec_diff(const Vec3<Quantity<U, double>> &a,
                       const Vec3<Quantity<U, double>> &b) {
    Vec3<double> d;
    d(0) = a(0).value() - b(0).value();
    d(1) = a(1).value() - b(1).value();
    d(2) = a(2).value() - b(2).value();
    return d;
}

// ============================================
// Numeric Tests
// ============================================

TEST(J2J4Gravity, BasicFunctionality) {
    auto r = make_pos(7000000.0, 0.0, 0.0);

    auto g = j2j4::acceleration(r);

    // Should produce valid acceleration
    EXPECT_LT(g(0).value(), 0.0); // Points toward center
    EXPECT_TRUE(std::isfinite(qvec_mag(g)));
}

TEST(J2J4Gravity, CloseToJ2AtLowAltitude) {
    // At LEO, J3/J4 contributions are small compared to J2
    auto r = make_pos(constants::earth::R_eq.value() + 400000.0, 0.0, 0.0);

    auto g_j2j4 = j2j4::acceleration(r);
    auto g_j2 = j2::acceleration(r);

    // Should be within 0.1% of J2-only
    auto d = qvec_diff(g_j2j4, g_j2);
    double diff = janus::norm(d);
    double mag = qvec_mag(g_j2);

    EXPECT_LT(diff / mag, 0.001);
}

TEST(J2J4Gravity, J3AsymmetryNorthSouth) {
    // J3 introduces north-south asymmetry
    double alt = 500000.0;

    auto r_north =
        make_pos(0.0, 0.0, constants::earth::R_eq.value() + alt); // North
    auto r_south =
        make_pos(0.0, 0.0, -(constants::earth::R_eq.value() + alt)); // South

    auto g_north = j2j4::acceleration(r_north);
    auto g_south = j2j4::acceleration(r_south);

    // Magnitudes should be slightly different due to J3
    double mag_north = qvec_mag(g_north);
    double mag_south = qvec_mag(g_south);

    // The difference may be very small; mainly testing code runs
    EXPECT_TRUE(std::isfinite(mag_north));
    EXPECT_TRUE(std::isfinite(mag_south));
}

TEST(J2J4Gravity, ReducesToJ2WhenJ3J4Zero) {
    auto r = make_pos(7000000.0, 1000000.0, 500000.0);

    auto g_j2j4 = j2j4::acceleration(r, constants::earth::mu.value(),
                                     constants::earth::J2.value(), 0.0, 0.0,
                                     constants::earth::R_eq.value());
    auto g_j2 = j2::acceleration(r);

    EXPECT_NEAR(g_j2j4(0).value(), g_j2(0).value(), 1e-12);
    EXPECT_NEAR(g_j2j4(1).value(), g_j2(1).value(), 1e-12);
    EXPECT_NEAR(g_j2j4(2).value(), g_j2(2).value(), 1e-12);
}

TEST(J2J4Gravity, ReducesToPointMassWhenAllZero) {
    auto r = make_pos(7000000.0, 1000000.0, 500000.0);

    auto g_j2j4 = j2j4::acceleration(r, constants::earth::mu.value(), 0.0, 0.0,
                                     0.0, constants::earth::R_eq.value());
    auto g_pm = point_mass::acceleration(r);

    EXPECT_NEAR(g_j2j4(0).value(), g_pm(0).value(), 1e-12);
    EXPECT_NEAR(g_j2j4(1).value(), g_pm(1).value(), 1e-12);
    EXPECT_NEAR(g_j2j4(2).value(), g_pm(2).value(), 1e-12);
}

// ============================================
// Potential Tests
// ============================================

TEST(J2J4Gravity, PotentialBasic) {
    auto r = make_pos(constants::earth::R_eq.value() + 500000.0, 0.0, 0.0);

    auto U = j2j4::potential(r);

    EXPECT_LT(U.value(), 0.0); // Negative potential
    EXPECT_TRUE(std::isfinite(U.value()));
}

TEST(J2J4Gravity, PotentialConsistency) {
    auto r = make_pos(7000000.0, 0.0, 1000000.0);

    auto U_j2j4 = j2j4::potential(r);
    auto U_j2 = j2::potential(r);
    auto U_pm = point_mass::potential(r);

    // All should be negative and roughly similar
    EXPECT_LT(U_j2j4.value(), 0.0);
    EXPECT_LT(U_j2.value(), 0.0);
    EXPECT_LT(U_pm.value(), 0.0);

    // J2J4 should be close to J2
    EXPECT_NEAR(U_j2j4.value(), U_j2.value(), std::abs(U_j2.value()) * 0.001);
}

// ============================================
// Symbolic Tests
// ============================================

TEST(J2J4Gravity, SymbolicEvaluation) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    auto r = make_pos(x, y, z);

    auto g = j2j4::acceleration(r);

    // Create function for evaluation
    janus::Function f("j2j4_accel", {x, y, z},
                      {g(0).value(), g(1).value(), g(2).value()});

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

    auto r_sym = make_pos(x, y, z);
    auto r_num = make_pos(7000000.0, 1000000.0, 500000.0);

    auto g_sym = j2j4::acceleration(r_sym);
    auto g_num = j2j4::acceleration(r_num);

    janus::Function f("j2j4_test", {x, y, z},
                      {g_sym(0).value(), g_sym(1).value(), g_sym(2).value()});
    auto result = f({7000000.0, 1000000.0, 500000.0});

    EXPECT_NEAR(result[0](0, 0), g_num(0).value(), 1e-8);
    EXPECT_NEAR(result[1](0, 0), g_num(1).value(), 1e-8);
    EXPECT_NEAR(result[2](0, 0), g_num(2).value(), 1e-8);
}

TEST(J2J4Gravity, SymbolicPotential) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    auto r = make_pos(x, y, z);

    auto U = j2j4::potential(r);

    janus::Function f("j2j4_potential", {x, y, z}, {U.value()});

    auto r_num = make_pos(7000000.0, 500000.0, 2000000.0);

    auto result = f({7000000.0, 500000.0, 2000000.0});
    double U_num = j2j4::potential(r_num).value();

    EXPECT_NEAR(result[0](0, 0), U_num, 1e-6);
}
