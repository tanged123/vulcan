#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/gravity/J2J4.hpp>
#include <vulcan/gravity/PointMass.hpp>
#include <vulcan/gravity/SphericalHarmonics.hpp>
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
template <auto U>
double qvec_mag(const Vec3<Quantity<U, double>> &v) {
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
// Legendre Polynomial Tests
// ============================================

TEST(SphericalHarmonics, LegendreP00) {
    // P_0,0(x) = 1
    EXPECT_NEAR(spherical_harmonics::legendre_Pnm(0, 0, 0.5), 1.0, 1e-12);
    EXPECT_NEAR(spherical_harmonics::legendre_Pnm(0, 0, 0.0), 1.0, 1e-12);
    EXPECT_NEAR(spherical_harmonics::legendre_Pnm(0, 0, 1.0), 1.0, 1e-12);
    EXPECT_NEAR(spherical_harmonics::legendre_Pnm(0, 0, -0.7), 1.0, 1e-12);
}

TEST(SphericalHarmonics, LegendreP10) {
    // P_1,0(x) = x
    EXPECT_NEAR(spherical_harmonics::legendre_Pnm(1, 0, 0.5), 0.5, 1e-12);
    EXPECT_NEAR(spherical_harmonics::legendre_Pnm(1, 0, 0.0), 0.0, 1e-12);
    EXPECT_NEAR(spherical_harmonics::legendre_Pnm(1, 0, 1.0), 1.0, 1e-12);
    EXPECT_NEAR(spherical_harmonics::legendre_Pnm(1, 0, -0.3), -0.3, 1e-12);
}

TEST(SphericalHarmonics, LegendreP20) {
    // P_2,0(x) = (3x^2 - 1) / 2
    for (double x : {-0.8, -0.3, 0.0, 0.5, 0.9}) {
        double expected = (3.0 * x * x - 1.0) / 2.0;
        EXPECT_NEAR(spherical_harmonics::legendre_Pnm(2, 0, x), expected,
                    1e-12);
    }
}

TEST(SphericalHarmonics, LegendreP30) {
    // P_3,0(x) = (5x^3 - 3x) / 2
    for (double x : {-0.8, -0.3, 0.0, 0.5, 0.9}) {
        double expected = (5.0 * x * x * x - 3.0 * x) / 2.0;
        EXPECT_NEAR(spherical_harmonics::legendre_Pnm(3, 0, x), expected,
                    1e-12);
    }
}

TEST(SphericalHarmonics, LegendreP40) {
    // P_4,0(x) = (35x^4 - 30x^2 + 3) / 8
    for (double x : {-0.8, -0.3, 0.0, 0.5, 0.9}) {
        double x2 = x * x;
        double expected = (35.0 * x2 * x2 - 30.0 * x2 + 3.0) / 8.0;
        EXPECT_NEAR(spherical_harmonics::legendre_Pnm(4, 0, x), expected,
                    1e-12);
    }
}

TEST(SphericalHarmonics, LegendreP11) {
    // P_1,1(x) = -sqrt(1-x^2)
    for (double x : {-0.8, -0.3, 0.0, 0.5, 0.9}) {
        double expected = -std::sqrt(1.0 - x * x);
        EXPECT_NEAR(spherical_harmonics::legendre_Pnm(1, 1, x), expected,
                    1e-12);
    }
}

TEST(SphericalHarmonics, LegendreP21) {
    // P_2,1(x) = -3x * sqrt(1-x^2)
    for (double x : {-0.8, -0.3, 0.0, 0.5, 0.9}) {
        double expected = -3.0 * x * std::sqrt(1.0 - x * x);
        EXPECT_NEAR(spherical_harmonics::legendre_Pnm(2, 1, x), expected,
                    1e-12);
    }
}

TEST(SphericalHarmonics, LegendreP22) {
    // P_2,2(x) = 3(1-x^2)
    for (double x : {-0.8, -0.3, 0.0, 0.5, 0.9}) {
        double expected = 3.0 * (1.0 - x * x);
        EXPECT_NEAR(spherical_harmonics::legendre_Pnm(2, 2, x), expected,
                    1e-12);
    }
}

TEST(SphericalHarmonics, LegendreMGreaterThanN) {
    // P_n,m(x) = 0 when m > n
    EXPECT_NEAR(spherical_harmonics::legendre_Pnm(2, 3, 0.5), 0.0, 1e-12);
    EXPECT_NEAR(spherical_harmonics::legendre_Pnm(0, 1, 0.5), 0.0, 1e-12);
    EXPECT_NEAR(spherical_harmonics::legendre_Pnm(3, 5, 0.5), 0.0, 1e-12);
}

TEST(SphericalHarmonics, LegendreSymbolic) {
    auto x = janus::sym("x");

    auto P20 = spherical_harmonics::legendre_Pnm(2, 0, x);

    janus::Function f("legendre_p20", {x}, {P20});

    for (double val : {-0.5, 0.0, 0.5, 0.8}) {
        auto result = f({val});
        double expected = (3.0 * val * val - 1.0) / 2.0;
        EXPECT_NEAR(result[0](0, 0), expected, 1e-12);
    }
}

// ============================================
// Coefficients Tests
// ============================================

TEST(SphericalHarmonics, CoefficientConstruction) {
    spherical_harmonics::GravityCoefficients coeffs(4);

    EXPECT_EQ(coeffs.n_max, 4);
    EXPECT_EQ(coeffs.C.size(), 5u); // 0 to 4

    // Monopole term
    EXPECT_NEAR(coeffs.C[0][0], 1.0, 1e-12);

    // Zonal harmonics
    EXPECT_NEAR(coeffs.C[2][0], -constants::earth::J2.value(), 1e-12);
    EXPECT_NEAR(coeffs.C[3][0], -constants::earth::J3.value(), 1e-12);
    EXPECT_NEAR(coeffs.C[4][0], -constants::earth::J4.value(), 1e-12);

    // All S coefficients should be zero for zonal-only model
    for (int n = 0; n <= 4; ++n) {
        for (int mm = 0; mm <= n; ++mm) {
            EXPECT_NEAR(
                coeffs.S[static_cast<size_t>(n)][static_cast<size_t>(mm)], 0.0,
                1e-12);
        }
    }
}

TEST(SphericalHarmonics, DefaultCoefficients) {
    const auto &coeffs = spherical_harmonics::default_coefficients();

    EXPECT_EQ(coeffs.n_max, 4);
    EXPECT_NEAR(coeffs.mu, constants::earth::mu.value(), 1e6);
    EXPECT_NEAR(coeffs.R_eq, constants::earth::R_eq.value(), 1.0);
    EXPECT_NEAR(coeffs.C[0][0], 1.0, 1e-12);
}

TEST(SphericalHarmonics, CustomBodyCoefficients) {
    // Test with custom body parameters (e.g., Moon-like)
    double mu_moon = 4.9048695e12;
    double R_moon = 1737400.0;

    spherical_harmonics::GravityCoefficients coeffs(2, mu_moon, R_moon);

    EXPECT_NEAR(coeffs.mu, mu_moon, 1.0);
    EXPECT_NEAR(coeffs.R_eq, R_moon, 1.0);
    EXPECT_NEAR(coeffs.C[0][0], 1.0, 1e-12);
}

// ============================================
// Acceleration Tests
// ============================================

TEST(SphericalHarmonics, PointMassOnlyMatchesPointMass) {
    // With only C[0][0] = 1, should match point mass gravity
    spherical_harmonics::GravityCoefficients coeffs(4);
    coeffs.C[2][0] = 0.0;
    coeffs.C[3][0] = 0.0;
    coeffs.C[4][0] = 0.0;

    auto r = make_pos(7000000.0, 1000000.0, 500000.0);

    auto g_sh = spherical_harmonics::acceleration(r, coeffs);
    auto g_pm = point_mass::acceleration(r);

    EXPECT_NEAR(g_sh(0).value(), g_pm(0).value(),
                std::abs(g_pm(0).value()) * 0.001);
    EXPECT_NEAR(g_sh(1).value(), g_pm(1).value(),
                std::abs(g_pm(1).value()) * 0.001);
    EXPECT_NEAR(g_sh(2).value(), g_pm(2).value(),
                std::abs(g_pm(2).value()) * 0.001);
}

TEST(SphericalHarmonics, AccelerationMagnitudeLEO) {
    // At LEO altitude, gravity should be close to surface value
    auto r = make_pos(constants::earth::R_eq.value() + 400000.0, 0.0, 0.0);

    auto g = spherical_harmonics::acceleration(r);
    double g_mag = qvec_mag(g);

    // LEO gravity ~8.7 m/s^2 (slightly less than 9.8 due to altitude)
    EXPECT_GT(g_mag, 8.0);
    EXPECT_LT(g_mag, 9.0);
}

TEST(SphericalHarmonics, AccelerationMagnitudeGEO) {
    // At GEO altitude (~35786 km), gravity is much weaker
    auto r =
        make_pos(constants::earth::R_eq.value() + 35786000.0, 0.0, 0.0);

    auto g = spherical_harmonics::acceleration(r);
    double g_mag = qvec_mag(g);

    // GEO gravity ~0.22 m/s^2
    EXPECT_GT(g_mag, 0.20);
    EXPECT_LT(g_mag, 0.25);
}

TEST(SphericalHarmonics, AccelerationDirectionTowardCenter) {
    auto r = make_pos(7000000.0, 500000.0, 300000.0);

    auto g = spherical_harmonics::acceleration(r);

    // Acceleration should point roughly toward center (negative dot product
    // with r)
    double dot = r(0).value() * g(0).value() + r(1).value() * g(1).value() +
                 r(2).value() * g(2).value();
    EXPECT_LT(dot, 0.0);
}

TEST(SphericalHarmonics, ZonalHarmonicsMatchJ2J4) {
    // With only zonal harmonics, spherical harmonics should match J2J4
    auto r = make_pos(7000000.0, 0.0, 1000000.0);

    auto g_sh = spherical_harmonics::acceleration(r);
    auto g_j2j4 = j2j4::acceleration(r);

    // Should match within 1% (small differences due to numerical methods)
    double tol = 0.01;
    EXPECT_NEAR(g_sh(0).value(), g_j2j4(0).value(),
                std::abs(g_j2j4(0).value()) * tol + 1e-8);
    EXPECT_NEAR(g_sh(1).value(), g_j2j4(1).value(),
                std::abs(g_j2j4(1).value()) * tol + 1e-8);
    EXPECT_NEAR(g_sh(2).value(), g_j2j4(2).value(),
                std::abs(g_j2j4(2).value()) * tol + 1e-8);
}

TEST(SphericalHarmonics, ZonalHarmonicsMatchJ2J4MultiplePositions) {
    // Test at multiple positions
    std::vector<Vec3<Quantity<m, double>>> positions;
    positions.push_back(make_pos(7000000.0, 0.0, 0.0));             // Equatorial
    positions.push_back(make_pos(0.0, 0.0, 7000000.0));             // Polar
    positions.push_back(make_pos(5000000.0, 3000000.0, 4000000.0)); // Off-axis
    positions.push_back(
        make_pos(8000000.0, 2000000.0, 1000000.0)); // Higher altitude

    for (const auto &pos : positions) {
        auto g_sh = spherical_harmonics::acceleration(pos);
        auto g_j2j4 = j2j4::acceleration(pos);

        // Use 1% relative tolerance with 1e-4 absolute floor
        double tol = 0.01;
        double abs_tol = 1e-4;
        EXPECT_NEAR(g_sh(0).value(), g_j2j4(0).value(),
                    std::abs(g_j2j4(0).value()) * tol + abs_tol);
        EXPECT_NEAR(g_sh(1).value(), g_j2j4(1).value(),
                    std::abs(g_j2j4(1).value()) * tol + abs_tol);
        EXPECT_NEAR(g_sh(2).value(), g_j2j4(2).value(),
                    std::abs(g_j2j4(2).value()) * tol + abs_tol);
    }
}

TEST(SphericalHarmonics, J2PerturbationStrongerThanJ4) {
    // J2 effect should dominate over J4
    spherical_harmonics::GravityCoefficients coeffs_j2_only(2);
    spherical_harmonics::GravityCoefficients coeffs_j4_only(4);
    coeffs_j4_only.C[2][0] = 0.0;
    coeffs_j4_only.C[3][0] = 0.0;

    auto r = make_pos(7000000.0, 0.0, 1000000.0);

    // Get point mass for reference
    spherical_harmonics::GravityCoefficients coeffs_pm(0);
    auto g_pm = spherical_harmonics::acceleration(r, coeffs_pm);
    auto g_j2 = spherical_harmonics::acceleration(r, coeffs_j2_only);
    auto g_j4 = spherical_harmonics::acceleration(r, coeffs_j4_only);

    // Perturbation magnitudes
    Vec3<double> delta_j2 = qvec_diff(g_j2, g_pm);
    Vec3<double> delta_j4 = qvec_diff(g_j4, g_pm);

    double pert_j2 = janus::norm(delta_j2);
    double pert_j4 = janus::norm(delta_j4);

    // J2 perturbation should be much larger than J4
    EXPECT_GT(pert_j2, pert_j4 * 100.0);
}

TEST(SphericalHarmonics, InverseSquareFalloff) {
    // Gravity magnitude should follow approximately inverse square law
    auto r1 = make_pos(7000000.0, 0.0, 0.0);
    auto r2 = make_pos(14000000.0, 0.0, 0.0); // Double the distance

    auto g1 = spherical_harmonics::acceleration(r1);
    auto g2 = spherical_harmonics::acceleration(r2);

    double mag1 = qvec_mag(g1);
    double mag2 = qvec_mag(g2);

    // At 2x distance, gravity should be ~1/4 (inverse square)
    double ratio = mag1 / mag2;
    EXPECT_NEAR(ratio, 4.0, 0.1); // Within 2.5%
}

// ============================================
// Potential Tests
// ============================================

TEST(SphericalHarmonics, PotentialNegative) {
    // Gravitational potential should be negative
    auto r = make_pos(constants::earth::R_eq.value() + 500000.0, 0.0, 0.0);

    auto U = spherical_harmonics::potential(r);

    EXPECT_LT(U.value(), 0.0);
    EXPECT_TRUE(std::isfinite(U.value()));
}

TEST(SphericalHarmonics, PotentialDecreasesWithAltitude) {
    // Potential magnitude decreases (becomes less negative) with altitude
    auto r_low =
        make_pos(constants::earth::R_eq.value() + 400000.0, 0.0, 0.0);
    auto r_high =
        make_pos(constants::earth::R_eq.value() + 800000.0, 0.0, 0.0);

    auto U_low = spherical_harmonics::potential(r_low);
    auto U_high = spherical_harmonics::potential(r_high);

    // Both negative, but high altitude is less negative (closer to zero)
    EXPECT_LT(U_low.value(), U_high.value());
}

TEST(SphericalHarmonics, PotentialMatchesJ2J4) {
    auto r = make_pos(7000000.0, 0.0, 1000000.0);

    auto U_sh = spherical_harmonics::potential(r);
    auto U_j2j4 = j2j4::potential(r);

    EXPECT_NEAR(U_sh.value(), U_j2j4.value(),
                std::abs(U_j2j4.value()) * 0.001);
}

TEST(SphericalHarmonics, PotentialMatchesJ2J4MultiplePositions) {
    std::vector<Vec3<Quantity<m, double>>> positions;
    positions.push_back(make_pos(7000000.0, 0.0, 0.0));
    positions.push_back(make_pos(0.0, 0.0, 7000000.0));
    positions.push_back(make_pos(5000000.0, 3000000.0, 4000000.0));

    for (const auto &pos : positions) {
        auto U_sh = spherical_harmonics::potential(pos);
        auto U_j2j4 = j2j4::potential(pos);

        EXPECT_NEAR(U_sh.value(), U_j2j4.value(),
                    std::abs(U_j2j4.value()) * 0.001);
    }
}

TEST(SphericalHarmonics, PotentialPointMassOnly) {
    spherical_harmonics::GravityCoefficients coeffs(0);

    auto r = make_pos(7000000.0, 0.0, 0.0);

    auto U_sh = spherical_harmonics::potential(r, coeffs);
    auto U_pm = point_mass::potential(r);

    EXPECT_NEAR(U_sh.value(), U_pm.value(),
                std::abs(U_pm.value()) * 0.001);
}

// ============================================
// Symbolic Tests
// ============================================

TEST(SphericalHarmonics, SymbolicAcceleration) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    auto r = make_pos(x, y, z);

    spherical_harmonics::GravityCoefficients coeffs(2);

    auto g = spherical_harmonics::acceleration(r, coeffs);

    janus::Function f("sh_accel", {x, y, z},
                      {g(0).value(), g(1).value(), g(2).value()});
    auto result = f({7000000.0, 0.0, 1000000.0});

    // Should produce finite values
    EXPECT_TRUE(std::isfinite(static_cast<double>(result[0](0, 0))));
    EXPECT_TRUE(std::isfinite(static_cast<double>(result[1](0, 0))));
    EXPECT_TRUE(std::isfinite(static_cast<double>(result[2](0, 0))));

    // And reasonable magnitudes
    double g_mag = std::sqrt(std::pow(static_cast<double>(result[0](0, 0)), 2) +
                             std::pow(static_cast<double>(result[1](0, 0)), 2) +
                             std::pow(static_cast<double>(result[2](0, 0)), 2));
    EXPECT_GT(g_mag, 5.0);
    EXPECT_LT(g_mag, 15.0);
}

TEST(SphericalHarmonics, SymbolicMatchesNumeric) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    auto r_sym = make_pos(x, y, z);

    spherical_harmonics::GravityCoefficients coeffs(2);

    auto g_sym = spherical_harmonics::acceleration(r_sym, coeffs);

    janus::Function f("sh_test", {x, y, z},
                      {g_sym(0).value(), g_sym(1).value(), g_sym(2).value()});

    // Test at multiple positions
    std::vector<std::array<double, 3>> test_positions = {
        {7000000.0, 0.0, 0.0},
        {7000000.0, 500000.0, 2000000.0},
        {5000000.0, 4000000.0, 3000000.0}};

    for (const auto &pos : test_positions) {
        auto r_num = make_pos(pos[0], pos[1], pos[2]);

        auto g_num = spherical_harmonics::acceleration(r_num, coeffs);
        auto result = f({pos[0], pos[1], pos[2]});

        EXPECT_NEAR(result[0](0, 0), g_num(0).value(),
                    std::abs(g_num(0).value()) * 0.01 + 1e-8);
        EXPECT_NEAR(result[1](0, 0), g_num(1).value(),
                    std::abs(g_num(1).value()) * 0.01 + 1e-8);
        EXPECT_NEAR(result[2](0, 0), g_num(2).value(),
                    std::abs(g_num(2).value()) * 0.01 + 1e-8);
    }
}

TEST(SphericalHarmonics, SymbolicPotential) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    auto r = make_pos(x, y, z);

    spherical_harmonics::GravityCoefficients coeffs(2);

    auto U_sym = spherical_harmonics::potential(r, coeffs);

    janus::Function f("sh_potential", {x, y, z}, {U_sym.value()});

    auto r_num = make_pos(7000000.0, 500000.0, 1000000.0);

    auto U_num = spherical_harmonics::potential(r_num, coeffs);
    auto result = f({7000000.0, 500000.0, 1000000.0});

    EXPECT_NEAR(result[0](0, 0), U_num.value(),
                std::abs(U_num.value()) * 0.001);
}

TEST(SphericalHarmonics, SymbolicJacobian) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    auto r = make_pos(x, y, z);

    spherical_harmonics::GravityCoefficients coeffs(2);

    auto g = spherical_harmonics::acceleration(r, coeffs);

    // Compute Jacobian of acceleration w.r.t. position
    auto J =
        janus::jacobian({g(0).value(), g(1).value(), g(2).value()}, {x, y, z});

    janus::Function f("sh_jacobian", {x, y, z},
                      {J(0, 0), J(0, 1), J(0, 2), J(1, 0), J(1, 1), J(1, 2),
                       J(2, 0), J(2, 1), J(2, 2)});

    auto result = f({7000000.0, 0.0, 1000000.0});

    // All Jacobian elements should be finite
    for (int i = 0; i < 9; ++i) {
        EXPECT_TRUE(std::isfinite(
            static_cast<double>(result[static_cast<size_t>(i)](0, 0))));
    }
}
