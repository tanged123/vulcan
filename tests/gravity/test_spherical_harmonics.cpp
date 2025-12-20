#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/gravity/J2J4.hpp>
#include <vulcan/gravity/PointMass.hpp>
#include <vulcan/gravity/SphericalHarmonics.hpp>

using namespace vulcan;
using namespace vulcan::gravity;

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
    // P_2,0(x) = (3x² - 1) / 2
    for (double x : {-0.8, -0.3, 0.0, 0.5, 0.9}) {
        double expected = (3.0 * x * x - 1.0) / 2.0;
        EXPECT_NEAR(spherical_harmonics::legendre_Pnm(2, 0, x), expected,
                    1e-12);
    }
}

TEST(SphericalHarmonics, LegendreP30) {
    // P_3,0(x) = (5x³ - 3x) / 2
    for (double x : {-0.8, -0.3, 0.0, 0.5, 0.9}) {
        double expected = (5.0 * x * x * x - 3.0 * x) / 2.0;
        EXPECT_NEAR(spherical_harmonics::legendre_Pnm(3, 0, x), expected,
                    1e-12);
    }
}

TEST(SphericalHarmonics, LegendreP40) {
    // P_4,0(x) = (35x⁴ - 30x² + 3) / 8
    for (double x : {-0.8, -0.3, 0.0, 0.5, 0.9}) {
        double x2 = x * x;
        double expected = (35.0 * x2 * x2 - 30.0 * x2 + 3.0) / 8.0;
        EXPECT_NEAR(spherical_harmonics::legendre_Pnm(4, 0, x), expected,
                    1e-12);
    }
}

TEST(SphericalHarmonics, LegendreP11) {
    // P_1,1(x) = -sqrt(1-x²)
    for (double x : {-0.8, -0.3, 0.0, 0.5, 0.9}) {
        double expected = -std::sqrt(1.0 - x * x);
        EXPECT_NEAR(spherical_harmonics::legendre_Pnm(1, 1, x), expected,
                    1e-12);
    }
}

TEST(SphericalHarmonics, LegendreP21) {
    // P_2,1(x) = -3x * sqrt(1-x²)
    for (double x : {-0.8, -0.3, 0.0, 0.5, 0.9}) {
        double expected = -3.0 * x * std::sqrt(1.0 - x * x);
        EXPECT_NEAR(spherical_harmonics::legendre_Pnm(2, 1, x), expected,
                    1e-12);
    }
}

TEST(SphericalHarmonics, LegendreP22) {
    // P_2,2(x) = 3(1-x²)
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
    EXPECT_NEAR(coeffs.C[2][0], -constants::earth::J2, 1e-12);
    EXPECT_NEAR(coeffs.C[3][0], -constants::earth::J3, 1e-12);
    EXPECT_NEAR(coeffs.C[4][0], -constants::earth::J4, 1e-12);

    // All S coefficients should be zero for zonal-only model
    for (int n = 0; n <= 4; ++n) {
        for (int m = 0; m <= n; ++m) {
            EXPECT_NEAR(
                coeffs.S[static_cast<size_t>(n)][static_cast<size_t>(m)], 0.0,
                1e-12);
        }
    }
}

TEST(SphericalHarmonics, DefaultCoefficients) {
    const auto &coeffs = spherical_harmonics::default_coefficients();

    EXPECT_EQ(coeffs.n_max, 4);
    EXPECT_NEAR(coeffs.mu, constants::earth::mu, 1e6);
    EXPECT_NEAR(coeffs.R_eq, constants::earth::R_eq, 1.0);
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

    Vec3<double> r;
    r << 7000000.0, 1000000.0, 500000.0;

    auto g_sh = spherical_harmonics::acceleration(r, coeffs);
    auto g_pm = point_mass::acceleration(r);

    EXPECT_NEAR(g_sh(0), g_pm(0), std::abs(g_pm(0)) * 0.001);
    EXPECT_NEAR(g_sh(1), g_pm(1), std::abs(g_pm(1)) * 0.001);
    EXPECT_NEAR(g_sh(2), g_pm(2), std::abs(g_pm(2)) * 0.001);
}

TEST(SphericalHarmonics, AccelerationMagnitudeLEO) {
    // At LEO altitude, gravity should be close to surface value
    Vec3<double> r;
    r << constants::earth::R_eq + 400000.0, 0.0, 0.0;

    auto g = spherical_harmonics::acceleration(r);
    double g_mag = janus::norm(g);

    // LEO gravity ~8.7 m/s² (slightly less than 9.8 due to altitude)
    EXPECT_GT(g_mag, 8.0);
    EXPECT_LT(g_mag, 9.0);
}

TEST(SphericalHarmonics, AccelerationMagnitudeGEO) {
    // At GEO altitude (~35786 km), gravity is much weaker
    Vec3<double> r;
    r << constants::earth::R_eq + 35786000.0, 0.0, 0.0;

    auto g = spherical_harmonics::acceleration(r);
    double g_mag = janus::norm(g);

    // GEO gravity ~0.22 m/s²
    EXPECT_GT(g_mag, 0.20);
    EXPECT_LT(g_mag, 0.25);
}

TEST(SphericalHarmonics, AccelerationDirectionTowardCenter) {
    Vec3<double> r;
    r << 7000000.0, 500000.0, 300000.0;

    auto g = spherical_harmonics::acceleration(r);

    // Acceleration should point roughly toward center (negative dot product
    // with r)
    double dot = r(0) * g(0) + r(1) * g(1) + r(2) * g(2);
    EXPECT_LT(dot, 0.0);
}

TEST(SphericalHarmonics, ZonalHarmonicsMatchJ2J4) {
    // With only zonal harmonics, spherical harmonics should match J2J4
    Vec3<double> r;
    r << 7000000.0, 0.0, 1000000.0;

    auto g_sh = spherical_harmonics::acceleration(r);
    auto g_j2j4 = j2j4::acceleration(r);

    // Should match within 1% (small differences due to numerical methods)
    double tol = 0.01;
    EXPECT_NEAR(g_sh(0), g_j2j4(0), std::abs(g_j2j4(0)) * tol + 1e-8);
    EXPECT_NEAR(g_sh(1), g_j2j4(1), std::abs(g_j2j4(1)) * tol + 1e-8);
    EXPECT_NEAR(g_sh(2), g_j2j4(2), std::abs(g_j2j4(2)) * tol + 1e-8);
}

TEST(SphericalHarmonics, ZonalHarmonicsMatchJ2J4MultiplePositions) {
    // Test at multiple positions
    std::vector<Vec3<double>> positions;
    Vec3<double> r1, r2, r3, r4;
    r1 << 7000000.0, 0.0, 0.0;             // Equatorial
    r2 << 0.0, 0.0, 7000000.0;             // Polar
    r3 << 5000000.0, 3000000.0, 4000000.0; // Off-axis
    r4 << 8000000.0, 2000000.0, 1000000.0; // Higher altitude
    positions = {r1, r2, r3, r4};

    for (const auto &pos : positions) {
        auto g_sh = spherical_harmonics::acceleration(pos);
        auto g_j2j4 = j2j4::acceleration(pos);

        // Use 1% relative tolerance with 1e-4 absolute floor for near-zero
        // values (spherical harmonics has small numerical artifacts at poles
        // due to different gradient formulation vs closed-form J2J4)
        double tol = 0.01;
        double abs_tol = 1e-4;
        EXPECT_NEAR(g_sh(0), g_j2j4(0), std::abs(g_j2j4(0)) * tol + abs_tol);
        EXPECT_NEAR(g_sh(1), g_j2j4(1), std::abs(g_j2j4(1)) * tol + abs_tol);
        EXPECT_NEAR(g_sh(2), g_j2j4(2), std::abs(g_j2j4(2)) * tol + abs_tol);
    }
}

TEST(SphericalHarmonics, J2PerturbationStrongerThanJ4) {
    // J2 effect should dominate over J4
    spherical_harmonics::GravityCoefficients coeffs_j2_only(2);
    spherical_harmonics::GravityCoefficients coeffs_j4_only(4);
    coeffs_j4_only.C[2][0] = 0.0;
    coeffs_j4_only.C[3][0] = 0.0;

    Vec3<double> r;
    r << 7000000.0, 0.0, 1000000.0;

    // Get point mass for reference
    spherical_harmonics::GravityCoefficients coeffs_pm(0);
    auto g_pm = spherical_harmonics::acceleration(r, coeffs_pm);
    auto g_j2 = spherical_harmonics::acceleration(r, coeffs_j2_only);
    auto g_j4 = spherical_harmonics::acceleration(r, coeffs_j4_only);

    // Perturbation magnitudes
    Vec3<double> delta_j2 = g_j2 - g_pm;
    Vec3<double> delta_j4 = g_j4 - g_pm;

    double pert_j2 = janus::norm(delta_j2);
    double pert_j4 = janus::norm(delta_j4);

    // J2 perturbation should be much larger than J4
    EXPECT_GT(pert_j2, pert_j4 * 100.0);
}

TEST(SphericalHarmonics, InverseSquareFalloff) {
    // Gravity magnitude should follow approximately inverse square law
    Vec3<double> r1, r2;
    r1 << 7000000.0, 0.0, 0.0;
    r2 << 14000000.0, 0.0, 0.0; // Double the distance

    auto g1 = spherical_harmonics::acceleration(r1);
    auto g2 = spherical_harmonics::acceleration(r2);

    double mag1 = janus::norm(g1);
    double mag2 = janus::norm(g2);

    // At 2x distance, gravity should be ~1/4 (inverse square)
    double ratio = mag1 / mag2;
    EXPECT_NEAR(ratio, 4.0, 0.1); // Within 2.5%
}

// ============================================
// Potential Tests
// ============================================

TEST(SphericalHarmonics, PotentialNegative) {
    // Gravitational potential should be negative
    Vec3<double> r;
    r << constants::earth::R_eq + 500000.0, 0.0, 0.0;

    auto U = spherical_harmonics::potential(r);

    EXPECT_LT(U, 0.0);
    EXPECT_TRUE(std::isfinite(U));
}

TEST(SphericalHarmonics, PotentialDecreasesWithAltitude) {
    // Potential magnitude decreases (becomes less negative) with altitude
    Vec3<double> r_low, r_high;
    r_low << constants::earth::R_eq + 400000.0, 0.0, 0.0;
    r_high << constants::earth::R_eq + 800000.0, 0.0, 0.0;

    auto U_low = spherical_harmonics::potential(r_low);
    auto U_high = spherical_harmonics::potential(r_high);

    // Both negative, but high altitude is less negative (closer to zero)
    EXPECT_LT(U_low, U_high);
}

TEST(SphericalHarmonics, PotentialMatchesJ2J4) {
    Vec3<double> r;
    r << 7000000.0, 0.0, 1000000.0;

    auto U_sh = spherical_harmonics::potential(r);
    auto U_j2j4 = j2j4::potential(r);

    EXPECT_NEAR(U_sh, U_j2j4, std::abs(U_j2j4) * 0.001);
}

TEST(SphericalHarmonics, PotentialMatchesJ2J4MultiplePositions) {
    std::vector<Vec3<double>> positions;
    Vec3<double> r1, r2, r3;
    r1 << 7000000.0, 0.0, 0.0;
    r2 << 0.0, 0.0, 7000000.0;
    r3 << 5000000.0, 3000000.0, 4000000.0;
    positions = {r1, r2, r3};

    for (const auto &pos : positions) {
        auto U_sh = spherical_harmonics::potential(pos);
        auto U_j2j4 = j2j4::potential(pos);

        EXPECT_NEAR(U_sh, U_j2j4, std::abs(U_j2j4) * 0.001);
    }
}

TEST(SphericalHarmonics, PotentialPointMassOnly) {
    spherical_harmonics::GravityCoefficients coeffs(0);

    Vec3<double> r;
    r << 7000000.0, 0.0, 0.0;

    auto U_sh = spherical_harmonics::potential(r, coeffs);
    auto U_pm = point_mass::potential(r);

    EXPECT_NEAR(U_sh, U_pm, std::abs(U_pm) * 0.001);
}

// ============================================
// Symbolic Tests
// ============================================

TEST(SphericalHarmonics, SymbolicAcceleration) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r;
    r << x, y, z;

    spherical_harmonics::GravityCoefficients coeffs(2);

    auto g = spherical_harmonics::acceleration(r, coeffs);

    janus::Function f("sh_accel", {x, y, z}, {g(0), g(1), g(2)});
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

    Vec3<janus::SymbolicScalar> r_sym;
    r_sym << x, y, z;

    spherical_harmonics::GravityCoefficients coeffs(2);

    auto g_sym = spherical_harmonics::acceleration(r_sym, coeffs);

    janus::Function f("sh_test", {x, y, z}, {g_sym(0), g_sym(1), g_sym(2)});

    // Test at multiple positions
    std::vector<std::array<double, 3>> test_positions = {
        {7000000.0, 0.0, 0.0},
        {7000000.0, 500000.0, 2000000.0},
        {5000000.0, 4000000.0, 3000000.0}};

    for (const auto &pos : test_positions) {
        Vec3<double> r_num;
        r_num << pos[0], pos[1], pos[2];

        auto g_num = spherical_harmonics::acceleration(r_num, coeffs);
        auto result = f({pos[0], pos[1], pos[2]});

        EXPECT_NEAR(result[0](0, 0), g_num(0),
                    std::abs(g_num(0)) * 0.01 + 1e-8);
        EXPECT_NEAR(result[1](0, 0), g_num(1),
                    std::abs(g_num(1)) * 0.01 + 1e-8);
        EXPECT_NEAR(result[2](0, 0), g_num(2),
                    std::abs(g_num(2)) * 0.01 + 1e-8);
    }
}

TEST(SphericalHarmonics, SymbolicPotential) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r;
    r << x, y, z;

    spherical_harmonics::GravityCoefficients coeffs(2);

    auto U_sym = spherical_harmonics::potential(r, coeffs);

    janus::Function f("sh_potential", {x, y, z}, {U_sym});

    Vec3<double> r_num;
    r_num << 7000000.0, 500000.0, 1000000.0;

    auto U_num = spherical_harmonics::potential(r_num, coeffs);
    auto result = f({r_num(0), r_num(1), r_num(2)});

    EXPECT_NEAR(result[0](0, 0), U_num, std::abs(U_num) * 0.001);
}

TEST(SphericalHarmonics, SymbolicJacobian) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r;
    r << x, y, z;

    spherical_harmonics::GravityCoefficients coeffs(2);

    auto g = spherical_harmonics::acceleration(r, coeffs);

    // Compute Jacobian of acceleration w.r.t. position
    auto J = janus::jacobian({g(0), g(1), g(2)}, {x, y, z});

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
