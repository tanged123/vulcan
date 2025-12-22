// Anomaly Conversion Tests
#include <gtest/gtest.h>
#include <vulcan/orbital/AnomalyConversions.hpp>

using namespace vulcan::orbital::anomaly;

// Test Kepler equation solver
TEST(AnomalyConversions, KeplerEquation_LowEcc) {
    double M = 1.0; // rad
    double e = 0.1;

    double E = mean_to_eccentric(M, e);

    // Verify: M = E - e*sin(E)
    double M_check = E - e * std::sin(E);
    EXPECT_NEAR(M, M_check, 1e-12);
}

TEST(AnomalyConversions, KeplerEquation_HighEcc) {
    double M = 0.5;
    double e = 0.9;

    double E = mean_to_eccentric(M, e);
    double M_check = E - e * std::sin(E);

    EXPECT_NEAR(M, M_check, 1e-12);
}

TEST(AnomalyConversions, KeplerEquation_EdgeCase_Zero) {
    double M = 0.0;
    double e = 0.5;

    double E = mean_to_eccentric(M, e);
    EXPECT_NEAR(E, 0.0, 1e-12);
}

// Test eccentric to true anomaly
TEST(AnomalyConversions, EccentricToTrue_Periapsis) {
    double E = 0.0; // At periapsis
    double e = 0.5;

    double nu = eccentric_to_true(E, e);
    EXPECT_NEAR(nu, 0.0, 1e-12);
}

TEST(AnomalyConversions, EccentricToTrue_Apoapsis) {
    double E = M_PI; // At apoapsis
    double e = 0.5;

    double nu = eccentric_to_true(E, e);
    EXPECT_NEAR(std::abs(nu), M_PI, 1e-10);
}

// Test true to eccentric anomaly
TEST(AnomalyConversions, TrueToEccentric_Roundtrip) {
    double nu_orig = 1.5;
    double e = 0.3;

    double E = true_to_eccentric(nu_orig, e);
    double nu_check = eccentric_to_true(E, e);

    EXPECT_NEAR(nu_orig, nu_check, 1e-12);
}

// Test mean to true full chain
TEST(AnomalyConversions, MeanToTrue_Roundtrip) {
    double M_orig = 2.0;
    double e = 0.4;

    double nu = mean_to_true(M_orig, e);
    double M_check = true_to_mean(nu, e);

    EXPECT_NEAR(M_orig, M_check, 1e-10);
}

// Test eccentric to mean
TEST(AnomalyConversions, EccentricToMean) {
    double E = 1.0;
    double e = 0.2;

    double M = eccentric_to_mean(E, e);
    double E_check = mean_to_eccentric(M, e);

    EXPECT_NEAR(E, E_check, 1e-12);
}

// Symbolic tests
TEST(AnomalyConversions, Symbolic_EccentricToTrue) {
    auto E_sym = janus::sym("E");
    auto e_sym = janus::sym("e");

    auto nu = eccentric_to_true(E_sym, e_sym);

    janus::Function f("e2true", {E_sym, e_sym}, {nu});
    auto result = f({0.5, 0.3});

    EXPECT_GT(std::abs(result[0](0, 0)), 0.0);
}

TEST(AnomalyConversions, Symbolic_KeplerSolver) {
    auto M_sym = janus::sym("M");
    auto e_sym = janus::sym("e");

    auto E = mean_to_eccentric(M_sym, e_sym);

    janus::Function f("kepler", {M_sym, e_sym}, {E});
    auto result = f({1.0, 0.3});

    EXPECT_GT(result[0](0, 0), 0.0);
}
