#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/atmosphere/StandardAtmosphere.hpp>

// ============================================
// Numeric Tests
// ============================================
TEST(StandardAtmosphere, SeaLevelTemperature) {
    double T = vulcan::standard_atmosphere::temperature(0.0);
    EXPECT_NEAR(T, 288.15, 1e-3);
}

TEST(StandardAtmosphere, SeaLevelDensity) {
    double rho = vulcan::standard_atmosphere::density(0.0);
    EXPECT_NEAR(rho, 1.225, 1e-3);
}

TEST(StandardAtmosphere, SeaLevelPressure) {
    double P = vulcan::standard_atmosphere::pressure(0.0);
    EXPECT_NEAR(P, 101325.0, 1.0);
}

TEST(StandardAtmosphere, TropopauseDensity) {
    double rho = vulcan::standard_atmosphere::density(11000.0);
    EXPECT_NEAR(rho, 0.3639, 1e-2);
}

TEST(StandardAtmosphere, SpeedOfSound) {
    double a = vulcan::standard_atmosphere::speed_of_sound(0.0);
    EXPECT_NEAR(a, 340.3, 0.5); // ~340 m/s at sea level
}

// ============================================
// Symbolic Tests (Graph Generation)
// ============================================
TEST(StandardAtmosphere, SymbolicEvaluation) {
    auto alt = janus::sym("altitude");
    auto rho = vulcan::standard_atmosphere::density(alt);

    // Verify symbolic expression was created
    EXPECT_FALSE(rho.is_constant());

    // Create a CasADi function to evaluate the symbolic expression
    janus::Function f("rho", {alt}, {rho});
    auto result = f({0.0});
    // result[0] is a matrix, extract scalar with (0,0)
    EXPECT_NEAR(result[0](0, 0), 1.225, 1e-3);
}

TEST(StandardAtmosphere, SymbolicGradient) {
    auto alt = janus::sym("altitude");
    auto rho = vulcan::standard_atmosphere::density(alt);

    // Verify derivatives exist
    auto drho_dalt = janus::jacobian(rho, alt);

    // Create function for gradient evaluation
    janus::Function f("drho_dalt", {alt}, {drho_dalt});
    auto result = f({5000.0});

    // Density should decrease with altitude (negative gradient)
    EXPECT_LT(result[0](0, 0), 0.0);
}

TEST(StandardAtmosphere, SymbolicTemperature) {
    auto alt = janus::sym("altitude");
    auto T = vulcan::standard_atmosphere::temperature(alt);

    // Create function for temperature evaluation
    janus::Function f("T", {alt}, {T});
    auto result = f({5000.0});
    double T_5km = result[0](0, 0);

    // Troposphere: T should decrease
    EXPECT_LT(T_5km, 288.15);
    EXPECT_GT(T_5km, 220.0);
}
