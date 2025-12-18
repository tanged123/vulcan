#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/atmosphere/StandardAtmosphere.hpp>
#include <vulcan/atmosphere/US76Atmosphere.hpp>

// ============================================
// Legacy Analytical Model Tests
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

// ============================================
// US76 Table-Based Model Tests
// ============================================

TEST(US76Atmosphere, SeaLevelValues) {
    // Reference: US Standard Atmosphere 1976 at z=0
    // T = 288.15 K, P = 101325 Pa, rho = 1.225 kg/m³, a = 340.3 m/s
    EXPECT_NEAR(vulcan::us76::temperature(0.0), 288.1, 0.5);
    EXPECT_NEAR(vulcan::us76::pressure(0.0), 101325, 10);
    EXPECT_NEAR(vulcan::us76::density(0.0), 1.225, 0.001);
    EXPECT_NEAR(vulcan::us76::speed_of_sound(0.0), 340.3, 0.5);
    EXPECT_NEAR(vulcan::us76::gravity(0.0), 9.807, 0.001);
}

TEST(US76Atmosphere, At10km) {
    // Reference: US Standard Atmosphere 1976 at z=10 km
    double alt = 10000.0; // 10 km in meters
    EXPECT_NEAR(vulcan::us76::temperature(alt), 223.3, 0.5);
    EXPECT_NEAR(vulcan::us76::pressure(alt), 26499, 50);
    EXPECT_NEAR(vulcan::us76::density(alt), 0.414, 0.01);
    EXPECT_NEAR(vulcan::us76::speed_of_sound(alt), 299.5, 0.5);
}

TEST(US76Atmosphere, At50km) {
    // Reference: US Standard Atmosphere 1976 at z=50 km (stratopause)
    double alt = 50000.0; // 50 km in meters
    EXPECT_NEAR(vulcan::us76::temperature(alt), 270.65, 1.0);
    EXPECT_NEAR(vulcan::us76::pressure(alt), 79.8, 5);
    EXPECT_NEAR(vulcan::us76::density(alt), 1.03e-3, 1e-4);
}

TEST(US76Atmosphere, Interpolation) {
    // Test that interpolation works for non-grid points
    double alt = 7500.0; // 7.5 km = grid point
    double T_grid = vulcan::us76::temperature(alt);
    EXPECT_NEAR(T_grid, 239.5, 0.1); // Exact grid value

    // 7.25 km - between 7.0 and 7.5 km
    double alt_interp = 7250.0;
    double T_interp = vulcan::us76::temperature(alt_interp);
    EXPECT_GT(T_interp, 239.5); // Higher than 7.5 km
    EXPECT_LT(T_interp, 242.7); // Lower than 7.0 km
}

TEST(US76Atmosphere, SymbolicEvaluation) {
    auto alt = janus::sym("altitude");
    auto T = vulcan::us76::temperature(alt);

    // Verify symbolic expression was created
    EXPECT_FALSE(T.is_constant());

    // Create function and evaluate
    janus::Function f("T_us76", {alt}, {T});
    auto result = f({10000.0});
    EXPECT_NEAR(result[0](0, 0), 223.3, 0.5);
}

TEST(US76Atmosphere, SymbolicGradient) {
    auto alt = janus::sym("altitude");
    auto rho = vulcan::us76::density(alt);

    // Compute derivative
    auto drho_dalt = janus::jacobian(rho, alt);

    janus::Function f("drho_dalt_us76", {alt}, {drho_dalt});
    auto result = f({5000.0});

    // Density should decrease with altitude
    EXPECT_LT(result[0](0, 0), 0.0);
}

TEST(US76Atmosphere, BatchQuery) {
    // Test batch evaluation
    janus::NumericVector altitudes(4);
    altitudes << 0, 5000, 10000, 20000; // 0, 5, 10, 20 km

    auto temps = vulcan::us76::detail::temperature_table()(altitudes);

    EXPECT_EQ(temps.size(), 4);
    EXPECT_NEAR(temps(0), 288.1, 0.5); // Sea level
    EXPECT_NEAR(temps(1), 255.7, 0.5); // 5 km
    EXPECT_NEAR(temps(2), 223.3, 0.5); // 10 km
    EXPECT_NEAR(temps(3), 216.6, 0.5); // 20 km (tropopause)
}
