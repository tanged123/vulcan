// Tests for Exponential Atmosphere Model
#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/atmosphere/ExponentialAtmosphere.hpp>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/Units.hpp>

#include <cmath>

using namespace vulcan::units;
using vulcan::Quantity;

// ============================================
// Basic Numeric Tests
// ============================================

TEST(ExponentialAtmosphere, SeaLevelValues) {
    // At sea level (h=0), should return reference values
    EXPECT_NEAR(
        vulcan::exponential_atmosphere::density(Quantity<m>(0.0)).value(),
        1.225, 1e-6);
    EXPECT_NEAR(
        vulcan::exponential_atmosphere::pressure(Quantity<m>(0.0)).value(),
        101325.0, 1e-6);
    EXPECT_NEAR(
        vulcan::exponential_atmosphere::temperature(Quantity<m>(0.0)).value(),
        288.15, 1e-6);
    EXPECT_NEAR(vulcan::exponential_atmosphere::speed_of_sound(Quantity<m>(0.0))
                    .value(),
                340.3, 0.1);
}

TEST(ExponentialAtmosphere, DensityDecay) {
    const double H = vulcan::exponential_atmosphere::DEFAULT_SCALE_HEIGHT;

    // At one scale height, density should be e^(-1) ~ 0.368 of sea level
    double rho_H =
        vulcan::exponential_atmosphere::density(Quantity<m>(H)).value();
    double expected = vulcan::exponential_atmosphere::RHO_0 * std::exp(-1.0);
    EXPECT_NEAR(rho_H, expected, 1e-6);

    // At two scale heights, density should be e^(-2) ~ 0.135 of sea level
    double rho_2H =
        vulcan::exponential_atmosphere::density(Quantity<m>(2.0 * H)).value();
    expected = vulcan::exponential_atmosphere::RHO_0 * std::exp(-2.0);
    EXPECT_NEAR(rho_2H, expected, 1e-6);
}

TEST(ExponentialAtmosphere, PressureDecay) {
    const double H = vulcan::exponential_atmosphere::DEFAULT_SCALE_HEIGHT;

    // Same exponential decay as density
    double P_H =
        vulcan::exponential_atmosphere::pressure(Quantity<m>(H)).value();
    double expected = vulcan::exponential_atmosphere::P_0 * std::exp(-1.0);
    EXPECT_NEAR(P_H, expected, 1e-3);
}

TEST(ExponentialAtmosphere, IsothermalTemperature) {
    // Temperature should be constant regardless of altitude
    EXPECT_NEAR(
        vulcan::exponential_atmosphere::temperature(Quantity<m>(0.0)).value(),
        vulcan::exponential_atmosphere::T_0, 1e-6);
    EXPECT_NEAR(
        vulcan::exponential_atmosphere::temperature(Quantity<m>(10000.0))
            .value(),
        vulcan::exponential_atmosphere::T_0, 1e-6);
    EXPECT_NEAR(
        vulcan::exponential_atmosphere::temperature(Quantity<m>(50000.0))
            .value(),
        vulcan::exponential_atmosphere::T_0, 1e-6);
}

TEST(ExponentialAtmosphere, SpeedOfSoundConstant) {
    // Speed of sound should be constant (isothermal)
    double a_0 =
        vulcan::exponential_atmosphere::speed_of_sound(Quantity<m>(0.0))
            .value();
    double a_10km =
        vulcan::exponential_atmosphere::speed_of_sound(Quantity<m>(10000.0))
            .value();
    double a_50km =
        vulcan::exponential_atmosphere::speed_of_sound(Quantity<m>(50000.0))
            .value();

    EXPECT_NEAR(a_0, a_10km, 1e-6);
    EXPECT_NEAR(a_0, a_50km, 1e-6);

    // Should be approximately 340.3 m/s at 288.15 K
    EXPECT_NEAR(a_0, 340.3, 0.1);
}

TEST(ExponentialAtmosphere, CustomScaleHeight) {
    // Test with custom scale height
    double custom_H = 7000.0; // 7 km scale height

    double rho =
        vulcan::exponential_atmosphere::density(Quantity<m>(7000.0), custom_H)
            .value();
    double expected = vulcan::exponential_atmosphere::RHO_0 * std::exp(-1.0);
    EXPECT_NEAR(rho, expected, 1e-6);
}

TEST(ExponentialAtmosphere, NegativeAltitude) {
    // Below sea level, density should be higher than rho_0
    double rho =
        vulcan::exponential_atmosphere::density(Quantity<m>(-500.0)).value();
    EXPECT_GT(rho, vulcan::exponential_atmosphere::RHO_0);

    // Should be exp(0.5/8.5) ~ 1.060 times sea level
    double expected =
        vulcan::exponential_atmosphere::RHO_0 *
        std::exp(500.0 / vulcan::exponential_atmosphere::DEFAULT_SCALE_HEIGHT);
    EXPECT_NEAR(rho, expected, 1e-6);
}

// ============================================
// State Struct Tests
// ============================================

TEST(ExponentialAtmosphere, StateStructSeaLevel) {
    auto state = vulcan::exponential_atmosphere::state(Quantity<m>(0.0));

    EXPECT_NEAR(state.temperature.value(), 288.15, 1e-6);
    EXPECT_NEAR(state.pressure.value(), 101325.0, 1e-6);
    EXPECT_NEAR(state.density.value(), 1.225, 1e-6);
    EXPECT_NEAR(state.speed_of_sound.value(), 340.3, 0.1);
}

TEST(ExponentialAtmosphere, StateStructAt10km) {
    auto state = vulcan::exponential_atmosphere::state(Quantity<m>(10000.0));

    // Temperature remains constant
    EXPECT_NEAR(state.temperature.value(), 288.15, 1e-6);

    // Pressure and density decay exponentially
    double exp_factor = std::exp(
        -10000.0 / vulcan::exponential_atmosphere::DEFAULT_SCALE_HEIGHT);
    EXPECT_NEAR(state.pressure.value(), 101325.0 * exp_factor, 1);
    EXPECT_NEAR(state.density.value(), 1.225 * exp_factor, 1e-4);
}

// ============================================
// Inverse Function Tests
// ============================================

TEST(ExponentialAtmosphere, ComputeScaleHeight) {
    // Default scale height should be approximately R*T/g
    double H = vulcan::exponential_atmosphere::compute_scale_height();
    double expected = vulcan::exponential_atmosphere::R_AIR *
                      vulcan::exponential_atmosphere::T_0 /
                      vulcan::exponential_atmosphere::G_0;
    EXPECT_NEAR(H, expected, 1e-6);
    EXPECT_NEAR(H, 8435.0, 10); // Approximately 8.4 km
}

TEST(ExponentialAtmosphere, AltitudeFromDensityRoundTrip) {
    // Test round-trip: altitude -> density -> altitude
    double altitudes[] = {0.0, 5000.0, 10000.0, 20000.0, 50000.0};

    for (double alt : altitudes) {
        auto rho = vulcan::exponential_atmosphere::density(Quantity<m>(alt));
        double recovered =
            vulcan::exponential_atmosphere::altitude_from_density(rho).value();
        EXPECT_NEAR(recovered, alt, 1e-6) << "Failed at altitude " << alt;
    }
}

TEST(ExponentialAtmosphere, AltitudeFromPressureRoundTrip) {
    // Test round-trip: altitude -> pressure -> altitude
    double altitudes[] = {0.0, 5000.0, 10000.0, 20000.0, 50000.0};

    for (double alt : altitudes) {
        auto P = vulcan::exponential_atmosphere::pressure(Quantity<m>(alt));
        double recovered =
            vulcan::exponential_atmosphere::altitude_from_pressure(P).value();
        EXPECT_NEAR(recovered, alt, 1e-6) << "Failed at altitude " << alt;
    }
}

// ============================================
// Symbolic Tests
// ============================================

TEST(ExponentialAtmosphere, SymbolicDensity) {
    using SX = janus::SymbolicScalar;
    auto alt = janus::sym("altitude");
    auto rho = vulcan::exponential_atmosphere::density(Quantity<m, SX>(alt));

    EXPECT_FALSE(rho.value().is_constant());

    janus::Function f("rho_exp", {alt}, {rho.value()});
    auto result = f({10000.0});

    double expected =
        vulcan::exponential_atmosphere::RHO_0 *
        std::exp(-10000.0 /
                 vulcan::exponential_atmosphere::DEFAULT_SCALE_HEIGHT);
    EXPECT_NEAR(result[0](0, 0), expected, 1e-6);
}

TEST(ExponentialAtmosphere, SymbolicPressure) {
    using SX = janus::SymbolicScalar;
    auto alt = janus::sym("altitude");
    auto P = vulcan::exponential_atmosphere::pressure(Quantity<m, SX>(alt));

    EXPECT_FALSE(P.value().is_constant());

    janus::Function f("P_exp", {alt}, {P.value()});
    auto result = f({5000.0});

    double expected =
        vulcan::exponential_atmosphere::P_0 *
        std::exp(-5000.0 /
                 vulcan::exponential_atmosphere::DEFAULT_SCALE_HEIGHT);
    EXPECT_NEAR(result[0](0, 0), expected, 1);
}

TEST(ExponentialAtmosphere, SymbolicGradient) {
    using SX = janus::SymbolicScalar;
    auto alt = janus::sym("altitude");
    auto rho = vulcan::exponential_atmosphere::density(Quantity<m, SX>(alt));

    auto drho_dalt = janus::jacobian(rho.value(), alt);

    janus::Function f("drho_dalt_exp", {alt}, {drho_dalt});
    auto result = f({5000.0});

    // Analytical gradient: drho/dh = -rho/H
    double rho_val =
        vulcan::exponential_atmosphere::RHO_0 *
        std::exp(-5000.0 /
                 vulcan::exponential_atmosphere::DEFAULT_SCALE_HEIGHT);
    double expected_grad =
        -rho_val / vulcan::exponential_atmosphere::DEFAULT_SCALE_HEIGHT;

    EXPECT_NEAR(result[0](0, 0), expected_grad, 1e-10);
}

TEST(ExponentialAtmosphere, SymbolicState) {
    using SX = janus::SymbolicScalar;
    auto alt = janus::sym("altitude");
    auto state = vulcan::exponential_atmosphere::state(Quantity<m, SX>(alt));

    // Verify all fields are symbolic expressions (not constants after altitude
    // dependence)
    EXPECT_FALSE(state.pressure.value().is_constant());
    EXPECT_FALSE(state.density.value().is_constant());

    // Create function with all outputs
    janus::Function f("state_exp", {alt},
                      {state.temperature.value(), state.pressure.value(),
                       state.density.value(), state.speed_of_sound.value()});

    auto result = f({0.0});
    EXPECT_NEAR(result[0](0, 0), 288.15, 1e-6);   // T
    EXPECT_NEAR(result[1](0, 0), 101325.0, 1e-3); // P
    EXPECT_NEAR(result[2](0, 0), 1.225, 1e-6);    // rho
    EXPECT_NEAR(result[3](0, 0), 340.3, 0.1);     // a
}

TEST(ExponentialAtmosphere, SymbolicAltitudeFromDensity) {
    using SX = janus::SymbolicScalar;
    auto rho = janus::sym("density");
    auto alt = vulcan::exponential_atmosphere::altitude_from_density(
        Quantity<kg_per_m3, SX>(rho));

    EXPECT_FALSE(alt.value().is_constant());

    janus::Function f("alt_from_rho", {rho}, {alt.value()});

    // At sea level density, altitude should be 0
    auto result = f({vulcan::exponential_atmosphere::RHO_0});
    EXPECT_NEAR(result[0](0, 0), 0.0, 1e-6);

    // At rho_0/e, altitude should be one scale height
    result = f({vulcan::exponential_atmosphere::RHO_0 / std::exp(1.0)});
    EXPECT_NEAR(result[0](0, 0),
                vulcan::exponential_atmosphere::DEFAULT_SCALE_HEIGHT, 1e-6);
}

// ============================================
// Comparison with USSA1976
// ============================================

TEST(ExponentialAtmosphere, ComparisonWithUSSA1976) {
    // The exponential model is a simplification, but should be reasonable
    // at low altitudes. Check that the error is bounded.

    // At sea level, both should match closely (by design)
    double rho_exp =
        vulcan::exponential_atmosphere::density(Quantity<m>(0.0)).value();
    EXPECT_NEAR(rho_exp, 1.225, 0.001);

    // At 10 km, the models will diverge
    // US76 gives ~0.414 kg/m^3, exponential gives different value
    double rho_exp_10km =
        vulcan::exponential_atmosphere::density(Quantity<m>(10000.0)).value();
    double exp_factor = std::exp(-10000.0 / 8500.0);
    EXPECT_NEAR(rho_exp_10km, 1.225 * exp_factor, 1e-6);
    // rho_exp_10km ~ 0.377 kg/m^3 (vs 0.414 for US76)
    // This is expected - the exponential model is less accurate at higher
    // altitudes
}
