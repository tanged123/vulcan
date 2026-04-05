// Tests for US Standard Atmosphere 1976 (USSA1976)
#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/atmosphere/USSA1976.hpp>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/Units.hpp>

using namespace vulcan::units;
using vulcan::Quantity;

// ============================================
// USSA1976 Table-Based Model Tests
// ============================================

TEST(USSA1976, SeaLevelValues) {
    // Reference: US Standard Atmosphere 1976 at z=0
    // T = 288.15 K, P = 101325 Pa, rho = 1.225 kg/m³, a = 340.3 m/s
    EXPECT_NEAR(vulcan::ussa1976::temperature(Quantity<m>(0.0)).value(), 288.1,
                0.5);
    EXPECT_NEAR(vulcan::ussa1976::pressure(Quantity<m>(0.0)).value(), 101325,
                10);
    EXPECT_NEAR(vulcan::ussa1976::density(Quantity<m>(0.0)).value(), 1.225,
                0.001);
    EXPECT_NEAR(vulcan::ussa1976::speed_of_sound(Quantity<m>(0.0)).value(),
                340.3, 0.5);
    EXPECT_NEAR(vulcan::ussa1976::gravity(Quantity<m>(0.0)).value(), 9.807,
                0.001);
}

TEST(USSA1976, At10km) {
    // Reference: US Standard Atmosphere 1976 at z=10 km
    auto alt = Quantity<m>(10000.0);
    EXPECT_NEAR(vulcan::ussa1976::temperature(alt).value(), 223.3, 0.5);
    EXPECT_NEAR(vulcan::ussa1976::pressure(alt).value(), 26499, 50);
    EXPECT_NEAR(vulcan::ussa1976::density(alt).value(), 0.414, 0.01);
    EXPECT_NEAR(vulcan::ussa1976::speed_of_sound(alt).value(), 299.5, 0.5);
}

TEST(USSA1976, At50km) {
    // Reference: US Standard Atmosphere 1976 at z=50 km (stratopause)
    auto alt = Quantity<m>(50000.0);
    EXPECT_NEAR(vulcan::ussa1976::temperature(alt).value(), 270.65, 1.0);
    EXPECT_NEAR(vulcan::ussa1976::pressure(alt).value(), 79.8, 5);
    EXPECT_NEAR(vulcan::ussa1976::density(alt).value(), 1.03e-3, 1e-4);
}

TEST(USSA1976, Interpolation) {
    // Test that interpolation works for non-grid points
    auto alt = Quantity<m>(7500.0); // 7.5 km = grid point
    double T_grid = vulcan::ussa1976::temperature(alt).value();
    EXPECT_NEAR(T_grid, 239.5, 0.1);

    // 7.25 km - between 7.0 and 7.5 km
    auto alt_interp = Quantity<m>(7250.0);
    double T_interp = vulcan::ussa1976::temperature(alt_interp).value();
    EXPECT_GT(T_interp, 239.5); // Higher than 7.5 km
    EXPECT_LT(T_interp, 242.7); // Lower than 7.0 km
}

// ============================================
// State Struct Tests
// ============================================

TEST(USSA1976, StateStructSeaLevel) {
    auto state = vulcan::ussa1976::state(Quantity<m>(0.0));

    EXPECT_NEAR(state.temperature.value(), 288.1, 0.5);
    EXPECT_NEAR(state.pressure.value(), 101325, 10);
    EXPECT_NEAR(state.density.value(), 1.225, 0.001);
    EXPECT_NEAR(state.speed_of_sound.value(), 340.3, 0.5);
    EXPECT_NEAR(state.gravity.value(), 9.807, 0.001);
    // Reference: 17.89 μPa·s = 1.789e-5 Pa·s
    EXPECT_NEAR(state.dynamic_viscosity.value(), 1.789e-5, 1e-7);
}

TEST(USSA1976, StateStructAt20km) {
    auto state = vulcan::ussa1976::state(Quantity<m>(20000.0));

    EXPECT_NEAR(state.temperature.value(), 216.6, 0.5);
    EXPECT_NEAR(state.pressure.value(), 5529, 10);
    EXPECT_NEAR(state.density.value(), 0.089, 0.005);
    EXPECT_NEAR(state.speed_of_sound.value(), 295.1, 0.5);
    // Reference: 14.22 μPa·s = 1.422e-5 Pa·s
    EXPECT_NEAR(state.dynamic_viscosity.value(), 1.422e-5, 1e-7);
}

// ============================================
// Symbolic Tests
// ============================================

TEST(USSA1976, SymbolicEvaluation) {
    using SX = janus::SymbolicScalar;
    auto alt = janus::sym("altitude");
    auto T = vulcan::ussa1976::temperature(Quantity<m, SX>(alt));

    EXPECT_FALSE(T.value().is_constant());

    janus::Function f("T_ussa1976", {alt}, {T.value()});
    auto result = f({10000.0});
    EXPECT_NEAR(result[0](0, 0), 223.3, 0.5);
}

TEST(USSA1976, SymbolicGradient) {
    using SX = janus::SymbolicScalar;
    auto alt = janus::sym("altitude");
    auto rho = vulcan::ussa1976::density(Quantity<m, SX>(alt));

    auto drho_dalt = janus::jacobian(rho.value(), alt);

    janus::Function f("drho_dalt_ussa1976", {alt}, {drho_dalt});
    auto result = f({5000.0});

    // Density should decrease with altitude
    EXPECT_LT(result[0](0, 0), 0.0);
}

TEST(USSA1976, SymbolicState) {
    using SX = janus::SymbolicScalar;
    auto alt = janus::sym("altitude");
    auto state = vulcan::ussa1976::state(Quantity<m, SX>(alt));

    // Verify all fields are symbolic
    EXPECT_FALSE(state.temperature.value().is_constant());
    EXPECT_FALSE(state.pressure.value().is_constant());
    EXPECT_FALSE(state.density.value().is_constant());
    EXPECT_FALSE(state.speed_of_sound.value().is_constant());
    EXPECT_FALSE(state.gravity.value().is_constant());

    // Create function with all outputs
    janus::Function f("state_ussa1976", {alt},
                      {state.temperature.value(), state.pressure.value(),
                       state.density.value(), state.speed_of_sound.value(),
                       state.gravity.value()});

    auto result = f({0.0});
    EXPECT_NEAR(result[0](0, 0), 288.1, 0.5);   // T
    EXPECT_NEAR(result[1](0, 0), 101325, 10);   // P
    EXPECT_NEAR(result[2](0, 0), 1.225, 0.001); // rho
}

TEST(USSA1976, BatchQuery) {
    janus::NumericVector altitudes(4);
    altitudes << 0, 5000, 10000, 20000;

    auto temps = vulcan::ussa1976::detail::temperature_table()(altitudes);

    EXPECT_EQ(temps.size(), 4);
    EXPECT_NEAR(temps(0), 288.1, 0.5); // Sea level
    EXPECT_NEAR(temps(1), 255.7, 0.5); // 5 km
    EXPECT_NEAR(temps(2), 223.3, 0.5); // 10 km
    EXPECT_NEAR(temps(3), 216.6, 0.5); // 20 km
}
