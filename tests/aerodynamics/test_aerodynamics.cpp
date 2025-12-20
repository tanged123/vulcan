// Tests for Vulcan Aerodynamics Module
#include <vulcan/aerodynamics/Aerodynamics.hpp>
#include <vulcan/atmosphere/USSA1976.hpp>
#include <vulcan/core/Constants.hpp>

#include <janus/janus.hpp>

#include <gtest/gtest.h>

namespace {

using namespace vulcan;
using namespace vulcan::aero;

// =============================================================================
// Dynamic Pressure Tests
// =============================================================================

TEST(Aerodynamics, DynamicPressureSeaLevel100ms) {
    // q = 0.5 * rho * V^2
    // At sea level: rho = 1.225 kg/m³, V = 100 m/s
    // Expected: q = 0.5 * 1.225 * 10000 = 6125 Pa
    double q = dynamic_pressure(constants::atmosphere::rho0, 100.0);
    EXPECT_NEAR(q, 6125.0, 1.0);
}

TEST(Aerodynamics, DynamicPressureZeroVelocity) {
    double q = dynamic_pressure(1.225, 0.0);
    EXPECT_NEAR(q, 0.0, 1e-10);
}

// =============================================================================
// Mach Number Tests
// =============================================================================

TEST(Aerodynamics, MachNumberSeaLevel) {
    // At sea level: a ≈ 340.3 m/s
    double M = mach_number(340.0, 340.3);
    EXPECT_NEAR(M, 1.0, 0.01);
}

TEST(Aerodynamics, MachNumberSupersonic) {
    double M = mach_number(680.0, 340.0);
    EXPECT_NEAR(M, 2.0, 0.01);
}

TEST(Aerodynamics, MachNumberSubsonic) {
    double M = mach_number(170.0, 340.0);
    EXPECT_NEAR(M, 0.5, 0.01);
}

// =============================================================================
// Reynolds Number Tests
// =============================================================================

TEST(Aerodynamics, ReynoldsNumberAircraft) {
    // Re = rho * V * L / mu
    // At sea level: rho = 1.225, V = 100 m/s, L = 1 m
    // mu ≈ 1.789e-5 Pa·s
    // Expected: Re ≈ 1.225 * 100 * 1 / 1.789e-5 ≈ 6.85e6
    double Re = reynolds_number(1.225, 100.0, 1.0, 1.789e-5);
    EXPECT_NEAR(Re, 6.85e6, 0.05e6);
}

TEST(Aerodynamics, ReynoldsNumberSmallScale) {
    // L = 0.1 m (small UAV)
    double Re = reynolds_number(1.225, 10.0, 0.1, 1.789e-5);
    EXPECT_NEAR(Re, 6.85e4, 0.05e4);
}

// =============================================================================
// Airspeed Tests
// =============================================================================

TEST(Aerodynamics, AirspeedNoWind) {
    Vec3<double> v_ground;
    v_ground << 100.0, 0.0, 0.0;
    Vec3<double> v_wind = Vec3<double>::Zero();

    double V = airspeed(v_ground, v_wind);
    EXPECT_NEAR(V, 100.0, 1e-10);
}

TEST(Aerodynamics, AirspeedHeadwind) {
    Vec3<double> v_ground;
    v_ground << 100.0, 0.0, 0.0;
    Vec3<double> v_wind;
    v_wind << -20.0, 0.0, 0.0; // Headwind

    double V = airspeed(v_ground, v_wind);
    EXPECT_NEAR(V, 120.0, 1e-10);
}

TEST(Aerodynamics, AirspeedTailwind) {
    Vec3<double> v_ground;
    v_ground << 100.0, 0.0, 0.0;
    Vec3<double> v_wind;
    v_wind << 20.0, 0.0, 0.0; // Tailwind

    double V = airspeed(v_ground, v_wind);
    EXPECT_NEAR(V, 80.0, 1e-10);
}

// =============================================================================
// Aerodynamic Angles Tests
// =============================================================================

TEST(Aerodynamics, AeroAnglesLevelFlight) {
    // Flying straight forward, no sideslip
    Vec3<double> v_body;
    v_body << 100.0, 0.0, 0.0;

    Vec2<double> angles = aero_angles(v_body);
    EXPECT_NEAR(angles(0), 0.0, 1e-10); // alpha
    EXPECT_NEAR(angles(1), 0.0, 1e-10); // beta
}

TEST(Aerodynamics, AeroAnglesPositiveAoA) {
    // Flow coming from below -> positive alpha
    Vec3<double> v_body;
    v_body << 100.0, 0.0, 10.0; // vz positive = flow from below

    Vec2<double> angles = aero_angles(v_body);
    EXPECT_GT(angles(0), 0.0); // alpha > 0
    EXPECT_NEAR(angles(0), std::atan2(10.0, 100.0), 1e-10);
    EXPECT_NEAR(angles(1), 0.0, 0.01); // beta ≈ 0
}

TEST(Aerodynamics, AeroAnglesPositiveSideslip) {
    // Flow coming from right -> positive beta
    Vec3<double> v_body;
    v_body << 100.0, 10.0, 0.0;

    Vec2<double> angles = aero_angles(v_body);
    EXPECT_NEAR(angles(0), 0.0, 0.01); // alpha ≈ 0
    EXPECT_GT(angles(1), 0.0);         // beta > 0
}

TEST(Aerodynamics, AeroAnglesZeroVelocity) {
    Vec3<double> v_body = Vec3<double>::Zero();

    Vec2<double> angles = aero_angles(v_body);
    EXPECT_NEAR(angles(0), 0.0, 1e-10); // No NaN
    EXPECT_NEAR(angles(1), 0.0, 1e-10);
}

// =============================================================================
// AeroState Tests
// =============================================================================

TEST(Aerodynamics, AeroStateComplete) {
    Vec3<double> v_body;
    v_body << 100.0, 5.0, 10.0;

    auto state = aero_state(1.225, 340.0, 1.789e-5, v_body, 1.0);

    EXPECT_GT(state.dynamic_pressure, 0.0);
    EXPECT_GT(state.mach, 0.0);
    EXPECT_GT(state.reynolds, 0.0);
    EXPECT_GT(state.airspeed, 0.0);
}

// =============================================================================
// Symbolic Mode Tests
// =============================================================================

TEST(Aerodynamics, SymbolicDynamicPressure) {
    using Scalar = janus::SymbolicScalar;

    auto rho = janus::sym("rho");
    auto V = janus::sym("V");

    auto q = dynamic_pressure(rho, V);

    // Create function and evaluate
    janus::Function f("q", {rho, V}, {q});
    auto result = f({1.225, 100.0});

    EXPECT_NEAR(result[0](0, 0), 6125.0, 1.0);
}

TEST(Aerodynamics, SymbolicMachNumber) {
    using Scalar = janus::SymbolicScalar;

    auto V = janus::sym("V");
    auto a = janus::sym("a");

    auto M = mach_number(V, a);

    janus::Function f("M", {V, a}, {M});
    auto result = f({340.0, 340.0});

    EXPECT_NEAR(result[0](0, 0), 1.0, 1e-6);
}

TEST(Aerodynamics, SymbolicReynoldsNumber) {
    using Scalar = janus::SymbolicScalar;

    auto rho = janus::sym("rho");
    auto V = janus::sym("V");
    auto L = janus::sym("L");
    auto mu = janus::sym("mu");

    auto Re = reynolds_number(rho, V, L, mu);

    janus::Function f("Re", {rho, V, L, mu}, {Re});
    auto result = f({1.225, 100.0, 1.0, 1.789e-5});

    EXPECT_NEAR(result[0](0, 0), 6.85e6, 0.05e6);
}

TEST(Aerodynamics, SymbolicAeroAngles) {
    using Scalar = janus::SymbolicScalar;

    auto vx = janus::sym("vx");
    auto vy = janus::sym("vy");
    auto vz = janus::sym("vz");

    Vec3<Scalar> v_body;
    v_body << vx, vy, vz;

    auto angles = aero_angles(v_body);

    janus::Function f("angles", {vx, vy, vz}, {angles(0), angles(1)});
    auto result = f({100.0, 0.0, 10.0});

    EXPECT_NEAR(result[0](0, 0), std::atan2(10.0, 100.0), 1e-6);
    EXPECT_NEAR(result[1](0, 0), 0.0, 1e-6);
}

} // namespace
