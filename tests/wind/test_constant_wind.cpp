// Tests for Constant Wind and Wind Types
// Tests WindVector construction, speed/direction calculations, and symbolic
// evaluation
#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/wind/ConstantWind.hpp>
#include <vulcan/wind/WindTypes.hpp>

#include <cmath>

// ============================================================================
// WindVector Tests
// ============================================================================

TEST(WindVector, Construction) {
    vulcan::wind::WindVector<double> wind{
        .north = 10.0, .east = 5.0, .down = 0.0};
    EXPECT_DOUBLE_EQ(wind.north, 10.0);
    EXPECT_DOUBLE_EQ(wind.east, 5.0);
    EXPECT_DOUBLE_EQ(wind.down, 0.0);
}

TEST(WindVector, HorizontalSpeed) {
    vulcan::wind::WindVector<double> wind{
        .north = 3.0, .east = 4.0, .down = 0.0};
    EXPECT_DOUBLE_EQ(wind.horizontal_speed(), 5.0);
}

TEST(WindVector, TotalSpeed) {
    vulcan::wind::WindVector<double> wind{
        .north = 3.0, .east = 4.0, .down = 0.0};
    EXPECT_DOUBLE_EQ(wind.speed(), 5.0);

    // With vertical component
    vulcan::wind::WindVector<double> wind3d{
        .north = 2.0, .east = 2.0, .down = 1.0};
    EXPECT_DOUBLE_EQ(wind3d.speed(), 3.0);
}

TEST(WindVector, DirectionFrom) {
    // Wind from North (north component positive, wind blows TO the south)
    vulcan::wind::WindVector<double> north_wind{
        .north = 10.0, .east = 0.0, .down = 0.0};
    EXPECT_NEAR(north_wind.direction_from(), 0.0, 1e-10);

    // Wind from East
    vulcan::wind::WindVector<double> east_wind{
        .north = 0.0, .east = 10.0, .down = 0.0};
    EXPECT_NEAR(east_wind.direction_from(), M_PI / 2, 1e-10);

    // Wind from South
    vulcan::wind::WindVector<double> south_wind{
        .north = -10.0, .east = 0.0, .down = 0.0};
    EXPECT_NEAR(std::abs(south_wind.direction_from()), M_PI, 1e-10);
}

// ============================================================================
// Constant Wind from NED Tests
// ============================================================================

TEST(ConstantWind, FromNED) {
    auto wind = vulcan::constant_wind::from_ned(10.0, 5.0, 2.0);
    EXPECT_DOUBLE_EQ(wind.north, 10.0);
    EXPECT_DOUBLE_EQ(wind.east, 5.0);
    EXPECT_DOUBLE_EQ(wind.down, 2.0);
}

TEST(ConstantWind, FromNEDDefaultDown) {
    auto wind = vulcan::constant_wind::from_ned(10.0, 5.0);
    EXPECT_DOUBLE_EQ(wind.north, 10.0);
    EXPECT_DOUBLE_EQ(wind.east, 5.0);
    EXPECT_DOUBLE_EQ(wind.down, 0.0);
}

// ============================================================================
// Constant Wind from Speed/Direction Tests
// ============================================================================

TEST(ConstantWind, FromSpeedDirection_Northerly) {
    // Northerly wind: comes FROM North (direction_from = 0)
    // Blows TO South, so NED velocity is (-speed, 0, 0)
    double speed = 10.0;
    double direction = 0.0; // From North
    auto wind = vulcan::constant_wind::from_speed_direction(speed, direction);

    EXPECT_NEAR(wind.north, -10.0, 1e-10);
    EXPECT_NEAR(wind.east, 0.0, 1e-10);
    EXPECT_NEAR(wind.down, 0.0, 1e-10);
}

TEST(ConstantWind, FromSpeedDirection_Easterly) {
    // Easterly wind: comes FROM East (direction_from = π/2)
    // Blows TO West, so NED velocity is (0, -speed, 0)
    double speed = 10.0;
    double direction = M_PI / 2; // From East
    auto wind = vulcan::constant_wind::from_speed_direction(speed, direction);

    EXPECT_NEAR(wind.north, 0.0, 1e-10);
    EXPECT_NEAR(wind.east, -10.0, 1e-10);
}

TEST(ConstantWind, FromSpeedDirection_Southerly) {
    // Southerly wind: comes FROM South (direction_from = π)
    // Blows TO North, so NED velocity is (+speed, 0, 0)
    double speed = 10.0;
    double direction = M_PI; // From South
    auto wind = vulcan::constant_wind::from_speed_direction(speed, direction);

    EXPECT_NEAR(wind.north, 10.0, 1e-10);
    EXPECT_NEAR(wind.east, 0.0, 1e-10);
}

TEST(ConstantWind, FromSpeedDirection_Westerly) {
    // Westerly wind: comes FROM West (direction_from = 3π/2 or -π/2)
    // Blows TO East, so NED velocity is (0, +speed, 0)
    double speed = 10.0;
    double direction = 3.0 * M_PI / 2; // From West
    auto wind = vulcan::constant_wind::from_speed_direction(speed, direction);

    EXPECT_NEAR(wind.north, 0.0, 1e-10);
    EXPECT_NEAR(wind.east, 10.0, 1e-10);
}

TEST(ConstantWind, SpeedRecovery) {
    // Create wind, verify speed() matches input
    double speed = 15.0;
    double direction = M_PI / 4; // NE wind
    auto wind = vulcan::constant_wind::from_speed_direction(speed, direction);

    EXPECT_NEAR(wind.horizontal_speed(), speed, 1e-10);
}

// ============================================================================
// MIL-Spec Parameters Tests
// ============================================================================

TEST(TurbulenceParams, MilSpecLowAltitudeLight) {
    auto params = vulcan::wind::mil_spec_params(
        30.0, vulcan::wind::TurbulenceSeverity::Light);

    // Low altitude: scale lengths should vary
    EXPECT_GT(params.L_u, 0.0);
    EXPECT_GT(params.L_v, 0.0);
    EXPECT_GT(params.L_w, 0.0);
    EXPECT_GT(params.sigma_u, 0.0);
    EXPECT_GT(params.sigma_v, 0.0);
    EXPECT_GT(params.sigma_w, 0.0);
}

TEST(TurbulenceParams, MilSpecHighAltitude) {
    auto params = vulcan::wind::mil_spec_params(
        1000.0, vulcan::wind::TurbulenceSeverity::Moderate);

    // High altitude: scale lengths should be constant at 533.4 m
    EXPECT_NEAR(params.L_u, 533.4, 1.0);
    EXPECT_NEAR(params.L_v, 533.4, 1.0);
    EXPECT_NEAR(params.L_w, 533.4, 1.0);

    // Moderate turbulence at high altitude: σ ≈ 3.0 m/s
    EXPECT_NEAR(params.sigma_u, 3.0, 0.5);
    EXPECT_NEAR(params.sigma_v, 3.0, 0.5);
    EXPECT_NEAR(params.sigma_w, 3.0, 0.5);
}

TEST(TurbulenceParams, MilSpecSevere) {
    auto params = vulcan::wind::mil_spec_params(
        1000.0, vulcan::wind::TurbulenceSeverity::Severe);

    // Severe turbulence: σ ≈ 7.0 m/s
    EXPECT_NEAR(params.sigma_u, 7.0, 0.5);
    EXPECT_NEAR(params.sigma_v, 7.0, 0.5);
    EXPECT_NEAR(params.sigma_w, 7.0, 0.5);
}

// ============================================================================
// Symbolic Tests
// ============================================================================

TEST(ConstantWind, SymbolicFromNED) {
    auto north = janus::sym("north");
    auto east = janus::sym("east");
    auto down = janus::sym("down");

    auto wind = vulcan::constant_wind::from_ned(north, east, down);

    // Create function to evaluate
    janus::Function f("wind_ned", {north, east, down},
                      {wind.north, wind.east, wind.down});

    auto result = f({10.0, 5.0, 2.0});
    EXPECT_DOUBLE_EQ(result[0](0, 0), 10.0);
    EXPECT_DOUBLE_EQ(result[1](0, 0), 5.0);
    EXPECT_DOUBLE_EQ(result[2](0, 0), 2.0);
}

TEST(ConstantWind, SymbolicFromSpeedDirection) {
    auto speed = janus::sym("speed");
    auto dir = janus::sym("direction");

    auto wind = vulcan::constant_wind::from_speed_direction(speed, dir);

    janus::Function f("wind_sd", {speed, dir},
                      {wind.north, wind.east, wind.speed()});

    // Northerly wind: speed=10, dir=0
    auto result = f({10.0, 0.0});
    EXPECT_NEAR(result[0](0, 0), -10.0, 1e-10); // north component
    EXPECT_NEAR(result[1](0, 0), 0.0, 1e-10);   // east component
    EXPECT_NEAR(result[2](0, 0), 10.0, 1e-10);  // speed
}

TEST(WindVector, SymbolicSpeed) {
    auto n = janus::sym("n");
    auto e = janus::sym("e");
    auto d = janus::sym("d");

    vulcan::wind::WindVector<janus::SymbolicScalar> wind{
        .north = n, .east = e, .down = d};

    auto spd = wind.speed();
    janus::Function f("wind_speed", {n, e, d}, {spd});

    // 3-4-0 triangle
    auto result = f({3.0, 4.0, 0.0});
    EXPECT_DOUBLE_EQ(result[0](0, 0), 5.0);
}

TEST(ConstantWind, SymbolicGradient) {
    auto speed = janus::sym("speed");
    auto dir = janus::sym("direction");

    auto wind = vulcan::constant_wind::from_speed_direction(speed, dir);

    // d(north)/d(speed) at dir=0 should be -1
    auto dn_ds = janus::jacobian(wind.north, speed);
    janus::Function f("dn_ds", {speed, dir}, {dn_ds});

    auto result = f({10.0, 0.0});
    EXPECT_NEAR(result[0](0, 0), -1.0, 1e-10);
}
