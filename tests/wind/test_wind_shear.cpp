// Tests for Wind Shear Models
// Tests linear, power law, and logarithmic wind profiles
#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/wind/WindShear.hpp>

#include <cmath>

// ============================================================================
// Linear Wind Shear Tests
// ============================================================================

TEST(WindShear, LinearAtBaseAltitude) {
    // At base altitude, should return base wind
    double base_wind = 10.0;
    double base_alt = 100.0;
    double shear_rate = 0.01; // 1 m/s per 100m

    double wind =
        vulcan::wind_shear::linear(base_alt, base_wind, base_alt, shear_rate);
    EXPECT_DOUBLE_EQ(wind, base_wind);
}

TEST(WindShear, LinearAboveBase) {
    double base_wind = 10.0;
    double base_alt = 100.0;
    double shear_rate = 0.01; // 1 m/s per 100m

    double wind =
        vulcan::wind_shear::linear(200.0, base_wind, base_alt, shear_rate);
    // At 200m (100m above base): 10 + 0.01 * 100 = 11 m/s
    EXPECT_DOUBLE_EQ(wind, 11.0);
}

TEST(WindShear, LinearBelowBase) {
    double base_wind = 10.0;
    double base_alt = 100.0;
    double shear_rate = 0.01;

    double wind =
        vulcan::wind_shear::linear(0.0, base_wind, base_alt, shear_rate);
    // At 0m (100m below base): 10 + 0.01 * (-100) = 9 m/s
    EXPECT_DOUBLE_EQ(wind, 9.0);
}

TEST(WindShear, LinearVectorPreservesDirection) {
    vulcan::wind::WindVector<double> base{
        .north = 6.0, .east = 8.0, .down = 0.0};
    double base_alt = 10.0;
    double shear_rate = 0.01;

    auto wind =
        vulcan::wind_shear::linear_vector(20.0, base, base_alt, shear_rate);

    // Base speed = 10 m/s, at 20m: 10 + 0.01 * 10 = 10.1 m/s
    // Scale = 10.1 / 10 = 1.01
    EXPECT_NEAR(wind.north, 6.0 * 1.01, 1e-10);
    EXPECT_NEAR(wind.east, 8.0 * 1.01, 1e-10);
}

// ============================================================================
// Power Law Wind Profile Tests
// ============================================================================

TEST(WindShear, PowerLawAtReference) {
    // At reference altitude, should return reference wind
    double ref_wind = 10.0;
    double ref_alt = 10.0;

    double wind = vulcan::wind_shear::power_law(ref_alt, ref_wind, ref_alt);
    EXPECT_DOUBLE_EQ(wind, ref_wind);
}

TEST(WindShear, PowerLawDoubleHeight) {
    double ref_wind = 10.0;
    double ref_alt = 10.0;
    double exponent = 1.0 / 7.0; // Unstable

    double wind =
        vulcan::wind_shear::power_law(20.0, ref_wind, ref_alt, exponent);

    // V(20) = 10 * (20/10)^(1/7) = 10 * 2^(1/7)
    double expected = ref_wind * std::pow(2.0, exponent);
    EXPECT_NEAR(wind, expected, 1e-10);
}

TEST(WindShear, PowerLawDifferentExponents) {
    double ref_wind = 10.0;
    double ref_alt = 10.0;
    double alt = 100.0;

    // Unstable (1/7)
    double wind_unstable = vulcan::wind_shear::power_law(
        alt, ref_wind, ref_alt, vulcan::wind_shear::exponent::UNSTABLE);

    // Neutral (1/4)
    double wind_neutral = vulcan::wind_shear::power_law(
        alt, ref_wind, ref_alt, vulcan::wind_shear::exponent::NEUTRAL);

    // Stable (1/3)
    double wind_stable = vulcan::wind_shear::power_law(
        alt, ref_wind, ref_alt, vulcan::wind_shear::exponent::STABLE);

    // Higher exponent = larger wind at same height
    EXPECT_LT(wind_unstable, wind_neutral);
    EXPECT_LT(wind_neutral, wind_stable);
}

TEST(WindShear, PowerLawVector) {
    vulcan::wind::WindVector<double> base{
        .north = 6.0, .east = 8.0, .down = 0.0};
    double ref_alt = 10.0;

    auto wind = vulcan::wind_shear::power_law_vector(100.0, base, ref_alt);

    // Check that speed increased (altitude > ref)
    EXPECT_GT(wind.speed(), 10.0);

    // Check that direction is preserved
    double base_dir = std::atan2(base.east, base.north);
    double new_dir = std::atan2(wind.east, wind.north);
    EXPECT_NEAR(new_dir, base_dir, 1e-10);
}

// ============================================================================
// Logarithmic Wind Profile Tests
// ============================================================================

TEST(WindShear, LogarithmicBasic) {
    double friction_vel = 0.5;                                      // m/s
    double roughness = vulcan::wind_shear::roughness::OPEN_TERRAIN; // 0.03 m

    // At 10m with no displacement
    double wind =
        vulcan::wind_shear::logarithmic(10.0, friction_vel, roughness);

    // V = (0.5 / 0.41) * ln(10 / 0.03) = 1.22 * ln(333.3) ≈ 7.1 m/s
    double expected = (friction_vel / 0.41) * std::log(10.0 / roughness);
    EXPECT_NEAR(wind, expected, 0.01);
}

TEST(WindShear, LogarithmicDifferentRoughness) {
    double friction_vel = 0.5;
    double alt = 100.0;

    double wind_water = vulcan::wind_shear::logarithmic(
        alt, friction_vel, vulcan::wind_shear::roughness::OPEN_WATER);
    double wind_rural = vulcan::wind_shear::logarithmic(
        alt, friction_vel, vulcan::wind_shear::roughness::RURAL);
    double wind_urban = vulcan::wind_shear::logarithmic(
        alt, friction_vel, vulcan::wind_shear::roughness::URBAN);

    // Smoother surface = higher wind at same height
    EXPECT_GT(wind_water, wind_rural);
    EXPECT_GT(wind_rural, wind_urban);
}

TEST(WindShear, LogarithmicWithDisplacement) {
    double friction_vel = 0.5;
    double roughness = 0.5;    // Suburban
    double displacement = 5.0; // e.g., vegetation height

    double wind = vulcan::wind_shear::logarithmic(15.0, friction_vel, roughness,
                                                  displacement);

    // V = (0.5 / 0.41) * ln((15 - 5) / 0.5) = 1.22 * ln(20)
    double expected = (friction_vel / 0.41) * std::log(10.0 / roughness);
    EXPECT_NEAR(wind, expected, 0.01);
}

TEST(WindShear, FrictionVelocityFromRef) {
    double ref_wind = 10.0;
    double ref_alt = 10.0;
    double roughness = 0.03;

    double u_star = vulcan::wind_shear::friction_velocity_from_ref(
        ref_wind, ref_alt, roughness);

    // Verify by computing wind at ref height with computed u*
    double wind_check =
        vulcan::wind_shear::logarithmic(ref_alt, u_star, roughness);
    EXPECT_NEAR(wind_check, ref_wind, 1e-10);
}

// ============================================================================
// Symbolic Tests
// ============================================================================

TEST(WindShear, SymbolicLinear) {
    auto alt = janus::sym("altitude");
    auto wind = vulcan::wind_shear::linear(alt, 10.0, 0.0, 0.01);

    janus::Function f("linear_wind", {alt}, {wind});

    auto result = f({100.0});
    // 10 + 0.01 * 100 = 11
    EXPECT_NEAR(result[0](0, 0), 11.0, 1e-10);
}

TEST(WindShear, SymbolicPowerLaw) {
    auto alt = janus::sym("altitude");
    auto wind = vulcan::wind_shear::power_law(alt, 10.0, 10.0);

    janus::Function f("power_wind", {alt}, {wind});

    // At alt=10, should be 10.0
    auto result = f({10.0});
    EXPECT_NEAR(result[0](0, 0), 10.0, 1e-10);

    // At alt=80, should be 10 * 8^(1/7)
    result = f({80.0});
    double expected = 10.0 * std::pow(8.0, 1.0 / 7.0);
    EXPECT_NEAR(result[0](0, 0), expected, 1e-10);
}

TEST(WindShear, SymbolicLogarithmic) {
    auto alt = janus::sym("altitude");
    auto wind = vulcan::wind_shear::logarithmic(alt, 0.5, 0.03);

    janus::Function f("log_wind", {alt}, {wind});

    auto result = f({10.0});
    double expected = (0.5 / 0.41) * std::log(10.0 / 0.03);
    EXPECT_NEAR(result[0](0, 0), expected, 0.01);
}

TEST(WindShear, SymbolicGradient) {
    auto alt = janus::sym("altitude");
    auto wind = vulcan::wind_shear::power_law(alt, 10.0, 10.0, 1.0 / 7.0);

    // dV/dh using Jacobian
    auto dv_dh = janus::jacobian(wind, alt);

    janus::Function f("dv_dh", {alt}, {dv_dh});

    // Gradient should be positive (wind increases with altitude)
    auto result = f({50.0});
    EXPECT_GT(result[0](0, 0), 0.0);
}

TEST(WindShear, SymbolicPowerLawVectorSpeed) {
    auto alt = janus::sym("altitude");
    vulcan::wind::WindVector<double> base{
        .north = 6.0, .east = 8.0, .down = 0.0};

    auto wind =
        vulcan::wind_shear::power_law_vector(alt, base, 10.0, 1.0 / 7.0);
    auto spd = wind.speed();

    janus::Function f("wind_speed", {alt}, {spd});

    // At 10m, speed should be 10.0
    auto result = f({10.0});
    EXPECT_NEAR(result[0](0, 0), 10.0, 1e-10);

    // At 100m, should be 10 * 10^(1/7)
    result = f({100.0});
    double expected = 10.0 * std::pow(10.0, 1.0 / 7.0);
    EXPECT_NEAR(result[0](0, 0), expected, 1e-10);
}
