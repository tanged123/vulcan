// Test suite for pseudo-5DOF guided vehicle dynamics
#include <vulcan/dynamics/Guided5Dof.hpp>

#include <gtest/gtest.h>

namespace vulcan::dynamics {

// =============================================================================
// Attitude Response Tests
// =============================================================================

TEST(AttitudeResponseTest, ZeroError) {
    double angle = 0.1;
    double angle_dot = 0.0;
    double angle_cmd = 0.1;
    double omega_n = 5.0;
    double zeta = 0.7;

    auto accel =
        attitude_response_accel(angle, angle_dot, angle_cmd, omega_n, zeta);

    EXPECT_NEAR(accel, 0.0, 1e-12);
}

TEST(AttitudeResponseTest, PositiveError) {
    double angle = 0.0;
    double angle_dot = 0.0;
    double angle_cmd = 0.1;
    double omega_n = 5.0;
    double zeta = 0.7;

    auto accel =
        attitude_response_accel(angle, angle_dot, angle_cmd, omega_n, zeta);

    // ω²(cmd - angle) = 25 * 0.1 = 2.5
    EXPECT_NEAR(accel, 2.5, 1e-12);
}

TEST(AttitudeResponseTest, DampingTerm) {
    double angle = 0.1;
    double angle_dot = 1.0;
    double angle_cmd = 0.1;
    double omega_n = 5.0;
    double zeta = 0.5;

    auto accel =
        attitude_response_accel(angle, angle_dot, angle_cmd, omega_n, zeta);

    // -2ζω*angle_dot = -2*0.5*5*1 = -5
    EXPECT_NEAR(accel, -5.0, 1e-12);
}

// =============================================================================
// Bank-to-Turn Direction Tests
// =============================================================================

TEST(BttDirectionTest, ThrustNorthLevel) {
    double gamma = 0.0;
    double chi = 0.0;

    auto dir = thrust_direction_btt(gamma, chi);

    EXPECT_NEAR(dir(0), 1.0, 1e-12); // North
    EXPECT_NEAR(dir(1), 0.0, 1e-12);
    EXPECT_NEAR(dir(2), 0.0, 1e-12);
}

TEST(BttDirectionTest, ThrustEastLevel) {
    double gamma = 0.0;
    double chi = M_PI / 2;

    auto dir = thrust_direction_btt(gamma, chi);

    EXPECT_NEAR(dir(0), 0.0, 1e-12);
    EXPECT_NEAR(dir(1), 1.0, 1e-12); // East
    EXPECT_NEAR(dir(2), 0.0, 1e-12);
}

TEST(BttDirectionTest, ThrustClimbing45) {
    double gamma = M_PI / 4; // 45° climb
    double chi = 0.0;

    auto dir = thrust_direction_btt(gamma, chi);

    double sqrt2_2 = std::sqrt(2.0) / 2.0;
    EXPECT_NEAR(dir(0), sqrt2_2, 1e-10);
    EXPECT_NEAR(dir(1), 0.0, 1e-12);
    EXPECT_NEAR(dir(2), -sqrt2_2, 1e-10); // Up
}

TEST(BttDirectionTest, LiftZeroBank) {
    double gamma = 0.0;
    double chi = 0.0;
    double phi = 0.0;

    auto dir = lift_direction_btt(gamma, chi, phi);

    // Zero bank, level flight: lift is pure up
    EXPECT_NEAR(dir(0), 0.0, 1e-12);
    EXPECT_NEAR(dir(1), 0.0, 1e-12);
    EXPECT_NEAR(dir(2), 1.0, 1e-10); // cos(0) = 1 for vertical component
}

TEST(BttDirectionTest, LiftBanked90) {
    double gamma = 0.0;
    double chi = 0.0;      // Heading North
    double phi = M_PI / 2; // 90° bank right

    auto dir = lift_direction_btt(gamma, chi, phi);

    // Full bank: lift points East (right from velocity direction)
    EXPECT_NEAR(dir(0), 0.0, 1e-10);
    EXPECT_NEAR(dir(1), 1.0, 1e-10); // Right of velocity = East
    EXPECT_NEAR(dir(2), 0.0, 1e-10);
}

// =============================================================================
// Skid-to-Turn Direction Tests
// =============================================================================

TEST(SttDirectionTest, ThrustNoseNorth) {
    double theta = 0.0;
    double psi = 0.0;

    auto dir = thrust_direction_stt(theta, psi);

    EXPECT_NEAR(dir(0), 1.0, 1e-12);
    EXPECT_NEAR(dir(1), 0.0, 1e-12);
    EXPECT_NEAR(dir(2), 0.0, 1e-12);
}

TEST(SttDirectionTest, ThrustNoseUp) {
    double theta = M_PI / 2;
    double psi = 0.0;

    auto dir = thrust_direction_stt(theta, psi);

    EXPECT_NEAR(dir(0), 0.0, 1e-10);
    EXPECT_NEAR(dir(1), 0.0, 1e-12);
    EXPECT_NEAR(dir(2), -1.0, 1e-10); // Up
}

TEST(SttDirectionTest, SideForce) {
    double theta = 0.0;
    double psi = 0.0;

    auto dir = side_force_direction_stt(theta, psi);

    // Body Y when pointing North = East
    EXPECT_NEAR(dir(0), 0.0, 1e-12);
    EXPECT_NEAR(dir(1), 1.0, 1e-12);
    EXPECT_NEAR(dir(2), 0.0, 1e-12);
}

TEST(SttDirectionTest, NormalForceLevel) {
    double theta = 0.0;
    double psi = 0.0;

    auto dir = normal_force_direction_stt(theta, psi);

    // Body -Z when level = Up
    EXPECT_NEAR(dir(0), 0.0, 1e-12);
    EXPECT_NEAR(dir(1), 0.0, 1e-12);
    EXPECT_NEAR(dir(2), 1.0, 1e-12); // cos(0) = 1
}

// =============================================================================
// Velocity Derivative Tests
// =============================================================================

TEST(VelocityDotBttTest, PureThrust) {
    double thrust = 1000.0;
    double drag = 0.0;
    double lift = 0.0;
    double mass = 100.0;
    double gravity = 0.0;
    double gamma = 0.0;
    double chi = 0.0;
    double phi = 0.0;

    auto v_dot =
        velocity_dot_btt(thrust, drag, lift, mass, gravity, gamma, chi, phi);

    // Pure thrust North
    EXPECT_NEAR(v_dot(0), 10.0, 1e-10);
    EXPECT_NEAR(v_dot(1), 0.0, 1e-12);
    EXPECT_NEAR(v_dot(2), 0.0, 1e-12);
}

TEST(VelocityDotBttTest, WithGravity) {
    double thrust = 0.0;
    double drag = 0.0;
    double lift = 0.0;
    double mass = 100.0;
    double gravity = 9.81;
    double gamma = 0.0;
    double chi = 0.0;
    double phi = 0.0;

    auto v_dot =
        velocity_dot_btt(thrust, drag, lift, mass, gravity, gamma, chi, phi);

    // Only gravity
    EXPECT_NEAR(v_dot(0), 0.0, 1e-12);
    EXPECT_NEAR(v_dot(1), 0.0, 1e-12);
    EXPECT_NEAR(v_dot(2), 9.81, 1e-10);
}

TEST(VelocityDotSttTest, PureThrust) {
    double thrust = 1000.0;
    double drag = 0.0;
    double normal = 0.0;
    double side = 0.0;
    double mass = 100.0;
    double gravity = 0.0;
    double theta = 0.0;
    double psi = 0.0;

    auto v_dot =
        velocity_dot_stt(thrust, drag, normal, side, mass, gravity, theta, psi);

    EXPECT_NEAR(v_dot(0), 10.0, 1e-10);
    EXPECT_NEAR(v_dot(1), 0.0, 1e-12);
    EXPECT_NEAR(v_dot(2), 0.0, 1e-12);
}

// =============================================================================
// Flight Path Rate Tests
// =============================================================================

TEST(GammaDotTest, LevelUnaccelerated) {
    double lift = 9810.0; // = weight
    double weight = 9810.0;
    double mass = 1000.0;
    double velocity = 100.0;
    double gamma = 0.0;
    double phi = 0.0;

    auto gd = gamma_dot(lift, weight, mass, velocity, gamma, phi);

    // L*cos(0) - W*cos(0) = 0
    EXPECT_NEAR(gd, 0.0, 1e-10);
}

TEST(GammaDotTest, PositiveLift) {
    double lift = 2 * 9810.0; // 2g lift
    double weight = 9810.0;
    double mass = 1000.0;
    double velocity = 100.0;
    double gamma = 0.0;
    double phi = 0.0;

    auto gd = gamma_dot(lift, weight, mass, velocity, gamma, phi);

    // (2W - W) / (m*V) = W / (m*V) = 9810 / 100000 ≈ 0.0981
    EXPECT_GT(gd, 0.0); // Climbing
}

TEST(ChiDotBttTest, ZeroBank) {
    double lift = 10000.0;
    double mass = 1000.0;
    double velocity = 100.0;
    double gamma = 0.0;
    double phi = 0.0;

    auto cd = chi_dot_btt(lift, mass, velocity, gamma, phi);

    // sin(0) = 0, no turn
    EXPECT_NEAR(cd, 0.0, 1e-12);
}

TEST(ChiDotBttTest, BankedTurn) {
    double lift = 10000.0;
    double mass = 1000.0;
    double velocity = 100.0;
    double gamma = 0.0;
    double phi = M_PI / 4; // 45° bank

    auto cd = chi_dot_btt(lift, mass, velocity, gamma, phi);

    // L*sin(45) / (m*V) = 10000*0.707 / 100000 ≈ 0.0707 rad/s
    EXPECT_GT(cd, 0.0); // Turning
}

// =============================================================================
// Utility Tests
// =============================================================================

TEST(LoadFactorTest, FromLift) {
    double lift = 10000.0;
    double weight = 5000.0;

    auto n = load_factor_from_lift(lift, weight);

    EXPECT_NEAR(n, 2.0, 1e-10);
}

TEST(BankForTurnTest, LevelTurn) {
    double velocity = 100.0;
    double chi_dot = 0.1; // rad/s
    double gravity = 10.0;

    auto phi = bank_for_turn_rate(velocity, chi_dot, gravity);

    // tan(φ) = V*χ̇/g = 100*0.1/10 = 1 -> φ = 45°
    EXPECT_NEAR(phi, M_PI / 4, 1e-10);
}

// =============================================================================
// Symbolic Tests
// =============================================================================

TEST(Guided5DofSymbolicTest, AttitudeResponse) {
    auto angle = janus::sym("angle");
    auto angle_dot = janus::sym("angle_dot");
    auto angle_cmd = janus::sym("angle_cmd");
    auto omega_n = janus::sym("omega_n");
    auto zeta = janus::sym("zeta");

    auto accel =
        attitude_response_accel(angle, angle_dot, angle_cmd, omega_n, zeta);

    janus::Function f("attitude_accel",
                      {angle, angle_dot, angle_cmd, omega_n, zeta}, {accel});

    auto result = f({0.0, 0.5, 0.1, 5.0, 0.7});

    double accel_num = attitude_response_accel(0.0, 0.5, 0.1, 5.0, 0.7);
    EXPECT_NEAR(result[0](0, 0), accel_num, 1e-10);
}

TEST(Guided5DofSymbolicTest, ThrustDirectionBtt) {
    auto gamma = janus::sym("gamma");
    auto chi = janus::sym("chi");

    auto dir = thrust_direction_btt(gamma, chi);

    janus::Function f("thrust_dir_btt", {gamma, chi}, {dir(0), dir(1), dir(2)});

    auto result = f({M_PI / 6, M_PI / 4});

    auto dir_num = thrust_direction_btt(M_PI / 6, M_PI / 4);
    EXPECT_NEAR(result[0](0, 0), dir_num(0), 1e-10);
    EXPECT_NEAR(result[1](0, 0), dir_num(1), 1e-10);
    EXPECT_NEAR(result[2](0, 0), dir_num(2), 1e-10);
}

TEST(Guided5DofSymbolicTest, VelocityDotBtt) {
    auto T = janus::sym("T");
    auto D = janus::sym("D");
    auto L = janus::sym("L");
    auto m = janus::sym("m");
    auto g = janus::sym("g");
    auto gamma = janus::sym("gamma");
    auto chi = janus::sym("chi");
    auto phi = janus::sym("phi");

    auto v_dot = velocity_dot_btt(T, D, L, m, g, gamma, chi, phi);

    janus::Function f("vdot_btt", {T, D, L, m, g, gamma, chi, phi},
                      {v_dot(0), v_dot(1), v_dot(2)});

    auto result = f({1000.0, 100.0, 5000.0, 100.0, 9.81, 0.1, 0.2, 0.3});

    auto v_dot_num =
        velocity_dot_btt(1000.0, 100.0, 5000.0, 100.0, 9.81, 0.1, 0.2, 0.3);
    EXPECT_NEAR(result[0](0, 0), v_dot_num(0), 1e-8);
    EXPECT_NEAR(result[1](0, 0), v_dot_num(1), 1e-8);
    EXPECT_NEAR(result[2](0, 0), v_dot_num(2), 1e-8);
}

} // namespace vulcan::dynamics
