// Test suite for rail launch dynamics
#include <vulcan/dynamics/RailLaunch.hpp>

#include <gtest/gtest.h>

namespace vulcan::dynamics {

// =============================================================================
// Rail Direction Tests
// =============================================================================

TEST(RailDirectionTest, NorthHorizontal) {
    double azimuth = 0.0;
    double elevation = 0.0;

    auto dir = rail_direction_ned(azimuth, elevation);

    EXPECT_NEAR(dir(0), 1.0, 1e-12); // Pure North
    EXPECT_NEAR(dir(1), 0.0, 1e-12);
    EXPECT_NEAR(dir(2), 0.0, 1e-12);
}

TEST(RailDirectionTest, EastHorizontal) {
    double azimuth = M_PI / 2; // 90° East
    double elevation = 0.0;

    auto dir = rail_direction_ned(azimuth, elevation);

    EXPECT_NEAR(dir(0), 0.0, 1e-12);
    EXPECT_NEAR(dir(1), 1.0, 1e-12); // Pure East
    EXPECT_NEAR(dir(2), 0.0, 1e-12);
}

TEST(RailDirectionTest, NorthVertical) {
    double azimuth = 0.0;
    double elevation = M_PI / 2; // Straight up

    auto dir = rail_direction_ned(azimuth, elevation);

    EXPECT_NEAR(dir(0), 0.0, 1e-12);
    EXPECT_NEAR(dir(1), 0.0, 1e-12);
    EXPECT_NEAR(dir(2), -1.0, 1e-12); // Up is -Z in NED
}

TEST(RailDirectionTest, NorthElevated45) {
    double azimuth = 0.0;
    double elevation = M_PI / 4; // 45°

    auto dir = rail_direction_ned(azimuth, elevation);

    double sqrt2_2 = std::sqrt(2.0) / 2.0;
    EXPECT_NEAR(dir(0), sqrt2_2, 1e-10); // North component
    EXPECT_NEAR(dir(1), 0.0, 1e-12);
    EXPECT_NEAR(dir(2), -sqrt2_2, 1e-10); // Up component
}

// =============================================================================
// Rail Acceleration Tests
// =============================================================================

TEST(RailAccelTest, PureThrust) {
    // Horizontal rail, no friction, pure thrust
    double s_dot = 0.0;
    double force_along = 1000.0;
    double force_perp_mag = 0.0;
    double mass = 100.0;
    double gravity = 10.0;
    double elevation = 0.0;
    double friction = 0.0;

    auto accel = rail_acceleration(s_dot, force_along, force_perp_mag, mass,
                                   gravity, elevation, friction);

    // a = F/m = 1000/100 = 10
    EXPECT_NEAR(accel, 10.0, 1e-12);
}

TEST(RailAccelTest, VerticalRail) {
    // Vertical rail (up), no thrust
    double s_dot = 0.0;
    double force_along = 0.0;
    double force_perp_mag = 0.0;
    double mass = 100.0;
    double gravity = 10.0;
    double elevation = M_PI / 2; // Vertical
    double friction = 0.0;

    auto accel = rail_acceleration(s_dot, force_along, force_perp_mag, mass,
                                   gravity, elevation, friction);

    // a = -g*sin(90°) = -10
    EXPECT_NEAR(accel, -10.0, 1e-10);
}

TEST(RailAccelTest, WithFriction) {
    // Moving forward with friction
    double s_dot = 5.0; // Moving forward
    double force_along = 1000.0;
    double force_perp_mag = 500.0; // Normal force
    double mass = 100.0;
    double gravity = 10.0;
    double elevation = 0.0;
    double friction = 0.1;

    auto accel = rail_acceleration(s_dot, force_along, force_perp_mag, mass,
                                   gravity, elevation, friction);

    // a = (F - μN)/m = (1000 - 0.1*500)/100 = 9.5
    EXPECT_NEAR(accel, 9.5, 1e-10);
}

TEST(RailAccelTest, FromBodyForce) {
    // Test body force decomposition
    Vec3<double> force_body{1000.0, 100.0, 50.0};
    double s_dot = 0.0;
    double mass = 100.0;
    double gravity = 10.0;
    double elevation = 0.0;
    double friction = 0.0;

    auto accel = rail_acceleration_body(s_dot, force_body, mass, gravity,
                                        elevation, friction);

    // a = Fx/m = 1000/100 = 10 (no friction, horizontal)
    EXPECT_NEAR(accel, 10.0, 1e-10);
}

// =============================================================================
// On-Rail Check Tests
// =============================================================================

TEST(OnRailTest, OnRail) {
    double s = 0.5;
    double rail_length = 5.0;

    auto result = on_rail(s, rail_length);

    EXPECT_NEAR(result, 1.0, 1e-12);
}

TEST(OnRailTest, AtEnd) {
    double s = 5.0; // Exactly at end
    double rail_length = 5.0;

    auto result = on_rail(s, rail_length);

    EXPECT_NEAR(result, 0.0, 1e-12); // Departed
}

TEST(OnRailTest, BeforeStart) {
    double s = -0.1;
    double rail_length = 5.0;

    auto result = on_rail(s, rail_length);

    EXPECT_NEAR(result, 0.0, 1e-12); // Invalid
}

// =============================================================================
// Position/Velocity Tests
// =============================================================================

TEST(RailPositionTest, AtOrigin) {
    double s = 0.0;
    Vec3<double> origin{100.0, 200.0, 300.0};
    Vec3<double> direction{1.0, 0.0, 0.0};

    auto pos = rail_position(s, origin, direction);

    EXPECT_NEAR(pos(0), 100.0, 1e-12);
    EXPECT_NEAR(pos(1), 200.0, 1e-12);
    EXPECT_NEAR(pos(2), 300.0, 1e-12);
}

TEST(RailPositionTest, AlongRail) {
    double s = 2.0;
    Vec3<double> origin{0.0, 0.0, 0.0};
    Vec3<double> direction{1.0, 0.0, 0.0};

    auto pos = rail_position(s, origin, direction);

    EXPECT_NEAR(pos(0), 2.0, 1e-12);
    EXPECT_NEAR(pos(1), 0.0, 1e-12);
    EXPECT_NEAR(pos(2), 0.0, 1e-12);
}

TEST(RailVelocityTest, Basic) {
    double s_dot = 10.0;
    Vec3<double> direction{0.0, 1.0, 0.0};

    auto vel = rail_velocity(s_dot, direction);

    EXPECT_NEAR(vel(0), 0.0, 1e-12);
    EXPECT_NEAR(vel(1), 10.0, 1e-12);
    EXPECT_NEAR(vel(2), 0.0, 1e-12);
}

// =============================================================================
// Attitude Tests
// =============================================================================

TEST(RailAttitudeTest, NorthHorizontal) {
    double azimuth = 0.0;
    double elevation = 0.0;

    auto q = rail_aligned_attitude(azimuth, elevation);

    // Body X pointing North = identity quaternion (NED aligned)
    EXPECT_NEAR(q.w, 1.0, 1e-10);
    EXPECT_NEAR(q.x, 0.0, 1e-10);
    EXPECT_NEAR(q.y, 0.0, 1e-10);
    EXPECT_NEAR(q.z, 0.0, 1e-10);
}

TEST(RailAttitudeTest, EastHorizontal) {
    double azimuth = M_PI / 2; // 90° East
    double elevation = 0.0;

    auto q = rail_aligned_attitude(azimuth, elevation);

    // Should be 90° yaw
    EXPECT_NEAR(q.w, std::cos(M_PI / 4), 1e-10);
    EXPECT_NEAR(q.z, std::sin(M_PI / 4), 1e-10);
}

TEST(RailAttitudeTest, NorthElevated) {
    double azimuth = 0.0;
    double elevation = M_PI / 4; // 45° up

    auto q = rail_aligned_attitude(azimuth, elevation);

    // Should be 45° pitch (rotation about Y)
    EXPECT_NEAR(q.w, std::cos(M_PI / 8), 1e-10);
    EXPECT_NEAR(q.y, std::sin(M_PI / 8), 1e-10);
}

// =============================================================================
// Reaction Force Tests
// =============================================================================

TEST(RailReactionTest, Basic) {
    Vec3<double> force_body{100.0, 50.0, 30.0};

    auto reaction = rail_reaction_force(force_body);

    // Rail reacts Y and Z components
    EXPECT_NEAR(reaction(0), 0.0, 1e-12);
    EXPECT_NEAR(reaction(1), -50.0, 1e-12);
    EXPECT_NEAR(reaction(2), -30.0, 1e-12);
}

// =============================================================================
// Symbolic Tests
// =============================================================================

TEST(RailSymbolicTest, Direction) {
    auto az = janus::sym("az");
    auto el = janus::sym("el");

    auto dir = rail_direction_ned(az, el);

    janus::Function f("rail_dir", {az, el}, {dir(0), dir(1), dir(2)});

    auto result = f({M_PI / 4, M_PI / 6});

    auto dir_num = rail_direction_ned(M_PI / 4, M_PI / 6);
    EXPECT_NEAR(result[0](0, 0), dir_num(0), 1e-10);
    EXPECT_NEAR(result[1](0, 0), dir_num(1), 1e-10);
    EXPECT_NEAR(result[2](0, 0), dir_num(2), 1e-10);
}

TEST(RailSymbolicTest, Acceleration) {
    auto s_dot = janus::sym("s_dot");
    auto F_along = janus::sym("F_along");
    auto F_perp = janus::sym("F_perp");
    auto mass = janus::sym("m");
    auto gravity = janus::sym("g");
    auto elevation = janus::sym("el");
    auto friction = janus::sym("mu");

    auto accel = rail_acceleration(s_dot, F_along, F_perp, mass, gravity,
                                   elevation, friction);

    janus::Function f(
        "rail_accel",
        {s_dot, F_along, F_perp, mass, gravity, elevation, friction}, {accel});

    auto result = f({0.0, 1000.0, 500.0, 100.0, 10.0, M_PI / 6, 0.1});

    double accel_num =
        rail_acceleration(0.0, 1000.0, 500.0, 100.0, 10.0, M_PI / 6, 0.1);
    EXPECT_NEAR(result[0](0, 0), accel_num, 1e-10);
}

TEST(RailSymbolicTest, Attitude) {
    auto az = janus::sym("az");
    auto el = janus::sym("el");

    auto q = rail_aligned_attitude(az, el);

    janus::Function f("rail_attitude", {az, el}, {q.w, q.x, q.y, q.z});

    auto result = f({0.0, M_PI / 4});

    auto q_num = rail_aligned_attitude(0.0, M_PI / 4);
    EXPECT_NEAR(result[0](0, 0), q_num.w, 1e-10);
    EXPECT_NEAR(result[1](0, 0), q_num.x, 1e-10);
    EXPECT_NEAR(result[2](0, 0), q_num.y, 1e-10);
    EXPECT_NEAR(result[3](0, 0), q_num.z, 1e-10);
}

} // namespace vulcan::dynamics
