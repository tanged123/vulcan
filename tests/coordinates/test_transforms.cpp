#include <gtest/gtest.h>
#include <vulcan/coordinates/BodyFrames.hpp>
#include <vulcan/coordinates/CoordinateFrame.hpp>
#include <vulcan/coordinates/EarthModel.hpp>
#include <vulcan/coordinates/LocalFrames.hpp>
#include <vulcan/coordinates/Transforms.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/Units.hpp>

#include <janus/janus.hpp>

// ============================================
// Body Frame Tests
// ============================================
TEST(BodyFrames, ZeroEulerAngles) {
    // Zero Euler angles should give body = NED
    auto ned = vulcan::CoordinateFrame<double>::ned(0.5, 0.3);
    auto body = vulcan::body_from_euler(ned, 0.0, 0.0, 0.0);

    EXPECT_TRUE(body.is_valid());

    // Body axes should match NED
    EXPECT_NEAR(body.x_axis(0), ned.x_axis(0), 1e-10);
    EXPECT_NEAR(body.x_axis(1), ned.x_axis(1), 1e-10);
    EXPECT_NEAR(body.x_axis(2), ned.x_axis(2), 1e-10);

    EXPECT_NEAR(body.y_axis(0), ned.y_axis(0), 1e-10);
    EXPECT_NEAR(body.y_axis(1), ned.y_axis(1), 1e-10);
    EXPECT_NEAR(body.y_axis(2), ned.y_axis(2), 1e-10);

    EXPECT_NEAR(body.z_axis(0), ned.z_axis(0), 1e-10);
    EXPECT_NEAR(body.z_axis(1), ned.z_axis(1), 1e-10);
    EXPECT_NEAR(body.z_axis(2), ned.z_axis(2), 1e-10);
}

TEST(BodyFrames, PureYaw) {
    // 90 degree yaw: body X should point East (NED Y), body Y should point
    // South (-NED X)
    double yaw = vulcan::constants::angle::pi / 2.0;
    auto ned = vulcan::CoordinateFrame<double>::ned(0.0, 0.0);
    auto body = vulcan::body_from_euler(ned, yaw, 0.0, 0.0);

    EXPECT_TRUE(body.is_valid());

    // Body X should be NED Y (East)
    EXPECT_NEAR(body.x_axis(0), ned.y_axis(0), 1e-10);
    EXPECT_NEAR(body.x_axis(1), ned.y_axis(1), 1e-10);
    EXPECT_NEAR(body.x_axis(2), ned.y_axis(2), 1e-10);

    // Body Y should be -NED X (South)
    EXPECT_NEAR(body.y_axis(0), -ned.x_axis(0), 1e-10);
    EXPECT_NEAR(body.y_axis(1), -ned.x_axis(1), 1e-10);
    EXPECT_NEAR(body.y_axis(2), -ned.x_axis(2), 1e-10);
}

TEST(BodyFrames, PurePitch) {
    // 30 degree pitch up
    double pitch = vulcan::constants::angle::pi / 6.0;
    auto ned = vulcan::CoordinateFrame<double>::ned(0.0, 0.0);
    auto body = vulcan::body_from_euler(ned, 0.0, pitch, 0.0);

    EXPECT_TRUE(body.is_valid());

    // Body X should have component in NED -Z (pointing up)
    // At equator/prime meridian: NED Z = -ECEF X
    // Body X in NED: [cos(pitch), 0, -sin(pitch)]
    vulcan::Vec3<double> x_body_ned = ned.from_ecef(body.x_axis);
    EXPECT_NEAR(x_body_ned(0), std::cos(pitch), 1e-10);
    EXPECT_NEAR(x_body_ned(1), 0.0, 1e-10);
    EXPECT_NEAR(x_body_ned(2), -std::sin(pitch), 1e-10);
}

TEST(BodyFrames, EulerRoundtrip) {
    // Create body frame from arbitrary Euler angles
    double yaw = 0.5;
    double pitch = 0.2;
    double roll = 0.1;

    auto ned = vulcan::CoordinateFrame<double>::ned(1.0, 0.5);
    auto body = vulcan::body_from_euler(ned, yaw, pitch, roll);

    EXPECT_TRUE(body.is_valid());

    // Extract Euler angles back
    auto euler = vulcan::euler_from_body(body, ned);

    EXPECT_NEAR(euler(0), yaw, 1e-10);
    EXPECT_NEAR(euler(1), pitch, 1e-10);
    EXPECT_NEAR(euler(2), roll, 1e-10);
}

TEST(BodyFrames, GimbalLock) {
    // Pitch = 90 degrees (gimbal lock)
    double pitch = vulcan::constants::angle::pi / 2.0;
    double yaw = 0.3;
    double roll = 0.0; // Must be zero due to our convention

    auto ned = vulcan::CoordinateFrame<double>::ned(0.0, 0.0);
    auto body = vulcan::body_from_euler(ned, yaw, pitch, roll);

    EXPECT_TRUE(body.is_valid());

    // Extract Euler angles - pitch should be pi/2
    auto euler = vulcan::euler_from_body(body, ned);
    EXPECT_NEAR(euler(1), pitch, 1e-6);
    // Roll is set to 0 in gimbal lock
    EXPECT_NEAR(euler(2), 0.0, 1e-6);
}

// ============================================
// Velocity Frame Tests
// ============================================
TEST(VelocityFrame, HorizontalNorth) {
    // Velocity pointing North, horizontal
    auto ned = vulcan::CoordinateFrame<double>::ned(0.0, 0.0);
    vulcan::Vec3<double> v_ned;
    v_ned << 100.0, 0.0, 0.0; // 100 m/s North
    vulcan::Vec3<double> v_ecef = ned.to_ecef(v_ned);

    auto vel_frame = vulcan::velocity_frame(v_ecef, ned);
    EXPECT_TRUE(vel_frame.is_valid());

    // Velocity frame X should be NED North
    vulcan::Vec3<double> x_vel_ned = ned.from_ecef(vel_frame.x_axis);
    EXPECT_NEAR(x_vel_ned(0), 1.0, 1e-10);
    EXPECT_NEAR(x_vel_ned(1), 0.0, 1e-10);
    EXPECT_NEAR(x_vel_ned(2), 0.0, 1e-10);
}

TEST(VelocityFrame, HorizontalEast) {
    // Velocity pointing East, horizontal
    auto ned = vulcan::CoordinateFrame<double>::ned(0.0, 0.0);
    vulcan::Vec3<double> v_ned;
    v_ned << 0.0, 100.0, 0.0; // 100 m/s East
    vulcan::Vec3<double> v_ecef = ned.to_ecef(v_ned);

    auto vel_frame = vulcan::velocity_frame(v_ecef, ned);
    EXPECT_TRUE(vel_frame.is_valid());

    // Velocity frame X should be NED East
    vulcan::Vec3<double> x_vel_ned = ned.from_ecef(vel_frame.x_axis);
    EXPECT_NEAR(x_vel_ned(0), 0.0, 1e-10);
    EXPECT_NEAR(x_vel_ned(1), 1.0, 1e-10);
    EXPECT_NEAR(x_vel_ned(2), 0.0, 1e-10);
}

// ============================================
// Flight Path Angle Tests
// ============================================
TEST(FlightPathAngles, LevelNorth) {
    vulcan::Vec3<double> v_ned;
    v_ned << 100.0, 0.0, 0.0; // Level flight North

    auto angles = vulcan::flight_path_angles(v_ned);

    EXPECT_NEAR(angles(0), 0.0, 1e-10); // gamma = 0 (level)
    EXPECT_NEAR(angles(1), 0.0, 1e-10); // psi = 0 (North)
}

TEST(FlightPathAngles, ClimbingEast) {
    // 45 degree climb, heading East
    double speed = 100.0;
    double gamma = vulcan::constants::angle::pi / 4.0;

    vulcan::Vec3<double> v_ned;
    v_ned << 0.0, speed * std::cos(gamma), -speed * std::sin(gamma);

    auto angles = vulcan::flight_path_angles(v_ned);

    EXPECT_NEAR(angles(0), gamma, 1e-10); // gamma
    EXPECT_NEAR(angles(1), vulcan::constants::angle::pi / 2.0,
                1e-10); // psi = East
}

TEST(FlightPathAngles, Descending) {
    // 30 degree descent, heading South
    double speed = 100.0;
    double gamma = -vulcan::constants::angle::pi / 6.0;

    vulcan::Vec3<double> v_ned;
    v_ned << -speed * std::cos(gamma), 0.0, -speed * std::sin(gamma);

    auto angles = vulcan::flight_path_angles(v_ned);

    EXPECT_NEAR(angles(0), gamma, 1e-10);
    EXPECT_NEAR(angles(1), vulcan::constants::angle::pi, 1e-6); // South
}

// ============================================
// Aerodynamic Angle Tests
// ============================================
TEST(AeroAngles, ZeroAngles) {
    // Pure forward velocity
    vulcan::Vec3<double> v_body;
    v_body << 100.0, 0.0, 0.0;

    auto angles = vulcan::aero_angles(v_body);

    EXPECT_NEAR(angles(0), 0.0, 1e-10); // alpha = 0
    EXPECT_NEAR(angles(1), 0.0, 1e-10); // beta = 0
}

TEST(AeroAngles, PositiveAlpha) {
    // Flow from below (nose up)
    double alpha_input = vulcan::constants::angle::pi / 6.0; // 30 deg
    double speed = 100.0;

    vulcan::Vec3<double> v_body;
    v_body << speed * std::cos(alpha_input), 0.0, speed * std::sin(alpha_input);

    auto angles = vulcan::aero_angles(v_body);

    EXPECT_NEAR(angles(0), alpha_input, 1e-10);
    EXPECT_NEAR(angles(1), 0.0, 1e-10);
}

TEST(AeroAngles, PositiveBeta) {
    // Sideslip from right
    double beta_input = vulcan::constants::angle::pi / 12.0; // 15 deg
    double speed = 100.0;

    vulcan::Vec3<double> v_body;
    v_body << speed * std::cos(beta_input), speed * std::sin(beta_input), 0.0;

    auto angles = vulcan::aero_angles(v_body);

    EXPECT_NEAR(angles(0), 0.0, 1e-10);
    EXPECT_NEAR(angles(1), beta_input, 1e-10);
}

// ============================================
// Velocity Transform Tests
// ============================================
TEST(VelocityTransforms, ECEF_ECI_ZeroGMST) {
    // At t=0 (GMST=0), ECI = ECEF
    auto eci = vulcan::CoordinateFrame<double>::eci(0.0);

    vulcan::Vec3<double> r_ecef;
    r_ecef << 6378137.0, 0.0, 0.0; // On equator, prime meridian

    vulcan::Vec3<double> v_ecef;
    v_ecef << 0.0, 7000.0, 0.0; // Eastward velocity

    auto v_eci = vulcan::velocity_ecef_to_eci(v_ecef, r_ecef, eci);

    // v_eci = v_ecef + omega x r
    // omega x r = [0, 0, omega] x [r, 0, 0] = [0, omega*r, 0]
    double omega_r = vulcan::constants::wgs84::omega * r_ecef(0);
    EXPECT_NEAR(v_eci(0), 0.0, 1e-6);
    EXPECT_NEAR(v_eci(1), 7000.0 + omega_r, 1e-6);
    EXPECT_NEAR(v_eci(2), 0.0, 1e-6);
}

TEST(VelocityTransforms, Roundtrip) {
    auto eci = vulcan::CoordinateFrame<double>::eci(0.5);

    vulcan::Vec3<double> r_ecef;
    r_ecef << 5000000.0, 3000000.0, 4000000.0;

    vulcan::Vec3<double> v_ecef;
    v_ecef << 100.0, 200.0, 300.0;

    auto v_eci = vulcan::velocity_ecef_to_eci(v_ecef, r_ecef, eci);
    auto v_back = vulcan::velocity_eci_to_ecef(v_eci, r_ecef, eci);

    EXPECT_NEAR(v_back(0), v_ecef(0), 1e-10);
    EXPECT_NEAR(v_back(1), v_ecef(1), 1e-10);
    EXPECT_NEAR(v_back(2), v_ecef(2), 1e-10);
}

// ============================================
// Non-Inertial Acceleration Tests
// ============================================
TEST(NonInertial, Coriolis_Eastward) {
    // Eastward velocity on equator
    vulcan::Vec3<double> r_ecef;
    r_ecef << 6378137.0, 0.0, 0.0;

    vulcan::Vec3<double> v_ecef;
    v_ecef << 0.0, 1000.0, 0.0; // 1 km/s East

    auto a_coriolis = vulcan::coriolis_acceleration(v_ecef);

    // Coriolis = -2(omega x v) = -2 * [0,0,omega] x [0,v,0] = -2 * [-omega*v,
    // 0, 0] = [2*omega*v, 0, 0]
    double expected_x = 2.0 * vulcan::constants::wgs84::omega * 1000.0;
    EXPECT_NEAR(a_coriolis(0), expected_x, 1e-10);
    EXPECT_NEAR(a_coriolis(1), 0.0, 1e-10);
    EXPECT_NEAR(a_coriolis(2), 0.0, 1e-10);
}

TEST(NonInertial, Centrifugal_Equator) {
    // Point on equator
    vulcan::Vec3<double> r_ecef;
    r_ecef << 6378137.0, 0.0, 0.0;

    auto a_centrifugal = vulcan::centrifugal_acceleration(r_ecef);

    // Centrifugal = -omega x (omega x r)
    // omega x r = [0,0,omega] x [r,0,0] = [0, omega*r, 0]
    // omega x (omega x r) = [0,0,omega] x [0, omega*r, 0] = [-omega^2*r, 0, 0]
    // -omega x (omega x r) = [omega^2*r, 0, 0]
    double omega = vulcan::constants::wgs84::omega;
    double expected_x = omega * omega * r_ecef(0);
    EXPECT_NEAR(a_centrifugal(0), expected_x, 1e-8);
    EXPECT_NEAR(a_centrifugal(1), 0.0, 1e-10);
    EXPECT_NEAR(a_centrifugal(2), 0.0, 1e-10);
}

TEST(NonInertial, Combined) {
    vulcan::Vec3<double> r_ecef;
    r_ecef << 6378137.0, 0.0, 0.0;

    vulcan::Vec3<double> v_ecef;
    v_ecef << 0.0, 1000.0, 0.0;

    auto a_total = vulcan::coriolis_centrifugal(r_ecef, v_ecef);
    auto a_coriolis = vulcan::coriolis_acceleration(v_ecef);
    auto a_centrifugal = vulcan::centrifugal_acceleration(r_ecef);

    EXPECT_NEAR(a_total(0), a_coriolis(0) + a_centrifugal(0), 1e-10);
    EXPECT_NEAR(a_total(1), a_coriolis(1) + a_centrifugal(1), 1e-10);
    EXPECT_NEAR(a_total(2), a_coriolis(2) + a_centrifugal(2), 1e-10);
}

// ============================================
// Range/Range Rate Tests
// ============================================
TEST(RangeTests, BasicRange) {
    vulcan::Vec3<double> r1, r2;
    r1 << 0.0, 0.0, 0.0;
    r2 << 3.0, 4.0, 0.0;

    double d = vulcan::range(r1, r2);
    EXPECT_NEAR(d, 5.0, 1e-10);
}

TEST(RangeTests, RangeRate_Closing) {
    // Target moving toward observer
    vulcan::Vec3<double> r_target, v_target, r_obs, v_obs;
    r_target << 100.0, 0.0, 0.0;
    v_target << -10.0, 0.0, 0.0; // Moving toward origin
    r_obs << 0.0, 0.0, 0.0;
    v_obs << 0.0, 0.0, 0.0;

    double rdot = vulcan::range_rate(r_target, v_target, r_obs, v_obs);
    EXPECT_NEAR(rdot, -10.0, 1e-10); // Negative = closing
}

TEST(RangeTests, RangeRate_Opening) {
    // Target moving away from observer
    vulcan::Vec3<double> r_target, v_target, r_obs, v_obs;
    r_target << 100.0, 0.0, 0.0;
    v_target << 10.0, 0.0, 0.0; // Moving away from origin
    r_obs << 0.0, 0.0, 0.0;
    v_obs << 0.0, 0.0, 0.0;

    double rdot = vulcan::range_rate(r_target, v_target, r_obs, v_obs);
    EXPECT_NEAR(rdot, 10.0, 1e-10); // Positive = opening
}

// ============================================
// Local Frame Tests
// ============================================
TEST(LocalFrames, NED_Wrapper) {
    // Verify local_ned is equivalent to CoordinateFrame::ned
    double lon = 0.5;
    double lat = 0.3;

    auto ned1 = vulcan::local_ned(lon, lat);
    auto ned2 = vulcan::CoordinateFrame<double>::ned(lon, lat);

    EXPECT_NEAR(ned1.x_axis(0), ned2.x_axis(0), 1e-10);
    EXPECT_NEAR(ned1.x_axis(1), ned2.x_axis(1), 1e-10);
    EXPECT_NEAR(ned1.x_axis(2), ned2.x_axis(2), 1e-10);
}

TEST(LocalFrames, Geocentric_Equator) {
    // At equator, geocentric = geodetic
    auto gc = vulcan::local_geocentric(0.0, 0.0);
    auto ned = vulcan::CoordinateFrame<double>::ned(0.0, 0.0);

    EXPECT_TRUE(gc.is_valid());

    // Should be identical at equator
    EXPECT_NEAR(gc.x_axis(0), ned.x_axis(0), 1e-10);
    EXPECT_NEAR(gc.y_axis(1), ned.y_axis(1), 1e-10);
    EXPECT_NEAR(gc.z_axis(0), ned.z_axis(0), 1e-10);
}

TEST(LocalFrames, NED_At_Position) {
    // Create NED at a position
    vulcan::Vec3<double> r_ecef;
    r_ecef << 6378137.0, 0.0, 0.0; // Equator, prime meridian

    auto ned = vulcan::local_ned_at(r_ecef);
    EXPECT_TRUE(ned.is_valid());

    // Should match NED at (0, 0)
    auto ned_expected = vulcan::CoordinateFrame<double>::ned(0.0, 0.0);
    EXPECT_NEAR(ned.x_axis(0), ned_expected.x_axis(0), 1e-8);
    EXPECT_NEAR(ned.x_axis(1), ned_expected.x_axis(1), 1e-8);
    EXPECT_NEAR(ned.x_axis(2), ned_expected.x_axis(2), 1e-8);
}

// ============================================
// Symbolic Tests
// ============================================
TEST(BodyFrames, Symbolic_Euler) {
    using Scalar = janus::SymbolicScalar;

    Scalar lon = janus::sym("lon");
    Scalar lat = janus::sym("lat");
    Scalar yaw = janus::sym("yaw");
    Scalar pitch = janus::sym("pitch");
    Scalar roll = janus::sym("roll");

    auto ned = vulcan::CoordinateFrame<Scalar>::ned(lon, lat);
    auto body = vulcan::body_from_euler(ned, yaw, pitch, roll);

    // Verify symbolic expression
    EXPECT_FALSE(body.x_axis(0).is_constant());

    // Create function and evaluate
    janus::Function f("body_euler", {lon, lat, yaw, pitch, roll},
                      {body.x_axis(0), body.x_axis(1), body.x_axis(2)});

    // Test values
    double test_lon = 0.5;
    double test_lat = 0.3;
    double test_yaw = 0.2;
    double test_pitch = 0.1;
    double test_roll = 0.05;

    auto result = f({test_lon, test_lat, test_yaw, test_pitch, test_roll});

    // Compare with numeric
    auto ned_num = vulcan::CoordinateFrame<double>::ned(test_lon, test_lat);
    auto body_num =
        vulcan::body_from_euler(ned_num, test_yaw, test_pitch, test_roll);

    EXPECT_NEAR(result[0](0, 0), body_num.x_axis(0), 1e-12);
    EXPECT_NEAR(result[1](0, 0), body_num.x_axis(1), 1e-12);
    EXPECT_NEAR(result[2](0, 0), body_num.x_axis(2), 1e-12);
}

TEST(Transforms, Symbolic_Coriolis) {
    using Scalar = janus::SymbolicScalar;

    Scalar vx = janus::sym("vx");
    Scalar vy = janus::sym("vy");
    Scalar vz = janus::sym("vz");

    vulcan::Vec3<Scalar> v_ecef;
    v_ecef << vx, vy, vz;

    auto a_coriolis = vulcan::coriolis_acceleration(v_ecef);

    // Verify symbolic
    EXPECT_FALSE(a_coriolis(0).is_constant());

    // Create function
    janus::Function f("coriolis", {vx, vy, vz},
                      {a_coriolis(0), a_coriolis(1), a_coriolis(2)});

    // Test
    vulcan::Vec3<double> test_v;
    test_v << 100.0, 200.0, 300.0;
    auto result = f({test_v(0), test_v(1), test_v(2)});

    auto a_num = vulcan::coriolis_acceleration(test_v);

    EXPECT_NEAR(result[0](0, 0), a_num(0), 1e-12);
    EXPECT_NEAR(result[1](0, 0), a_num(1), 1e-12);
    EXPECT_NEAR(result[2](0, 0), a_num(2), 1e-12);
}

TEST(FlightPathAngles, Symbolic) {
    using Scalar = janus::SymbolicScalar;

    Scalar vn = janus::sym("vn");
    Scalar ve = janus::sym("ve");
    Scalar vd = janus::sym("vd");

    vulcan::Vec3<Scalar> v_ned;
    v_ned << vn, ve, vd;

    auto angles = vulcan::flight_path_angles(v_ned);

    // Create function
    janus::Function f("fpa", {vn, ve, vd}, {angles(0), angles(1)});

    // Test
    double test_vn = 100.0;
    double test_ve = 50.0;
    double test_vd = -30.0; // Climbing

    auto result = f({test_vn, test_ve, test_vd});

    vulcan::Vec3<double> test_v;
    test_v << test_vn, test_ve, test_vd;
    auto angles_num = vulcan::flight_path_angles(test_v);

    EXPECT_NEAR(result[0](0, 0), angles_num(0), 1e-12);
    EXPECT_NEAR(result[1](0, 0), angles_num(1), 1e-12);
}

// ============================================
// NED Transport Rate Tests
// ============================================
TEST(TransportRate, Omega_NED_ECEF) {
    // At equator, moving North
    vulcan::Vec3<double> v_ned;
    v_ned << 100.0, 0.0, 0.0; // 100 m/s North

    auto omega = vulcan::omega_ned_wrt_ecef(v_ned, 0.0, 0.0);

    // At equator (lat=0): R_n = a*(1-e2) for meridian radius of curvature
    // omega = [-v_n/R_n, 0, 0]
    auto wgs84 = vulcan::EarthModel::WGS84();
    double R_n = wgs84.a * (1.0 - wgs84.e2); // Meridian radius at equator
    EXPECT_NEAR(omega(0), -100.0 / R_n, 1e-12);
    EXPECT_NEAR(omega(1), 0.0, 1e-12);
    EXPECT_NEAR(omega(2), 0.0, 1e-12);
}

TEST(TransportRate, Omega_NED_ECI) {
    // At equator, stationary
    vulcan::Vec3<double> v_ned;
    v_ned << 0.0, 0.0, 0.0;

    auto omega = vulcan::omega_ned_wrt_eci(v_ned, 0.0, 0.0);

    // Earth rotation in NED at equator: [omega, 0, 0]
    EXPECT_NEAR(omega(0), vulcan::constants::wgs84::omega, 1e-14);
    EXPECT_NEAR(omega(1), 0.0, 1e-14);
    EXPECT_NEAR(omega(2), 0.0, 1e-14);
}
