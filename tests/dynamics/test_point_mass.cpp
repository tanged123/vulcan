// Test suite for 3-DOF point mass dynamics
#include <vulcan/dynamics/PointMass.hpp>

#include <gtest/gtest.h>

namespace vulcan::dynamics {

// =============================================================================
// Basic Point Mass Tests
// =============================================================================

TEST(PointMassTest, AccelerationFromForce) {
    Vec3<double> force{100.0, 0.0, 0.0};
    double mass = 10.0;

    auto accel = point_mass_acceleration(force, mass);

    EXPECT_NEAR(accel(0), 10.0, 1e-12);
    EXPECT_NEAR(accel(1), 0.0, 1e-12);
    EXPECT_NEAR(accel(2), 0.0, 1e-12);
}

TEST(PointMassTest, Speed) {
    Vec3<double> velocity{3.0, 4.0, 0.0};

    auto v = speed(velocity);

    EXPECT_NEAR(v, 5.0, 1e-12);
}

TEST(PointMassTest, VelocityDirection) {
    Vec3<double> velocity{10.0, 0.0, 0.0};

    auto v_hat = velocity_direction(velocity);

    EXPECT_NEAR(v_hat(0), 1.0, 1e-12);
    EXPECT_NEAR(v_hat(1), 0.0, 1e-12);
    EXPECT_NEAR(v_hat(2), 0.0, 1e-12);
}

// =============================================================================
// ECEF Dynamics Tests
// =============================================================================

TEST(PointMassEcefTest, ZeroRotation) {
    Vec3<double> pos{6.371e6, 0.0, 0.0};
    Vec3<double> vel{0.0, 100.0, 0.0};
    Vec3<double> force{0.0, 0.0, 0.0};
    double mass = 1.0;
    Vec3<double> omega_e{0.0, 0.0, 0.0}; // No rotation

    auto accel = point_mass_acceleration_ecef(pos, vel, force, mass, omega_e);

    // With no rotation, should just be F/m = 0
    EXPECT_NEAR(accel(0), 0.0, 1e-12);
    EXPECT_NEAR(accel(1), 0.0, 1e-12);
    EXPECT_NEAR(accel(2), 0.0, 1e-12);
}

TEST(PointMassEcefTest, CoriolisEffect) {
    constexpr double omega_e = 7.2921159e-5;
    Vec3<double> pos{6.371e6, 0.0, 0.0};
    Vec3<double> vel{0.0, 100.0, 0.0};
    Vec3<double> force{0.0, 0.0, 0.0};
    double mass = 1.0;
    Vec3<double> omega{0.0, 0.0, omega_e};

    auto accel = point_mass_acceleration_ecef(pos, vel, force, mass, omega);

    // Coriolis: -2*ω×v = 2*ω*v in x direction at equator
    double expected_coriolis_x = 2.0 * omega_e * 100.0;
    EXPECT_GT(accel(0), expected_coriolis_x * 0.9); // Approximate check
}

// =============================================================================
// G-load Tests
// =============================================================================

TEST(GLoadTest, Magnitude) {
    Vec3<double> accel{0.0, 0.0, -19.62}; // ~2g upward

    auto n = g_load_magnitude(accel, 9.81);

    EXPECT_NEAR(n, 2.0, 1e-3);
}

TEST(GLoadTest, Vector) {
    Vec3<double> accel{9.81, 0.0, 0.0};

    auto n = g_load(accel, 9.81);

    EXPECT_NEAR(n(0), 1.0, 1e-10);
    EXPECT_NEAR(n(1), 0.0, 1e-12);
    EXPECT_NEAR(n(2), 0.0, 1e-12);
}

// =============================================================================
// Flight Path Angle Tests
// =============================================================================

TEST(FlightPathTest, LevelFlight) {
    // Horizontal velocity (North)
    Vec3<double> velocity{100.0, 0.0, 0.0};

    auto gamma = flight_path_angle(velocity);

    EXPECT_NEAR(gamma, 0.0, 1e-10);
}

TEST(FlightPathTest, Climbing) {
    // 45° climb (North-Up)
    double v = 100.0 / std::sqrt(2.0);
    Vec3<double> velocity{v, 0.0, -v}; // Z-down, so -vz = climbing

    auto gamma = flight_path_angle(velocity);

    EXPECT_NEAR(gamma, M_PI / 4, 1e-10);
}

TEST(FlightPathTest, HeadingNorth) {
    Vec3<double> velocity{100.0, 0.0, 0.0};

    auto chi = heading_angle(velocity);

    EXPECT_NEAR(chi, 0.0, 1e-10);
}

TEST(FlightPathTest, HeadingEast) {
    Vec3<double> velocity{0.0, 100.0, 0.0};

    auto chi = heading_angle(velocity);

    EXPECT_NEAR(chi, M_PI / 2, 1e-10);
}

TEST(FlightPathTest, VelocityFromAngles) {
    double spd = 100.0;
    double gamma = M_PI / 6; // 30° climb
    double chi = M_PI / 4;   // 45° heading (NE)

    auto vel = velocity_from_angles(spd, gamma, chi);

    // Check magnitude
    EXPECT_NEAR(std::sqrt(vel(0) * vel(0) + vel(1) * vel(1) + vel(2) * vel(2)),
                spd, 1e-10);

    // Check flight path angle
    auto gamma_computed = flight_path_angle(vel);
    EXPECT_NEAR(gamma_computed, gamma, 1e-10);

    // Check heading
    auto chi_computed = heading_angle(vel);
    EXPECT_NEAR(chi_computed, chi, 1e-10);
}

// =============================================================================
// Energy Tests
// =============================================================================

TEST(EnergyTest, KineticEnergy) {
    Vec3<double> velocity{100.0, 0.0, 0.0};

    auto KE = specific_kinetic_energy(velocity);

    // KE = 0.5 * v² = 0.5 * 10000 = 5000 J/kg
    EXPECT_NEAR(KE, 5000.0, 1e-10);
}

TEST(EnergyTest, PotentialEnergy) {
    Vec3<double> position{7000.0e3, 0.0, 0.0}; // 7000 km from Earth center
    double mu = 3.986004418e14;                // Earth

    auto PE = specific_potential_energy(position, mu);

    // PE = -μ/r ≈ -56.94 MJ/kg
    EXPECT_NEAR(PE, -mu / 7000.0e3, 1e-6);
}

TEST(EnergyTest, TotalEnergy) {
    Vec3<double> pos{7000.0e3, 0.0, 0.0};
    double v_circ = std::sqrt(3.986004418e14 / 7000.0e3);
    Vec3<double> vel{0.0, v_circ, 0.0};
    double mu = 3.986004418e14;

    auto E = specific_energy(pos, vel, mu);

    // Circular orbit: E = -μ/(2a) = -μ/(2r)
    EXPECT_NEAR(E, -mu / (2.0 * 7000.0e3), 1e-6);
}

// =============================================================================
// Symbolic Tests
// =============================================================================

TEST(PointMassSymbolicTest, Acceleration) {
    auto Fx = janus::sym("Fx");
    auto Fy = janus::sym("Fy");
    auto Fz = janus::sym("Fz");
    auto m = janus::sym("m");

    Vec3<casadi::MX> force{Fx, Fy, Fz};
    auto accel = point_mass_acceleration(force, m);

    janus::Function f("pm_accel", {Fx, Fy, Fz, m},
                      {accel(0), accel(1), accel(2)});

    auto result = f({100.0, 50.0, 25.0, 10.0});

    Vec3<double> force_num{100.0, 50.0, 25.0};
    auto accel_num = point_mass_acceleration(force_num, 10.0);

    EXPECT_NEAR(result[0](0, 0), accel_num(0), 1e-12);
    EXPECT_NEAR(result[1](0, 0), accel_num(1), 1e-12);
    EXPECT_NEAR(result[2](0, 0), accel_num(2), 1e-12);
}

TEST(PointMassSymbolicTest, FlightPathAngle) {
    auto vx = janus::sym("vx");
    auto vy = janus::sym("vy");
    auto vz = janus::sym("vz");

    Vec3<casadi::MX> vel{vx, vy, vz};
    auto gamma = flight_path_angle(vel);

    janus::Function f("fpa", {vx, vy, vz}, {gamma});

    auto result = f({100.0, 0.0, -50.0});

    Vec3<double> vel_num{100.0, 0.0, -50.0};
    auto gamma_num = flight_path_angle(vel_num);

    EXPECT_NEAR(result[0](0, 0), gamma_num, 1e-10);
}

TEST(PointMassSymbolicTest, Energy) {
    auto rx = janus::sym("rx");
    auto ry = janus::sym("ry");
    auto rz = janus::sym("rz");
    auto vx = janus::sym("vx");
    auto vy = janus::sym("vy");
    auto vz = janus::sym("vz");
    auto mu = janus::sym("mu");

    Vec3<casadi::MX> pos{rx, ry, rz};
    Vec3<casadi::MX> vel{vx, vy, vz};
    auto E = specific_energy(pos, vel, mu);

    janus::Function f("energy", {rx, ry, rz, vx, vy, vz, mu}, {E});

    auto result = f({7000.0e3, 0.0, 0.0, 0.0, 7500.0, 0.0, 3.986004418e14});

    Vec3<double> pos_num{7000.0e3, 0.0, 0.0};
    Vec3<double> vel_num{0.0, 7500.0, 0.0};
    auto E_num = specific_energy(pos_num, vel_num, 3.986004418e14);

    EXPECT_NEAR(result[0](0, 0), E_num, 1e-6);
}

} // namespace vulcan::dynamics
