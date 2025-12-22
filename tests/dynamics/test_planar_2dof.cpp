// Test suite for 2-DOF planar and pendulum dynamics
#include <vulcan/dynamics/Planar2Dof.hpp>

#include <gtest/gtest.h>

namespace vulcan::dynamics {

// =============================================================================
// Planar Point Mass Tests
// =============================================================================

TEST(PlanarTest, NoForce) {
    double fx = 0.0;
    double fy = 0.0;
    double mass = 1.0;

    auto accel = planar_acceleration(fx, fy, mass);

    EXPECT_NEAR(accel(0), 0.0, 1e-12);
    EXPECT_NEAR(accel(1), 0.0, 1e-12);
}

TEST(PlanarTest, PureXForce) {
    double fx = 10.0;
    double fy = 0.0;
    double mass = 2.0;

    auto accel = planar_acceleration(fx, fy, mass);

    EXPECT_NEAR(accel(0), 5.0, 1e-12); // F/m = 10/2
    EXPECT_NEAR(accel(1), 0.0, 1e-12);
}

TEST(PlanarTest, WithGravity) {
    double fx = 0.0;
    double fy = 0.0;
    double mass = 1.0;
    double gravity = 9.81;

    auto accel = planar_acceleration_gravity(fx, fy, mass, gravity);

    EXPECT_NEAR(accel(0), 0.0, 1e-12);
    EXPECT_NEAR(accel(1), 9.81, 1e-12); // Y-down gravity
}

TEST(PlanarTest, VectorForce) {
    Vec2<double> force{10.0, 20.0};
    double mass = 2.0;

    auto accel = planar_acceleration(force, mass);

    EXPECT_NEAR(accel(0), 5.0, 1e-12);
    EXPECT_NEAR(accel(1), 10.0, 1e-12);
}

// =============================================================================
// Simple Pendulum Tests
// =============================================================================

TEST(SimplePendulumTest, EquilibriumAtBottom) {
    // At equilibrium (theta=0), no acceleration
    double theta = 0.0;
    double theta_dot = 0.0;
    double length = 1.0;
    double gravity = 9.81;
    double zeta = 0.0;

    auto theta_ddot =
        simple_pendulum_acceleration(theta, theta_dot, length, gravity, zeta);

    EXPECT_NEAR(theta_ddot, 0.0, 1e-12);
}

TEST(SimplePendulumTest, RestoringForce) {
    // Displaced pendulum has restoring acceleration
    double theta = 0.1; // Small angle
    double theta_dot = 0.0;
    double length = 1.0;
    double gravity = 10.0;
    double zeta = 0.0;

    auto theta_ddot =
        simple_pendulum_acceleration(theta, theta_dot, length, gravity, zeta);

    // θ̈ ≈ -(g/L)*sin(θ) ≈ -10*0.0998 ≈ -0.998
    EXPECT_LT(theta_ddot, 0.0); // Restoring (negative)
    EXPECT_NEAR(theta_ddot, -10.0 * std::sin(0.1), 1e-6);
}

TEST(SimplePendulumTest, DampedMotion) {
    // Moving pendulum with damping
    double theta = 0.0;
    double theta_dot = 1.0;
    double length = 1.0;
    double gravity = 10.0;
    double zeta = 0.1;

    auto theta_ddot =
        simple_pendulum_acceleration(theta, theta_dot, length, gravity, zeta);

    // Should have negative (damping) acceleration
    EXPECT_LT(theta_ddot, 0.0);
}

TEST(SimplePendulumTest, Period) {
    double length = 1.0;
    double gravity = 9.81;

    auto T = simple_pendulum_period(length, gravity);

    // T = 2π√(L/g) ≈ 2.006 s
    EXPECT_NEAR(T, 2.0 * M_PI * std::sqrt(1.0 / 9.81), 1e-10);
}

TEST(SimplePendulumTest, Omega) {
    double length = 1.0;
    double gravity = 10.0;

    auto omega = simple_pendulum_omega(length, gravity);

    // ω = √(g/L) = √10 ≈ 3.162
    EXPECT_NEAR(omega, std::sqrt(10.0), 1e-10);
}

// =============================================================================
// Spherical Pendulum Tests
// =============================================================================

TEST(SphericalPendulumTest, ThetaAccelAtEquilibrium) {
    // At equilibrium (theta=0), only restoring force from gravity
    double theta = 0.0;
    double theta_dot = 0.0;
    double phi_dot = 0.0;
    double length = 1.0;
    double gravity = 10.0;
    double zeta = 0.0;

    auto theta_ddot = spherical_pendulum_theta_ddot(theta, theta_dot, phi_dot,
                                                    length, gravity, zeta);

    EXPECT_NEAR(theta_ddot, 0.0, 1e-10);
}

TEST(SphericalPendulumTest, PhiAccelAtEquilibriumPolar) {
    // At theta≈0, phi motion is undefined (pole of spherical coords)
    // For theta not zero, with phi_dot=0, should be 0
    double theta = 0.5;
    double theta_dot = 0.0;
    double phi_dot = 0.0;
    double length = 1.0;
    double gravity = 10.0;
    double zeta = 0.0;

    auto phi_ddot = spherical_pendulum_phi_ddot(theta, theta_dot, phi_dot,
                                                length, gravity, zeta);

    EXPECT_NEAR(phi_ddot, 0.0, 1e-10);
}

TEST(SphericalPendulumTest, CentrifugalEffect) {
    // Swinging pendulum with azimuthal motion has centrifugal term
    double theta = M_PI / 4; // 45°
    double theta_dot = 0.0;
    double phi_dot = 1.0;
    double length = 1.0;
    double gravity = 10.0;
    double zeta = 0.0;

    auto theta_ddot = spherical_pendulum_theta_ddot(theta, theta_dot, phi_dot,
                                                    length, gravity, zeta);

    // sin(θ)cos(θ)φ̇² - (g/L)sin(θ)
    double expected = std::sin(M_PI / 4) * std::cos(M_PI / 4) * 1.0 -
                      10.0 * std::sin(M_PI / 4);
    EXPECT_NEAR(theta_ddot, expected, 1e-10);
}

TEST(SphericalPendulumTest, Position) {
    double theta = M_PI / 4; // 45°
    double phi = M_PI / 2;   // 90° (East)
    double length = 2.0;

    auto pos = spherical_pendulum_position(theta, phi, length);

    // x = L*sin(θ)*cos(φ) = 2*0.707*0 = 0
    // y = L*sin(θ)*sin(φ) = 2*0.707*1 ≈ 1.414
    // z = L*cos(θ) = 2*0.707 ≈ 1.414
    EXPECT_NEAR(pos(0), 0.0, 1e-10);
    EXPECT_NEAR(pos(1), 2.0 * std::sin(M_PI / 4), 1e-10);
    EXPECT_NEAR(pos(2), 2.0 * std::cos(M_PI / 4), 1e-10);
}

TEST(SphericalPendulumTest, Energy) {
    double theta = M_PI / 6; // 30°
    double theta_dot = 0.0;
    double phi_dot = 0.0;
    double length = 1.0;
    double mass = 1.0;
    double gravity = 10.0;

    auto E = spherical_pendulum_energy(theta, theta_dot, phi_dot, length, mass,
                                       gravity);

    // At rest: E = PE = m*g*L*(1 - cos(θ)) = 1*10*1*(1-0.866) ≈ 1.34
    double PE = mass * gravity * length * (1.0 - std::cos(M_PI / 6));
    EXPECT_NEAR(E, PE, 1e-10);
}

// =============================================================================
// Symbolic Tests
// =============================================================================

TEST(PlanarSymbolicTest, Acceleration) {
    auto fx = janus::sym("fx");
    auto fy = janus::sym("fy");
    auto mass = janus::sym("m");

    auto accel = planar_acceleration(fx, fy, mass);

    janus::Function f("planar_accel", {fx, fy, mass}, {accel(0), accel(1)});

    auto result = f({10.0, 20.0, 2.0});

    auto accel_num = planar_acceleration(10.0, 20.0, 2.0);
    EXPECT_NEAR(result[0](0, 0), accel_num(0), 1e-12);
    EXPECT_NEAR(result[1](0, 0), accel_num(1), 1e-12);
}

TEST(SimplePendulumSymbolicTest, Acceleration) {
    auto theta = janus::sym("theta");
    auto theta_dot = janus::sym("theta_dot");
    auto L = janus::sym("L");
    auto g = janus::sym("g");
    auto zeta = janus::sym("zeta");

    auto theta_ddot =
        simple_pendulum_acceleration(theta, theta_dot, L, g, zeta);

    janus::Function f("simple_pendulum", {theta, theta_dot, L, g, zeta},
                      {theta_ddot});

    auto result = f({0.3, 0.5, 2.0, 9.81, 0.1});

    double theta_ddot_num =
        simple_pendulum_acceleration(0.3, 0.5, 2.0, 9.81, 0.1);
    EXPECT_NEAR(result[0](0, 0), theta_ddot_num, 1e-10);
}

TEST(SphericalPendulumSymbolicTest, ThetaAccel) {
    auto theta = janus::sym("theta");
    auto theta_dot = janus::sym("theta_dot");
    auto phi_dot = janus::sym("phi_dot");
    auto L = janus::sym("L");
    auto g = janus::sym("g");
    auto zeta = janus::sym("zeta");

    auto theta_ddot =
        spherical_pendulum_theta_ddot(theta, theta_dot, phi_dot, L, g, zeta);

    janus::Function f("spherical_theta_ddot",
                      {theta, theta_dot, phi_dot, L, g, zeta}, {theta_ddot});

    auto result = f({0.5, 0.1, 0.2, 1.0, 10.0, 0.01});

    double theta_ddot_num =
        spherical_pendulum_theta_ddot(0.5, 0.1, 0.2, 1.0, 10.0, 0.01);
    EXPECT_NEAR(result[0](0, 0), theta_ddot_num, 1e-10);
}

TEST(SphericalPendulumSymbolicTest, Energy) {
    auto theta = janus::sym("theta");
    auto theta_dot = janus::sym("theta_dot");
    auto phi_dot = janus::sym("phi_dot");
    auto L = janus::sym("L");
    auto m = janus::sym("m");
    auto g = janus::sym("g");

    auto E = spherical_pendulum_energy(theta, theta_dot, phi_dot, L, m, g);

    janus::Function f("spherical_energy", {theta, theta_dot, phi_dot, L, m, g},
                      {E});

    auto result = f({0.5, 0.2, 0.1, 1.0, 1.0, 10.0});

    double E_num = spherical_pendulum_energy(0.5, 0.2, 0.1, 1.0, 1.0, 10.0);
    EXPECT_NEAR(result[0](0, 0), E_num, 1e-10);
}

} // namespace vulcan::dynamics
