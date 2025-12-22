// Test suite for fuel slosh dynamics
#include <vulcan/dynamics/Slosh.hpp>

#include <gtest/gtest.h>

namespace vulcan::dynamics {

// =============================================================================
// Pendulum Slosh Tests
// =============================================================================

TEST(PendulumSloshTest, EquilibriumNoForcing) {
    // At equilibrium (theta=0), no forcing, should have no acceleration
    double theta = 0.0;
    double theta_dot = 0.0;
    double length = 0.5;
    double zeta = 0.01;
    double accel_transverse = 0.0;
    double gravity = 9.81;

    auto theta_ddot = pendulum_slosh_acceleration(
        theta, theta_dot, length, zeta, accel_transverse, gravity);

    EXPECT_NEAR(theta_ddot, 0.0, 1e-10);
}

TEST(PendulumSloshTest, RestoringForce) {
    // Displaced pendulum has restoring force
    double theta = 0.1;
    double theta_dot = 0.0;
    double length = 1.0;
    double zeta = 0.0;
    double accel_transverse = 0.0;
    double gravity = 9.81;

    auto theta_ddot = pendulum_slosh_acceleration(
        theta, theta_dot, length, zeta, accel_transverse, gravity);

    // θ̈ ≈ -(g/L)*sin(θ) ≈ -9.81*0.0998 ≈ -0.979
    EXPECT_LT(theta_ddot, 0.0);
    EXPECT_NEAR(theta_ddot, -9.81 * std::sin(0.1), 0.01);
}

TEST(PendulumSloshTest, DampedMotion) {
    // Moving pendulum with damping
    double theta = 0.0;
    double theta_dot = 1.0;
    double length = 1.0;
    double zeta = 0.1;
    double accel_transverse = 0.0;
    double gravity = 9.81;

    auto theta_ddot = pendulum_slosh_acceleration(
        theta, theta_dot, length, zeta, accel_transverse, gravity);

    // Damping should oppose motion
    EXPECT_LT(theta_ddot, 0.0);
}

TEST(PendulumSloshTest, ForcedResponse) {
    // Vehicle acceleration forces pendulum
    double theta = 0.0;
    double theta_dot = 0.0;
    double length = 0.5;
    double zeta = 0.0;
    double accel_transverse = 5.0; // 5 m/s² transverse
    double gravity = 9.81;

    auto theta_ddot = pendulum_slosh_acceleration(
        theta, theta_dot, length, zeta, accel_transverse, gravity);

    // At θ=0: θ̈ = (a_t/L)*cos(0) = 5/0.5 = 10 rad/s²
    EXPECT_NEAR(theta_ddot, 10.0, 0.1);
}

TEST(PendulumSloshTest, TransverseAccel) {
    Vec3<double> accel_body{5.0, 0.0, 3.0};
    double theta = 0.0;

    auto a_t = slosh_transverse_accel(accel_body, theta);

    // At θ=0: a_t = a_x*cos(0) - a_z*sin(0) = 5
    EXPECT_NEAR(a_t, 5.0, 1e-10);
}

TEST(PendulumSloshTest, Position) {
    double theta = M_PI / 6; // 30°
    double length = 2.0;

    auto pos = pendulum_slosh_position(theta, length);

    // x = L*sin(θ) = 2*0.5 = 1
    // y = 0
    // z = L*cos(θ) = 2*0.866 ≈ 1.732
    EXPECT_NEAR(pos(0), 1.0, 1e-10);
    EXPECT_NEAR(pos(1), 0.0, 1e-10);
    EXPECT_NEAR(pos(2), 2.0 * std::cos(M_PI / 6), 1e-10);
}

// =============================================================================
// Spring-Mass Slosh Tests
// =============================================================================

TEST(SpringSloshTest, EquilibriumRestoring) {
    // Displaced mass should have restoring acceleration
    double displacement = 0.1;
    double velocity = 0.0;
    double stiffness = 1000.0; // N/m
    double damping = 0.0;
    double mass = 100.0;
    double vehicle_accel = 0.0;

    auto accel = spring_slosh_acceleration(displacement, velocity, stiffness,
                                           damping, mass, vehicle_accel);

    // a = -k*x/m = -1000*0.1/100 = -1.0 m/s²
    EXPECT_NEAR(accel, -1.0, 1e-10);
}

TEST(SpringSloshTest, DampedMotion) {
    // Moving mass should be damped
    double displacement = 0.0;
    double velocity = 1.0;
    double stiffness = 0.0;
    double damping = 50.0; // N·s/m
    double mass = 100.0;
    double vehicle_accel = 0.0;

    auto accel = spring_slosh_acceleration(displacement, velocity, stiffness,
                                           damping, mass, vehicle_accel);

    // a = -c*v/m = -50*1/100 = -0.5 m/s²
    EXPECT_NEAR(accel, -0.5, 1e-10);
}

TEST(SpringSloshTest, VehicleForcing) {
    // Vehicle acceleration forces mass
    double displacement = 0.0;
    double velocity = 0.0;
    double stiffness = 0.0;
    double damping = 0.0;
    double mass = 100.0;
    double vehicle_accel = 5.0;

    auto accel = spring_slosh_acceleration(displacement, velocity, stiffness,
                                           damping, mass, vehicle_accel);

    // a = -m*a_vehicle/m = -5 m/s²
    EXPECT_NEAR(accel, -5.0, 1e-10);
}

TEST(SpringSloshTest, Acceleration3D) {
    Vec3<double> displacement{0.1, 0.05, 0.0};
    Vec3<double> velocity{0.5, 0.2, 0.0};
    Vec3<double> stiffness{1000.0, 1000.0, 1000.0};
    Vec3<double> damping{50.0, 50.0, 50.0};
    double mass = 100.0;
    Vec3<double> accel_body{0.0, 0.0, 0.0};

    auto accel = spring_slosh_acceleration_3d(displacement, velocity, stiffness,
                                              damping, mass, accel_body);

    // ax = (-50*0.5 - 1000*0.1)/100 = -1.25
    // ay = (-50*0.2 - 1000*0.05)/100 = -0.6
    EXPECT_NEAR(accel(0), -1.25, 1e-10);
    EXPECT_NEAR(accel(1), -0.6, 1e-10);
    EXPECT_NEAR(accel(2), 0.0, 1e-10);
}

TEST(SpringSloshTest, ReactionForce) {
    Vec3<double> displacement{0.1, 0.05, 0.0};
    Vec3<double> velocity{0.0, 0.0, 0.0};
    Vec3<double> stiffness{1000.0, 1000.0, 1000.0};
    Vec3<double> damping{0.0, 0.0, 0.0};

    auto force = spring_slosh_force(displacement, velocity, stiffness, damping);

    // Fx = k*x = 1000*0.1 = 100 N
    // Fy = k*y = 1000*0.05 = 50 N
    EXPECT_NEAR(force(0), 100.0, 1e-10);
    EXPECT_NEAR(force(1), 50.0, 1e-10);
    EXPECT_NEAR(force(2), 0.0, 1e-10);
}

// =============================================================================
// NASA SP-8009 Parameter Estimation Tests
// =============================================================================

TEST(SloshParamsTest, CylindricalFrequency) {
    double tank_radius = 1.0;
    double fill_level = 0.8;
    double gravity = 9.81;

    auto omega = slosh_frequency_cylindrical(tank_radius, fill_level, gravity);

    // Should be in reasonable range (1-5 rad/s for meter-scale tanks)
    EXPECT_GT(omega, 1.0);
    EXPECT_LT(omega, 10.0);
}

TEST(SloshParamsTest, SphericalFrequency) {
    double tank_radius = 0.5;
    double gravity = 9.81;

    // At 50% fill, should have meaningful frequency
    auto omega_50 = slosh_frequency_spherical(tank_radius, 0.5, gravity);
    EXPECT_GT(omega_50, 1.0);

    // At extremes (empty/full), frequency should be lower
    auto omega_10 = slosh_frequency_spherical(tank_radius, 0.1, gravity);
    auto omega_90 = slosh_frequency_spherical(tank_radius, 0.9, gravity);
    EXPECT_LT(omega_10, omega_50);
    EXPECT_LT(omega_90, omega_50);
}

TEST(SloshParamsTest, PendulumLength) {
    double omega = 2.0; // rad/s
    double gravity = 10.0;

    auto L = slosh_pendulum_length(omega, gravity);

    // L = g/ω² = 10/4 = 2.5 m
    EXPECT_NEAR(L, 2.5, 1e-10);
}

TEST(SloshParamsTest, MassFraction) {
    // Deep fill should have ~70% slosh mass
    auto fraction_deep = slosh_mass_fraction_cylindrical(0.8);
    EXPECT_NEAR(fraction_deep, 0.7, 0.1);

    // Shallow fill should have less
    auto fraction_shallow = slosh_mass_fraction_cylindrical(0.2);
    EXPECT_LT(fraction_shallow, fraction_deep);
}

TEST(SloshParamsTest, SloshMass) {
    double total_mass = 1000.0;
    double fill_level = 0.8;

    auto m_slosh = slosh_mass_cylindrical(total_mass, fill_level);

    // Deep fill: ~70% = 700 kg
    EXPECT_GT(m_slosh, 500.0);
    EXPECT_LT(m_slosh, 900.0);
}

} // namespace vulcan::dynamics
