// Test suite for 1-DOF oscillator dynamics
#include <vulcan/dynamics/Oscillator1Dof.hpp>

#include <gtest/gtest.h>

namespace vulcan::dynamics {

// =============================================================================
// Harmonic Oscillator Tests
// =============================================================================

TEST(OscillatorTest, UndampedOscillator) {
    // Undamped oscillator at displacement x=1, no force
    // Expected: x_ddot = -ω²x = -4*1 = -4 for ω=2
    double x = 1.0;
    double x_dot = 0.0;
    double omega_n = 2.0;
    double zeta = 0.0;
    double force = 0.0;
    double mass = 1.0;

    auto x_ddot = oscillator_acceleration(x, x_dot, omega_n, zeta, force, mass);

    EXPECT_NEAR(x_ddot, -4.0, 1e-12);
}

TEST(OscillatorTest, DampedOscillator) {
    // Damped oscillator with velocity
    // ẍ = F/m - 2ζωẋ - ω²x
    double x = 0.0;
    double x_dot = 1.0; // Moving with unit velocity
    double omega_n = 2.0;
    double zeta = 0.5;
    double force = 0.0;
    double mass = 1.0;

    auto x_ddot = oscillator_acceleration(x, x_dot, omega_n, zeta, force, mass);

    // Expected: -2*0.5*2*1 = -2
    EXPECT_NEAR(x_ddot, -2.0, 1e-12);
}

TEST(OscillatorTest, ForcedOscillator) {
    // Static equilibrium case: F = k*x
    // For ω²x = F/m, we need F = m*ω²*x
    double x = 1.0;
    double x_dot = 0.0;
    double omega_n = 2.0; // ω² = 4
    double zeta = 0.0;
    double mass = 2.0;
    double force = 2.0 * 4.0 * 1.0; // F = m*ω²*x = 8 N

    auto x_ddot = oscillator_acceleration(x, x_dot, omega_n, zeta, force, mass);

    // Should be in equilibrium
    EXPECT_NEAR(x_ddot, 0.0, 1e-12);
}

// =============================================================================
// Spring-Damper Tests
// =============================================================================

TEST(SpringDamperTest, StaticEquilibrium) {
    // Spring at rest position, no force
    double x = 0.0;
    double x_dot = 0.0;
    double k = 100.0; // N/m
    double c = 10.0;  // N·s/m
    double force = 0.0;
    double mass = 1.0;

    auto x_ddot = spring_damper_acceleration(x, x_dot, k, c, force, mass);

    EXPECT_NEAR(x_ddot, 0.0, 1e-12);
}

TEST(SpringDamperTest, DisplacedSpring) {
    // Displaced spring: ẍ = -k*x/m = -100*0.1/1 = -10
    double x = 0.1;
    double x_dot = 0.0;
    double k = 100.0;
    double c = 0.0; // No damping
    double force = 0.0;
    double mass = 1.0;

    auto x_ddot = spring_damper_acceleration(x, x_dot, k, c, force, mass);

    EXPECT_NEAR(x_ddot, -10.0, 1e-12);
}

TEST(SpringDamperTest, MovingWithDamping) {
    // Moving mass with damping: ẍ = -c*ẋ/m = -10*1/1 = -10
    double x = 0.0;
    double x_dot = 1.0;
    double k = 0.0; // No spring
    double c = 10.0;
    double force = 0.0;
    double mass = 1.0;

    auto x_ddot = spring_damper_acceleration(x, x_dot, k, c, force, mass);

    EXPECT_NEAR(x_ddot, -10.0, 1e-12);
}

// =============================================================================
// Parameter Conversion Tests
// =============================================================================

TEST(OscillatorParamsTest, SpringToOmega) {
    double k = 100.0; // N/m
    double m = 1.0;   // kg
    // ω = √(k/m) = 10 rad/s

    auto omega = spring_to_omega(k, m);

    EXPECT_NEAR(omega, 10.0, 1e-12);
}

TEST(OscillatorParamsTest, SpringToZeta) {
    double k = 100.0; // N/m
    double c = 2.0;   // N·s/m
    double m = 1.0;   // kg
    // ζ = c / (2√(km)) = 2 / (2*10) = 0.1

    auto zeta = spring_to_zeta(k, c, m);

    EXPECT_NEAR(zeta, 0.1, 1e-12);
}

TEST(OscillatorParamsTest, Period) {
    double omega_n = 2.0 * M_PI; // 1 Hz

    auto T = oscillator_period(omega_n);

    EXPECT_NEAR(T, 1.0, 1e-12); // Period = 1 second
}

// =============================================================================
// Energy Tests
// =============================================================================

TEST(OscillatorEnergyTest, KineticEnergy) {
    double x = 0.0;
    double x_dot = 2.0;
    double omega_n = 2.0;
    double mass = 1.0;

    auto E = oscillator_energy(x, x_dot, omega_n, mass);
    auto KE = oscillator_kinetic_energy(x_dot, mass);

    // At x=0: E = KE = 0.5*m*ẋ² = 0.5*1*4 = 2 J
    EXPECT_NEAR(E, 2.0, 1e-12);
    EXPECT_NEAR(KE, 2.0, 1e-12);
}

TEST(OscillatorEnergyTest, PotentialEnergy) {
    double x = 1.0;
    double x_dot = 0.0;
    double omega_n = 2.0;
    double mass = 1.0;

    auto E = oscillator_energy(x, x_dot, omega_n, mass);
    auto PE = oscillator_potential_energy(x, omega_n, mass);

    // At rest: E = PE = 0.5*m*ω²*x² = 0.5*1*4*1 = 2 J
    EXPECT_NEAR(E, 2.0, 1e-12);
    EXPECT_NEAR(PE, 2.0, 1e-12);
}

TEST(OscillatorEnergyTest, TotalEnergy) {
    double x = 0.5;
    double x_dot = 1.0;
    double omega_n = 2.0;
    double mass = 2.0;

    auto E = oscillator_energy(x, x_dot, omega_n, mass);

    // E = 0.5*m*(ẋ² + ω²x²) = 0.5*2*(1 + 4*0.25) = 0.5*2*2 = 2 J
    EXPECT_NEAR(E, 2.0, 1e-12);
}

} // namespace vulcan::dynamics
