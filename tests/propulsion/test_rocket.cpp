#include <casadi/casadi.hpp>
#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/propulsion/Rocket.hpp>

using namespace vulcan::propulsion::rocket;

TEST(RocketTest, ThrustFromMdot) {
    double mdot = 10.0;
    double Ve = 3000.0;
    EXPECT_NEAR(thrust_from_mdot(mdot, Ve), 30000.0, 1e-6);
}

TEST(RocketTest, ExhaustVelocity) {
    double Isp = 300.0;
    double g0 = 9.80665;
    EXPECT_NEAR(exhaust_velocity(Isp, g0), 300.0 * 9.80665, 1e-6);
}

TEST(RocketTest, SpecificImpulse) {
    double F = 30000.0;
    double mdot = 10.0; // Ve = 3000
    double g0 = 9.80665;
    // Isp = Ve / g0 = 3000 / 9.80665 approx 305.9
    EXPECT_NEAR(specific_impulse(F, mdot, g0), 3000.0 / 9.80665, 1e-6);
}

TEST(RocketTest, DeltaV) {
    double Ve = 3000.0;
    double m0 = 1000.0;
    double mf = 100.0; // mass ratio 10
    // dV = 3000 * ln(10) approx 3000 * 2.302585 = 6907.7
    double expected = 3000.0 * std::log(10.0);
    EXPECT_NEAR(delta_v(Ve, m0, mf), expected, 1e-6);
}

TEST(RocketTest, PropellantMass) {
    double Ve = 3000.0;
    double m0 = 1000.0;
    double dv = 3000.0 * std::log(10.0); // Should result in mf=100 -> mp=900
    double mp = propellant_mass(dv, m0, Ve);
    EXPECT_NEAR(mp, 900.0, 1e-6);
}

TEST(RocketTest, BurnTime) {
    double mp = 900.0;
    double mdot = 10.0;
    EXPECT_NEAR(burn_time(mp, mdot), 90.0, 1e-6);
}

TEST(RocketTest, SymbolicInstantiation) {
    casadi::MX mdot = casadi::MX::sym("mdot");
    casadi::MX Ve = casadi::MX::sym("Ve");
    casadi::MX F = thrust_from_mdot(mdot, Ve);

    // Just verify it creates a node
    EXPECT_EQ(F.n_dep(), 2); // Depends on mdot and Ve
}
