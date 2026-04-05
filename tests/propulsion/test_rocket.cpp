#include <casadi/casadi.hpp>
#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/propulsion/Rocket.hpp>

using namespace vulcan::propulsion::rocket;
using namespace vulcan::units;
using vulcan::Quantity;

TEST(RocketTest, ThrustFromMdot) {
    Quantity<kg_per_s, double> mdot{10.0};
    Quantity<mps, double> Ve{3000.0};
    EXPECT_NEAR(thrust_from_mdot(mdot, Ve).value(), 30000.0, 1e-6);
}

TEST(RocketTest, ExhaustVelocity) {
    Quantity<s, double> Isp{300.0};
    double g0 = 9.80665;
    EXPECT_NEAR(exhaust_velocity(Isp, g0).value(), 300.0 * 9.80665, 1e-6);
}

TEST(RocketTest, SpecificImpulse) {
    Quantity<N, double> F{30000.0};
    Quantity<kg_per_s, double> mdot{10.0}; // Ve = 3000
    double g0 = 9.80665;
    // Isp = Ve / g0 = 3000 / 9.80665 approx 305.9
    EXPECT_NEAR(specific_impulse(F, mdot, g0).value(), 3000.0 / 9.80665, 1e-6);
}

TEST(RocketTest, DeltaV) {
    Quantity<mps, double> Ve{3000.0};
    Quantity<kg, double> m0{1000.0};
    Quantity<kg, double> mf{100.0}; // mass ratio 10
    // dV = 3000 * ln(10) approx 3000 * 2.302585 = 6907.7
    double expected = 3000.0 * std::log(10.0);
    EXPECT_NEAR(delta_v(Ve, m0, mf).value(), expected, 1e-6);
}

TEST(RocketTest, PropellantMass) {
    Quantity<mps, double> Ve{3000.0};
    Quantity<kg, double> m0{1000.0};
    Quantity<mps, double> dv{
        3000.0 * std::log(10.0)}; // Should result in mf=100 -> mp=900
    double mp = propellant_mass(dv, m0, Ve).value();
    EXPECT_NEAR(mp, 900.0, 1e-6);
}

TEST(RocketTest, BurnTime) {
    Quantity<kg, double> mp{900.0};
    Quantity<kg_per_s, double> mdot{10.0};
    EXPECT_NEAR(burn_time(mp, mdot).value(), 90.0, 1e-6);
}

TEST(RocketTest, SymbolicInstantiation) {
    casadi::MX mdot_val = casadi::MX::sym("mdot");
    casadi::MX Ve_val = casadi::MX::sym("Ve");
    Quantity<kg_per_s, casadi::MX> mdot{mdot_val};
    Quantity<mps, casadi::MX> Ve{Ve_val};
    auto F = thrust_from_mdot(mdot, Ve);

    // Just verify it creates a node
    EXPECT_EQ(F.value().n_dep(), 2); // Depends on mdot and Ve
}
