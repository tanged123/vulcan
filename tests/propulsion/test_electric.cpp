#include <casadi/casadi.hpp>
#include <gtest/gtest.h>
#include <vulcan/propulsion/Electric.hpp>

using namespace vulcan::propulsion::electric;

TEST(ElectricPropulsionTest, ThrustFromPower) {
    double P = 2000.0;   // 2 kW
    double Ve = 30000.0; // 30 km/s
    double eff = 0.7;

    // F = 2 * 0.7 * 2000 / 30000 = 2800 / 30000 = 0.09333... N
    double expected = 2.0 * eff * P / Ve;
    EXPECT_NEAR(thrust_from_power(P, Ve, eff), expected, 1e-6);
}

TEST(ElectricPropulsionTest, MassFlowFromPower) {
    double P = 2000.0;
    double Ve = 30000.0;
    double eff = 0.7;

    // mdot = 2 * 0.7 * 2000 / 30000^2
    double expected = (2.0 * eff * P) / (Ve * Ve);
    EXPECT_NEAR(mass_flow_from_power(P, Ve, eff), expected, 1e-9);
}

TEST(ElectricPropulsionTest, CharacteristicVelocity) {
    double P = 2000.0;
    double eff = 0.7;
    double Ve = 30000.0;
    // Calculate mdot consistent with Ve
    double mdot = mass_flow_from_power(P, Ve, eff);

    // c* should recover Ve
    EXPECT_NEAR(characteristic_velocity(P, eff, mdot), Ve, 1e-6);
}

TEST(ElectricPropulsionTest, SymbolicInstantiation) {
    casadi::MX P = casadi::MX::sym("P");
    casadi::MX Ve = casadi::MX::sym("Ve");
    casadi::MX eff = casadi::MX::sym("eff");

    casadi::MX F = thrust_from_power(P, Ve, eff);
    EXPECT_FALSE(F.is_constant());
    EXPECT_EQ(F.size1(), 1);
}
