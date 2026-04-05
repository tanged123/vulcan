#include <casadi/casadi.hpp>
#include <gtest/gtest.h>
#include <vulcan/propulsion/Electric.hpp>

using namespace vulcan::propulsion::electric;
using namespace vulcan::units;
using vulcan::Quantity;

TEST(ElectricPropulsionTest, ThrustFromPower) {
    Quantity<W, double> P{2000.0};     // 2 kW
    Quantity<mps, double> Ve{30000.0}; // 30 km/s
    Quantity<dimensionless, double> eff{0.7};

    // F = 2 * 0.7 * 2000 / 30000 = 2800 / 30000 = 0.09333... N
    double expected = 2.0 * 0.7 * 2000.0 / 30000.0;
    EXPECT_NEAR(thrust_from_power(P, Ve, eff).value(), expected, 1e-6);
}

TEST(ElectricPropulsionTest, MassFlowFromPower) {
    Quantity<W, double> P{2000.0};
    Quantity<mps, double> Ve{30000.0};
    Quantity<dimensionless, double> eff{0.7};

    // mdot = 2 * 0.7 * 2000 / 30000^2
    double expected = (2.0 * 0.7 * 2000.0) / (30000.0 * 30000.0);
    EXPECT_NEAR(mass_flow_from_power(P, Ve, eff).value(), expected, 1e-9);
}

TEST(ElectricPropulsionTest, CharacteristicVelocity) {
    Quantity<W, double> P{2000.0};
    Quantity<dimensionless, double> eff{0.7};
    Quantity<mps, double> Ve{30000.0};
    // Calculate mdot consistent with Ve
    auto mdot = mass_flow_from_power(P, Ve, eff);

    // c* should recover Ve
    EXPECT_NEAR(characteristic_velocity(P, eff, mdot).value(), 30000.0, 1e-6);
}

TEST(ElectricPropulsionTest, SymbolicInstantiation) {
    Quantity<W, casadi::MX> P{casadi::MX::sym("P")};
    Quantity<mps, casadi::MX> Ve{casadi::MX::sym("Ve")};
    Quantity<dimensionless, casadi::MX> eff{casadi::MX::sym("eff")};

    auto F = thrust_from_power(P, Ve, eff);
    EXPECT_FALSE(F.value().is_constant());
    EXPECT_EQ(F.value().size1(), 1);
}
