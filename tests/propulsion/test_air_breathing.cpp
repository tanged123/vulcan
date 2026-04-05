#include <casadi/casadi.hpp>
#include <gtest/gtest.h>
#include <vulcan/propulsion/AirBreathing.hpp>

using namespace vulcan::propulsion::air_breathing;
using namespace vulcan::units;
using vulcan::Quantity;

TEST(AirBreathingTest, FuelFlowRate) {
    Quantity<N, double> F{10000.0};
    Quantity<per_s, double> TSFC{1.0e-4}; // 1/s
    // mdot = 10000 * 1e-4 = 1.0
    EXPECT_NEAR(fuel_flow_rate(F, TSFC).value(), 1.0, 1e-6);
}

TEST(AirBreathingTest, BreguetRange) {
    Quantity<mps, double> V{250.0};
    Quantity<per_s, double> TSFC{1.0e-4};
    Quantity<dimensionless, double> L_D{18.0};
    Quantity<N, double> W0{20000.0};
    Quantity<N, double> W1{15000.0};

    // R = (250 / 1e-4) * 18 * ln(20000/15000)
    //   = 2.5e6 * 18 * ln(1.333...)
    //   = 45e6 * 0.28768
    double expected = (250.0 / 1.0e-4) * 18.0 * std::log(20000.0 / 15000.0);

    EXPECT_NEAR(breguet_range(V, TSFC, L_D, W0, W1).value(), expected, 1e-6);
}

TEST(AirBreathingTest, BreguetEndurance) {
    Quantity<per_s, double> TSFC{1.0e-4};
    Quantity<dimensionless, double> L_D{18.0};
    Quantity<N, double> W0{20000.0};
    Quantity<N, double> W1{15000.0};

    // E = (1/TSFC) * L_D * ln(W0/W1)
    double expected = (1.0 / 1.0e-4) * 18.0 * std::log(20000.0 / 15000.0);

    EXPECT_NEAR(breguet_endurance(TSFC, L_D, W0, W1).value(), expected, 1e-6);
}

TEST(AirBreathingTest, SymbolicInstantiation) {
    Quantity<mps, casadi::MX> V{casadi::MX::sym("V")};
    Quantity<per_s, casadi::MX> TSFC{casadi::MX::sym("TSFC")};
    Quantity<dimensionless, casadi::MX> L_D{casadi::MX(15.0)};
    Quantity<N, casadi::MX> W0{casadi::MX(100.0)};
    Quantity<N, casadi::MX> W1{casadi::MX(80.0)};

    auto range = breguet_range(V, TSFC, L_D, W0, W1);
    EXPECT_EQ(range.value().n_dep(), 2);
}
