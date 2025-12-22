#include <casadi/casadi.hpp>
#include <gtest/gtest.h>
#include <vulcan/propulsion/AirBreathing.hpp>

using namespace vulcan::propulsion::air_breathing;

TEST(AirBreathingTest, FuelFlowRate) {
    double F = 10000.0;
    double TSFC = 1.0e-4; // 1/s
    // mdot = 10000 * 1e-4 = 1.0
    EXPECT_NEAR(fuel_flow_rate(F, TSFC), 1.0, 1e-6);
}

TEST(AirBreathingTest, BreguetRange) {
    double V = 250.0;
    double TSFC = 1.0e-4;
    double L_D = 18.0;
    double W0 = 20000.0;
    double W1 = 15000.0;

    // R = (250 / 1e-4) * 18 * ln(20000/15000)
    //   = 2.5e6 * 18 * ln(1.333...)
    //   = 45e6 * 0.28768
    double expected = (V / TSFC) * L_D * std::log(W0 / W1);

    EXPECT_NEAR(breguet_range(V, TSFC, L_D, W0, W1), expected, 1e-6);
}

TEST(AirBreathingTest, BreguetEndurance) {
    double TSFC = 1.0e-4;
    double L_D = 18.0;
    double W0 = 20000.0;
    double W1 = 15000.0;

    // E = (1/TSFC) * L_D * ln(W0/W1)
    double expected = (1.0 / TSFC) * L_D * std::log(W0 / W1);

    EXPECT_NEAR(breguet_endurance(TSFC, L_D, W0, W1), expected, 1e-6);
}

TEST(AirBreathingTest, SymbolicInstantiation) {
    casadi::MX V = casadi::MX::sym("V");
    casadi::MX TSFC = casadi::MX::sym("TSFC");
    casadi::MX range = breguet_range(V, TSFC, casadi::MX(15.0),
                                     casadi::MX(100.0), casadi::MX(80.0));
    EXPECT_EQ(range.n_dep(), 2);
}
