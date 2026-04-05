#include <casadi/casadi.hpp>
#include <gtest/gtest.h>
#include <vulcan/propulsion/AltitudeThrust.hpp>

using namespace vulcan::propulsion;
using namespace vulcan::units;
using vulcan::Quantity;

TEST(AltitudeThrustTest, VacuumThrust) {
    // At vacuum (P_atm = 0), F should equal F_vac
    Quantity<N, double> F_vac{5000.0};
    Quantity<Pa, double> P_atm{0.0};
    double P_exit = 1000.0; // Irrelevant for this implementation
    double A_exit = 0.05;

    EXPECT_NEAR(altitude_thrust(F_vac, P_atm, P_exit, A_exit).value(), 5000.0,
                1e-6);
}

TEST(AltitudeThrustTest, SeaLevelThrust) {
    // F_sl = F_vac - P_atm * A_e
    Quantity<N, double> F_vac{5000.0};
    Quantity<Pa, double> P_atm{101325.0};
    double A_exit = 0.01;
    double expected = 5000.0 - 101325.0 * 0.01; // 5000 - 1013.25 = 3986.75

    EXPECT_NEAR(altitude_thrust(F_vac, P_atm, 0.0, A_exit).value(), expected,
                1e-6);
}

TEST(AltitudeThrustTest, ThrustCoefficient) {
    Quantity<N, double> F{4000.0};
    Quantity<Pa, double> Pc{20e5}; // 2 MPa
    double At = 0.001;
    // Cf = 4000 / (2e6 * 0.001) = 4000 / 2000 = 2.0
    EXPECT_NEAR(thrust_coefficient(F, Pc, At).value(), 2.0, 1e-6);
}

TEST(AltitudeThrustTest, SymbolicInstantiation) {
    casadi::MX F_vac_val = casadi::MX::sym("F_vac");
    casadi::MX P_atm_val = casadi::MX::sym("P_atm");
    Quantity<N, casadi::MX> F_vac{F_vac_val};
    Quantity<Pa, casadi::MX> P_atm{P_atm_val};
    double P_exit = 500.0;
    double A_exit = 0.1;

    auto F = altitude_thrust(F_vac, P_atm, P_exit, A_exit);
    EXPECT_EQ(F.value().n_dep(), 2);
}
