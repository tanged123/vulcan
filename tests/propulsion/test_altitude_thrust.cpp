#include <casadi/casadi.hpp>
#include <gtest/gtest.h>
#include <vulcan/propulsion/AltitudeThrust.hpp>

using namespace vulcan::propulsion;

TEST(AltitudeThrustTest, VacuumThrust) {
    // At vacuum (P_atm = 0), F should equal F_vac
    double F_vac = 5000.0;
    double P_atm = 0.0;
    double P_exit = 1000.0; // Irrelevant for this implementation
    double A_exit = 0.05;

    EXPECT_NEAR(altitude_thrust(F_vac, P_atm, P_exit, A_exit), 5000.0, 1e-6);
}

TEST(AltitudeThrustTest, SeaLevelThrust) {
    // F_sl = F_vac - P_atm * A_e
    double F_vac = 5000.0;
    double P_atm = 101325.0;
    double A_exit = 0.01;
    double expected = 5000.0 - 101325.0 * 0.01; // 5000 - 1013.25 = 3986.75

    EXPECT_NEAR(altitude_thrust(F_vac, P_atm, 0.0, A_exit), expected, 1e-6);
}

TEST(AltitudeThrustTest, ThrustCoefficient) {
    double F = 4000.0;
    double Pc = 20e5; // 2 MPa
    double At = 0.001;
    // Cf = 4000 / (2e6 * 0.001) = 4000 / 2000 = 2.0
    EXPECT_NEAR(thrust_coefficient(F, Pc, At), 2.0, 1e-6);
}

TEST(AltitudeThrustTest, SymbolicInstantiation) {
    casadi::MX F_vac = casadi::MX::sym("F_vac");
    casadi::MX P_atm = casadi::MX::sym("P_atm");
    double P_exit = 500.0;
    double A_exit = 0.1;

    casadi::MX F = altitude_thrust(F_vac, P_atm, P_exit, A_exit);
    EXPECT_EQ(F.n_dep(), 2);
}
