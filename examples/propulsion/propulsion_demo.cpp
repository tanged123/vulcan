#include <iomanip>
#include <iostream>
#include <vector>
#include <vulcan/propulsion/Propulsion.hpp>

using namespace vulcan::propulsion;

void run_rocket_demo() {
    std::cout << "--- Rocket Propulsion Demo ---\n";

    double isp = 300.0; // s
    double g0 = 9.80665;
    double Ve = rocket::exhaust_velocity(isp, g0);

    double m0 = 1000.0; // kg
    double mf = 100.0;  // kg

    double dV = rocket::delta_v(Ve, m0, mf);

    std::cout << "Isp: " << isp << " s\n";
    std::cout << "Ve:  " << Ve << " m/s\n";
    std::cout << "m0:  " << m0 << " kg, mf: " << mf << " kg\n";
    std::cout << "Delta-V: " << dV << " m/s\n\n";
}

void run_altitude_demo() {
    std::cout << "--- Altitude Compensated Thrust Demo ---\n";

    double F_vac = 100000.0; // 100 kN
    double A_exit = 0.5;     // m^2
    double P_sl = 101325.0;  // Pa
    double P_vac = 0.0;

    double F_sl = altitude_thrust(F_vac, P_sl, 0.0, A_exit);
    double F_space = altitude_thrust(F_vac, P_vac, 0.0, A_exit);

    std::cout << "Vacuum Thrust: " << F_vac << " N\n";
    std::cout << "Exit Area:     " << A_exit << " m^2\n";
    std::cout << "Sea Level Thrust: " << F_sl << " N (Loss: " << (F_vac - F_sl)
              << " N)\n";
    std::cout << "Space Thrust:     " << F_space << " N\n\n";
}

void run_air_breathing_demo() {
    std::cout << "--- Air Breathing Demo ---\n";

    double TSFC = 1.5e-5; // 1/s approx (very efficient jet?)
    double L_D = 15.0;
    double V = 250.0; // m/s
    double W0 = 50000.0;
    double W1 = 40000.0;

    double range = air_breathing::breguet_range(V, TSFC, L_D, W0, W1);

    std::cout << "Velocity: " << V << " m/s\n";
    std::cout << "TSFC:     " << TSFC << " 1/s\n";
    std::cout << "L/D:      " << L_D << "\n";
    std::cout << "Range:    " << range / 1000.0 << " km\n\n";
}

void run_electric_demo() {
    std::cout << "--- Electric Propulsion Demo ---\n";

    double P = 5000.0; // 5 kW
    double eff = 0.65;
    double Ve = 30000.0; // 30 km/s

    double F = electric::thrust_from_power(P, Ve, eff);
    double mdot = electric::mass_flow_from_power(P, Ve, eff);

    std::cout << "Power: " << P << " W\n";
    std::cout << "Eff:   " << eff << "\n";
    std::cout << "Ve:    " << Ve << " m/s\n";
    std::cout << "Thrust: " << F * 1000.0 << " mN\n";
    std::cout << "Mdot:   " << mdot * 1e6 << " mg/s\n\n";
}

int main() {
    run_rocket_demo();
    run_altitude_demo();
    run_air_breathing_demo();
    run_electric_demo();
    return 0;
}
