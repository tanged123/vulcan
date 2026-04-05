#include <iomanip>
#include <iostream>
#include <vector>
#include <vulcan/propulsion/Propulsion.hpp>

using namespace vulcan::propulsion;
using namespace vulcan::units;
using vulcan::Quantity;

void run_rocket_demo() {
    std::cout << "--- Rocket Propulsion Demo ---\n";

    Quantity<s, double> isp{300.0}; // s
    double g0 = 9.80665;
    auto Ve = rocket::exhaust_velocity(isp, g0);

    Quantity<kg, double> m0{1000.0}; // kg
    Quantity<kg, double> mf{100.0};  // kg

    auto dV = rocket::delta_v(Ve, m0, mf);

    std::cout << "Isp: " << isp.value() << " s\n";
    std::cout << "Ve:  " << Ve.value() << " m/s\n";
    std::cout << "m0:  " << m0.value() << " kg, mf: " << mf.value() << " kg\n";
    std::cout << "Delta-V: " << dV.value() << " m/s\n\n";
}

void run_altitude_demo() {
    std::cout << "--- Altitude Compensated Thrust Demo ---\n";

    Quantity<N, double> F_vac{100000.0}; // 100 kN
    double A_exit = 0.5;                 // m^2
    Quantity<Pa, double> P_sl{101325.0}; // Pa
    Quantity<Pa, double> P_vac{0.0};

    auto F_sl = altitude_thrust(F_vac, P_sl, 0.0, A_exit);
    auto F_space = altitude_thrust(F_vac, P_vac, 0.0, A_exit);

    std::cout << "Vacuum Thrust: " << F_vac.value() << " N\n";
    std::cout << "Exit Area:     " << A_exit << " m^2\n";
    std::cout << "Sea Level Thrust: " << F_sl.value()
              << " N (Loss: " << (F_vac.value() - F_sl.value()) << " N)\n";
    std::cout << "Space Thrust:     " << F_space.value() << " N\n\n";
}

void run_air_breathing_demo() {
    std::cout << "--- Air Breathing Demo ---\n";

    Quantity<per_s, double> TSFC{1.5e-5}; // 1/s approx (very efficient jet?)
    Quantity<dimensionless, double> L_D{15.0};
    Quantity<mps, double> V{250.0}; // m/s
    Quantity<N, double> W0{50000.0};
    Quantity<N, double> W1{40000.0};

    auto range = air_breathing::breguet_range(V, TSFC, L_D, W0, W1);

    std::cout << "Velocity: " << V.value() << " m/s\n";
    std::cout << "TSFC:     " << TSFC.value() << " 1/s\n";
    std::cout << "L/D:      " << L_D.value() << "\n";
    std::cout << "Range:    " << range.value() / 1000.0 << " km\n\n";
}

void run_electric_demo() {
    std::cout << "--- Electric Propulsion Demo ---\n";

    Quantity<W, double> P{5000.0}; // 5 kW
    Quantity<dimensionless, double> eff{0.65};
    Quantity<mps, double> Ve{30000.0}; // 30 km/s

    auto F = electric::thrust_from_power(P, Ve, eff);
    auto mdot = electric::mass_flow_from_power(P, Ve, eff);

    std::cout << "Power: " << P.value() << " W\n";
    std::cout << "Eff:   " << eff.value() << "\n";
    std::cout << "Ve:    " << Ve.value() << " m/s\n";
    std::cout << "Thrust: " << F.value() * 1000.0 << " mN\n";
    std::cout << "Mdot:   " << mdot.value() * 1e6 << " mg/s\n\n";
}

int main() {
    run_rocket_demo();
    run_altitude_demo();
    run_air_breathing_demo();
    run_electric_demo();
    return 0;
}
