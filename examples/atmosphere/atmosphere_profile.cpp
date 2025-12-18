// Atmosphere Profile Example
// Demonstrates US Standard Atmosphere 1976 with Janus optimization
#include <iostream>
#include <janus/optimization/Opti.hpp>
#include <vulcan/vulcan.hpp>

// Templated model function - works in both modes
template <typename Scalar> Scalar air_density(const Scalar &altitude) {
    return vulcan::ussa1976::density(altitude);
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "   US Standard Atmosphere 1976 Profile  " << std::endl;
    std::cout << "========================================" << std::endl;

    // ========================================
    // 1. Numeric Mode: Atmosphere profile using state() struct
    // ========================================
    std::cout << "\n=== Atmosphere Profile ===" << std::endl;
    std::cout
        << "Alt (km)  T (K)    P (Pa)      rho (kg/m^3)  a (m/s)   g (m/s^2)"
        << std::endl;
    std::cout
        << "----------------------------------------------------------------"
        << std::endl;

    for (int h_km = 0; h_km <= 25; h_km += 5) {
        double h = h_km * 1000.0; // Convert to meters

        // Use state() to get all properties in one call
        auto atm = vulcan::ussa1976::state(h);

        printf("%5d    %6.2f   %10.2f  %11.6f   %6.2f   %6.4f\n", h_km,
               atm.temperature, atm.pressure, atm.density, atm.speed_of_sound,
               atm.gravity);
    }

    // ========================================
    // 2. Symbolic Mode: Optimization example
    // ========================================
    std::cout << "\n=== Symbolic Mode: Density Optimization ===" << std::endl;

    janus::Opti opti;
    auto h = opti.variable(5000.0); // Altitude as decision variable

    auto rho_sym = air_density(h);

    // Example: Find altitude with specific density target
    // We'll minimize (rho - target)^2
    double target_rho = 0.5; // kg/m^3
    auto objective = (rho_sym - target_rho) * (rho_sym - target_rho);

    opti.minimize(objective);
    opti.subject_to(h >= 0.0);
    opti.subject_to(h <= 50000.0);

    auto sol = opti.solve();

    double h_opt = static_cast<double>(sol.value(h));
    double rho_at_opt = vulcan::ussa1976::density(h_opt);

    std::cout << "  Target density: " << target_rho << " kg/m^3" << std::endl;
    std::cout << "  Optimal altitude: " << h_opt / 1000.0 << " km" << std::endl;
    std::cout << "  Actual density: " << rho_at_opt << " kg/m^3" << std::endl;

    std::cout << "\n✅ Atmosphere profile complete!" << std::endl;

    return 0;
}
