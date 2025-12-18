// Atmosphere Profile Example
// Demonstrates US Standard Atmosphere 1976 with Janus optimization
#include <iostream>
#include <vulcan/vulcan.hpp>

// Templated model function - works in both modes
template <typename Scalar>
Scalar air_density(const Scalar& altitude) {
    return vulcan::standard_atmosphere::density(altitude);
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "   US Standard Atmosphere 1976 Profile  " << std::endl;
    std::cout << "========================================" << std::endl;

    // ========================================
    // 1. Numeric Mode: Atmosphere profile
    // ========================================
    std::cout << "\n=== Atmosphere Profile ===" << std::endl;
    std::cout << "Alt (km)  T (K)    P (Pa)      rho (kg/m^3)  a (m/s)"
              << std::endl;
    std::cout << "--------------------------------------------------------"
              << std::endl;

    for (int h_km = 0; h_km <= 25; h_km += 5) {
        double h = h_km * 1000.0; // Convert to meters
        double T = vulcan::standard_atmosphere::temperature(h);
        double P = vulcan::standard_atmosphere::pressure(h);
        double rho = vulcan::standard_atmosphere::density(h);
        double a = vulcan::standard_atmosphere::speed_of_sound(h);

        printf("%5d    %6.2f   %10.2f  %11.6f   %6.2f\n", h_km, T, P, rho, a);
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
    auto objective = janus::pow(rho_sym - target_rho, 2.0);

    opti.minimize(objective);
    opti.subject_to_bounds(h, 0.0, 50000.0);

    auto sol = opti.solve();

    double h_opt = sol.value(h);
    double rho_at_opt = vulcan::standard_atmosphere::density(h_opt);

    std::cout << "  Target density: " << target_rho << " kg/m^3" << std::endl;
    std::cout << "  Optimal altitude: " << h_opt / 1000.0 << " km" << std::endl;
    std::cout << "  Actual density: " << rho_at_opt << " kg/m^3" << std::endl;

    std::cout << "\n✅ Atmosphere profile complete!" << std::endl;

    return 0;
}
