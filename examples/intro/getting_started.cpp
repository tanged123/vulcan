// Getting Started with Vulcan
// This example demonstrates both numeric and symbolic modes
#include <iostream>
#include <janus/core/Function.hpp>
#include <janus/math/AutoDiff.hpp>
#include <vulcan/vulcan.hpp>

// Templated model function - works in both modes
template <typename Scalar> Scalar air_density(const Scalar &altitude) {
    return vulcan::ussa1976::density(altitude);
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "       Vulcan: Getting Started          " << std::endl;
    std::cout << "========================================" << std::endl;

    // ========================================
    // 1. Numeric Mode: Fast evaluation
    // ========================================
    std::cout << "\n=== Numeric Mode ===" << std::endl;

    double alt = 10000.0; // 10 km
    double rho = air_density(alt);
    std::cout << "  Density at 10km: " << rho << " kg/m^3" << std::endl;

    // Use state() to get all properties at once
    auto atm = vulcan::ussa1976::state(alt);
    std::cout << "  Temperature at 10km: " << atm.temperature << " K"
              << std::endl;
    std::cout << "  Speed of sound at 10km: " << atm.speed_of_sound << " m/s"
              << std::endl;

    // ========================================
    // 2. Unit Conversions
    // ========================================
    std::cout << "\n=== Unit Conversions ===" << std::endl;
    double alt_ft = vulcan::units::m_to_ft(alt);
    std::cout << "  10 km = " << alt_ft << " ft" << std::endl;

    double T_C = vulcan::units::K_to_C(atm.temperature);
    std::cout << "  " << atm.temperature << " K = " << T_C << " °C"
              << std::endl;

    // ========================================
    // 3. Physical Constants
    // ========================================
    std::cout << "\n=== Constants ===" << std::endl;
    std::cout << "  Earth mu: " << vulcan::constants::earth::mu << " m^3/s^2"
              << std::endl;
    std::cout << "  WGS84 a: " << vulcan::constants::wgs84::a << " m"
              << std::endl;
    std::cout << "  Sea level density: " << vulcan::constants::atmosphere::rho0
              << " kg/m^3" << std::endl;

    // ========================================
    // 4. Symbolic Mode: For optimization
    // ========================================
    std::cout << "\n=== Symbolic Mode ===" << std::endl;

    auto h = janus::sym("h");
    auto rho_sym = air_density(h);

    std::cout << "  Created symbolic density expression" << std::endl;

    // Create a function to evaluate
    janus::Function rho_func("rho", {h}, {rho_sym});
    auto result = rho_func({5000.0});
    std::cout << "  Evaluating at h=5000: " << result[0](0, 0) << " kg/m^3"
              << std::endl;

    // Compute gradient
    auto drho_dh = janus::jacobian(rho_sym, h);
    janus::Function grad_func("drho_dh", {h}, {drho_dh});
    auto grad_result = grad_func({5000.0});
    std::cout << "  d(rho)/dh at h=5000: " << grad_result[0](0, 0) << " kg/m^4"
              << std::endl;

    std::cout << "\n✅ Vulcan is working correctly!" << std::endl;

    return 0;
}
