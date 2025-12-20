#include <iomanip>
#include <iostream>
#include <vulcan/vulcan.hpp>

using namespace vulcan;

// Templated function works for both numeric and symbolic types
template <typename Scalar>
void analyze_flight_condition(const Scalar &altitude,
                              const Vec3<Scalar> &v_body,
                              const Scalar &char_length) {
    // 1. Get Atmospheric Conditions
    auto atm = vulcan::ussa1976::state(altitude);

    // 2. Compute Aerodynamic State
    auto aero =
        vulcan::aero::aero_state(atm.density, atm.speed_of_sound,
                                 atm.dynamic_viscosity, v_body, char_length);

    // Print results (only for numeric types)
    if constexpr (std::is_same_v<Scalar, double>) {
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "  Altitude:      " << altitude << " m" << std::endl;
        std::cout << "  Velocity Body: [" << v_body.transpose() << "] m/s"
                  << std::endl;
        std::cout << "  ----------------------------------------" << std::endl;
        std::cout << "  Airspeed:      " << aero.airspeed << " m/s"
                  << std::endl;
        std::cout << "  Mach Number:   " << aero.mach << std::endl;
        std::cout << "  Dyn Pressure:  " << aero.dynamic_pressure << " Pa"
                  << std::endl;
        std::cout << "  Reynolds No:   " << std::scientific << aero.reynolds
                  << std::defaultfloat << std::endl;
        std::cout << "  Alpha (AoA):   "
                  << vulcan::units::rad_to_deg(aero.alpha) << " deg"
                  << std::endl;
        std::cout << "  Beta (Slip):   " << vulcan::units::rad_to_deg(aero.beta)
                  << " deg" << std::endl;
        std::cout << std::endl;
    }
}

int main() {
    std::cout << "=== Vulcan Aerodynamics Demo ===" << std::endl << std::endl;

    // =========================================================================
    // 1. Numeric Mode (Standard C++ Execution)
    // =========================================================================
    std::cout << "--- Numeric Mode ---" << std::endl;

    double h = 10000.0; // 10 km altitude
    double L = 5.0;     // 5m characteristic length

    // Case A: Level Flight
    Vec3<double> v_level;
    v_level << 250.0, 0.0, 0.0;
    std::cout << "Case A: Level Flight" << std::endl;
    analyze_flight_condition(h, v_level, L);

    // Case B: High Alpha Maneuver
    // Velocity has strong Z-component (down in body frame = flow from below)
    Vec3<double> v_maneuver;
    v_maneuver << 200.0, 0.0, 35.0; // ~10 deg alpha
    std::cout << "Case B: High Angle of Attack" << std::endl;
    analyze_flight_condition(h, v_maneuver, L);

    // =========================================================================
    // 2. Symbolic Mode (Graph Generation for Optimization)
    // =========================================================================
    std::cout << "--- Symbolic Mode ---" << std::endl;

    // Define symbolic variables
    janus::SymbolicScalar sym_h = janus::sym("h");
    janus::SymbolicScalar sym_vx = janus::sym("vx");
    janus::SymbolicScalar sym_vz = janus::sym("vz");
    janus::SymbolicScalar sym_L = janus::sym("L");

    Vec3<janus::SymbolicScalar> sym_v;
    sym_v << sym_vx, 0.0, sym_vz;

    // Compute chain of operations symbolically
    // This builds the computational graph: h -> atm -> aero -> q
    auto atm = vulcan::ussa1976::state(sym_h);
    auto aero = vulcan::aero::aero_state(atm.density, atm.speed_of_sound,
                                         atm.dynamic_viscosity, sym_v, sym_L);

    // We can now output this graph or derivatives
    std::cout << "Generated symbolic graph for Mach number:" << std::endl;
    std::cout << "  M = f(h, vx, vz)" << std::endl;

    // Calculate sensitivity of Mach number impacting drag coefficients
    // d(Mach)/d(Height) - essential for trajectory opt
    auto dM_dh = janus::jacobian(aero.mach, sym_h);

    std::cout << "Computed symbolic derivative d(Mach)/d(h)." << std::endl;
    std::cout << "Evaluating derivative at h=10000m, V=250m/s..." << std::endl;

    // Create function to evaluate the derivative
    janus::Function f_grad("grad_M", {sym_h, sym_vx, sym_vz, sym_L}, {dM_dh});

    // Evaluate
    auto result = f_grad({10000.0, 250.0, 0.0, 5.0});
    std::cout << "  Result: " << std::scientific << result[0](0) << " m^-1"
              << std::endl;
    std::cout << "  (Expected positive: as h increases, speed of sound "
                 "decreases, so Mach increases)"
              << std::endl;

    return 0;
}
