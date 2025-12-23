/**
 * @file hypersonic_reentry.cpp
 * @brief Hypersonic Glide Vehicle Energy Management - Comprehensive Vulcan Demo
 *
 * This demo optimizes the bank angle of a hypersonic glide vehicle during
 * atmospheric reentry to maximize cross-range while respecting thermal and
 * structural limits.
 *
 * Vulcan Modules Integrated:
 *   - atmosphere/StandardAtmosphere.hpp  — USSA1976 density model
 *   - aerodynamics/Aerodynamics.hpp      — dynamic_pressure()
 *   - dynamics/Guided5Dof.hpp            — gamma_dot(), chi_dot_btt()
 *   - core/Constants.hpp                 — Earth parameters, physics constants
 *
 * Pattern:
 *   1. Define templated reentry physics using Vulcan utilities
 *   2. Numeric bank angle sweep (double)
 *   3. Symbolic optimization with constraints (casadi::MX)
 *   4. Computational graph export for visualization
 */

#include <cmath>
#include <iomanip>
#include <iostream>
#include <janus/janus.hpp>
#include <vector>
#include <vulcan/vulcan.hpp>

using namespace vulcan;
using namespace vulcan::constants;
using namespace vulcan::dynamics;

// =============================================================================
// Reentry Vehicle Parameters
// =============================================================================

struct ReentryVehicle {
    double mass = 8000.0;     // Vehicle mass [kg]
    double Sref = 15.0;       // Reference area [m²]
    double L_D = 1.5;         // Lift-to-drag ratio (hypersonic)
    double CL = 0.4;          // Lift coefficient
    double CD = 0.267;        // Drag coefficient (CL/L_D)
    double nose_radius = 0.3; // Nose radius for heating [m]
};

// =============================================================================
// Templated Physics Model
// =============================================================================

/**
 * @brief Compute reentry dynamics at a given state
 *
 * Implements 3-DOF equations for a gliding reentry vehicle:
 *   dV/dt = -D/m - g*sin(gamma)
 *   dgamma/dt = (L*cos(bank) - W*cos(gamma)) / (m*V)
 *   dchi/dt = L*sin(bank) / (m*V*cos(gamma))
 *   dh/dt = V*sin(gamma)
 *
 * @tparam Scalar double or casadi::MX
 * @param altitude Current altitude [m]
 * @param velocity Current velocity [m/s]
 * @param gamma Flight path angle [rad] (negative = descending)
 * @param bank Bank angle [rad]
 * @param vehicle Vehicle parameters
 * @return Derivatives and constraint values
 */
template <typename Scalar> struct ReentryOutputs {
    Scalar v_dot;        // Velocity derivative [m/s²]
    Scalar gamma_dot;    // Flight path angle rate [rad/s]
    Scalar chi_dot;      // Heading rate [rad/s]
    Scalar h_dot;        // Altitude rate [m/s]
    Scalar q;            // Dynamic pressure [Pa]
    Scalar load_factor;  // Normal load factor [g]
    Scalar heating_rate; // Stagnation heating rate [W/m²]
};

template <typename Scalar>
ReentryOutputs<Scalar> reentry_physics(const Scalar &altitude,
                                       const Scalar &velocity,
                                       const Scalar &gamma, const Scalar &bank,
                                       const ReentryVehicle &vehicle) {
    // 1. Atmosphere - use USSA1976 for realistic density profile
    Scalar rho = ussa1976::density(altitude);

    // 2. Aerodynamic forces using Vulcan aero module
    Scalar q = aero::dynamic_pressure(rho, velocity);
    Scalar L = q * Scalar(vehicle.Sref) * Scalar(vehicle.CL);
    Scalar D = q * Scalar(vehicle.Sref) * Scalar(vehicle.CD);
    Scalar W = Scalar(vehicle.mass) * Scalar(physics::g0);

    // 3. Gravity - spherical model with altitude correction
    Scalar r = Scalar(earth::R_eq) + altitude;
    Scalar g = Scalar(earth::mu) / (r * r);
    Scalar weight = Scalar(vehicle.mass) * g;

    // 4. Dynamics using Vulcan 5DOF utilities
    // dV/dt = -(D/m) - g*sin(gamma)
    Scalar v_dot = -D / Scalar(vehicle.mass) - g * janus::sin(gamma);

    // gamma_dot using Vulcan's gamma_dot function
    // Signature: gamma_dot(lift, weight, mass, velocity, gamma, phi)
    Scalar gamma_dot_val =
        dynamics::gamma_dot(L,                    // lift [N]
                            weight,               // weight [N]
                            Scalar(vehicle.mass), // mass [kg]
                            velocity,             // velocity [m/s]
                            gamma,                // gamma [rad]
                            bank                  // phi (bank) [rad]
        );

    // chi_dot using Vulcan's chi_dot_btt function
    // Signature: chi_dot_btt(lift, mass, velocity, gamma, phi)
    Scalar chi_dot_val =
        dynamics::chi_dot_btt(L,                    // lift [N]
                              Scalar(vehicle.mass), // mass [kg]
                              velocity,             // velocity [m/s]
                              gamma,                // gamma [rad]
                              bank                  // phi (bank) [rad]
        );

    // dh/dt = V*sin(gamma)
    Scalar h_dot = velocity * janus::sin(gamma);

    // 5. Load factor (for structural constraint)
    Scalar load_factor = janus::sqrt(L * L + D * D) / W;

    // 6. Stagnation point heating rate (Sutton-Graves correlation)
    Scalar k_heating = Scalar(1.83e-4);
    Scalar heating_rate = k_heating *
                          janus::sqrt(rho / Scalar(vehicle.nose_radius)) *
                          janus::pow(velocity, Scalar(3.0));

    return {v_dot, gamma_dot_val, chi_dot_val, h_dot,
            q,     load_factor,   heating_rate};
}

/**
 * @brief Compute cross-range achieved for a given bank angle profile
 *
 * Integrates the heading rate over altitude to estimate cross-range.
 * Uses a simplified single-segment model for optimization.
 *
 * @tparam Scalar Numeric or symbolic type
 * @param bank_angle Constant bank angle for segment [rad]
 * @param initial_velocity Entry velocity [m/s]
 * @param initial_altitude Entry altitude [m]
 * @param final_altitude Pullout altitude [m]
 * @param vehicle Vehicle parameters
 * @return Cross-range estimate [m] and total heating load
 */
template <typename Scalar> struct CrossRangeResult {
    Scalar cross_range;      // [m]
    Scalar total_heating;    // [J/m²] integrated heating load
    Scalar max_load_factor;  // [g]
    Scalar max_heating_rate; // [W/m²]
};

template <typename Scalar>
CrossRangeResult<Scalar> estimate_cross_range(const Scalar &bank_angle,
                                              const Scalar &initial_velocity,
                                              const Scalar &initial_altitude,
                                              const Scalar &final_altitude,
                                              const ReentryVehicle &vehicle) {
    // Discretize altitude range for integration
    int N = 10;
    Scalar delta_h = (initial_altitude - final_altitude) / Scalar(N);

    Scalar cross_range = Scalar(0.0);
    Scalar total_heating = Scalar(0.0);
    Scalar max_load = Scalar(0.0);
    Scalar max_heat_rate = Scalar(0.0);

    Scalar velocity = initial_velocity;
    Scalar gamma = Scalar(-0.05); // Shallow reentry angle (~ -3 degrees)

    for (int i = 0; i < N; ++i) {
        Scalar altitude = initial_altitude - Scalar(i) * delta_h;

        auto outputs =
            reentry_physics(altitude, velocity, gamma, bank_angle, vehicle);

        // Time for this altitude segment (h_dot is negative for descent)
        Scalar dt = -delta_h /
                    (outputs.h_dot - Scalar(0.1)); // Ensure no division by zero

        // Cross-range: accumulate heading change, then convert to distance
        // chi_dot [rad/s], dt [s] → total heading change [rad]
        // Cross-range = R_earth * heading_change
        cross_range = cross_range + outputs.chi_dot * dt;

        // Heating accumulation - banking extends flight path, increasing total
        // heating
        Scalar bank_heating_factor = Scalar(1.0) + Scalar(0.5) *
                                                       janus::sin(bank_angle) *
                                                       janus::sin(bank_angle);
        total_heating =
            total_heating + outputs.heating_rate * dt * bank_heating_factor;

        // Load factor increases with bank angle (centripetal acceleration from
        // turn) Real physics: n = L/W = (1/cos(bank)) for level turn
        Scalar bank_load_contribution =
            outputs.load_factor / janus::cos(bank_angle + Scalar(0.01));
        max_load = janus::where(bank_load_contribution > max_load,
                                bank_load_contribution, max_load);

        // Peak heating rate increases with bank angle
        // Higher bank → shallower descent → longer time at high velocity in
        // denser air
        Scalar heating_bank_factor =
            Scalar(1.0) / janus::cos(bank_angle * Scalar(0.7) + Scalar(0.01));
        Scalar effective_heating = outputs.heating_rate * heating_bank_factor;
        max_heat_rate = janus::where(effective_heating > max_heat_rate,
                                     effective_heating, max_heat_rate);

        // Update velocity for next segment
        velocity = velocity + outputs.v_dot * dt;
        velocity =
            janus::where(velocity < Scalar(1000.0), Scalar(1000.0), velocity);

        // Update gamma
        gamma = gamma + outputs.gamma_dot * dt;
    }

    // Convert heading angle (radians) to distance (meters): d = R × θ
    Scalar cross_range_m = cross_range * Scalar(earth::R_eq);

    return {cross_range_m, total_heating, max_load, max_heat_rate};
}

// =============================================================================
// Main Demo
// =============================================================================

int main() {
    std::cout
        << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout
        << "║  Comprehensive Demo 2: Hypersonic Glide Vehicle Reentry    ║\n";
    std::cout
        << "╚════════════════════════════════════════════════════════════╝\n\n";

    ReentryVehicle vehicle;

    std::cout << "Vehicle Configuration (X-37B-like):\n";
    std::cout << "  Mass:        " << vehicle.mass << " kg\n";
    std::cout << "  Wing area:   " << vehicle.Sref << " m²\n";
    std::cout << "  L/D:         " << vehicle.L_D << "\n";
    std::cout << "  Nose radius: " << vehicle.nose_radius << " m\n\n";

    // Entry conditions
    double h_entry = 120000.0;  // 120 km entry interface
    double v_entry = 7500.0;    // ~orbital velocity
    double h_pullout = 30000.0; // 30 km pullout altitude

    std::cout << "Entry Conditions:\n";
    std::cout << "  Entry altitude: " << h_entry / 1000.0 << " km\n";
    std::cout << "  Entry velocity: " << v_entry << " m/s (Mach ~25)\n";
    std::cout << "  Pullout alt:    " << h_pullout / 1000.0 << " km\n\n";

    // =========================================================================
    // Part 1: Numeric Mode - Physics at a single point
    // =========================================================================
    std::cout << "=== Part 1: Numeric Evaluation at Peak Heating ===\n\n";

    double h_peak = 70000.0;           // ~70 km is often peak heating
    double v_peak = 6500.0;            // Still hypersonic
    double gamma = -0.05;              // Shallow descent
    double bank = 45.0 * M_PI / 180.0; // 45° bank

    auto outputs =
        reentry_physics<double>(h_peak, v_peak, gamma, bank, vehicle);

    std::cout << "State at h = 70 km, V = 6500 m/s, bank = 45°:\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Deceleration:  " << -outputs.v_dot << " m/s² ("
              << -outputs.v_dot / 9.81 << " g)\n";
    std::cout << "  Gamma rate:    " << outputs.gamma_dot * 180 / M_PI
              << " deg/s\n";
    std::cout << "  Heading rate:  " << outputs.chi_dot * 180 / M_PI
              << " deg/s\n";
    std::cout << "  Dyn pressure:  " << outputs.q / 1000.0 << " kPa\n";
    std::cout << "  Load factor:   " << outputs.load_factor << " g\n";
    std::cout << "  Heating rate:  " << outputs.heating_rate / 1e6
              << " MW/m²\n\n";

    // =========================================================================
    // Part 2: Numeric Mode - Bank Angle Sweep
    // =========================================================================
    std::cout << "=== Part 2: Bank Angle vs Cross-Range Sweep ===\n\n";

    std::cout
        << "Bank [deg] | Cross-Range [km] | Max Load [g] | Max Heat [MW/m²]\n";
    std::cout
        << "-----------|------------------|--------------|------------------\n";

    double best_bank = 0.0;
    double best_range = 0.0;

    for (double bank_deg = 0; bank_deg <= 80; bank_deg += 10) {
        double bank_rad = bank_deg * M_PI / 180.0;
        auto result = estimate_cross_range<double>(bank_rad, v_entry, h_entry,
                                                   h_pullout, vehicle);

        std::cout << std::setw(10) << bank_deg << " | " << std::setw(16)
                  << std::setprecision(1) << result.cross_range / 1000.0
                  << " | " << std::setw(12) << std::setprecision(2)
                  << result.max_load_factor << " | " << std::setw(17)
                  << std::setprecision(2) << result.max_heating_rate / 1e6
                  << "\n";

        if (result.cross_range > best_range && result.max_load_factor < 25) {
            best_range = result.cross_range;
            best_bank = bank_deg;
        }
    }

    std::cout << "\nBest feasible from sweep: " << best_bank << "° bank, "
              << best_range / 1000.0 << " km cross-range\n\n";

    // =========================================================================
    // Part 3: Symbolic Optimization with janus::Opti
    // =========================================================================
    std::cout << "=== Part 3: Optimization with janus::Opti ===\n\n";

    janus::Opti opti;

    // Decision variable: bank angle
    auto bank_var = opti.variable(best_bank * M_PI / 180.0);

    // Wrap constants as symbolic
    casadi::MX v_entry_mx = v_entry;
    casadi::MX h_entry_mx = h_entry;
    casadi::MX h_pullout_mx = h_pullout;

    // Compute cross-range and constraints symbolically
    auto result = estimate_cross_range(bank_var, v_entry_mx, h_entry_mx,
                                       h_pullout_mx, vehicle);

    // Objective: MAXIMIZE cross-range (minimize negative)
    opti.minimize(-result.cross_range);

    // Constraints
    opti.subject_to(bank_var >=
                    0.01); // Bank > 0 (need some bank for cross-range)
    opti.subject_to(bank_var <=
                    80.0 * M_PI / 180.0); // Bank <= 80° (structural limit)
    opti.subject_to(result.max_load_factor <= 25.0);  // Max load <= 25g
    opti.subject_to(result.max_heating_rate <= 13e6); // Max heating <= 13 MW/m²

    std::cout << "Optimization setup:\n";
    std::cout << "  Variables: bank_angle\n";
    std::cout << "  Objective: maximize cross-range\n";
    std::cout << "  Constraints:\n";
    std::cout << "    - 0° < bank <= 80°\n";
    std::cout << "    - Max load factor <= 25 g\n";
    std::cout << "    - Max heating rate <= 13 MW/m²\n\n";

    // Solve
    auto solution = opti.solve({.verbose = false});

    double opt_bank = solution.value(bank_var) * 180 / M_PI;
    double opt_range = solution.value(result.cross_range);
    double opt_load = solution.value(result.max_load_factor);
    double opt_heat = solution.value(result.max_heating_rate);

    std::cout << "Optimization Results:\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Optimal bank angle: " << opt_bank << "°\n";
    std::cout << "  Cross-range:        " << opt_range / 1000.0 << " km\n";
    std::cout << "  Max load factor:    " << opt_load << " g\n";
    std::cout << "  Max heating rate:   " << opt_heat / 1e6 << " MW/m²\n";
    std::cout << "  Solver iterations:  " << solution.num_iterations()
              << "\n\n";

    // =========================================================================
    // Part 4: Export Computational Graphs
    // =========================================================================
    std::cout
        << "=== Part 4: Exporting Interactive Computational Graphs ===\n\n";

    // Create symbolic variables
    auto h_sym = janus::sym("altitude");
    auto v_sym = janus::sym("velocity");
    auto gamma_sym = janus::sym("gamma");
    auto bank_sym = janus::sym("bank");

    // Build physics graph
    auto out_sym = reentry_physics(h_sym, v_sym, gamma_sym, bank_sym, vehicle);

    // Export heading rate (cross-range driver)
    janus::export_graph_html(out_sym.chi_dot, "graph_reentry_heading_rate",
                             "Heading_Rate");
    std::cout
        << "✓ Exported: graph_reentry_heading_rate.html (cross-range rate)\n";

    // Export heating rate
    janus::export_graph_html(out_sym.heating_rate, "graph_reentry_heating",
                             "Stagnation_Heating");
    std::cout
        << "✓ Exported: graph_reentry_heating.html (Sutton-Graves heating)\n";

    // Export load factor
    janus::export_graph_html(out_sym.load_factor, "graph_reentry_load",
                             "Load_Factor");
    std::cout << "✓ Exported: graph_reentry_load.html (structural load)\n";

    // // Export cross-range objective (from optimization) >> graph is too large
    // !!! janus::export_graph_html(result.cross_range,
    // "graph_reentry_crossrange",
    //                          "Cross_Range_Objective");
    // std::cout << "✓ Exported: graph_reentry_crossrange.html (optimization
    // objective)\n";

    std::cout << "\nOpen these HTML files in a browser to explore the "
                 "computational graphs!\n";

    // =========================================================================
    // Part 5: Summary
    // =========================================================================
    std::cout << "\n═══════════════════════════════════════════════════════════"
                 "════\n";
    std::cout << "Hypersonic Reentry Demo Complete!\n\n";
    std::cout << "This demo integrated:\n";
    std::cout << "  • USSA1976 atmosphere (realistic density profile)\n";
    std::cout << "  • Vulcan aerodynamics (dynamic pressure, L/D)\n";
    std::cout << "  • Vulcan 5DOF dynamics (gamma_dot, chi_dot_btt)\n";
    std::cout << "  • Sutton-Graves heating correlation\n";
    std::cout << "  • Bank-angle-dependent load factor (1/cos(φ))\n";
    std::cout << "\n";
    std::cout << "The optimization found the bank angle that:\n";
    std::cout << "  • Maximizes cross-range capability\n";
    std::cout << "  • Respects thermal protection limits (15 MW/m²)\n";
    std::cout << "  • Respects structural limits (25g)\n";
    std::cout
        << "═══════════════════════════════════════════════════════════════\n";

    return 0;
}
