/**
 * @file wind_optimization.cpp
 * @brief Demonstrates wind-aware trajectory optimization using Vulcan wind
 * models
 *
 * This example shows how to use symbolic wind shear profiles with janus::Opti
 * for gradient-based optimization. We optimize an aircraft's climb profile to
 * minimize effective headwind by choosing optimal altitude waypoints.
 *
 * Scenario:
 *   - Aircraft climbing from 100m to 2000m over a fixed ground distance
 *   - Wind shear varies with altitude (power-law profile)
 *   - Goal: minimize integrated headwind exposure
 *
 * Key insight: Vulcan's wind models (shear profiles, turbulence PSDs) are
 * fully symbolic-compatible with Janus, enabling gradient-based optimization.
 */

#include <cmath>
#include <iomanip>
#include <iostream>
#include <janus/janus.hpp>
#include <vulcan/vulcan.hpp>

using namespace vulcan::wind;
using namespace vulcan::wind_shear;

/**
 * @brief Compute effective headwind at altitude
 *
 * Models a wind shear environment where headwind increases with altitude
 * following a power-law profile.
 *
 * @tparam Scalar Numeric or symbolic type
 * @param altitude Aircraft altitude [m]
 * @param ref_wind Reference wind at 10m [m/s]
 * @return Headwind component [m/s] (positive = opposing)
 */
template <typename Scalar>
Scalar headwind_at_altitude(const Scalar &altitude, double ref_wind = 10.0) {
    // Wind speed increases with altitude (power-law)
    Scalar wind_speed = power_law(altitude, ref_wind, 10.0, exponent::NEUTRAL);

    // Assume headwind (wind blowing against aircraft direction)
    return wind_speed;
}

/**
 * @brief Compute fuel consumption rate as function of headwind
 *
 * Simplified model: fuel consumption increases with headwind squared
 * (drag increases with airspeed squared to maintain ground speed)
 *
 * @tparam Scalar Numeric or symbolic type
 * @param headwind Headwind component [m/s]
 * @param cruise_speed Aircraft cruise airspeed [m/s]
 * @return Normalized fuel consumption rate
 */
template <typename Scalar>
Scalar fuel_rate(const Scalar &headwind, double cruise_speed = 100.0) {
    // Ground speed = airspeed - headwind
    // More headwind = more airspeed needed = more fuel
    Scalar airspeed_ratio = (cruise_speed + headwind) / cruise_speed;
    return airspeed_ratio * airspeed_ratio; // Quadratic with airspeed
}

/**
 * @brief Compute total mission cost for a 3-waypoint climb profile
 *
 * Integrates fuel consumption over altitude segments, accounting
 * for wind shear at each waypoint altitude.
 *
 * @tparam Scalar Numeric or symbolic type
 * @param alt1 First waypoint altitude [m]
 * @param alt2 Second waypoint altitude [m]
 * @param ref_wind Reference wind at 10m [m/s]
 * @return Integrated mission cost
 */
template <typename Scalar>
Scalar mission_cost(const Scalar &alt1, const Scalar &alt2, double ref_wind) {
    // Start at 100m, climb through alt1, alt2, then to 2000m
    Scalar alt_start(100.0);
    Scalar alt_end(2000.0);

    // Segment costs (trapezoidal integration)
    Scalar hw0 = headwind_at_altitude(alt_start, ref_wind);
    Scalar hw1 = headwind_at_altitude(alt1, ref_wind);
    Scalar hw2 = headwind_at_altitude(alt2, ref_wind);
    Scalar hw3 = headwind_at_altitude(alt_end, ref_wind);

    Scalar cost1 = (fuel_rate(hw0) + fuel_rate(hw1)) * (alt1 - alt_start) * 0.5;
    Scalar cost2 = (fuel_rate(hw1) + fuel_rate(hw2)) * (alt2 - alt1) * 0.5;
    Scalar cost3 = (fuel_rate(hw2) + fuel_rate(hw3)) * (alt_end - alt2) * 0.5;

    return cost1 + cost2 + cost3;
}

int main() {
    std::cout
        << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout
        << "║     Vulcan Wind Models - janus::Opti Optimization          ║\n";
    std::cout
        << "╚════════════════════════════════════════════════════════════╝\n\n";

    // =========================================================================
    // Part 1: Numeric Mode - Survey wind shear profile
    // =========================================================================
    std::cout << "=== Part 1: Wind Shear Profile (Numeric Mode) ===\n\n";

    double ref_wind = 10.0; // 10 m/s at 10m (moderate breeze)
    std::cout << "Reference wind: " << ref_wind << " m/s at 10m\n";
    std::cout << "Profile: Power-law with neutral exponent (1/4)\n\n";

    std::cout << "Altitude [m] | Wind [m/s] | Fuel Rate (rel)\n";
    std::cout << "-------------|------------|----------------\n";

    for (int alt = 100; alt <= 2000; alt += 200) {
        double hw = headwind_at_altitude(double(alt), ref_wind);
        double fr = fuel_rate(hw);
        std::cout << std::setw(12) << alt << " | " << std::setw(10)
                  << std::fixed << std::setprecision(2) << hw << " | "
                  << std::setprecision(4) << fr << "\n";
    }

    // =========================================================================
    // Part 2: Naive Strategy - Uniform climb
    // =========================================================================
    std::cout << "\n=== Part 2: Baseline - Uniform Climb ===\n\n";

    double uniform_alt1 = 100.0 + (2000.0 - 100.0) / 3.0;       // 733m
    double uniform_alt2 = 100.0 + 2.0 * (2000.0 - 100.0) / 3.0; // 1367m

    double baseline_cost = mission_cost(uniform_alt1, uniform_alt2, ref_wind);

    std::cout << "Uniform climb profile:\n";
    std::cout << "  Waypoint 1: " << std::fixed << std::setprecision(0)
              << uniform_alt1 << "m\n";
    std::cout << "  Waypoint 2: " << uniform_alt2 << "m\n";
    std::cout << "  Total cost: " << std::setprecision(2) << baseline_cost
              << "\n";

    // =========================================================================
    // Part 3: Optimization with janus::Opti
    // =========================================================================
    std::cout << "\n=== Part 3: Optimization with janus::Opti ===\n\n";

    janus::Opti opti;

    // Decision variables: intermediate waypoint altitudes
    auto alt1 = opti.variable(uniform_alt1); // Initialize at baseline
    auto alt2 = opti.variable(uniform_alt2);

    // Objective: minimize mission cost
    auto cost = mission_cost(alt1, alt2, ref_wind);
    opti.minimize(cost);

    // Constraints: altitudes must be ordered and within bounds
    opti.subject_to(alt1 >= 200.0);        // Min 200m for waypoint 1
    opti.subject_to(alt1 <= 1200.0);       // Max before halfway
    opti.subject_to(alt2 >= alt1 + 100.0); // alt2 > alt1 with gap
    opti.subject_to(alt2 <= 1800.0);       // Max before top

    std::cout << "Optimization setup:\n";
    std::cout << "  Variables: alt1, alt2 (waypoint altitudes)\n";
    std::cout << "  Objective: minimize integrated fuel cost\n";
    std::cout
        << "  Constraints: 200 <= alt1 <= 1200, alt1+100 <= alt2 <= 1800\n\n";

    // Solve
    auto solution = opti.solve({.verbose = false});

    double opt_alt1 = solution.value(alt1);
    double opt_alt2 = solution.value(alt2);
    double opt_cost = solution.value(cost);

    // =========================================================================
    // Part 4: Results
    // =========================================================================
    std::cout << "=== Optimization Results ===\n\n";

    std::cout << "Optimal climb profile:\n";
    std::cout << "  Waypoint 1: " << std::fixed << std::setprecision(1)
              << opt_alt1 << "m\n";
    std::cout << "  Waypoint 2: " << opt_alt2 << "m\n";
    std::cout << "  Total cost: " << std::setprecision(2) << opt_cost << "\n\n";

    double improvement = (baseline_cost - opt_cost) / baseline_cost * 100.0;
    std::cout << "Improvement over baseline: " << std::setprecision(2)
              << improvement << "%\n";
    std::cout << "Solver iterations: " << solution.num_iterations() << "\n";

    // =========================================================================
    // Part 5: MIL-Spec Turbulence Parameters
    // =========================================================================
    std::cout << "\n=== Part 5: MIL-Spec Turbulence at Waypoints ===\n\n";

    std::cout << "Turbulence intensity (σ) at optimal altitudes:\n";
    std::cout << "(Using MIL-F-8785C Moderate severity)\n\n";

    std::cout << "Altitude [m] | σ_u [m/s] | σ_v [m/s] | σ_w [m/s] | L_u [m]\n";
    std::cout << "-------------|-----------|-----------|-----------|--------\n";

    for (double alt : {100.0, opt_alt1, opt_alt2, 2000.0}) {
        auto params = mil_spec_params(alt, TurbulenceSeverity::Moderate);
        std::cout << std::setw(12) << std::fixed << std::setprecision(1) << alt
                  << " | " << std::setw(9) << std::setprecision(2)
                  << params.sigma_u << " | " << std::setw(9) << params.sigma_v
                  << " | " << std::setw(9) << params.sigma_w << " | "
                  << std::setw(6) << std::setprecision(1) << params.L_u << "\n";
    }

    // =========================================================================
    // Part 6: Export Computational Graphs
    // =========================================================================
    std::cout
        << "\n=== Part 6: Exporting Interactive Computational Graphs ===\n\n";

    // Create symbolic altitude for graph visualization
    auto alt_sym = janus::sym("altitude");

    // Export wind shear profile
    auto wind_profile = power_law(alt_sym, ref_wind, 10.0, exponent::NEUTRAL);
    janus::export_graph_html(wind_profile, "graph_wind_profile",
                             "Wind_Profile");
    std::cout << "✓ Exported: graph_wind_profile.html (power-law wind shear)\n";

    // Export fuel rate as function of altitude
    auto hw_sym = headwind_at_altitude(alt_sym, ref_wind);
    auto fr_sym = fuel_rate(hw_sym);
    janus::export_graph_html(fr_sym, "graph_fuel_rate",
                             "Fuel_Rate_vs_Altitude");
    std::cout << "✓ Exported: graph_fuel_rate.html (fuel consumption model)\n";

    // Export mission cost objective
    auto alt1_sym = janus::sym("alt1");
    auto alt2_sym = janus::sym("alt2");
    auto cost_sym = mission_cost(alt1_sym, alt2_sym, ref_wind);
    janus::export_graph_html(cost_sym, "graph_mission_cost", "Mission_Cost");
    std::cout << "✓ Exported: graph_mission_cost.html (full optimization "
                 "objective)\n";

    // Export logarithmic profile for comparison
    double u_star =
        friction_velocity_from_ref(ref_wind, 10.0, roughness::OPEN_TERRAIN);
    auto log_profile = logarithmic(alt_sym, u_star, roughness::OPEN_TERRAIN);
    janus::export_graph_html(log_profile, "graph_log_profile",
                             "Log_Wind_Profile");
    std::cout
        << "✓ Exported: graph_log_profile.html (logarithmic wind profile)\n";

    std::cout << "\nOpen these HTML files in a browser to explore the "
                 "computational graphs!\n";

    std::cout << "\n✓ Optimization complete using janus::Opti + Vulcan wind "
                 "infrastructure!\n";

    return 0;
}
