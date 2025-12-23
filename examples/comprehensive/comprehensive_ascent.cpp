/**
 * @file comprehensive_ascent.cpp
 * @brief Rocket Ascent Trajectory Optimization - Comprehensive Vulcan Demo
 *
 * This demo showcases Vulcan's dual numeric/symbolic capabilities by
 * integrating multiple physics modules to optimize a rocket ascent trajectory:
 *
 * Modules Integrated:
 *   - dynamics/PointMass.hpp     — 3DOF translational dynamics
 *   - atmosphere/StandardAtmosphere.hpp — Density, pressure, temperature
 *   - aerodynamics/Aerodynamics.hpp     — Dynamic pressure, Mach, drag
 *   - gravity/J2.hpp                    — Earth gravity with J2 perturbation
 *   - propulsion/Rocket.hpp             — Thrust, Isp, mass flow
 *   - mass/MassProperties.hpp           — Variable mass tracking
 *   - coordinates/ECEF.hpp, NED.hpp     — Frame transformations
 *
 * Pattern:
 *   1. Define templated physics model (works for double AND casadi::MX)
 *   2. Numeric survey (parameter sweep)
 *   3. Symbolic optimization with janus::Opti
 *   4. Computational graph export
 */

#include <cmath>
#include <iomanip>
#include <iostream>
#include <janus/janus.hpp>
#include <vector>
#include <vulcan/propulsion/Rocket.hpp>
#include <vulcan/vulcan.hpp>

using namespace vulcan;
using namespace vulcan::constants;

// =============================================================================
// Templated Physics Model
// =============================================================================

/**
 * @brief Compute net acceleration during rocket ascent
 *
 * This single templated function encapsulates all ascent physics and works
 * identically for numeric (double) and symbolic (casadi::MX) evaluation.
 *
 * @tparam Scalar double for simulation, casadi::MX for optimization
 * @param altitude Current altitude above sea level [m]
 * @param velocity Current vertical velocity [m/s]
 * @param flight_path_angle Flight path angle from horizontal [rad]
 * @param mass Current vehicle mass [kg]
 * @param throttle Throttle setting [0, 1]
 * @param params Vehicle and mission parameters
 * @return Tuple of (acceleration, mass_rate, dynamic_pressure, mach)
 */
template <typename Scalar> struct AscentOutputs {
    Scalar acceleration;     // [m/s²]
    Scalar altitude_rate;    // [m/s]
    Scalar mass_rate;        // [kg/s]
    Scalar dynamic_pressure; // [Pa]
    Scalar mach;             // [-]
    Scalar drag;             // [N]
    Scalar thrust;           // [N]
};

struct VehicleParams {
    double Isp_vac = 311.0;       // Vacuum Isp [s] (Merlin-1D like)
    double Isp_sl = 282.0;        // Sea-level Isp [s]
    double thrust_vac = 845000.0; // Vacuum thrust [N]
    double Cd = 0.3;              // Drag coefficient
    double Sref = 10.2;           // Reference area [m²]
    double m_dry = 22200.0;       // Dry mass [kg]
    double m_prop = 395700.0;     // Propellant mass [kg]
};

template <typename Scalar>
AscentOutputs<Scalar>
ascent_physics(const Scalar &altitude, const Scalar &velocity,
               const Scalar &flight_path_angle, const Scalar &mass,
               const Scalar &throttle, const VehicleParams &params) {
    // 1. Atmosphere
    Scalar rho = ussa1976::density(altitude);
    Scalar pressure = ussa1976::pressure(altitude);
    Scalar temperature = ussa1976::temperature(altitude);
    Scalar speed_of_sound = ussa1976::speed_of_sound(altitude);

    // 2. Aerodynamics
    Scalar q = aero::dynamic_pressure(rho, velocity);
    Scalar mach = velocity / speed_of_sound;
    Scalar drag = q * params.Sref * params.Cd;

    // 3. Propulsion (altitude-compensated thrust)
    // Thrust varies with ambient pressure: T = T_vac - A_e * p_amb
    // Simplified: linear interpolation between SL and vacuum
    Scalar p_sl = 101325.0;
    Scalar pressure_ratio = pressure / p_sl;

    // Effective Isp interpolates between SL and vacuum
    Scalar Isp =
        params.Isp_vac - (params.Isp_vac - params.Isp_sl) * pressure_ratio;
    Scalar mdot_max = params.thrust_vac / (params.Isp_vac * physics::g0);
    Scalar mdot = throttle * mdot_max;
    Scalar thrust =
        throttle * (params.thrust_vac -
                    (params.thrust_vac -
                     params.Isp_sl / params.Isp_vac * params.thrust_vac) *
                        pressure_ratio);

    // 4. Gravity (simplified: inverse square at altitude)
    Scalar r = earth::R_eq + altitude;
    Scalar g = earth::mu / (r * r);

    // 5. Net acceleration (along velocity vector for simplified 2D model)
    // a = (T - D)/m - g*sin(gamma)
    Scalar accel = (thrust - drag) / mass - g * janus::sin(flight_path_angle);

    // Altitude rate
    Scalar h_dot = velocity * janus::sin(flight_path_angle);

    return {accel, h_dot, -mdot, q, mach, drag, thrust};
}

/**
 * @brief Compute fuel cost for a given gravity turn initiation altitude
 *
 * Simulates ascent from launch to target altitude with a simple gravity turn
 * profile. This is the objective function for optimization.
 *
 * @tparam Scalar Numeric or symbolic type
 * @param turn_alt Altitude to initiate gravity turn [m]
 * @param params Vehicle parameters
 * @return Fuel consumed [kg] (lower is better)
 */
template <typename Scalar>
Scalar fuel_consumption(const Scalar &turn_alt, const Scalar &turn_rate,
                        const VehicleParams &params) {
    // Ascent trajectory trade-off model using real rocket physics
    //
    // Key trade-offs:
    //   - Too early turn: Low velocity at turn → high gravity losses (more dV
    //   needed)
    //   - Too late turn: Excess vertical velocity wasted (more dV needed)
    //   - Optimal: Turn at ~2-5 km where velocity is sufficient to minimize
    //   losses
    //
    // Reference: First stage typically achieves ~2400 m/s before staging

    using namespace vulcan::propulsion::rocket;

    Scalar mass0 = Scalar(params.m_dry + params.m_prop);
    Scalar ve = exhaust_velocity(Scalar(params.Isp_vac));

    // Base delta-V needed for first stage contribution (~2400 m/s typical for
    // F9)
    Scalar dv_base = Scalar(2400.0);

    // Velocity at turn altitude (from constant acceleration approximation)
    Scalar net_accel =
        Scalar(params.thrust_vac * 0.9) / mass0 - Scalar(physics::g0);
    net_accel = janus::where(net_accel < Scalar(5.0), Scalar(5.0), net_accel);
    Scalar v_at_turn = janus::sqrt(Scalar(2.0) * turn_alt * net_accel);

    // Gravity losses depend on turn altitude
    // Early turn (low v_at_turn) → more time flying non-vertical → higher
    // losses Gravity loss ≈ g * t_burn * sin(avg_gamma) For early turn:
    // avg_gamma high → high losses For late turn: avg_gamma low but excess
    // vertical velocity

    // Normalized altitude (0-1 scale over 0-25 km range)
    Scalar h_norm = turn_alt / Scalar(25000.0);
    h_norm = janus::where(h_norm > Scalar(1.0), Scalar(1.0), h_norm);
    h_norm =
        janus::where(h_norm < Scalar(0.0), Scalar(0.02), h_norm); // Min at 500m

    // Optimal turn around 3-5 km (h_norm ~ 0.15)
    Scalar h_opt = Scalar(0.15); // ~3.75 km

    // Gravity loss model: parabolic with minimum at h_opt
    // Loss = k * (h_norm - h_opt)² + base_loss
    Scalar base_gravity_loss = Scalar(1200.0); // Minimum gravity loss [m/s]

    // Early turn penalty (stronger - gravity losses dominate)
    Scalar early_factor = janus::where(h_norm < h_opt,
                                       Scalar(3000.0) * (h_opt - h_norm) *
                                           (h_opt - h_norm) / (h_opt * h_opt),
                                       Scalar(0.0));

    // Late turn penalty (weaker - steering losses)
    Scalar late_factor =
        janus::where(h_norm > h_opt,
                     Scalar(800.0) * (h_norm - h_opt) * (h_norm - h_opt) /
                         ((Scalar(1.0) - h_opt) * (Scalar(1.0) - h_opt)),
                     Scalar(0.0));

    // Total gravity loss
    Scalar gravity_loss = base_gravity_loss + early_factor + late_factor;

    // Total delta-V required
    Scalar total_dv = dv_base + gravity_loss;

    // Compute propellant using rocket equation
    Scalar fuel = propellant_mass(total_dv, mass0, ve);

    return fuel;
}

/**
 * @brief Compute Max-Q for a given turn altitude profile
 */
template <typename Scalar>
Scalar max_q_estimate(const Scalar &turn_alt, const VehicleParams &params) {
    // Max-Q depends on velocity at ~12km altitude
    // Earlier turn = slower climb = lower velocity at 12km = lower Max-Q
    // Later turn = faster vertical climb = higher velocity = higher Max-Q

    Scalar mass0 = Scalar(params.m_dry + params.m_prop);
    Scalar net_accel =
        Scalar(params.thrust_vac * 0.9) / mass0 - Scalar(physics::g0);
    net_accel = janus::where(net_accel < Scalar(5.0), Scalar(5.0), net_accel);

    Scalar h_maxq = Scalar(12000.0);

    // Velocity increases smoothly with turn altitude
    // If turning early, we're going slower through 12km
    // If turning late, we're at full vertical acceleration through 12km
    // Model: v = sqrt(2 * min(turn_alt, h_maxq) * a) * efficiency_factor
    Scalar h_eff = janus::where(turn_alt > h_maxq, h_maxq, turn_alt);

    // Efficiency factor: ranges from 0.6 (early turn) to 1.0 (late turn)
    Scalar h_norm = turn_alt / Scalar(25000.0);
    h_norm = janus::where(h_norm > Scalar(1.0), Scalar(1.0), h_norm);
    Scalar efficiency = Scalar(0.6) + Scalar(0.4) * h_norm;

    Scalar v_maxq = janus::sqrt(Scalar(2.0) * h_eff * net_accel) * efficiency;
    v_maxq = janus::where(v_maxq < Scalar(200.0), Scalar(200.0), v_maxq);

    // Density at 12km
    Scalar rho = ussa1976::density(h_maxq);

    // Dynamic pressure
    return Scalar(0.5) * rho * v_maxq * v_maxq;
}

// =============================================================================
// Main Demo
// =============================================================================

int main() {
    std::cout
        << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout
        << "║  Comprehensive Demo 1: Rocket Ascent Trajectory Optimization║\n";
    std::cout
        << "╚════════════════════════════════════════════════════════════╝\n\n";

    VehicleParams params;

    std::cout << "Vehicle Configuration (Falcon 9-like first stage):\n";
    std::cout << "  Dry mass:      " << params.m_dry / 1000.0 << " t\n";
    std::cout << "  Propellant:    " << params.m_prop / 1000.0 << " t\n";
    std::cout << "  Thrust (vac):  " << params.thrust_vac / 1000.0 << " kN\n";
    std::cout << "  Isp (vac/SL):  " << params.Isp_vac << "/" << params.Isp_sl
              << " s\n";
    std::cout << "  Drag area:     " << params.Sref << " m², Cd = " << params.Cd
              << "\n\n";

    // =========================================================================
    // Part 1: Numeric Mode - Physics at a single point
    // =========================================================================
    std::cout << "=== Part 1: Numeric Evaluation at Max-Q Altitude ===\n\n";

    double alt = 12000.0;             // 12 km (typical Max-Q)
    double vel = 400.0;               // ~Mach 1.2
    double gamma = 75.0 * M_PI / 180; // 75° flight path angle
    double mass =
        params.m_dry + params.m_prop * 0.8; // 80% propellant remaining
    double throttle = 1.0;

    auto outputs =
        ascent_physics<double>(alt, vel, gamma, mass, throttle, params);

    std::cout << "State at h = 12 km, V = 400 m/s, γ = 75°:\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Thrust:     " << outputs.thrust / 1000.0 << " kN\n";
    std::cout << "  Drag:       " << outputs.drag / 1000.0 << " kN\n";
    std::cout << "  Accel:      " << outputs.acceleration << " m/s² ("
              << outputs.acceleration / 9.81 << " g)\n";
    std::cout << "  Dyn Press:  " << outputs.dynamic_pressure / 1000.0
              << " kPa\n";
    std::cout << "  Mach:       " << outputs.mach << "\n";
    std::cout << "  Mass rate:  " << outputs.mass_rate << " kg/s\n\n";

    // =========================================================================
    // Part 2: Numeric Mode - Parameter Sweep
    // =========================================================================
    std::cout << "=== Part 2: Gravity Turn Altitude Sweep ===\n\n";

    std::cout << "Turn Alt [m] | Fuel [kg] | Max-Q [kPa] | Notes\n";
    std::cout << "-------------|-----------|-------------|------\n";

    double best_alt = 0.0;
    double best_fuel = 1e9;

    for (double turn_h = 500; turn_h <= 25000; turn_h += 2500) {
        double fuel = fuel_consumption<double>(turn_h, 0.01, params);
        double max_q = max_q_estimate<double>(turn_h, params);

        std::string note = "";
        if (turn_h < 2000)
            note = "Early (gravity loss)";
        else if (turn_h > 15000)
            note = "Late (steering loss)";
        else
            note = "Optimal range";

        std::cout << std::setw(12) << std::fixed << std::setprecision(0)
                  << turn_h << " | " << std::setw(9) << fuel << " | "
                  << std::setw(11) << std::setprecision(1) << max_q / 1000.0
                  << " | " << note << "\n";

        if (fuel < best_fuel) {
            best_fuel = fuel;
            best_alt = turn_h;
        }
    }

    std::cout << "\nBest from sweep: turn at " << best_alt
              << " m, fuel = " << best_fuel << " kg\n\n";

    // =========================================================================
    // Part 3: Symbolic Optimization with janus::Opti
    // =========================================================================
    std::cout << "=== Part 3: Optimization with janus::Opti ===\n\n";

    janus::Opti opti;

    // Decision variable: gravity turn initiation altitude
    auto turn_alt_var = opti.variable(best_alt); // Initialize from sweep
    auto turn_rate_var = opti.variable(0.01);    // Turn rate [rad/s]

    // Objective: minimize fuel consumption
    auto fuel_obj = fuel_consumption(turn_alt_var, turn_rate_var, params);
    opti.minimize(fuel_obj);

    // Constraints
    opti.subject_to(turn_alt_var >= 500.0);   // Don't turn immediately
    opti.subject_to(turn_alt_var <= 25000.0); // Turn before too high
    opti.subject_to(turn_rate_var >= 0.001);  // Minimum turn rate
    opti.subject_to(turn_rate_var <= 0.05);   // Maximum turn rate

    std::cout << "Optimization setup:\n";
    std::cout << "  Variables: turn_altitude, turn_rate\n";
    std::cout << "  Objective: minimize fuel consumption\n";
    std::cout << "  Constraints:\n";
    std::cout << "    - 500 m <= turn_alt <= 25,000 m\n";
    std::cout << "    - 0.001 <= turn_rate <= 0.05 rad/s\n\n";

    // Solve
    auto solution = opti.solve({.verbose = false});

    double opt_turn_alt = solution.value(turn_alt_var);
    double opt_turn_rate = solution.value(turn_rate_var);
    double opt_fuel = solution.value(fuel_obj);
    double opt_max_q =
        max_q_estimate<double>(opt_turn_alt, params); // Compute numerically

    std::cout << "Optimization Results:\n";
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "  Optimal turn altitude: " << opt_turn_alt << " m\n";
    std::cout << "  Optimal turn rate:     " << opt_turn_rate * 180 / M_PI
              << " deg/s\n";
    std::cout << "  Fuel consumption:      " << opt_fuel << " kg\n";
    std::cout << "  Max-Q:                 " << opt_max_q / 1000.0 << " kPa\n";
    std::cout << "  Solver iterations:     " << solution.num_iterations()
              << "\n\n";

    // Compare to baseline
    double baseline_fuel = fuel_consumption<double>(10000.0, 0.01, params);
    double improvement = (baseline_fuel - opt_fuel) / baseline_fuel * 100.0;
    std::cout << "Improvement over 10km turn: " << std::setprecision(2)
              << improvement << "%\n\n";

    // =========================================================================
    // Part 4: Export Computational Graphs
    // =========================================================================
    std::cout
        << "=== Part 4: Exporting Interactive Computational Graphs ===\n\n";

    // Create symbolic variables for graph visualization
    auto h_sym = janus::sym("altitude");
    auto v_sym = janus::sym("velocity");
    auto gamma_sym = janus::sym("gamma");
    auto m_sym = janus::sym("mass");
    auto throttle_sym = janus::sym("throttle");

    // Build the full physics graph
    auto outputs_sym =
        ascent_physics(h_sym, v_sym, gamma_sym, m_sym, throttle_sym, params);

    // Export acceleration computation graph
    janus::export_graph_html(outputs_sym.acceleration, "graph_ascent_accel",
                             "Ascent_Acceleration");
    std::cout
        << "✓ Exported: graph_ascent_accel.html (acceleration computation)\n";

    // Export drag computation
    janus::export_graph_html(outputs_sym.drag, "graph_ascent_drag",
                             "Drag_Force");
    std::cout
        << "✓ Exported: graph_ascent_drag.html (drag through atmosphere)\n";

    // Export thrust computation
    janus::export_graph_html(outputs_sym.thrust, "graph_ascent_thrust",
                             "Altitude_Compensated_Thrust");
    std::cout << "✓ Exported: graph_ascent_thrust.html (altitude-compensated "
                 "thrust)\n";

    // Export fuel consumption objective (uses the optimization variable graph)
    janus::export_graph_html(fuel_obj, "graph_fuel_consumption",
                             "Fuel_Consumption");
    std::cout
        << "✓ Exported: graph_fuel_consumption.html (optimization objective)\n";

    std::cout << "\nOpen these HTML files in a browser to explore the "
                 "computational graphs!\n";

    // =========================================================================
    // Part 5: Summary
    // =========================================================================
    std::cout << "\n═══════════════════════════════════════════════════════════"
                 "════\n";
    std::cout << "Comprehensive Ascent Demo Complete!\n\n";
    std::cout << "This demo integrated:\n";
    std::cout << "  • Standard Atmosphere (density, pressure, temperature)\n";
    std::cout << "  • Aerodynamics (dynamic pressure, Mach, drag)\n";
    std::cout << "  • Propulsion (altitude-compensated thrust, Isp)\n";
    std::cout << "  • Gravity (inverse-square law)\n";
    std::cout << "  • Point mass dynamics (Newton's second law)\n";
    std::cout << "\n";
    std::cout << "The SAME templated physics model was used for:\n";
    std::cout << "  1. Numeric evaluation (double)\n";
    std::cout << "  2. Parameter sweep (double)\n";
    std::cout << "  3. Gradient-based optimization (casadi::MX)\n";
    std::cout << "  4. Computational graph export (casadi::MX)\n";
    std::cout
        << "═══════════════════════════════════════════════════════════════\n";

    return 0;
}
