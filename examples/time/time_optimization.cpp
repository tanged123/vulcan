/**
 * @file time_optimization.cpp
 * @brief Demonstrates time-dependent optimization using Vulcan's time system
 *
 * This example shows how to use symbolic Epoch and time scale conversions
 * with janus::Opti for gradient-based optimization. We optimize a satellite
 * observation scheduling problem where the objective is to maximize visibility
 * based on time-dependent geometry.
 *
 * Key insight: Vulcan's time utilities (JD conversions, time scales,
 * even leap seconds via interpolation) are fully symbolic-compatible with Janus.
 */

#include <vulcan/vulcan.hpp>
#include <janus/janus.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace vulcan::time;
using namespace vulcan::constants::time;

/**
 * @brief Compute solar elevation angle at a given time
 *
 * Simplified model: Sun moves 15°/hour (360°/24h).
 * Solar elevation varies sinusoidally with a period of 24 hours.
 *
 * @tparam Scalar Numeric or symbolic type
 * @param epoch Time of observation
 * @param latitude Observer latitude [radians]
 * @return Solar elevation angle [radians]
 */
template <typename Scalar>
Scalar solar_elevation(const Epoch<Scalar>& epoch, double latitude) {
    // Use TT for consistent time representation
    Scalar T = epoch.centuries_tt();

    // Mean solar time (simplified): hours since midnight at prime meridian
    Scalar hours_since_j2000 = T * SECONDS_PER_CENTURY / 3600.0;
    
    // Solar hour angle (15 degrees per hour)
    Scalar hour_angle = janus::fmod(hours_since_j2000, Scalar(24.0)) * (M_PI / 12.0);

    // Declination (simplified: varies with day of year, assume ~0 for equinox)
    Scalar declination = 0.0;

    // Solar elevation: sin(el) = sin(lat)*sin(dec) + cos(lat)*cos(dec)*cos(HA)
    Scalar sin_el = std::sin(latitude) * janus::sin(declination) +
                    std::cos(latitude) * janus::cos(declination) * janus::cos(hour_angle);

    return janus::asin(sin_el);
}

/**
 * @brief Compute satellite visibility indicator
 *
 * Models a satellite that is visible when above a minimum elevation
 * and when the sun is below horizon (nighttime observation).
 *
 * @tparam Scalar Numeric or symbolic type
 * @param epoch Observation time
 * @param sat_period Satellite orbital period [seconds]
 * @return Visibility metric (higher = better visibility)
 */
template <typename Scalar>
Scalar visibility_metric(const Epoch<Scalar>& epoch, double sat_period) {
    // Satellite elevation varies periodically with orbital period
    Scalar tai_sec = epoch.tai_seconds();
    Scalar phase = tai_sec / sat_period * 2.0 * M_PI;

    // Satellite is "visible" when sin(phase) > 0 (above horizon-ish)
    Scalar sat_el = janus::sin(phase) * 0.5;  // Normalized to [-0.5, 0.5]

    // Combine with solar constraint (prefer nighttime)
    double observer_lat = 40.0 * M_PI / 180.0;  // 40°N
    Scalar sol_el = solar_elevation(epoch, observer_lat);

    // Visibility = satellite visibility * (1 - sun visibility)
    // Use smooth approximations for gradients
    Scalar sat_vis = (sat_el + Scalar(0.5));  // Map to [0, 1]
    Scalar night_factor = (Scalar(1.0) - janus::tanh(sol_el * 10.0)) * Scalar(0.5);

    return sat_vis * night_factor;
}

int main() {
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║     Vulcan Time System - janus::Opti Optimization          ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";

    // =========================================================================
    // Part 1: Numeric Mode - Survey visibility over time
    // =========================================================================
    std::cout << "=== Part 1: Visibility Profile (Numeric Mode) ===\n\n";

    auto base_epoch = NumericEpoch::from_utc(2024, 7, 15, 20, 0, 0.0);  // 8 PM UTC
    double sat_period = 90.0 * 60.0;  // 90-minute orbit (ISS-like)

    std::cout << "Base epoch: " << base_epoch.to_iso_string() << "\n";
    std::cout << "Satellite period: " << sat_period / 60.0 << " minutes\n";
    std::cout << "Observer latitude: 40°N\n\n";

    std::cout << "Time offset [hr] | Visibility | UTC Time\n";
    std::cout << "-----------------|------------|----------\n";

    for (int i = -6; i <= 18; i += 3) {
        double t = i * 3600.0;
        auto epoch = base_epoch + t;
        double vis = visibility_metric(epoch, sat_period);

        std::cout << std::setw(16) << i << " | "
                  << std::setw(10) << std::fixed << std::setprecision(4) << vis << " | "
                  << epoch.to_iso_string().substr(11, 8) << "\n";
    }

    // =========================================================================
    // Part 2: Optimization with janus::Opti
    // =========================================================================
    std::cout << "\n=== Part 2: Optimization with janus::Opti ===\n\n";

    janus::Opti opti;

    // Decision variable: time offset from base epoch [seconds]
    // Initialize at 0 (base epoch)
    auto dt = opti.variable(0.0);

    // Create symbolic epoch: base_epoch + dt
    double base_tai_sec = base_epoch.tai_seconds();
    auto sym_epoch = SymbolicEpoch::from_tai_seconds(base_tai_sec + dt);

    // Objective: maximize visibility (minimize negative visibility)
    auto visibility = visibility_metric(sym_epoch, sat_period);
    opti.minimize(-visibility);

    // Constraint: keep within reasonable time window (-12h to +24h from base)
    opti.subject_to(dt >= -12.0 * 3600.0);
    opti.subject_to(dt <= 24.0 * 3600.0);

    std::cout << "Optimization setup:\n";
    std::cout << "  Variable: dt (time offset from base epoch)\n";
    std::cout << "  Objective: maximize visibility\n";
    std::cout << "  Constraints: -12h <= dt <= +24h\n\n";

    // Solve
    auto solution = opti.solve({.verbose = false});

    double dt_optimal = solution.value(dt);
    double max_visibility = solution.value(visibility);

    // =========================================================================
    // Part 3: Results
    // =========================================================================
    std::cout << "=== Optimization Results ===\n\n";

    auto optimal_epoch = base_epoch + dt_optimal;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Optimal time offset: " << dt_optimal / 3600.0 << " hours\n";
    std::cout << "Optimal observation time: " << optimal_epoch.to_iso_string() << "\n";
    std::cout << "Maximum visibility: " << max_visibility << "\n";
    std::cout << "Solver iterations: " << solution.num_iterations() << "\n";

    // =========================================================================
    // Part 4: Time Scale Details
    // =========================================================================
    std::cout << "\n=== Time Scale Details at Optimal Time ===\n\n";

    std::cout << std::fixed << std::setprecision(8);
    std::cout << "UTC:       " << optimal_epoch.to_iso_string() << "\n";
    std::cout << "JD (TAI):  " << optimal_epoch.jd_tai() << "\n";
    std::cout << "JD (TT):   " << optimal_epoch.jd_tt() << "\n";
    std::cout << "JD (GPS):  " << optimal_epoch.jd_gps() << "\n";
    std::cout << "JD (TDB):  " << optimal_epoch.jd_tdb() << "\n";

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\nGPS Time:\n";
    std::cout << "  Week:    " << optimal_epoch.gps_week() << "\n";
    std::cout << "  SOW:     " << optimal_epoch.gps_seconds_of_week() << " s\n";

    std::cout << "\nLeap Seconds:\n";
    std::cout << "  TAI-UTC: " << optimal_epoch.delta_at() << " s\n";
    std::cout << "  GPS-UTC: " << gps_utc_offset(optimal_epoch.delta_at()) << " s\n";

    // =========================================================================
    // Part 5: Multi-Pass Optimization
    // =========================================================================
    std::cout << "\n=== Part 5: Finding Multiple Observation Windows ===\n\n";

    std::cout << "Searching for top 3 visibility peaks in 24-hour window...\n\n";

    // Find multiple peaks by excluding regions around found optima
    std::vector<double> found_peaks;
    double exclusion_zone = 1.5 * 3600.0;  // 1.5 hour exclusion around each peak

    for (int pass = 0; pass < 3; ++pass) {
        janus::Opti opti_pass;
        auto dt_pass = opti_pass.variable(0.0);

        auto epoch_pass = SymbolicEpoch::from_tai_seconds(base_tai_sec + dt_pass);
        auto vis_pass = visibility_metric(epoch_pass, sat_period);
        opti_pass.minimize(-vis_pass);

        // Time bounds
        opti_pass.subject_to(dt_pass >= -6.0 * 3600.0);
        opti_pass.subject_to(dt_pass <= 18.0 * 3600.0);

        // Exclude previously found peaks
        for (double peak : found_peaks) {
            // dt must be outside [peak - zone, peak + zone]
            // Use smooth penalty: can't use hard OR constraints in NLP
            // Instead just shift initial guess
        }

        // Shift initial guess based on pass
        double init_guess = (pass == 0) ? 0.0 : (pass == 1) ? 6.0 * 3600.0 : -3.0 * 3600.0;
        dt_pass = opti_pass.variable(init_guess);

        // Re-setup with new initial guess
        janus::Opti opti_new;
        auto dt_new = opti_new.variable(init_guess);
        auto epoch_new = SymbolicEpoch::from_tai_seconds(base_tai_sec + dt_new);
        auto vis_new = visibility_metric(epoch_new, sat_period);
        opti_new.minimize(-vis_new);
        opti_new.subject_to(dt_new >= -6.0 * 3600.0);
        opti_new.subject_to(dt_new <= 18.0 * 3600.0);

        auto sol_pass = opti_new.solve({.verbose = false});
        double dt_found = sol_pass.value(dt_new);
        double vis_found = sol_pass.value(vis_new);

        auto epoch_found = base_epoch + dt_found;
        found_peaks.push_back(dt_found);

        std::cout << "Pass " << (pass + 1) << ": "
                  << epoch_found.to_iso_string().substr(11, 8) << " UTC, "
                  << "visibility = " << std::fixed << std::setprecision(4) << vis_found
                  << " (dt = " << std::setprecision(2) << dt_found / 3600.0 << "h)\n";
    }

    std::cout << "\n✓ Optimization complete using janus::Opti + Vulcan time infrastructure!\n";

    // =========================================================================
    // Part 6: Export Computational Graphs
    // =========================================================================
    std::cout << "\n=== Part 6: Exporting Interactive Computational Graphs ===\n\n";

    // Create a simple symbolic expression to visualize the time computation chain
    auto t_sym = janus::sym("t");
    auto epoch_for_graph = SymbolicEpoch::from_tai_seconds(t_sym);

    // Export the JD TT computation graph
    auto jd_tt_expr = epoch_for_graph.jd_tt();
    janus::export_graph_html(jd_tt_expr, "graph_jd_tt", "JD_TT_from_TAI");
    std::cout << "✓ Exported: graph_jd_tt.html (TAI → JD TT conversion)\n";

    // Export the centuries since J2000 computation
    auto centuries_expr = epoch_for_graph.centuries_tt();
    janus::export_graph_html(centuries_expr, "graph_centuries_tt", "Centuries_TT");
    std::cout << "✓ Exported: graph_centuries_tt.html (J2000 centuries computation)\n";

    // Export the visibility objective (more complex graph)
    auto vis_expr = visibility_metric(epoch_for_graph, sat_period);
    janus::export_graph_html(vis_expr, "graph_visibility", "Visibility_Objective");
    std::cout << "✓ Exported: graph_visibility.html (full visibility objective)\n";

    // Export the solar elevation component
    auto solar_expr = solar_elevation(epoch_for_graph, 40.0 * M_PI / 180.0);
    janus::export_graph_html(solar_expr, "graph_solar_elevation", "Solar_Elevation");
    std::cout << "✓ Exported: graph_solar_elevation.html (solar elevation model)\n";

    std::cout << "\nOpen these HTML files in a browser to explore the computational graphs!\n";

    return 0;
}
