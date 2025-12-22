/**
 * @file orbital_optimization.cpp
 * @brief Demonstrates orbital mechanics optimization using janus::Opti
 *
 * This example shows how to use Vulcan's orbital mechanics with Janus for
 * gradient-based optimization. We solve several classic orbital mechanics
 * problems:
 *
 * 1. Optimal Hohmann vs Bielliptic transfer selection
 * 2. Minimum-fuel lunar injection timing
 * 3. Orbital rendezvous phasing optimization
 *
 * Key insight: Vulcan's orbital utilities (Kepler solver, state conversions,
 * ephemeris) are fully symbolic-compatible with Janus optimization.
 */

#include <cmath>
#include <iomanip>
#include <iostream>
#include <janus/janus.hpp>
#include <vulcan/vulcan.hpp>

using namespace vulcan;
using namespace vulcan::orbital;
using Scalar = janus::SymbolicScalar;

int main() {
    std::cout
        << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout
        << "║     Vulcan Orbital Mechanics - janus::Opti Optimization    ║\n";
    std::cout
        << "╚════════════════════════════════════════════════════════════╝\n\n";

    // =========================================================================
    // Problem 1: Optimal Transfer Orbit Selection
    // =========================================================================
    std::cout << "=== Problem 1: Optimal Intermediate Apoapsis for Bielliptic "
                 "===\n\n";

    // For transfers with large radius ratios (r2/r1 > 11.94), bielliptic can
    // be more efficient than Hohmann. Let's optimize the intermediate apoapsis.

    double r1_val = 7000.0e3;   // Initial orbit: 7000 km
    double r2_val = 105000.0e3; // Final orbit: 105,000 km (ratio ~ 15)

    std::cout << "Initial orbit radius: " << r1_val / 1e3 << " km\n";
    std::cout << "Final orbit radius:   " << r2_val / 1e3 << " km\n";
    std::cout << "Radius ratio:         " << r2_val / r1_val
              << " (> 11.94 threshold)\n\n";

    // Compare Hohmann (numeric)
    double dv_hohmann = transfer::hohmann_total_delta_v(r1_val, r2_val);
    std::cout << "Hohmann Δv:           " << dv_hohmann / 1000.0 << " km/s\n\n";

    // Optimize bielliptic intermediate apoapsis
    janus::Opti opti1;

    // Decision variable: intermediate apoapsis radius
    auto r_b = opti1.variable(150000.0e3); // Initial guess: 150,000 km

    // Wrap constants as symbolic for type consistency
    Scalar r1 = r1_val;
    Scalar r2 = r2_val;

    // Compute bielliptic delta-v symbolically
    auto [dv1, dv2, dv3] = transfer::bielliptic_delta_v(r1, r2, r_b);
    auto total_dv = dv1 + dv2 + dv3;

    opti1.minimize(total_dv);

    // Constraints: r_b must be larger than both r1 and r2
    opti1.subject_to(r_b >= r2_val * 1.01); // At least 1% larger than r2
    opti1.subject_to(r_b <= r2_val * 10.0); // Upper bound for practicality

    auto solution1 = opti1.solve({.verbose = false});

    double r_b_opt = solution1.value(r_b);
    double dv_bielliptic = solution1.value(total_dv);

    std::cout << "Optimized Bielliptic Transfer:\n";
    std::cout << "  Optimal r_b:        " << r_b_opt / 1e3 << " km\n";
    std::cout << "  Bielliptic Δv:      " << dv_bielliptic / 1000.0
              << " km/s\n";
    std::cout << "  Savings vs Hohmann: "
              << (dv_hohmann - dv_bielliptic) / 1000.0 << " km/s ("
              << (dv_hohmann - dv_bielliptic) / dv_hohmann * 100 << "%)\n";
    std::cout << "  Solver iterations:  " << solution1.num_iterations()
              << "\n\n";

    // =========================================================================
    // Problem 2: Lunar Transfer Window Optimization
    // =========================================================================
    std::cout << "=== Problem 2: Optimal Lunar Transfer Timing ===\n\n";

    // Find the best time to launch to minimize distance to Moon at arrival.
    // We'll optimize the departure time to align with Moon position.

    double jd_base = 2451545.0;                 // J2000 epoch
    double transfer_time = 3.0 * 24.0 * 3600.0; // ~3 day transfer (simplified)

    std::cout << "Transfer duration: ~3 days (simplified)\n";
    std::cout << "Optimizing departure time to minimize arrival distance to "
                 "Moon...\n\n";

    janus::Opti opti2;

    // Decision variable: departure offset from base epoch [days]
    auto dt_days = opti2.variable(0.0);

    // Departure epoch (symbolic)
    auto jd_depart = jd_base + dt_days;

    // Arrival epoch
    auto jd_arrive = jd_depart + transfer_time / 86400.0;

    // Moon position at arrival
    auto r_moon = ephemeris::analytical::moon_position_eci(jd_arrive);

    // Simplified: spacecraft on a straight-line trajectory from Earth
    // Target should be where Moon will be at arrival
    // We want Moon to be at a favorable distance (not too close for capture,
    // but aligned with transfer trajectory)

    // For simplicity: minimize deviation from mean lunar distance at intercept
    auto moon_dist = janus::norm(r_moon);
    auto dist_error = (moon_dist - constants::moon::mean_distance);

    // Also consider Moon phase for lighting (simplified: x-component favorable)
    // Want Moon ahead of Earth in orbit for favorable approach geometry
    auto geometry_score = -r_moon(1); // Favor positive y (Moon "ahead")

    // Combined objective
    opti2.minimize(dist_error * dist_error * 1e-18 + geometry_score * 1e-9);

    // Constraint: search within 30-day window
    opti2.subject_to(dt_days >= 0.0);
    opti2.subject_to(dt_days <= 30.0);

    auto solution2 = opti2.solve({.verbose = false});

    double optimal_depart_offset = solution2.value(dt_days);
    double jd_optimal_depart = jd_base + optimal_depart_offset;
    double jd_optimal_arrive = jd_optimal_depart + transfer_time / 86400.0;

    // Evaluate Moon position at optimal arrival
    Vec3<double> r_moon_opt =
        ephemeris::analytical::moon_position_eci(jd_optimal_arrive);
    double moon_distance = janus::norm(r_moon_opt);

    std::cout << "Optimal departure: J2000 + " << optimal_depart_offset
              << " days\n";
    std::cout << "Moon distance at arrival: " << moon_distance / 1e3 << " km\n";
    std::cout << "Moon position (ECI): [" << r_moon_opt(0) / 1e6 << ", "
              << r_moon_opt(1) / 1e6 << ", " << r_moon_opt(2) / 1e6
              << "] × 10⁶ m\n";
    std::cout << "Solver iterations: " << solution2.num_iterations() << "\n\n";

    // =========================================================================
    // Problem 3: Orbital Rendezvous Phase Angle
    // =========================================================================
    std::cout << "=== Problem 3: Rendezvous Phase Angle Optimization ===\n\n";

    // Two satellites in circular coplanar orbits at different altitudes.
    // Optimize the phase angle for a Hohmann rendezvous.

    double r_chaser = constants::earth::R_eq + 400.0e3; // Chaser at 400 km
    double r_target = constants::earth::R_eq + 450.0e3; // Target at 450 km

    std::cout << "Chaser altitude:  400 km\n";
    std::cout << "Target altitude:  450 km\n\n";

    // Hohmann transfer time
    double t_transfer = transfer::hohmann_transfer_time(r_chaser, r_target);

    // Mean motions
    double n_chaser = quantities::mean_motion(r_chaser);
    double n_target = quantities::mean_motion(r_target);

    // Target travels during transfer
    double angle_traveled = n_target * t_transfer;

    // Required lead angle for target (target must be ahead by pi -
    // angle_traveled)
    double lead_angle = M_PI - angle_traveled;

    // Wrap to [0, 2π)
    while (lead_angle < 0)
        lead_angle += 2 * M_PI;
    while (lead_angle >= 2 * M_PI)
        lead_angle -= 2 * M_PI;

    // Wait time for phase alignment
    double phase_rate = n_chaser - n_target; // Relative angular rate
    double wait_time = lead_angle / phase_rate;

    std::cout << "Transfer time:     " << t_transfer / 60.0 << " minutes\n";
    std::cout << "Required lead angle: " << lead_angle * 180.0 / M_PI << "°\n";
    std::cout << "Wait time for alignment: " << wait_time / 60.0
              << " minutes\n\n";

    // Optimize using janus::Opti (verify our analytical solution)
    janus::Opti opti3;

    auto phase_angle = opti3.variable(M_PI); // Initial guess

    // After transfer, target should be at the same position
    // target_angle = phase_angle + n_target * t_transfer
    // chaser arrives at angle = π (opposite side after half orbit)
    // We want: phase_angle + n_target * t_transfer ≡ π (mod 2π)

    auto target_final = phase_angle + n_target * t_transfer;
    auto error = janus::sin(target_final - M_PI); // Zero when aligned

    opti3.minimize(error * error);

    // Constraint: phase angle in [0, 2π]
    opti3.subject_to(phase_angle >= 0.0);
    opti3.subject_to(phase_angle <= 2.0 * M_PI);

    auto solution3 = opti3.solve({.verbose = false});

    double phase_optimized = solution3.value(phase_angle);

    std::cout << "Optimized phase angle: " << phase_optimized * 180.0 / M_PI
              << "°\n";
    std::cout << "Analytical solution:   " << lead_angle * 180.0 / M_PI
              << "°\n";
    std::cout << "Solver iterations:     " << solution3.num_iterations()
              << "\n\n";

    // =========================================================================
    // Problem 4: Minimum Δv Plane Change Location
    // =========================================================================
    std::cout << "=== Problem 4: Optimal Plane Change Location ===\n\n";

    // For a combined altitude + inclination change, it's most efficient to
    // perform the plane change at apoapsis where velocity is lowest.

    double r_initial = 6678.0e3;              // 300 km circular
    double r_final = 42164.0e3;               // GEO
    double delta_i_val = 28.5 * M_PI / 180.0; // 28.5° inclination change

    std::cout << "Initial: 300 km circular, 28.5° inclination\n";
    std::cout << "Final:   GEO, 0° inclination\n\n";

    janus::Opti opti4;

    // Strategy: split plane change between first and second burn
    auto plane_fraction = opti4.variable(0.5); // Fraction done at first burn

    // First burn: raise apoapsis + partial plane change
    auto delta_i_1 = delta_i_val * plane_fraction;
    auto delta_i_2 = delta_i_val * (Scalar(1.0) - plane_fraction);

    // Velocities (as symbolic for type consistency)
    Scalar v1_circ = quantities::circular_velocity(r_initial);
    Scalar v1_transfer =
        quantities::velocity(r_initial, (r_initial + r_final) / 2.0);
    Scalar v2_transfer =
        quantities::velocity(r_final, (r_initial + r_final) / 2.0);
    Scalar v2_circ = quantities::circular_velocity(r_final);

    // Combined Δv at each burn
    auto dv_burn1 =
        transfer::combined_maneuver_delta_v(v1_circ, v1_transfer, delta_i_1);
    auto dv_burn2 =
        transfer::combined_maneuver_delta_v(v2_transfer, v2_circ, delta_i_2);
    auto total_dv4 = dv_burn1 + dv_burn2;

    opti4.minimize(total_dv4);

    opti4.subject_to(plane_fraction >= 0.0);
    opti4.subject_to(plane_fraction <= 1.0);

    auto solution4 = opti4.solve({.verbose = false});

    double opt_fraction = solution4.value(plane_fraction);
    double opt_dv = solution4.value(total_dv4);

    std::cout << "Optimal plane change split:\n";
    std::cout << "  At periapsis (burn 1): " << opt_fraction * 100.0 << "%\n";
    std::cout << "  At apoapsis (burn 2):  " << (1.0 - opt_fraction) * 100.0
              << "%\n";
    std::cout << "  Total Δv:              " << opt_dv / 1000.0 << " km/s\n";
    std::cout << "  Solver iterations:     " << solution4.num_iterations()
              << "\n\n";

    // Compare with all-at-apoapsis (numeric)
    double v1_t = quantities::velocity(r_initial, (r_initial + r_final) / 2.0);
    double v1_c = quantities::circular_velocity(r_initial);
    double v2_t = quantities::velocity(r_final, (r_initial + r_final) / 2.0);
    double v2_c = quantities::circular_velocity(r_final);
    double dv_all_apo = (v1_t - v1_c) + transfer::combined_maneuver_delta_v(
                                            v2_t, v2_c, delta_i_val);
    std::cout << "All plane change at apoapsis: " << dv_all_apo / 1000.0
              << " km/s\n";

    // =========================================================================
    // Summary
    // =========================================================================
    std::cout << "\n═══════════════════════════════════════════════════════════"
                 "════\n";
    std::cout
        << "✓ All optimizations complete using janus::Opti + Vulcan orbital!\n";
    std::cout
        << "═══════════════════════════════════════════════════════════════\n";

    return 0;
}
