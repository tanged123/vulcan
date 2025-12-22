// Orbital Mechanics Demo
// Demonstrates orbital state conversions, anomaly solvers, transfers, and
// ephemeris
#include <iomanip>
#include <iostream>
#include <vulcan/vulcan.hpp>

using namespace vulcan;
using namespace vulcan::orbital;

int main() {
    std::cout << std::fixed << std::setprecision(4);

    std::cout << "=== Vulcan Orbital Mechanics Demo ===\n\n";

    // =========================================================================
    // 1. Orbital Quantities
    // =========================================================================
    std::cout << "--- 1. Orbital Quantities ---\n\n";

    // ISS orbit parameters
    double r_iss = constants::earth::R_eq + 400.0e3; // 400 km altitude
    double T_iss = quantities::period(r_iss);
    double v_iss = quantities::circular_velocity(r_iss);
    double v_esc = quantities::escape_velocity(r_iss);

    std::cout << "ISS-like orbit (h = 400 km):\n";
    std::cout << "  Period:          " << T_iss / 60.0 << " minutes\n";
    std::cout << "  Circular velocity: " << v_iss / 1000.0 << " km/s\n";
    std::cout << "  Escape velocity:   " << v_esc / 1000.0 << " km/s\n";
    std::cout << "  v_esc/v_circ:      " << v_esc / v_iss
              << " (should be √2 ≈ 1.414)\n\n";

    // GEO orbit
    double r_geo = 42164.0e3;
    double T_geo = quantities::period(r_geo);
    std::cout << "GEO orbit:\n";
    std::cout << "  Period: " << T_geo / 3600.0
              << " hours (sidereal day ≈ 23.93 hr)\n\n";

    // =========================================================================
    // 2. Kepler's Equation (Anomaly Conversions)
    // =========================================================================
    std::cout << "--- 2. Kepler's Equation ---\n\n";

    double e = 0.5; // Elliptical orbit
    double M = 1.0; // Mean anomaly

    double E = anomaly::mean_to_eccentric(M, e);
    double nu = anomaly::eccentric_to_true(E, e);

    std::cout << "Orbit with e = " << e << ":\n";
    std::cout << "  Mean anomaly M:      " << M << " rad\n";
    std::cout << "  Eccentric anomaly E: " << E << " rad\n";
    std::cout << "  True anomaly ν:      " << nu << " rad ("
              << nu * 180.0 / M_PI << "°)\n";

    // Verify round-trip
    double M_check = anomaly::true_to_mean(nu, e);
    std::cout << "  Round-trip M:        " << M_check
              << " rad (error: " << std::abs(M - M_check) << ")\n\n";

    // =========================================================================
    // 3. State Conversions (Cartesian ↔ Keplerian)
    // =========================================================================
    std::cout << "--- 3. State Conversions ---\n\n";

    // Create a circular equatorial orbit
    double a = 7000.0e3;
    double v_circ = quantities::circular_velocity(a);

    Vec3<double> r_cart, v_cart;
    r_cart << a, 0.0, 0.0;
    v_cart << 0.0, v_circ, 0.0;

    auto oe = elements::cartesian_to_keplerian(r_cart, v_cart);

    std::cout << "Circular equatorial orbit (a = 7000 km):\n";
    std::cout << "  Position r: [" << r_cart(0) / 1e3 << ", " << r_cart(1) / 1e3
              << ", " << r_cart(2) / 1e3 << "] km\n";
    std::cout << "  Velocity v: [" << v_cart(0) / 1e3 << ", " << v_cart(1) / 1e3
              << ", " << v_cart(2) / 1e3 << "] km/s\n";
    std::cout << "\n";
    std::cout << "Keplerian elements:\n";
    std::cout << "  a = " << oe.a / 1e3 << " km\n";
    std::cout << "  e = " << oe.e << "\n";
    std::cout << "  i = " << oe.i * 180.0 / M_PI << "°\n";
    std::cout << "  Ω = " << oe.Omega * 180.0 / M_PI << "°\n";
    std::cout << "  ω = " << oe.omega * 180.0 / M_PI << "°\n";
    std::cout << "  ν = " << oe.nu * 180.0 / M_PI << "°\n\n";

    // Round-trip
    auto [r2, v2] = elements::keplerian_to_cartesian(oe);
    std::cout << "Round-trip error:\n";
    std::cout << "  Position: " << (r_cart - r2).norm() << " m\n";
    std::cout << "  Velocity: " << (v_cart - v2).norm() * 1000.0 << " mm/s\n\n";

    // =========================================================================
    // 4. Transfer Mechanics
    // =========================================================================
    std::cout << "--- 4. Transfer Mechanics ---\n\n";

    double r_leo = constants::earth::R_eq + 300.0e3;

    auto [dv1, dv2] = transfer::hohmann_delta_v(r_leo, r_geo);
    double t_transfer = transfer::hohmann_transfer_time(r_leo, r_geo);

    std::cout << "Hohmann LEO (300 km) to GEO transfer:\n";
    std::cout << "  Δv₁ (periapsis burn): " << dv1 / 1000.0 << " km/s\n";
    std::cout << "  Δv₂ (apoapsis burn):  " << dv2 / 1000.0 << " km/s\n";
    std::cout << "  Total Δv:             " << (dv1 + dv2) / 1000.0
              << " km/s\n";
    std::cout << "  Transfer time:        " << t_transfer / 3600.0
              << " hours\n\n";

    // Plane change
    double delta_i = 28.5 * M_PI / 180.0; // Cape Canaveral to equatorial
    double dv_plane = transfer::plane_change_delta_v(
        quantities::circular_velocity(r_geo), delta_i);
    std::cout << "Plane change at GEO (28.5° inclination):\n";
    std::cout << "  Δv: " << dv_plane / 1000.0 << " km/s\n\n";

    // =========================================================================
    // 5. Sun and Moon Ephemeris
    // =========================================================================
    std::cout << "--- 5. Analytical Ephemeris ---\n\n";

    // J2000 epoch
    double jd_j2000 = 2451545.0; // 2000-01-01 12:00:00 TT

    Vec3<double> r_sun = ephemeris::analytical::sun_position_eci(jd_j2000);
    Vec3<double> r_moon = ephemeris::analytical::moon_position_eci(jd_j2000);

    double sun_dist_au = janus::norm(r_sun) / constants::sun::AU;
    double moon_dist_km = janus::norm(r_moon) / 1000.0;

    std::cout << "At J2000 epoch (2000-01-01 12:00 TT):\n\n";
    std::cout << "Sun position (ECI):\n";
    std::cout << "  r = [" << r_sun(0) / 1e9 << ", " << r_sun(1) / 1e9 << ", "
              << r_sun(2) / 1e9 << "] × 10⁹ m\n";
    std::cout << "  Distance: " << sun_dist_au << " AU\n\n";

    std::cout << "Moon position (ECI):\n";
    std::cout << "  r = [" << r_moon(0) / 1e6 << ", " << r_moon(1) / 1e6 << ", "
              << r_moon(2) / 1e6 << "] × 10⁶ m\n";
    std::cout << "  Distance: " << moon_dist_km << " km\n\n";

    // =========================================================================
    // 6. Symbolic Computation (for Optimization)
    // =========================================================================
    std::cout << "--- 6. Symbolic Computation ---\n\n";

    auto a_sym = janus::sym("a");
    auto e_sym = janus::sym("e");

    // Create symbolic period function
    auto T_sym = quantities::period(a_sym);
    janus::Function f_period("period", {a_sym}, {T_sym});

    // Evaluate
    auto T_result = f_period({7000.0e3});
    std::cout << "Symbolic period evaluation:\n";
    std::cout << "  period(7000 km) = " << T_result[0](0, 0) / 60.0
              << " minutes\n\n";

    // Kepler solver symbolically
    auto M_sym = janus::sym("M");
    auto E_solve = anomaly::mean_to_eccentric(M_sym, e_sym);
    janus::Function f_kepler("kepler", {M_sym, e_sym}, {E_solve});

    auto E_result = f_kepler({1.0, 0.5});
    std::cout << "Symbolic Kepler solver:\n";
    std::cout << "  mean_to_eccentric(M=1, e=0.5) = " << E_result[0](0, 0)
              << " rad\n\n";

    std::cout << "=== Demo Complete ===\n";

    return 0;
}
