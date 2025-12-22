// Environment Utilities Demo
// Demonstrates solar position, eclipse detection, and magnetic field models
//
// This example shows both numeric and symbolic usage patterns for
// spacecraft environment analysis.

#include <iomanip>
#include <iostream>
#include <vulcan/vulcan.hpp>

using namespace vulcan;
using namespace vulcan::environment;

// =============================================================================
// Helper: Print separator
// =============================================================================
void section(const std::string &title) {
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << title << "\n";
    std::cout << std::string(60, '=') << "\n";
}

// =============================================================================
// Demo 1: Solar Position
// =============================================================================
void demo_solar_position() {
    section("1. Solar Position Ephemeris");

    // Current date (example: 2024 summer solstice)
    double jd = time::calendar_to_jd(2024, 6, 21, 12, 0, 0.0);

    std::cout << "Date: 2024-06-21 12:00:00 UTC (Summer Solstice)\n";
    std::cout << "Julian Date: " << std::fixed << std::setprecision(6) << jd
              << "\n\n";

    // Get Sun RA/Dec
    auto [ra, dec] = solar::ra_dec(jd);
    std::cout << "Sun Position:\n";
    std::cout << "  Right Ascension: " << std::setprecision(2)
              << ra * constants::angle::rad2deg << "°\n";
    std::cout << "  Declination:     " << dec * constants::angle::rad2deg
              << "° (max ~23.44° at solstice)\n";

    // Distance
    double dist = solar::distance(jd);
    std::cout << "  Distance:        " << std::setprecision(3) << dist / 1e9
              << " million km\n";

    // ECI position
    auto pos = solar::position_eci(jd);
    std::cout << "  ECI Position:    [" << std::setprecision(3) << pos(0) / 1e9
              << ", " << pos(1) / 1e9 << ", " << pos(2) / 1e9 << "] × 10⁹ m\n";

    // Show variation over seasons
    std::cout << "\nSeasonal Declination Variation:\n";
    const char *seasons[] = {"Winter Solstice", "Vernal Equinox",
                             "Summer Solstice", "Autumnal Equinox"};
    int months[] = {12, 3, 6, 9};
    int days[] = {21, 20, 21, 22};
    for (int i = 0; i < 4; i++) {
        double jd_season = time::calendar_to_jd(2024, months[i], days[i], 12);
        auto [ra_s, dec_s] = solar::ra_dec(jd_season);
        std::cout << "  " << std::setw(18) << seasons[i] << ": " << std::setw(7)
                  << std::setprecision(2) << dec_s * constants::angle::rad2deg
                  << "°\n";
    }
}

// =============================================================================
// Demo 2: Eclipse Detection
// =============================================================================
void demo_eclipse() {
    section("2. Eclipse Detection");

    // Sun position (simplified as +X direction at 1 AU)
    Vec3<double> r_sun;
    r_sun << 1.5e11, 0.0, 0.0;

    std::cout << "Sun direction: +X axis (1 AU)\n\n";

    // Test various satellite positions
    struct TestCase {
        std::string name;
        double x, y, z;
    };

    TestCase cases[] = {
        {"LEO sunlit (dawn side)", 0.0, 7000e3, 0.0},
        {"LEO sunlit (subsolar)", 7000e3, 0.0, 0.0},
        {"LEO in shadow", -7000e3, 0.0, 0.0},
        {"LEO edge of shadow", -7000e3, 6400e3, 0.0},
        {"GEO in shadow", -(constants::earth::R_eq + 35786e3), 0.0, 0.0},
        {"GEO sunlit", 0.0, constants::earth::R_eq + 35786e3, 0.0},
    };

    std::cout << std::setw(25) << "Position" << std::setw(15) << "Cylindrical"
              << std::setw(15) << "Conical"
              << "\n";
    std::cout << std::string(55, '-') << "\n";

    for (const auto &tc : cases) {
        Vec3<double> r_sat;
        r_sat << tc.x, tc.y, tc.z;

        double nu_cyl = eclipse::shadow_cylindrical(r_sat, r_sun);
        double nu_con = eclipse::shadow_conical(r_sat, r_sun);

        std::cout << std::setw(25) << tc.name << std::setw(15)
                  << std::setprecision(3) << nu_cyl << std::setw(15) << nu_con
                  << "\n";
    }

    std::cout << "\nNote: 1.0 = fully sunlit, 0.0 = full shadow\n";
}

// =============================================================================
// Demo 3: Magnetic Field
// =============================================================================
void demo_magnetic_field() {
    section("3. Earth's Magnetic Field (Dipole Model)");

    std::cout << "Reference field at equator: " << magnetic::constants::B0 * 1e6
              << " μT\n\n";

    // Field at various locations
    std::cout << "Surface Field Magnitude:\n";

    double lats[] = {0, 30, 45, 60, 90};
    for (double lat_deg : lats) {
        double lat = lat_deg * constants::angle::deg2rad;
        double B = magnetic::surface_intensity(lat);
        double inc = magnetic::inclination(lat);

        std::cout << "  Latitude " << std::setw(3) << static_cast<int>(lat_deg)
                  << "°: " << std::setw(6) << std::setprecision(2) << B * 1e6
                  << " μT, inclination " << std::setw(6)
                  << inc * constants::angle::rad2deg << "°\n";
    }

    // Field decay with altitude
    std::cout << "\nField Decay with Altitude (Equator):\n";
    double alts_km[] = {0, 400, 2000, 20000, 35786};
    for (double alt_km : alts_km) {
        Vec3<double> r;
        r << constants::earth::R_eq + alt_km * 1e3, 0.0, 0.0;
        double B = magnetic::field_magnitude(r);

        std::cout << "  " << std::setw(6) << static_cast<int>(alt_km)
                  << " km: " << std::setw(8) << std::setprecision(3);
        if (B > 1e-6) {
            std::cout << B * 1e6 << " μT\n";
        } else {
            std::cout << B * 1e9 << " nT\n";
        }
    }
}

// =============================================================================
// Demo 4: Symbolic Mode (Optimization Example)
// =============================================================================
void demo_symbolic() {
    section("4. Symbolic Mode: Optimal Eclipse Avoidance");

    std::cout << "Using Janus symbolic mode for autodiff...\n\n";

    // Create symbolic satellite position
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r_sat;
    r_sat << x, y, z;

    // Fixed sun position
    Vec3<janus::SymbolicScalar> r_sun;
    r_sun << janus::SymbolicScalar(1.5e11), janus::SymbolicScalar(0.0),
        janus::SymbolicScalar(0.0);

    // Shadow function (want to maximize for optimal illumination)
    auto nu = eclipse::shadow_cylindrical(r_sat, r_sun);

    // Create a CasADi function for the shadow model
    janus::Function shadow_fn("shadow", {x, y, z}, {nu});

    // Evaluate gradient at a point near the shadow boundary
    std::cout << "Shadow function at test point (-7000km, 5000km, 0):\n";
    auto result = shadow_fn({-7000e3, 5000e3, 0.0});
    std::cout << "  ν = " << result[0](0, 0) << "\n";

    // Show that we can compute gradients for optimization
    auto grad = janus::jacobian(nu, x);
    janus::Function grad_fn("grad_shadow", {x, y, z}, {grad});
    auto grad_result = grad_fn({-7000e3, 5000e3, 0.0});
    std::cout << "  ∂ν/∂x = " << grad_result[0](0, 0)
              << " (use for gradient-based optimization)\n";
}

// =============================================================================
// Main
// =============================================================================
int main() {
    std::cout
        << "╔══════════════════════════════════════════════════════════╗\n";
    std::cout
        << "║       Vulcan Environment Utilities Demonstration         ║\n";
    std::cout
        << "╚══════════════════════════════════════════════════════════╝\n";

    demo_solar_position();
    demo_eclipse();
    demo_magnetic_field();
    demo_symbolic();

    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "Demo complete!\n";

    return 0;
}
