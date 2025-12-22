// Orbital Quantities Tests
#include <gtest/gtest.h>
#include <vulcan/orbital/OrbitalQuantities.hpp>

using namespace vulcan::orbital::quantities;

// Test period for known orbits
TEST(OrbitalQuantities, Period_GEO) {
    // GEO orbit: a ≈ 42164 km, period ≈ 24 hours (86164 seconds sidereal)
    double a_geo = 42164.0e3; // meters
    double T = period(a_geo);

    EXPECT_NEAR(T, 86164.0, 100.0); // Within 100 seconds
}

TEST(OrbitalQuantities, Period_LEO) {
    // ISS-like orbit: a ≈ 6778 km, period ≈ 92 minutes
    double a_leo = 6778.0e3;
    double T = period(a_leo);

    EXPECT_NEAR(T / 60.0, 92.0, 1.0); // Within 1 minute
}

// Test circular velocity
TEST(OrbitalQuantities, CircularVelocity_LEO) {
    double r = vulcan::constants::earth::R_eq + 400.0e3;
    double v = circular_velocity(r);

    // LEO velocity ~ 7.7 km/s
    EXPECT_NEAR(v / 1000.0, 7.7, 0.1);
}

// Test escape velocity
TEST(OrbitalQuantities, EscapeVelocity_Surface) {
    double r = vulcan::constants::earth::R_eq;
    double v_esc = escape_velocity(r);

    // Earth surface escape velocity ~ 11.2 km/s
    EXPECT_NEAR(v_esc / 1000.0, 11.2, 0.1);
}

// Test relationship: v_esc = sqrt(2) * v_circ
TEST(OrbitalQuantities, EscapeCircularRelation) {
    double r = vulcan::constants::earth::R_eq + 500.0e3;
    double v_circ = circular_velocity(r);
    double v_esc = escape_velocity(r);

    EXPECT_NEAR(v_esc / v_circ, std::sqrt(2.0), 1e-10);
}

// Test vis-viva equation
TEST(OrbitalQuantities, VisViva_Periapsis) {
    double r_peri = 6678.0e3; // Periapsis
    double r_apo = 42164.0e3; // Apoapsis (GTO)
    double a = (r_peri + r_apo) / 2.0;

    double v_peri = velocity(r_peri, a);

    // GTO periapsis velocity ~ 10.2 km/s
    EXPECT_GT(v_peri / 1000.0, 10.0);
    EXPECT_LT(v_peri / 1000.0, 10.5);
}

// Test orbital energy
TEST(OrbitalQuantities, Energy_Negative) {
    double a = 7000.0e3;
    double E = energy(a);

    // Bound orbit has negative energy
    EXPECT_LT(E, 0.0);
}

// Test mean motion
TEST(OrbitalQuantities, MeanMotion) {
    double a = 7000.0e3;
    double n = mean_motion(a);
    double T = period(a);

    // n = 2*pi / T
    EXPECT_NEAR(n, 2.0 * M_PI / T, 1e-12);
}

// Test symbolic compatibility
TEST(OrbitalQuantities, SymbolicPeriod) {
    auto a = janus::sym("a");
    auto T = period(a);

    janus::Function f("period", {a}, {T});
    auto result = f({7000.0e3});

    EXPECT_GT(result[0](0, 0), 0.0);
}

TEST(OrbitalQuantities, SymbolicVelocity) {
    auto r = janus::sym("r");
    auto a = janus::sym("a");
    auto v = velocity(r, a);

    janus::Function f("velocity", {r, a}, {v});
    auto result = f({6678.0e3, 7000.0e3});

    EXPECT_GT(result[0](0, 0), 0.0);
}
