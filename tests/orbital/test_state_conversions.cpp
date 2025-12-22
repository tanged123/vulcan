// State Conversion Tests
#include <gtest/gtest.h>
#include <vulcan/orbital/StateConversions.hpp>

using namespace vulcan::orbital;
using namespace vulcan::orbital::elements;

// Test round-trip conversion
TEST(StateConversions, RoundTrip_LEO) {
    vulcan::Vec3<double> r, v;
    r << 6678.0e3, 0.0, 0.0;
    v << 0.0, 7.73e3, 0.0;

    auto oe = cartesian_to_keplerian(r, v);
    auto [r2, v2] = keplerian_to_cartesian(oe);

    EXPECT_NEAR((r - r2).norm(), 0.0, 1.0);  // Within 1 m
    EXPECT_NEAR((v - v2).norm(), 0.0, 1e-6); // Within 1 mm/s
}

TEST(StateConversions, RoundTrip_Elliptical) {
    // Elliptical orbit with non-zero inclination
    vulcan::Vec3<double> r, v;
    r << 4453783.586, 5038203.756, 426384.456;
    v << -3829.428, -2943.567, -5611.621;

    auto oe = cartesian_to_keplerian(r, v);
    auto [r2, v2] = keplerian_to_cartesian(oe);

    EXPECT_NEAR((r - r2).norm(), 0.0, 10.0); // Within 10 m
    EXPECT_NEAR((v - v2).norm(), 0.0, 0.01); // Within 1 cm/s
}

// Test known orbit parameters
TEST(StateConversions, CircularEquatorial) {
    // Circular equatorial orbit
    double r_mag = 7000.0e3;
    double v_mag = std::sqrt(vulcan::constants::earth::mu / r_mag);

    vulcan::Vec3<double> r, v;
    r << r_mag, 0.0, 0.0;
    v << 0.0, v_mag, 0.0;

    auto oe = cartesian_to_keplerian(r, v);

    EXPECT_NEAR(oe.a, r_mag, 1.0);
    EXPECT_NEAR(oe.e, 0.0, 1e-6);
    EXPECT_NEAR(oe.i, 0.0, 1e-6);
}

TEST(StateConversions, PolarOrbit) {
    // Polar orbit (i = 90°)
    double r_mag = 7000.0e3;
    double v_mag = std::sqrt(vulcan::constants::earth::mu / r_mag);

    vulcan::Vec3<double> r, v;
    r << r_mag, 0.0, 0.0;
    v << 0.0, 0.0, v_mag; // Velocity in Z direction = polar

    auto oe = cartesian_to_keplerian(r, v);

    EXPECT_NEAR(oe.i, M_PI / 2.0, 1e-6); // 90 degrees
}

// Test elliptical orbit
TEST(StateConversions, Elliptical) {
    // Create orbit with known eccentricity
    double r_peri = 6678.0e3;
    double r_apo = 42164.0e3;
    double a = (r_peri + r_apo) / 2.0;
    double e = (r_apo - r_peri) / (r_apo + r_peri);

    // At periapsis
    double v_peri =
        std::sqrt(vulcan::constants::earth::mu * (2.0 / r_peri - 1.0 / a));

    vulcan::Vec3<double> r, v;
    r << r_peri, 0.0, 0.0;
    v << 0.0, v_peri, 0.0;

    auto oe = cartesian_to_keplerian(r, v);

    EXPECT_NEAR(oe.a, a, 100.0); // Within 100 m
    EXPECT_NEAR(oe.e, e, 1e-6);
    EXPECT_NEAR(oe.nu, 0.0, 1e-6); // At periapsis, true anomaly = 0
}

// Test symbolic compatibility
TEST(StateConversions, Symbolic_KeplerianToCartesian) {
    OrbitalElements<janus::SymbolicScalar> oe;
    oe.a = janus::sym("a");
    oe.e = janus::sym("e");
    oe.i = janus::sym("i");
    oe.Omega = janus::sym("Omega");
    oe.omega = janus::sym("omega");
    oe.nu = janus::sym("nu");

    auto [r, v] = keplerian_to_cartesian(oe);

    janus::Function f("kep2cart", {oe.a, oe.e, oe.i, oe.Omega, oe.omega, oe.nu},
                      {r(0), r(1), r(2)});

    auto result = f({7000.0e3, 0.1, 0.5, 1.0, 0.5, 0.0});
    EXPECT_NE(result[0](0, 0), 0.0);
}
