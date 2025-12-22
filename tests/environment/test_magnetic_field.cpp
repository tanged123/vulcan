// Magnetic Field Tests
// Verify dipole model accuracy and r^-3 decay

#include <cmath>
#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/environment/MagneticField.hpp>

using namespace vulcan;
using namespace vulcan::environment;

// =============================================================================
// Surface Field Tests
// =============================================================================

// Equatorial surface - minimum field strength
TEST(MagneticField, EquatorSurface) {
    Vec3<double> r_eq;
    r_eq << constants::earth::R_eq, 0.0, 0.0;

    double B = magnetic::field_magnitude(r_eq);

    // Equatorial surface field ~31.2 μT
    EXPECT_NEAR(B * 1e6, 31.2, 3.0); // μT with tolerance
}

// Polar surface - maximum field strength (2x equator for dipole)
TEST(MagneticField, PoleSurface) {
    Vec3<double> r_pole;
    r_pole << 0.0, 0.0, constants::earth::R_pol;

    double B = magnetic::field_magnitude(r_pole);

    // Polar surface field ~62 μT (2x equator)
    EXPECT_NEAR(B * 1e6, 62.0, 6.0); // μT with tolerance
}

// Field direction at equator should be horizontal (northward)
TEST(MagneticField, EquatorFieldDirection) {
    Vec3<double> r_eq;
    r_eq << constants::earth::R_eq, 0.0, 0.0;

    auto B = magnetic::dipole_field_ecef(r_eq);

    // At equator on X-axis, field should be in -Z direction (toward north pole)
    EXPECT_NEAR(B(0), 0.0, 1e-10);
    EXPECT_NEAR(B(1), 0.0, 1e-10);
    EXPECT_LT(B(2), 0.0); // Negative Z = toward north pole
}

// Field direction at north pole
// Note: Centered dipole with +Z moment produces +Z field at north pole
// Real Earth has -Z moment (field enters at north), but this tests the math
// model
TEST(MagneticField, NorthPoleFieldDirection) {
    Vec3<double> r_pole;
    r_pole << 0.0, 0.0, constants::earth::R_pol;

    auto B = magnetic::dipole_field_ecef(r_pole);

    // At north pole, field is along Z-axis (vertical)
    EXPECT_NEAR(B(0), 0.0, 1e-10);
    EXPECT_NEAR(B(1), 0.0, 1e-10);
    // For +Z dipole moment: B_z > 0 (field exits at north pole)
    EXPECT_GT(B(2), 0.0);
}

// =============================================================================
// Altitude Decay Tests
// =============================================================================

// Field should decrease as r^-3
TEST(MagneticField, FieldDecreaseWithAltitude) {
    Vec3<double> r_surface, r_leo;
    double R = constants::earth::R_eq;
    r_surface << R, 0.0, 0.0;
    r_leo << R + 400e3, 0.0, 0.0; // 400 km altitude

    double B_surface = magnetic::field_magnitude(r_surface);
    double B_leo = magnetic::field_magnitude(r_leo);

    double ratio = B_leo / B_surface;
    double expected_ratio = std::pow(R / (R + 400e3), 3);

    EXPECT_NEAR(ratio, expected_ratio, 0.01);
}

// GEO altitude check
TEST(MagneticField, GEOAltitude) {
    double GEO_alt = 35786e3; // GEO altitude
    Vec3<double> r_geo;
    r_geo << constants::earth::R_eq + GEO_alt, 0.0, 0.0;

    double B = magnetic::field_magnitude(r_geo);

    // GEO field should be much weaker (~100 nT range)
    EXPECT_LT(B * 1e9, 200.0); // Less than 200 nT
    EXPECT_GT(B * 1e9, 50.0);  // More than 50 nT
}

// =============================================================================
// NED Field Tests
// =============================================================================

TEST(MagneticField, NEDAtEquator) {
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;

    auto B_ned = magnetic::field_ned(lat, lon, alt);

    // At equator, field should be primarily North
    EXPECT_GT(std::abs(B_ned(0)), 0.0); // Non-zero North component
    EXPECT_NEAR(B_ned(1), 0.0, 1e-10);  // No East component
    EXPECT_NEAR(B_ned(2), 0.0, 1e-6);   // Small Down component at equator
}

TEST(MagneticField, NEDAtPole) {
    double lat = 90.0 * constants::angle::deg2rad; // North pole
    double lon = 0.0;
    double alt = 0.0;

    auto B_ned = magnetic::field_ned(lat, lon, alt);

    // At pole, field should be entirely vertical (Down component)
    EXPECT_NEAR(B_ned(0), 0.0, 1e-6);  // No North
    EXPECT_NEAR(B_ned(1), 0.0, 1e-10); // No East
    // For centered dipole model, Down component is positive at north pole
    EXPECT_GT(B_ned(2), 0.0);
}

// =============================================================================
// Surface Intensity Formula
// =============================================================================

TEST(MagneticField, SurfaceIntensityFormula) {
    // Test the analytic formula: |B| = B0 * sqrt(1 + 3sin²(lat))

    double lat_equator = 0.0;
    double lat_45 = 45.0 * constants::angle::deg2rad;
    double lat_pole = 90.0 * constants::angle::deg2rad;

    double B_eq = magnetic::surface_intensity(lat_equator);
    double B_45 = magnetic::surface_intensity(lat_45);
    double B_pole = magnetic::surface_intensity(lat_pole);

    // Equator: sqrt(1 + 0) = 1
    EXPECT_NEAR(B_eq, magnetic::constants::B0, 1e-10);

    // 45°: sqrt(1 + 3*0.5) = sqrt(2.5) ≈ 1.58
    EXPECT_NEAR(B_45 / magnetic::constants::B0, std::sqrt(2.5), 1e-6);

    // Pole: sqrt(1 + 3) = 2
    EXPECT_NEAR(B_pole / magnetic::constants::B0, 2.0, 1e-6);
}

// =============================================================================
// Inclination Tests
// =============================================================================

TEST(MagneticField, InclinationAtEquator) {
    double lat = 0.0;
    double inc = magnetic::inclination(lat);

    // At equator, inclination should be zero (horizontal field)
    EXPECT_NEAR(inc, 0.0, 1e-10);
}

TEST(MagneticField, InclinationAtPole) {
    double lat = 90.0 * constants::angle::deg2rad;
    double inc = magnetic::inclination(lat);

    // At pole, inclination should be 90° (vertical field)
    EXPECT_NEAR(inc, 90.0 * constants::angle::deg2rad, 0.01);
}

TEST(MagneticField, InclinationAt45Degrees) {
    double lat = 45.0 * constants::angle::deg2rad;
    double inc = magnetic::inclination(lat);

    // tan(I) = 2*tan(45°) = 2, so I = atan(2) ≈ 63.43°
    double expected = std::atan(2.0);
    EXPECT_NEAR(inc, expected, 1e-6);
}

// =============================================================================
// Symbolic Tests
// =============================================================================

TEST(MagneticField, SymbolicEvaluation) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r;
    r << x, y, z;

    auto B = magnetic::dipole_field_ecef(r);

    // Create function for evaluation
    janus::Function f("mag_field", {x, y, z}, {B(0), B(1), B(2)});

    // Evaluate at equator
    double R = constants::earth::R_eq;
    auto result = f({R, 0.0, 0.0});

    EXPECT_NEAR(result[0](0, 0), 0.0, 1e-10);
    EXPECT_NEAR(result[1](0, 0), 0.0, 1e-10);
    EXPECT_LT(result[2](0, 0), 0.0);
}

TEST(MagneticField, SymbolicGradient) {
    auto x = janus::sym("x");

    Vec3<janus::SymbolicScalar> r;
    r << x, janus::SymbolicScalar(0.0), janus::SymbolicScalar(0.0);

    auto B_mag = magnetic::field_magnitude(r);

    // Compute derivative w.r.t. x (radial direction at equator)
    auto dB_dx = janus::jacobian(B_mag, x);

    // Create function for evaluation
    janus::Function f("mag_grad", {x}, {dB_dx});

    // Evaluate at surface
    double R = constants::earth::R_eq;
    auto result = f({R});
    double grad = result[0](0, 0);

    // Field should decrease with distance (negative gradient)
    EXPECT_LT(grad, 0.0);
}
