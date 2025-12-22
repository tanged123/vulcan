// Eclipse Detection Tests
// Verify cylindrical and conical shadow models

#include <cmath>
#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/environment/Eclipse.hpp>

using namespace vulcan;
using namespace vulcan::environment;

// =============================================================================
// Cylindrical Shadow Model Tests
// =============================================================================

// Satellite on sunlit side (between Earth and Sun)
TEST(Eclipse, SunlitSatellite) {
    Vec3<double> r_sat;
    r_sat << 7000e3, 0.0, 0.0; // 7000 km on +X axis

    Vec3<double> r_sun;
    r_sun << 1.5e11, 0.0, 0.0; // Sun in +X direction (1 AU)

    double nu = eclipse::shadow_cylindrical(r_sat, r_sun);
    EXPECT_NEAR(nu, 1.0, 1e-6); // Fully sunlit
}

// Satellite behind Earth
TEST(Eclipse, SatelliteInShadow) {
    Vec3<double> r_sat;
    r_sat << -7000e3, 0.0, 0.0; // Behind Earth (-X axis)

    Vec3<double> r_sun;
    r_sun << 1.5e11, 0.0, 0.0; // Sun in +X direction

    double nu = eclipse::shadow_cylindrical(r_sat, r_sun);
    EXPECT_NEAR(nu, 0.0, 1e-6); // In shadow
}

// Satellite outside shadow cylinder (behind but off-axis)
TEST(Eclipse, OutsideShadowCylinder) {
    Vec3<double> r_sat;
    // Behind Earth but far enough off-axis to miss the shadow
    r_sat << -7000e3, 7000e3, 0.0; // ~10000 km at 45 degrees

    Vec3<double> r_sun;
    r_sun << 1.5e11, 0.0, 0.0; // Sun in +X direction

    double nu = eclipse::shadow_cylindrical(r_sat, r_sun);
    EXPECT_NEAR(nu, 1.0, 1e-6); // Sunlit (outside cylinder)
}

// Satellite perpendicular to Sun direction
TEST(Eclipse, PerpendicularToSunDirection) {
    Vec3<double> r_sat;
    r_sat << 0.0, 7000e3, 0.0; // On Y-axis (perpendicular to Sun)

    Vec3<double> r_sun;
    r_sun << 1.5e11, 0.0, 0.0; // Sun in +X direction

    double nu = eclipse::shadow_cylindrical(r_sat, r_sun);
    EXPECT_NEAR(nu, 1.0, 1e-6); // Sunlit
}

// LEO satellite at GEO distance shadow
TEST(Eclipse, GEOShadow) {
    // GEO satellite in shadow
    double GEO_alt = 35786e3; // GEO altitude
    Vec3<double> r_sat;
    r_sat << -(constants::earth::R_eq + GEO_alt), 0.0, 0.0;

    Vec3<double> r_sun;
    r_sun << 1.5e11, 0.0, 0.0;

    double nu = eclipse::shadow_cylindrical(r_sat, r_sun);
    EXPECT_NEAR(nu, 0.0, 1e-6); // In shadow (cylindrical model)
}

// =============================================================================
// Binary Check Tests
// =============================================================================

TEST(Eclipse, IsShadowCheck) {
    Vec3<double> r_sunlit, r_shadow;
    r_sunlit << 7000e3, 0.0, 0.0;
    r_shadow << -7000e3, 0.0, 0.0;

    Vec3<double> r_sun;
    r_sun << 1.5e11, 0.0, 0.0;

    EXPECT_NEAR(eclipse::is_in_shadow(r_sunlit, r_sun), 0.0, 1e-6);
    EXPECT_NEAR(eclipse::is_in_shadow(r_shadow, r_sun), 1.0, 1e-6);

    EXPECT_NEAR(eclipse::is_sunlit(r_sunlit, r_sun), 1.0, 1e-6);
    EXPECT_NEAR(eclipse::is_sunlit(r_shadow, r_sun), 0.0, 1e-6);
}

// =============================================================================
// Conical Shadow Model Tests
// =============================================================================

TEST(Eclipse, ConicalSunlit) {
    Vec3<double> r_sat;
    r_sat << 7000e3, 0.0, 0.0;

    Vec3<double> r_sun;
    r_sun << 1.5e11, 0.0, 0.0;

    double nu = eclipse::shadow_conical(r_sat, r_sun);
    EXPECT_NEAR(nu, 1.0, 1e-6); // Fully sunlit
}

TEST(Eclipse, ConicalDeepShadow) {
    Vec3<double> r_sat;
    r_sat << -7000e3, 0.0, 0.0; // Directly behind Earth

    Vec3<double> r_sun;
    r_sun << 1.5e11, 0.0, 0.0;

    double nu = eclipse::shadow_conical(r_sat, r_sun);
    // Should be in umbra or deep penumbra
    EXPECT_LT(nu, 0.5);
}

// =============================================================================
// Symbolic Tests
// =============================================================================

TEST(Eclipse, SymbolicEvaluation) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r_sat;
    r_sat << x, y, z;

    // Use symbolic sun position
    Vec3<janus::SymbolicScalar> r_sun_sym;
    r_sun_sym << janus::SymbolicScalar(1.5e11), janus::SymbolicScalar(0.0),
        janus::SymbolicScalar(0.0);

    auto nu = eclipse::shadow_cylindrical(r_sat, r_sun_sym);

    // Create function for evaluation
    janus::Function f("shadow_test", {x, y, z}, {nu});

    // Evaluate at sunlit position
    auto result = f({7000e3, 0.0, 0.0});
    EXPECT_NEAR(result[0](0, 0), 1.0, 1e-6);

    // Evaluate at shadow position
    result = f({-7000e3, 0.0, 0.0});
    EXPECT_NEAR(result[0](0, 0), 0.0, 1e-6);
}
