// Test suite for mass properties
#include <vulcan/mass/MassProperties.hpp>

#include <gtest/gtest.h>

namespace vulcan::mass {

// =============================================================================
// Factory Constructor Tests
// =============================================================================

TEST(MassPropertiesTest, PointMass) {
    auto mp =
        MassProperties<double>::point_mass(10.0, Vec3<double>{1.0, 2.0, 3.0});

    EXPECT_DOUBLE_EQ(mp.mass, 10.0);
    EXPECT_DOUBLE_EQ(mp.cg(0), 1.0);
    EXPECT_DOUBLE_EQ(mp.cg(1), 2.0);
    EXPECT_DOUBLE_EQ(mp.cg(2), 3.0);

    // Point mass has zero inertia about its own CG
    EXPECT_DOUBLE_EQ(mp.inertia.norm(), 0.0);
}

TEST(MassPropertiesTest, DiagonalWithCg) {
    auto mp = MassProperties<double>::diagonal(
        100.0, Vec3<double>{1.0, 0.0, 0.0}, 10.0, 20.0, 30.0);

    EXPECT_DOUBLE_EQ(mp.mass, 100.0);
    EXPECT_DOUBLE_EQ(mp.cg(0), 1.0);
    EXPECT_DOUBLE_EQ(mp.inertia(0, 0), 10.0);
    EXPECT_DOUBLE_EQ(mp.inertia(1, 1), 20.0);
    EXPECT_DOUBLE_EQ(mp.inertia(2, 2), 30.0);
    EXPECT_DOUBLE_EQ(mp.inertia(0, 1), 0.0);
}

TEST(MassPropertiesTest, SolidSphere) {
    // I = (2/5) * m * r² for solid sphere
    auto mp = MassProperties<double>::solid_sphere(100.0, 2.0);

    double expected_I = 0.4 * 100.0 * 4.0; // 160 kg·m²
    EXPECT_NEAR(mp.inertia(0, 0), expected_I, 1e-12);
    EXPECT_NEAR(mp.inertia(1, 1), expected_I, 1e-12);
    EXPECT_NEAR(mp.inertia(2, 2), expected_I, 1e-12);
}

TEST(MassPropertiesTest, SolidCylinder) {
    // Cylinder axis along Z
    // Ixx = Iyy = (1/12) * m * (3*r² + L²)
    // Izz = (1/2) * m * r²
    double m = 100.0, r = 1.0, L = 2.0;
    auto mp = MassProperties<double>::solid_cylinder(m, r, L);

    double Ixx_expected = (1.0 / 12.0) * m * (3.0 * r * r + L * L);
    double Izz_expected = 0.5 * m * r * r;

    EXPECT_NEAR(mp.inertia(0, 0), Ixx_expected, 1e-12);
    EXPECT_NEAR(mp.inertia(1, 1), Ixx_expected, 1e-12);
    EXPECT_NEAR(mp.inertia(2, 2), Izz_expected, 1e-12);
}

TEST(MassPropertiesTest, SolidBox) {
    // Ixx = (1/12) * m * (dy² + dz²)
    double m = 120.0, dx = 1.0, dy = 2.0, dz = 3.0;
    auto mp = MassProperties<double>::solid_box(m, dx, dy, dz);

    double Ixx = (1.0 / 12.0) * m * (dy * dy + dz * dz);
    double Iyy = (1.0 / 12.0) * m * (dx * dx + dz * dz);
    double Izz = (1.0 / 12.0) * m * (dx * dx + dy * dy);

    EXPECT_NEAR(mp.inertia(0, 0), Ixx, 1e-12);
    EXPECT_NEAR(mp.inertia(1, 1), Iyy, 1e-12);
    EXPECT_NEAR(mp.inertia(2, 2), Izz, 1e-12);
}

// =============================================================================
// Backward Compatibility Tests
// =============================================================================

TEST(MassPropertiesTest, FromMassBackwardCompat) {
    auto mp = MassProperties<double>::from_mass(100.0);

    EXPECT_DOUBLE_EQ(mp.mass, 100.0);
    EXPECT_DOUBLE_EQ(mp.cg.norm(), 0.0);
    // Legacy behavior: inertia = m*I for point mass at origin
    EXPECT_DOUBLE_EQ(mp.inertia(0, 0), 100.0);
}

TEST(MassPropertiesTest, DiagonalBackwardCompat4Arg) {
    // Old 4-arg signature: diagonal(m, Ixx, Iyy, Izz) places at origin
    auto mp = MassProperties<double>::diagonal(100.0, 10.0, 20.0, 30.0);

    EXPECT_DOUBLE_EQ(mp.mass, 100.0);
    EXPECT_DOUBLE_EQ(mp.cg.norm(), 0.0);
    EXPECT_DOUBLE_EQ(mp.inertia(0, 0), 10.0);
    EXPECT_DOUBLE_EQ(mp.inertia(1, 1), 20.0);
    EXPECT_DOUBLE_EQ(mp.inertia(2, 2), 30.0);
}

TEST(MassPropertiesTest, FullBackwardCompat) {
    auto mp =
        MassProperties<double>::full(100.0, 10.0, 20.0, 30.0, 1.0, 2.0, 3.0);

    EXPECT_DOUBLE_EQ(mp.mass, 100.0);
    EXPECT_DOUBLE_EQ(mp.inertia(0, 0), 10.0);
    EXPECT_DOUBLE_EQ(mp.inertia(1, 1), 20.0);
    EXPECT_DOUBLE_EQ(mp.inertia(2, 2), 30.0);
    // Products negated
    EXPECT_DOUBLE_EQ(mp.inertia(0, 1), -1.0);
    EXPECT_DOUBLE_EQ(mp.inertia(0, 2), -2.0);
    EXPECT_DOUBLE_EQ(mp.inertia(1, 2), -3.0);
}

// =============================================================================
// Parallel Axis Theorem Tests
// =============================================================================

TEST(MassPropertiesTest, InertiaAboutPointPointMass) {
    // Point mass at origin, query inertia about (1, 0, 0)
    auto mp = MassProperties<double>::point_mass(1.0, Vec3<double>::Zero());

    auto I_about_pt = mp.inertia_about_point(Vec3<double>{1.0, 0.0, 0.0});

    // r = (0,0,0) - (1,0,0) = (-1,0,0)
    // r·r = 1
    // J = 0 + m*(|r|²I - r⊗r)
    // Jxx = 1*(1 - 1) = 0
    // Jyy = 1*(1 - 0) = 1
    // Jzz = 1*(1 - 0) = 1
    EXPECT_NEAR(I_about_pt(0, 0), 0.0, 1e-12);
    EXPECT_NEAR(I_about_pt(1, 1), 1.0, 1e-12);
    EXPECT_NEAR(I_about_pt(2, 2), 1.0, 1e-12);
}

TEST(MassPropertiesTest, InertiaAboutPointDiagonalBody) {
    // Sphere at (2, 0, 0), query about origin
    auto mp = MassProperties<double>::solid_sphere(10.0, 1.0,
                                                   Vec3<double>{2.0, 0.0, 0.0});

    double I_sphere = 0.4 * 10.0 * 1.0; // 4 kg·m²
    auto I_about_origin = mp.inertia_about_point(Vec3<double>::Zero());

    // r = (2,0,0) - (0,0,0) = (2,0,0), |r|² = 4
    // Jxx = 4 + 10*(4 - 4) = 4
    // Jyy = 4 + 10*(4 - 0) = 44
    // Jzz = 4 + 10*(4 - 0) = 44
    EXPECT_NEAR(I_about_origin(0, 0), I_sphere, 1e-12);
    EXPECT_NEAR(I_about_origin(1, 1), I_sphere + 10.0 * 4.0, 1e-12);
    EXPECT_NEAR(I_about_origin(2, 2), I_sphere + 10.0 * 4.0, 1e-12);
}

// =============================================================================
// Aggregation Tests
// =============================================================================

TEST(MassPropertiesTest, AddTwoPointMasses) {
    auto mp1 =
        MassProperties<double>::point_mass(1.0, Vec3<double>{0.0, 0.0, 0.0});
    auto mp2 =
        MassProperties<double>::point_mass(1.0, Vec3<double>{2.0, 0.0, 0.0});

    auto combined = mp1 + mp2;

    EXPECT_DOUBLE_EQ(combined.mass, 2.0);
    EXPECT_DOUBLE_EQ(combined.cg(0), 1.0); // midpoint
    EXPECT_DOUBLE_EQ(combined.cg(1), 0.0);
    EXPECT_DOUBLE_EQ(combined.cg(2), 0.0);

    // Each mass is 1m from combined CG
    // Iyy = Izz = 1*1² + 1*1² = 2
    EXPECT_NEAR(combined.inertia(0, 0), 0.0, 1e-12); // About x-axis
    EXPECT_NEAR(combined.inertia(1, 1), 2.0, 1e-12);
    EXPECT_NEAR(combined.inertia(2, 2), 2.0, 1e-12);
}

TEST(MassPropertiesTest, AddAsymmetricMasses) {
    auto mp1 =
        MassProperties<double>::point_mass(2.0, Vec3<double>{0.0, 0.0, 0.0});
    auto mp2 =
        MassProperties<double>::point_mass(1.0, Vec3<double>{3.0, 0.0, 0.0});

    auto combined = mp1 + mp2;

    EXPECT_DOUBLE_EQ(combined.mass, 3.0);
    EXPECT_DOUBLE_EQ(combined.cg(0), 1.0); // (2*0 + 1*3)/3 = 1

    // mp1 at x=0, mp2 at x=3, combined CG at x=1
    // mp1 is 1m from CG: contributes 2*1² = 2 to Iyy, Izz
    // mp2 is 2m from CG: contributes 1*4 = 4 to Iyy, Izz
    // Total Iyy = Izz = 6
    EXPECT_NEAR(combined.inertia(1, 1), 6.0, 1e-12);
    EXPECT_NEAR(combined.inertia(2, 2), 6.0, 1e-12);
}

TEST(MassPropertiesTest, SubtractMass) {
    auto solid =
        MassProperties<double>::point_mass(10.0, Vec3<double>{0.0, 0.0, 0.0});
    auto hole =
        MassProperties<double>::point_mass(2.0, Vec3<double>{1.0, 0.0, 0.0});

    auto result = solid - hole;

    EXPECT_DOUBLE_EQ(result.mass, 8.0);
    // CG: (10*0 - 2*1)/8 = -0.25
    EXPECT_NEAR(result.cg(0), -0.25, 1e-12);
}

TEST(MassPropertiesTest, ScaleMass) {
    auto mp = MassProperties<double>::solid_sphere(100.0, 2.0);

    auto scaled = mp * 0.5;
    EXPECT_DOUBLE_EQ(scaled.mass, 50.0);
    EXPECT_NEAR(scaled.inertia(0, 0), 0.5 * mp.inertia(0, 0), 1e-12);

    auto scaled2 = 2.0 * mp;
    EXPECT_DOUBLE_EQ(scaled2.mass, 200.0);
}

TEST(MassPropertiesTest, AggregateFreeFunction) {
    std::vector<MassProperties<double>> components = {
        MassProperties<double>::point_mass(1.0, Vec3<double>{0.0, 0.0, 0.0}),
        MassProperties<double>::point_mass(1.0, Vec3<double>{2.0, 0.0, 0.0}),
        MassProperties<double>::point_mass(1.0, Vec3<double>{0.0, 2.0, 0.0})};

    auto combined = aggregate_mass_properties(components);

    EXPECT_DOUBLE_EQ(combined.mass, 3.0);
    EXPECT_NEAR(combined.cg(0), 2.0 / 3.0, 1e-12);
    EXPECT_NEAR(combined.cg(1), 2.0 / 3.0, 1e-12);
}

TEST(MassPropertiesTest, InPlaceAddition) {
    auto mp1 =
        MassProperties<double>::point_mass(1.0, Vec3<double>{0.0, 0.0, 0.0});
    auto mp2 =
        MassProperties<double>::point_mass(1.0, Vec3<double>{2.0, 0.0, 0.0});

    mp1 += mp2;

    EXPECT_DOUBLE_EQ(mp1.mass, 2.0);
    EXPECT_DOUBLE_EQ(mp1.cg(0), 1.0);
}

// =============================================================================
// Physical Validation Tests
// =============================================================================

TEST(MassPropertiesTest, IsPhysicallyValid) {
    auto valid = MassProperties<double>::solid_sphere(100.0, 2.0);
    EXPECT_TRUE(is_physically_valid(valid));

    // Negative mass
    auto invalid_mass =
        MassProperties<double>::point_mass(-1.0, Vec3<double>::Zero());
    EXPECT_FALSE(is_physically_valid(invalid_mass));
}

TEST(MassPropertiesTest, IsPointMass) {
    auto pm = MassProperties<double>::point_mass(10.0, Vec3<double>::Zero());
    EXPECT_TRUE(is_point_mass(pm));

    auto sphere = MassProperties<double>::solid_sphere(10.0, 1.0);
    EXPECT_FALSE(is_point_mass(sphere));
}

TEST(MassPropertiesTest, PrincipalMomentsSymmetric) {
    auto sphere = MassProperties<double>::solid_sphere(100.0, 2.0);
    auto moments = principal_moments(sphere);

    // Sphere has equal principal moments
    double expected = 0.4 * 100.0 * 4.0;
    EXPECT_NEAR(moments(0), expected, 1e-10);
    EXPECT_NEAR(moments(1), expected, 1e-10);
    EXPECT_NEAR(moments(2), expected, 1e-10);
}

// =============================================================================
// Symbolic Compatibility Tests
// =============================================================================

TEST(MassPropertiesSymbolic, PointMassInstantiation) {
    using MX = casadi::MX;

    auto m = janus::sym("m");
    auto x = janus::sym("x");

    Vec3<MX> pos{x, MX(0), MX(0)};
    auto mp = MassProperties<MX>::point_mass(m, pos);

    EXPECT_FALSE(mp.mass.is_empty());
    EXPECT_FALSE(mp.cg(0).is_empty());
}

TEST(MassPropertiesSymbolic, AggregationFunction) {
    using MX = casadi::MX;

    auto m1 = janus::sym("m1");
    auto m2 = janus::sym("m2");
    auto x1 = janus::sym("x1");
    auto x2 = janus::sym("x2");

    auto mp1 = MassProperties<MX>::point_mass(m1, Vec3<MX>{x1, MX(0), MX(0)});
    auto mp2 = MassProperties<MX>::point_mass(m2, Vec3<MX>{x2, MX(0), MX(0)});

    auto combined = mp1 + mp2;

    // Build a function
    janus::Function f("aggregate", {m1, m2, x1, x2},
                      {combined.mass, combined.cg(0)});

    auto result = f({1.0, 1.0, 0.0, 2.0});
    EXPECT_NEAR(result[0](0, 0), 2.0, 1e-12); // mass
    EXPECT_NEAR(result[1](0, 0), 1.0, 1e-12); // cg_x = midpoint
}

TEST(MassPropertiesSymbolic, InertiaAboutPointFunction) {
    using MX = casadi::MX;

    auto m = janus::sym("m");
    auto r = janus::sym("r");

    auto mp = MassProperties<MX>::point_mass(m, Vec3<MX>{r, MX(0), MX(0)});

    auto I_about_origin = mp.inertia_about_point(Vec3<MX>::Zero());

    janus::Function f("inertia", {m, r}, {I_about_origin(1, 1)});

    // m=10, r=2: Iyy = 10 * 2² = 40
    auto result = f({10.0, 2.0});
    EXPECT_NEAR(result[0](0, 0), 40.0, 1e-12);
}

} // namespace vulcan::mass
