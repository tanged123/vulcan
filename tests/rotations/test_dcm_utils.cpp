#include <gtest/gtest.h>
#include <vulcan/rotations/Rotations.hpp>

#include <janus/janus.hpp>

#include <cmath>
#include <numbers>

// =============================================================================
// Skew Matrix Tests
// =============================================================================

TEST(DCMUtils, SkewMatrix) {
    vulcan::Vec3<double> v;
    v << 1.0, 2.0, 3.0;

    auto S = vulcan::skew(v);

    // Verify skew-symmetric: S^T = -S
    EXPECT_NEAR(S(0, 1), -S(1, 0), 1e-15);
    EXPECT_NEAR(S(0, 2), -S(2, 0), 1e-15);
    EXPECT_NEAR(S(1, 2), -S(2, 1), 1e-15);

    // Verify diagonal is zero
    EXPECT_NEAR(S(0, 0), 0.0, 1e-15);
    EXPECT_NEAR(S(1, 1), 0.0, 1e-15);
    EXPECT_NEAR(S(2, 2), 0.0, 1e-15);

    // Verify structure
    EXPECT_NEAR(S(0, 1), -v(2), 1e-15);
    EXPECT_NEAR(S(0, 2), v(1), 1e-15);
    EXPECT_NEAR(S(1, 2), -v(0), 1e-15);
}

TEST(DCMUtils, SkewCrossProduct) {
    // Verify [v]× * u = v × u
    vulcan::Vec3<double> v, u;
    v << 1.0, 2.0, 3.0;
    u << 4.0, 5.0, 6.0;

    auto S = vulcan::skew(v);
    vulcan::Vec3<double> cross_via_skew = S * u;
    vulcan::Vec3<double> cross_direct = janus::cross(v, u);

    EXPECT_NEAR(cross_via_skew(0), cross_direct(0), 1e-15);
    EXPECT_NEAR(cross_via_skew(1), cross_direct(1), 1e-15);
    EXPECT_NEAR(cross_via_skew(2), cross_direct(2), 1e-15);
}

TEST(DCMUtils, UnskewRoundtrip) {
    vulcan::Vec3<double> v;
    v << 1.5, -2.5, 3.5;

    auto S = vulcan::skew(v);
    auto v_back = vulcan::unskew(S);

    EXPECT_NEAR(v_back(0), v(0), 1e-15);
    EXPECT_NEAR(v_back(1), v(1), 1e-15);
    EXPECT_NEAR(v_back(2), v(2), 1e-15);
}

// =============================================================================
// DCM Composition Tests
// =============================================================================

TEST(DCMUtils, ComposeDCM) {
    // Two 45° rotations about Z should give 90°
    double theta = std::numbers::pi / 4.0;
    auto R1 = janus::rotation_matrix_3d(theta, 2);
    auto R2 = janus::rotation_matrix_3d(theta, 2);

    auto R_composed = vulcan::compose_dcm(R1, R2);
    auto R_90 = janus::rotation_matrix_3d(std::numbers::pi / 2.0, 2);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(R_composed(i, j), R_90(i, j), 1e-12);
        }
    }
}

TEST(DCMUtils, RelativeDCM) {
    // If R_A and R_B are same, relative should be identity
    auto R = janus::rotation_matrix_3d(0.5, 1);
    auto R_rel = vulcan::relative_dcm(R, R);

    EXPECT_NEAR(R_rel(0, 0), 1.0, 1e-12);
    EXPECT_NEAR(R_rel(1, 1), 1.0, 1e-12);
    EXPECT_NEAR(R_rel(2, 2), 1.0, 1e-12);
    EXPECT_NEAR(R_rel(0, 1), 0.0, 1e-12);
}

TEST(DCMUtils, RelativeDCM_Inverse) {
    // R_rel(A, B) * R_rel(B, A) = I
    auto R_A = janus::rotation_matrix_3d(0.3, 0);
    auto R_B = janus::rotation_matrix_3d(0.5, 1);

    auto R_AB = vulcan::relative_dcm(R_A, R_B);
    auto R_BA = vulcan::relative_dcm(R_B, R_A);

    auto product = R_AB * R_BA;

    EXPECT_NEAR(product(0, 0), 1.0, 1e-12);
    EXPECT_NEAR(product(1, 1), 1.0, 1e-12);
    EXPECT_NEAR(product(2, 2), 1.0, 1e-12);
}

// =============================================================================
// Small-Angle Approximation Tests
// =============================================================================

TEST(DCMUtils, SmallAngleDCM) {
    // For very small angles, should be close to exact
    vulcan::Vec3<double> theta;
    theta << 0.001, 0.002, 0.003;

    auto R_approx = vulcan::dcm_from_small_angle(theta);

    // Compare with exact Rodrigues
    auto R_exact = vulcan::dcm_from_rotation_vector(theta);

    // Should be within ~0.1% for 1 mrad angles
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(R_approx(i, j), R_exact(i, j), 1e-5);
        }
    }
}

TEST(DCMUtils, SmallAngleRoundtrip) {
    vulcan::Vec3<double> theta;
    theta << 0.01, -0.02, 0.015;

    auto R = vulcan::dcm_from_small_angle(theta);
    auto theta_back = vulcan::small_angle_from_dcm(R);

    // Should be exact for first-order approximation
    EXPECT_NEAR(theta_back(0), theta(0), 1e-6);
    EXPECT_NEAR(theta_back(1), theta(1), 1e-6);
    EXPECT_NEAR(theta_back(2), theta(2), 1e-6);
}

// =============================================================================
// DCM Validation Tests
// =============================================================================

TEST(DCMUtils, IsValidDCM) {
    auto R = janus::rotation_matrix_3d(0.5, 1);
    EXPECT_TRUE(vulcan::is_valid_dcm(R));

    // Non-orthonormal matrix
    vulcan::Mat3<double> bad = vulcan::Mat3<double>::Identity();
    bad(0, 0) = 2.0;
    EXPECT_FALSE(vulcan::is_valid_dcm(bad));
}

// =============================================================================
// Symbolic Tests
// =============================================================================

TEST(DCMUtils, SymbolicSkew) {
    using Scalar = janus::SymbolicScalar;

    Scalar x = janus::sym("x");
    Scalar y = janus::sym("y");
    Scalar z = janus::sym("z");

    vulcan::Vec3<Scalar> v;
    v << x, y, z;

    auto S = vulcan::skew(v);

    // Verify symbolic
    EXPECT_FALSE(S(0, 1).is_constant());

    // Create function
    janus::Function f("skew", {x, y, z}, {S(0, 1), S(0, 2), S(1, 2)});

    auto result = f({1.0, 2.0, 3.0});

    EXPECT_NEAR(result[0](0, 0), -3.0, 1e-12); // S[0,1] = -z
    EXPECT_NEAR(result[1](0, 0), 2.0, 1e-12);  // S[0,2] = y
    EXPECT_NEAR(result[2](0, 0), -1.0, 1e-12); // S[1,2] = -x
}
