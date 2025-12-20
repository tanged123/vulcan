#include <gtest/gtest.h>
#include <vulcan/rotations/Rotations.hpp>

#include <janus/janus.hpp>

#include <cmath>
#include <numbers>

// =============================================================================
// Axis-Angle to Quaternion Tests
// =============================================================================

TEST(AxisAngle, QuaternionFromAxisAngle_ZAxis) {
    // 90° rotation about Z
    vulcan::Vec3<double> axis;
    axis << 0.0, 0.0, 1.0;
    double angle = std::numbers::pi / 2.0;

    auto q = vulcan::quaternion_from_axis_angle(axis, angle);

    // Expected: w = cos(45°), x = 0, y = 0, z = sin(45°)
    double expected_w = std::cos(angle / 2.0);
    double expected_z = std::sin(angle / 2.0);

    EXPECT_NEAR(q.w, expected_w, 1e-12);
    EXPECT_NEAR(q.x, 0.0, 1e-12);
    EXPECT_NEAR(q.y, 0.0, 1e-12);
    EXPECT_NEAR(q.z, expected_z, 1e-12);
}

TEST(AxisAngle, QuaternionFromAxisAngle_Identity) {
    // Zero angle should give identity quaternion
    vulcan::Vec3<double> axis;
    axis << 1.0, 0.0, 0.0;
    double angle = 0.0;

    auto q = vulcan::quaternion_from_axis_angle(axis, angle);

    EXPECT_NEAR(q.w, 1.0, 1e-12);
    EXPECT_NEAR(q.x, 0.0, 1e-12);
    EXPECT_NEAR(q.y, 0.0, 1e-12);
    EXPECT_NEAR(q.z, 0.0, 1e-12);
}

// =============================================================================
// Axis-Angle to DCM Tests (Rodrigues)
// =============================================================================

TEST(AxisAngle, DCMFromAxisAngle_ZAxis) {
    vulcan::Vec3<double> axis;
    axis << 0.0, 0.0, 1.0;
    double angle = std::numbers::pi / 2.0;

    auto R = vulcan::dcm_from_axis_angle(axis, angle);

    // 90° about Z: [0,1,0; -1,0,0; 0,0,1]
    EXPECT_NEAR(R(0, 0), 0.0, 1e-10);
    EXPECT_NEAR(R(0, 1), -1.0, 1e-10);
    EXPECT_NEAR(R(1, 0), 1.0, 1e-10);
    EXPECT_NEAR(R(1, 1), 0.0, 1e-10);
    EXPECT_NEAR(R(2, 2), 1.0, 1e-10);
}

TEST(AxisAngle, DCMFromAxisAngle_Arbitrary) {
    // Arbitrary axis and angle
    vulcan::Vec3<double> axis;
    axis << 1.0, 1.0, 1.0;
    axis.normalize();
    double angle = 0.7;

    auto R = vulcan::dcm_from_axis_angle(axis, angle);

    // Verify valid rotation matrix
    EXPECT_TRUE(vulcan::is_valid_dcm(R));

    // Verify via quaternion
    auto q = vulcan::quaternion_from_axis_angle(axis, angle);
    auto R_via_quat = q.to_rotation_matrix();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(R(i, j), R_via_quat(i, j), 1e-12);
        }
    }
}

// =============================================================================
// Rotation Vector Tests
// =============================================================================

TEST(AxisAngle, RotationVectorRoundtrip) {
    vulcan::Vec3<double> rot_vec;
    rot_vec << 0.3, -0.2, 0.5;

    auto q = vulcan::quaternion_from_rotation_vector(rot_vec);
    auto rot_vec_back = vulcan::rotation_vector_from_quaternion(q);

    EXPECT_NEAR(rot_vec_back(0), rot_vec(0), 1e-10);
    EXPECT_NEAR(rot_vec_back(1), rot_vec(1), 1e-10);
    EXPECT_NEAR(rot_vec_back(2), rot_vec(2), 1e-10);
}

TEST(AxisAngle, DCMFromRotationVector) {
    vulcan::Vec3<double> rot_vec;
    rot_vec << 0.4, 0.3, 0.2;

    auto R = vulcan::dcm_from_rotation_vector(rot_vec);
    auto q = vulcan::quaternion_from_rotation_vector(rot_vec);
    auto R_via_quat = q.to_rotation_matrix();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(R(i, j), R_via_quat(i, j), 1e-12);
        }
    }
}

// =============================================================================
// Axis-Angle Extraction Tests
// =============================================================================

TEST(AxisAngle, ExtractFromQuaternion) {
    vulcan::Vec3<double> axis;
    axis << 0.0, 1.0, 0.0; // Y-axis
    double angle = 0.8;

    auto q = vulcan::quaternion_from_axis_angle(axis, angle);
    auto [axis_back, angle_back] = vulcan::axis_angle_from_quaternion(q);

    EXPECT_NEAR(angle_back, angle, 1e-10);
    EXPECT_NEAR(axis_back(0), axis(0), 1e-10);
    EXPECT_NEAR(axis_back(1), axis(1), 1e-10);
    EXPECT_NEAR(axis_back(2), axis(2), 1e-10);
}

TEST(AxisAngle, ExtractFromDCM) {
    vulcan::Vec3<double> axis;
    axis << 1.0, 0.0, 0.0; // X-axis
    double angle = 1.2;

    auto R = vulcan::dcm_from_axis_angle(axis, angle);
    auto [axis_back, angle_back] = vulcan::axis_angle_from_dcm(R);

    EXPECT_NEAR(angle_back, angle, 1e-10);
    EXPECT_NEAR(std::abs(axis_back(0)), 1.0, 1e-10);
    EXPECT_NEAR(axis_back(1), 0.0, 1e-10);
    EXPECT_NEAR(axis_back(2), 0.0, 1e-10);
}

TEST(AxisAngle, RotationVectorFromDCM) {
    vulcan::Vec3<double> rot_vec;
    rot_vec << 0.5, 0.3, -0.4;

    auto R = vulcan::dcm_from_rotation_vector(rot_vec);
    auto rot_vec_back = vulcan::rotation_vector_from_dcm(R);

    EXPECT_NEAR(rot_vec_back(0), rot_vec(0), 1e-10);
    EXPECT_NEAR(rot_vec_back(1), rot_vec(1), 1e-10);
    EXPECT_NEAR(rot_vec_back(2), rot_vec(2), 1e-10);
}

// =============================================================================
// Edge Cases
// =============================================================================

TEST(AxisAngle, SmallAngle) {
    // Very small angle
    vulcan::Vec3<double> axis;
    axis << 0.0, 0.0, 1.0;
    double angle = 1e-8;

    auto q = vulcan::quaternion_from_axis_angle(axis, angle);
    EXPECT_NEAR(q.w, 1.0, 1e-10);
    EXPECT_NEAR(q.norm(), 1.0, 1e-12);
}

TEST(AxisAngle, PiRotation) {
    // 180° rotation
    vulcan::Vec3<double> axis;
    axis << 1.0, 0.0, 0.0;
    double angle = std::numbers::pi;

    auto R = vulcan::dcm_from_axis_angle(axis, angle);
    EXPECT_TRUE(vulcan::is_valid_dcm(R));

    // 180° about X: diag(1, -1, -1)
    EXPECT_NEAR(R(0, 0), 1.0, 1e-10);
    EXPECT_NEAR(R(1, 1), -1.0, 1e-10);
    EXPECT_NEAR(R(2, 2), -1.0, 1e-10);
}

// =============================================================================
// Symbolic Tests
// =============================================================================

TEST(AxisAngle, SymbolicRodrigues) {
    using Scalar = janus::SymbolicScalar;

    Scalar angle = janus::sym("angle");

    vulcan::Vec3<Scalar> axis;
    axis << Scalar(0), Scalar(0), Scalar(1); // Fixed Z-axis

    auto R = vulcan::dcm_from_axis_angle(axis, angle);

    // Verify symbolic
    EXPECT_FALSE(R(0, 0).is_constant());

    // Create function
    janus::Function f("rodrigues_z", {angle},
                      {R(0, 0), R(0, 1), R(1, 0), R(1, 1)});

    double test_angle = std::numbers::pi / 4.0;
    auto result = f({test_angle});

    EXPECT_NEAR(result[0](0, 0), std::cos(test_angle), 1e-12);
    EXPECT_NEAR(result[1](0, 0), -std::sin(test_angle), 1e-12);
    EXPECT_NEAR(result[2](0, 0), std::sin(test_angle), 1e-12);
    EXPECT_NEAR(result[3](0, 0), std::cos(test_angle), 1e-12);
}
