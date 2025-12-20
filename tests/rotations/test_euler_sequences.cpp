#include <gtest/gtest.h>
#include <vulcan/rotations/Rotations.hpp>

#include <janus/janus.hpp>

#include <cmath>
#include <numbers>

// =============================================================================
// Euler Sequence Enum Tests
// =============================================================================

TEST(EulerSequences, AxisMapping) {
    // Verify axis mappings for key sequences
    auto zyx = vulcan::euler_axes(vulcan::EulerSequence::ZYX);
    EXPECT_EQ(zyx[0], 2); // Z
    EXPECT_EQ(zyx[1], 1); // Y
    EXPECT_EQ(zyx[2], 0); // X

    auto xyz = vulcan::euler_axes(vulcan::EulerSequence::XYZ);
    EXPECT_EQ(xyz[0], 0); // X
    EXPECT_EQ(xyz[1], 1); // Y
    EXPECT_EQ(xyz[2], 2); // Z

    auto zxz = vulcan::euler_axes(vulcan::EulerSequence::ZXZ);
    EXPECT_EQ(zxz[0], 2); // Z
    EXPECT_EQ(zxz[1], 0); // X
    EXPECT_EQ(zxz[2], 2); // Z
}

TEST(EulerSequences, IsProperEuler) {
    // Tait-Bryan: all different axes
    EXPECT_FALSE(vulcan::is_proper_euler(vulcan::EulerSequence::ZYX));
    EXPECT_FALSE(vulcan::is_proper_euler(vulcan::EulerSequence::XYZ));
    EXPECT_FALSE(vulcan::is_proper_euler(vulcan::EulerSequence::YZX));

    // Proper Euler: first == third axis
    EXPECT_TRUE(vulcan::is_proper_euler(vulcan::EulerSequence::ZXZ));
    EXPECT_TRUE(vulcan::is_proper_euler(vulcan::EulerSequence::ZYZ));
    EXPECT_TRUE(vulcan::is_proper_euler(vulcan::EulerSequence::XYX));
}

// =============================================================================
// DCM from Euler - Identity Tests
// =============================================================================

TEST(DCMFromEuler, IdentityAllSequences) {
    // Zero angles should give identity for all sequences
    std::vector<vulcan::EulerSequence> all_sequences = {
        vulcan::EulerSequence::XYZ, vulcan::EulerSequence::XZY,
        vulcan::EulerSequence::YXZ, vulcan::EulerSequence::YZX,
        vulcan::EulerSequence::ZXY, vulcan::EulerSequence::ZYX,
        vulcan::EulerSequence::XYX, vulcan::EulerSequence::XZX,
        vulcan::EulerSequence::YXY, vulcan::EulerSequence::YZY,
        vulcan::EulerSequence::ZXZ, vulcan::EulerSequence::ZYZ};

    for (auto seq : all_sequences) {
        auto R = vulcan::dcm_from_euler(0.0, 0.0, 0.0, seq);
        EXPECT_NEAR(R(0, 0), 1.0, 1e-10)
            << "Failed for " << vulcan::euler_sequence_name(seq);
        EXPECT_NEAR(R(1, 1), 1.0, 1e-10)
            << "Failed for " << vulcan::euler_sequence_name(seq);
        EXPECT_NEAR(R(2, 2), 1.0, 1e-10)
            << "Failed for " << vulcan::euler_sequence_name(seq);
        EXPECT_NEAR(R(0, 1), 0.0, 1e-10)
            << "Failed for " << vulcan::euler_sequence_name(seq);
    }
}

TEST(DCMFromEuler, ZYX_MatchesJanus) {
    // Verify ZYX matches Janus rotation_matrix_from_euler_angles
    double roll = 0.3;
    double pitch = 0.2;
    double yaw = 0.5;

    auto R_vulcan =
        vulcan::dcm_from_euler(yaw, pitch, roll, vulcan::EulerSequence::ZYX);
    auto R_janus = janus::rotation_matrix_from_euler_angles(roll, pitch, yaw);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(R_vulcan(i, j), R_janus(i, j), 1e-12)
                << "Mismatch at (" << i << "," << j << ")";
        }
    }
}

// =============================================================================
// Euler Roundtrip Tests - Tait-Bryan
// =============================================================================

class TaitBryanRoundtrip
    : public ::testing::TestWithParam<vulcan::EulerSequence> {};

TEST_P(TaitBryanRoundtrip, DCMRoundtrip) {
    auto seq = GetParam();

    // Test angles (avoiding gimbal lock)
    double e1 = 0.3;
    double e2 = 0.2;
    double e3 = 0.1;

    auto R = vulcan::dcm_from_euler(e1, e2, e3, seq);
    auto euler_back = vulcan::euler_from_dcm(R, seq);

    EXPECT_NEAR(euler_back(0), e1, 1e-10)
        << "e1 failed for " << vulcan::euler_sequence_name(seq);
    EXPECT_NEAR(euler_back(1), e2, 1e-10)
        << "e2 failed for " << vulcan::euler_sequence_name(seq);
    EXPECT_NEAR(euler_back(2), e3, 1e-10)
        << "e3 failed for " << vulcan::euler_sequence_name(seq);
}

TEST_P(TaitBryanRoundtrip, QuaternionRoundtrip) {
    auto seq = GetParam();

    double e1 = 0.4;
    double e2 = 0.15;
    double e3 = 0.25;

    auto q = vulcan::quaternion_from_euler(e1, e2, e3, seq);
    auto euler_back = vulcan::euler_from_quaternion(q, seq);

    EXPECT_NEAR(euler_back(0), e1, 1e-10)
        << "e1 failed for " << vulcan::euler_sequence_name(seq);
    EXPECT_NEAR(euler_back(1), e2, 1e-10)
        << "e2 failed for " << vulcan::euler_sequence_name(seq);
    EXPECT_NEAR(euler_back(2), e3, 1e-10)
        << "e3 failed for " << vulcan::euler_sequence_name(seq);
}

INSTANTIATE_TEST_SUITE_P(
    TaitBryan, TaitBryanRoundtrip,
    ::testing::Values(vulcan::EulerSequence::XYZ, vulcan::EulerSequence::XZY,
                      vulcan::EulerSequence::YXZ, vulcan::EulerSequence::YZX,
                      vulcan::EulerSequence::ZXY, vulcan::EulerSequence::ZYX));

// =============================================================================
// Euler Roundtrip Tests - Proper Euler
// =============================================================================

class ProperEulerRoundtrip
    : public ::testing::TestWithParam<vulcan::EulerSequence> {};

TEST_P(ProperEulerRoundtrip, DCMRoundtrip) {
    auto seq = GetParam();

    // For proper Euler, e2 must not be 0 or π (singularity)
    double e1 = 0.3;
    double e2 = 0.8; // Not near 0 or π
    double e3 = 0.25;

    auto R = vulcan::dcm_from_euler(e1, e2, e3, seq);
    auto euler_back = vulcan::euler_from_dcm(R, seq);

    EXPECT_NEAR(euler_back(0), e1, 1e-10)
        << "e1 failed for " << vulcan::euler_sequence_name(seq);
    EXPECT_NEAR(euler_back(1), e2, 1e-10)
        << "e2 failed for " << vulcan::euler_sequence_name(seq);
    EXPECT_NEAR(euler_back(2), e3, 1e-10)
        << "e3 failed for " << vulcan::euler_sequence_name(seq);
}

INSTANTIATE_TEST_SUITE_P(
    ProperEuler, ProperEulerRoundtrip,
    ::testing::Values(vulcan::EulerSequence::XYX, vulcan::EulerSequence::XZX,
                      vulcan::EulerSequence::YXY, vulcan::EulerSequence::YZY,
                      vulcan::EulerSequence::ZXZ, vulcan::EulerSequence::ZYZ));

// =============================================================================
// Gimbal Lock Tests
// =============================================================================

TEST(GimbalLock, ZYX_Pitch90) {
    // Gimbal lock at pitch = 90°
    double yaw = 0.5;
    double pitch = std::numbers::pi / 2.0;
    double roll = 0.0; // At gimbal lock, roll is arbitrary; we set to 0

    auto R =
        vulcan::dcm_from_euler(yaw, pitch, roll, vulcan::EulerSequence::ZYX);
    auto euler_back = vulcan::euler_from_dcm(R, vulcan::EulerSequence::ZYX);

    // Pitch should be preserved
    EXPECT_NEAR(euler_back(1), pitch, 1e-6);

    // Roll should be 0 (our convention at gimbal lock)
    EXPECT_NEAR(euler_back(2), 0.0, 1e-6);

    // Reconstruct and verify same rotation
    auto R_reconstructed =
        vulcan::dcm_from_euler(euler_back(0), euler_back(1), euler_back(2),
                               vulcan::EulerSequence::ZYX);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(R(i, j), R_reconstructed(i, j), 1e-6);
        }
    }
}

TEST(GimbalLock, ZXZ_Nutation0) {
    // Gimbal lock at nutation = 0
    double precession = 0.5;
    double nutation = 0.0; // Singularity
    double spin = 0.3;

    auto R = vulcan::dcm_from_euler(precession, nutation, spin,
                                    vulcan::EulerSequence::ZXZ);
    auto euler_back = vulcan::euler_from_dcm(R, vulcan::EulerSequence::ZXZ);

    // Nutation should be preserved
    EXPECT_NEAR(euler_back(1), nutation, 1e-6);

    // Spin should be 0 (our convention at gimbal lock)
    EXPECT_NEAR(euler_back(2), 0.0, 1e-6);

    // Reconstruct and verify same rotation
    auto R_reconstructed =
        vulcan::dcm_from_euler(euler_back(0), euler_back(1), euler_back(2),
                               vulcan::EulerSequence::ZXZ);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(R(i, j), R_reconstructed(i, j), 1e-6);
        }
    }
}

// =============================================================================
// Symbolic Compatibility Tests
// =============================================================================

TEST(SymbolicEuler, DCMConstruction) {
    using Scalar = janus::SymbolicScalar;

    Scalar e1 = janus::sym("e1");
    Scalar e2 = janus::sym("e2");
    Scalar e3 = janus::sym("e3");

    auto R = vulcan::dcm_from_euler(e1, e2, e3, vulcan::EulerSequence::ZYX);

    // Verify it's symbolic
    EXPECT_FALSE(R(0, 0).is_constant());

    // Create function and evaluate
    janus::Function f("dcm_zyx", {e1, e2, e3}, {R(0, 0), R(1, 0), R(2, 0)});

    double test_e1 = 0.5;
    double test_e2 = 0.2;
    double test_e3 = 0.1;

    auto result = f({test_e1, test_e2, test_e3});

    // Compare with numeric
    auto R_numeric = vulcan::dcm_from_euler(test_e1, test_e2, test_e3,
                                            vulcan::EulerSequence::ZYX);

    EXPECT_NEAR(result[0](0, 0), R_numeric(0, 0), 1e-12);
    EXPECT_NEAR(result[1](0, 0), R_numeric(1, 0), 1e-12);
    EXPECT_NEAR(result[2](0, 0), R_numeric(2, 0), 1e-12);
}
