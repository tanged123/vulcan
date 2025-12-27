/**
 * @file test_yaml_convert.cpp
 * @brief Unit tests for yaml-cpp conversion traits for Janus types
 */

#include <vulcan/io/YamlConvert.hpp>

#include <gtest/gtest.h>

namespace vulcan::io {
namespace {

// =============================================================================
// Vec3 Round-Trip Tests
// =============================================================================

TEST(YamlConvertTest, Vec3Encode) {
    janus::Vec3<double> v{1.0, 2.0, 3.0};

    YAML::Node node;
    node = v;

    EXPECT_TRUE(node.IsSequence());
    EXPECT_EQ(node.size(), 3u);
    EXPECT_DOUBLE_EQ(node[0].as<double>(), 1.0);
    EXPECT_DOUBLE_EQ(node[1].as<double>(), 2.0);
    EXPECT_DOUBLE_EQ(node[2].as<double>(), 3.0);
}

TEST(YamlConvertTest, Vec3Decode) {
    YAML::Node node;
    node.push_back(4.0);
    node.push_back(5.0);
    node.push_back(6.0);

    auto v = node.as<janus::Vec3<double>>();

    EXPECT_DOUBLE_EQ(v.x(), 4.0);
    EXPECT_DOUBLE_EQ(v.y(), 5.0);
    EXPECT_DOUBLE_EQ(v.z(), 6.0);
}

TEST(YamlConvertTest, Vec3RoundTrip) {
    janus::Vec3<double> original{-1.5, 0.0, 3.14159};

    YAML::Emitter out;
    out << original;

    YAML::Node node = YAML::Load(out.c_str());
    auto restored = node.as<janus::Vec3<double>>();

    EXPECT_DOUBLE_EQ(restored.x(), original.x());
    EXPECT_DOUBLE_EQ(restored.y(), original.y());
    EXPECT_DOUBLE_EQ(restored.z(), original.z());
}

TEST(YamlConvertTest, Vec3DecodeWrongSize) {
    YAML::Node node;
    node.push_back(1.0);
    node.push_back(2.0); // Only 2 elements

    janus::Vec3<double> v;
    EXPECT_FALSE(YAML::convert<janus::Vec3<double>>::decode(node, v));
}

// =============================================================================
// Quaternion Round-Trip Tests
// =============================================================================

TEST(YamlConvertTest, QuaternionEncode) {
    janus::Quaternion<double> q{1.0, 0.0, 0.0, 0.0}; // Identity

    YAML::Node node;
    node = q;

    EXPECT_TRUE(node.IsSequence());
    EXPECT_EQ(node.size(), 4u);
    EXPECT_DOUBLE_EQ(node[0].as<double>(), 1.0); // w
    EXPECT_DOUBLE_EQ(node[1].as<double>(), 0.0); // x
    EXPECT_DOUBLE_EQ(node[2].as<double>(), 0.0); // y
    EXPECT_DOUBLE_EQ(node[3].as<double>(), 0.0); // z
}

TEST(YamlConvertTest, QuaternionDecode) {
    YAML::Node node;
    node.push_back(0.707);
    node.push_back(0.707);
    node.push_back(0.0);
    node.push_back(0.0);

    auto q = node.as<janus::Quaternion<double>>();

    EXPECT_DOUBLE_EQ(q.w, 0.707);
    EXPECT_DOUBLE_EQ(q.x, 0.707);
    EXPECT_DOUBLE_EQ(q.y, 0.0);
    EXPECT_DOUBLE_EQ(q.z, 0.0);
}

TEST(YamlConvertTest, QuaternionRoundTrip) {
    // 90 degree rotation around Z
    janus::Quaternion<double> original{0.7071067811865476, 0.0, 0.0,
                                       0.7071067811865476};

    YAML::Emitter out;
    out << original;

    YAML::Node node = YAML::Load(out.c_str());
    auto restored = node.as<janus::Quaternion<double>>();

    EXPECT_NEAR(restored.w, original.w, 1e-10);
    EXPECT_NEAR(restored.x, original.x, 1e-10);
    EXPECT_NEAR(restored.y, original.y, 1e-10);
    EXPECT_NEAR(restored.z, original.z, 1e-10);
}

TEST(YamlConvertTest, QuaternionDecodeWrongSize) {
    YAML::Node node;
    node.push_back(1.0);
    node.push_back(0.0);
    node.push_back(0.0);
    // Only 3 elements

    janus::Quaternion<double> q;
    EXPECT_FALSE(YAML::convert<janus::Quaternion<double>>::decode(node, q));
}

// =============================================================================
// Mat3 Round-Trip Tests
// =============================================================================

TEST(YamlConvertTest, Mat3EncodeAsNested) {
    janus::Mat3<double> m = janus::Mat3<double>::Identity();

    YAML::Node node;
    node = m;

    EXPECT_TRUE(node.IsSequence());
    EXPECT_EQ(node.size(), 3u);
    EXPECT_TRUE(node[0].IsSequence());
    EXPECT_EQ(node[0].size(), 3u);

    // Check identity matrix
    EXPECT_DOUBLE_EQ(node[0][0].as<double>(), 1.0);
    EXPECT_DOUBLE_EQ(node[1][1].as<double>(), 1.0);
    EXPECT_DOUBLE_EQ(node[2][2].as<double>(), 1.0);
    EXPECT_DOUBLE_EQ(node[0][1].as<double>(), 0.0);
}

TEST(YamlConvertTest, Mat3DecodeNested) {
    YAML::Node node = YAML::Load("[[1, 2, 3], [4, 5, 6], [7, 8, 9]]");

    auto m = node.as<janus::Mat3<double>>();

    EXPECT_DOUBLE_EQ(m(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(m(0, 2), 3.0);
    EXPECT_DOUBLE_EQ(m(1, 1), 5.0);
    EXPECT_DOUBLE_EQ(m(2, 2), 9.0);
}

TEST(YamlConvertTest, Mat3DecodeFlat) {
    YAML::Node node = YAML::Load("[1, 2, 3, 4, 5, 6, 7, 8, 9]");

    auto m = node.as<janus::Mat3<double>>();

    // Row-major order
    EXPECT_DOUBLE_EQ(m(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(m(0, 1), 2.0);
    EXPECT_DOUBLE_EQ(m(0, 2), 3.0);
    EXPECT_DOUBLE_EQ(m(1, 0), 4.0);
    EXPECT_DOUBLE_EQ(m(2, 2), 9.0);
}

TEST(YamlConvertTest, Mat3RoundTrip) {
    janus::Mat3<double> original;
    original << 1, 2, 3, 4, 5, 6, 7, 8, 9;

    YAML::Emitter out;
    out << original;

    YAML::Node node = YAML::Load(out.c_str());
    auto restored = node.as<janus::Mat3<double>>();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_DOUBLE_EQ(restored(i, j), original(i, j));
        }
    }
}

TEST(YamlConvertTest, Mat3DecodeInvalidFormat) {
    // Wrong number of elements
    YAML::Node node = YAML::Load("[1, 2, 3, 4, 5]");

    janus::Mat3<double> m;
    EXPECT_FALSE(YAML::convert<janus::Mat3<double>>::decode(node, m));
}

TEST(YamlConvertTest, Mat3DecodeNestedWrongRowSize) {
    // Second row has wrong size
    YAML::Node node = YAML::Load("[[1, 2, 3], [4, 5], [7, 8, 9]]");

    janus::Mat3<double> m;
    EXPECT_FALSE(YAML::convert<janus::Mat3<double>>::decode(node, m));
}

} // namespace
} // namespace vulcan::io
