/**
 * @file test_yaml_node.cpp
 * @brief Unit tests for YamlNode wrapper class
 */

#include <vulcan/io/YamlNode.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace vulcan::io {
namespace {

class YamlNodeTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Create temp directory for test files
        temp_dir_ = std::filesystem::temp_directory_path() / "vulcan_yaml_test";
        std::filesystem::create_directories(temp_dir_);
    }

    void TearDown() override { std::filesystem::remove_all(temp_dir_); }

    void WriteFile(const std::string &name, const std::string &content) {
        std::ofstream out(temp_dir_ / name);
        out << content;
    }

    std::filesystem::path temp_dir_;
};

// =============================================================================
// Parse Tests
// =============================================================================

TEST_F(YamlNodeTest, ParseFromString) {
    auto node = YamlNode::Parse("key: value");
    EXPECT_TRUE(node.IsDefined());
    EXPECT_TRUE(node.IsMap());
    EXPECT_EQ(node.Require<std::string>("key"), "value");
}

TEST_F(YamlNodeTest, LoadFromFile) {
    WriteFile("test.yaml", "number: 42\nname: test");

    auto node = YamlNode::LoadFile((temp_dir_ / "test.yaml").string());
    EXPECT_EQ(node.Require<int>("number"), 42);
    EXPECT_EQ(node.Require<std::string>("name"), "test");
}

TEST_F(YamlNodeTest, LoadNonexistentFileThrows) {
    EXPECT_THROW(YamlNode::LoadFile("/nonexistent/file.yaml"), YamlError);
}

// =============================================================================
// Scalar Type Tests
// =============================================================================

TEST_F(YamlNodeTest, ExtractDouble) {
    auto node = YamlNode::Parse("value: 3.14159");
    EXPECT_DOUBLE_EQ(node.Require<double>("value"), 3.14159);
}

TEST_F(YamlNodeTest, ExtractFloat) {
    auto node = YamlNode::Parse("value: 2.5");
    EXPECT_FLOAT_EQ(node.Require<float>("value"), 2.5f);
}

TEST_F(YamlNodeTest, ExtractInt) {
    auto node = YamlNode::Parse("value: -42");
    EXPECT_EQ(node.Require<int>("value"), -42);
}

TEST_F(YamlNodeTest, ExtractInt64) {
    auto node = YamlNode::Parse("value: 9223372036854775807");
    EXPECT_EQ(node.Require<int64_t>("value"), INT64_MAX);
}

TEST_F(YamlNodeTest, ExtractBoolTrue) {
    auto node = YamlNode::Parse("value: true");
    EXPECT_TRUE(node.Require<bool>("value"));
}

TEST_F(YamlNodeTest, ExtractBoolFalse) {
    auto node = YamlNode::Parse("value: false");
    EXPECT_FALSE(node.Require<bool>("value"));
}

TEST_F(YamlNodeTest, ExtractString) {
    auto node = YamlNode::Parse("value: hello world");
    EXPECT_EQ(node.Require<std::string>("value"), "hello world");
}

// =============================================================================
// Navigation Tests
// =============================================================================

TEST_F(YamlNodeTest, NestedAccess) {
    auto node = YamlNode::Parse(R"(
parent:
  child:
    value: 123
)");
    EXPECT_EQ(node["parent"]["child"].Require<int>("value"), 123);
}

TEST_F(YamlNodeTest, SequenceAccess) {
    auto node = YamlNode::Parse(R"(
items:
  - first
  - second
  - third
)");
    auto items = node["items"];
    EXPECT_TRUE(items.IsSequence());
    EXPECT_EQ(items.Size(), 3u);
    EXPECT_EQ(items[0].As<std::string>(), "first");
    EXPECT_EQ(items[1].As<std::string>(), "second");
    EXPECT_EQ(items[2].As<std::string>(), "third");
}

TEST_F(YamlNodeTest, HasKey) {
    auto node = YamlNode::Parse("exists: 1");
    EXPECT_TRUE(node.Has("exists"));
    EXPECT_FALSE(node.Has("missing"));
}

// =============================================================================
// Optional Accessors
// =============================================================================

TEST_F(YamlNodeTest, GetWithDefault) {
    auto node = YamlNode::Parse("present: 10");
    EXPECT_EQ(node.Get<int>("present", 99), 10);
    EXPECT_EQ(node.Get<int>("missing", 99), 99);
}

TEST_F(YamlNodeTest, GetOptional) {
    auto node = YamlNode::Parse("present: hello");
    auto present = node.GetOptional<std::string>("present");
    auto missing = node.GetOptional<std::string>("missing");

    EXPECT_TRUE(present.has_value());
    EXPECT_EQ(present.value(), "hello");
    EXPECT_FALSE(missing.has_value());
}

// =============================================================================
// Error Messages & Path Tracking
// =============================================================================

TEST_F(YamlNodeTest, PathTrackingInErrors) {
    auto node = YamlNode::Parse(R"(
outer:
  inner:
    value: not_a_number
)");

    try {
        node["outer"]["inner"].Require<int>("value");
        FAIL() << "Expected YamlError";
    } catch (const YamlError &e) {
        std::string msg = e.what();
        EXPECT_TRUE(msg.find("outer.inner.value") != std::string::npos)
            << "Error should contain path, got: " << msg;
    }
}

TEST_F(YamlNodeTest, RequireMissingKeyThrows) {
    auto node = YamlNode::Parse("key: value");
    EXPECT_THROW(node.Require<std::string>("missing"), YamlError);
}

// =============================================================================
// Iteration
// =============================================================================

TEST_F(YamlNodeTest, ForEach) {
    auto node = YamlNode::Parse(R"(
items:
  - 1
  - 2
  - 3
)");

    int sum = 0;
    node["items"].ForEach(
        [&sum](const YamlNode &item) { sum += item.As<int>(); });
    EXPECT_EQ(sum, 6);
}

TEST_F(YamlNodeTest, ForEachEntry) {
    auto node = YamlNode::Parse(R"(
a: 1
b: 2
c: 3
)");

    int sum = 0;
    node.ForEachEntry([&sum](const std::string &key, const YamlNode &value) {
        (void)key;
        sum += value.As<int>();
    });
    EXPECT_EQ(sum, 6);
}

TEST_F(YamlNodeTest, ToVector) {
    auto node = YamlNode::Parse("values: [1.0, 2.0, 3.0]");
    auto vec = node["values"].ToVector<double>();

    EXPECT_EQ(vec.size(), 3u);
    EXPECT_DOUBLE_EQ(vec[0], 1.0);
    EXPECT_DOUBLE_EQ(vec[1], 2.0);
    EXPECT_DOUBLE_EQ(vec[2], 3.0);
}

// =============================================================================
// Janus Type Tests
// =============================================================================

TEST_F(YamlNodeTest, Vec3Extraction) {
    auto node = YamlNode::Parse("pos: [1.0, 2.0, 3.0]");
    auto vec = node.Require<janus::Vec3<double>>("pos");

    EXPECT_DOUBLE_EQ(vec.x(), 1.0);
    EXPECT_DOUBLE_EQ(vec.y(), 2.0);
    EXPECT_DOUBLE_EQ(vec.z(), 3.0);
}

TEST_F(YamlNodeTest, QuaternionExtraction) {
    auto node = YamlNode::Parse("quat: [1.0, 0.0, 0.0, 0.0]");
    auto q = node.Require<janus::Quaternion<double>>("quat");

    EXPECT_DOUBLE_EQ(q.w, 1.0);
    EXPECT_DOUBLE_EQ(q.x, 0.0);
    EXPECT_DOUBLE_EQ(q.y, 0.0);
    EXPECT_DOUBLE_EQ(q.z, 0.0);
}

TEST_F(YamlNodeTest, Mat3NestedFormat) {
    auto node = YamlNode::Parse(R"(
matrix:
  - [1, 0, 0]
  - [0, 1, 0]
  - [0, 0, 1]
)");
    auto m = node.Require<janus::Mat3<double>>("matrix");

    // Should be identity matrix
    EXPECT_DOUBLE_EQ(m(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(m(1, 1), 1.0);
    EXPECT_DOUBLE_EQ(m(2, 2), 1.0);
    EXPECT_DOUBLE_EQ(m(0, 1), 0.0);
}

TEST_F(YamlNodeTest, Mat3FlatFormat) {
    auto node = YamlNode::Parse("matrix: [1, 2, 3, 4, 5, 6, 7, 8, 9]");
    auto m = node.Require<janus::Mat3<double>>("matrix");

    EXPECT_DOUBLE_EQ(m(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(m(0, 2), 3.0);
    EXPECT_DOUBLE_EQ(m(2, 2), 9.0);
}

TEST_F(YamlNodeTest, Vec3WrongSizeThrows) {
    auto node = YamlNode::Parse("bad: [1.0, 2.0]");
    EXPECT_THROW(node.Require<janus::Vec3<double>>("bad"), YamlError);
}

// =============================================================================
// Container Type Tests
// =============================================================================

TEST_F(YamlNodeTest, VectorDouble) {
    auto node = YamlNode::Parse("values: [1.5, 2.5, 3.5]");
    auto vec = node.Require<std::vector<double>>("values");

    EXPECT_EQ(vec.size(), 3u);
    EXPECT_DOUBLE_EQ(vec[0], 1.5);
    EXPECT_DOUBLE_EQ(vec[2], 3.5);
}

TEST_F(YamlNodeTest, VectorInt) {
    auto node = YamlNode::Parse("values: [1, 2, 3]");
    auto vec = node.Require<std::vector<int>>("values");

    EXPECT_EQ(vec.size(), 3u);
    EXPECT_EQ(vec[0], 1);
    EXPECT_EQ(vec[2], 3);
}

TEST_F(YamlNodeTest, VectorString) {
    auto node = YamlNode::Parse("values: [a, b, c]");
    auto vec = node.Require<std::vector<std::string>>("values");

    EXPECT_EQ(vec.size(), 3u);
    EXPECT_EQ(vec[0], "a");
    EXPECT_EQ(vec[2], "c");
}

} // namespace
} // namespace vulcan::io
