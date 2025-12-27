/**
 * @file test_yaml_env.cpp
 * @brief Unit tests for YamlEnv environment variable expansion
 */

#include <vulcan/io/YamlEnv.hpp>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace vulcan::io {
namespace {

class YamlEnvTest : public ::testing::Test {
  protected:
    void SetUp() override {
        temp_dir_ =
            std::filesystem::temp_directory_path() / "vulcan_yaml_env_test";
        std::filesystem::create_directories(temp_dir_);

        // Set test environment variables
        setenv("VULCAN_TEST_VAR", "test_value", 1);
        setenv("VULCAN_TEST_NUM", "42", 1);
        setenv("VULCAN_TEST_EMPTY", "", 1);
    }

    void TearDown() override {
        std::filesystem::remove_all(temp_dir_);

        // Clean up environment variables
        unsetenv("VULCAN_TEST_VAR");
        unsetenv("VULCAN_TEST_NUM");
        unsetenv("VULCAN_TEST_EMPTY");
    }

    void WriteFile(const std::string &name, const std::string &content) {
        std::ofstream out(temp_dir_ / name);
        out << content;
    }

    std::filesystem::path temp_dir_;
};

// =============================================================================
// String Expansion Tests
// =============================================================================

TEST_F(YamlEnvTest, ExpandDefinedVariable) {
    std::string result = YamlEnv::Expand("prefix_${VULCAN_TEST_VAR}_suffix");
    EXPECT_EQ(result, "prefix_test_value_suffix");
}

TEST_F(YamlEnvTest, ExpandMultipleVariables) {
    std::string result =
        YamlEnv::Expand("${VULCAN_TEST_VAR}/${VULCAN_TEST_NUM}");
    EXPECT_EQ(result, "test_value/42");
}

TEST_F(YamlEnvTest, ExpandWithDefault) {
    std::string result = YamlEnv::Expand("${UNDEFINED_VAR:default_value}");
    EXPECT_EQ(result, "default_value");
}

TEST_F(YamlEnvTest, ExpandWithEmptyDefault) {
    std::string result = YamlEnv::Expand("prefix${UNDEFINED_VAR:}suffix");
    EXPECT_EQ(result, "prefixsuffix");
}

TEST_F(YamlEnvTest, ExpandDefinedOverridesDefault) {
    std::string result = YamlEnv::Expand("${VULCAN_TEST_VAR:ignored}");
    EXPECT_EQ(result, "test_value");
}

TEST_F(YamlEnvTest, ExpandUndefinedStrictThrows) {
    EXPECT_THROW(YamlEnv::Expand("${UNDEFINED_VAR}", true), EnvVarError);
}

TEST_F(YamlEnvTest, ExpandUndefinedNonStrictReturnsEmpty) {
    std::string result = YamlEnv::Expand("prefix${UNDEFINED_VAR}suffix", false);
    EXPECT_EQ(result, "prefixsuffix");
}

TEST_F(YamlEnvTest, ExpandEscapeSequence) {
    std::string result = YamlEnv::Expand("literal $${VAR} here");
    EXPECT_EQ(result, "literal ${VAR} here");
}

TEST_F(YamlEnvTest, ExpandMixedEscapeAndReal) {
    std::string result = YamlEnv::Expand("$${ESCAPED} and ${VULCAN_TEST_VAR}");
    EXPECT_EQ(result, "${ESCAPED} and test_value");
}

TEST_F(YamlEnvTest, ExpandNoVariables) {
    std::string result = YamlEnv::Expand("no variables here");
    EXPECT_EQ(result, "no variables here");
}

TEST_F(YamlEnvTest, ExpandEmptyVariable) {
    std::string result = YamlEnv::Expand("${VULCAN_TEST_EMPTY}");
    EXPECT_EQ(result, "");
}

// =============================================================================
// Contains Check Tests
// =============================================================================

TEST_F(YamlEnvTest, ContainsEnvVarsTrue) {
    EXPECT_TRUE(YamlEnv::ContainsEnvVars("has ${VAR} variable"));
    EXPECT_TRUE(YamlEnv::ContainsEnvVars("${VAR}"));
    EXPECT_TRUE(YamlEnv::ContainsEnvVars("${VAR:default}"));
}

TEST_F(YamlEnvTest, ContainsEnvVarsFalse) {
    EXPECT_FALSE(YamlEnv::ContainsEnvVars("no variables"));
    EXPECT_FALSE(YamlEnv::ContainsEnvVars("$VAR without braces"));
    EXPECT_FALSE(YamlEnv::ContainsEnvVars(""));
}

// =============================================================================
// GetEnv Tests
// =============================================================================

TEST_F(YamlEnvTest, GetEnvDefined) {
    auto result = YamlEnv::GetEnv("VULCAN_TEST_VAR");
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), "test_value");
}

TEST_F(YamlEnvTest, GetEnvUndefined) {
    auto result = YamlEnv::GetEnv("UNDEFINED_VAR_12345");
    EXPECT_FALSE(result.has_value());
}

TEST_F(YamlEnvTest, GetEnvWithDefault) {
    EXPECT_EQ(YamlEnv::GetEnv("VULCAN_TEST_VAR", "default"), "test_value");
    EXPECT_EQ(YamlEnv::GetEnv("UNDEFINED_VAR_12345", "default"), "default");
}

// =============================================================================
// Node Expansion Tests
// =============================================================================

TEST_F(YamlEnvTest, ExpandAllStrings) {
    auto node = YamlNode::Parse(R"(
key1: ${VULCAN_TEST_VAR}
key2: literal
nested:
  key3: prefix_${VULCAN_TEST_NUM}
)");

    auto expanded = YamlEnv::ExpandAll(node);

    EXPECT_EQ(expanded.Require<std::string>("key1"), "test_value");
    EXPECT_EQ(expanded.Require<std::string>("key2"), "literal");
    EXPECT_EQ(expanded["nested"].Require<std::string>("key3"), "prefix_42");
}

TEST_F(YamlEnvTest, ExpandAllSequence) {
    auto node = YamlNode::Parse(R"(
items:
  - ${VULCAN_TEST_VAR}
  - literal
  - ${VULCAN_TEST_NUM}
)");

    auto expanded = YamlEnv::ExpandAll(node);
    auto items = expanded["items"];

    EXPECT_EQ(items[0].As<std::string>(), "test_value");
    EXPECT_EQ(items[1].As<std::string>(), "literal");
    EXPECT_EQ(items[2].As<std::string>(), "42");
}

TEST_F(YamlEnvTest, ExpandAllPreservesNumbers) {
    auto node = YamlNode::Parse(R"(
number: 123
string: ${VULCAN_TEST_NUM}
)");

    auto expanded = YamlEnv::ExpandAll(node);

    // Number stays as number
    EXPECT_EQ(expanded.Require<int>("number"), 123);
    // Expanded variable becomes string "42"
    EXPECT_EQ(expanded.Require<std::string>("string"), "42");
}

// =============================================================================
// File Loading with Environment Tests
// =============================================================================

TEST_F(YamlEnvTest, LoadFileWithEnv) {
    WriteFile("config.yaml", R"(
setting: ${VULCAN_TEST_VAR}
port: ${VULCAN_TEST_NUM}
)");

    auto node = YamlEnv::LoadFileWithEnv((temp_dir_ / "config.yaml").string());

    EXPECT_EQ(node.Require<std::string>("setting"), "test_value");
    EXPECT_EQ(node.Require<std::string>("port"), "42");
}

TEST_F(YamlEnvTest, LoadWithIncludesAndEnv) {
    WriteFile("child.yaml", "value: ${VULCAN_TEST_VAR}");
    WriteFile("parent.yaml", "child: !include child.yaml");

    auto node =
        YamlEnv::LoadWithIncludesAndEnv((temp_dir_ / "parent.yaml").string());

    EXPECT_EQ(node["child"].Require<std::string>("value"), "test_value");
}

// =============================================================================
// EnvVarError Tests
// =============================================================================

TEST_F(YamlEnvTest, EnvVarErrorContainsVarName) {
    try {
        YamlEnv::Expand("${UNDEFINED_VARIABLE_NAME}", true);
        FAIL() << "Expected EnvVarError";
    } catch (const EnvVarError &e) {
        EXPECT_EQ(e.var_name(), "UNDEFINED_VARIABLE_NAME");
        std::string msg = e.what();
        EXPECT_TRUE(msg.find("UNDEFINED_VARIABLE_NAME") != std::string::npos);
    }
}

} // namespace
} // namespace vulcan::io
