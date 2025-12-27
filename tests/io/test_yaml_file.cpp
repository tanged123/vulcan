/**
 * @file test_yaml_file.cpp
 * @brief Unit tests for YamlFile utilities (includes, merge, discovery)
 */

#include <vulcan/io/YamlFile.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace vulcan::io {
namespace {

class YamlFileTest : public ::testing::Test {
  protected:
    void SetUp() override {
        temp_dir_ =
            std::filesystem::temp_directory_path() / "vulcan_yaml_file_test";
        std::filesystem::create_directories(temp_dir_);
    }

    void TearDown() override { std::filesystem::remove_all(temp_dir_); }

    void WriteFile(const std::string &name, const std::string &content) {
        auto path = temp_dir_ / name;
        std::filesystem::create_directories(path.parent_path());
        std::ofstream out(path);
        out << content;
    }

    std::filesystem::path temp_dir_;
};

// =============================================================================
// !include Directive Tests
// =============================================================================

TEST_F(YamlFileTest, IncludeDirective) {
    WriteFile("child.yaml", "value: 42");
    WriteFile("parent.yaml", "child: !include child.yaml");

    auto node =
        YamlFile::LoadWithIncludes((temp_dir_ / "parent.yaml").string());

    EXPECT_EQ(node["child"].Require<int>("value"), 42);
}

TEST_F(YamlFileTest, NestedIncludes) {
    WriteFile("level2.yaml", "deep: value");
    WriteFile("level1.yaml", "nested: !include level2.yaml");
    WriteFile("root.yaml", "level1: !include level1.yaml");

    auto node = YamlFile::LoadWithIncludes((temp_dir_ / "root.yaml").string());

    EXPECT_EQ(node["level1"]["nested"].Require<std::string>("deep"), "value");
}

TEST_F(YamlFileTest, IncludeInSubdirectory) {
    WriteFile("subdir/config.yaml", "setting: enabled");
    WriteFile("main.yaml", "config: !include subdir/config.yaml");

    auto node = YamlFile::LoadWithIncludes((temp_dir_ / "main.yaml").string());

    EXPECT_EQ(node["config"].Require<std::string>("setting"), "enabled");
}

TEST_F(YamlFileTest, IncludeSequenceElement) {
    WriteFile("item.yaml", "name: included_item");
    WriteFile("list.yaml", R"(
items:
  - name: first
  - !include item.yaml
  - name: last
)");

    auto node = YamlFile::LoadWithIncludes((temp_dir_ / "list.yaml").string());
    auto items = node["items"];

    EXPECT_EQ(items.Size(), 3u);
    EXPECT_EQ(items[1].Require<std::string>("name"), "included_item");
}

TEST_F(YamlFileTest, IncludeMissingFileThrows) {
    WriteFile("bad.yaml", "missing: !include nonexistent.yaml");

    EXPECT_THROW(YamlFile::LoadWithIncludes((temp_dir_ / "bad.yaml").string()),
                 YamlError);
}

// =============================================================================
// File Merge Tests
// =============================================================================

TEST_F(YamlFileTest, MergeTwoFiles) {
    WriteFile("base.yaml", "a: 1\nb: 2");
    WriteFile("overlay.yaml", "b: 20\nc: 3");

    auto node = YamlFile::MergeFiles({(temp_dir_ / "base.yaml").string(),
                                      (temp_dir_ / "overlay.yaml").string()});

    EXPECT_EQ(node.Require<int>("a"), 1);  // From base
    EXPECT_EQ(node.Require<int>("b"), 20); // Overridden by overlay
    EXPECT_EQ(node.Require<int>("c"), 3);  // From overlay
}

TEST_F(YamlFileTest, MergeNestedMaps) {
    WriteFile("base.yaml", R"(
database:
  host: localhost
  port: 5432
)");
    WriteFile("overlay.yaml", R"(
database:
  port: 5433
  name: mydb
)");

    auto node = YamlFile::MergeFiles({(temp_dir_ / "base.yaml").string(),
                                      (temp_dir_ / "overlay.yaml").string()});

    auto db = node["database"];
    EXPECT_EQ(db.Require<std::string>("host"), "localhost"); // Preserved
    EXPECT_EQ(db.Require<int>("port"), 5433);                // Overridden
    EXPECT_EQ(db.Require<std::string>("name"), "mydb");      // Added
}

TEST_F(YamlFileTest, MergeThreeFiles) {
    WriteFile("a.yaml", "x: 1");
    WriteFile("b.yaml", "x: 2\ny: 2");
    WriteFile("c.yaml", "y: 3\nz: 3");

    auto node = YamlFile::MergeFiles({(temp_dir_ / "a.yaml").string(),
                                      (temp_dir_ / "b.yaml").string(),
                                      (temp_dir_ / "c.yaml").string()});

    EXPECT_EQ(node.Require<int>("x"), 2); // b overrides a
    EXPECT_EQ(node.Require<int>("y"), 3); // c overrides b
    EXPECT_EQ(node.Require<int>("z"), 3); // From c
}

TEST_F(YamlFileTest, MergeEmptyListThrows) {
    EXPECT_THROW(YamlFile::MergeFiles({}), YamlError);
}

// =============================================================================
// Validation Tests
// =============================================================================

TEST_F(YamlFileTest, IsValidYamlTrue) {
    WriteFile("valid.yaml", "key: value");
    EXPECT_TRUE(YamlFile::IsValidYaml((temp_dir_ / "valid.yaml").string()));
}

TEST_F(YamlFileTest, IsValidYamlFalse) {
    WriteFile("invalid.yaml", "key: [unclosed");
    EXPECT_FALSE(YamlFile::IsValidYaml((temp_dir_ / "invalid.yaml").string()));
}

TEST_F(YamlFileTest, IsValidYamlNonexistent) {
    EXPECT_FALSE(YamlFile::IsValidYaml("/nonexistent/file.yaml"));
}

// =============================================================================
// Directory Discovery Tests
// =============================================================================

TEST_F(YamlFileTest, FindYamlFilesNonRecursive) {
    WriteFile("a.yaml", "");
    WriteFile("b.yml", "");
    WriteFile("c.txt", "");
    WriteFile("subdir/d.yaml", "");

    auto files = YamlFile::FindYamlFiles(temp_dir_.string(), false);

    EXPECT_EQ(files.size(), 2u); // a.yaml and b.yml, not subdir/d.yaml
}

TEST_F(YamlFileTest, FindYamlFilesRecursive) {
    WriteFile("a.yaml", "");
    WriteFile("subdir/b.yaml", "");
    WriteFile("subdir/deep/c.yml", "");

    auto files = YamlFile::FindYamlFiles(temp_dir_.string(), true);

    EXPECT_EQ(files.size(), 3u);
}

TEST_F(YamlFileTest, FindYamlFilesEmptyDirectory) {
    auto files = YamlFile::FindYamlFiles(temp_dir_.string(), false);
    EXPECT_TRUE(files.empty());
}

TEST_F(YamlFileTest, FindYamlFilesNonexistentDirectory) {
    auto files = YamlFile::FindYamlFiles("/nonexistent/dir", false);
    EXPECT_TRUE(files.empty());
}

} // namespace
} // namespace vulcan::io
