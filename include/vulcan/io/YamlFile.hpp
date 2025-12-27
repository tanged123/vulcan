/**
 * @file YamlFile.hpp
 * @brief YAML file utilities for includes, merging, and discovery
 *
 * Provides:
 * - !include directive resolution
 * - Multi-file merging for config layering
 * - YAML file discovery in directories
 */

#pragma once

#include <vulcan/io/YamlNode.hpp>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace vulcan::io {

/**
 * @brief YAML file utilities
 */
class YamlFile {
  public:
    /**
     * @brief Load YAML from file with !include directive resolution
     *
     * Example:
     *   components: !include components.yaml
     *   routes: !include routes/main.yaml
     *
     * Include paths are relative to the parent YAML file.
     */
    static YamlNode LoadWithIncludes(const std::string &path) {
        auto base_dir = std::filesystem::path(path).parent_path();
        try {
            YAML::Node root = YAML::LoadFile(path);
            ResolveIncludes(root, base_dir);
            return YamlNode(root, path);
        } catch (const YAML::Exception &e) {
            throw YamlError(path, e.what());
        }
    }

    /**
     * @brief Merge multiple YAML files (later files override earlier)
     *
     * Useful for config layering:
     *   base.yaml + env.yaml + local.yaml
     */
    static YamlNode MergeFiles(const std::vector<std::string> &paths) {
        if (paths.empty()) {
            throw YamlError("merge", "no files provided for merge");
        }

        YAML::Node result;
        try {
            result = YAML::LoadFile(paths[0]);
        } catch (const YAML::Exception &e) {
            throw YamlError(paths[0], e.what());
        }

        for (std::size_t i = 1; i < paths.size(); ++i) {
            try {
                YAML::Node overlay = YAML::LoadFile(paths[i]);
                MergeNodes(result, overlay);
            } catch (const YAML::Exception &e) {
                throw YamlError(paths[i], e.what());
            }
        }

        return YamlNode(result, "merged");
    }

    /**
     * @brief Check if file exists and is valid YAML
     */
    static bool IsValidYaml(const std::string &path) {
        if (!std::filesystem::exists(path)) {
            return false;
        }
        try {
            YAML::LoadFile(path);
            return true;
        } catch (const YAML::Exception &) {
            return false;
        }
    }

    /**
     * @brief Get all YAML files in directory
     */
    static std::vector<std::filesystem::path>
    FindYamlFiles(const std::string &directory, bool recursive = false) {
        std::vector<std::filesystem::path> results;

        if (!std::filesystem::exists(directory)) {
            return results;
        }

        auto is_yaml = [](const std::filesystem::path &p) {
            auto ext = p.extension().string();
            return ext == ".yaml" || ext == ".yml";
        };

        if (recursive) {
            for (const auto &entry :
                 std::filesystem::recursive_directory_iterator(directory)) {
                if (entry.is_regular_file() && is_yaml(entry.path())) {
                    results.push_back(entry.path());
                }
            }
        } else {
            for (const auto &entry :
                 std::filesystem::directory_iterator(directory)) {
                if (entry.is_regular_file() && is_yaml(entry.path())) {
                    results.push_back(entry.path());
                }
            }
        }

        return results;
    }

  private:
    /// Recursively resolve !include directives
    static void ResolveIncludes(YAML::Node &node,
                                const std::filesystem::path &base_dir) {
        if (node.IsMap()) {
            for (auto it = node.begin(); it != node.end(); ++it) {
                auto value = it->second;

                // Check for !include tag
                if (value.Tag() == "!include" && value.IsScalar()) {
                    std::string include_path = value.as<std::string>();
                    auto full_path = base_dir / include_path;

                    try {
                        YAML::Node included =
                            YAML::LoadFile(full_path.string());
                        auto include_dir = full_path.parent_path();
                        ResolveIncludes(included, include_dir);
                        node[it->first] = included;
                    } catch (const YAML::Exception &e) {
                        throw YamlError(full_path.string(), e.what());
                    }
                } else {
                    ResolveIncludes(value, base_dir);
                }
            }
        } else if (node.IsSequence()) {
            for (std::size_t i = 0; i < node.size(); ++i) {
                auto element = node[i];

                // Check for !include in sequence
                if (element.Tag() == "!include" && element.IsScalar()) {
                    std::string include_path = element.as<std::string>();
                    auto full_path = base_dir / include_path;

                    try {
                        YAML::Node included =
                            YAML::LoadFile(full_path.string());
                        auto include_dir = full_path.parent_path();
                        ResolveIncludes(included, include_dir);
                        node[i] = included;
                    } catch (const YAML::Exception &e) {
                        throw YamlError(full_path.string(), e.what());
                    }
                } else {
                    ResolveIncludes(element, base_dir);
                }
            }
        }
    }

    /// Recursively merge overlay into base (overlay wins)
    static void MergeNodes(YAML::Node &base, const YAML::Node &overlay) {
        if (overlay.IsMap()) {
            for (const auto &pair : overlay) {
                std::string key = pair.first.as<std::string>();
                if (base[key] && base[key].IsMap() && pair.second.IsMap()) {
                    // Recursive merge for nested maps
                    YAML::Node child = base[key];
                    MergeNodes(child, pair.second);
                    base[key] = child;
                } else {
                    // Override
                    base[key] = pair.second;
                }
            }
        } else {
            // Non-map overlay replaces base entirely
            base = overlay;
        }
    }
};

} // namespace vulcan::io
