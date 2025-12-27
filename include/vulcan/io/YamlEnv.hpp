/**
 * @file YamlEnv.hpp
 * @brief Environment variable expansion for YAML configuration
 *
 * Supports:
 * - ${VAR} - Required variable (throws if undefined)
 * - ${VAR:default} - Optional with fallback value
 * - $${VAR} - Escape to produce literal ${VAR}
 */

#pragma once

#include <vulcan/io/YamlFile.hpp>
#include <vulcan/io/YamlNode.hpp>

#include <cstdlib>
#include <optional>
#include <string>

namespace vulcan::io {

/**
 * @brief Exception for undefined environment variables
 */
class EnvVarError : public YamlError {
  public:
    explicit EnvVarError(const std::string &var_name)
        : YamlError("env", "undefined environment variable: " + var_name),
          var_name_(var_name) {}

    [[nodiscard]] const std::string &var_name() const { return var_name_; }

  private:
    std::string var_name_;
};

/**
 * @brief Environment variable expansion utilities
 */
class YamlEnv {
  public:
    // =========================================================================
    // String Expansion
    // =========================================================================

    /**
     * @brief Expand environment variables in a string
     *
     * @param value String potentially containing ${VAR} or ${VAR:default}
     * @param strict If true, throws on undefined variables without defaults
     * @return Expanded string
     * @throws EnvVarError if strict=true and required variable is undefined
     */
    static std::string Expand(const std::string &value, bool strict = true) {
        std::string result;
        result.reserve(value.size());

        std::size_t i = 0;
        while (i < value.size()) {
            // Check for escape sequence $${ -> ${
            if (i + 2 < value.size() && value[i] == '$' &&
                value[i + 1] == '$' && value[i + 2] == '{') {
                result += "${";
                i += 3;
                continue;
            }

            // Check for variable ${VAR} or ${VAR:default}
            if (i + 1 < value.size() && value[i] == '$' &&
                value[i + 1] == '{') {
                std::size_t start = i + 2;
                std::size_t end = value.find('}', start);
                if (end == std::string::npos) {
                    // No closing brace, treat as literal
                    result += value[i];
                    ++i;
                    continue;
                }

                std::string content = value.substr(start, end - start);
                std::string var_name;
                std::string default_value;
                bool has_default = false;

                // Split on first colon for default value
                std::size_t colon_pos = content.find(':');
                if (colon_pos != std::string::npos) {
                    var_name = content.substr(0, colon_pos);
                    default_value = content.substr(colon_pos + 1);
                    has_default = true;
                } else {
                    var_name = content;
                }

                auto env_value = GetEnv(var_name);
                if (env_value.has_value()) {
                    result += env_value.value();
                } else if (has_default) {
                    result += default_value;
                } else if (strict) {
                    throw EnvVarError(var_name);
                }
                // If not strict and no default, variable is removed (empty)

                i = end + 1;
                continue;
            }

            // Regular character
            result += value[i];
            ++i;
        }

        return result;
    }

    /**
     * @brief Check if string contains environment variable references
     */
    static bool ContainsEnvVars(const std::string &value) {
        // Look for ${ followed eventually by }
        std::size_t pos = 0;
        while ((pos = value.find("${", pos)) != std::string::npos) {
            std::size_t end = value.find('}', pos + 2);
            if (end != std::string::npos) {
                return true;
            }
            pos += 2;
        }
        return false;
    }

    // =========================================================================
    // Node Expansion (Recursive)
    // =========================================================================

    /**
     * @brief Recursively expand all string values in a YAML tree
     *
     * @param node Root node to expand
     * @return New YamlNode with expanded values
     */
    static YamlNode ExpandAll(const YamlNode &node) {
        YAML::Node expanded = ExpandNodeRecursive(node.Raw());
        return YamlNode(expanded, node.Path());
    }

    // =========================================================================
    // File Loading with Expansion
    // =========================================================================

    /**
     * @brief Load YAML file with environment variable expansion
     *
     * Order of operations:
     * 1. Load file
     * 2. Expand envvars in all string values
     */
    static YamlNode LoadFileWithEnv(const std::string &path) {
        auto node = YamlNode::LoadFile(path);
        return ExpandAll(node);
    }

    /**
     * @brief Load with includes AND environment expansion
     *
     * Order of operations:
     * 1. Expand envvars in the path itself
     * 2. Resolve !include directives (with envvar expansion in paths)
     * 3. Expand envvars in all string values
     */
    static YamlNode LoadWithIncludesAndEnv(const std::string &path) {
        std::string expanded_path = Expand(path, true);
        auto node = YamlFile::LoadWithIncludes(expanded_path);
        return ExpandAll(node);
    }

    /**
     * @brief Expand envvars in path, then load the file
     */
    static YamlNode LoadFromEnvPath(const std::string &path_with_vars) {
        std::string expanded_path = Expand(path_with_vars, true);
        return YamlNode::LoadFile(expanded_path);
    }

    // =========================================================================
    // Environment Access
    // =========================================================================

    /**
     * @brief Get environment variable value
     *
     * @param name Variable name (without $)
     * @return Value if defined, nullopt otherwise
     */
    static std::optional<std::string> GetEnv(const std::string &name) {
        const char *value = std::getenv(name.c_str());
        if (value == nullptr) {
            return std::nullopt;
        }
        return std::string(value);
    }

    /**
     * @brief Get environment variable with default
     */
    static std::string GetEnv(const std::string &name,
                              const std::string &default_value) {
        return GetEnv(name).value_or(default_value);
    }

  private:
    /// Recursively expand environment variables in YAML nodes
    static YAML::Node ExpandNodeRecursive(const YAML::Node &node) {
        if (node.IsScalar()) {
            std::string value = node.as<std::string>();
            if (ContainsEnvVars(value)) {
                return YAML::Node(Expand(value, true));
            }
            return node;
        }

        if (node.IsSequence()) {
            YAML::Node result;
            for (std::size_t i = 0; i < node.size(); ++i) {
                result.push_back(ExpandNodeRecursive(node[i]));
            }
            return result;
        }

        if (node.IsMap()) {
            YAML::Node result;
            for (const auto &pair : node) {
                std::string key = pair.first.as<std::string>();
                // Expand envvars in keys too
                if (ContainsEnvVars(key)) {
                    key = Expand(key, true);
                }
                result[key] = ExpandNodeRecursive(pair.second);
            }
            return result;
        }

        return node;
    }
};

} // namespace vulcan::io
