/**
 * @file YamlNode.hpp
 * @brief Type-safe YAML wrapper with path-aware error messages
 *
 * Provides ergonomic access to YAML configuration with:
 * - Type-safe extraction for common types
 * - Path tracking for debugging
 * - Support for Janus math types (Vec3, Mat3, Quaternion)
 */

#pragma once

#include <janus/core/JanusTypes.hpp>
#include <janus/math/Quaternion.hpp>
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace vulcan::io {

/**
 * @brief Exception for YAML parsing/access errors
 */
class YamlError : public std::runtime_error {
  public:
    YamlError(const std::string &path, const std::string &msg)
        : std::runtime_error("YAML error at '" + path + "': " + msg),
          path_(path) {}

    [[nodiscard]] const std::string &path() const { return path_; }

  private:
    std::string path_;
};

/**
 * @brief Wrapper around yaml-cpp node with ergonomic accessors
 *
 * Provides:
 * - Type-safe extraction with good error messages
 * - Path tracking for debugging
 * - Support for Janus math types
 */
class YamlNode {
  public:
    // =========================================================================
    // Construction
    // =========================================================================

    /// Load from file
    static YamlNode LoadFile(const std::string &path) {
        try {
            return YamlNode(YAML::LoadFile(path), path);
        } catch (const YAML::Exception &e) {
            throw YamlError(path, e.what());
        }
    }

    /// Parse from string
    static YamlNode Parse(const std::string &yaml_content) {
        try {
            return YamlNode(YAML::Load(yaml_content), "root");
        } catch (const YAML::Exception &e) {
            throw YamlError("root", e.what());
        }
    }

    /// Wrap an existing yaml-cpp node
    explicit YamlNode(YAML::Node node, std::string path = "root")
        : node_(std::move(node)), path_(std::move(path)) {}

    // =========================================================================
    // Navigation
    // =========================================================================

    /// Access child by key (map)
    [[nodiscard]] YamlNode operator[](const std::string &key) const {
        return YamlNode(node_[key], path_ + "." + key);
    }

    /// Access child by index (sequence)
    [[nodiscard]] YamlNode operator[](std::size_t index) const {
        return YamlNode(node_[index],
                        path_ + "[" + std::to_string(index) + "]");
    }

    /// Check if key exists
    [[nodiscard]] bool Has(const std::string &key) const {
        return node_[key].IsDefined();
    }

    /// Check if node is defined
    [[nodiscard]] bool IsDefined() const { return node_.IsDefined(); }

    /// Check if node is a sequence
    [[nodiscard]] bool IsSequence() const { return node_.IsSequence(); }

    /// Check if node is a map
    [[nodiscard]] bool IsMap() const { return node_.IsMap(); }

    /// Check if node is a scalar
    [[nodiscard]] bool IsScalar() const { return node_.IsScalar(); }

    /// Check if node is null
    [[nodiscard]] bool IsNull() const { return node_.IsNull(); }

    /// Get sequence size
    [[nodiscard]] std::size_t Size() const { return node_.size(); }

    /// Get current path (for error messages)
    [[nodiscard]] const std::string &Path() const { return path_; }

    // =========================================================================
    // Typed Accessors - Required (throws if missing)
    // =========================================================================

    /// Get required value (throws YamlError if missing or wrong type)
    template <typename T>
    [[nodiscard]] T Require(const std::string &key) const {
        auto child = (*this)[key];
        if (!child.IsDefined()) {
            throw YamlError(child.Path(), "required key not found");
        }
        return child.As<T>();
    }

    /// Get required value from current node
    template <typename T> [[nodiscard]] T As() const;

    // =========================================================================
    // Typed Accessors - Optional (returns default if missing)
    // =========================================================================

    /// Get optional value with default
    template <typename T>
    [[nodiscard]] T Get(const std::string &key, const T &default_value) const {
        auto child = (*this)[key];
        if (!child.IsDefined() || child.IsNull()) {
            return default_value;
        }
        return child.As<T>();
    }

    /// Get optional value (returns nullopt if missing)
    template <typename T>
    [[nodiscard]] std::optional<T> GetOptional(const std::string &key) const {
        auto child = (*this)[key];
        if (!child.IsDefined() || child.IsNull()) {
            return std::nullopt;
        }
        return child.As<T>();
    }

    // =========================================================================
    // Iteration
    // =========================================================================

    /// Iterate over sequence elements
    template <typename Func> void ForEach(Func &&func) const {
        if (!IsSequence()) {
            throw YamlError(path_, "expected sequence for iteration");
        }
        for (std::size_t i = 0; i < Size(); ++i) {
            func((*this)[i]);
        }
    }

    /// Iterate over map entries: func(key, node)
    template <typename Func> void ForEachEntry(Func &&func) const {
        if (!IsMap()) {
            throw YamlError(path_, "expected map for iteration");
        }
        for (const auto &pair : node_) {
            std::string key = pair.first.as<std::string>();
            func(key, YamlNode(pair.second, path_ + "." + key));
        }
    }

    /// Convert sequence to vector
    template <typename T> [[nodiscard]] std::vector<T> ToVector() const {
        if (!IsSequence()) {
            throw YamlError(path_, "expected sequence for ToVector");
        }
        std::vector<T> result;
        result.reserve(Size());
        for (std::size_t i = 0; i < Size(); ++i) {
            result.push_back((*this)[i].As<T>());
        }
        return result;
    }

    // =========================================================================
    // Raw Access
    // =========================================================================

    /// Get underlying yaml-cpp node
    [[nodiscard]] const YAML::Node &Raw() const { return node_; }

  private:
    YAML::Node node_;
    std::string path_;
};

// =============================================================================
// Template Specializations for Common Types
// =============================================================================

namespace detail {

template <typename T>
T yaml_as(const YAML::Node &node, const std::string &path) {
    try {
        return node.as<T>();
    } catch (const YAML::Exception &e) {
        throw YamlError(path, e.what());
    }
}

} // namespace detail

// --- Scalars ---
/// @brief Convert node to double
template <> inline double YamlNode::As<double>() const {
    return detail::yaml_as<double>(node_, path_);
}

/// @brief Convert node to float
template <> inline float YamlNode::As<float>() const {
    return detail::yaml_as<float>(node_, path_);
}

/// @brief Convert node to int
template <> inline int YamlNode::As<int>() const {
    return detail::yaml_as<int>(node_, path_);
}

/// @brief Convert node to int64_t
template <> inline int64_t YamlNode::As<int64_t>() const {
    return detail::yaml_as<int64_t>(node_, path_);
}

/// @brief Convert node to bool
template <> inline bool YamlNode::As<bool>() const {
    return detail::yaml_as<bool>(node_, path_);
}

/// @brief Convert node to string
template <> inline std::string YamlNode::As<std::string>() const {
    return detail::yaml_as<std::string>(node_, path_);
}

// --- Janus types (double specialization) ---

/// @brief Convert node to Vec3<double> from [x, y, z]
template <>
inline janus::Vec3<double> YamlNode::As<janus::Vec3<double>>() const {
    if (!IsSequence() || Size() != 3) {
        throw YamlError(path_, "expected sequence of 3 elements for Vec3");
    }
    return janus::Vec3<double>{(*this)[0].As<double>(), (*this)[1].As<double>(),
                               (*this)[2].As<double>()};
}

/// @brief Convert node to Quaternion<double> from [w, x, y, z]
template <>
inline janus::Quaternion<double>
YamlNode::As<janus::Quaternion<double>>() const {
    if (!IsSequence() || Size() != 4) {
        throw YamlError(
            path_, "expected sequence of 4 elements for Quaternion [w,x,y,z]");
    }
    // Scalar-first convention: [w, x, y, z]
    return janus::Quaternion<double>{
        (*this)[0].As<double>(), (*this)[1].As<double>(),
        (*this)[2].As<double>(), (*this)[3].As<double>()};
}

/// @brief Convert node to Mat3<double> from nested or flat array
template <>
inline janus::Mat3<double> YamlNode::As<janus::Mat3<double>>() const {
    if (!IsSequence()) {
        throw YamlError(path_, "expected sequence for Mat3");
    }

    janus::Mat3<double> m;

    // Nested format: [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]]
    if (Size() == 3 && (*this)[0].IsSequence()) {
        for (int i = 0; i < 3; ++i) {
            auto row = (*this)[static_cast<std::size_t>(i)];
            if (row.Size() != 3) {
                throw YamlError(path_, "Mat3 row " + std::to_string(i) +
                                           " must have 3 elements");
            }
            for (int j = 0; j < 3; ++j) {
                m(i, j) = row[static_cast<std::size_t>(j)].As<double>();
            }
        }
        return m;
    }

    // Flat format: [r00, r01, r02, r10, r11, r12, r20, r21, r22] (row-major)
    if (Size() == 9) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m(i, j) =
                    (*this)[static_cast<std::size_t>(i * 3 + j)].As<double>();
            }
        }
        return m;
    }

    throw YamlError(path_,
                    "Mat3 must be 3x3 nested array or 9-element flat array");
}

// --- Containers ---

/// @brief Convert node to vector of doubles
template <>
inline std::vector<double> YamlNode::As<std::vector<double>>() const {
    return ToVector<double>();
}

/// @brief Convert node to vector of ints
template <> inline std::vector<int> YamlNode::As<std::vector<int>>() const {
    return ToVector<int>();
}

/// @brief Convert node to vector of strings
template <>
inline std::vector<std::string> YamlNode::As<std::vector<std::string>>() const {
    return ToVector<std::string>();
}

} // namespace vulcan::io
