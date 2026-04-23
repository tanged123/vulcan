/**
 * @file YamlConvert.hpp
 * @brief yaml-cpp conversion traits for Metis/Vulcan types
 *
 * Provides YAML::convert<> specializations for:
 * - metis::Vec3<T> stored as [x, y, z]
 * - metis::Quaternion<T> stored as [w, x, y, z] (scalar-first)
 * - metis::Mat3<T> stored as nested or flat array
 */

#pragma once

#include <metis/core/MetisTypes.hpp>
#include <metis/math/Quaternion.hpp>
#include <yaml-cpp/yaml.h>

namespace YAML {

// =============================================================================
// metis::Vec3<T> - stored as [x, y, z]
// =============================================================================

/// @brief Conversion trait for metis::Vec3<T>
template <typename T> struct convert<metis::Vec3<T>> {
    /// @brief Encode Vec3 to YAML sequence [x, y, z]
    static Node encode(const metis::Vec3<T> &v) {
        Node node;
        node.push_back(v.x());
        node.push_back(v.y());
        node.push_back(v.z());
        return node;
    }

    /// @brief Decode YAML sequence to Vec3
    static bool decode(const Node &node, metis::Vec3<T> &v) {
        if (!node.IsSequence() || node.size() != 3) {
            return false;
        }
        try {
            v = metis::Vec3<T>{node[0].as<T>(), node[1].as<T>(),
                               node[2].as<T>()};
            return true;
        } catch (const YAML::Exception &) {
            return false;
        }
    }
};

// =============================================================================
// metis::Quaternion<T> - stored as [w, x, y, z] (scalar-first)
// =============================================================================

/// @brief Conversion trait for metis::Quaternion<T>
template <typename T> struct convert<metis::Quaternion<T>> {
    /// @brief Encode Quaternion to YAML sequence [w, x, y, z]
    static Node encode(const metis::Quaternion<T> &q) {
        Node node;
        node.push_back(q.w);
        node.push_back(q.x);
        node.push_back(q.y);
        node.push_back(q.z);
        return node;
    }

    /// @brief Decode YAML sequence to Quaternion
    static bool decode(const Node &node, metis::Quaternion<T> &q) {
        if (!node.IsSequence() || node.size() != 4) {
            return false;
        }
        try {
            q = metis::Quaternion<T>{node[0].as<T>(),  // w
                                     node[1].as<T>(),  // x
                                     node[2].as<T>(),  // y
                                     node[3].as<T>()}; // z
            return true;
        } catch (const YAML::Exception &) {
            return false;
        }
    }
};

// =============================================================================
// metis::Mat3<T> - stored as [[row0], [row1], [row2]] or flat [9 elements]
// =============================================================================

/// @brief Conversion trait for metis::Mat3<T>
template <typename T> struct convert<metis::Mat3<T>> {
    /// @brief Encode Mat3 to nested YAML sequence
    static Node encode(const metis::Mat3<T> &m) {
        Node node;
        for (int i = 0; i < 3; ++i) {
            Node row;
            for (int j = 0; j < 3; ++j) {
                row.push_back(m(i, j));
            }
            node.push_back(row);
        }
        return node;
    }

    /// @brief Decode YAML sequence to Mat3 (nested or flat)
    static bool decode(const Node &node, metis::Mat3<T> &m) {
        if (!node.IsSequence()) {
            return false;
        }

        try {
            // Nested format: [[r00, r01, r02], [r10, r11, r12], [r20, r21,
            // r22]]
            if (node.size() == 3 && node[0].IsSequence()) {
                for (int i = 0; i < 3; ++i) {
                    if (node[i].size() != 3) {
                        return false;
                    }
                    for (int j = 0; j < 3; ++j) {
                        m(i, j) = node[i][j].as<T>();
                    }
                }
                return true;
            }

            // Flat format: [r00, r01, r02, r10, r11, r12, r20, r21, r22]
            // (row-major)
            if (node.size() == 9) {
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        m(i, j) = node[i * 3 + j].as<T>();
                    }
                }
                return true;
            }
        } catch (const YAML::Exception &) {
            return false;
        }

        return false;
    }
};

// =============================================================================
// Emitter operators for Metis types (required for YAML::Emitter << type)
// =============================================================================

/// @brief Emit Vec3 to YAML stream
template <typename T>
inline Emitter &operator<<(Emitter &emitter, const metis::Vec3<T> &v) {
    emitter << YAML::Flow << YAML::BeginSeq;
    emitter << v.x() << v.y() << v.z();
    emitter << YAML::EndSeq;
    return emitter;
}

/// @brief Emit Quaternion to YAML stream
template <typename T>
inline Emitter &operator<<(Emitter &emitter, const metis::Quaternion<T> &q) {
    emitter << YAML::Flow << YAML::BeginSeq;
    emitter << q.w << q.x << q.y << q.z;
    emitter << YAML::EndSeq;
    return emitter;
}

/// @brief Emit Mat3 to YAML stream
template <typename T>
inline Emitter &operator<<(Emitter &emitter, const metis::Mat3<T> &m) {
    emitter << YAML::BeginSeq;
    for (int i = 0; i < 3; ++i) {
        emitter << YAML::Flow << YAML::BeginSeq;
        emitter << m(i, 0) << m(i, 1) << m(i, 2);
        emitter << YAML::EndSeq;
    }
    emitter << YAML::EndSeq;
    return emitter;
}

} // namespace YAML
