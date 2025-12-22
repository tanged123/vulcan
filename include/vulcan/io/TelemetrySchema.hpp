/**
 * @file TelemetrySchema.hpp
 * @brief Telemetry signal schema definition for Vulcan I/O
 *
 * Defines the structure of telemetry data with scalar-only signals.
 * Supports double, int32, and int64 types with static/dynamic lifecycle.
 * Vectors are represented as multiple named scalars (e.g., position.x, .y, .z).
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <vulcan/core/VulcanError.hpp>
#include <vulcan/io/Signal.hpp>

namespace vulcan::io {

/**
 * @brief Telemetry schema builder and container
 *
 * Defines the structure of telemetry frames with named scalar signals.
 * All signals occupy 8-byte aligned slots for wire format compatibility.
 *
 * Example:
 * @code
 * TelemetrySchema schema;
 * schema.add_vec3("position", SignalLifecycle::Dynamic, "m");
 * schema.add_int32("gnc.phase", SignalLifecycle::Dynamic, "enum");
 * schema.add_double("mass.dry_mass", SignalLifecycle::Static, "kg");
 *
 * // Resulting signals: position.x, position.y, position.z, gnc.phase,
 * mass.dry_mass
 * @endcode
 */
class TelemetrySchema {
  public:
    TelemetrySchema() = default;

    // =========================================================================
    // Core signal registration (scalar only)
    // =========================================================================

    /**
     * @brief Add a double signal
     * @param name Signal name
     * @param lifecycle Static or Dynamic
     * @param unit Physical unit
     * @return Reference to this for chaining
     */
    TelemetrySchema &
    add_double(const std::string &name,
               SignalLifecycle lifecycle = SignalLifecycle::Dynamic,
               const std::string &unit = "") {
        add_signal({name, SignalType::Double, lifecycle, unit, "", 0});
        return *this;
    }

    /**
     * @brief Add an int32 signal
     * @param name Signal name
     * @param lifecycle Static or Dynamic
     * @param semantic Optional semantic hint ("boolean", "enum")
     * @return Reference to this for chaining
     */
    TelemetrySchema &
    add_int32(const std::string &name,
              SignalLifecycle lifecycle = SignalLifecycle::Dynamic,
              const std::string &semantic = "") {
        add_signal({name, SignalType::Int32, lifecycle, "", semantic, 0});
        return *this;
    }

    /**
     * @brief Add an int64 signal
     * @param name Signal name
     * @param lifecycle Static or Dynamic
     * @return Reference to this for chaining
     */
    TelemetrySchema &
    add_int64(const std::string &name,
              SignalLifecycle lifecycle = SignalLifecycle::Dynamic) {
        add_signal({name, SignalType::Int64, lifecycle, "", "", 0});
        return *this;
    }

    // =========================================================================
    // Convenience: vector expansion
    // =========================================================================

    /**
     * @brief Add a 3-component vector (expands to name.x, name.y, name.z)
     * @param name Base signal name
     * @param lifecycle Static or Dynamic
     * @param unit Physical unit
     * @return Reference to this for chaining
     */
    TelemetrySchema &
    add_vec3(const std::string &name,
             SignalLifecycle lifecycle = SignalLifecycle::Dynamic,
             const std::string &unit = "") {
        add_double(name + ".x", lifecycle, unit);
        add_double(name + ".y", lifecycle, unit);
        add_double(name + ".z", lifecycle, unit);
        return *this;
    }

    /**
     * @brief Add a quaternion (expands to name.w, name.x, name.y, name.z)
     * @param name Base signal name
     * @param lifecycle Static or Dynamic
     * @return Reference to this for chaining
     */
    TelemetrySchema &
    add_quat(const std::string &name,
             SignalLifecycle lifecycle = SignalLifecycle::Dynamic) {
        add_double(name + ".w", lifecycle);
        add_double(name + ".x", lifecycle);
        add_double(name + ".y", lifecycle);
        add_double(name + ".z", lifecycle);
        return *this;
    }

    // =========================================================================
    // Access
    // =========================================================================

    /// Get all signals
    const std::vector<SignalDescriptor> &signals() const { return signals_; }

    /// Get only dynamic signals
    std::vector<SignalDescriptor> dynamic_signals() const {
        std::vector<SignalDescriptor> result;
        for (const auto &s : signals_) {
            if (s.lifecycle == SignalLifecycle::Dynamic) {
                result.push_back(s);
            }
        }
        return result;
    }

    /// Get only static signals
    std::vector<SignalDescriptor> static_signals() const {
        std::vector<SignalDescriptor> result;
        for (const auto &s : signals_) {
            if (s.lifecycle == SignalLifecycle::Static) {
                result.push_back(s);
            }
        }
        return result;
    }

    /// Get signal by name
    const SignalDescriptor &signal(const std::string &name) const {
        auto it = index_.find(name);
        if (it == index_.end()) {
            throw vulcan::SignalError("Signal not found: " + name);
        }
        return signals_[it->second];
    }

    /// Check if signal exists
    bool has_signal(const std::string &name) const {
        return index_.find(name) != index_.end();
    }

    /// Get offset for signal
    size_t offset(const std::string &name) const { return signal(name).offset; }

    /// Get index for signal
    size_t index(const std::string &name) const {
        auto it = index_.find(name);
        if (it == index_.end()) {
            throw vulcan::SignalError("Signal not found: " + name);
        }
        return it->second;
    }

    /// Total frame size in bytes (all signals)
    size_t frame_size_bytes() const { return next_offset_; }

    /// Dynamic signals frame size in bytes (for streaming)
    size_t dynamic_frame_size_bytes() const {
        size_t size = 0;
        for (const auto &s : signals_) {
            if (s.lifecycle == SignalLifecycle::Dynamic) {
                size += s.size_bytes();
            }
        }
        return size;
    }

    /// Static signals frame size in bytes
    size_t static_frame_size_bytes() const {
        size_t size = 0;
        for (const auto &s : signals_) {
            if (s.lifecycle == SignalLifecycle::Static) {
                size += s.size_bytes();
            }
        }
        return size;
    }

    /// Number of signals
    size_t signal_count() const { return signals_.size(); }

    /// Get all signal names
    std::vector<std::string> signal_names() const {
        std::vector<std::string> names;
        names.reserve(signals_.size());
        for (const auto &s : signals_) {
            names.push_back(s.name);
        }
        return names;
    }

    // =========================================================================
    // Validation
    // =========================================================================

    /// Validate schema (throws on errors)
    void validate() const {
        // Check for empty schema
        if (signals_.empty()) {
            throw vulcan::SignalError("Schema has no signals");
        }
        // Duplicate names are already caught during add_signal
    }

    // =========================================================================
    // JSON serialization
    // =========================================================================

    /// Serialize to JSON string
    std::string to_json() const {
        std::string json = "{\n  \"signals\": [\n";
        for (size_t i = 0; i < signals_.size(); ++i) {
            const auto &s = signals_[i];
            json += "    {\n";
            json += "      \"name\": \"" + s.name + "\",\n";
            json += "      \"type\": \"" + type_to_string(s.type) + "\",\n";
            json += "      \"lifecycle\": \"" +
                    lifecycle_to_string(s.lifecycle) + "\",\n";
            json += "      \"offset\": " + std::to_string(s.offset);
            if (!s.unit.empty()) {
                json += ",\n      \"unit\": \"" + s.unit + "\"";
            }
            if (!s.semantic.empty()) {
                json += ",\n      \"semantic\": \"" + s.semantic + "\"";
            }
            json += "\n    }";
            if (i + 1 < signals_.size()) {
                json += ",";
            }
            json += "\n";
        }
        json += "  ],\n";
        json +=
            "  \"frame_size_bytes\": " + std::to_string(next_offset_) + "\n";
        json += "}";
        return json;
    }

    /// Deserialize from JSON string (basic parser)
    static TelemetrySchema from_json(const std::string &json);

  private:
    std::vector<SignalDescriptor> signals_;
    std::unordered_map<std::string, size_t> index_;
    size_t next_offset_ = 0;

    void add_signal(SignalDescriptor desc) {
        if (index_.find(desc.name) != index_.end()) {
            throw vulcan::SignalError("Duplicate signal name: " + desc.name);
        }
        desc.offset = next_offset_;
        next_offset_ += 8; // All signals are 8-byte aligned
        index_[desc.name] = signals_.size();
        signals_.push_back(std::move(desc));
    }

    static std::string type_to_string(SignalType type) {
        switch (type) {
        case SignalType::Double:
            return "double";
        case SignalType::Int32:
            return "int32";
        case SignalType::Int64:
            return "int64";
        }
        return "unknown";
    }

    static std::string lifecycle_to_string(SignalLifecycle lc) {
        switch (lc) {
        case SignalLifecycle::Static:
            return "static";
        case SignalLifecycle::Dynamic:
            return "dynamic";
        }
        return "unknown";
    }
};

} // namespace vulcan::io
