/**
 * @file Frame.hpp
 * @brief Telemetry frame data container for Vulcan I/O
 *
 * Holds a single timestep of telemetry data with scalar signals.
 * Used for both HDF5 logging and real-time streaming to Hermes.
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <vulcan/io/TelemetrySchema.hpp>

namespace vulcan::io {

/**
 * @brief Single timestep of telemetry data
 *
 * Contains scalar signal values in an 8-byte aligned buffer.
 * Provides typed setters/getters and raw access for serialization.
 *
 * Example:
 * @code
 * Frame frame(schema);
 * frame.set_time(0.001);
 * frame.set("position", Eigen::Vector3d(1, 2, 3));  // Sets .x, .y, .z
 * frame.set("gnc.phase", int32_t{2});
 *
 * auto pos = frame.get_vec3("position");  // Reads .x, .y, .z
 * @endcode
 */
class Frame {
  public:
    /**
     * @brief Construct frame with schema
     * @param schema Reference to telemetry schema (must outlive Frame)
     */
    explicit Frame(const TelemetrySchema &schema)
        : schema_(&schema), time_(0.0),
          buffer_(schema.frame_size_bytes(), std::byte{0}) {}

    // =========================================================================
    // Time
    // =========================================================================

    /// Set frame timestamp
    void set_time(double t) { time_ = t; }

    /// Get frame timestamp
    double time() const { return time_; }

    // =========================================================================
    // Scalar setters
    // =========================================================================

    /// Set double signal value
    void set(const std::string &signal, double value) {
        const auto &desc = schema_->signal(signal);
        if (desc.type != SignalType::Double) {
            throw std::runtime_error("Type mismatch for signal: " + signal);
        }
        write_at<double>(desc.offset, value);
    }

    /// Set int32 signal value
    void set(const std::string &signal, int32_t value) {
        const auto &desc = schema_->signal(signal);
        if (desc.type != SignalType::Int32) {
            throw std::runtime_error("Type mismatch for signal: " + signal);
        }
        // Store as int32 in 8-byte slot (4 bytes value + 4 bytes padding)
        write_at<int64_t>(desc.offset, static_cast<int64_t>(value));
    }

    /// Set int64 signal value
    void set(const std::string &signal, int64_t value) {
        const auto &desc = schema_->signal(signal);
        if (desc.type != SignalType::Int64) {
            throw std::runtime_error("Type mismatch for signal: " + signal);
        }
        write_at<int64_t>(desc.offset, value);
    }

    // =========================================================================
    // Convenience setters: vectors
    // =========================================================================

    /// Set 3-component vector (expands to signal.x, signal.y, signal.z)
    void set(const std::string &signal, const Eigen::Vector3d &v) {
        set(signal + ".x", v.x());
        set(signal + ".y", v.y());
        set(signal + ".z", v.z());
    }

    /// Set 4-component vector (expands to signal.w, signal.x, signal.y,
    /// signal.z) Used for quaternions
    void set(const std::string &signal, const Eigen::Vector4d &v) {
        set(signal + ".w", v.w());
        set(signal + ".x", v.x());
        set(signal + ".y", v.y());
        set(signal + ".z", v.z());
    }

    /// Set quaternion (expands to signal.w, signal.x, signal.y, signal.z)
    void set(const std::string &signal, const Eigen::Quaterniond &q) {
        set(signal + ".w", q.w());
        set(signal + ".x", q.x());
        set(signal + ".y", q.y());
        set(signal + ".z", q.z());
    }

    // =========================================================================
    // Scalar getters
    // =========================================================================

    /// Get double signal value
    double get_double(const std::string &signal) const {
        const auto &desc = schema_->signal(signal);
        if (desc.type != SignalType::Double) {
            throw std::runtime_error("Type mismatch for signal: " + signal);
        }
        return read_at<double>(desc.offset);
    }

    /// Get int32 signal value
    int32_t get_int32(const std::string &signal) const {
        const auto &desc = schema_->signal(signal);
        if (desc.type != SignalType::Int32) {
            throw std::runtime_error("Type mismatch for signal: " + signal);
        }
        return static_cast<int32_t>(read_at<int64_t>(desc.offset));
    }

    /// Get int64 signal value
    int64_t get_int64(const std::string &signal) const {
        const auto &desc = schema_->signal(signal);
        if (desc.type != SignalType::Int64) {
            throw std::runtime_error("Type mismatch for signal: " + signal);
        }
        return read_at<int64_t>(desc.offset);
    }

    // =========================================================================
    // Convenience getters: vectors
    // =========================================================================

    /// Get 3-component vector from signal.x, signal.y, signal.z
    Eigen::Vector3d get_vec3(const std::string &signal) const {
        return Eigen::Vector3d(get_double(signal + ".x"),
                               get_double(signal + ".y"),
                               get_double(signal + ".z"));
    }

    /// Get quaternion from signal.w, signal.x, signal.y, signal.z
    Eigen::Quaterniond get_quat(const std::string &signal) const {
        return Eigen::Quaterniond(
            get_double(signal + ".w"), get_double(signal + ".x"),
            get_double(signal + ".y"), get_double(signal + ".z"));
    }

    /// Get 4-component vector from signal.w, signal.x, signal.y, signal.z
    Eigen::Vector4d get_vec4(const std::string &signal) const {
        return Eigen::Vector4d(
            get_double(signal + ".w"), get_double(signal + ".x"),
            get_double(signal + ".y"), get_double(signal + ".z"));
    }

    // =========================================================================
    // Raw access (for serialization)
    // =========================================================================

    /// Get raw buffer pointer (const)
    const std::byte *data() const { return buffer_.data(); }

    /// Get raw buffer pointer (mutable)
    std::byte *data() { return buffer_.data(); }

    /// Get buffer size in bytes
    size_t size_bytes() const { return buffer_.size(); }

    /// Get schema reference
    const TelemetrySchema &schema() const { return *schema_; }

    /// Clear all values to zero
    void clear() {
        time_ = 0.0;
        std::fill(buffer_.begin(), buffer_.end(), std::byte{0});
    }

  private:
    const TelemetrySchema *schema_;
    double time_;
    std::vector<std::byte> buffer_;

    template <typename T> void write_at(size_t offset, T value) {
        std::memcpy(buffer_.data() + offset, &value, sizeof(T));
    }

    template <typename T> T read_at(size_t offset) const {
        T value;
        std::memcpy(&value, buffer_.data() + offset, sizeof(T));
        return value;
    }
};

} // namespace vulcan::io
