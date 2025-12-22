/**
 * @file FrameSerializer.hpp
 * @brief Binary frame serialization for Hermes real-time streaming
 *
 * Serializes Frame data to binary format for lock-free buffer transfer
 * to the Hermes middleware. Supports separate serialization of static
 * and dynamic signals for bandwidth optimization.
 */

#pragma once

#include <cstddef>
#include <cstring>
#include <span>
#include <vector>

#include <vulcan/core/VulcanError.hpp>
#include <vulcan/io/Frame.hpp>
#include <vulcan/io/TelemetrySchema.hpp>

namespace vulcan::io {

/**
 * @brief Binary frame serializer for Hermes real-time streaming
 *
 * Serializes Frame data to a compact binary format suitable for
 * lock-free buffer transfer and UDP streaming. Dynamic signals are
 * serialized for every frame; static signals are sent once during
 * schema negotiation.
 *
 * Binary format (little-endian):
 * @code
 * +------------------+------------------------+
 * | time (f64, 8B)   | signal_0 ... signal_N  |
 * +------------------+------------------------+
 * @endcode
 *
 * Example:
 * @code
 * FrameSerializer serializer(schema);
 *
 * Frame frame(schema);
 * frame.set_time(0.001);
 * frame.set("position", pos);
 *
 * // For 60Hz UDP telemetry (dynamic signals only)
 * auto bytes = serializer.serialize(frame);
 * udp_socket.send(bytes.data(), bytes.size());
 *
 * // For schema handshake (static signals only)
 * auto statics = serializer.serialize_statics(frame);
 * @endcode
 */
class FrameSerializer {
  public:
    /**
     * @brief Construct serializer for a schema
     * @param schema Telemetry schema (must outlive serializer)
     */
    explicit FrameSerializer(const TelemetrySchema &schema)
        : schema_(schema),
          dynamic_buffer_(8 + schema.dynamic_frame_size_bytes()),
          static_buffer_(schema.static_frame_size_bytes()) {
        // Build offset maps for dynamic and static signals
        size_t dynamic_offset = 8; // After time
        size_t static_offset = 0;

        for (const auto &sig : schema.signals()) {
            if (sig.lifecycle == SignalLifecycle::Dynamic) {
                dynamic_offsets_.push_back({sig.offset, dynamic_offset});
                dynamic_offset += 8;
            } else {
                static_offsets_.push_back({sig.offset, static_offset});
                static_offset += 8;
            }
        }
    }

    /**
     * @brief Serialize frame (time + dynamic signals) to binary
     *
     * Returns a span pointing to an internal buffer. The span is valid
     * until the next call to serialize() or serialize_statics().
     *
     * @param frame Source frame
     * @return Span of serialized bytes (little-endian)
     */
    std::span<const std::byte> serialize(const Frame &frame) {
        // Write time
        double time = frame.time();
        std::memcpy(dynamic_buffer_.data(), &time, sizeof(double));

        // Write dynamic signals
        for (const auto &[src_offset, dst_offset] : dynamic_offsets_) {
            std::memcpy(dynamic_buffer_.data() + dst_offset,
                        frame.data() + src_offset, 8);
        }

        return std::span<const std::byte>(dynamic_buffer_);
    }

    /**
     * @brief Serialize static signals only (for schema handshake)
     *
     * @param frame Source frame
     * @return Span of serialized bytes
     */
    std::span<const std::byte> serialize_statics(const Frame &frame) {
        for (const auto &[src_offset, dst_offset] : static_offsets_) {
            std::memcpy(static_buffer_.data() + dst_offset,
                        frame.data() + src_offset, 8);
        }
        return std::span<const std::byte>(static_buffer_);
    }

    /**
     * @brief Deserialize binary to frame (time + dynamic signals)
     *
     * @param data Binary data (from serialize())
     * @param frame Target frame to populate
     */
    void deserialize(std::span<const std::byte> data, Frame &frame) {
        if (data.size() < 8) {
            throw vulcan::SignalError("Invalid frame data: too small");
        }

        // Read time
        double time;
        std::memcpy(&time, data.data(), sizeof(double));
        frame.set_time(time);

        // Read dynamic signals
        for (const auto &[src_offset, dst_offset] : dynamic_offsets_) {
            if (dst_offset + 8 > data.size()) {
                throw vulcan::SignalError(
                    "Invalid frame data: incomplete signal data");
            }
            std::memcpy(frame.data() + src_offset, data.data() + dst_offset, 8);
        }
    }

    /**
     * @brief Deserialize static signals
     *
     * @param data Binary data (from serialize_statics())
     * @param frame Target frame to populate
     */
    void deserialize_statics(std::span<const std::byte> data, Frame &frame) {
        for (const auto &[src_offset, dst_offset] : static_offsets_) {
            if (dst_offset + 8 > data.size()) {
                throw vulcan::SignalError(
                    "Invalid static data: incomplete signal data");
            }
            std::memcpy(frame.data() + src_offset, data.data() + dst_offset, 8);
        }
    }

    /// Get dynamic frame size in bytes (time + dynamic signals)
    size_t dynamic_size_bytes() const { return dynamic_buffer_.size(); }

    /// Get static frame size in bytes
    size_t static_size_bytes() const { return static_buffer_.size(); }

    /// Get schema reference
    const TelemetrySchema &schema() const { return schema_; }

  private:
    TelemetrySchema schema_;
    std::vector<std::byte> dynamic_buffer_;
    std::vector<std::byte> static_buffer_;

    // Maps: (frame buffer offset, serialized buffer offset)
    std::vector<std::pair<size_t, size_t>> dynamic_offsets_;
    std::vector<std::pair<size_t, size_t>> static_offsets_;
};

} // namespace vulcan::io
