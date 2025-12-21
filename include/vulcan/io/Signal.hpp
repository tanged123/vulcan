/**
 * @file Signal.hpp
 * @brief Core signal types for telemetry and streaming
 *
 * Defines the fundamental signal primitives used by both Vulcan's
 * TelemetrySchema and the Icarus Protocol for real-time data streaming.
 * All signals use 8-byte aligned wire format for efficiency.
 */

#pragma once

#include <cstddef>
#include <string>

namespace vulcan::io {

/// Signal data type (all scalar, 8-byte aligned slots)
enum class SignalType {
    Double, ///< 8 bytes - physical quantities
    Int32,  ///< 4 bytes + 4 padding - modes, phases, counters
    Int64   ///< 8 bytes - timestamps, large counters
};

/// Signal lifecycle for telemetry optimization
enum class SignalLifecycle {
    Static, ///< Constant after init, sent in schema handshake only
    Dynamic ///< Updated each step, included in streaming telemetry
};

/// Individual signal descriptor
struct SignalDescriptor {
    std::string name;          ///< Signal name (e.g., "position.x")
    SignalType type;           ///< Data type
    SignalLifecycle lifecycle; ///< Static or dynamic
    std::string unit;          ///< Physical unit (e.g., "m", "rad/s")
    std::string semantic;      ///< Optional semantic hint ("boolean", "enum")
    size_t offset;             ///< Byte offset in frame buffer

    /// Get size in bytes (always 8 for alignment)
    constexpr size_t size_bytes() const { return 8; }
};

} // namespace vulcan::io
