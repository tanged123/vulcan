// Vulcan Frame Identifiers
// Lightweight frame identity for the coordinate frame tree
#pragma once

#include <cstdint>

namespace vulcan {

/// Built-in frame identifiers.
enum class BuiltinFrame : uint16_t {
    ECI = 0,
    ECEF,
    NED,
    ENU,
    Body,
    Wind,
    Stability,
    Geocentric,
    Rail,
    CDA,
    _BuiltinCount
};

/// Universal frame identifier.
struct FrameID {
    uint32_t id;

    constexpr FrameID() : id(static_cast<uint32_t>(BuiltinFrame::ECI)) {}
    constexpr FrameID(
        BuiltinFrame builtin) // NOLINT(google-explicit-constructor)
        : id(static_cast<uint32_t>(builtin)) {}
    explicit constexpr FrameID(uint32_t raw) : id(raw) {}

    [[nodiscard]] constexpr bool is_builtin() const {
        return id < static_cast<uint32_t>(BuiltinFrame::_BuiltinCount);
    }

    [[nodiscard]] constexpr bool operator==(const FrameID &other) const {
        return id == other.id;
    }
    [[nodiscard]] constexpr bool operator!=(const FrameID &other) const {
        return id != other.id;
    }
    [[nodiscard]] constexpr bool operator<(const FrameID &other) const {
        return id < other.id;
    }
};

inline constexpr FrameID FRAME_ECI = BuiltinFrame::ECI;
inline constexpr FrameID FRAME_ECEF = BuiltinFrame::ECEF;
inline constexpr FrameID FRAME_NED = BuiltinFrame::NED;
inline constexpr FrameID FRAME_ENU = BuiltinFrame::ENU;
inline constexpr FrameID FRAME_BODY = BuiltinFrame::Body;
inline constexpr FrameID FRAME_WIND = BuiltinFrame::Wind;
inline constexpr FrameID FRAME_STABILITY = BuiltinFrame::Stability;
inline constexpr FrameID FRAME_GEOCENTRIC = BuiltinFrame::Geocentric;
inline constexpr FrameID FRAME_RAIL = BuiltinFrame::Rail;
inline constexpr FrameID FRAME_CDA = BuiltinFrame::CDA;

} // namespace vulcan
