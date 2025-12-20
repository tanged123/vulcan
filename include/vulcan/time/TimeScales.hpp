#pragma once

#include <cmath>
#include <janus/janus.hpp>
#include <vulcan/time/JulianDate.hpp>
#include <vulcan/time/LeapSeconds.hpp>
#include <vulcan/time/TimeConstants.hpp>

namespace vulcan::time {

// =============================================================================
// Enumeration of Time Scales
// =============================================================================

enum class TimeScale {
    UTC, ///< Coordinated Universal Time (civil time, has leap seconds)
    TAI, ///< International Atomic Time (continuous SI seconds)
    TT,  ///< Terrestrial Time (TT = TAI + 32.184s)
    TDB, ///< Barycentric Dynamical Time (for solar system dynamics)
    GPS, ///< GPS Time (GPS = TAI - 19s, continuous since 1980)
    UT1  ///< Universal Time 1 (Earth rotation, requires IERS data)
};

// =============================================================================
// TAI ↔ TT Conversions (Templated - Fixed Offset)
// =============================================================================

/**
 * @brief Convert TAI Julian Date to TT Julian Date
 *
 * TT = TAI + 32.184 seconds (exact, by definition)
 *
 * @tparam Scalar Numeric or symbolic type
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar tai_to_tt(const Scalar &tai_jd) {
    return tai_jd +
           constants::time::TT_TAI_OFFSET / constants::time::SECONDS_PER_DAY;
}

/**
 * @brief Convert TT Julian Date to TAI Julian Date
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar tt_to_tai(const Scalar &tt_jd) {
    return tt_jd -
           constants::time::TT_TAI_OFFSET / constants::time::SECONDS_PER_DAY;
}

// =============================================================================
// TAI ↔ GPS Conversions (Templated - Fixed Offset)
// =============================================================================

/**
 * @brief Convert TAI Julian Date to GPS Julian Date
 *
 * GPS = TAI - 19 seconds (exact, GPS was synchronized with TAI at epoch)
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar tai_to_gps(const Scalar &tai_jd) {
    return tai_jd +
           constants::time::GPS_TAI_OFFSET / constants::time::SECONDS_PER_DAY;
}

/**
 * @brief Convert GPS Julian Date to TAI Julian Date
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar gps_to_tai(const Scalar &gps_jd) {
    return gps_jd +
           constants::time::TAI_GPS_OFFSET / constants::time::SECONDS_PER_DAY;
}

// =============================================================================
// TAI ↔ UTC Conversions (Templated with Leap Second Parameter)
// =============================================================================

/**
 * @brief Convert UTC Julian Date to TAI Julian Date (templated)
 *
 * @tparam Scalar Numeric or symbolic type
 * @param utc_jd Julian Date in UTC scale
 * @param delta_at TAI - UTC offset in seconds (from leap second table)
 * @return Julian Date in TAI scale
 *
 * @note For symbolic mode, user provides delta_at as constant for their epoch
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar utc_to_tai(const Scalar &utc_jd, int delta_at) {
    return utc_jd +
           static_cast<double>(delta_at) / constants::time::SECONDS_PER_DAY;
}

/**
 * @brief Convert TAI Julian Date to UTC Julian Date (templated)
 *
 * @tparam Scalar Numeric or symbolic type
 * @param tai_jd Julian Date in TAI scale
 * @param delta_at TAI - UTC offset in seconds
 * @return Julian Date in UTC scale
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar tai_to_utc(const Scalar &tai_jd, int delta_at) {
    return tai_jd -
           static_cast<double>(delta_at) / constants::time::SECONDS_PER_DAY;
}

// =============================================================================
// TAI ↔ UTC Conversions (Numeric with Automatic Lookup)
// =============================================================================

/**
 * @brief Convert UTC Julian Date to TAI Julian Date (numeric, auto lookup)
 *
 * Automatically looks up leap seconds from the table.
 */
[[nodiscard]] inline double utc_to_tai(double utc_jd) {
    int delta_at = leap_seconds_at_utc(utc_jd);
    return utc_to_tai(utc_jd, delta_at);
}

/**
 * @brief Convert TAI Julian Date to UTC Julian Date (numeric, auto lookup)
 */
[[nodiscard]] inline double tai_to_utc(double tai_jd) {
    int delta_at = leap_seconds_at_tai(tai_jd);
    return tai_to_utc(tai_jd, delta_at);
}

// =============================================================================
// UTC ↔ GPS Conversions (Convenience)
// =============================================================================

/**
 * @brief Convert UTC Julian Date to GPS Julian Date (templated)
 *
 * GPS = TAI - 19s, so GPS = UTC + (delta_at - 19)
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar utc_to_gps(const Scalar &utc_jd, int delta_at) {
    Scalar tai_jd = utc_to_tai(utc_jd, delta_at);
    return tai_to_gps(tai_jd);
}

/**
 * @brief Convert GPS Julian Date to UTC Julian Date (templated)
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar gps_to_utc(const Scalar &gps_jd, int delta_at) {
    Scalar tai_jd = gps_to_tai(gps_jd);
    return tai_to_utc(tai_jd, delta_at);
}

/**
 * @brief Convert UTC Julian Date to GPS Julian Date (numeric, auto lookup)
 */
[[nodiscard]] inline double utc_to_gps(double utc_jd) {
    int delta_at = leap_seconds_at_utc(utc_jd);
    return utc_to_gps(utc_jd, delta_at);
}

/**
 * @brief Convert GPS Julian Date to UTC Julian Date (numeric, auto lookup)
 */
[[nodiscard]] inline double gps_to_utc(double gps_jd) {
    // GPS JD to TAI JD, then lookup
    double tai_jd = gps_to_tai(gps_jd);
    int delta_at = leap_seconds_at_tai(tai_jd);
    return gps_to_utc(gps_jd, delta_at);
}

// =============================================================================
// TT ↔ TDB Conversions (Templated - Periodic Approximation)
// =============================================================================

/**
 * @brief Convert TT Julian Date to TDB Julian Date
 *
 * TDB = TT + periodic terms due to Earth's orbital eccentricity.
 * Uses the simplified formula from Fairhead & Bretagnon (1990):
 *   TDB - TT ≈ 0.001657 * sin(g) + 0.000022 * sin(L - D)
 *
 * where g is the mean anomaly of Earth's orbit.
 * Accuracy: ~2 ms (sufficient for most applications)
 *
 * @tparam Scalar Numeric or symbolic type
 */
template <typename Scalar> [[nodiscard]] Scalar tt_to_tdb(const Scalar &tt_jd) {
    // Julian centuries since J2000.0 TT
    Scalar T = jd_to_j2000_centuries(tt_jd);

    // Mean anomaly of Earth around Sun (radians)
    // g = 357.5277233° + 35999.05034° * T
    Scalar g = (357.5277233 + 35999.05034 * T) * M_PI / 180.0;

    // TDB - TT in seconds (simplified Fairhead & Bretagnon)
    Scalar dt = 0.001657 * janus::sin(g);

    return tt_jd + dt / constants::time::SECONDS_PER_DAY;
}

/**
 * @brief Convert TDB Julian Date to TT Julian Date
 *
 * Inverse of tt_to_tdb. Since the correction is small (~1.7 ms max),
 * a single iteration is sufficient.
 */
template <typename Scalar>
[[nodiscard]] Scalar tdb_to_tt(const Scalar &tdb_jd) {
    // First approximation: TT ≈ TDB
    Scalar T = jd_to_j2000_centuries(tdb_jd);

    // Mean anomaly
    Scalar g = (357.5277233 + 35999.05034 * T) * M_PI / 180.0;

    // Correction
    Scalar dt = 0.001657 * janus::sin(g);

    return tdb_jd - dt / constants::time::SECONDS_PER_DAY;
}

// =============================================================================
// Symbolic UTC ↔ TAI with Interpolated Leap Seconds
// =============================================================================

/**
 * @brief Convert UTC Julian Date to TAI Julian Date (fully symbolic)
 *
 * Uses janus::Interpolator for smooth leap second lookup.
 * Suitable for optimization problems spanning leap second boundaries.
 *
 * @tparam Scalar Numeric or symbolic type
 * @param utc_jd Julian Date in UTC scale
 * @return Julian Date in TAI scale (with smooth leap second approximation)
 */
template <typename Scalar>
[[nodiscard]] Scalar utc_to_tai_symbolic(const Scalar &utc_jd) {
    Scalar delta_at = leap_seconds_symbolic(utc_jd);
    return utc_jd + delta_at / constants::time::SECONDS_PER_DAY;
}

/**
 * @brief Convert TAI Julian Date to UTC Julian Date (fully symbolic)
 *
 * Uses janus::Interpolator for smooth leap second lookup.
 * Note: For TAI→UTC, we approximate by using the TAI JD directly
 * in the interpolator (small error during leap second transitions).
 */
template <typename Scalar>
[[nodiscard]] Scalar tai_to_utc_symbolic(const Scalar &tai_jd) {
    // Approximate: use TAI JD in interpolator (error < 37 seconds offset)
    // This is acceptable for smooth optimization purposes
    Scalar delta_at = leap_seconds_symbolic(tai_jd);
    return tai_jd - delta_at / constants::time::SECONDS_PER_DAY;
}

} // namespace vulcan::time
