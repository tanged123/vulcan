#pragma once

#include <vulcan/time/TimeConstants.hpp>
#include <vulcan/time/JulianDate.hpp>
#include <vulcan/time/TimeScales.hpp>
#include <janus/janus.hpp>
#include <cmath>

namespace vulcan::time {

// =============================================================================
// GPS Week Number Utilities
// =============================================================================

/**
 * @brief GPS week number rollover constant
 *
 * GPS week numbers roll over every 1024 weeks (~19.7 years).
 * First rollover: August 22, 1999
 * Second rollover: April 6, 2019
 */
inline constexpr int GPS_WEEK_ROLLOVER = 1024;

/**
 * @brief Current GPS week rollover count (post-April 2019)
 */
inline constexpr int GPS_CURRENT_ROLLOVER = 2;

/**
 * @brief Calculate full GPS week number from a potentially rolled-over week
 *
 * GPS receivers may report week numbers modulo 1024. This function
 * recovers the full week number based on expected rollover.
 *
 * @param week_mod Week number modulo 1024 (from receiver)
 * @param rollover_count Which rollover epoch (0, 1, 2, ...)
 * @return Full GPS week number
 */
[[nodiscard]] inline constexpr int full_gps_week(int week_mod, int rollover_count = GPS_CURRENT_ROLLOVER) {
    return week_mod + rollover_count * GPS_WEEK_ROLLOVER;
}

/**
 * @brief Convert GPS week/seconds to UTC Julian Date
 *
 * @param week Full GPS week number
 * @param seconds_of_week Seconds into the week (0 to 604799.999...)
 * @param delta_at TAI - UTC offset (default: 37 for post-2017)
 * @return Julian Date in UTC scale
 */
template <typename Scalar>
[[nodiscard]] Scalar gps_week_to_utc_jd(int week, const Scalar& seconds_of_week, int delta_at = 37) {
    // GPS time as JD
    Scalar gps_jd = constants::time::JD_GPS_EPOCH +
                    static_cast<double>(week) * 7.0 +
                    seconds_of_week / constants::time::SECONDS_PER_DAY;

    // Convert GPS JD to UTC JD
    return gps_to_utc(gps_jd, delta_at);
}

/**
 * @brief Convert GPS week/seconds to TAI Julian Date
 */
template <typename Scalar>
[[nodiscard]] Scalar gps_week_to_tai_jd(int week, const Scalar& seconds_of_week) {
    Scalar gps_jd = constants::time::JD_GPS_EPOCH +
                    static_cast<double>(week) * 7.0 +
                    seconds_of_week / constants::time::SECONDS_PER_DAY;

    return gps_to_tai(gps_jd);
}

/**
 * @brief Convert UTC Julian Date to GPS week and seconds
 *
 * @param utc_jd Julian Date in UTC scale
 * @param delta_at TAI - UTC offset
 * @return Pair of (week, seconds_of_week)
 */
[[nodiscard]] inline std::pair<int, double> utc_jd_to_gps_week(double utc_jd, int delta_at) {
    double gps_jd = utc_to_gps(utc_jd, delta_at);
    double days_since_epoch = gps_jd - constants::time::JD_GPS_EPOCH;

    int week = static_cast<int>(std::floor(days_since_epoch / 7.0));
    double days_into_week = days_since_epoch - week * 7.0;
    double seconds_of_week = days_into_week * constants::time::SECONDS_PER_DAY;

    return {week, seconds_of_week};
}

/**
 * @brief Convert UTC Julian Date to GPS week and seconds (auto leap second lookup)
 */
[[nodiscard]] inline std::pair<int, double> utc_jd_to_gps_week_auto(double utc_jd) {
    int delta_at = leap_seconds_at_utc(utc_jd);
    return utc_jd_to_gps_week(utc_jd, delta_at);
}

// =============================================================================
// GPS-UTC Offset
// =============================================================================

/**
 * @brief Get GPS - UTC offset for a given UTC Julian Date
 *
 * GPS tracks TAI with a fixed 19-second offset, while UTC has leap seconds.
 * GPS - UTC = (TAI - UTC) - 19 = delta_at - 19
 *
 * @param utc_jd Julian Date in UTC scale
 * @return GPS - UTC in seconds
 */
[[nodiscard]] inline int gps_utc_offset(double utc_jd) {
    int delta_at = leap_seconds_at_utc(utc_jd);
    return delta_at - static_cast<int>(constants::time::TAI_GPS_OFFSET);
}

/**
 * @brief Get GPS - UTC offset (templated with explicit delta_at)
 */
[[nodiscard]] inline constexpr int gps_utc_offset(int delta_at) {
    return delta_at - static_cast<int>(constants::time::TAI_GPS_OFFSET);
}

// =============================================================================
// Day of Week Utilities
// =============================================================================

/**
 * @brief GPS day of week enumeration
 *
 * GPS weeks start on Sunday (0).
 */
enum class GpsDayOfWeek {
    Sunday = 0,
    Monday = 1,
    Tuesday = 2,
    Wednesday = 3,
    Thursday = 4,
    Friday = 5,
    Saturday = 6
};

/**
 * @brief Get GPS day of week from seconds into the week
 */
[[nodiscard]] inline GpsDayOfWeek gps_day_of_week(double seconds_of_week) {
    int day = static_cast<int>(std::floor(seconds_of_week / constants::time::SECONDS_PER_DAY));
    return static_cast<GpsDayOfWeek>(day % 7);
}

/**
 * @brief Get time of day from GPS seconds of week
 *
 * @param seconds_of_week Seconds into GPS week
 * @return Seconds into the current day (0 to 86399.999...)
 */
[[nodiscard]] inline double gps_time_of_day(double seconds_of_week) {
    return std::fmod(seconds_of_week, constants::time::SECONDS_PER_DAY);
}

}  // namespace vulcan::time
