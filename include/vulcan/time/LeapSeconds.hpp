#pragma once

#include <array>
#include <janus/math/Interpolate.hpp>
#include <vulcan/time/JulianDate.hpp>
#include <vulcan/time/TimeConstants.hpp>

namespace vulcan::time {

// =============================================================================
// Leap Second Table (IERS Bulletin C)
// =============================================================================

/**
 * @brief Leap second entry: date and cumulative offset
 *
 * Each entry represents the start of a period where TAI - UTC equals delta_at.
 * The date is when the leap second took effect (00:00:00 UTC).
 */
struct LeapSecondEntry {
    int year;
    int month;
    int day;
    int delta_at; ///< TAI - UTC [seconds]
};

/**
 * @brief Complete leap second table from 1972 to present
 *
 * Source: IERS Bulletin C (https://hpiers.obspm.fr/iers/bul/bulc/bulletinc.dat)
 * Last updated: Bulletin C 69 (July 2024) - no leap second through June 2025
 *
 * @note Before 1972, UTC used fractional offsets (not supported here).
 *       For pre-1972 dates, use TAI = UTC + 10s as approximation.
 */
inline constexpr std::array<LeapSecondEntry, 28> LEAP_SECOND_TABLE = {{
    // Year, Month, Day, TAI-UTC
    {1972, 1, 1, 10}, // Initial offset when leap seconds began
    {1972, 7, 1, 11}, {1973, 1, 1, 12}, {1974, 1, 1, 13}, {1975, 1, 1, 14},
    {1976, 1, 1, 15}, {1977, 1, 1, 16}, {1978, 1, 1, 17}, {1979, 1, 1, 18},
    {1980, 1, 1, 19}, {1981, 7, 1, 20}, {1982, 7, 1, 21}, {1983, 7, 1, 22},
    {1985, 7, 1, 23}, {1988, 1, 1, 24}, {1990, 1, 1, 25}, {1991, 1, 1, 26},
    {1992, 7, 1, 27}, {1993, 7, 1, 28}, {1994, 7, 1, 29}, {1996, 1, 1, 30},
    {1997, 7, 1, 31}, {1999, 1, 1, 32}, {2006, 1, 1, 33}, {2009, 1, 1, 34},
    {2012, 7, 1, 35}, {2015, 7, 1, 36}, {2017, 1, 1, 37}, // Most recent leap
                                                          // second
}};

/**
 * @brief Get TAI - UTC offset (leap seconds) for a given TAI Julian Date
 *
 * @param tai_jd Julian Date in TAI scale
 * @return TAI - UTC in integer seconds
 *
 * @note Returns 10 for dates before 1972-01-01 (approximation)
 * @note Returns latest value (37) for dates after the table
 */
[[nodiscard]] inline int leap_seconds_at_tai(double tai_jd) {
    // Before 1972: use 10 seconds as approximation
    double jd_1972 = calendar_to_jd(1972, 1, 1, 0, 0, 0.0) +
                     10.0 / constants::time::SECONDS_PER_DAY;
    if (tai_jd < jd_1972) {
        return 10;
    }

    // Search backwards through table (most queries will be recent dates)
    for (int i = static_cast<int>(LEAP_SECOND_TABLE.size()) - 1; i >= 0; --i) {
        const auto &entry = LEAP_SECOND_TABLE[i];
        // Convert entry date to TAI JD
        // Entry is when delta_at takes effect at 00:00:00 UTC
        // In TAI, this is 00:00:00 UTC + delta_at seconds
        double entry_tai_jd =
            calendar_to_jd(entry.year, entry.month, entry.day, 0, 0, 0.0) +
            entry.delta_at / constants::time::SECONDS_PER_DAY;

        if (tai_jd >= entry_tai_jd) {
            return entry.delta_at;
        }
    }

    return 10; // Fallback (shouldn't reach for valid inputs)
}

/**
 * @brief Get TAI - UTC offset for a given UTC Julian Date
 *
 * @param utc_jd Julian Date in UTC scale
 * @return TAI - UTC in integer seconds
 *
 * @warning During a leap second (23:59:60 UTC), this function returns
 *          the new offset. The leap second itself is ambiguous in JD.
 */
[[nodiscard]] inline int leap_seconds_at_utc(double utc_jd) {
    // Before 1972: use 10 seconds
    double jd_1972 = calendar_to_jd(1972, 1, 1, 0, 0, 0.0);
    if (utc_jd < jd_1972) {
        return 10;
    }

    // Search backwards
    for (int i = static_cast<int>(LEAP_SECOND_TABLE.size()) - 1; i >= 0; --i) {
        const auto &entry = LEAP_SECOND_TABLE[i];
        double entry_utc_jd =
            calendar_to_jd(entry.year, entry.month, entry.day, 0, 0, 0.0);

        if (utc_jd >= entry_utc_jd) {
            return entry.delta_at;
        }
    }

    return 10;
}

/**
 * @brief Check if a given UTC date/time is during a leap second
 *
 * Leap seconds occur at 23:59:60 UTC on June 30 or December 31.
 *
 * @param year Year
 * @param month Month (1-12)
 * @param day Day of month
 * @param hour Hour (0-23)
 * @param min Minute (0-59)
 * @param sec Second (0-60)
 * @return true if this is a leap second instant
 */
[[nodiscard]] inline bool is_leap_second(int year, int month, int day, int hour,
                                         int min, double sec) {
    // Leap seconds only occur at 23:59:60 on June 30 or December 31
    if (hour != 23 || min != 59 || sec < 60.0) {
        return false;
    }

    if (!((month == 6 && day == 30) || (month == 12 && day == 31))) {
        return false;
    }

    // Check if this date had a leap second
    int next_month = (month == 6) ? 7 : 1;
    int next_year = (month == 6) ? year : year + 1;

    for (const auto &entry : LEAP_SECOND_TABLE) {
        if (entry.year == next_year && entry.month == next_month) {
            return true;
        }
    }

    return false;
}

/**
 * @brief Get the date of the next leap second after a given date
 *
 * @param utc_jd Julian Date in UTC scale
 * @return Julian Date of next leap second, or 0.0 if unknown/none scheduled
 */
[[nodiscard]] inline double next_leap_second(double utc_jd) {
    for (const auto &entry : LEAP_SECOND_TABLE) {
        double entry_jd =
            calendar_to_jd(entry.year, entry.month, entry.day, 0, 0, 0.0);
        if (entry_jd > utc_jd) {
            return entry_jd;
        }
    }
    return 0.0; // No future leap second known
}

// =============================================================================
// Symbolic Leap Second Lookup (via janus::Interpolator)
// =============================================================================

/**
 * @brief Get a symbolic-compatible leap second interpolator
 *
 * Returns a cached Interpolator that maps UTC JD to TAI-UTC offset.
 * Uses linear interpolation for smooth (differentiable) approximation.
 *
 * @note The returned interpolator is constructed once and cached.
 * @note Linear interpolation gives a smooth ramp during leap second
 *       transitions. For gradient-based optimization spanning months,
 *       this smooth approximation provides well-defined gradients.
 */
[[nodiscard]] inline const janus::Interpolator &leap_second_interpolator() {
    static const janus::Interpolator interp = []() {
        janus::NumericVector jd_points(
            static_cast<int>(LEAP_SECOND_TABLE.size()));
        janus::NumericVector delta_at(
            static_cast<int>(LEAP_SECOND_TABLE.size()));
        for (size_t i = 0; i < LEAP_SECOND_TABLE.size(); ++i) {
            const auto &entry = LEAP_SECOND_TABLE[i];
            jd_points(static_cast<int>(i)) =
                calendar_to_jd(entry.year, entry.month, entry.day);
            delta_at(static_cast<int>(i)) = static_cast<double>(entry.delta_at);
        }
        return janus::Interpolator(jd_points, delta_at,
                                   janus::InterpolationMethod::Linear);
    }();
    return interp;
}

/**
 * @brief Symbolic leap second lookup (smooth approximation)
 *
 * Uses janus::Interpolator for symbolic-compatible table lookup.
 * Linear interpolation provides smooth transitions at leap second boundaries.
 *
 * @tparam Scalar Numeric or symbolic type
 * @param utc_jd Julian Date in UTC scale
 * @return TAI - UTC offset (smooth approximation for symbolic, exact at table
 * points)
 */
template <typename Scalar>
[[nodiscard]] Scalar leap_seconds_symbolic(const Scalar &utc_jd) {
    return leap_second_interpolator()(utc_jd);
}

} // namespace vulcan::time
