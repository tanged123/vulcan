#pragma once

#include <cmath>
#include <janus/janus.hpp>
#include <tuple>
#include <vulcan/time/TimeConstants.hpp>

namespace vulcan::time {

// =============================================================================
// Calendar ↔ Julian Date Conversions (Numeric Only - I/O Operations)
// =============================================================================

/**
 * @brief Convert calendar date/time to Julian Date
 *
 * Uses the algorithm from Vallado's "Fundamentals of Astrodynamics and
 * Applications" Valid for dates from 4713 BCE to far future.
 *
 * @note This is numeric-only as calendar conversions are I/O operations.
 *       For symbolic computations, create an Epoch from numeric calendar
 *       then use templated arithmetic.
 */
[[nodiscard]] inline double calendar_to_jd(int year, int month, int day,
                                           int hour = 0, int min = 0,
                                           double sec = 0.0) {

    // Algorithm from Vallado, "Fundamentals of Astrodynamics", 4th Ed., Eq.
    // 3-14 JD = 367*Y - floor(7*(Y + floor((M+9)/12))/4) + floor(275*M/9) + D +
    // 1721013.5
    //      + ((sec/60 + min)/60 + hour)/24

    double jd =
        367.0 * year -
        std::floor(7.0 * (year + std::floor((month + 9.0) / 12.0)) / 4.0) +
        std::floor(275.0 * month / 9.0) + day + 1721013.5 +
        ((sec / 60.0 + min) / 60.0 + hour) / 24.0;

    return jd;
}

/**
 * @brief Convert Julian Date to calendar date/time (numeric only)
 */
[[nodiscard]] inline std::tuple<int, int, int, int, int, double>
jd_to_calendar(double jd) {
    double jd_plus = jd + 0.5;
    int Z = static_cast<int>(std::floor(jd_plus));
    double F = jd_plus - Z;

    int A;
    if (Z < 2299161) {
        A = Z;
    } else {
        int alpha = static_cast<int>(std::floor((Z - 1867216.25) / 36524.25));
        A = Z + 1 + alpha - alpha / 4;
    }

    int B = A + 1524;
    int C = static_cast<int>(std::floor((B - 122.1) / 365.25));
    int D = static_cast<int>(std::floor(365.25 * C));
    int E = static_cast<int>(std::floor((B - D) / 30.6001));

    double day_frac = B - D - std::floor(30.6001 * E) + F;
    int day = static_cast<int>(std::floor(day_frac));

    int month = (E < 14) ? E - 1 : E - 13;
    int year = (month > 2) ? C - 4716 : C - 4715;

    double frac = day_frac - day;
    double hours_total = frac * 24.0;
    int hour = static_cast<int>(std::floor(hours_total));
    double mins_total = (hours_total - hour) * 60.0;
    int min = static_cast<int>(std::floor(mins_total));
    double sec = (mins_total - min) * 60.0;

    // Handle numerical edge cases
    if (sec >= 60.0 - 1e-10) {
        sec = 0.0;
        min += 1;
    }
    if (min >= 60) {
        min = 0;
        hour += 1;
    }
    if (hour >= 24) {
        hour = 0;
        day += 1;
    }

    return {year, month, day, hour, min, sec};
}

// =============================================================================
// Modified Julian Date Conversions (Templated)
// =============================================================================

/**
 * @brief Convert Julian Date to Modified Julian Date
 * @tparam Scalar Numeric or symbolic type
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar jd_to_mjd(const Scalar &jd) {
    return jd - constants::time::MJD_OFFSET;
}

/**
 * @brief Convert Modified Julian Date to Julian Date
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar mjd_to_jd(const Scalar &mjd) {
    return mjd + constants::time::MJD_OFFSET;
}

// =============================================================================
// J2000.0 Reference Utilities (Templated)
// =============================================================================

/**
 * @brief Convert Julian Date to seconds since J2000.0
 * @tparam Scalar Numeric or symbolic type
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar jd_to_j2000_seconds(const Scalar &jd) {
    return (jd - constants::time::JD_J2000) * constants::time::SECONDS_PER_DAY;
}

/**
 * @brief Convert seconds since J2000.0 to Julian Date
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar j2000_seconds_to_jd(const Scalar &sec) {
    return constants::time::JD_J2000 + sec / constants::time::SECONDS_PER_DAY;
}

/**
 * @brief Convert Julian Date to Julian centuries since J2000.0
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar jd_to_j2000_centuries(const Scalar &jd) {
    return (jd - constants::time::JD_J2000) / constants::time::DAYS_PER_CENTURY;
}

/**
 * @brief Convert Julian centuries since J2000.0 to Julian Date
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar j2000_centuries_to_jd(const Scalar &T) {
    return constants::time::JD_J2000 + T * constants::time::DAYS_PER_CENTURY;
}

// =============================================================================
// Day of Year Utilities (Numeric - Calendar Operations)
// =============================================================================

[[nodiscard]] inline constexpr bool is_leap_year(int year) {
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

[[nodiscard]] inline int day_of_year(int year, int month, int day) {
    constexpr int days_before_month[] = {0,   31,  59,  90,  120, 151,
                                         181, 212, 243, 273, 304, 334};
    int doy = days_before_month[month - 1] + day;
    if (month > 2 && is_leap_year(year)) {
        doy += 1;
    }
    return doy;
}

[[nodiscard]] inline std::tuple<int, int> doy_to_month_day(int year, int doy) {
    constexpr int days_in_month[] = {31, 28, 31, 30, 31, 30,
                                     31, 31, 30, 31, 30, 31};
    int remaining = doy;
    for (int m = 1; m <= 12; ++m) {
        int days = days_in_month[m - 1];
        if (m == 2 && is_leap_year(year))
            days = 29;
        if (remaining <= days)
            return {m, remaining};
        remaining -= days;
    }
    return {12, 31};
}

} // namespace vulcan::time
