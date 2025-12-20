# Phase 9: Time Systems — Comprehensive Implementation Plan

> **Purpose**: Implement a robust time infrastructure for aerospace simulations, providing accurate conversions between time scales (UTC, TAI, TT, GPS, TDB), Julian date utilities, and leap second handling. This module underpins coordinate frame transformations (ECEF↔ECI) and future SOFA integration.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Design Decisions](#2-design-decisions)
3. [Time Scale Reference](#3-time-scale-reference)
4. [Component 1: Time Constants](#4-component-1-time-constants)
5. [Component 2: Julian Date Utilities](#5-component-2-julian-date-utilities)
6. [Component 3: Leap Second Table](#6-component-3-leap-second-table)
7. [Component 4: Time Scale Conversions](#7-component-4-time-scale-conversions)
8. [Component 5: Epoch Class](#8-component-5-epoch-class)
9. [Component 6: GPS Time Utilities](#9-component-6-gps-time-utilities)
10. [Integration with Existing Code](#10-integration-with-existing-code)
11. [Test Plan](#11-test-plan)
12. [File Summary](#12-file-summary)
13. [Implementation Order](#13-implementation-order)
14. [Verification Checklist](#14-verification-checklist)

---

## 1. Overview

### Scope

This phase implements Phase 3b from the unified coordinate frames plan, focused on time infrastructure:

| Component | Purpose | Accuracy Target |
|-----------|---------|-----------------|
| **Julian Date** | Calendar ↔ JD ↔ MJD conversions | Sub-microsecond |
| **Leap Seconds** | UTC ↔ TAI offset table (IERS Bulletin C) | Exact (integer seconds) |
| **Time Scales** | UTC, TAI, TT, GPS, TDB conversions | ~1 μs (TDB: ~2 ms) |
| **Epoch** | Unified time representation | All scales consistent |
| **GPS Time** | Week/seconds, GPS↔UTC | Sub-microsecond |

### Key Constraints

1. **Dual Symbolic/Numeric Support**: Time utilities are templated on `Scalar` following Vulcan conventions:
   - Core time arithmetic (JD conversions, scale offsets) fully templated
   - Leap second handling: numeric lookup OR user-provided constant for symbolic mode
   - Enables optimization over time-dependent trajectories

2. **No External Dependencies**: Pure C++ implementation (no SOFA yet — that's Phase 3c)

3. **Thread Safety**: Leap second table is const after static initialization

4. **Leap Second Updates**: Design allows easy updates when IERS publishes new leap seconds

### Symbolic Compatibility Strategy

Two approaches for leap second lookup in symbolic mode:

#### Approach 1: User-Provided Constant (Simple)

For most optimization problems, the simulation epoch is known and leap seconds are constant:

```cpp
// Templated version: user provides leap second count (constant for their epoch)
template <typename Scalar>
Scalar utc_to_tai(const Scalar& utc_jd, int delta_at);

// Numeric convenience: looks up leap seconds automatically
inline double utc_to_tai(double utc_jd) {
    return utc_to_tai(utc_jd, leap_seconds_at_utc(utc_jd));
}
```

**Rationale**: In optimization, the user knows their simulation's time range. Leap seconds change at most once per year, so providing a constant is practical and enables full symbolic tracing.

#### Approach 2: Symbolic Table Lookup via `janus::Interpolator`

For trajectories spanning leap second boundaries, use Janus's symbolic-compatible interpolation:

```cpp
#include <janus/math/Interpolate.hpp>

// Create a leap second lookup table as Linear interpolator
// Linear gives smooth (differentiable) approximation at transitions
inline const janus::Interpolator& leap_second_interpolator() {
    static janus::Interpolator interp = []() {
        // Build JD and delta_at vectors from LEAP_SECOND_TABLE
        janus::NumericVector jd_points(LEAP_SECOND_TABLE.size());
        janus::NumericVector delta_at(LEAP_SECOND_TABLE.size());
        for (size_t i = 0; i < LEAP_SECOND_TABLE.size(); ++i) {
            const auto& entry = LEAP_SECOND_TABLE[i];
            jd_points(i) = calendar_to_jd(entry.year, entry.month, entry.day);
            delta_at(i) = static_cast<double>(entry.delta_at);
        }
        return janus::Interpolator(jd_points, delta_at,
                                   janus::InterpolationMethod::Linear);
    }();
    return interp;
}

// Fully symbolic leap second lookup (smooth approximation)
template <typename Scalar>
Scalar leap_seconds_symbolic(const Scalar& utc_jd) {
    return leap_second_interpolator()(utc_jd);
}

// Fully symbolic UTC to TAI conversion
template <typename Scalar>
Scalar utc_to_tai_symbolic(const Scalar& utc_jd) {
    Scalar delta_at = leap_seconds_symbolic(utc_jd);
    return utc_jd + delta_at / constants::time::SECONDS_PER_DAY;
}
```

**Trade-offs**:

| Approach | Accuracy | Symbolic | Use Case |
|----------|----------|----------|----------|
| **Constant delta_at** | Exact | Yes | Single-epoch optimization |
| **Interpolator** | Smooth approx | Yes | Multi-month trajectories |
| **Numeric lookup** | Exact | No | Simulation, post-processing |

> [!NOTE]
> Linear interpolation gives a smooth ramp during leap second transitions (~1 day).
> For gradient-based optimization spanning months, the smooth approximation is often
> preferred as it provides well-defined gradients at all points.

---

## 2. Design Decisions

### 2.1 Internal Representation

> [!IMPORTANT]
> **Epoch stores TAI seconds since J2000.0 TT internally**
>
> - TAI is continuous (no leap seconds) — arithmetic is simple
> - J2000.0 is the standard astronomical epoch (2000-01-01 12:00:00 TT)
> - All other time scales derived via offsets

### 2.2 Julian Date Epoch

| Constant | Value | Definition |
|----------|-------|------------|
| `JD_J2000` | 2451545.0 | J2000.0 epoch (2000-01-01 12:00:00 TT) |
| `MJD_J2000` | 51544.5 | J2000.0 in Modified Julian Date |
| `JD_GPS_EPOCH` | 2444244.5 | GPS epoch (1980-01-06 00:00:00 UTC) |
| `JD_UNIX_EPOCH` | 2440587.5 | Unix epoch (1970-01-01 00:00:00 UTC) |

### 2.3 Time Scale Offsets

```
TT  = TAI + 32.184 s           (exact, by definition)
GPS = TAI - 19 s               (exact, GPS tracks TAI with 19s offset)
TDB ≈ TT + periodic terms      (~1.7 ms amplitude, due to Earth's orbital eccentricity)
UTC = TAI - leap_seconds(TAI)  (lookup table)
```

### 2.4 Leap Second Table Design

```cpp
// Compact representation: each entry is (TAI JD, cumulative leap seconds)
// Table grows by ~1 entry every 1-2 years
struct LeapSecondEntry {
    double tai_jd;      // TAI Julian Date when this leap second took effect
    int delta_at;       // TAI - UTC in seconds (cumulative)
};

// Latest entry (as of 2017): TAI - UTC = 37 seconds
// No new leap seconds announced through end of 2024
```

---

## 3. Time Scale Reference

### 3.1 Time Scale Relationships

```
           +32.184s              -19s
    TAI ─────────────▶ TT    TAI ────────▶ GPS
     │
     │ -ΔAT (leap seconds)
     ▼
    UTC
     │
     │ +ΔUT1 (Earth rotation)
     ▼
    UT1 ────────────────────▶ GMST ────▶ ECI/ECEF rotation
```

### 3.2 Time Scale Definitions

| Scale | Full Name | Definition | Use Case |
|-------|-----------|------------|----------|
| **TAI** | International Atomic Time | SI seconds, continuous | Internal representation |
| **UTC** | Coordinated Universal Time | TAI + leap seconds, tracks UT1 | Civil time, input/output |
| **TT** | Terrestrial Time | TAI + 32.184s | Ephemerides, dynamics |
| **TDB** | Barycentric Dynamical Time | TT + periodic (~1.7 ms) | Solar system dynamics |
| **GPS** | GPS Time | TAI - 19s | GPS receivers, navigation |
| **UT1** | Universal Time 1 | Earth rotation angle | ECI/ECEF transformation |

### 3.3 When to Use Each Scale

| Application | Time Scale | Notes |
|-------------|------------|-------|
| User I/O, file timestamps | UTC | Human-readable |
| Orbit propagation | TT or TDB | Dynamics equations |
| GPS position fixes | GPS | Direct from receiver |
| Coordinate transforms | UT1 (via UTC+DUT1) | Earth rotation |
| Internal storage | TAI | Continuous, no ambiguity |

---

## 4. Component 1: Time Constants

### [NEW] `include/vulcan/time/TimeConstants.hpp`

```cpp
#pragma once

namespace vulcan::constants::time {

// =============================================================================
// Julian Date Reference Epochs
// =============================================================================

/// Julian Date of J2000.0 epoch (2000-01-01 12:00:00 TT)
inline constexpr double JD_J2000 = 2451545.0;

/// Modified Julian Date of J2000.0
inline constexpr double MJD_J2000 = 51544.5;

/// Julian Date offset: JD = MJD + MJD_OFFSET
inline constexpr double MJD_OFFSET = 2400000.5;

/// Julian Date of GPS epoch (1980-01-06 00:00:00 UTC)
inline constexpr double JD_GPS_EPOCH = 2444244.5;

/// Julian Date of Unix epoch (1970-01-01 00:00:00 UTC)
inline constexpr double JD_UNIX_EPOCH = 2440587.5;

/// Julian Date of J1900.0 (1899-12-31 12:00:00 UT)
inline constexpr double JD_J1900 = 2415020.0;

// =============================================================================
// Time Scale Offsets (seconds)
// =============================================================================

/// TT - TAI offset [s] (exact, by definition)
inline constexpr double TT_TAI_OFFSET = 32.184;

/// GPS - TAI offset [s] (exact: GPS = TAI - 19s at GPS epoch)
inline constexpr double GPS_TAI_OFFSET = -19.0;

/// TAI - GPS offset [s]
inline constexpr double TAI_GPS_OFFSET = 19.0;

// =============================================================================
// Time Conversion Factors
// =============================================================================

/// Seconds per day
inline constexpr double SECONDS_PER_DAY = 86400.0;

/// Seconds per Julian century
inline constexpr double SECONDS_PER_CENTURY = 86400.0 * 36525.0;

/// Days per Julian century
inline constexpr double DAYS_PER_CENTURY = 36525.0;

/// Days per Julian year
inline constexpr double DAYS_PER_YEAR = 365.25;

/// Seconds per week (for GPS)
inline constexpr double SECONDS_PER_WEEK = 604800.0;

// =============================================================================
// Precision Constants
// =============================================================================

/// Tolerance for Julian Date comparisons [days] (~1 microsecond)
inline constexpr double JD_TOLERANCE = 1.0e-11;

/// Tolerance for time comparisons [seconds] (~1 nanosecond)
inline constexpr double TIME_TOLERANCE = 1.0e-9;

}  // namespace vulcan::constants::time
```

---

## 5. Component 2: Julian Date Utilities

### [NEW] `include/vulcan/time/JulianDate.hpp`

```cpp
#pragma once

#include <vulcan/time/TimeConstants.hpp>
#include <janus/janus.hpp>
#include <cmath>
#include <tuple>

namespace vulcan::time {

// =============================================================================
// Calendar ↔ Julian Date Conversions (Numeric Only - I/O Operations)
// =============================================================================

/**
 * @brief Convert calendar date/time to Julian Date
 *
 * Uses the algorithm from Vallado's "Fundamentals of Astrodynamics and Applications"
 * Valid for dates from 4713 BCE to far future.
 *
 * @note This is numeric-only as calendar conversions are I/O operations.
 *       For symbolic computations, create an Epoch from numeric calendar
 *       then use templated arithmetic.
 */
[[nodiscard]] inline double calendar_to_jd(
    int year, int month, int day,
    int hour = 12, int min = 0, double sec = 0.0) {

    // Adjust for January/February (treat as months 13/14 of previous year)
    int y = year;
    int m = month;
    if (m <= 2) {
        y -= 1;
        m += 12;
    }

    // Julian Day Number at noon
    int A = y / 100;
    int B = 2 - A + A / 4;  // Gregorian calendar correction

    double jd_noon = std::floor(365.25 * (y + 4716)) +
                     std::floor(30.6001 * (m + 1)) +
                     day + B - 1524.5;

    // Add fractional day
    double frac_day = (hour - 12 + min / 60.0 + sec / 3600.0) / 24.0;

    return jd_noon + frac_day;
}

/**
 * @brief Convert Julian Date to calendar date/time (numeric only)
 */
[[nodiscard]] inline std::tuple<int, int, int, int, int, double> jd_to_calendar(double jd) {
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
    if (sec >= 60.0 - 1e-10) { sec = 0.0; min += 1; }
    if (min >= 60) { min = 0; hour += 1; }
    if (hour >= 24) { hour = 0; day += 1; }

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
[[nodiscard]] constexpr Scalar jd_to_mjd(const Scalar& jd) {
    return jd - constants::time::MJD_OFFSET;
}

/**
 * @brief Convert Modified Julian Date to Julian Date
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar mjd_to_jd(const Scalar& mjd) {
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
[[nodiscard]] constexpr Scalar jd_to_j2000_seconds(const Scalar& jd) {
    return (jd - constants::time::JD_J2000) * constants::time::SECONDS_PER_DAY;
}

/**
 * @brief Convert seconds since J2000.0 to Julian Date
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar j2000_seconds_to_jd(const Scalar& sec) {
    return constants::time::JD_J2000 + sec / constants::time::SECONDS_PER_DAY;
}

/**
 * @brief Convert Julian Date to Julian centuries since J2000.0
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar jd_to_j2000_centuries(const Scalar& jd) {
    return (jd - constants::time::JD_J2000) / constants::time::DAYS_PER_CENTURY;
}

/**
 * @brief Convert Julian centuries since J2000.0 to Julian Date
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar j2000_centuries_to_jd(const Scalar& T) {
    return constants::time::JD_J2000 + T * constants::time::DAYS_PER_CENTURY;
}

// =============================================================================
// Day of Year Utilities (Numeric - Calendar Operations)
// =============================================================================

[[nodiscard]] inline constexpr bool is_leap_year(int year) {
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

[[nodiscard]] inline int day_of_year(int year, int month, int day) {
    constexpr int days_before_month[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    int doy = days_before_month[month - 1] + day;
    if (month > 2 && is_leap_year(year)) {
        doy += 1;
    }
    return doy;
}

[[nodiscard]] inline std::tuple<int, int> doy_to_month_day(int year, int doy) {
    constexpr int days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int remaining = doy;
    for (int m = 1; m <= 12; ++m) {
        int days = days_in_month[m - 1];
        if (m == 2 && is_leap_year(year)) days = 29;
        if (remaining <= days) return {m, remaining};
        remaining -= days;
    }
    return {12, 31};
}

}  // namespace vulcan::time
```

---

## 6. Component 3: Leap Second Table

### [NEW] `include/vulcan/time/LeapSeconds.hpp`

```cpp
#pragma once

#include <vulcan/time/TimeConstants.hpp>
#include <vulcan/time/JulianDate.hpp>
#include <janus/math/Interpolate.hpp>  // For symbolic leap second lookup
#include <array>
#include <stdexcept>

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
    int delta_at;  ///< TAI - UTC [seconds]
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
    {1972,  1,  1, 10},  // Initial offset when leap seconds began
    {1972,  7,  1, 11},
    {1973,  1,  1, 12},
    {1974,  1,  1, 13},
    {1975,  1,  1, 14},
    {1976,  1,  1, 15},
    {1977,  1,  1, 16},
    {1978,  1,  1, 17},
    {1979,  1,  1, 18},
    {1980,  1,  1, 19},
    {1981,  7,  1, 20},
    {1982,  7,  1, 21},
    {1983,  7,  1, 22},
    {1985,  7,  1, 23},
    {1988,  1,  1, 24},
    {1990,  1,  1, 25},
    {1991,  1,  1, 26},
    {1992,  7,  1, 27},
    {1993,  7,  1, 28},
    {1994,  7,  1, 29},
    {1996,  1,  1, 30},
    {1997,  7,  1, 31},
    {1999,  1,  1, 32},
    {2006,  1,  1, 33},
    {2009,  1,  1, 34},
    {2012,  7,  1, 35},
    {2015,  7,  1, 36},
    {2017,  1,  1, 37},  // Most recent leap second
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
    // Convert first entry to TAI JD for comparison
    // Leap second takes effect at 00:00:00 UTC, which is 00:00:delta_at TAI

    // Before 1972: use 10 seconds as approximation
    double jd_1972 = calendar_to_jd(1972, 1, 1, 0, 0, 0.0) + 10.0 / constants::time::SECONDS_PER_DAY;
    if (tai_jd < jd_1972) {
        return 10;
    }

    // Search backwards through table (most queries will be recent dates)
    for (int i = static_cast<int>(LEAP_SECOND_TABLE.size()) - 1; i >= 0; --i) {
        const auto& entry = LEAP_SECOND_TABLE[i];
        // Convert entry date to TAI JD
        // Entry is when delta_at takes effect at 00:00:00 UTC
        // In TAI, this is 00:00:00 UTC + delta_at seconds
        double entry_tai_jd = calendar_to_jd(entry.year, entry.month, entry.day, 0, 0, 0.0) +
                              entry.delta_at / constants::time::SECONDS_PER_DAY;

        if (tai_jd >= entry_tai_jd) {
            return entry.delta_at;
        }
    }

    return 10;  // Fallback (shouldn't reach for valid inputs)
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
        const auto& entry = LEAP_SECOND_TABLE[i];
        double entry_utc_jd = calendar_to_jd(entry.year, entry.month, entry.day, 0, 0, 0.0);

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
[[nodiscard]] inline bool is_leap_second(int year, int month, int day,
                                          int hour, int min, double sec) {
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

    for (const auto& entry : LEAP_SECOND_TABLE) {
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
    for (const auto& entry : LEAP_SECOND_TABLE) {
        double entry_jd = calendar_to_jd(entry.year, entry.month, entry.day, 0, 0, 0.0);
        if (entry_jd > utc_jd) {
            return entry_jd;
        }
    }
    return 0.0;  // No future leap second known
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
 */
[[nodiscard]] inline const janus::Interpolator& leap_second_interpolator() {
    static const janus::Interpolator interp = []() {
        janus::NumericVector jd_points(LEAP_SECOND_TABLE.size());
        janus::NumericVector delta_at(LEAP_SECOND_TABLE.size());
        for (size_t i = 0; i < LEAP_SECOND_TABLE.size(); ++i) {
            const auto& entry = LEAP_SECOND_TABLE[i];
            jd_points(static_cast<int>(i)) = calendar_to_jd(entry.year, entry.month, entry.day);
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
 * @return TAI - UTC offset (smooth approximation for symbolic, exact at table points)
 */
template <typename Scalar>
[[nodiscard]] Scalar leap_seconds_symbolic(const Scalar& utc_jd) {
    return leap_second_interpolator()(utc_jd);
}

}  // namespace vulcan::time
```

---

## 7. Component 4: Time Scale Conversions

### [NEW] `include/vulcan/time/TimeScales.hpp`

```cpp
#pragma once

#include <vulcan/time/TimeConstants.hpp>
#include <vulcan/time/JulianDate.hpp>
#include <vulcan/time/LeapSeconds.hpp>
#include <janus/janus.hpp>
#include <cmath>

namespace vulcan::time {

// =============================================================================
// Enumeration of Time Scales
// =============================================================================

enum class TimeScale {
    UTC,  ///< Coordinated Universal Time (civil time, has leap seconds)
    TAI,  ///< International Atomic Time (continuous SI seconds)
    TT,   ///< Terrestrial Time (TT = TAI + 32.184s)
    TDB,  ///< Barycentric Dynamical Time (for solar system dynamics)
    GPS,  ///< GPS Time (GPS = TAI - 19s, continuous since 1980)
    UT1   ///< Universal Time 1 (Earth rotation, requires IERS data)
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
[[nodiscard]] constexpr Scalar tai_to_tt(const Scalar& tai_jd) {
    return tai_jd + constants::time::TT_TAI_OFFSET / constants::time::SECONDS_PER_DAY;
}

/**
 * @brief Convert TT Julian Date to TAI Julian Date
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar tt_to_tai(const Scalar& tt_jd) {
    return tt_jd - constants::time::TT_TAI_OFFSET / constants::time::SECONDS_PER_DAY;
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
[[nodiscard]] constexpr Scalar tai_to_gps(const Scalar& tai_jd) {
    return tai_jd + constants::time::GPS_TAI_OFFSET / constants::time::SECONDS_PER_DAY;
}

/**
 * @brief Convert GPS Julian Date to TAI Julian Date
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar gps_to_tai(const Scalar& gps_jd) {
    return gps_jd + constants::time::TAI_GPS_OFFSET / constants::time::SECONDS_PER_DAY;
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
 * @note For symbolic mode, user provides delta_at as a constant.
 *       For numeric mode, use the convenience overload without delta_at.
 */
template <typename Scalar>
[[nodiscard]] Scalar utc_to_tai(const Scalar& utc_jd, int delta_at) {
    return utc_jd + static_cast<double>(delta_at) / constants::time::SECONDS_PER_DAY;
}

/**
 * @brief Convert UTC Julian Date to TAI Julian Date (numeric convenience)
 *
 * Automatically looks up leap seconds from the IERS table.
 */
[[nodiscard]] inline double utc_to_tai(double utc_jd) {
    return utc_to_tai(utc_jd, leap_seconds_at_utc(utc_jd));
}

/**
 * @brief Convert TAI Julian Date to UTC Julian Date (templated)
 *
 * @param delta_at TAI - UTC offset in seconds
 */
template <typename Scalar>
[[nodiscard]] Scalar tai_to_utc(const Scalar& tai_jd, int delta_at) {
    return tai_jd - static_cast<double>(delta_at) / constants::time::SECONDS_PER_DAY;
}

/**
 * @brief Convert TAI Julian Date to UTC Julian Date (numeric convenience)
 */
[[nodiscard]] inline double tai_to_utc(double tai_jd) {
    return tai_to_utc(tai_jd, leap_seconds_at_tai(tai_jd));
}

// =============================================================================
// UTC ↔ TT Conversions (Templated via TAI)
// =============================================================================

/**
 * @brief Convert UTC Julian Date to TT Julian Date (templated)
 */
template <typename Scalar>
[[nodiscard]] Scalar utc_to_tt(const Scalar& utc_jd, int delta_at) {
    return tai_to_tt(utc_to_tai(utc_jd, delta_at));
}

[[nodiscard]] inline double utc_to_tt(double utc_jd) {
    return tai_to_tt(utc_to_tai(utc_jd));
}

/**
 * @brief Convert TT Julian Date to UTC Julian Date (templated)
 */
template <typename Scalar>
[[nodiscard]] Scalar tt_to_utc(const Scalar& tt_jd, int delta_at) {
    return tai_to_utc(tt_to_tai(tt_jd), delta_at);
}

[[nodiscard]] inline double tt_to_utc(double tt_jd) {
    return tai_to_utc(tt_to_tai(tt_jd));
}

// =============================================================================
// UTC ↔ GPS Conversions (Templated)
// =============================================================================

template <typename Scalar>
[[nodiscard]] Scalar utc_to_gps(const Scalar& utc_jd, int delta_at) {
    return tai_to_gps(utc_to_tai(utc_jd, delta_at));
}

[[nodiscard]] inline double utc_to_gps(double utc_jd) {
    return tai_to_gps(utc_to_tai(utc_jd));
}

template <typename Scalar>
[[nodiscard]] Scalar gps_to_utc(const Scalar& gps_jd, int delta_at) {
    return tai_to_utc(gps_to_tai(gps_jd), delta_at);
}

[[nodiscard]] inline double gps_to_utc(double gps_jd) {
    return tai_to_utc(gps_to_tai(gps_jd));
}

/**
 * @brief Get GPS - UTC offset (leap seconds since GPS epoch)
 */
[[nodiscard]] inline int gps_utc_offset(double utc_jd) {
    return leap_seconds_at_utc(utc_jd) - 19;
}

// =============================================================================
// TT ↔ TDB Conversions (Templated - Trigonometric)
// =============================================================================

/**
 * @brief Convert TT Julian Date to TDB Julian Date (templated)
 *
 * TDB differs from TT by periodic terms due to Earth's orbital motion.
 * Maximum amplitude ~1.7 ms. Uses IERS Conventions 2010 expression.
 *
 * @tparam Scalar Numeric or symbolic type
 */
template <typename Scalar>
[[nodiscard]] Scalar tt_to_tdb(const Scalar& tt_jd) {
    // Julian centuries since J2000.0 TT
    Scalar T = jd_to_j2000_centuries(tt_jd);

    // Mean anomaly of Earth (radians)
    Scalar g = 6.24006013 + 628.301955 * T;

    // TDB - TT in seconds (simplified formula, ~30 μs accuracy)
    Scalar dt = 0.001657 * janus::sin(g) + 0.000022 * janus::sin(2.0 * g);

    return tt_jd + dt / constants::time::SECONDS_PER_DAY;
}

/**
 * @brief Convert TDB Julian Date to TT Julian Date (templated)
 *
 * Uses single Newton iteration (sufficient for μs accuracy).
 */
template <typename Scalar>
[[nodiscard]] Scalar tdb_to_tt(const Scalar& tdb_jd) {
    // Initial guess: TT ≈ TDB
    Scalar tt_approx = tdb_jd;
    Scalar tdb_calc = tt_to_tdb(tt_approx);
    Scalar correction = tdb_jd - tdb_calc;
    return tt_approx + correction;
}

// =============================================================================
// General Conversion Interface (Numeric Only - Uses Table Lookup)
// =============================================================================

/**
 * @brief Convert Julian Date between time scales (numeric)
 *
 * @note For symbolic conversions, use the individual templated functions
 *       with explicit leap second parameters.
 */
[[nodiscard]] inline double convert_timescale(double jd, TimeScale from, TimeScale to) {
    if (from == to) return jd;

    // Convert to TAI first (hub)
    double tai_jd;
    switch (from) {
        case TimeScale::TAI: tai_jd = jd; break;
        case TimeScale::UTC: tai_jd = utc_to_tai(jd); break;
        case TimeScale::TT:  tai_jd = tt_to_tai(jd); break;
        case TimeScale::GPS: tai_jd = gps_to_tai(jd); break;
        case TimeScale::TDB: tai_jd = tt_to_tai(tdb_to_tt(jd)); break;
        case TimeScale::UT1:
            throw std::invalid_argument("UT1 conversion requires IERS DUT1 data");
    }

    // Convert from TAI to target
    switch (to) {
        case TimeScale::TAI: return tai_jd;
        case TimeScale::UTC: return tai_to_utc(tai_jd);
        case TimeScale::TT:  return tai_to_tt(tai_jd);
        case TimeScale::GPS: return tai_to_gps(tai_jd);
        case TimeScale::TDB: return tt_to_tdb(tai_to_tt(tai_jd));
        case TimeScale::UT1:
            throw std::invalid_argument("UT1 conversion requires IERS DUT1 data");
    }
    return tai_jd;
}

}  // namespace vulcan::time
```

---

## 8. Component 5: Epoch Class

### [NEW] `include/vulcan/time/Epoch.hpp`

```cpp
#pragma once

#include <vulcan/time/TimeConstants.hpp>
#include <vulcan/time/JulianDate.hpp>
#include <vulcan/time/LeapSeconds.hpp>
#include <vulcan/time/TimeScales.hpp>
#include <janus/janus.hpp>
#include <string>
#include <sstream>
#include <iomanip>

namespace vulcan::time {

/**
 * @brief Unified time representation across all astronomical time scales
 *
 * Epoch<Scalar> stores time internally as TAI seconds since J2000.0 TT epoch.
 * This representation is:
 * - Continuous (no leap seconds to handle in arithmetic)
 * - High precision (double gives ~μs precision over centuries)
 * - Symbolic-compatible (templated on Scalar type)
 *
 * @tparam Scalar Numeric (double) or symbolic (casadi::MX) type
 *
 * Example usage:
 * @code
 *   // Numeric mode: Create from UTC calendar
 *   auto epoch = Epoch<double>::from_utc(2024, 7, 15, 12, 30, 0.0);
 *   double jd_tt = epoch.jd_tt();
 *
 *   // Symbolic mode: Create from symbolic TAI seconds
 *   auto t = janus::sym("t");  // Optimization variable
 *   auto sym_epoch = Epoch<janus::SymbolicScalar>::from_tai_seconds(t);
 *   auto jd = sym_epoch.jd_tt();  // Symbolic expression
 * @endcode
 */
template <typename Scalar>
class Epoch {
public:
    // =========================================================================
    // Constructors
    // =========================================================================

    /**
     * @brief Default constructor: J2000.0 epoch
     */
    constexpr Epoch() : tai_seconds_since_j2000_(Scalar(0.0)) {}

    /**
     * @brief Construct from TAI seconds since J2000.0 TT
     */
    explicit constexpr Epoch(const Scalar& tai_seconds)
        : tai_seconds_since_j2000_(tai_seconds) {}

    // =========================================================================
    // Factory Methods (Numeric-Only: Use Calendar/Table Lookup)
    // =========================================================================

    /**
     * @brief Create Epoch from UTC calendar date/time (numeric only)
     *
     * @note This is numeric-only because it requires leap second lookup.
     *       For symbolic mode, use from_tai_seconds() or from_jd_tt().
     */
    [[nodiscard]] static Epoch from_utc(int year, int month, int day,
                                         int hour = 12, int min = 0, double sec = 0.0) {
        static_assert(std::is_same_v<Scalar, double>,
                      "from_utc() requires numeric Scalar (use from_tai_seconds for symbolic)");
        double utc_jd = calendar_to_jd(year, month, day, hour, min, sec);
        double tai_jd = utc_to_tai(utc_jd);
        double tai_sec = jd_to_j2000_seconds(tai_jd);
        return Epoch(tai_sec);
    }

    /**
     * @brief Create Epoch from GPS week and seconds (numeric only)
     */
    [[nodiscard]] static Epoch from_gps_week(int week, double seconds) {
        static_assert(std::is_same_v<Scalar, double>,
                      "from_gps_week() requires numeric Scalar");
        double gps_epoch_tai_sec = jd_to_j2000_seconds(
            utc_to_tai(constants::time::JD_GPS_EPOCH));
        double gps_seconds = week * constants::time::SECONDS_PER_WEEK + seconds;
        double tai_sec = gps_epoch_tai_sec + gps_seconds + constants::time::TAI_GPS_OFFSET;
        return Epoch(tai_sec);
    }

    /**
     * @brief Create Epoch from Unix timestamp (numeric only)
     */
    [[nodiscard]] static Epoch from_unix(double unix_time) {
        static_assert(std::is_same_v<Scalar, double>,
                      "from_unix() requires numeric Scalar");
        double utc_jd = constants::time::JD_UNIX_EPOCH +
                        unix_time / constants::time::SECONDS_PER_DAY;
        double tai_jd = utc_to_tai(utc_jd);
        return Epoch(jd_to_j2000_seconds(tai_jd));
    }

    // =========================================================================
    // Factory Methods (Templated: Work in Both Modes)
    // =========================================================================

    /**
     * @brief Create Epoch from TAI seconds since J2000.0 (templated)
     *
     * Primary entry point for symbolic mode.
     */
    [[nodiscard]] static Epoch from_tai_seconds(const Scalar& tai_sec) {
        return Epoch(tai_sec);
    }

    /**
     * @brief Create Epoch from Julian Date in TT scale (templated)
     */
    [[nodiscard]] static Epoch from_jd_tt(const Scalar& jd_tt) {
        Scalar tai_jd = tt_to_tai(jd_tt);
        return Epoch(jd_to_j2000_seconds(tai_jd));
    }

    /**
     * @brief Create Epoch from Julian Date in TAI scale (templated)
     */
    [[nodiscard]] static Epoch from_jd_tai(const Scalar& jd_tai) {
        return Epoch(jd_to_j2000_seconds(jd_tai));
    }

    /**
     * @brief Create Epoch from Julian Date in GPS scale (templated)
     */
    [[nodiscard]] static Epoch from_jd_gps(const Scalar& jd_gps) {
        Scalar tai_jd = gps_to_tai(jd_gps);
        return Epoch(jd_to_j2000_seconds(tai_jd));
    }

    /**
     * @brief Create Epoch from JD in UTC scale (templated, requires delta_at)
     *
     * @param jd_utc Julian Date in UTC scale
     * @param delta_at TAI - UTC offset (leap seconds) for this epoch
     */
    [[nodiscard]] static Epoch from_jd_utc(const Scalar& jd_utc, int delta_at) {
        Scalar tai_jd = utc_to_tai(jd_utc, delta_at);
        return Epoch(jd_to_j2000_seconds(tai_jd));
    }

    /**
     * @brief J2000.0 epoch (2000-01-01 12:00:00 TT)
     */
    [[nodiscard]] static constexpr Epoch j2000() {
        return Epoch(Scalar(0.0));
    }

    // =========================================================================
    // Accessors - TAI (Internal Representation)
    // =========================================================================

    /**
     * @brief Get TAI seconds since J2000.0 TT (internal representation)
     */
    [[nodiscard]] constexpr const Scalar& tai_seconds() const {
        return tai_seconds_since_j2000_;
    }

    /**
     * @brief Get Julian Date in TAI scale
     */
    [[nodiscard]] Scalar jd_tai() const {
        return j2000_seconds_to_jd(tai_seconds_since_j2000_);
    }

    // =========================================================================
    // Accessors - TT Scale (Templated)
    // =========================================================================

    /**
     * @brief Get Julian Date in TT scale
     */
    [[nodiscard]] Scalar jd_tt() const {
        return tai_to_tt(jd_tai());
    }

    /**
     * @brief Get Modified Julian Date in TT scale
     */
    [[nodiscard]] Scalar mjd_tt() const {
        return jd_to_mjd(jd_tt());
    }

    /**
     * @brief Get Julian centuries since J2000.0 TT
     */
    [[nodiscard]] Scalar centuries_tt() const {
        return jd_to_j2000_centuries(jd_tt());
    }

    // =========================================================================
    // Accessors - GPS Scale (Templated)
    // =========================================================================

    /**
     * @brief Get Julian Date in GPS scale
     */
    [[nodiscard]] Scalar jd_gps() const {
        return tai_to_gps(jd_tai());
    }

    // =========================================================================
    // Accessors - UTC Scale (Requires Leap Second Parameter for Symbolic)
    // =========================================================================

    /**
     * @brief Get Julian Date in UTC scale (templated, requires delta_at)
     */
    [[nodiscard]] Scalar jd_utc(int delta_at) const {
        return tai_to_utc(jd_tai(), delta_at);
    }

    /**
     * @brief Get Julian Date in UTC scale (numeric convenience)
     */
    [[nodiscard]] double jd_utc() const {
        static_assert(std::is_same_v<Scalar, double>,
                      "jd_utc() without delta_at requires numeric Scalar");
        return tai_to_utc(jd_tai());
    }

    // =========================================================================
    // Accessors - TDB Scale (Templated)
    // =========================================================================

    /**
     * @brief Get Julian Date in TDB scale
     */
    [[nodiscard]] Scalar jd_tdb() const {
        return tt_to_tdb(jd_tt());
    }

    // =========================================================================
    // Arithmetic Operators (Templated)
    // =========================================================================

    /**
     * @brief Add seconds to epoch
     */
    [[nodiscard]] Epoch operator+(const Scalar& seconds) const {
        return Epoch(tai_seconds_since_j2000_ + seconds);
    }

    /**
     * @brief Subtract seconds from epoch
     */
    [[nodiscard]] Epoch operator-(const Scalar& seconds) const {
        return Epoch(tai_seconds_since_j2000_ - seconds);
    }

    /**
     * @brief Get time difference between epochs in seconds
     */
    [[nodiscard]] Scalar operator-(const Epoch& other) const {
        return tai_seconds_since_j2000_ - other.tai_seconds_since_j2000_;
    }

    /**
     * @brief Add seconds to this epoch (in-place)
     */
    Epoch& operator+=(const Scalar& seconds) {
        tai_seconds_since_j2000_ = tai_seconds_since_j2000_ + seconds;
        return *this;
    }

    /**
     * @brief Subtract seconds from this epoch (in-place)
     */
    Epoch& operator-=(const Scalar& seconds) {
        tai_seconds_since_j2000_ = tai_seconds_since_j2000_ - seconds;
        return *this;
    }

    // =========================================================================
    // Numeric-Only Accessors (Calendar, Formatting)
    // =========================================================================

    /**
     * @brief Get calendar date/time in UTC (numeric only)
     */
    [[nodiscard]] auto calendar_utc() const {
        static_assert(std::is_same_v<Scalar, double>,
                      "calendar_utc() requires numeric Scalar");
        return jd_to_calendar(jd_utc());
    }

    /**
     * @brief Get GPS week number (numeric only)
     */
    [[nodiscard]] int gps_week() const {
        static_assert(std::is_same_v<Scalar, double>,
                      "gps_week() requires numeric Scalar");
        double gps_jd = jd_gps();
        double days_since_gps_epoch = gps_jd - constants::time::JD_GPS_EPOCH;
        double seconds_since_gps_epoch = days_since_gps_epoch * constants::time::SECONDS_PER_DAY;
        return static_cast<int>(seconds_since_gps_epoch / constants::time::SECONDS_PER_WEEK);
    }

    /**
     * @brief Get GPS seconds of week (numeric only)
     */
    [[nodiscard]] double gps_seconds_of_week() const {
        static_assert(std::is_same_v<Scalar, double>,
                      "gps_seconds_of_week() requires numeric Scalar");
        double gps_jd = jd_gps();
        double days_since_gps_epoch = gps_jd - constants::time::JD_GPS_EPOCH;
        double seconds_since_gps_epoch = days_since_gps_epoch * constants::time::SECONDS_PER_DAY;
        return std::fmod(seconds_since_gps_epoch, constants::time::SECONDS_PER_WEEK);
    }

    /**
     * @brief Format as ISO 8601 string (numeric only)
     */
    [[nodiscard]] std::string to_iso_string() const {
        static_assert(std::is_same_v<Scalar, double>,
                      "to_iso_string() requires numeric Scalar");
        auto [y, m, d, h, min, sec] = calendar_utc();
        std::ostringstream oss;
        oss << std::setfill('0')
            << std::setw(4) << y << "-"
            << std::setw(2) << m << "-"
            << std::setw(2) << d << "T"
            << std::setw(2) << h << ":"
            << std::setw(2) << min << ":"
            << std::setw(6) << std::fixed << std::setprecision(3) << sec << "Z";
        return oss.str();
    }

private:
    Scalar tai_seconds_since_j2000_;  ///< TAI seconds since J2000.0 TT epoch
};

// =============================================================================
// Type Aliases
// =============================================================================

/// Numeric epoch for simulation
using NumericEpoch = Epoch<double>;

/// Symbolic epoch for optimization
using SymbolicEpoch = Epoch<janus::SymbolicScalar>;

// =============================================================================
// Stream Output (Numeric Only)
// =============================================================================

inline std::ostream& operator<<(std::ostream& os, const NumericEpoch& epoch) {
    return os << epoch.to_iso_string();
}

}  // namespace vulcan::time
```

---

## 9. Component 6: GPS Time Utilities

### [NEW] `include/vulcan/time/GPSTime.hpp`

```cpp
#pragma once

#include <vulcan/time/TimeConstants.hpp>
#include <vulcan/time/Epoch.hpp>

namespace vulcan::time {

// =============================================================================
// GPS Time Representation
// =============================================================================

/**
 * @brief GPS time as week number and seconds of week
 *
 * GPS time is continuous (no leap seconds) and is commonly represented
 * as a week number and seconds within that week.
 *
 * Week 0 began 1980-01-06 00:00:00 UTC.
 * GPS weeks roll over every 1024 weeks (~19.7 years).
 */
struct GPSTime {
    int week;           ///< GPS week number (from 1980-01-06)
    double seconds;     ///< Seconds of week [0, 604800)

    /**
     * @brief Create GPSTime from week and seconds
     */
    constexpr GPSTime(int week_, double seconds_) : week(week_), seconds(seconds_) {}

    /**
     * @brief Create GPSTime from Epoch
     */
    explicit GPSTime(const Epoch& epoch)
        : week(epoch.gps_week()), seconds(epoch.gps_seconds_of_week()) {}

    /**
     * @brief Convert to Epoch
     */
    [[nodiscard]] Epoch to_epoch() const {
        return Epoch::from_gps_week(week, seconds);
    }

    /**
     * @brief Get day of week (0=Sunday, 6=Saturday)
     */
    [[nodiscard]] constexpr int day_of_week() const {
        return static_cast<int>(seconds / constants::time::SECONDS_PER_DAY);
    }

    /**
     * @brief Get seconds of day
     */
    [[nodiscard]] constexpr double seconds_of_day() const {
        return std::fmod(seconds, constants::time::SECONDS_PER_DAY);
    }

    /**
     * @brief Add seconds
     */
    [[nodiscard]] GPSTime operator+(double dt) const {
        double new_seconds = seconds + dt;
        int new_week = week;

        while (new_seconds >= constants::time::SECONDS_PER_WEEK) {
            new_seconds -= constants::time::SECONDS_PER_WEEK;
            new_week++;
        }
        while (new_seconds < 0) {
            new_seconds += constants::time::SECONDS_PER_WEEK;
            new_week--;
        }

        return GPSTime(new_week, new_seconds);
    }

    /**
     * @brief Difference in seconds
     */
    [[nodiscard]] double operator-(const GPSTime& other) const {
        return (week - other.week) * constants::time::SECONDS_PER_WEEK +
               (seconds - other.seconds);
    }
};

// =============================================================================
// GPS Week Rollover Utilities
// =============================================================================

/**
 * @brief Resolve 10-bit GPS week number (0-1023) to full week
 *
 * GPS navigation messages transmit only 10 bits of the week number,
 * which rolls over every 1024 weeks (~19.7 years). This function
 * resolves the full week number given a reference epoch.
 *
 * @param week_10bit 10-bit week number from GPS receiver (0-1023)
 * @param reference Reference epoch (typically current time)
 * @return Full GPS week number
 */
[[nodiscard]] inline int resolve_gps_week_rollover(int week_10bit, const Epoch& reference) {
    int ref_week = reference.gps_week();
    int ref_cycle = ref_week / 1024;  // Current 1024-week cycle

    // Assume the 10-bit week is in the current or adjacent cycle
    int full_week = ref_cycle * 1024 + week_10bit;

    // If this gives a week far in the past, it's probably the next cycle
    if (full_week < ref_week - 512) {
        full_week += 1024;
    }
    // If far in the future, it's probably the previous cycle
    else if (full_week > ref_week + 512) {
        full_week -= 1024;
    }

    return full_week;
}

/**
 * @brief Get current GPS rollover cycle number
 *
 * Cycle 0: 1980-01-06 to 1999-08-21
 * Cycle 1: 1999-08-22 to 2019-04-06
 * Cycle 2: 2019-04-07 to 2038-11-20
 *
 * @param epoch Epoch to query
 * @return Rollover cycle number (0, 1, 2, ...)
 */
[[nodiscard]] inline int gps_rollover_cycle(const Epoch& epoch) {
    return epoch.gps_week() / 1024;
}

}  // namespace vulcan::time
```

---

## 10. Integration with Existing Code

### 10.1 Update Main Header

#### [MODIFY] `include/vulcan/vulcan.hpp`

Add after coordinate includes:

```cpp
// Time systems
#include <vulcan/time/TimeConstants.hpp>
#include <vulcan/time/JulianDate.hpp>
#include <vulcan/time/LeapSeconds.hpp>
#include <vulcan/time/TimeScales.hpp>
#include <vulcan/time/Epoch.hpp>
#include <vulcan/time/GPSTime.hpp>
```

### 10.2 Integration with EarthModel.hpp

The existing `GMSTRotation` class in `EarthModel.hpp` can be enhanced to use the new time utilities:

```cpp
// In GMSTRotation::gmst(), add option to accept Epoch:
[[nodiscard]] double gmst(const Epoch& epoch) const {
    // Use TT for GMST calculation
    double T = epoch.centuries_since_j2000(TimeScale::TT);
    // ... existing formula ...
}
```

### 10.3 Future SOFA Integration (Phase 3c)

The Epoch class provides a clean interface for SOFA integration:

```cpp
// Future: include/vulcan/orientation/SOFA.hpp
namespace vulcan::orientation {

/// Earth Rotation Angle (ERA) - modern replacement for GMST
double era(const time::Epoch& epoch);

/// ITRF to GCRF transformation matrix (mas accuracy)
Mat3<double> itrf_to_gcrf(const time::Epoch& epoch);

/// GCRF to ITRF transformation matrix
Mat3<double> gcrf_to_itrf(const time::Epoch& epoch);

}  // namespace vulcan::orientation
```

---

## 11. Test Plan

### 11.1 Test File Structure

```
tests/
└── time/
    ├── test_julian_date.cpp     # JD/MJD conversions
    ├── test_leap_seconds.cpp    # Leap second table
    ├── test_time_scales.cpp     # UTC/TAI/TT/GPS conversions
    ├── test_epoch.cpp           # Epoch class
    └── test_gps_time.cpp        # GPS week/seconds
```

### 11.2 Test Cases

#### test_julian_date.cpp

| Test | Description | Reference |
|------|-------------|-----------|
| `CalendarToJD_J2000` | 2000-01-01 12:00:00 TT → JD 2451545.0 | Definition |
| `CalendarToJD_GPSEpoch` | 1980-01-06 00:00:00 → JD 2444244.5 | GPS ICD |
| `CalendarToJD_UnixEpoch` | 1970-01-01 00:00:00 → JD 2440587.5 | IEEE/POSIX |
| `JDToCalendar_Roundtrip` | JD → Calendar → JD | |
| `MJD_Conversion` | JD ↔ MJD consistency | |
| `LeapYear_Detection` | 2000, 2024 are leap; 1900, 2100 not | |
| `DayOfYear_Roundtrip` | DOY → Month/Day → DOY | |

#### test_leap_seconds.cpp

| Test | Description | Reference |
|------|-------------|-----------|
| `LeapSeconds_Pre1972` | Before 1972 → 10 seconds | Approximation |
| `LeapSeconds_1972` | 1972-01-01 → 10, 1972-07-01 → 11 | IERS |
| `LeapSeconds_2017` | 2017-01-01 → 37 (most recent) | IERS |
| `LeapSeconds_Future` | 2030-01-01 → 37 (no new announced) | IERS |
| `LeapSecond_IsLeapSecond` | 2016-12-31 23:59:60 is leap second | IERS |
| `UTC_TAI_Roundtrip` | UTC → TAI → UTC consistency | |

#### test_time_scales.cpp

| Test | Description | Reference |
|------|-------------|-----------|
| `TAI_TT_Offset` | TT - TAI = 32.184 s exactly | IAU |
| `TAI_GPS_Offset` | TAI - GPS = 19 s exactly | GPS ICD |
| `GPS_UTC_J2000` | GPS - UTC = 13 s at J2000 | IERS |
| `GPS_UTC_2024` | GPS - UTC = 18 s in 2024 | IERS |
| `TDB_TT_Range` | \|TDB - TT\| < 2 ms | IERS |
| `ConvertTimescale_AllPairs` | All scale combinations | |
| `Symbolic_TAI_TT` | Symbolic tai_to_tt graph | |
| `Symbolic_TT_TDB` | Symbolic tt_to_tdb (uses janus::sin) | |
| `Symbolic_UTC_TAI_WithDeltaAT` | Symbolic utc_to_tai with parameter | |

#### test_epoch.cpp

| Test | Description | Reference |
|------|-------------|-----------|
| `J2000_Factory` | Epoch::j2000() → JD 2451545.0 TT | Definition |
| `FromUTC_Calendar` | From 2024-07-15 12:30:00 UTC | |
| `FromGPSWeek` | From week 2320, 0.0 s | |
| `ToISO8601` | Formatting consistency | ISO 8601 |
| `Arithmetic_AddSubtract` | epoch + 3600 - epoch = 3600 | |
| `GPSWeek_Accessor` | Consistent with from_gps_week | |
| `UnixTime_Roundtrip` | Unix → Epoch → Unix | |
| `Symbolic_FromTAISeconds` | Symbolic epoch creation | |
| `Symbolic_JD_TT` | Symbolic JD(TT) computation | |
| `Symbolic_Arithmetic` | Symbolic epoch + dt | |
| `Symbolic_TDB_Conversion` | Symbolic TT→TDB (uses janus::sin) | |

#### test_gps_time.cpp

| Test | Description | Reference |
|------|-------------|-----------|
| `GPSTime_ToEpoch_Roundtrip` | GPSTime ↔ Epoch | |
| `WeekRollover_Cycle0` | Week 1023 → 0 in 1999 | GPS ICD |
| `WeekRollover_Cycle1` | Week 2047 → 0 in 2019 | GPS ICD |
| `Resolve10BitWeek` | 10-bit week resolution | |
| `DayOfWeek_Sunday` | Seconds 0 → Sunday | |
| `DayOfWeek_Saturday` | Seconds 518400 → Saturday | |

### 11.3 Reference Validation Points

From IERS Bulletin C and GPS ICD:

```
J2000.0 TT:        2000-01-01 12:00:00 TT = JD 2451545.0
                   = 2000-01-01 11:58:55.816 UTC (TAI-UTC=32s, TT-TAI=32.184s)

GPS Epoch:         1980-01-06 00:00:00 UTC = JD 2444244.5
                   TAI-UTC = 19s at this instant

2024-01-01 00:00:00 UTC:
                   TAI-UTC = 37 s
                   GPS-UTC = 18 s
                   GPS Week ≈ 2294
```

---

## 12. File Summary

### New Files

| Type | Path | Templated | Purpose |
|------|------|-----------|---------|
| [NEW] | `include/vulcan/time/TimeConstants.hpp` | No (constants) | JD epochs, offsets, conversion factors |
| [NEW] | `include/vulcan/time/JulianDate.hpp` | Partial | Calendar (numeric), JD arithmetic (templated) |
| [NEW] | `include/vulcan/time/LeapSeconds.hpp` | No (table lookup) | IERS leap second table |
| [NEW] | `include/vulcan/time/TimeScales.hpp` | Yes | UTC/TAI/TT/GPS/TDB conversions |
| [NEW] | `include/vulcan/time/Epoch.hpp` | Yes (`Epoch<Scalar>`) | Unified time representation |
| [NEW] | `include/vulcan/time/GPSTime.hpp` | Partial | GPS week/seconds utilities |
| [NEW] | `tests/time/test_julian_date.cpp` | - | JD/MJD tests (numeric + symbolic) |
| [NEW] | `tests/time/test_leap_seconds.cpp` | - | Leap second tests (numeric) |
| [NEW] | `tests/time/test_time_scales.cpp` | - | Time scale tests (numeric + symbolic) |
| [NEW] | `tests/time/test_epoch.cpp` | - | Epoch tests (numeric + symbolic) |
| [NEW] | `tests/time/test_gps_time.cpp` | - | GPS time tests |

### Modified Files

| Type | Path | Changes |
|------|------|---------|
| [MODIFY] | `include/vulcan/vulcan.hpp` | Add time includes |
| [MODIFY] | `tests/CMakeLists.txt` | Add test_time executable |

---

## 13. Implementation Order

### Step 1: Time Constants (30 min)
- Create `TimeConstants.hpp` with all epoch and conversion constants
- No dependencies, foundational for all other components

### Step 2: Julian Date Utilities (1 hour)
- Create `JulianDate.hpp` with calendar ↔ JD conversions
- Create `test_julian_date.cpp` and verify against known values
- Dependencies: TimeConstants

### Step 3: Leap Seconds (1 hour)
- Create `LeapSeconds.hpp` with IERS table and lookup functions
- Create `test_leap_seconds.cpp` and verify against IERS Bulletin C
- Dependencies: JulianDate

### Step 4: Time Scale Conversions (1.5 hours)
- Create `TimeScales.hpp` with UTC/TAI/TT/GPS/TDB conversions
- Create `test_time_scales.cpp` and verify chain conversions
- Dependencies: LeapSeconds

### Step 5: Epoch Class (2 hours)
- Create `Epoch.hpp` with unified time representation
- Create `test_epoch.cpp` with comprehensive tests
- Dependencies: TimeScales

### Step 6: GPS Time Utilities (1 hour)
- Create `GPSTime.hpp` with week/seconds and rollover handling
- Create `test_gps_time.cpp`
- Dependencies: Epoch

### Step 7: Integration (30 min)
- Update `vulcan.hpp` with new includes
- Update `tests/CMakeLists.txt`
- Run full test suite

---

## 14. Verification Checklist

Before marking Phase 9 complete:

### Compilation
- [ ] All headers compile without warnings (`-Wall -Wextra`)
- [ ] Templates instantiate correctly for `double`
- [ ] Templates instantiate correctly for `janus::SymbolicScalar`

### Numeric Mode Tests
- [ ] All tests pass in numeric mode
- [ ] Julian Date roundtrip error < 1 μs
- [ ] Leap second table matches IERS Bulletin C 69
- [ ] GPS week number correct for current date
- [ ] Epoch::to_iso_string() produces valid ISO 8601

### Symbolic Mode Tests
- [ ] `Epoch<SymbolicScalar>` creates valid symbolic graphs
- [ ] Time scale conversions (TAI↔TT, TAI↔GPS, TT↔TDB) produce traceable expressions
- [ ] Arithmetic operations (+, -, +=, -=) work symbolically
- [ ] `janus::Function` can be created from time expressions
- [ ] Numeric evaluation of symbolic expressions matches direct numeric computation

### Documentation & Integration
- [ ] Documentation in each header is complete
- [ ] `./scripts/verify.sh` passes
- [ ] vulcan.hpp includes all time headers

### Reference Implementation Check

Validate against external tools:
```bash
# Python astropy (if available)
python3 -c "
from astropy.time import Time
t = Time('2024-07-15 12:30:00', scale='utc')
print(f'JD UTC: {t.jd}')
print(f'JD TT:  {t.tt.jd}')
print(f'GPS week: {t.gps.jd1 + t.gps.jd2 - 2444244.5}')
"
```

---

## Appendix A: Usage Examples

### Numeric Mode

```cpp
#include <vulcan/vulcan.hpp>
#include <iostream>

int main() {
    using namespace vulcan::time;

    // Create epoch from UTC calendar
    auto epoch = NumericEpoch::from_utc(2024, 7, 15, 12, 30, 0.0);

    std::cout << "=== Numeric Time Example ===" << std::endl;
    std::cout << "UTC:         " << epoch.to_iso_string() << std::endl;
    std::cout << "JD (TT):     " << epoch.jd_tt() << std::endl;
    std::cout << "JD (TAI):    " << epoch.jd_tai() << std::endl;
    std::cout << "JD (GPS):    " << epoch.jd_gps() << std::endl;
    std::cout << "GPS Week:    " << epoch.gps_week() << std::endl;
    std::cout << "GPS Seconds: " << epoch.gps_seconds_of_week() << std::endl;

    // Time arithmetic
    auto one_hour_later = epoch + 3600.0;
    std::cout << "\n+1 hour:     " << one_hour_later.to_iso_string() << std::endl;

    return 0;
}
```

### Symbolic Mode (Optimization)

```cpp
#include <vulcan/vulcan.hpp>
#include <janus/janus.hpp>

// Physics model: orbital period depends on time (e.g., due to drag)
template <typename Scalar>
Scalar orbital_period(const vulcan::time::Epoch<Scalar>& epoch) {
    // Get Julian centuries since J2000 for secular terms
    Scalar T = epoch.centuries_tt();

    // Simplified: period changes linearly with time
    return 5400.0 + 0.001 * T;  // ~90 min orbit, slowly decaying
}

int main() {
    using namespace vulcan::time;
    using Scalar = janus::SymbolicScalar;

    // Create symbolic time variable
    auto t_sym = janus::sym("t");  // TAI seconds since J2000

    // Create symbolic epoch
    auto epoch = Epoch<Scalar>::from_tai_seconds(t_sym);

    // Compute symbolic orbital period
    auto period = orbital_period(epoch);

    // Create evaluable function
    janus::Function f("period", {t_sym}, {period});

    // Evaluate at specific times
    double t_2024 = 788918400.0;  // Approx mid-2024 in TAI seconds since J2000
    auto result = f({t_2024});
    std::cout << "Period at 2024: " << result[0](0, 0) << " seconds" << std::endl;

    // Can also compute gradients!
    auto jacobian = janus::jacobian(period, t_sym);
    janus::Function df("dperiod_dt", {t_sym}, {jacobian});
    auto grad = df({t_2024});
    std::cout << "dPeriod/dt: " << grad[0](0, 0) << " s/s" << std::endl;

    return 0;
}
```

### Handling Leap Seconds in Symbolic Mode

```cpp
#include <vulcan/vulcan.hpp>

template <typename Scalar>
Scalar compute_utc_offset(const Scalar& tai_jd, int delta_at) {
    // Use the templated version with explicit leap second parameter
    return vulcan::time::tai_to_utc(tai_jd, delta_at);
}

int main() {
    using Scalar = janus::SymbolicScalar;

    auto tai_jd = janus::sym("tai_jd");

    // User knows their simulation epoch is in 2024, where TAI-UTC = 37s
    constexpr int DELTA_AT_2024 = 37;

    auto utc_jd = compute_utc_offset(tai_jd, DELTA_AT_2024);

    // This creates a traceable symbolic expression
    janus::Function f("tai_to_utc", {tai_jd}, {utc_jd});

    // Works correctly for any TAI JD in the 2024 leap second era
    double test_tai_jd = 2460500.5;  // Some date in 2024
    auto result = f({test_tai_jd});
    std::cout << "UTC JD: " << result[0](0, 0) << std::endl;

    return 0;
}
```

---

## Appendix B: Leap Second Table Update Procedure

When IERS announces a new leap second (typically Bulletin C, issued every 6 months):

1. Check IERS Bulletin C: https://hpiers.obspm.fr/iers/bul/bulc/bulletinc.dat
2. Add new entry to `LEAP_SECOND_TABLE` in `LeapSeconds.hpp`
3. Update "Last updated" comment
4. Add test case for new leap second
5. Run verification
6. Document in commit message: "Update leap second table to Bulletin C XX"

---

## Appendix C: Time Scale Diagram

```
                          ┌─────────────────────────────────────────────────┐
                          │                    TDB                          │
                          │     (Barycentric Dynamical Time)                │
                          │     Solar system ephemeris                      │
                          └───────────────────┬─────────────────────────────┘
                                              │ ±1.7 ms (periodic)
                          ┌───────────────────▼─────────────────────────────┐
                          │                    TT                           │
                          │     (Terrestrial Time)                          │
                          │     Satellite dynamics, orbit propagation       │
                          └───────────────────┬─────────────────────────────┘
                                              │ +32.184 s (exact)
                          ┌───────────────────▼─────────────────────────────┐
                          │                    TAI                          │
                          │     (International Atomic Time)                 │
                          │     SI seconds, continuous, internal storage    │
                          └───────────────────┬─────────────────────────────┘
                             -19 s (exact)    │    -ΔAT (leap seconds)
                    ┌─────────────────────────┼─────────────────────────────┐
                    │                         │                             │
        ┌───────────▼───────────┐   ┌─────────▼─────────┐                   │
        │         GPS           │   │        UTC        │                   │
        │    (GPS Time)         │   │  (Coord. Univ.)   │                   │
        │    Navigation         │   │  Civil time       │                   │
        └───────────────────────┘   └─────────┬─────────┘                   │
                                              │ +DUT1 (IERS, ~±0.5s)        │
                                    ┌─────────▼─────────┐                   │
                                    │        UT1        │◄──────────────────┘
                                    │  (Universal Time) │   Derived from
                                    │  Earth rotation   │   Earth orientation
                                    └───────────────────┘
```

---

*Last updated: Phase 9 planning, December 2024*
*Revision: Added dual symbolic/numeric support per Vulcan conventions*
