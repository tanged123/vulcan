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

} // namespace vulcan::constants::time
