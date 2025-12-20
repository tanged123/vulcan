#pragma once

#include <iomanip>
#include <janus/janus.hpp>
#include <sstream>
#include <string>
#include <vulcan/time/JulianDate.hpp>
#include <vulcan/time/LeapSeconds.hpp>
#include <vulcan/time/TimeConstants.hpp>
#include <vulcan/time/TimeScales.hpp>

namespace vulcan::time {

/**
 * @brief Unified time representation for aerospace applications
 *
 * Epoch stores time internally as TAI seconds since J2000.0 TT.
 * This representation is:
 * - Continuous (no leap seconds in TAI)
 * - High precision (sub-microsecond)
 * - Efficient for arithmetic
 *
 * Provides accessors for all time scales (UTC, TAI, TT, GPS, TDB).
 *
 * @tparam Scalar Numeric (double) or symbolic (casadi::MX) type
 *
 * @example
 * ```cpp
 * // Numeric mode
 * auto epoch = NumericEpoch::from_utc(2024, 7, 15, 12, 30, 0.0);
 * std::cout << epoch.to_iso_string() << std::endl;
 * std::cout << "JD TT: " << epoch.jd_tt() << std::endl;
 *
 * // Symbolic mode
 * auto t_sym = janus::sym("t");
 * auto sym_epoch = SymbolicEpoch::from_tai_seconds(t_sym);
 * auto centuries = sym_epoch.centuries_tt();
 * ```
 */
template <typename Scalar> class Epoch {
  public:
    // =========================================================================
    // Constructors
    // =========================================================================

    /**
     * @brief Default constructor (J2000.0 epoch)
     */
    Epoch() : m_tai_sec(Scalar(0.0)), m_delta_at(37) {}

    /**
     * @brief Construct from TAI seconds since J2000.0
     *
     * @param tai_seconds TAI seconds since J2000.0 TT
     * @param delta_at TAI - UTC offset for this epoch (default: 37 for
     * post-2017)
     */
    explicit Epoch(const Scalar &tai_seconds, int delta_at = 37)
        : m_tai_sec(tai_seconds), m_delta_at(delta_at) {}

    // =========================================================================
    // Factory Methods (Static Construction)
    // =========================================================================

    /**
     * @brief Create Epoch from TAI seconds since J2000.0
     */
    static Epoch from_tai_seconds(const Scalar &tai_sec, int delta_at = 37) {
        return Epoch(tai_sec, delta_at);
    }

    /**
     * @brief Create Epoch from TAI Julian Date
     */
    static Epoch from_jd_tai(const Scalar &jd_tai, int delta_at = 37) {
        Scalar tai_sec = jd_to_j2000_seconds(jd_tai);
        return Epoch(tai_sec, delta_at);
    }

    /**
     * @brief Create Epoch from TT Julian Date
     */
    static Epoch from_jd_tt(const Scalar &jd_tt, int delta_at = 37) {
        Scalar jd_tai = tt_to_tai(jd_tt);
        return from_jd_tai(jd_tai, delta_at);
    }

    /**
     * @brief Create Epoch from GPS Julian Date
     */
    static Epoch from_jd_gps(const Scalar &jd_gps, int delta_at = 37) {
        Scalar jd_tai = gps_to_tai(jd_gps);
        return from_jd_tai(jd_tai, delta_at);
    }

    /**
     * @brief Create Epoch from UTC calendar (numeric only)
     *
     * @note This is numeric-only because calendar parsing is an I/O operation.
     *       For symbolic mode, create from numeric calendar then add symbolic
     * duration.
     */
    static Epoch from_utc(int year, int month, int day, int hour = 0,
                          int min = 0, double sec = 0.0)
        requires std::is_floating_point_v<Scalar>
    {
        double utc_jd = calendar_to_jd(year, month, day, hour, min, sec);
        int delta_at = leap_seconds_at_utc(utc_jd);
        double tai_jd = utc_to_tai(utc_jd, delta_at);
        double tai_sec = jd_to_j2000_seconds(tai_jd);
        return Epoch(tai_sec, delta_at);
    }

    /**
     * @brief Create Epoch from GPS week and seconds
     */
    static Epoch from_gps_week(int week, const Scalar &seconds_of_week,
                               int delta_at = 37) {
        Scalar gps_sec_since_epoch =
            static_cast<double>(week) * constants::time::SECONDS_PER_WEEK +
            seconds_of_week;
        // GPS epoch is 1980-01-06 00:00:00 UTC
        // In TAI seconds since J2000.0:
        // GPS epoch JD = 2444244.5
        // J2000.0 JD = 2451545.0
        // Delta = -7300.5 days = -630763200 seconds
        constexpr double GPS_EPOCH_TAI_SEC =
            (constants::time::JD_GPS_EPOCH - constants::time::JD_J2000) *
                constants::time::SECONDS_PER_DAY +
            constants::time::TAI_GPS_OFFSET;
        Scalar tai_sec = GPS_EPOCH_TAI_SEC + gps_sec_since_epoch;
        return Epoch(tai_sec, delta_at);
    }

    // =========================================================================
    // Time Scale Accessors (Julian Dates)
    // =========================================================================

    /**
     * @brief Get Julian Date in TAI scale
     */
    [[nodiscard]] Scalar jd_tai() const {
        return j2000_seconds_to_jd(m_tai_sec);
    }

    /**
     * @brief Get Julian Date in TT scale
     */
    [[nodiscard]] Scalar jd_tt() const { return tai_to_tt(jd_tai()); }

    /**
     * @brief Get Julian Date in GPS scale
     */
    [[nodiscard]] Scalar jd_gps() const { return tai_to_gps(jd_tai()); }

    /**
     * @brief Get Julian Date in UTC scale (using stored delta_at)
     */
    [[nodiscard]] Scalar jd_utc() const {
        return tai_to_utc(jd_tai(), m_delta_at);
    }

    /**
     * @brief Get Julian Date in TDB scale
     */
    [[nodiscard]] Scalar jd_tdb() const { return tt_to_tdb(jd_tt()); }

    /**
     * @brief Get Modified Julian Date in TT scale
     */
    [[nodiscard]] Scalar mjd_tt() const { return jd_to_mjd(jd_tt()); }

    // =========================================================================
    // Time Scale Accessors (Seconds)
    // =========================================================================

    /**
     * @brief Get TAI seconds since J2000.0
     */
    [[nodiscard]] const Scalar &tai_seconds() const { return m_tai_sec; }

    /**
     * @brief Get TT seconds since J2000.0
     */
    [[nodiscard]] Scalar tt_seconds() const {
        return m_tai_sec + constants::time::TT_TAI_OFFSET;
    }

    /**
     * @brief Get Julian centuries since J2000.0 in TT scale
     *
     * Commonly used for precession/nutation models.
     */
    [[nodiscard]] Scalar centuries_tt() const {
        return tt_seconds() / constants::time::SECONDS_PER_CENTURY;
    }

    // =========================================================================
    // GPS Time Accessors
    // =========================================================================

    /**
     * @brief Get GPS week number
     */
    [[nodiscard]] int gps_week() const
        requires std::is_floating_point_v<Scalar>
    {
        Scalar gps_jd = jd_gps();
        double days_since_gps_epoch = gps_jd - constants::time::JD_GPS_EPOCH;
        return static_cast<int>(std::floor(days_since_gps_epoch / 7.0));
    }

    /**
     * @brief Get seconds within the GPS week
     */
    [[nodiscard]] Scalar gps_seconds_of_week() const {
        Scalar gps_jd = jd_gps();
        Scalar days_since_gps_epoch = gps_jd - constants::time::JD_GPS_EPOCH;
        // fmod for symbolic
        if constexpr (std::is_floating_point_v<Scalar>) {
            double weeks =
                std::floor(static_cast<double>(days_since_gps_epoch) / 7.0);
            double days_into_week = days_since_gps_epoch - weeks * 7.0;
            return days_into_week * constants::time::SECONDS_PER_DAY;
        } else {
            // Symbolic: use janus::fmod
            Scalar days_into_week =
                janus::fmod(days_since_gps_epoch, Scalar(7.0));
            return days_into_week * constants::time::SECONDS_PER_DAY;
        }
    }

    // =========================================================================
    // Leap Second Access
    // =========================================================================

    /**
     * @brief Get the stored TAI - UTC offset
     */
    [[nodiscard]] int delta_at() const { return m_delta_at; }

    /**
     * @brief Set the TAI - UTC offset
     */
    void set_delta_at(int delta_at) { m_delta_at = delta_at; }

    // =========================================================================
    // Arithmetic Operators
    // =========================================================================

    /**
     * @brief Add duration in seconds
     */
    Epoch operator+(const Scalar &seconds) const {
        return Epoch(m_tai_sec + seconds, m_delta_at);
    }

    /**
     * @brief Subtract duration in seconds
     */
    Epoch operator-(const Scalar &seconds) const {
        return Epoch(m_tai_sec - seconds, m_delta_at);
    }

    /**
     * @brief Time difference in seconds
     */
    Scalar operator-(const Epoch &other) const {
        return m_tai_sec - other.m_tai_sec;
    }

    /**
     * @brief Add duration in seconds (in-place)
     */
    Epoch &operator+=(const Scalar &seconds) {
        m_tai_sec = m_tai_sec + seconds;
        return *this;
    }

    /**
     * @brief Subtract duration in seconds (in-place)
     */
    Epoch &operator-=(const Scalar &seconds) {
        m_tai_sec = m_tai_sec - seconds;
        return *this;
    }

    // =========================================================================
    // String Conversion (Numeric Only)
    // =========================================================================

    /**
     * @brief Convert to ISO 8601 string in UTC
     */
    [[nodiscard]] std::string to_iso_string() const
        requires std::is_floating_point_v<Scalar>
    {
        double utc_jd = jd_utc();
        auto [year, month, day, hour, min, sec] = jd_to_calendar(utc_jd);

        std::ostringstream oss;
        oss << std::setfill('0') << std::setw(4) << year << '-' << std::setw(2)
            << month << '-' << std::setw(2) << day << 'T' << std::setw(2)
            << hour << ':' << std::setw(2) << min << ':' << std::setw(2)
            << static_cast<int>(sec);

        // Add fractional seconds if significant
        double frac = sec - std::floor(sec);
        if (frac > 1e-6) {
            oss << '.' << std::setw(3) << static_cast<int>(frac * 1000);
        }

        oss << 'Z';
        return oss.str();
    }

    /**
     * @brief Convert to calendar components in UTC
     */
    [[nodiscard]] std::tuple<int, int, int, int, int, double>
    to_utc_calendar() const
        requires std::is_floating_point_v<Scalar>
    {
        double utc_jd = jd_utc();
        return jd_to_calendar(utc_jd);
    }

  private:
    Scalar m_tai_sec; ///< TAI seconds since J2000.0 TT
    int m_delta_at;   ///< TAI - UTC offset for conversions
};

// =============================================================================
// Type Aliases
// =============================================================================

/// Numeric epoch (double-precision)
using NumericEpoch = Epoch<double>;

/// Symbolic epoch (CasADi MX)
using SymbolicEpoch = Epoch<janus::SymbolicScalar>;

} // namespace vulcan::time
