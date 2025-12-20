#include <cmath>
#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/time/Time.hpp>

using namespace vulcan::time;
using namespace vulcan::constants::time;

// =============================================================================
// TimeConstants Tests
// =============================================================================

TEST(TimeConstantsTest, ReferenceEpochs) {
    // J2000.0 epoch: 2000-01-01 12:00:00 TT
    EXPECT_DOUBLE_EQ(JD_J2000, 2451545.0);
    EXPECT_DOUBLE_EQ(MJD_J2000, 51544.5);

    // Verify JD/MJD relationship
    EXPECT_DOUBLE_EQ(JD_J2000 - MJD_OFFSET, MJD_J2000);

    // GPS epoch: 1980-01-06 00:00:00 UTC
    EXPECT_DOUBLE_EQ(JD_GPS_EPOCH, 2444244.5);

    // Unix epoch: 1970-01-01 00:00:00 UTC
    EXPECT_DOUBLE_EQ(JD_UNIX_EPOCH, 2440587.5);
}

TEST(TimeConstantsTest, TimeScaleOffsets) {
    EXPECT_DOUBLE_EQ(TT_TAI_OFFSET, 32.184);
    EXPECT_DOUBLE_EQ(GPS_TAI_OFFSET, -19.0);
    EXPECT_DOUBLE_EQ(TAI_GPS_OFFSET, 19.0);
}

TEST(TimeConstantsTest, ConversionFactors) {
    EXPECT_DOUBLE_EQ(SECONDS_PER_DAY, 86400.0);
    EXPECT_DOUBLE_EQ(SECONDS_PER_WEEK, 604800.0);
    EXPECT_DOUBLE_EQ(DAYS_PER_CENTURY, 36525.0);
    EXPECT_DOUBLE_EQ(SECONDS_PER_CENTURY, 86400.0 * 36525.0);
}

// =============================================================================
// JulianDate Tests
// =============================================================================

TEST(JulianDateTest, CalendarToJD_J2000) {
    // J2000.0 epoch: 2000-01-01 12:00:00
    double jd = calendar_to_jd(2000, 1, 1, 12, 0, 0.0);
    EXPECT_NEAR(jd, JD_J2000, 1e-10);
}

TEST(JulianDateTest, CalendarToJD_GPSEpoch) {
    // GPS epoch: 1980-01-06 00:00:00
    double jd = calendar_to_jd(1980, 1, 6, 0, 0, 0.0);
    EXPECT_NEAR(jd, JD_GPS_EPOCH, 1e-10);
}

TEST(JulianDateTest, CalendarToJD_UnixEpoch) {
    // Unix epoch: 1970-01-01 00:00:00
    double jd = calendar_to_jd(1970, 1, 1, 0, 0, 0.0);
    EXPECT_NEAR(jd, JD_UNIX_EPOCH, 1e-10);
}

TEST(JulianDateTest, JDToCalendar_J2000) {
    auto [year, month, day, hour, min, sec] = jd_to_calendar(JD_J2000);
    EXPECT_EQ(year, 2000);
    EXPECT_EQ(month, 1);
    EXPECT_EQ(day, 1);
    EXPECT_EQ(hour, 12);
    EXPECT_EQ(min, 0);
    EXPECT_NEAR(sec, 0.0, 1e-6);
}

TEST(JulianDateTest, CalendarRoundtrip) {
    // Test various dates
    std::vector<std::tuple<int, int, int, int, int, double>> test_dates = {
        {1999, 12, 31, 23, 59, 59.0},
        {2024, 7, 15, 12, 30, 45.5},
        {1985, 6, 30, 18, 45, 30.0},
        {2050, 3, 1, 0, 0, 0.0},
    };

    for (const auto &[y, m, d, h, mi, s] : test_dates) {
        double jd = calendar_to_jd(y, m, d, h, mi, s);
        auto [y2, m2, d2, h2, mi2, s2] = jd_to_calendar(jd);
        EXPECT_EQ(y, y2) << "Year mismatch for " << y << "-" << m << "-" << d;
        EXPECT_EQ(m, m2) << "Month mismatch";
        EXPECT_EQ(d, d2) << "Day mismatch";
        EXPECT_EQ(h, h2) << "Hour mismatch";
        EXPECT_EQ(mi, mi2) << "Minute mismatch";
        EXPECT_NEAR(s, s2, 1e-4)
            << "Second mismatch"; // ~0.1 ms precision due to JD representation
    }
}

TEST(JulianDateTest, MJDConversions) {
    double jd = JD_J2000;
    double mjd = jd_to_mjd(jd);
    EXPECT_NEAR(mjd, MJD_J2000, 1e-10);
    EXPECT_NEAR(mjd_to_jd(mjd), jd, 1e-10);
}

TEST(JulianDateTest, J2000SecondsConversions) {
    // At J2000.0, seconds should be zero
    double sec = jd_to_j2000_seconds(JD_J2000);
    EXPECT_NEAR(sec, 0.0, 1e-10);

    // One day later
    double sec_1day = jd_to_j2000_seconds(JD_J2000 + 1.0);
    EXPECT_NEAR(sec_1day, SECONDS_PER_DAY, 1e-6);

    // Roundtrip
    double jd_back = j2000_seconds_to_jd(sec_1day);
    EXPECT_NEAR(jd_back, JD_J2000 + 1.0, 1e-10);
}

TEST(JulianDateTest, J2000CenturiesConversions) {
    // One Julian century after J2000.0
    double jd_century = JD_J2000 + DAYS_PER_CENTURY;
    double T = jd_to_j2000_centuries(jd_century);
    EXPECT_NEAR(T, 1.0, 1e-10);
}

TEST(JulianDateTest, LeapYearDetection) {
    EXPECT_TRUE(is_leap_year(2000));  // Century year divisible by 400
    EXPECT_FALSE(is_leap_year(1900)); // Century year not divisible by 400
    EXPECT_TRUE(is_leap_year(2004));  // Divisible by 4
    EXPECT_FALSE(is_leap_year(2001)); // Not divisible by 4
    EXPECT_TRUE(is_leap_year(2024));
}

TEST(JulianDateTest, DayOfYear) {
    // January 1
    EXPECT_EQ(day_of_year(2024, 1, 1), 1);
    // February 29 in leap year
    EXPECT_EQ(day_of_year(2024, 2, 29), 60);
    // December 31 in leap year
    EXPECT_EQ(day_of_year(2024, 12, 31), 366);
    // December 31 in non-leap year
    EXPECT_EQ(day_of_year(2023, 12, 31), 365);
}

// =============================================================================
// LeapSeconds Tests
// =============================================================================

TEST(LeapSecondsTest, TableSize) { EXPECT_EQ(LEAP_SECOND_TABLE.size(), 28); }

TEST(LeapSecondsTest, TableBoundaries) {
    // First entry: 1972-01-01, delta_at = 10
    EXPECT_EQ(LEAP_SECOND_TABLE[0].year, 1972);
    EXPECT_EQ(LEAP_SECOND_TABLE[0].delta_at, 10);

    // Last entry: 2017-01-01, delta_at = 37
    EXPECT_EQ(LEAP_SECOND_TABLE[27].year, 2017);
    EXPECT_EQ(LEAP_SECOND_TABLE[27].delta_at, 37);
}

TEST(LeapSecondsTest, LookupAtUTC) {
    // Before 1972
    EXPECT_EQ(leap_seconds_at_utc(calendar_to_jd(1970, 1, 1, 0, 0, 0.0)), 10);

    // In 1972 (after first entry)
    EXPECT_EQ(leap_seconds_at_utc(calendar_to_jd(1972, 6, 15, 12, 0, 0.0)), 10);

    // After second leap second
    EXPECT_EQ(leap_seconds_at_utc(calendar_to_jd(1972, 7, 1, 0, 0, 0.0)), 11);

    // Current era (2024)
    EXPECT_EQ(leap_seconds_at_utc(calendar_to_jd(2024, 7, 15, 12, 0, 0.0)), 37);
}

TEST(LeapSecondsTest, SymbolicInterpolator) {
    // Test that interpolator works with numeric values
    double utc_jd = calendar_to_jd(2024, 7, 15, 12, 0, 0.0);
    double delta_at = leap_seconds_symbolic(utc_jd);
    EXPECT_NEAR(delta_at, 37.0, 1e-6);

    // Value between leap seconds should be interpolated
    double utc_jd_2016 = calendar_to_jd(2016, 7, 1, 12, 0, 0.0);
    double delta_at_2016 = leap_seconds_symbolic(utc_jd_2016);
    // Should be between 36 and 37 (linear interpolation)
    EXPECT_GT(delta_at_2016, 35.5);
    EXPECT_LT(delta_at_2016, 37.5);
}

// =============================================================================
// TimeScales Tests
// =============================================================================

TEST(TimeScalesTest, TAI_TT_Conversions) {
    double tai_jd = JD_J2000;
    double tt_jd = tai_to_tt(tai_jd);
    EXPECT_NEAR(tt_jd - tai_jd, TT_TAI_OFFSET / SECONDS_PER_DAY, 1e-9);

    // Roundtrip
    EXPECT_NEAR(tt_to_tai(tt_jd), tai_jd, 1e-9);
}

TEST(TimeScalesTest, TAI_GPS_Conversions) {
    double tai_jd = JD_J2000;
    double gps_jd = tai_to_gps(tai_jd);
    EXPECT_NEAR(gps_jd - tai_jd, GPS_TAI_OFFSET / SECONDS_PER_DAY, 1e-9);

    EXPECT_NEAR(gps_to_tai(gps_jd), tai_jd, 1e-9);
}

TEST(TimeScalesTest, UTC_TAI_Conversions) {
    double utc_jd = calendar_to_jd(2024, 7, 15, 12, 0, 0.0);
    double tai_jd = utc_to_tai(utc_jd);

    // TAI should be ahead of UTC by 37 seconds
    double expected_offset = 37.0 / SECONDS_PER_DAY;
    EXPECT_NEAR(tai_jd - utc_jd, expected_offset, 1e-10);

    // Roundtrip
    EXPECT_NEAR(tai_to_utc(tai_jd), utc_jd, 1e-10);
}

TEST(TimeScalesTest, TT_TDB_Conversions) {
    // TDB differs from TT by up to ~1.7 ms
    double tt_jd = JD_J2000;
    double tdb_jd = tt_to_tdb(tt_jd);

    // At J2000.0, the difference should be small
    double diff_ms = (tdb_jd - tt_jd) * SECONDS_PER_DAY * 1000.0;
    EXPECT_LT(std::abs(diff_ms), 2.0); // Should be < 2 ms

    // Roundtrip (approximate)
    EXPECT_NEAR(tdb_to_tt(tdb_jd), tt_jd, 1e-8);
}

// =============================================================================
// Epoch Tests
// =============================================================================

TEST(EpochTest, DefaultConstruction) {
    NumericEpoch epoch;
    EXPECT_DOUBLE_EQ(epoch.tai_seconds(), 0.0);
    EXPECT_NEAR(epoch.jd_tai(), JD_J2000, 1e-10);
}

TEST(EpochTest, FromUTC) {
    auto epoch = NumericEpoch::from_utc(2024, 7, 15, 12, 30, 0.0);
    auto [y, m, d, h, mi, s] = epoch.to_utc_calendar();
    EXPECT_EQ(y, 2024);
    EXPECT_EQ(m, 7);
    EXPECT_EQ(d, 15);
    EXPECT_EQ(h, 12);
    EXPECT_EQ(mi, 30);
    EXPECT_NEAR(s, 0.0, 1e-4); // ~0.1ms precision due to JD representation
}

TEST(EpochTest, TimeScaleAccessors) {
    auto epoch = NumericEpoch::from_utc(2024, 7, 15, 12, 0, 0.0);

    // All JD accessors should return reasonable values
    EXPECT_GT(epoch.jd_tai(), 2460000.0);
    EXPECT_GT(epoch.jd_tt(), epoch.jd_tai());  // TT > TAI
    EXPECT_LT(epoch.jd_gps(), epoch.jd_tai()); // GPS < TAI
    EXPECT_LT(epoch.jd_utc(), epoch.jd_tai()); // UTC < TAI
}

TEST(EpochTest, Arithmetic) {
    auto epoch1 = NumericEpoch::from_utc(2024, 7, 15, 10, 30,
                                         0.0); // Use 10:30 to avoid edge cases
    auto epoch2 = epoch1 + 3600.0;             // Add one hour

    double diff = epoch2 - epoch1;
    EXPECT_NEAR(diff, 3600.0, 1e-9);

    auto [y, m, d, h, mi, s] = epoch2.to_utc_calendar();
    EXPECT_EQ(h, 11) << "Expected hour 11, got " << h; // Should be 11:30:00

    // In-place subtraction
    epoch2 -= 3600.0;
    EXPECT_NEAR(epoch2 - epoch1, 0.0, 1e-9);
}

TEST(EpochTest, ISOString) {
    auto epoch = NumericEpoch::from_utc(2024, 7, 15, 12, 30, 45.0);
    std::string iso = epoch.to_iso_string();
    EXPECT_EQ(iso.substr(0, 19), "2024-07-15T12:30:45");
}

TEST(EpochTest, GPSWeek) {
    // GPS epoch itself: 1980-01-06 00:00:00 UTC
    auto gps_epoch = NumericEpoch::from_utc(1980, 1, 6, 0, 0, 0.0);
    EXPECT_EQ(gps_epoch.gps_week(), 0);
    EXPECT_NEAR(gps_epoch.gps_seconds_of_week(), 0.0,
                1.0); // Allow 1s tolerance for leap seconds

    // A date in 2024
    auto epoch_2024 = NumericEpoch::from_utc(2024, 7, 15, 12, 0, 0.0);
    int week = epoch_2024.gps_week();
    EXPECT_GT(week, 2300); // Should be > 2300 by 2024
}

// =============================================================================
// GPSTime Tests
// =============================================================================

TEST(GPSTimeTest, FullGPSWeek) {
    // Week 0 + 2 rollovers = week 2048
    EXPECT_EQ(full_gps_week(0, 2), 2048);
    EXPECT_EQ(full_gps_week(100, 2), 2148);
}

TEST(GPSTimeTest, GPSWeekToUTC) {
    // GPS week 0, second 0 should be close to GPS epoch
    double utc_jd = gps_week_to_utc_jd(0, 0.0, 19); // 19 leap seconds in 1980
    double expected = calendar_to_jd(1980, 1, 6, 0, 0, 0.0);
    EXPECT_NEAR(utc_jd, expected, 1e-6);
}

TEST(GPSTimeTest, UTCToGPSWeek) {
    double utc_jd = calendar_to_jd(2024, 7, 15, 12, 0, 0.0);
    auto [week, sow] = utc_jd_to_gps_week_auto(utc_jd);
    EXPECT_GT(week, 2300);
    EXPECT_GE(sow, 0.0);
    EXPECT_LT(sow, SECONDS_PER_WEEK);
}

TEST(GPSTimeTest, GPS_UTCOffset) {
    // In 2024: TAI - UTC = 37, so GPS - UTC = 37 - 19 = 18
    double utc_jd = calendar_to_jd(2024, 7, 15, 12, 0, 0.0);
    int offset = gps_utc_offset(utc_jd);
    EXPECT_EQ(offset, 18);
}

// =============================================================================
// Symbolic Mode Tests
// =============================================================================

TEST(SymbolicTimeTest, EpochArithmetic) {
    auto t = janus::sym("t");
    auto epoch = SymbolicEpoch::from_tai_seconds(t);

    // JD should be symbolic
    auto jd_tt = epoch.jd_tt();

    // Create function to evaluate
    janus::Function f("jd_tt", {t}, {jd_tt});

    // Evaluate at J2000.0 (t = 0)
    auto result = f({0.0});
    double expected_tt_jd = JD_J2000 + TT_TAI_OFFSET / SECONDS_PER_DAY;
    EXPECT_NEAR(result[0](0, 0), expected_tt_jd, 1e-10);

    // Evaluate at t = 86400 (one day)
    auto result_day = f({SECONDS_PER_DAY});
    EXPECT_NEAR(result_day[0](0, 0), expected_tt_jd + 1.0, 1e-10);
}

TEST(SymbolicTimeTest, TimeScaleConversions) {
    auto jd = janus::sym("jd");

    // TAI to TT
    auto tt = tai_to_tt(jd);
    janus::Function f_tt("tai_to_tt", {jd}, {tt});

    double tai_test = JD_J2000;
    auto result = f_tt({tai_test});
    EXPECT_NEAR(result[0](0, 0), tai_test + TT_TAI_OFFSET / SECONDS_PER_DAY,
                1e-10);
}

TEST(SymbolicTimeTest, SymbolicLeapSecondLookup) {
    auto utc_jd = janus::sym("utc_jd");

    // Symbolic UTC to TAI using interpolator
    auto tai_jd = utc_to_tai_symbolic(utc_jd);

    janus::Function f("utc_to_tai", {utc_jd}, {tai_jd});

    // Test at 2024 (delta_at = 37)
    double utc_test = calendar_to_jd(2024, 7, 15, 12, 0, 0.0);
    auto result = f({utc_test});
    double expected = utc_test + 37.0 / SECONDS_PER_DAY;
    EXPECT_NEAR(result[0](0, 0), expected,
                1e-6); // Interpolation has some error
}

TEST(SymbolicTimeTest, DualModeConsistency) {
    // Verify symbolic and numeric give same results
    double tai_jd = calendar_to_jd(2024, 7, 15, 12, 0, 0.0);

    // Numeric computations
    double tt_numeric = tai_to_tt(tai_jd);
    double gps_numeric = tai_to_gps(tai_jd);

    // Symbolic computations
    auto t = janus::sym("t");
    auto tt_sym = tai_to_tt(t);
    auto gps_sym = tai_to_gps(t);

    janus::Function f_tt("tt", {t}, {tt_sym});
    janus::Function f_gps("gps", {t}, {gps_sym});

    auto tt_result = f_tt({tai_jd});
    auto gps_result = f_gps({tai_jd});

    EXPECT_NEAR(tt_result[0](0, 0), tt_numeric, 1e-10);
    EXPECT_NEAR(gps_result[0](0, 0), gps_numeric, 1e-10);
}
