/**
 * @file mission_planning.cpp
 * @brief Multi-month mission optimization with symbolic leap second handling
 *
 * This example demonstrates Vulcan's symbolic leap second interpolation
 * across a mission spanning multiple months - where leap seconds actually
 * matter for precise timing. The janus::Interpolator provides smooth,
 * differentiable leap second lookup for gradient-based optimization.
 *
 * Scenario: Plan observation windows for a 6-month Earth observation campaign
 * where ground station visibility depends on time of day and seasonal effects.
 */

#include <cmath>
#include <iomanip>
#include <iostream>
#include <janus/janus.hpp>
#include <vector>
#include <vulcan/vulcan.hpp>

using namespace vulcan::time;
using namespace vulcan::constants::time;

/**
 * @brief Ground station visibility model
 *
 * A ground station at given latitude has visibility windows that depend on:
 * - Local solar time (station must be on dark side for optical observations)
 * - Season (affects day length at high latitudes)
 *
 * @tparam Scalar Numeric or symbolic type
 * @param utc_jd UTC Julian Date (symbolic or numeric)
 * @param station_lat Station latitude [radians]
 * @return Visibility score [0, 1]
 */
template <typename Scalar>
Scalar ground_station_visibility(const Scalar &utc_jd, double station_lat) {
    // Convert UTC to TAI using symbolic interpolator (key feature!)
    Scalar tai_jd = utc_to_tai_symbolic(utc_jd);

    // Compute local hour angle (simplified: assume station at prime meridian)
    Scalar days_since_j2000 = tai_jd - JD_J2000;
    Scalar local_hour = janus::fmod(days_since_j2000, Scalar(1.0)) * 24.0;

    // Night visibility: peaks at local midnight (hour = 0 or 24)
    // Use smooth cosine model
    Scalar hour_angle = (local_hour - Scalar(12.0)) * M_PI / 12.0;
    Scalar night_factor = (Scalar(1.0) + janus::cos(hour_angle)) * Scalar(0.5);

    // Seasonal factor: more visible in winter (longer nights)
    // Use day-of-year to estimate season
    Scalar doy = janus::fmod(days_since_j2000, Scalar(365.25));
    // Winter solstice around day 355, summer around day 172
    Scalar season_angle = (doy - Scalar(355.0)) * 2.0 * M_PI / 365.25;
    Scalar season_factor =
        (Scalar(1.0) + janus::cos(season_angle)) * Scalar(0.5);

    // High-latitude stations have more extreme seasonal variation
    double lat_effect = std::abs(std::sin(station_lat));
    Scalar visibility =
        night_factor * (Scalar(1.0 - 0.5 * lat_effect) +
                        Scalar(0.5 * lat_effect) * season_factor);

    return visibility;
}

/**
 * @brief Satellite revisit model
 *
 * Models when a satellite crosses over the target area based on orbital period.
 *
 * @tparam Scalar Numeric or symbolic type
 * @param utc_jd UTC Julian Date
 * @param orbit_period Orbital period [days]
 * @param phase_offset Initial phase offset
 * @return Satellite overhead factor [0, 1]
 */
template <typename Scalar>
Scalar satellite_overhead(const Scalar &utc_jd, double orbit_period,
                          double phase_offset) {
    Scalar tai_jd = utc_to_tai_symbolic(utc_jd);
    Scalar orbit_phase = (tai_jd - JD_J2000) / orbit_period + phase_offset;
    Scalar phase_wrap = janus::fmod(orbit_phase, Scalar(1.0));

    // Satellite is overhead when phase is near 0 or 1
    // Use narrow Gaussian window
    Scalar dist_from_overhead = janus::abs(phase_wrap - Scalar(0.5));
    Scalar overhead_factor =
        janus::exp(-dist_from_overhead * dist_from_overhead * 100.0);

    return overhead_factor;
}

/**
 * @brief Mission objective: maximize total data downlink
 *
 * Combines ground station visibility and satellite overhead.
 */
template <typename Scalar>
Scalar mission_objective(const Scalar &mission_start_jd,
                         double mission_duration_days, double station_lat,
                         double orbit_period) {
    // Integrate visibility over mission duration
    // Use trapezoidal approximation with N sample points
    constexpr int N = 20;
    Scalar total_visibility = 0.0;

    for (int i = 0; i <= N; ++i) {
        Scalar t = mission_start_jd + (mission_duration_days * i) / N;
        Scalar gs_vis = ground_station_visibility(t, station_lat);
        Scalar sat_vis = satellite_overhead(t, orbit_period, 0.0);
        Scalar combined = gs_vis * sat_vis;

        // Trapezoidal rule weights
        double weight = (i == 0 || i == N) ? 0.5 : 1.0;
        total_visibility = total_visibility + combined * weight;
    }

    return total_visibility;
}

int main() {
    std::cout
        << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout
        << "║   Multi-Month Mission Planning with Symbolic Leap Seconds  ║\n";
    std::cout
        << "╚════════════════════════════════════════════════════════════╝\n\n";

    // =========================================================================
    // Part 1: Demonstrate symbolic leap second lookup
    // =========================================================================
    std::cout << "=== Part 1: Symbolic Leap Second Interpolation ===\n\n";

    // Show leap second table usage
    std::cout << "Leap second table (IERS Bulletin C):\n";
    std::cout << "  Total entries: " << LEAP_SECOND_TABLE.size() << "\n";
    std::cout << "  Range: 1972-01-01 to 2017-01-01\n";
    std::cout << "  Current TAI-UTC: " << LEAP_SECOND_TABLE.back().delta_at
              << " seconds\n\n";

    // Create symbolic UTC JD and show interpolated leap seconds
    auto utc_jd_sym = janus::sym("utc_jd");
    auto delta_at_sym = leap_seconds_symbolic(utc_jd_sym);

    janus::Function f_leap("leap_seconds", {utc_jd_sym}, {delta_at_sym});

    std::cout << "Symbolic leap second lookup (via janus::Interpolator):\n";
    std::vector<std::tuple<int, int, int>> test_dates = {
        {2010, 1, 1},  // delta_at = 34
        {2012, 7, 1},  // delta_at = 35
        {2015, 7, 1},  // delta_at = 36
        {2017, 1, 1},  // delta_at = 37
        {2024, 7, 15}, // delta_at = 37 (extrapolated)
    };

    for (const auto &[y, m, d] : test_dates) {
        double utc_jd = calendar_to_jd(y, m, d, 12, 0, 0.0);
        auto result = f_leap({utc_jd});
        std::cout << "  " << y << "-" << std::setw(2) << std::setfill('0') << m
                  << "-" << std::setw(2) << d << std::setfill(' ')
                  << ": TAI-UTC = " << std::fixed << std::setprecision(2)
                  << result[0](0, 0) << " s\n";
    }

    // =========================================================================
    // Part 2: Symbolic UTC to TAI conversion
    // =========================================================================
    std::cout << "\n=== Part 2: Symbolic UTC→TAI Conversion ===\n\n";

    auto tai_jd_sym = utc_to_tai_symbolic(utc_jd_sym);
    janus::Function f_utc_tai("utc_to_tai", {utc_jd_sym}, {tai_jd_sym});

    std::cout << "UTC to TAI conversion (fully symbolic with interpolated leap "
                 "seconds):\n";
    for (const auto &[y, m, d] : test_dates) {
        double utc_jd = calendar_to_jd(y, m, d, 12, 0, 0.0);
        auto result = f_utc_tai({utc_jd});
        double offset_sec = (result[0](0, 0) - utc_jd) * SECONDS_PER_DAY;
        std::cout << "  " << y << "-" << std::setw(2) << std::setfill('0') << m
                  << "-" << std::setw(2) << d << std::setfill(' ')
                  << ": TAI = UTC + " << std::fixed << std::setprecision(2)
                  << offset_sec << " s\n";
    }

    // =========================================================================
    // Part 3: Multi-month mission optimization
    // =========================================================================
    std::cout << "\n=== Part 3: 6-Month Mission Optimization ===\n\n";

    // Mission parameters
    double station_lat = 65.0 * M_PI / 180.0; // Kiruna, Sweden (65°N)
    double orbit_period = 1.5; // Satellite revisits every 1.5 days

    std::cout << "Mission parameters:\n";
    std::cout << "  Ground station: Kiruna, Sweden (65°N)\n";
    std::cout << "  Satellite revisit period: " << orbit_period << " days\n";
    std::cout << "  Mission duration: 180 days (6 months)\n\n";

    janus::Opti opti;

    // Decision variable: mission start date (as JD offset from 2024-01-01)
    double base_jd = calendar_to_jd(2024, 1, 1, 0, 0, 0.0);
    auto start_offset = opti.variable(0.0); // Days from Jan 1, 2024

    // Mission duration is fixed at 180 days
    double duration = 180.0;

    // Symbolic mission objective using leap second interpolation
    auto start_jd = base_jd + start_offset;
    auto objective =
        mission_objective(start_jd, duration, station_lat, orbit_period);

    // Maximize total visibility (minimize negative)
    opti.minimize(-objective);

    // Constraint: mission must start within 2024
    opti.subject_to(start_offset >= 0.0);
    opti.subject_to(start_offset <=
                    365.0 - duration); // Room for 180-day mission

    std::cout
        << "Optimizing mission start date for maximum data collection...\n\n";

    auto solution = opti.solve({.verbose = false});

    double optimal_offset = solution.value(start_offset);
    double optimal_obj = solution.value(objective);

    // Convert to calendar
    double optimal_start_jd = base_jd + optimal_offset;
    auto [y1, m1, d1, h1, mi1, s1] = jd_to_calendar(optimal_start_jd);
    auto [y2, m2, d2, h2, mi2, s2] =
        jd_to_calendar(optimal_start_jd + duration);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Optimal mission window:\n";
    std::cout << "  Start: " << y1 << "-" << std::setw(2) << std::setfill('0')
              << m1 << "-" << std::setw(2) << d1 << std::setfill(' ') << "\n";
    std::cout << "  End:   " << y2 << "-" << std::setw(2) << std::setfill('0')
              << m2 << "-" << std::setw(2) << d2 << std::setfill(' ') << "\n";
    std::cout << "  Total visibility score: " << optimal_obj << "\n";
    std::cout << "  Solver iterations: " << solution.num_iterations() << "\n";

    // =========================================================================
    // Part 4: Compare start dates
    // =========================================================================
    std::cout << "\n=== Part 4: Mission Start Date Comparison ===\n\n";

    std::cout << "Start Month | Visibility Score | Season\n";
    std::cout << "------------|------------------|--------\n";

    janus::Function f_obj("mission_obj", {start_offset},
                          {mission_objective(base_jd + start_offset, duration,
                                             station_lat, orbit_period)});

    for (int month = 1; month <= 12; month += 2) {
        double month_jd = calendar_to_jd(2024, month, 1, 0, 0, 0.0);
        double offset = month_jd - base_jd;

        // Skip if mission would extend past year
        if (offset + duration > 365.0)
            continue;

        auto result = f_obj({offset});
        double score = result[0](0, 0);

        std::string season;
        if (month <= 2 || month >= 11)
            season = "Winter";
        else if (month <= 5)
            season = "Spring";
        else if (month <= 8)
            season = "Summer";
        else
            season = "Autumn";

        std::cout << std::setw(11) << month << " | " << std::setw(16)
                  << std::fixed << std::setprecision(4) << score << " | "
                  << season << "\n";
    }

    // =========================================================================
    // Part 5: Export computational graph
    // =========================================================================
    std::cout << "\n=== Part 5: Exporting Computational Graph ===\n\n";

    // Show the leap second lookup graph
    janus::export_graph_html(delta_at_sym, "graph_leap_seconds",
                             "Leap_Second_Interpolator");
    std::cout << "✓ Exported: graph_leap_seconds.html\n";

    // Show the full UTC to TAI conversion
    janus::export_graph_html(tai_jd_sym, "graph_utc_to_tai",
                             "UTC_to_TAI_Symbolic");
    std::cout << "✓ Exported: graph_utc_to_tai.html\n";

    // Ground station visibility model
    auto gs_vis_sym = ground_station_visibility(utc_jd_sym, station_lat);
    janus::export_graph_html(gs_vis_sym, "graph_ground_station",
                             "Ground_Station_Visibility");
    std::cout << "✓ Exported: graph_ground_station.html\n";

    std::cout << "\n✓ Multi-month mission planning complete!\n";
    std::cout << "  The optimization used symbolic leap second interpolation\n";
    std::cout
        << "  to maintain differentiability across the 6-month timespan.\n";

    return 0;
}
