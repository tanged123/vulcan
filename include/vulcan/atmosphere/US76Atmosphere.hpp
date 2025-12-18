// US Standard Atmosphere 1976 - Table-Based Implementation
// Vetted reference data embedded as constexpr arrays for table interpolation
#pragma once

#include <array>
#include <cstddef>
#include <vulcan/core/TableInterpolator.hpp>

namespace vulcan::us76 {

// ============================================================================
// Embedded Reference Data (US Standard Atmosphere 1976)
// ============================================================================
// Data source: reference/atmos_temp.txt
// Fine resolution (0.5 km) from -0.5 to 20 km, coarse (5-50 km) from 25-1000 km
// Total: 93 data points

namespace detail {

// Number of table entries
inline constexpr std::size_t N_POINTS = 94;

// Geometric altitude [km]
inline constexpr std::array<double, N_POINTS> altitude_km = {
    -0.5, 0.0,  0.5,  1.0,  1.5,  2.0,  2.5,  3.0,  3.5,  4.0,  4.5,  5.0,
    5.5,  6.0,  6.5,  7.0,  7.5,  8.0,  8.5,  9.0,  9.5,  10.0, 10.5, 11.0,
    11.5, 12.0, 12.5, 13.0, 13.5, 14.0, 14.5, 15.0, 15.5, 16.0, 16.5, 17.0,
    17.5, 18.0, 18.5, 19.0, 19.5, 20.0, 25,   30,   35,   40,   45,   50,
    55,   60,   65,   70,   75,   80,   85,   90,   95,   100,  105,  110,
    115,  120,  125,  130,  135,  140,  145,  150,  155,  160,  165,  170,
    175,  180,  185,  190,  195,  200,  250,  300,  350,  400,  450,  500,
    550,  600,  650,  700,  750,  800,  850,  900,  950,  1000};

// Temperature [K]
inline constexpr std::array<double, N_POINTS> temperature_K = {
    291.4,   288.1,   284.9,   281.7,   278.4,   275.2,   271.9,   268.7,
    265.4,   262.2,   258.9,   255.7,   252.4,   249.2,   245.9,   242.7,
    239.5,   236.2,   233.0,   229.7,   226.5,   223.3,   220.0,   216.8,
    216.6,   216.6,   216.6,   216.6,   216.6,   216.6,   216.6,   216.6,
    216.6,   216.6,   216.6,   216.6,   216.6,   216.6,   216.6,   216.6,
    216.6,   216.6,   221.552, 226.509, 236.513, 250.350, 264.164, 270.650,
    260.771, 247.021, 233.292, 219.585, 208.399, 198.639, 188.893, 186.867,
    188.418, 195.081, 208.835, 240.000, 300.000, 360.000, 417.231, 469.268,
    516.589, 559.627, 598.776, 634.392, 666.799, 696.290, 723.132, 747.566,
    769.811, 790.066, 808.511, 825.312, 840.616, 854.559, 941.330, 976.008,
    990.057, 995.825, 998.225, 999.236, 999.667, 999.853, 999.934, 999.970,
    999.986, 999.994, 999.997, 999.999, 999.999, 1000.000};

// Pressure [Pa]
inline constexpr std::array<double, N_POINTS> pressure_Pa = {
    107477,  101325,  95461,   89876,   84559,   79501,   74691,   70121,
    65780,   61660,   57752,   54048,   50539,   47217,   44075,   41105,
    38299,   35651,   33154,   30800,   28584,   26499,   24540,   22699,
    20984,   19399,   17933,   16579,   15327,   14170,   13100,   12111,
    11197,   10352,   9571,    8849,    8182,    7565,    6994,    6467,
    5979,    5529,    2550,    1200,    575,     287,     149,     79.8,
    42.5,    22.0,    10.9,    5.22,    2.39,    1.05,    0.446,   0.184,
    0.0758,  0.0320,  0.0144,  0.00715, 0.00400, 0.00254, 0.00174, 0.00125,
    9.36e-4, 7.20e-4, 5.67e-4, 4.54e-4, 3.69e-4, 3.04e-4, 2.53e-4, 2.12e-4,
    1.79e-4, 1.53e-4, 1.31e-4, 1.13e-4, 9.75e-5, 8.47e-5, 2.48e-5, 8.77e-6,
    3.44e-6, 1.45e-6, 6.45e-7, 3.02e-7, 1.51e-7, 8.21e-8, 4.89e-8, 3.19e-8,
    2.26e-8, 1.70e-8, 1.34e-8, 1.09e-8, 9.00e-9, 7.51e-9};

// Density [kg/m³]
inline constexpr std::array<double, N_POINTS> density_kgm3 = {
    1.285,    1.225,    1.167,    1.112,    1.058,    1.007,    0.957,
    0.909,    0.863,    0.819,    0.777,    0.736,    0.697,    0.660,
    0.624,    0.590,    0.557,    0.526,    0.496,    0.467,    0.440,
    0.414,    0.389,    0.365,    0.337,    0.312,    0.288,    0.267,
    0.246,    0.228,    0.211,    0.195,    0.180,    0.166,    0.154,
    0.142,    0.132,    0.122,    0.112,    0.104,    0.096,    0.089,
    4.01e-2,  1.84e-2,  8.46e-3,  4.00e-3,  1.97e-3,  1.03e-3,  5.68e-4,
    3.10e-4,  1.63e-4,  8.28e-5,  3.99e-5,  1.85e-5,  8.22e-6,  3.44e-6,
    1.39e-6,  5.60e-7,  2.33e-7,  9.67e-8,  4.28e-8,  2.22e-8,  1.29e-8,
    8.15e-9,  5.46e-9,  3.83e-9,  2.78e-9,  2.08e-9,  1.58e-9,  1.23e-9,
    9.75e-10, 7.82e-10, 6.34e-10, 5.19e-10, 4.30e-10, 3.58e-10, 3.01e-10,
    2.54e-10, 6.07e-11, 1.92e-11, 7.00e-12, 2.80e-12, 1.18e-12, 5.21e-13,
    2.38e-13, 1.14e-13, 5.71e-14, 3.07e-14, 1.79e-14, 1.14e-14, 7.82e-15,
    5.76e-15, 4.45e-15, 3.56e-15};

// Speed of sound [m/s]
inline constexpr std::array<double, N_POINTS> speed_of_sound_ms = {
    342.2,  340.3,  338.4,  336.4,  334.5,  332.5,  330.6,  328.6,  326.6,
    324.6,  322.6,  320.5,  318.5,  316.5,  314.4,  312.3,  310.2,  308.1,
    306.0,  303.8,  301.7,  299.5,  297.4,  295.2,  295.1,  295.1,  295.1,
    295.1,  295.1,  295.1,  295.1,  295.1,  295.1,  295.1,  295.1,  295.1,
    295.1,  295.1,  295.1,  295.1,  295.1,  295.1,  298.39, 301.71, 308.30,
    317.19, 325.82, 329.80, 323.72, 315.07, 306.19, 297.06, 289.40, 282.54,
    275.52, 274.04, 275.17, 280.00, 289.70, 310.56, 347.22, 380.36, 409.48,
    434.27, 455.64, 474.24, 490.54, 504.92, 517.66, 528.98, 539.08, 548.11,
    556.21, 563.48, 570.02, 575.91, 581.22, 586.02, 615.06, 626.28, 630.78,
    632.61, 633.37, 633.69, 633.83, 633.89, 633.91, 633.93, 633.93, 633.93,
    633.93, 633.93, 633.94, 633.94};

// Gravitational acceleration [m/s²]
inline constexpr std::array<double, N_POINTS> gravity_ms2 = {
    9.810, 9.807, 9.804, 9.801, 9.798, 9.795, 9.792, 9.789, 9.786, 9.782, 9.779,
    9.776, 9.773, 9.770, 9.767, 9.764, 9.761, 9.757, 9.754, 9.751, 9.748, 9.745,
    9.741, 9.738, 9.735, 9.732, 9.728, 9.725, 9.722, 9.719, 9.715, 9.712, 9.709,
    9.706, 9.702, 9.699, 9.696, 9.693, 9.689, 9.686, 9.683, 9.680, 9.730, 9.715,
    9.700, 9.684, 9.669, 9.654, 9.639, 9.624, 9.609, 9.594, 9.579, 9.564, 9.550,
    9.535, 9.520, 9.505, 9.491, 9.476, 9.461, 9.447, 9.432, 9.418, 9.403, 9.389,
    9.374, 9.360, 9.345, 9.331, 9.317, 9.302, 9.288, 9.274, 9.260, 9.246, 9.232,
    9.218, 9.079, 8.943, 8.810, 8.680, 8.553, 8.429, 8.307, 8.188, 8.072, 7.958,
    7.846, 7.737, 7.630, 7.525, 7.422, 7.322};

// Convert std::array to Eigen::Map for Table1D
inline janus::NumericVector altitude_m_vec() {
    // Convert km to m and return as vector
    janus::NumericVector v(N_POINTS);
    for (std::size_t i = 0; i < N_POINTS; ++i) {
        v(static_cast<Eigen::Index>(i)) = altitude_km[i] * 1000.0;
    }
    return v;
}

template <std::size_t N>
inline janus::NumericVector array_to_vec(const std::array<double, N> &arr) {
    janus::NumericVector v(N);
    for (std::size_t i = 0; i < N; ++i) {
        v(static_cast<Eigen::Index>(i)) = arr[i];
    }
    return v;
}

// Lazy-initialized interpolation tables (constructed on first use)
inline const Table1D &temperature_table() {
    static Table1D table(altitude_m_vec(), array_to_vec(temperature_K));
    return table;
}

inline const Table1D &pressure_table() {
    static Table1D table(altitude_m_vec(), array_to_vec(pressure_Pa));
    return table;
}

inline const Table1D &density_table() {
    static Table1D table(altitude_m_vec(), array_to_vec(density_kgm3));
    return table;
}

inline const Table1D &speed_of_sound_table() {
    static Table1D table(altitude_m_vec(), array_to_vec(speed_of_sound_ms));
    return table;
}

inline const Table1D &gravity_table() {
    static Table1D table(altitude_m_vec(), array_to_vec(gravity_ms2));
    return table;
}

} // namespace detail

// ============================================================================
// Public API - US Standard Atmosphere 1976 (Table-Based)
// ============================================================================

/**
 * @brief US Standard Atmosphere 1976 - Temperature (table-based)
 *
 * Interpolated from vetted reference data covering -500m to 1000km.
 * Values outside this range are clamped to boundary values.
 *
 * @tparam Scalar Type for scalar operations (double or casadi::MX)
 * @param altitude Geometric altitude [m]
 * @return Temperature [K]
 */
template <typename Scalar> Scalar temperature(const Scalar &altitude) {
    return detail::temperature_table()(altitude);
}

/**
 * @brief US Standard Atmosphere 1976 - Pressure (table-based)
 *
 * Interpolated from vetted reference data covering -500m to 1000km.
 *
 * @tparam Scalar Type for scalar operations (double or casadi::MX)
 * @param altitude Geometric altitude [m]
 * @return Pressure [Pa]
 */
template <typename Scalar> Scalar pressure(const Scalar &altitude) {
    return detail::pressure_table()(altitude);
}

/**
 * @brief US Standard Atmosphere 1976 - Density (table-based)
 *
 * Interpolated from vetted reference data covering -500m to 1000km.
 *
 * @tparam Scalar Type for scalar operations (double or casadi::MX)
 * @param altitude Geometric altitude [m]
 * @return Density [kg/m³]
 */
template <typename Scalar> Scalar density(const Scalar &altitude) {
    return detail::density_table()(altitude);
}

/**
 * @brief US Standard Atmosphere 1976 - Speed of Sound (table-based)
 *
 * Interpolated from vetted reference data covering -500m to 1000km.
 *
 * @tparam Scalar Type for scalar operations (double or casadi::MX)
 * @param altitude Geometric altitude [m]
 * @return Speed of sound [m/s]
 */
template <typename Scalar> Scalar speed_of_sound(const Scalar &altitude) {
    return detail::speed_of_sound_table()(altitude);
}

/**
 * @brief US Standard Atmosphere 1976 - Gravitational Acceleration (table-based)
 *
 * Interpolated from vetted reference data covering -500m to 1000km.
 * Note: This is the effective gravity at altitude, accounting for altitude.
 *
 * @tparam Scalar Type for scalar operations (double or casadi::MX)
 * @param altitude Geometric altitude [m]
 * @return Gravitational acceleration [m/s²]
 */
template <typename Scalar> Scalar gravity(const Scalar &altitude) {
    return detail::gravity_table()(altitude);
}

// ============================================================================
// Altitude Bounds
// ============================================================================

/// Minimum supported altitude [m]
inline constexpr double MIN_ALTITUDE = -500.0;

/// Maximum supported altitude [m]
inline constexpr double MAX_ALTITUDE = 1000000.0;

} // namespace vulcan::us76
