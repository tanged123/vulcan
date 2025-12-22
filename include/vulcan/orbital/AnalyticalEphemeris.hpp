// Vulcan Analytical Ephemeris
// Low-precision Sun and Moon positions using Meeus/Vallado algorithms
#pragma once

#include <janus/janus.hpp>
#include <utility>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/time/JulianDate.hpp>

namespace vulcan::orbital::ephemeris::analytical {

// =============================================================================
// Sun Ephemeris
// =============================================================================

/**
 * @brief Compute Sun's right ascension and declination
 *
 * Low-precision ephemeris from Vallado "Fundamentals of Astrodynamics"
 * Accurate to ~0.01° over several decades around J2000.
 *
 * @tparam Scalar double or casadi::MX
 * @param jd Julian Date (TDB or TT)
 * @return {right_ascension [rad], declination [rad]}
 */
template <typename Scalar>
std::pair<Scalar, Scalar> sun_ra_dec(const Scalar &jd) {
    using janus::asin;
    using janus::atan2;
    using janus::cos;
    using janus::sin;

    // Julian centuries since J2000.0
    const Scalar T = time::jd_to_j2000_centuries(jd);

    // Mean longitude of the Sun [deg -> rad]
    const Scalar lambda_M =
        (280.460 + 36000.771 * T) * constants::angle::deg2rad;

    // Mean anomaly [deg -> rad]
    const Scalar M =
        (357.5291092 + 35999.05034 * T) * constants::angle::deg2rad;

    // Ecliptic longitude [rad]
    const Scalar lambda_ec =
        lambda_M + (1.914666471 * sin(M) + 0.019994643 * sin(2.0 * M)) *
                       constants::angle::deg2rad;

    // Obliquity of the ecliptic [rad]
    const Scalar epsilon =
        (23.439291 - 0.0130042 * T) * constants::angle::deg2rad;

    // Right ascension and declination
    const Scalar sin_lambda = sin(lambda_ec);
    const Scalar cos_lambda = cos(lambda_ec);
    const Scalar cos_eps = cos(epsilon);
    const Scalar sin_eps = sin(epsilon);

    const Scalar ra = atan2(cos_eps * sin_lambda, cos_lambda);
    const Scalar dec = asin(sin_eps * sin_lambda);

    return {ra, dec};
}

/**
 * @brief Earth-Sun distance
 *
 * @tparam Scalar double or casadi::MX
 * @param jd Julian Date
 * @return Distance [m]
 */
template <typename Scalar> Scalar sun_distance(const Scalar &jd) {
    const Scalar T = time::jd_to_j2000_centuries(jd);
    const Scalar M =
        (357.5291092 + 35999.05034 * T) * constants::angle::deg2rad;

    // Distance in AU
    const Scalar r_au = 1.000140612 - 0.016708617 * janus::cos(M) -
                        0.000139589 * janus::cos(2.0 * M);

    return r_au * constants::sun::AU;
}

/**
 * @brief Sun position in ECI frame (J2000 equatorial)
 *
 * @tparam Scalar double or casadi::MX
 * @param jd Julian Date
 * @return Position vector [m]
 */
template <typename Scalar> Vec3<Scalar> sun_position_eci(const Scalar &jd) {
    auto [ra, dec] = sun_ra_dec(jd);
    const Scalar r = sun_distance(jd);

    const Scalar cos_dec = janus::cos(dec);

    Vec3<Scalar> pos;
    pos(0) = r * cos_dec * janus::cos(ra);
    pos(1) = r * cos_dec * janus::sin(ra);
    pos(2) = r * janus::sin(dec);

    return pos;
}

/**
 * @brief Sun unit direction vector in ECI
 *
 * @tparam Scalar double or casadi::MX
 * @param jd Julian Date
 * @return Unit vector pointing from Earth to Sun
 */
template <typename Scalar> Vec3<Scalar> sun_unit_vector_eci(const Scalar &jd) {
    auto [ra, dec] = sun_ra_dec(jd);
    const Scalar cos_dec = janus::cos(dec);

    Vec3<Scalar> u;
    u(0) = cos_dec * janus::cos(ra);
    u(1) = cos_dec * janus::sin(ra);
    u(2) = janus::sin(dec);

    return u;
}

// =============================================================================
// Moon Ephemeris
// =============================================================================

/**
 * @brief Moon position in ECI (geocentric equatorial)
 *
 * Computes the Moon's position using a simplified analytical model
 * based on Meeus, "Astronomical Algorithms" (1998), Chapter 47.
 *
 * Accuracy: ~0.3° in longitude, ~10 km in position
 *
 * @tparam Scalar double or casadi::MX
 * @param jd Julian Date (TDB or TT)
 * @return Moon position in ECI (J2000) [m]
 */
template <typename Scalar> Vec3<Scalar> moon_position_eci(const Scalar &jd) {
    // Julian centuries from J2000.0
    const Scalar T = time::jd_to_j2000_centuries(jd);
    const Scalar T2 = T * T;
    const Scalar T3 = T2 * T;
    const Scalar T4 = T3 * T;

    // Moon's mean longitude (degrees)
    const Scalar Lp = 218.3164477 + 481267.88123421 * T - 0.0015786 * T2 +
                      T3 / 538841.0 - T4 / 65194000.0;

    // Mean elongation of the Moon (degrees)
    const Scalar D = 297.8501921 + 445267.1114034 * T - 0.0018819 * T2 +
                     T3 / 545868.0 - T4 / 113065000.0;

    // Sun's mean anomaly (degrees)
    const Scalar M =
        357.5291092 + 35999.0502909 * T - 0.0001536 * T2 + T3 / 24490000.0;

    // Moon's mean anomaly (degrees)
    const Scalar Mp = 134.9633964 + 477198.8675055 * T + 0.0087414 * T2 +
                      T3 / 69699.0 - T4 / 14712000.0;

    // Moon's argument of latitude (degrees)
    const Scalar F = 93.2720950 + 483202.0175233 * T - 0.0036539 * T2 -
                     T3 / 3526000.0 + T4 / 863310000.0;

    // Convert to radians
    const Scalar D_rad = D * constants::angle::deg2rad;
    const Scalar M_rad = M * constants::angle::deg2rad;
    const Scalar Mp_rad = Mp * constants::angle::deg2rad;
    const Scalar F_rad = F * constants::angle::deg2rad;

    // Longitude perturbations (simplified - main terms only)
    const Scalar dL = 6288774.0 * janus::sin(Mp_rad) +
                      1274027.0 * janus::sin(2.0 * D_rad - Mp_rad) +
                      658314.0 * janus::sin(2.0 * D_rad) +
                      213618.0 * janus::sin(2.0 * Mp_rad) -
                      185116.0 * janus::sin(M_rad) -
                      114332.0 * janus::sin(2.0 * F_rad);

    // Latitude perturbations (simplified)
    const Scalar dB = 5128122.0 * janus::sin(F_rad) +
                      280602.0 * janus::sin(Mp_rad + F_rad) +
                      277693.0 * janus::sin(Mp_rad - F_rad) +
                      173237.0 * janus::sin(2.0 * D_rad - F_rad);

    // Distance perturbations (simplified)
    const Scalar dR = -20905355.0 * janus::cos(Mp_rad) -
                      3699111.0 * janus::cos(2.0 * D_rad - Mp_rad) -
                      2955968.0 * janus::cos(2.0 * D_rad) -
                      569925.0 * janus::cos(2.0 * Mp_rad);

    // Ecliptic longitude and latitude (degrees)
    const Scalar lambda = Lp + dL / 1000000.0;
    const Scalar beta = dB / 1000000.0;

    // Distance (km, then convert to m)
    const Scalar dist_km = 385000.56 + dR / 1000.0;
    const Scalar dist = dist_km * 1000.0;

    // Convert to radians
    const Scalar lambda_rad = lambda * constants::angle::deg2rad;
    const Scalar beta_rad = beta * constants::angle::deg2rad;

    // Mean obliquity of the ecliptic
    const Scalar epsilon =
        (23.439291 - 0.0130042 * T) * constants::angle::deg2rad;

    // Ecliptic to equatorial transformation
    const Scalar cos_lambda = janus::cos(lambda_rad);
    const Scalar sin_lambda = janus::sin(lambda_rad);
    const Scalar cos_beta = janus::cos(beta_rad);
    const Scalar sin_beta = janus::sin(beta_rad);
    const Scalar cos_eps = janus::cos(epsilon);
    const Scalar sin_eps = janus::sin(epsilon);

    Vec3<Scalar> r_moon;
    r_moon(0) = dist * cos_beta * cos_lambda;
    r_moon(1) = dist * (cos_eps * cos_beta * sin_lambda - sin_eps * sin_beta);
    r_moon(2) = dist * (sin_eps * cos_beta * sin_lambda + cos_eps * sin_beta);

    return r_moon;
}

/**
 * @brief Moon distance from Earth
 *
 * @tparam Scalar double or casadi::MX
 * @param jd Julian Date
 * @return Distance [m]
 */
template <typename Scalar> Scalar moon_distance(const Scalar &jd) {
    Vec3<Scalar> pos = moon_position_eci(jd);
    return janus::norm(pos);
}

// =============================================================================
// ECEF Conversions
// =============================================================================

/**
 * @brief Sun position in ECEF
 *
 * @tparam Scalar double or casadi::MX
 * @param jd Julian Date
 * @return Sun position in ECEF [m]
 */
template <typename Scalar> Vec3<Scalar> sun_position_ecef(const Scalar &jd) {
    Vec3<Scalar> r_eci = sun_position_eci(jd);

    // Greenwich Mean Sidereal Time
    const Scalar T = time::jd_to_j2000_centuries(jd);
    const Scalar gmst_deg = 280.46061837 + 360.98564736629 * (jd - 2451545.0) +
                            0.000387933 * T * T - T * T * T / 38710000.0;
    const Scalar gmst_rad = gmst_deg * constants::angle::deg2rad;

    // Rotate ECI to ECEF
    const Scalar cos_gmst = janus::cos(gmst_rad);
    const Scalar sin_gmst = janus::sin(gmst_rad);

    Vec3<Scalar> r_ecef;
    r_ecef(0) = cos_gmst * r_eci(0) + sin_gmst * r_eci(1);
    r_ecef(1) = -sin_gmst * r_eci(0) + cos_gmst * r_eci(1);
    r_ecef(2) = r_eci(2);

    return r_ecef;
}

/**
 * @brief Moon position in ECEF
 *
 * @tparam Scalar double or casadi::MX
 * @param jd Julian Date
 * @return Moon position in ECEF [m]
 */
template <typename Scalar> Vec3<Scalar> moon_position_ecef(const Scalar &jd) {
    Vec3<Scalar> r_eci = moon_position_eci(jd);

    const Scalar T = time::jd_to_j2000_centuries(jd);
    const Scalar gmst_deg = 280.46061837 + 360.98564736629 * (jd - 2451545.0) +
                            0.000387933 * T * T - T * T * T / 38710000.0;
    const Scalar gmst_rad = gmst_deg * constants::angle::deg2rad;

    const Scalar cos_gmst = janus::cos(gmst_rad);
    const Scalar sin_gmst = janus::sin(gmst_rad);

    Vec3<Scalar> r_ecef;
    r_ecef(0) = cos_gmst * r_eci(0) + sin_gmst * r_eci(1);
    r_ecef(1) = -sin_gmst * r_eci(0) + cos_gmst * r_eci(1);
    r_ecef(2) = r_eci(2);

    return r_ecef;
}

} // namespace vulcan::orbital::ephemeris::analytical
