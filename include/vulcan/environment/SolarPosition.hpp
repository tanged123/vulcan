// Solar Position Ephemeris
// Low-precision approximate Sun position using algorithm from Vallado
#pragma once

#include <janus/janus.hpp>
#include <utility>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/time/JulianDate.hpp>

namespace vulcan::environment::solar {

namespace constants {
/// Astronomical Unit in meters
inline constexpr double AU = 149597870700.0;
} // namespace constants

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
template <typename Scalar> std::pair<Scalar, Scalar> ra_dec(const Scalar &jd) {
    using janus::asin;
    using janus::atan2;
    using janus::cos;
    using janus::sin;

    // Julian centuries since J2000.0
    const Scalar T = time::jd_to_j2000_centuries(jd);

    // Mean longitude of the Sun [deg -> rad]
    const Scalar lambda_M =
        (280.460 + 36000.771 * T) * vulcan::constants::angle::deg2rad;

    // Mean anomaly [deg -> rad]
    const Scalar M =
        (357.5291092 + 35999.05034 * T) * vulcan::constants::angle::deg2rad;

    // Ecliptic longitude [rad]
    const Scalar lambda_ec =
        lambda_M + (1.914666471 * sin(M) + 0.019994643 * sin(2.0 * M)) *
                       vulcan::constants::angle::deg2rad;

    // Obliquity of the ecliptic [rad]
    const Scalar epsilon =
        (23.439291 - 0.0130042 * T) * vulcan::constants::angle::deg2rad;

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
 * @brief Right ascension of the Sun
 * @param jd Julian Date
 * @return Right ascension [rad]
 */
template <typename Scalar> Scalar right_ascension(const Scalar &jd) {
    return ra_dec(jd).first;
}

/**
 * @brief Declination of the Sun
 * @param jd Julian Date
 * @return Declination [rad]
 */
template <typename Scalar> Scalar declination(const Scalar &jd) {
    return ra_dec(jd).second;
}

/**
 * @brief Earth-Sun distance
 * @param jd Julian Date
 * @return Distance [m]
 */
template <typename Scalar> Scalar distance(const Scalar &jd) {
    const Scalar T = time::jd_to_j2000_centuries(jd);
    const Scalar M =
        (357.5291092 + 35999.05034 * T) * vulcan::constants::angle::deg2rad;

    // Distance in AU
    const Scalar r_au = 1.000140612 - 0.016708617 * janus::cos(M) -
                        0.000139589 * janus::cos(2.0 * M);

    return r_au * constants::AU;
}

/**
 * @brief Sun position in ECI frame (J2000 equatorial)
 * @param jd Julian Date
 * @return Position vector [m]
 */
template <typename Scalar> Vec3<Scalar> position_eci(const Scalar &jd) {
    auto [ra, dec] = ra_dec(jd);
    const Scalar r = distance(jd);

    const Scalar cos_dec = janus::cos(dec);

    Vec3<Scalar> pos;
    pos(0) = r * cos_dec * janus::cos(ra);
    pos(1) = r * cos_dec * janus::sin(ra);
    pos(2) = r * janus::sin(dec);

    return pos;
}

/**
 * @brief Sun unit direction vector in ECI
 * @param jd Julian Date
 * @return Unit vector pointing from Earth to Sun
 */
template <typename Scalar> Vec3<Scalar> unit_vector_eci(const Scalar &jd) {
    auto [ra, dec] = ra_dec(jd);
    const Scalar cos_dec = janus::cos(dec);

    Vec3<Scalar> u;
    u(0) = cos_dec * janus::cos(ra);
    u(1) = cos_dec * janus::sin(ra);
    u(2) = janus::sin(dec);

    return u;
}

} // namespace vulcan::environment::solar
