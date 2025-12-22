// Earth Magnetic Field Models
// Centered dipole model for geomagnetic field
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::environment::magnetic {

namespace constants {
/// Reference magnetic field at equator surface [T] (~31.2 μT)
inline constexpr double B0 = 3.12e-5;

/// Magnetic dipole tilt angle from rotation axis [rad] (~11.5°)
inline constexpr double dipole_tilt = 11.5 * vulcan::constants::angle::deg2rad;

/// Magnetic pole longitude (West is negative) [rad]
inline constexpr double pole_longitude =
    -72.6 * vulcan::constants::angle::deg2rad;
} // namespace constants

/**
 * @brief Centered dipole magnetic field in ECEF
 *
 * Simplified model with dipole aligned with Earth's rotation axis.
 * For geomagnetic navigation, use the tilted dipole or IGRF models.
 *
 * Field follows: B = (B0 * R³/r³) * [3(m̂·r̂)r̂ - m̂]
 * where m̂ is the dipole direction (Z-axis for centered dipole)
 *
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF [m]
 * @param B0 Reference field at equator surface [T]
 * @param R Reference radius [m]
 * @return Magnetic field vector in ECEF [T]
 */
template <typename Scalar>
Vec3<Scalar> dipole_field_ecef(const Vec3<Scalar> &r_ecef,
                               double B0 = constants::B0,
                               double R = vulcan::constants::earth::R_eq) {
    const Scalar x = r_ecef(0);
    const Scalar y = r_ecef(1);
    const Scalar z = r_ecef(2);

    const Scalar r2 = x * x + y * y + z * z;
    const Scalar r = janus::sqrt(r2);
    const Scalar r5 = r2 * r2 * r;

    // Dipole field coefficient: B0 * R³ / r⁵
    const Scalar coeff = B0 * (R * R * R) / r5;

    // For centered dipole aligned with Z-axis:
    // B_x = 3 * coeff * x * z
    // B_y = 3 * coeff * y * z
    // B_z = coeff * (2z² - x² - y²)
    Vec3<Scalar> B;
    B(0) = 3.0 * coeff * x * z;
    B(1) = 3.0 * coeff * y * z;
    B(2) = coeff * (2.0 * z * z - x * x - y * y);

    return B;
}

/**
 * @brief Magnetic field magnitude at given position
 * @param r_ecef Position in ECEF [m]
 * @return |B| in Tesla
 */
template <typename Scalar>
Scalar field_magnitude(const Vec3<Scalar> &r_ecef, double B0 = constants::B0,
                       double R = vulcan::constants::earth::R_eq) {
    const Vec3<Scalar> B = dipole_field_ecef(r_ecef, B0, R);
    return janus::norm(B);
}

/**
 * @brief Magnetic field in local NED frame at geodetic coordinates
 *
 * Uses spherical approximation for simplicity.
 *
 * @param lat Geodetic latitude [rad]
 * @param lon Geodetic longitude [rad] (unused for centered dipole)
 * @param alt Altitude above ellipsoid [m]
 * @return {B_north, B_east, B_down} in Tesla
 */
template <typename Scalar>
Vec3<Scalar> field_ned(const Scalar &lat, [[maybe_unused]] const Scalar &lon,
                       const Scalar &alt, double B0 = constants::B0,
                       double R = vulcan::constants::earth::R_eq) {
    // Spherical approximation: r = R + alt
    const Scalar r = R + alt;

    // Use geocentric latitude (spherical approx)
    const Scalar phi = lat;

    // Radial component: B_r = -2 * B0 * (R/r)³ * sin(φ)
    // Theta component:  B_θ =  B0 * (R/r)³ * cos(φ)
    const Scalar R_over_r_cubed = (R * R * R) / (r * r * r);

    const Scalar sin_phi = janus::sin(phi);
    const Scalar cos_phi = janus::cos(phi);

    const Scalar B_r = -2.0 * B0 * R_over_r_cubed * sin_phi;
    const Scalar B_theta = B0 * R_over_r_cubed * cos_phi;

    // Convert to NED:
    // North = -B_θ (θ increases southward)
    // East  = 0 (axial symmetry for centered dipole)
    // Down  = -B_r (down is negative radial)
    Vec3<Scalar> B_ned;
    B_ned(0) = -B_theta;    // North
    B_ned(1) = Scalar(0.0); // East
    B_ned(2) = -B_r;        // Down

    return B_ned;
}

/**
 * @brief Total field intensity at surface for given latitude
 *
 * |B| = B0 * sqrt(1 + 3sin²(lat))
 *
 * @param lat Geodetic/geocentric latitude [rad]
 * @return Surface field intensity [T]
 */
template <typename Scalar>
Scalar surface_intensity(const Scalar &lat, double B0 = constants::B0) {
    const Scalar sin_lat = janus::sin(lat);
    return B0 * janus::sqrt(1.0 + 3.0 * sin_lat * sin_lat);
}

/**
 * @brief Magnetic inclination (dip angle) at given latitude
 *
 * For a centered dipole: tan(I) = 2 * tan(lat)
 *
 * @param lat Latitude [rad]
 * @return Inclination angle [rad] (positive downward in north)
 */
template <typename Scalar> Scalar inclination(const Scalar &lat) {
    const Scalar tan_lat = janus::tan(lat);
    return janus::atan(2.0 * tan_lat);
}

} // namespace vulcan::environment::magnetic
