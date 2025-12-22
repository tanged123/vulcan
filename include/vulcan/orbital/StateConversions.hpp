// Vulcan State Conversions
// Cartesian <-> Keplerian orbital element transformations
#pragma once

#include <cmath>
#include <janus/janus.hpp>
#include <utility>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/orbital/OrbitalTypes.hpp>

namespace vulcan::orbital::elements {

/**
 * @brief Convert Cartesian state to Keplerian elements
 *
 * Transforms position and velocity vectors to classical orbital elements.
 * Handles singular cases (circular, equatorial orbits) with numerical guards.
 *
 * @tparam Scalar double or casadi::MX
 * @param r Position vector in inertial frame [m]
 * @param v Velocity vector in inertial frame [m/s]
 * @param mu Gravitational parameter [m³/s²] (default: Earth)
 * @return OrbitalElements<Scalar>
 */
template <typename Scalar>
OrbitalElements<Scalar>
cartesian_to_keplerian(const Vec3<Scalar> &r, const Vec3<Scalar> &v,
                       double mu = constants::earth::mu) {
    OrbitalElements<Scalar> oe;

    const Scalar r_mag = janus::norm(r);
    const Scalar v_mag = janus::norm(v);

    // Specific angular momentum vector
    const Vec3<Scalar> h = janus::cross(r, v);
    const Scalar h_mag = janus::norm(h);

    // Node vector (K x h) - points to ascending node
    Vec3<Scalar> n;
    n << -h(1), h(0), Scalar(0.0);
    const Scalar n_mag = janus::norm(n);

    // Eccentricity vector
    const Scalar r_dot_v = janus::dot(r, v);
    const Vec3<Scalar> e_vec =
        ((v_mag * v_mag - mu / r_mag) * r - r_dot_v * v) / mu;
    oe.e = janus::norm(e_vec);

    // Specific mechanical energy
    const Scalar energy = v_mag * v_mag / 2.0 - mu / r_mag;

    // Semi-major axis (negative for hyperbolic)
    oe.a = -mu / (2.0 * energy);

    // Inclination [0, pi]
    oe.i = janus::acos(janus::clamp(h(2) / h_mag, Scalar(-1.0), Scalar(1.0)));

    // RAAN [0, 2pi] - handle equatorial case
    const Scalar n_mag_safe = janus::where(n_mag < 1e-10, Scalar(1.0), n_mag);
    Scalar Omega_temp =
        janus::acos(janus::clamp(n(0) / n_mag_safe, Scalar(-1.0), Scalar(1.0)));
    oe.Omega = janus::where(n(1) < 0.0, 2.0 * M_PI - Omega_temp, Omega_temp);
    oe.Omega = janus::where(n_mag < 1e-10, Scalar(0.0), oe.Omega);

    // Argument of periapsis [0, 2pi] - handle circular/equatorial case
    const Scalar e_safe = janus::where(oe.e < 1e-10, Scalar(1.0), oe.e);
    const Scalar n_dot_e = janus::dot(n, e_vec);
    Scalar omega_temp = janus::acos(janus::clamp(
        n_dot_e / (n_mag_safe * e_safe), Scalar(-1.0), Scalar(1.0)));
    oe.omega =
        janus::where(e_vec(2) < 0.0, 2.0 * M_PI - omega_temp, omega_temp);
    oe.omega = janus::where(oe.e < 1e-10, Scalar(0.0), oe.omega);
    oe.omega = janus::where(n_mag < 1e-10, Scalar(0.0), oe.omega);

    // True anomaly [0, 2pi]
    const Scalar e_dot_r = janus::dot(e_vec, r);
    Scalar nu_temp = janus::acos(
        janus::clamp(e_dot_r / (e_safe * r_mag), Scalar(-1.0), Scalar(1.0)));
    oe.nu = janus::where(r_dot_v < 0.0, 2.0 * M_PI - nu_temp, nu_temp);
    oe.nu = janus::where(oe.e < 1e-10, Scalar(0.0), oe.nu);

    return oe;
}

/**
 * @brief Convert Keplerian elements to Cartesian state
 *
 * Transforms classical orbital elements to position and velocity vectors.
 *
 * @tparam Scalar double or casadi::MX
 * @param oe Orbital elements
 * @param mu Gravitational parameter [m³/s²] (default: Earth)
 * @return Pair of (position, velocity) vectors in inertial frame
 */
template <typename Scalar>
std::pair<Vec3<Scalar>, Vec3<Scalar>>
keplerian_to_cartesian(const OrbitalElements<Scalar> &oe,
                       double mu = constants::earth::mu) {

    // Semi-latus rectum
    const Scalar p = oe.a * (1.0 - oe.e * oe.e);

    // Position magnitude at true anomaly
    const Scalar r_mag = p / (1.0 + oe.e * janus::cos(oe.nu));

    const Scalar cos_nu = janus::cos(oe.nu);
    const Scalar sin_nu = janus::sin(oe.nu);

    // Position in perifocal (PQW) frame
    Vec3<Scalar> r_pqw;
    r_pqw << r_mag * cos_nu, r_mag * sin_nu, Scalar(0.0);

    // Velocity in perifocal frame
    const Scalar sqrt_mu_p = janus::sqrt(mu / p);
    Vec3<Scalar> v_pqw;
    v_pqw << -sqrt_mu_p * sin_nu, sqrt_mu_p * (oe.e + cos_nu), Scalar(0.0);

    // Rotation matrix from perifocal to inertial (ECI)
    const Scalar cos_O = janus::cos(oe.Omega);
    const Scalar sin_O = janus::sin(oe.Omega);
    const Scalar cos_i = janus::cos(oe.i);
    const Scalar sin_i = janus::sin(oe.i);
    const Scalar cos_w = janus::cos(oe.omega);
    const Scalar sin_w = janus::sin(oe.omega);

    Mat3<Scalar> R;
    R(0, 0) = cos_O * cos_w - sin_O * sin_w * cos_i;
    R(0, 1) = -cos_O * sin_w - sin_O * cos_w * cos_i;
    R(0, 2) = sin_O * sin_i;
    R(1, 0) = sin_O * cos_w + cos_O * sin_w * cos_i;
    R(1, 1) = -sin_O * sin_w + cos_O * cos_w * cos_i;
    R(1, 2) = -cos_O * sin_i;
    R(2, 0) = sin_w * sin_i;
    R(2, 1) = cos_w * sin_i;
    R(2, 2) = cos_i;

    Vec3<Scalar> r = R * r_pqw;
    Vec3<Scalar> v = R * v_pqw;

    return {r, v};
}

} // namespace vulcan::orbital::elements
