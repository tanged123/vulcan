// Vulcan Transfer Mechanics
// Orbital maneuver delta-v computations
#pragma once

#include <cmath>
#include <janus/janus.hpp>
#include <tuple>
#include <utility>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/orbital/OrbitalQuantities.hpp>

namespace vulcan::orbital::transfer {

/**
 * @brief Hohmann transfer delta-v
 *
 * Computes the delta-v required for a two-impulse Hohmann transfer
 * between coplanar circular orbits.
 *
 * @tparam Scalar double or casadi::MX
 * @param r1 Initial orbit radius [m]
 * @param r2 Final orbit radius [m]
 * @param mu Gravitational parameter [m³/s²] (default: Earth)
 * @return Pair of (dv1, dv2) at periapsis and apoapsis [m/s]
 */
template <typename Scalar>
std::pair<Scalar, Scalar> hohmann_delta_v(const Scalar &r1, const Scalar &r2,
                                          double mu = constants::earth::mu) {
    // Transfer orbit semi-major axis
    const Scalar a_t = (r1 + r2) / 2.0;

    // Velocities using vis-viva
    const Scalar v1 = quantities::circular_velocity(r1, mu);
    const Scalar v2 = quantities::circular_velocity(r2, mu);
    const Scalar v_t1 = quantities::velocity(r1, a_t, mu);
    const Scalar v_t2 = quantities::velocity(r2, a_t, mu);

    // Delta-v magnitudes (absolute value for outward transfer)
    const Scalar dv1 = janus::abs(v_t1 - v1);
    const Scalar dv2 = janus::abs(v2 - v_t2);

    return {dv1, dv2};
}

/**
 * @brief Total Hohmann transfer delta-v
 *
 * @tparam Scalar double or casadi::MX
 * @param r1 Initial orbit radius [m]
 * @param r2 Final orbit radius [m]
 * @param mu Gravitational parameter [m³/s²] (default: Earth)
 * @return Total delta-v [m/s]
 */
template <typename Scalar>
Scalar hohmann_total_delta_v(const Scalar &r1, const Scalar &r2,
                             double mu = constants::earth::mu) {
    auto [dv1, dv2] = hohmann_delta_v(r1, r2, mu);
    return dv1 + dv2;
}

/**
 * @brief Hohmann transfer time
 *
 * @tparam Scalar double or casadi::MX
 * @param r1 Initial orbit radius [m]
 * @param r2 Final orbit radius [m]
 * @param mu Gravitational parameter [m³/s²] (default: Earth)
 * @return Transfer time (half period) [s]
 */
template <typename Scalar>
Scalar hohmann_transfer_time(const Scalar &r1, const Scalar &r2,
                             double mu = constants::earth::mu) {
    const Scalar a_t = (r1 + r2) / 2.0;
    return M_PI * janus::sqrt(a_t * a_t * a_t / mu);
}

/**
 * @brief Bielliptic transfer delta-v
 *
 * Three-impulse transfer that can be more efficient than Hohmann
 * for large radius ratios (r2/r1 > 11.94).
 *
 * @tparam Scalar double or casadi::MX
 * @param r1 Initial orbit radius [m]
 * @param r2 Final orbit radius [m]
 * @param r_b Intermediate apoapsis radius [m] (should be > max(r1, r2))
 * @param mu Gravitational parameter [m³/s²] (default: Earth)
 * @return Tuple of (dv1, dv2, dv3) [m/s]
 */
template <typename Scalar>
std::tuple<Scalar, Scalar, Scalar>
bielliptic_delta_v(const Scalar &r1, const Scalar &r2, const Scalar &r_b,
                   double mu = constants::earth::mu) {
    // First transfer orbit: r1 to r_b
    const Scalar a1 = (r1 + r_b) / 2.0;
    // Second transfer orbit: r_b to r2
    const Scalar a2 = (r_b + r2) / 2.0;

    const Scalar v1 = quantities::circular_velocity(r1, mu);
    const Scalar v2 = quantities::circular_velocity(r2, mu);

    const Scalar v_t1_peri = quantities::velocity(r1, a1, mu);
    const Scalar v_t1_apo = quantities::velocity(r_b, a1, mu);
    const Scalar v_t2_apo = quantities::velocity(r_b, a2, mu);
    const Scalar v_t2_peri = quantities::velocity(r2, a2, mu);

    const Scalar dv1 = janus::abs(v_t1_peri - v1);
    const Scalar dv2 = janus::abs(v_t2_apo - v_t1_apo);
    const Scalar dv3 = janus::abs(v2 - v_t2_peri);

    return {dv1, dv2, dv3};
}

/**
 * @brief Simple plane change delta-v
 *
 * Delta-v for a pure inclination change at constant altitude.
 * Most efficient at apoapsis where velocity is lowest.
 *
 * @tparam Scalar double or casadi::MX
 * @param v Orbital velocity magnitude [m/s]
 * @param delta_i Inclination change [rad]
 * @return Delta-v required [m/s]
 */
template <typename Scalar>
Scalar plane_change_delta_v(const Scalar &v, const Scalar &delta_i) {
    return 2.0 * v * janus::sin(janus::abs(delta_i) / 2.0);
}

/**
 * @brief Combined plane change and altitude change
 *
 * For a single-impulse maneuver that changes both plane and altitude.
 *
 * @tparam Scalar double or casadi::MX
 * @param v1 Initial velocity magnitude [m/s]
 * @param v2 Final velocity magnitude [m/s]
 * @param delta_i Inclination change [rad]
 * @return Delta-v required [m/s]
 */
template <typename Scalar>
Scalar combined_maneuver_delta_v(const Scalar &v1, const Scalar &v2,
                                 const Scalar &delta_i) {
    const Scalar cos_di = janus::cos(delta_i);
    return janus::sqrt(v1 * v1 + v2 * v2 - 2.0 * v1 * v2 * cos_di);
}

} // namespace vulcan::orbital::transfer
