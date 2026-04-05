// Vulcan Point Mass Gravity Model
// Simple inverse-square gravitational acceleration
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/QuantityEigen.hpp>
#include <vulcan/quantity/Units.hpp>

namespace vulcan::gravity::point_mass {

/**
 * @brief Point mass gravitational acceleration
 *
 * The simplest gravity model treating Earth as a uniform sphere.
 *
 * g = -mu/r^3 * r_vec
 *
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF frame [m]
 * @param mu Gravitational parameter [m^3/s^2] (default: Earth)
 * @return Gravitational acceleration in ECEF [m/s^2]
 */
template <typename Scalar>
Vec3<Quantity<units::m / (units::s * units::s), Scalar>>
acceleration(const Vec3<Quantity<units::m, Scalar>> &r_ecef,
             double mu = constants::earth::mu.value()) {
    // Unwrap Quantity inputs to raw Scalar
    const Scalar rx = r_ecef(0).value();
    const Scalar ry = r_ecef(1).value();
    const Scalar rz = r_ecef(2).value();

    Vec3<Scalar> r_raw;
    r_raw(0) = rx;
    r_raw(1) = ry;
    r_raw(2) = rz;

    const Scalar r_mag = janus::norm(r_raw);
    const Scalar r_cubed = r_mag * r_mag * r_mag;

    // g = -mu/r^3 * r
    Vec3<Scalar> g_raw = -mu / r_cubed * r_raw;

    // Wrap output in Quantity
    using AccelQ = Quantity<units::m / (units::s * units::s), Scalar>;
    Vec3<AccelQ> result;
    result(0) = AccelQ{g_raw(0)};
    result(1) = AccelQ{g_raw(1)};
    result(2) = AccelQ{g_raw(2)};
    return result;
}

/**
 * @brief Point mass gravitational potential
 *
 * U = -mu/r
 *
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF frame [m]
 * @param mu Gravitational parameter [m^3/s^2]
 * @return Gravitational potential [m^2/s^2]
 */
template <typename Scalar>
Quantity<units::m * units::m / (units::s * units::s), Scalar>
potential(const Vec3<Quantity<units::m, Scalar>> &r_ecef,
          double mu = constants::earth::mu.value()) {
    const Scalar rx = r_ecef(0).value();
    const Scalar ry = r_ecef(1).value();
    const Scalar rz = r_ecef(2).value();

    Vec3<Scalar> r_raw;
    r_raw(0) = rx;
    r_raw(1) = ry;
    r_raw(2) = rz;

    const Scalar r_mag = janus::norm(r_raw);
    return Quantity<units::m * units::m / (units::s * units::s), Scalar>{-mu /
                                                                         r_mag};
}

/**
 * @brief Gravitational acceleration magnitude at distance r
 *
 * |g| = mu/r^2
 *
 * Convenience function when only magnitude is needed.
 *
 * @tparam Scalar double or casadi::MX
 * @param r_mag Radial distance from center [m]
 * @param mu Gravitational parameter [m^3/s^2]
 * @return Acceleration magnitude [m/s^2]
 */
template <typename Scalar>
Quantity<units::m / (units::s * units::s), Scalar>
acceleration_magnitude(const Quantity<units::m, Scalar> &r_mag,
                       double mu = constants::earth::mu.value()) {
    const Scalar r = r_mag.value();
    return Quantity<units::m / (units::s * units::s), Scalar>{mu / (r * r)};
}

} // namespace vulcan::gravity::point_mass
