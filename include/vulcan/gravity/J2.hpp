// Vulcan J2 Gravity Model
// Accounts for Earth's oblateness (equatorial bulge)
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/QuantityEigen.hpp>
#include <vulcan/quantity/Units.hpp>

namespace vulcan::gravity::j2 {

/**
 * @brief J2 gravitational acceleration (oblate Earth)
 *
 * Accounts for Earth's equatorial bulge. The J2 term is the dominant
 * perturbation for low and medium Earth orbits.
 *
 * Mathematical form:
 *   a_x = -mu*x/r^3 * [1 - 1.5*J2*(R_eq/r)^2*(5(z/r)^2 - 1)]
 *   a_y = -mu*y/r^3 * [1 - 1.5*J2*(R_eq/r)^2*(5(z/r)^2 - 1)]
 *   a_z = -mu*z/r^3 * [1 - 1.5*J2*(R_eq/r)^2*(5(z/r)^2 - 3)]
 *
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF frame [m]
 * @param mu Gravitational parameter [m^3/s^2]
 * @param J2_coeff J2 zonal harmonic coefficient
 * @param R_eq Equatorial radius [m]
 * @return Gravitational acceleration in ECEF [m/s^2]
 */
template <typename Scalar>
Vec3<Quantity<units::m / (units::s * units::s), Scalar>>
acceleration(const Vec3<Quantity<units::m, Scalar>> &r_ecef,
             double mu = constants::earth::mu.value(),
             double J2_coeff = constants::earth::J2.value(),
             double R_eq = constants::earth::R_eq.value()) {
    const Scalar x = r_ecef(0).value();
    const Scalar y = r_ecef(1).value();
    const Scalar z = r_ecef(2).value();

    const Scalar r2 = x * x + y * y + z * z;
    const Scalar r = janus::sqrt(r2);
    const Scalar r3 = r2 * r;

    // Precompute common terms
    const Scalar z2_over_r2 = z * z / r2;
    const Scalar R_eq_over_r_sq = (R_eq * R_eq) / r2;
    const Scalar J2_factor = 1.5 * J2_coeff * R_eq_over_r_sq;

    // Point mass term coefficient: -mu/r^3
    const Scalar pm_coeff = -mu / r3;

    // J2 perturbation factors
    const Scalar xy_factor = 1.0 - J2_factor * (5.0 * z2_over_r2 - 1.0);
    const Scalar z_factor = 1.0 - J2_factor * (5.0 * z2_over_r2 - 3.0);

    using AccelQ = Quantity<units::m / (units::s * units::s), Scalar>;
    Vec3<AccelQ> accel;
    accel(0) = AccelQ{pm_coeff * x * xy_factor};
    accel(1) = AccelQ{pm_coeff * y * xy_factor};
    accel(2) = AccelQ{pm_coeff * z * z_factor};

    return accel;
}

/**
 * @brief J2 gravitational potential
 *
 * U = -mu/r * [1 - J2*(R_eq/r)^2*P2(sin phi)]
 *
 * where P2(x) = (3x^2 - 1)/2 is the Legendre polynomial of degree 2.
 *
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF frame [m]
 * @param mu Gravitational parameter [m^3/s^2]
 * @param J2_coeff J2 zonal harmonic coefficient
 * @param R_eq Equatorial radius [m]
 * @return Gravitational potential [m^2/s^2]
 */
template <typename Scalar>
Quantity<units::m * units::m / (units::s * units::s), Scalar>
potential(const Vec3<Quantity<units::m, Scalar>> &r_ecef,
          double mu = constants::earth::mu.value(),
          double J2_coeff = constants::earth::J2.value(),
          double R_eq = constants::earth::R_eq.value()) {
    const Scalar x = r_ecef(0).value();
    const Scalar y = r_ecef(1).value();
    const Scalar z = r_ecef(2).value();

    const Scalar r2 = x * x + y * y + z * z;
    const Scalar r = janus::sqrt(r2);

    // sin(phi) = z/r (geocentric latitude)
    const Scalar sin_phi = z / r;
    const Scalar sin_phi_sq = sin_phi * sin_phi;

    // P2(sin phi) = (3 sin^2 phi - 1) / 2
    const Scalar P2 = (3.0 * sin_phi_sq - 1.0) / 2.0;

    // U = -mu/r * [1 - J2*(R_eq/r)^2*P2]
    const Scalar R_eq_over_r_sq = (R_eq * R_eq) / r2;

    return Quantity<units::m * units::m / (units::s * units::s), Scalar>{
        -mu / r * (1.0 - J2_coeff * R_eq_over_r_sq * P2)};
}

} // namespace vulcan::gravity::j2
