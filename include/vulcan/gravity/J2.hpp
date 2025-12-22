// Vulcan J2 Gravity Model
// Accounts for Earth's oblateness (equatorial bulge)
#pragma once

#include <type_traits>
#include <vulcan/core/VulcanError.hpp>

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::gravity::j2 {

/**
 * @brief J2 gravitational acceleration (oblate Earth)
 *
 * Accounts for Earth's equatorial bulge. The J2 term is the dominant
 * perturbation for low and medium Earth orbits.
 *
 * Mathematical form:
 *   a_x = -μx/r³ · [1 - 1.5·J2·(R_eq/r)²·(5(z/r)² - 1)]
 *   a_y = -μy/r³ · [1 - 1.5·J2·(R_eq/r)²·(5(z/r)² - 1)]
 *   a_z = -μz/r³ · [1 - 1.5·J2·(R_eq/r)²·(5(z/r)² - 3)]
 *
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF frame [m]
 * @param mu Gravitational parameter [m³/s²]
 * @param J2_coeff J2 zonal harmonic coefficient
 * @param R_eq Equatorial radius [m]
 * @return Gravitational acceleration in ECEF [m/s²]
 */
template <typename Scalar>
Vec3<Scalar> acceleration(const Vec3<Scalar> &r_ecef,
                          double mu = constants::earth::mu,
                          double J2_coeff = constants::earth::J2,
                          double R_eq = constants::earth::R_eq) {
    const Scalar x = r_ecef(0);
    const Scalar y = r_ecef(1);
    const Scalar z = r_ecef(2);

    const Scalar r2 = x * x + y * y + z * z;
    const Scalar r = janus::sqrt(r2);

    // Singularity check: position inside Earth's center
    if constexpr (std::is_same_v<Scalar, double>) {
        if (r < 1.0) {
            throw GravityError(
                "j2::acceleration: position magnitude < 1m (singularity)");
        }
    }

    const Scalar r3 = r2 * r;

    // Precompute common terms
    const Scalar z2_over_r2 = z * z / r2;
    const Scalar R_eq_over_r_sq = (R_eq * R_eq) / r2;
    const Scalar J2_factor = 1.5 * J2_coeff * R_eq_over_r_sq;

    // Point mass term coefficient: -μ/r³
    const Scalar pm_coeff = -mu / r3;

    // J2 perturbation factors
    const Scalar xy_factor = 1.0 - J2_factor * (5.0 * z2_over_r2 - 1.0);
    const Scalar z_factor = 1.0 - J2_factor * (5.0 * z2_over_r2 - 3.0);

    Vec3<Scalar> accel;
    accel(0) = pm_coeff * x * xy_factor;
    accel(1) = pm_coeff * y * xy_factor;
    accel(2) = pm_coeff * z * z_factor;

    return accel;
}

/**
 * @brief J2 gravitational potential
 *
 * U = -μ/r · [1 - J2·(R_eq/r)²·P₂(sin φ)]
 *
 * where P₂(x) = (3x² - 1)/2 is the Legendre polynomial of degree 2.
 *
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF frame [m]
 * @param mu Gravitational parameter [m³/s²]
 * @param J2_coeff J2 zonal harmonic coefficient
 * @param R_eq Equatorial radius [m]
 * @return Gravitational potential [m²/s²]
 */
template <typename Scalar>
Scalar potential(const Vec3<Scalar> &r_ecef, double mu = constants::earth::mu,
                 double J2_coeff = constants::earth::J2,
                 double R_eq = constants::earth::R_eq) {
    const Scalar x = r_ecef(0);
    const Scalar y = r_ecef(1);
    const Scalar z = r_ecef(2);

    const Scalar r2 = x * x + y * y + z * z;
    const Scalar r = janus::sqrt(r2);

    // Singularity check
    if constexpr (std::is_same_v<Scalar, double>) {
        if (r < 1.0) {
            throw GravityError(
                "j2::potential: position magnitude < 1m (singularity)");
        }
    }

    // sin(φ) = z/r (geocentric latitude)
    const Scalar sin_phi = z / r;
    const Scalar sin_phi_sq = sin_phi * sin_phi;

    // P₂(sin φ) = (3 sin²φ - 1) / 2
    const Scalar P2 = (3.0 * sin_phi_sq - 1.0) / 2.0;

    // U = -μ/r · [1 - J2·(R_eq/r)²·P₂]
    const Scalar R_eq_over_r_sq = (R_eq * R_eq) / r2;

    return -mu / r * (1.0 - J2_coeff * R_eq_over_r_sq * P2);
}

} // namespace vulcan::gravity::j2
