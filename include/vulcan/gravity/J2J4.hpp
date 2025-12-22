// Vulcan J2-J4 Gravity Model
// Includes J2, J3, and J4 zonal harmonic perturbations
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::gravity::j2j4 {

/**
 * @brief J2/J3/J4 gravitational acceleration
 *
 * Includes first three zonal harmonics for high-fidelity gravity modeling.
 * - J2 captures Earth's equatorial bulge (oblateness)
 * - J3 captures the slight pear shape of Earth (north/south asymmetry)
 * - J4 captures additional higher-order oblateness
 *
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF frame [m]
 * @param mu Gravitational parameter [m³/s²]
 * @param J2_coeff J2 zonal harmonic coefficient
 * @param J3_coeff J3 zonal harmonic coefficient
 * @param J4_coeff J4 zonal harmonic coefficient
 * @param R_eq Equatorial radius [m]
 * @return Gravitational acceleration in ECEF [m/s²]
 */
template <typename Scalar>
Vec3<Scalar> acceleration(const Vec3<Scalar> &r_ecef,
                          double mu = constants::earth::mu,
                          double J2_coeff = constants::earth::J2,
                          double J3_coeff = constants::earth::J3,
                          double J4_coeff = constants::earth::J4,
                          double R_eq = constants::earth::R_eq) {
    const Scalar x = r_ecef(0);
    const Scalar y = r_ecef(1);
    const Scalar z = r_ecef(2);

    const Scalar r2 = x * x + y * y + z * z;
    const Scalar r = janus::sqrt(r2);
    const Scalar r3 = r2 * r;

    // Normalized z coordinate
    const Scalar z_r = z / r;
    const Scalar z_r2 = z_r * z_r;
    const Scalar z_r3 = z_r2 * z_r;
    const Scalar z_r4 = z_r2 * z_r2;

    // R_eq / r ratios
    const Scalar Re_r = R_eq / r;
    const Scalar Re_r2 = Re_r * Re_r;
    const Scalar Re_r3 = Re_r2 * Re_r;
    const Scalar Re_r4 = Re_r2 * Re_r2;

    // Base acceleration magnitude
    const Scalar base = -mu / r3;

    // ============================================
    // J2 Terms
    // ============================================
    const Scalar J2_xy = -1.5 * J2_coeff * Re_r2 * (5.0 * z_r2 - 1.0);
    const Scalar J2_z = -1.5 * J2_coeff * Re_r2 * (5.0 * z_r2 - 3.0);

    // ============================================
    // J3 Terms (odd harmonic - asymmetric about equator)
    // Simplified form valid everywhere
    // ============================================
    const Scalar J3_xy = 2.5 * J3_coeff * Re_r3 * z_r * (7.0 * z_r2 - 3.0);
    const Scalar J3_z = 2.5 * J3_coeff * Re_r3 * (7.0 * z_r3 - 3.0 * z_r);

    // ============================================
    // J4 Terms
    // ============================================
    const Scalar J4_xy =
        (15.0 / 8.0) * J4_coeff * Re_r4 * (1.0 - 14.0 * z_r2 + 21.0 * z_r4);
    const Scalar J4_z = (15.0 / 8.0) * J4_coeff * Re_r4 *
                        (5.0 - 70.0 / 3.0 * z_r2 + 21.0 * z_r4);

    // ============================================
    // Combined acceleration
    // ============================================
    const Scalar factor_xy = 1.0 + J2_xy + J3_xy + J4_xy;
    const Scalar factor_z = 1.0 + J2_z + J3_z + J4_z;

    Vec3<Scalar> accel;
    accel(0) = base * x * factor_xy;
    accel(1) = base * y * factor_xy;
    accel(2) = base * z * factor_z;

    return accel;
}

/**
 * @brief J2/J3/J4 gravitational potential
 *
 * U = -μ/r · [1 - Σ Jn·(R_eq/r)^n·Pn(sin φ)]
 *
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF frame [m]
 * @param mu Gravitational parameter [m³/s²]
 * @param J2_coeff J2 zonal harmonic coefficient
 * @param J3_coeff J3 zonal harmonic coefficient
 * @param J4_coeff J4 zonal harmonic coefficient
 * @param R_eq Equatorial radius [m]
 * @return Gravitational potential [m²/s²]
 */
template <typename Scalar>
Scalar potential(const Vec3<Scalar> &r_ecef, double mu = constants::earth::mu,
                 double J2_coeff = constants::earth::J2,
                 double J3_coeff = constants::earth::J3,
                 double J4_coeff = constants::earth::J4,
                 double R_eq = constants::earth::R_eq) {
    const Scalar x = r_ecef(0);
    const Scalar y = r_ecef(1);
    const Scalar z = r_ecef(2);

    const Scalar r2 = x * x + y * y + z * z;
    const Scalar r = janus::sqrt(r2);

    // sin(φ) = z/r
    const Scalar sin_phi = z / r;
    const Scalar sin_phi2 = sin_phi * sin_phi;
    const Scalar sin_phi3 = sin_phi2 * sin_phi;
    const Scalar sin_phi4 = sin_phi2 * sin_phi2;

    // R_eq / r ratios
    const Scalar Re_r = R_eq / r;
    const Scalar Re_r2 = Re_r * Re_r;
    const Scalar Re_r3 = Re_r2 * Re_r;
    const Scalar Re_r4 = Re_r2 * Re_r2;

    // Legendre polynomials P_n(sin φ)
    const Scalar P2 = (3.0 * sin_phi2 - 1.0) / 2.0;
    const Scalar P3 = (5.0 * sin_phi3 - 3.0 * sin_phi) / 2.0;
    const Scalar P4 = (35.0 * sin_phi4 - 30.0 * sin_phi2 + 3.0) / 8.0;

    // U = -μ/r · [1 - Σ Jn·(R_eq/r)^n·Pn(sin φ)]
    const Scalar correction =
        J2_coeff * Re_r2 * P2 + J3_coeff * Re_r3 * P3 + J4_coeff * Re_r4 * P4;

    return -mu / r * (1.0 - correction);
}

} // namespace vulcan::gravity::j2j4
