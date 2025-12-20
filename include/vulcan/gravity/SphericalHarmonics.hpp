// Vulcan Spherical Harmonics Gravity Model
// General spherical harmonic expansion for high-fidelity applications
#pragma once

#include <janus/janus.hpp>
#include <vulcan/coordinates/Geodetic.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>

#include <vector>

namespace vulcan::gravity::spherical_harmonics {

/**
 * @brief Gravity model coefficients container
 *
 * Stores normalized C and S coefficients for spherical harmonic expansion.
 * Supports up to degree/order N_MAX (compile-time or runtime).
 */
struct GravityCoefficients {
    int n_max;                          ///< Maximum degree
    std::vector<std::vector<double>> C; ///< Cosine coefficients C[n][m]
    std::vector<std::vector<double>> S; ///< Sine coefficients S[n][m]
    double mu;                          ///< Gravitational parameter [m³/s²]
    double R_eq;                        ///< Reference radius [m]

    /// Initialize with given maximum degree
    explicit GravityCoefficients(int n_max_val = 20,
                                 double mu_val = constants::earth::mu,
                                 double R_eq_val = constants::earth::R_eq)
        : n_max(n_max_val), mu(mu_val), R_eq(R_eq_val) {
        C.resize(static_cast<size_t>(n_max + 1));
        S.resize(static_cast<size_t>(n_max + 1));
        for (int n = 0; n <= n_max; ++n) {
            C[static_cast<size_t>(n)].resize(static_cast<size_t>(n + 1), 0.0);
            S[static_cast<size_t>(n)].resize(static_cast<size_t>(n + 1), 0.0);
        }
        // Monopole (point mass) term
        C[0][0] = 1.0;

        // J2, J3, J4 as zonal harmonics (m=0)
        if (n_max >= 2)
            C[2][0] = -constants::earth::J2;
        if (n_max >= 3)
            C[3][0] = -constants::earth::J3;
        if (n_max >= 4)
            C[4][0] = -constants::earth::J4;
    }
};

/// @brief Default Earth model with zonal harmonics only
inline const GravityCoefficients &default_coefficients() {
    static GravityCoefficients coeffs(4);
    return coeffs;
}

/**
 * @brief Compute associated Legendre polynomial P_nm(x)
 *
 * Uses recurrence relations, fully symbolic-compatible.
 * Loop bounds are structural (integers), so this works in symbolic mode.
 *
 * @tparam Scalar double or casadi::MX
 * @param n Degree
 * @param m Order
 * @param x Argument (typically sin(latitude))
 * @return P_nm(x)
 */
template <typename Scalar> Scalar legendre_Pnm(int n, int m, const Scalar &x) {
    // Handle m > n case
    if (m > n)
        return Scalar(0.0);

    // P_mm: diagonal recurrence
    // P_mm = (-1)^m (2m-1)!! (1-x²)^(m/2)
    Scalar pmm = Scalar(1.0);
    if (m > 0) {
        Scalar somx2 = janus::sqrt((1.0 - x) * (1.0 + x));
        double fact = 1.0;
        for (int i = 1; i <= m; ++i) {
            pmm = -pmm * fact * somx2;
            fact += 2.0;
        }
    }

    if (n == m)
        return pmm;

    // P_{m+1,m} = x(2m+1)P_mm
    Scalar pmm1 = x * (2.0 * m + 1.0) * pmm;
    if (n == m + 1)
        return pmm1;

    // General recurrence for n > m+1
    // P_nm = [(2n-1)x P_{n-1,m} - (n+m-1)P_{n-2,m}] / (n-m)
    Scalar pnm = Scalar(0.0);
    for (int nn = m + 2; nn <= n; ++nn) {
        pnm = ((2.0 * nn - 1.0) * x * pmm1 - (nn + m - 1.0) * pmm) / (nn - m);
        pmm = pmm1;
        pmm1 = pnm;
    }

    return pnm;
}

/**
 * @brief Spherical harmonic gravitational acceleration
 *
 * General expansion using partial derivatives in spherical coordinates,
 * then transformed to ECEF.
 *
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF [m]
 * @param coeffs Gravity coefficients (C, S, n_max)
 * @return Acceleration in ECEF [m/s²]
 *
 * @note For symbolic mode, n_max must be fixed at trace time (structural loop).
 */
template <typename Scalar>
Vec3<Scalar>
acceleration(const Vec3<Scalar> &r_ecef,
             const GravityCoefficients &coeffs = default_coefficients()) {
    // Convert to spherical coordinates
    const Spherical<Scalar> sph = ecef_to_spherical(r_ecef);
    const Scalar r = sph.radius;
    const Scalar lon = sph.lon;
    const Scalar lat_gc = sph.lat_gc;

    const Scalar sin_lat = janus::sin(lat_gc);
    const Scalar cos_lat = janus::cos(lat_gc);

    const double mu = coeffs.mu;
    const double R_eq = coeffs.R_eq;
    const int n_max = coeffs.n_max;

    // Initialize partial derivatives of potential
    Scalar dU_dr = Scalar(0.0);   // ∂U/∂r
    Scalar dU_dlat = Scalar(0.0); // ∂U/∂φ
    Scalar dU_dlon = Scalar(0.0); // ∂U/∂λ

    // Summation over degrees and orders
    // Loop bounds are structural (n_max is int, not Scalar)
    for (int n = 0; n <= n_max; ++n) {
        const Scalar Re_r_n = janus::pow(R_eq / r, static_cast<double>(n));

        for (int m = 0; m <= n; ++m) {
            const double C_nm =
                coeffs.C[static_cast<size_t>(n)][static_cast<size_t>(m)];
            const double S_nm =
                coeffs.S[static_cast<size_t>(n)][static_cast<size_t>(m)];

            // Skip zero coefficients for efficiency
            if (C_nm == 0.0 && S_nm == 0.0)
                continue;

            const Scalar cos_m_lon = janus::cos(static_cast<double>(m) * lon);
            const Scalar sin_m_lon = janus::sin(static_cast<double>(m) * lon);

            const Scalar P_nm = legendre_Pnm(n, m, sin_lat);

            // Derivative of Legendre polynomial using recurrence
            const Scalar P_nm1 =
                (n > 0) ? legendre_Pnm(n - 1, m, sin_lat) : Scalar(0.0);
            const Scalar tan_lat = janus::tan(lat_gc);
            const Scalar dP_dlat = static_cast<double>(n) * tan_lat * P_nm -
                                   static_cast<double>(n + m) / cos_lat * P_nm1;

            const Scalar trig_term = C_nm * cos_m_lon + S_nm * sin_m_lon;
            const Scalar dtrig_dlon =
                static_cast<double>(m) * (-C_nm * sin_m_lon + S_nm * cos_m_lon);

            // Accumulate partial derivatives
            dU_dr += static_cast<double>(n + 1) * Re_r_n / r * trig_term * P_nm;
            dU_dlat += Re_r_n * trig_term * dP_dlat;
            dU_dlon += Re_r_n * dtrig_dlon * P_nm;
        }
    }

    // Scale by μ/r
    const Scalar scale = mu / r;
    dU_dr = dU_dr * scale;
    dU_dlat = dU_dlat * scale;
    dU_dlon = dU_dlon * scale;

    // Convert spherical gradient to ECEF acceleration
    // g = -∇U
    const Scalar sin_lon = janus::sin(lon);
    const Scalar cos_lon = janus::cos(lon);

    // Spherical to Cartesian transformation
    const Scalar g_r = -dU_dr;
    const Scalar g_lat = -dU_dlat / r;
    const Scalar g_lon = -dU_dlon / (r * cos_lat);

    // Transform to ECEF
    Vec3<Scalar> g_ecef;
    g_ecef(0) =
        cos_lat * cos_lon * g_r - sin_lat * cos_lon * g_lat - sin_lon * g_lon;
    g_ecef(1) =
        cos_lat * sin_lon * g_r - sin_lat * sin_lon * g_lat + cos_lon * g_lon;
    g_ecef(2) = sin_lat * g_r + cos_lat * g_lat;

    return g_ecef;
}

/**
 * @brief Spherical harmonic gravitational potential
 *
 * U = μ/r Σ Σ (R_eq/r)^n [C_nm cos(mλ) + S_nm sin(mλ)] P_nm(sin φ)
 *
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF [m]
 * @param coeffs Gravity coefficients (C, S, n_max)
 * @return Gravitational potential [m²/s²]
 */
template <typename Scalar>
Scalar potential(const Vec3<Scalar> &r_ecef,
                 const GravityCoefficients &coeffs = default_coefficients()) {
    const Spherical<Scalar> sph = ecef_to_spherical(r_ecef);
    const Scalar r = sph.radius;
    const Scalar lon = sph.lon;
    const Scalar lat_gc = sph.lat_gc;

    const Scalar sin_lat = janus::sin(lat_gc);

    const double mu = coeffs.mu;
    const double R_eq = coeffs.R_eq;
    const int n_max = coeffs.n_max;

    Scalar U = -mu / r; // Point mass term

    for (int n = 2; n <= n_max; ++n) {
        const Scalar Re_r_n = janus::pow(R_eq / r, static_cast<double>(n));

        for (int m = 0; m <= n; ++m) {
            const double C_nm =
                coeffs.C[static_cast<size_t>(n)][static_cast<size_t>(m)];
            const double S_nm =
                coeffs.S[static_cast<size_t>(n)][static_cast<size_t>(m)];

            if (C_nm == 0.0 && S_nm == 0.0)
                continue;

            const Scalar cos_m_lon = janus::cos(static_cast<double>(m) * lon);
            const Scalar sin_m_lon = janus::sin(static_cast<double>(m) * lon);
            const Scalar P_nm = legendre_Pnm(n, m, sin_lat);

            U = U -
                mu / r * Re_r_n * (C_nm * cos_m_lon + S_nm * sin_m_lon) * P_nm;
        }
    }

    return U;
}

} // namespace vulcan::gravity::spherical_harmonics
