# Phase 6: Gravity Models Implementation Plan

> **Purpose**: This document provides a comprehensive implementation plan for robust gravity models in Vulcan, following the established Janus-compatible patterns.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Architecture](#2-architecture)
3. [Implementation Details](#3-implementation-details)
4. [Testing Strategy](#4-testing-strategy)
5. [Verification & Validation](#5-verification--validation)
6. [Implementation Phases](#6-implementation-phases)

---

## 1. Overview

### Goals

Implement a suite of gravity models with increasing fidelity:

| Model | Fidelity | Use Case | Symbolic Support |
|-------|----------|----------|------------------|
| **Point Mass** | Low | Quick estimates, LEO | Full |
| **J2** | Medium | LEO/MEO, 6-DOF sim | Full |
| **J2-J4** | High | Precise orbit propagation | Full |
| **Spherical Harmonics** | Very High | High-precision applications | Full (fixed N) |

### Existing Infrastructure

Constants already defined in `vulcan/core/Constants.hpp`:
```cpp
vulcan::constants::earth::mu      // 3.986004418e14 m³/s²
vulcan::constants::earth::R_eq    // 6378137.0 m
vulcan::constants::earth::J2      // 1.08263e-3
vulcan::constants::earth::J3      // -2.54e-6
vulcan::constants::earth::J4      // -1.61e-6
```

---

## 2. Architecture

### Namespace Organization

Following the Vulcan pattern of namespace-organized free functions (no factories):

```
vulcan::gravity::
├── point_mass::
│   ├── acceleration(r_ecef)           // Returns Vec3<Scalar>
│   ├── potential(r_ecef)              // Returns Scalar
│   └── acceleration_magnitude(r)       // Returns Scalar (convenience)
├── j2::
│   ├── acceleration(r_ecef)           // With default Earth params
│   ├── acceleration(r_ecef, mu, J2, R_eq)  // Customizable
│   └── potential(r_ecef)
├── j2j4::
│   ├── acceleration(r_ecef)           // J2 + J3 + J4 terms
│   ├── acceleration(r_ecef, mu, J2, J3, J4, R_eq)
│   └── potential(r_ecef)
└── spherical_harmonics::
    ├── acceleration(r_ecef, n_max, m_max)  // Up to degree n_max
    ├── potential(r_ecef, n_max, m_max)
    └── load_coefficients(filename)    // Load EGM96/EGM2008 (numeric only)
```

### File Structure

```
include/vulcan/gravity/
├── PointMass.hpp           # Simple inverse-square gravity
├── J2.hpp                  # J2 oblateness perturbation
├── J2J4.hpp                # J2 + J3 + J4 zonal harmonics
├── SphericalHarmonics.hpp  # General spherical harmonic expansion
├── GravityTypes.hpp        # Shared types and constants
└── Gravity.hpp             # Umbrella header

tests/gravity/
├── test_point_mass.cpp     # (extend existing)
├── test_j2.cpp
├── test_j2j4.cpp
└── test_spherical_harmonics.cpp

examples/gravity/
├── gravity_comparison.cpp      # Compare all models
├── orbit_perturbation.cpp      # J2 effects on orbit
└── gravity_gradient.cpp        # Symbolic gradient demo
```

---

## 3. Implementation Details

### 3.1 Point Mass Gravity (`PointMass.hpp`)

The simplest model: spherical Earth with uniform density.

```cpp
// include/vulcan/gravity/PointMass.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::gravity::point_mass {

/**
 * @brief Point mass gravitational acceleration
 *
 * g = -μ/r³ · r_vec
 *
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF frame [m]
 * @param mu Gravitational parameter [m³/s²] (default: Earth)
 * @return Gravitational acceleration in ECEF [m/s²]
 */
template <typename Scalar>
Vec3<Scalar> acceleration(const Vec3<Scalar>& r_ecef,
                          double mu = constants::earth::mu) {
    const Scalar r_mag = janus::norm(r_ecef);
    const Scalar r_cubed = r_mag * r_mag * r_mag;

    // g = -μ/r³ · r
    return -mu / r_cubed * r_ecef;
}

/**
 * @brief Point mass gravitational potential
 *
 * U = -μ/r
 *
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF frame [m]
 * @param mu Gravitational parameter [m³/s²]
 * @return Gravitational potential [m²/s²]
 */
template <typename Scalar>
Scalar potential(const Vec3<Scalar>& r_ecef,
                 double mu = constants::earth::mu) {
    const Scalar r_mag = janus::norm(r_ecef);
    return -mu / r_mag;
}

/**
 * @brief Gravitational acceleration magnitude at distance r
 *
 * |g| = μ/r²
 *
 * Convenience function when only magnitude is needed.
 */
template <typename Scalar>
Scalar acceleration_magnitude(const Scalar& r_mag,
                              double mu = constants::earth::mu) {
    return mu / (r_mag * r_mag);
}

} // namespace vulcan::gravity::point_mass
```

### 3.2 J2 Gravity Model (`J2.hpp`)

Models Earth's oblateness (equatorial bulge). This is the dominant perturbation for LEO satellites.

**Mathematical Background:**

The J2 perturbing acceleration in ECEF coordinates:

```
a_x = -μx/r³ · [1 - 1.5·J2·(R_eq/r)²·(5(z/r)² - 1)]
a_y = -μy/r³ · [1 - 1.5·J2·(R_eq/r)²·(5(z/r)² - 1)]
a_z = -μz/r³ · [1 - 1.5·J2·(R_eq/r)²·(5(z/r)² - 3)]
```

```cpp
// include/vulcan/gravity/J2.hpp
#pragma once

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
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF frame [m]
 * @param mu Gravitational parameter [m³/s²]
 * @param J2 J2 zonal harmonic coefficient
 * @param R_eq Equatorial radius [m]
 * @return Gravitational acceleration in ECEF [m/s²]
 */
template <typename Scalar>
Vec3<Scalar> acceleration(const Vec3<Scalar>& r_ecef,
                          double mu = constants::earth::mu,
                          double J2 = constants::earth::J2,
                          double R_eq = constants::earth::R_eq) {
    const Scalar x = r_ecef(0);
    const Scalar y = r_ecef(1);
    const Scalar z = r_ecef(2);

    const Scalar r2 = x*x + y*y + z*z;
    const Scalar r = janus::sqrt(r2);
    const Scalar r5 = r2 * r2 * r;

    // Precompute common terms
    const Scalar z2_over_r2 = z * z / r2;
    const Scalar R_eq_over_r_sq = (R_eq * R_eq) / r2;
    const Scalar J2_factor = 1.5 * J2 * R_eq_over_r_sq;

    // Point mass term coefficient: -μ/r³
    const Scalar pm_coeff = -mu / (r2 * r);

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
 */
template <typename Scalar>
Scalar potential(const Vec3<Scalar>& r_ecef,
                 double mu = constants::earth::mu,
                 double J2 = constants::earth::J2,
                 double R_eq = constants::earth::R_eq) {
    const Scalar x = r_ecef(0);
    const Scalar y = r_ecef(1);
    const Scalar z = r_ecef(2);

    const Scalar r2 = x*x + y*y + z*z;
    const Scalar r = janus::sqrt(r2);

    // sin(φ) = z/r (geocentric latitude)
    const Scalar sin_phi = z / r;
    const Scalar sin_phi_sq = sin_phi * sin_phi;

    // P₂(sin φ) = (3 sin²φ - 1) / 2
    const Scalar P2 = (3.0 * sin_phi_sq - 1.0) / 2.0;

    // U = -μ/r · [1 - J2·(R_eq/r)²·P₂]
    const Scalar R_eq_over_r_sq = (R_eq * R_eq) / r2;

    return -mu / r * (1.0 - J2 * R_eq_over_r_sq * P2);
}

} // namespace vulcan::gravity::j2
```

### 3.3 J2-J4 Gravity Model (`J2J4.hpp`)

Extends J2 with J3 (pear-shape) and J4 (higher-order oblateness) terms.

```cpp
// include/vulcan/gravity/J2J4.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::gravity::j2j4 {

/**
 * @brief J2/J3/J4 gravitational acceleration
 *
 * Includes first three zonal harmonics for high-fidelity gravity modeling.
 * J3 captures the slight pear shape of Earth.
 * J4 captures additional higher-order oblateness.
 *
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF frame [m]
 * @return Gravitational acceleration in ECEF [m/s²]
 */
template <typename Scalar>
Vec3<Scalar> acceleration(const Vec3<Scalar>& r_ecef,
                          double mu = constants::earth::mu,
                          double J2 = constants::earth::J2,
                          double J3 = constants::earth::J3,
                          double J4 = constants::earth::J4,
                          double R_eq = constants::earth::R_eq) {
    const Scalar x = r_ecef(0);
    const Scalar y = r_ecef(1);
    const Scalar z = r_ecef(2);

    const Scalar r2 = x*x + y*y + z*z;
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
    const Scalar J2_xy = -1.5 * J2 * Re_r2 * (5.0 * z_r2 - 1.0);
    const Scalar J2_z = -1.5 * J2 * Re_r2 * (5.0 * z_r2 - 3.0);

    // ============================================
    // J3 Terms (odd harmonic - asymmetric about equator)
    // ============================================
    const Scalar J3_xy = 2.5 * J3 * Re_r3 * (7.0 * z_r3 - 3.0 * z_r);
    const Scalar J3_z = 2.5 * J3 * Re_r3 * (7.0 * z_r3 - 4.5 * z_r -
                        1.5 * z_r / janus::where(z_r2 > 1e-10, z_r2, Scalar(1e-10)));
    // Note: The J3 z-term has a special form; we use a simplified version
    // that's valid away from the equatorial plane
    const Scalar J3_z_safe = 2.5 * J3 * Re_r3 * z_r * (7.0 * z_r2 - 3.0);

    // ============================================
    // J4 Terms
    // ============================================
    const Scalar J4_xy = (15.0/8.0) * J4 * Re_r4 * (1.0 - 14.0 * z_r2 + 21.0 * z_r4);
    const Scalar J4_z = (15.0/8.0) * J4 * Re_r4 * (5.0 - 70.0/3.0 * z_r2 + 21.0 * z_r4);

    // ============================================
    // Combined acceleration
    // ============================================
    const Scalar factor_xy = 1.0 + J2_xy + J3_xy + J4_xy;
    const Scalar factor_z = 1.0 + J2_z + J3_z_safe + J4_z;

    Vec3<Scalar> accel;
    accel(0) = base * x * factor_xy;
    accel(1) = base * y * factor_xy;
    accel(2) = base * z * factor_z;

    return accel;
}

/**
 * @brief J2/J3/J4 gravitational potential
 */
template <typename Scalar>
Scalar potential(const Vec3<Scalar>& r_ecef,
                 double mu = constants::earth::mu,
                 double J2 = constants::earth::J2,
                 double J3 = constants::earth::J3,
                 double J4 = constants::earth::J4,
                 double R_eq = constants::earth::R_eq) {
    const Scalar x = r_ecef(0);
    const Scalar y = r_ecef(1);
    const Scalar z = r_ecef(2);

    const Scalar r2 = x*x + y*y + z*z;
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
    const Scalar correction = J2 * Re_r2 * P2 +
                              J3 * Re_r3 * P3 +
                              J4 * Re_r4 * P4;

    return -mu / r * (1.0 - correction);
}

} // namespace vulcan::gravity::j2j4
```

### 3.4 Spherical Harmonics (`SphericalHarmonics.hpp`)

General expansion for high-fidelity applications.

```cpp
// include/vulcan/gravity/SphericalHarmonics.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/coordinates/Geodetic.hpp>

#include <vector>
#include <array>

namespace vulcan::gravity::spherical_harmonics {

/**
 * @brief Gravity model coefficients container
 *
 * Stores normalized C and S coefficients for spherical harmonic expansion.
 * Supports up to degree/order N_MAX (compile-time or runtime).
 */
struct GravityCoefficients {
    int n_max;                              ///< Maximum degree
    std::vector<std::vector<double>> C;     ///< Cosine coefficients C[n][m]
    std::vector<std::vector<double>> S;     ///< Sine coefficients S[n][m]
    double mu;                              ///< Gravitational parameter [m³/s²]
    double R_eq;                            ///< Reference radius [m]

    /// Initialize with given maximum degree
    GravityCoefficients(int n_max_val = 20,
                        double mu_val = constants::earth::mu,
                        double R_eq_val = constants::earth::R_eq)
        : n_max(n_max_val), mu(mu_val), R_eq(R_eq_val) {
        C.resize(n_max + 1);
        S.resize(n_max + 1);
        for (int n = 0; n <= n_max; ++n) {
            C[n].resize(n + 1, 0.0);
            S[n].resize(n + 1, 0.0);
        }
        // J2, J3, J4 as zonal harmonics (m=0)
        if (n_max >= 2) C[2][0] = -constants::earth::J2;
        if (n_max >= 3) C[3][0] = -constants::earth::J3;
        if (n_max >= 4) C[4][0] = -constants::earth::J4;
    }
};

// Default Earth model (zonal harmonics only)
inline const GravityCoefficients& default_coefficients() {
    static GravityCoefficients coeffs(4);
    return coeffs;
}

/**
 * @brief Compute associated Legendre polynomial P_nm(x)
 *
 * Uses recurrence relations, fully symbolic-compatible.
 *
 * @tparam Scalar double or casadi::MX
 * @param n Degree
 * @param m Order
 * @param x Argument (typically sin(latitude))
 * @return P_nm(x)
 */
template <typename Scalar>
Scalar legendre_Pnm(int n, int m, const Scalar& x) {
    // Handle m > n case
    if (m > n) return Scalar(0.0);

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

    if (n == m) return pmm;

    // P_{m+1,m} = x(2m+1)P_mm
    Scalar pmm1 = x * (2.0 * m + 1.0) * pmm;
    if (n == m + 1) return pmm1;

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
 * General expansion:
 * U = μ/r Σ Σ (R_eq/r)^n [C_nm cos(mλ) + S_nm sin(mλ)] P_nm(sin φ)
 *
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF [m]
 * @param coeffs Gravity coefficients (C, S, n_max)
 * @return Acceleration in ECEF [m/s²]
 *
 * @note For symbolic mode, n_max must be fixed at trace time (structural loop).
 */
template <typename Scalar>
Vec3<Scalar> acceleration(const Vec3<Scalar>& r_ecef,
                          const GravityCoefficients& coeffs = default_coefficients()) {
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
    Scalar dU_dr = Scalar(0.0);     // ∂U/∂r
    Scalar dU_dlat = Scalar(0.0);   // ∂U/∂φ
    Scalar dU_dlon = Scalar(0.0);   // ∂U/∂λ

    // Summation over degrees and orders
    // Loop bounds are structural (n_max is int, not Scalar)
    for (int n = 0; n <= n_max; ++n) {
        const Scalar Re_r_n = janus::pow(R_eq / r, static_cast<double>(n));

        for (int m = 0; m <= n; ++m) {
            const double C_nm = coeffs.C[n][m];
            const double S_nm = coeffs.S[n][m];

            // Skip zero coefficients for efficiency
            if (C_nm == 0.0 && S_nm == 0.0) continue;

            const Scalar cos_m_lon = janus::cos(static_cast<double>(m) * lon);
            const Scalar sin_m_lon = janus::sin(static_cast<double>(m) * lon);

            const Scalar P_nm = legendre_Pnm(n, m, sin_lat);

            // Derivative of Legendre polynomial w.r.t. latitude
            // dP_nm/dφ = [n tan(φ) P_nm - (n+m) / cos(φ) P_{n-1,m}]
            // (simplified form for numerical stability)
            const Scalar P_nm1 = (n > 0) ? legendre_Pnm(n - 1, m, sin_lat) : Scalar(0.0);
            const Scalar dP_dlat = static_cast<double>(n) * janus::tan(lat_gc) * P_nm -
                                   static_cast<double>(n + m) / cos_lat * P_nm1;

            const Scalar trig_term = C_nm * cos_m_lon + S_nm * sin_m_lon;
            const Scalar dtrig_dlon = static_cast<double>(m) * (-C_nm * sin_m_lon + S_nm * cos_m_lon);

            // Accumulate partial derivatives
            dU_dr += -static_cast<double>(n + 1) * Re_r_n / r * trig_term * P_nm;
            dU_dlat += Re_r_n * trig_term * dP_dlat;
            dU_dlon += Re_r_n * dtrig_dlon * P_nm;
        }
    }

    // Scale by μ/r
    const Scalar scale = mu / r;
    dU_dr *= scale;
    dU_dlat *= scale;
    dU_dlon *= scale;

    // Convert spherical gradient to ECEF acceleration
    // g = -∇U
    const Scalar sin_lon = janus::sin(lon);
    const Scalar cos_lon = janus::cos(lon);

    // Spherical to Cartesian transformation
    const Scalar g_r = -dU_dr;
    const Scalar g_lat = -dU_dlat / r;
    const Scalar g_lon = -dU_dlon / (r * cos_lat);

    // Transform to ECEF
    // [g_x]   [cos(φ)cos(λ)  -sin(φ)cos(λ)  -sin(λ)] [g_r  ]
    // [g_y] = [cos(φ)sin(λ)  -sin(φ)sin(λ)   cos(λ)] [g_lat]
    // [g_z]   [sin(φ)         cos(φ)         0     ] [g_lon]
    Vec3<Scalar> g_ecef;
    g_ecef(0) = cos_lat * cos_lon * g_r - sin_lat * cos_lon * g_lat - sin_lon * g_lon;
    g_ecef(1) = cos_lat * sin_lon * g_r - sin_lat * sin_lon * g_lat + cos_lon * g_lon;
    g_ecef(2) = sin_lat * g_r + cos_lat * g_lat;

    return g_ecef;
}

/**
 * @brief Spherical harmonic gravitational potential
 */
template <typename Scalar>
Scalar potential(const Vec3<Scalar>& r_ecef,
                 const GravityCoefficients& coeffs = default_coefficients()) {
    const Spherical<Scalar> sph = ecef_to_spherical(r_ecef);
    const Scalar r = sph.radius;
    const Scalar lon = sph.lon;
    const Scalar lat_gc = sph.lat_gc;

    const Scalar sin_lat = janus::sin(lat_gc);

    const double mu = coeffs.mu;
    const double R_eq = coeffs.R_eq;
    const int n_max = coeffs.n_max;

    Scalar U = -mu / r;  // Point mass term

    for (int n = 2; n <= n_max; ++n) {
        const Scalar Re_r_n = janus::pow(R_eq / r, static_cast<double>(n));

        for (int m = 0; m <= n; ++m) {
            const double C_nm = coeffs.C[n][m];
            const double S_nm = coeffs.S[n][m];

            if (C_nm == 0.0 && S_nm == 0.0) continue;

            const Scalar cos_m_lon = janus::cos(static_cast<double>(m) * lon);
            const Scalar sin_m_lon = janus::sin(static_cast<double>(m) * lon);
            const Scalar P_nm = legendre_Pnm(n, m, sin_lat);

            U -= mu / r * Re_r_n * (C_nm * cos_m_lon + S_nm * sin_m_lon) * P_nm;
        }
    }

    return U;
}

} // namespace vulcan::gravity::spherical_harmonics
```

### 3.5 Shared Types (`GravityTypes.hpp`)

```cpp
// include/vulcan/gravity/GravityTypes.hpp
#pragma once

#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::gravity {

/**
 * @brief Gravitational state vector
 *
 * Contains both acceleration and potential for efficiency when both are needed.
 */
template <typename Scalar>
struct GravityState {
    Vec3<Scalar> acceleration;  ///< Gravitational acceleration [m/s²]
    Scalar potential;           ///< Gravitational potential [m²/s²]
};

} // namespace vulcan::gravity
```

### 3.6 Umbrella Header (`Gravity.hpp`)

```cpp
// include/vulcan/gravity/Gravity.hpp
#pragma once

#include <vulcan/gravity/GravityTypes.hpp>
#include <vulcan/gravity/PointMass.hpp>
#include <vulcan/gravity/J2.hpp>
#include <vulcan/gravity/J2J4.hpp>
#include <vulcan/gravity/SphericalHarmonics.hpp>
```

---

## 4. Testing Strategy

### 4.1 Test Categories

| Category | Purpose | Data Source |
|----------|---------|-------------|
| **Constants Verification** | Ensure constants match standards | IERS/IAU values |
| **Numeric Accuracy** | Compare against reference implementations | STK/GMAT/Vallado |
| **Symbolic Compatibility** | Verify autodiff works | Finite difference comparison |
| **Edge Cases** | Poles, equator, singularities | Manual analysis |
| **Consistency** | Compare models at limiting cases | Internal cross-check |

### 4.2 Test Implementation

```cpp
// tests/gravity/test_j2.cpp
#include <gtest/gtest.h>
#include <vulcan/gravity/J2.hpp>
#include <janus/janus.hpp>

using namespace vulcan;
using namespace vulcan::gravity;

// ==============================================
// Numeric Tests
// ==============================================

TEST(J2Gravity, EquatorialSurface) {
    // At equator, J2 correction should reduce gravity
    Vec3<double> r_eq;
    r_eq << constants::earth::R_eq, 0.0, 0.0;

    auto g_j2 = j2::acceleration(r_eq);
    auto g_pm = point_mass::acceleration(r_eq);

    // J2 should reduce equatorial gravity
    EXPECT_LT(janus::norm(g_j2), janus::norm(g_pm));
}

TEST(J2Gravity, PolarSurface) {
    // At poles, J2 correction should increase gravity
    Vec3<double> r_pole;
    r_pole << 0.0, 0.0, constants::earth::R_pol;

    auto g_j2 = j2::acceleration(r_pole);
    auto g_pm = point_mass::acceleration(r_pole);

    // J2 should increase polar gravity
    EXPECT_GT(janus::norm(g_j2), janus::norm(g_pm));
}

TEST(J2Gravity, ReferenceValue_LEO) {
    // Compare against known LEO gravity value
    // Reference: 400 km altitude at equator
    Vec3<double> r;
    r << constants::earth::R_eq + 400000.0, 0.0, 0.0;

    auto g = j2::acceleration(r);
    double g_mag = janus::norm(g);

    // Expected ~8.7 m/s² at 400 km
    EXPECT_NEAR(g_mag, 8.7, 0.1);
}

// ==============================================
// Symbolic Tests
// ==============================================

TEST(J2Gravity, SymbolicEvaluation) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r;
    r << x, y, z;

    auto g = j2::acceleration(r);

    // Evaluate at specific point
    double R = constants::earth::R_eq;
    auto g0 = janus::eval(g(0), {{"x", R}, {"y", 0.0}, {"z", 0.0}});
    auto g1 = janus::eval(g(1), {{"x", R}, {"y", 0.0}, {"z", 0.0}});
    auto g2 = janus::eval(g(2), {{"x", R}, {"y", 0.0}, {"z", 0.0}});

    // Should point toward Earth center
    EXPECT_LT(g0, 0.0);
    EXPECT_NEAR(g1, 0.0, 1e-10);
    EXPECT_NEAR(g2, 0.0, 1e-10);
}

TEST(J2Gravity, SymbolicGradient) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r;
    r << x, y, z;

    // Potential
    auto U = j2::potential(r);

    // Compute gradient symbolically
    auto dU_dx = janus::jacobian(U, x);
    auto dU_dy = janus::jacobian(U, y);
    auto dU_dz = janus::jacobian(U, z);

    // Acceleration from gradient: g = -∇U
    // Compare with direct acceleration computation
    double R = constants::earth::R_eq + 500000.0;  // 500 km altitude

    auto g = j2::acceleration(r);

    double test_x = R, test_y = 0.0, test_z = R * 0.1;  // Slight inclination
    std::map<std::string, double> vals = {{"x", test_x}, {"y", test_y}, {"z", test_z}};

    double grad_x = -janus::eval(dU_dx, vals);
    double accel_x = janus::eval(g(0), vals);

    EXPECT_NEAR(grad_x, accel_x, 1e-6);
}

// ==============================================
// Consistency Tests
// ==============================================

TEST(J2Gravity, J2ReducesToPointMass) {
    // With J2=0, J2 model should equal point mass
    Vec3<double> r;
    r << 7000000.0, 1000000.0, 500000.0;

    auto g_j2 = j2::acceleration(r, constants::earth::mu, 0.0, constants::earth::R_eq);
    auto g_pm = point_mass::acceleration(r);

    EXPECT_NEAR(g_j2(0), g_pm(0), 1e-12);
    EXPECT_NEAR(g_j2(1), g_pm(1), 1e-12);
    EXPECT_NEAR(g_j2(2), g_pm(2), 1e-12);
}
```

---

## 5. Verification & Validation

### 5.1 Reference Data Sources

| Source | Models | Notes |
|--------|--------|-------|
| **Vallado (2013)** | J2, J2-J4 | "Fundamentals of Astrodynamics" |
| **Montenbruck & Gill (2000)** | All | "Satellite Orbits" textbook |
| **IERS Conventions (2010)** | Constants | Official IAU/IERS values |
| **STK/GMAT** | All | Cross-validation with commercial/open tools |

### 5.2 Validation Test Cases

```
Test Case 1: ISS Orbit (LEO)
- Altitude: 400 km, Inclination: 51.6°
- Expected J2 effect: ~5 km/day nodal precession

Test Case 2: Molniya Orbit (HEO)
- Perigee: 500 km, Apogee: 40,000 km
- Critical inclination: 63.4°

Test Case 3: Geostationary Orbit (GEO)
- Altitude: 35,786 km
- J2 effect should be minimal

Test Case 4: Sun-Synchronous Orbit
- Altitude: ~700 km, Inclination: ~98°
- J2 used for nodal precession matching solar motion
```

---

## 6. Implementation Phases

### Phase 6.1: Point Mass (1-2 hours)
- [x] Create `include/vulcan/gravity/` directory
- [ ] Implement `PointMass.hpp`
- [ ] Implement `GravityTypes.hpp`
- [ ] Extend `tests/gravity/test_point_mass.cpp`
- [ ] Add numeric and symbolic tests

### Phase 6.2: J2 Model (2-3 hours)
- [ ] Implement `J2.hpp`
- [ ] Create `tests/gravity/test_j2.cpp`
- [ ] Validate against Vallado reference values
- [ ] Test symbolic gradient computation

### Phase 6.3: J2-J4 Model (2-3 hours)
- [ ] Implement `J2J4.hpp`
- [ ] Create `tests/gravity/test_j2j4.cpp`
- [ ] Validate J3 asymmetry (north/south difference)
- [ ] Cross-check with J2-only model

### Phase 6.4: Spherical Harmonics (3-4 hours)
- [ ] Implement `SphericalHarmonics.hpp`
- [ ] Implement Legendre polynomial recursion
- [ ] Create `tests/gravity/test_spherical_harmonics.cpp`
- [ ] Validate zonal-only case matches J2J4 model
- [ ] Add EGM96/EGM2008 coefficient loader (numeric only)

### Phase 6.5: Documentation & Examples (1-2 hours)
- [ ] Create `examples/gravity/gravity_comparison.cpp`
- [ ] Create `examples/gravity/orbit_perturbation.cpp`
- [ ] Update `docs/user_guides/gravity_models.md`
- [ ] Add to `vulcan.hpp` umbrella header

### Phase 6.6: Integration & Verification (1-2 hours)
- [ ] Run full test suite
- [ ] Verify all examples run correctly
- [ ] Cross-validate models against each other
- [ ] Performance benchmarking (optional)

---

## Appendix A: Mathematical Reference

### Gravitational Potential Expansion

The gravitational potential outside a body can be expressed as:

```
U(r,φ,λ) = μ/r [1 + Σ Σ (R_eq/r)^n (C_nm cos(mλ) + S_nm sin(mλ)) P_nm(sin φ)]
```

where:
- `r` = radial distance from center
- `φ` = geocentric latitude
- `λ` = longitude
- `P_nm` = associated Legendre polynomial of degree n, order m
- `C_nm, S_nm` = spherical harmonic coefficients
- `R_eq` = reference radius (equatorial)

### Zonal Harmonics (m = 0)

For zonal harmonics (axially symmetric), only `C_n0` terms exist:
- `C_20 = -J2` (oblateness)
- `C_30 = -J3` (pear shape)
- `C_40 = -J4` (higher-order oblateness)

### J2 Acceleration Derivation

From `U = -μ/r [1 - J2 (R_eq/r)² P_2(sin φ)]`:

The acceleration `g = -∇U` in ECEF coordinates:

```
a_x = -μx/r³ [1 - 1.5 J2 (R_eq/r)² (5z²/r² - 1)]
a_y = -μy/r³ [1 - 1.5 J2 (R_eq/r)² (5z²/r² - 1)]
a_z = -μz/r³ [1 - 1.5 J2 (R_eq/r)² (5z²/r² - 3)]
```

---

## Appendix B: Constants Reference

From `vulcan/core/Constants.hpp`:

| Constant | Value | Units | Source |
|----------|-------|-------|--------|
| μ (Earth) | 3.986004418e14 | m³/s² | IERS 2010 |
| R_eq | 6378137.0 | m | WGS84 |
| J2 | 1.08263e-3 | - | EGM96 |
| J3 | -2.54e-6 | - | EGM96 |
| J4 | -1.61e-6 | - | EGM96 |

---

This implementation plan should provide all context needed to implement robust, Janus-compatible gravity models for Vulcan.
