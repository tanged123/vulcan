# Phase 13: Orbital Mechanics Primitives

> **Purpose**: Orbital mechanics state conversions, two-body computations, transfer analysis, and analytical third-body ephemeris (Sun and Moon models).

> **Dependencies**: Phase 9 (Time Systems) - required for ephemeris time conversions.

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

Implement classical two-body orbital mechanics computations and analytical ephemeris for third-body positioning:

| Component | Purpose | Symbolic Support |
|-----------|---------|------------------|
| **State Conversions** | Cartesian ↔ Keplerian | Full |
| **Anomaly Solvers** | Mean/Eccentric/True conversions | Full |
| **Orbital Quantities** | Period, velocity, energy | Full |
| **Transfer Computations** | Hohmann, bielliptic, plane change | Full |
| **Sun Ephemeris (Analytical)** | Sun position for third-body gravity | Full |
| **Moon Ephemeris (Analytical)** | Moon position for third-body gravity | Full |

### Design Principles

Following Vulcan's stateless philosophy:
- **No propagation** - that belongs in Icarus components
- **Pure functions** - inputs → outputs with no side effects
- **Dual-backend** - works with both `double` and `casadi::MX`

---

## 2. Architecture

### Namespace Organization

```
vulcan::orbital::
├── elements::                     # State conversions
│   ├── cartesian_to_keplerian()
│   ├── keplerian_to_cartesian()
│   └── OrbitalElements struct
│
├── anomaly::                      # Anomaly conversions
│   ├── mean_to_eccentric()        # Kepler's equation
│   ├── eccentric_to_true()
│   └── true_to_eccentric()
│
├── quantities::                   # Orbital parameters
│   ├── period()
│   ├── velocity()                 # Vis-viva
│   ├── energy()
│   ├── escape_velocity()
│   └── circular_velocity()
│
├── transfer::                     # Maneuver planning
│   ├── hohmann_delta_v()
│   ├── bielliptic_delta_v()
│   └── plane_change_delta_v()
│
└── ephemeris::                    # Celestial body positions
    └── analytical::
        ├── sun_position_eci()
        ├── moon_position_eci()
        ├── sun_position_ecef()
        └── moon_position_ecef()
```

### File Structure

```
include/vulcan/orbital/
├── OrbitalTypes.hpp          # OrbitalElements struct, constants
├── StateConversions.hpp      # Cartesian ↔ Keplerian
├── AnomalyConversions.hpp    # Mean/Eccentric/True anomaly
├── OrbitalQuantities.hpp     # Period, velocity, energy
├── TransferMechanics.hpp     # Hohmann, bielliptic, plane change
├── AnalyticalEphemeris.hpp   # Sun/Moon positions (Meeus algorithms)
└── Orbital.hpp               # Umbrella header

tests/orbital/
├── test_state_conversions.cpp
├── test_anomaly_conversions.cpp
├── test_orbital_quantities.cpp
├── test_transfer_mechanics.cpp
└── test_analytical_ephemeris.cpp

examples/orbital/
├── orbit_elements_demo.cpp
└── ephemeris_demo.cpp
```

---

## 3. Implementation Details

### 3.1 Orbital Types (`OrbitalTypes.hpp`)

> [!IMPORTANT]
> **Constants & Code Unification**: Before implementing, consolidate celestial constants and ephemeris code:
>
> **1. Move `SolarPosition.hpp` to `orbital/ephemeris/`:**
> - Rename `environment/SolarPosition.hpp` → `orbital/AnalyticalEphemeris.hpp`
> - Keep `environment/Environment.hpp` re-exporting for backwards compatibility
> - Consolidate Sun + Moon ephemeris in one place
>
> **2. Add to `vulcan::constants::` in `core/Constants.hpp`:**
> ```cpp
> namespace sun {
>     inline constexpr double AU = 149597870700.0;  // Astronomical Unit [m]
>     inline constexpr double mu = 1.32712440018e20; // Gravitational parameter [m³/s²]
>     inline constexpr double radius = 6.96e8;       // Mean radius [m]
> }
> namespace moon {
>     inline constexpr double mu = 4.9028695e12;     // Gravitational parameter [m³/s²]
>     inline constexpr double radius = 1.7374e6;     // Mean radius [m]  
>     inline constexpr double mean_distance = 3.844e8; // Mean Earth-Moon distance [m]
> }
> ```
>
> **3. Update `environment/SolarPosition.hpp`** to just re-export:
> ```cpp
> // Backwards compatibility - solar ephemeris moved to orbital module
> #include <vulcan/orbital/AnalyticalEphemeris.hpp>
> namespace vulcan::environment::solar {
>     using namespace vulcan::orbital::ephemeris::analytical;
> }
> ```

```cpp
// include/vulcan/orbital/OrbitalTypes.hpp
#pragma once

#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/core/Constants.hpp>  // For AU

namespace vulcan::orbital {

// =============================================================================
// Orbital Elements Structure
// =============================================================================

/**
 * @brief Classical Keplerian orbital elements
 */
template <typename Scalar>
struct OrbitalElements {
    Scalar a;      ///< Semi-major axis [m]
    Scalar e;      ///< Eccentricity [-]
    Scalar i;      ///< Inclination [rad]
    Scalar Omega;  ///< Right ascension of ascending node (RAAN) [rad]
    Scalar omega;  ///< Argument of periapsis [rad]
    Scalar nu;     ///< True anomaly [rad]
};

// =============================================================================
// Celestial Body Constants
// =============================================================================

namespace constants {

namespace sun {
/// Gravitational parameter [m³/s²]
inline constexpr double mu = 1.32712440018e20;

/// Mean radius [m]
inline constexpr double radius = 6.96e8;

// NOTE: Use vulcan::constants::AU from core/Constants.hpp
// inline constexpr double AU = vulcan::constants::AU;
} // namespace sun

namespace moon {
/// Gravitational parameter [m³/s²]
inline constexpr double mu = 4.9028695e12;

/// Mean radius [m]
inline constexpr double radius = 1.7374e6;

/// Mean distance from Earth [m]
inline constexpr double mean_distance = 3.844e8;
} // namespace moon

} // namespace constants

} // namespace vulcan::orbital
```

### 3.2 State Conversions (`StateConversions.hpp`)

```cpp
// include/vulcan/orbital/StateConversions.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/orbital/OrbitalTypes.hpp>

namespace vulcan::orbital::elements {

/**
 * @brief Convert Cartesian state to Keplerian elements
 *
 * @tparam Scalar double or casadi::MX
 * @param r Position vector [m]
 * @param v Velocity vector [m/s]
 * @param mu Gravitational parameter [m³/s²]
 * @return OrbitalElements
 */
template <typename Scalar>
OrbitalElements<Scalar> cartesian_to_keplerian(const Vec3<Scalar>& r,
                                                const Vec3<Scalar>& v,
                                                double mu) {
    OrbitalElements<Scalar> oe;
    
    const Scalar r_mag = janus::norm(r);
    const Scalar v_mag = janus::norm(v);
    
    // Specific angular momentum
    const Vec3<Scalar> h = janus::cross(r, v);
    const Scalar h_mag = janus::norm(h);
    
    // Node vector
    Vec3<Scalar> n;
    n << -h(1), h(0), Scalar(0.0);
    const Scalar n_mag = janus::norm(n);
    
    // Eccentricity vector
    const Scalar r_dot_v = janus::dot(r, v);
    const Vec3<Scalar> e_vec = ((v_mag * v_mag - mu / r_mag) * r - r_dot_v * v) / mu;
    oe.e = janus::norm(e_vec);
    
    // Specific mechanical energy
    const Scalar energy = v_mag * v_mag / 2.0 - mu / r_mag;
    
    // Semi-major axis
    oe.a = -mu / (2.0 * energy);
    
    // Inclination
    oe.i = janus::acos(h(2) / h_mag);
    
    // RAAN
    oe.Omega = janus::acos(n(0) / n_mag);
    oe.Omega = janus::where(n(1) < 0.0, 2.0 * M_PI - oe.Omega, oe.Omega);
    
    // Argument of periapsis
    oe.omega = janus::acos(janus::dot(n, e_vec) / (n_mag * oe.e));
    oe.omega = janus::where(e_vec(2) < 0.0, 2.0 * M_PI - oe.omega, oe.omega);
    
    // True anomaly
    oe.nu = janus::acos(janus::dot(e_vec, r) / (oe.e * r_mag));
    oe.nu = janus::where(r_dot_v < 0.0, 2.0 * M_PI - oe.nu, oe.nu);
    
    return oe;
}

/**
 * @brief Convert Keplerian elements to Cartesian state
 *
 * @tparam Scalar double or casadi::MX
 * @param oe Orbital elements
 * @param mu Gravitational parameter [m³/s²]
 * @return Pair of position and velocity vectors
 */
template <typename Scalar>
std::pair<Vec3<Scalar>, Vec3<Scalar>> keplerian_to_cartesian(
    const OrbitalElements<Scalar>& oe, double mu) {
    
    // Semi-latus rectum
    const Scalar p = oe.a * (1.0 - oe.e * oe.e);
    
    // Position in perifocal frame
    const Scalar r_mag = p / (1.0 + oe.e * janus::cos(oe.nu));
    
    const Scalar cos_nu = janus::cos(oe.nu);
    const Scalar sin_nu = janus::sin(oe.nu);
    
    Vec3<Scalar> r_pqw;
    r_pqw << r_mag * cos_nu, r_mag * sin_nu, Scalar(0.0);
    
    // Velocity in perifocal frame
    const Scalar sqrt_mu_p = janus::sqrt(mu / p);
    Vec3<Scalar> v_pqw;
    v_pqw << -sqrt_mu_p * sin_nu, sqrt_mu_p * (oe.e + cos_nu), Scalar(0.0);
    
    // Rotation matrix from perifocal to ECI
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
```

### 3.3 Anomaly Conversions (`AnomalyConversions.hpp`)

```cpp
// include/vulcan/orbital/AnomalyConversions.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::orbital::anomaly {

/**
 * @brief Solve Kepler's equation: M = E - e*sin(E)
 *
 * Uses Newton-Raphson iteration for numeric types.
 * For symbolic types, uses a fixed number of Newton steps.
 *
 * @tparam Scalar double or casadi::MX
 * @param M Mean anomaly [rad]
 * @param e Eccentricity [-]
 * @param tol Convergence tolerance (numeric only)
 * @param max_iter Maximum iterations (numeric only)
 * @return Eccentric anomaly [rad]
 */
template <typename Scalar>
Scalar mean_to_eccentric(const Scalar& M, const Scalar& e,
                          double tol = 1e-12, int max_iter = 50) {
    // Initial guess
    Scalar E = M;
    
    if constexpr (std::is_same_v<Scalar, double>) {
        // Newton-Raphson for numeric
        for (int i = 0; i < max_iter; ++i) {
            double f = E - e * std::sin(E) - M;
            double f_prime = 1.0 - e * std::cos(E);
            double dE = f / f_prime;
            E -= dE;
            if (std::abs(dE) < tol) break;
        }
    } else {
        // Fixed Newton steps for symbolic (enables autodiff)
        // 10 iterations sufficient for most cases
        for (int i = 0; i < 10; ++i) {
            Scalar f = E - e * janus::sin(E) - M;
            Scalar f_prime = 1.0 - e * janus::cos(E);
            E = E - f / f_prime;
        }
    }
    
    return E;
}

/**
 * @brief Convert eccentric anomaly to true anomaly
 *
 * @tparam Scalar double or casadi::MX
 * @param E Eccentric anomaly [rad]
 * @param e Eccentricity [-]
 * @return True anomaly [rad]
 */
template <typename Scalar>
Scalar eccentric_to_true(const Scalar& E, const Scalar& e) {
    const Scalar beta = e / (1.0 + janus::sqrt(1.0 - e * e));
    return E + 2.0 * janus::atan2(beta * janus::sin(E),
                                   1.0 - beta * janus::cos(E));
}

/**
 * @brief Convert true anomaly to eccentric anomaly
 *
 * @tparam Scalar double or casadi::MX
 * @param nu True anomaly [rad]
 * @param e Eccentricity [-]
 * @return Eccentric anomaly [rad]
 */
template <typename Scalar>
Scalar true_to_eccentric(const Scalar& nu, const Scalar& e) {
    return janus::atan2(janus::sqrt(1.0 - e * e) * janus::sin(nu),
                        e + janus::cos(nu));
}

} // namespace vulcan::orbital::anomaly
```

### 3.4 Orbital Quantities (`OrbitalQuantities.hpp`)

```cpp
// include/vulcan/orbital/OrbitalQuantities.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::orbital::quantities {

/**
 * @brief Orbital period
 *
 * @tparam Scalar double or casadi::MX
 * @param a Semi-major axis [m]
 * @param mu Gravitational parameter [m³/s²]
 * @return Period [s]
 */
template <typename Scalar>
Scalar period(const Scalar& a, double mu) {
    return 2.0 * M_PI * janus::sqrt(a * a * a / mu);
}

/**
 * @brief Orbital velocity (vis-viva equation)
 *
 * @tparam Scalar double or casadi::MX
 * @param r Current radius [m]
 * @param a Semi-major axis [m]
 * @param mu Gravitational parameter [m³/s²]
 * @return Velocity magnitude [m/s]
 */
template <typename Scalar>
Scalar velocity(const Scalar& r, const Scalar& a, double mu) {
    return janus::sqrt(mu * (2.0 / r - 1.0 / a));
}

/**
 * @brief Specific orbital energy
 *
 * @tparam Scalar double or casadi::MX
 * @param a Semi-major axis [m]
 * @param mu Gravitational parameter [m³/s²]
 * @return Specific energy [J/kg]
 */
template <typename Scalar>
Scalar energy(const Scalar& a, double mu) {
    return -mu / (2.0 * a);
}

/**
 * @brief Escape velocity at given radius
 *
 * @tparam Scalar double or casadi::MX
 * @param r Radius [m]
 * @param mu Gravitational parameter [m³/s²]
 * @return Escape velocity [m/s]
 */
template <typename Scalar>
Scalar escape_velocity(const Scalar& r, double mu) {
    return janus::sqrt(2.0 * mu / r);
}

/**
 * @brief Circular orbit velocity at given radius
 *
 * @tparam Scalar double or casadi::MX
 * @param r Radius [m]
 * @param mu Gravitational parameter [m³/s²]
 * @return Circular velocity [m/s]
 */
template <typename Scalar>
Scalar circular_velocity(const Scalar& r, double mu) {
    return janus::sqrt(mu / r);
}

} // namespace vulcan::orbital::quantities
```

### 3.5 Transfer Mechanics (`TransferMechanics.hpp`)

```cpp
// include/vulcan/orbital/TransferMechanics.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/orbital/OrbitalQuantities.hpp>

namespace vulcan::orbital::transfer {

/**
 * @brief Hohmann transfer delta-v
 *
 * @tparam Scalar double or casadi::MX
 * @param r1 Initial orbit radius [m]
 * @param r2 Final orbit radius [m]
 * @param mu Gravitational parameter [m³/s²]
 * @return Pair of (dv1, dv2) [m/s]
 */
template <typename Scalar>
std::pair<Scalar, Scalar> hohmann_delta_v(const Scalar& r1,
                                           const Scalar& r2,
                                           double mu) {
    // Transfer orbit semi-major axis
    const Scalar a_t = (r1 + r2) / 2.0;
    
    // Velocities using vis-viva
    const Scalar v1 = quantities::circular_velocity(r1, mu);
    const Scalar v2 = quantities::circular_velocity(r2, mu);
    const Scalar v_t1 = quantities::velocity(r1, a_t, mu);
    const Scalar v_t2 = quantities::velocity(r2, a_t, mu);
    
    const Scalar dv1 = v_t1 - v1;
    const Scalar dv2 = v2 - v_t2;
    
    return {dv1, dv2};
}

/**
 * @brief Bielliptic transfer delta-v
 *
 * @tparam Scalar double or casadi::MX
 * @param r1 Initial orbit radius [m]
 * @param r2 Final orbit radius [m]
 * @param r_b Intermediate apoapsis radius [m]
 * @param mu Gravitational parameter [m³/s²]
 * @return Tuple of (dv1, dv2, dv3) [m/s]
 */
template <typename Scalar>
std::tuple<Scalar, Scalar, Scalar> bielliptic_delta_v(const Scalar& r1,
                                                        const Scalar& r2,
                                                        const Scalar& r_b,
                                                        double mu) {
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
    
    const Scalar dv1 = v_t1_peri - v1;
    const Scalar dv2 = v_t2_apo - v_t1_apo;
    const Scalar dv3 = v2 - v_t2_peri;
    
    return {dv1, dv2, dv3};
}

/**
 * @brief Simple plane change delta-v
 *
 * @tparam Scalar double or casadi::MX
 * @param v Orbital velocity magnitude [m/s]
 * @param delta_i Inclination change [rad]
 * @return Delta-v required [m/s]
 */
template <typename Scalar>
Scalar plane_change_delta_v(const Scalar& v, const Scalar& delta_i) {
    return 2.0 * v * janus::sin(delta_i / 2.0);
}

} // namespace vulcan::orbital::transfer
```

### 3.6 Analytical Ephemeris (`AnalyticalEphemeris.hpp`)

Based on Meeus, "Astronomical Algorithms" (1998). Accuracy: ~0.01° for Sun, ~0.3° for Moon.

```cpp
// include/vulcan/orbital/AnalyticalEphemeris.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/orbital/OrbitalTypes.hpp>

namespace vulcan::orbital::ephemeris::analytical {

/**
 * @brief Sun position in ECI (geocentric equatorial)
 *
 * Computes the Sun's position using the low-precision algorithm from
 * Meeus, "Astronomical Algorithms" (1998), Chapter 25.
 *
 * Accuracy: ~0.01° in longitude, ~1 arcmin in position
 * Valid range: 1950-2050 (acceptable accuracy outside this range)
 *
 * @tparam Scalar double or casadi::MX
 * @param jd_tt Julian Date in Terrestrial Time
 * @return Sun position in ECI (J2000) [m]
 */
template <typename Scalar>
Vec3<Scalar> sun_position_eci(const Scalar& jd_tt) {
    // Julian centuries from J2000.0
    const Scalar T = (jd_tt - 2451545.0) / 36525.0;

    // Mean longitude of the Sun (degrees)
    const Scalar L0 = 280.46646 + 36000.76983 * T + 0.0003032 * T * T;

    // Mean anomaly of the Sun (degrees)
    const Scalar M = 357.52911 + 35999.05029 * T - 0.0001537 * T * T;

    // Eccentricity of Earth's orbit
    const Scalar e = 0.016708634 - 0.000042037 * T - 0.0000001267 * T * T;

    // Convert to radians
    const Scalar M_rad = M * vulcan::constants::angle::deg2rad;

    // Equation of center (degrees)
    const Scalar C = (1.914602 - 0.004817 * T - 0.000014 * T * T) * janus::sin(M_rad) +
                     (0.019993 - 0.000101 * T) * janus::sin(2.0 * M_rad) +
                     0.000289 * janus::sin(3.0 * M_rad);

    // True longitude (degrees)
    const Scalar sun_lon = L0 + C;
    const Scalar sun_lon_rad = sun_lon * vulcan::constants::angle::deg2rad;

    // True anomaly
    const Scalar v = M + C;
    const Scalar v_rad = v * vulcan::constants::angle::deg2rad;

    // Distance in AU
    const Scalar R_AU = 1.000001018 * (1.0 - e * e) / (1.0 + e * janus::cos(v_rad));

    // Convert to meters
    const Scalar R = R_AU * constants::sun::AU;

    // Mean obliquity of the ecliptic (degrees)
    const Scalar epsilon = 23.439291 - 0.0130042 * T - 0.00000016 * T * T;
    const Scalar epsilon_rad = epsilon * vulcan::constants::angle::deg2rad;

    // Convert ecliptic to equatorial (ECI)
    const Scalar cos_lon = janus::cos(sun_lon_rad);
    const Scalar sin_lon = janus::sin(sun_lon_rad);
    const Scalar cos_eps = janus::cos(epsilon_rad);
    const Scalar sin_eps = janus::sin(epsilon_rad);

    Vec3<Scalar> r_sun;
    r_sun(0) = R * cos_lon;
    r_sun(1) = R * sin_lon * cos_eps;
    r_sun(2) = R * sin_lon * sin_eps;

    return r_sun;
}

/**
 * @brief Moon position in ECI (geocentric equatorial)
 *
 * Computes the Moon's position using a simplified analytical model
 * based on Meeus, "Astronomical Algorithms" (1998), Chapter 47.
 *
 * Accuracy: ~0.3° in longitude, ~10 km in position
 *
 * @tparam Scalar double or casadi::MX
 * @param jd_tt Julian Date in Terrestrial Time
 * @return Moon position in ECI (J2000) [m]
 */
template <typename Scalar>
Vec3<Scalar> moon_position_eci(const Scalar& jd_tt) {
    // Julian centuries from J2000.0
    const Scalar T = (jd_tt - 2451545.0) / 36525.0;
    const Scalar T2 = T * T;
    const Scalar T3 = T2 * T;
    const Scalar T4 = T3 * T;

    // Moon's mean longitude (degrees)
    const Scalar Lp = 218.3164477 + 481267.88123421 * T - 0.0015786 * T2 +
                      T3 / 538841.0 - T4 / 65194000.0;

    // Mean elongation of the Moon (degrees)
    const Scalar D = 297.8501921 + 445267.1114034 * T - 0.0018819 * T2 +
                     T3 / 545868.0 - T4 / 113065000.0;

    // Sun's mean anomaly (degrees)
    const Scalar M = 357.5291092 + 35999.0502909 * T - 0.0001536 * T2 +
                     T3 / 24490000.0;

    // Moon's mean anomaly (degrees)
    const Scalar Mp = 134.9633964 + 477198.8675055 * T + 0.0087414 * T2 +
                      T3 / 69699.0 - T4 / 14712000.0;

    // Moon's argument of latitude (degrees)
    const Scalar F = 93.2720950 + 483202.0175233 * T - 0.0036539 * T2 -
                     T3 / 3526000.0 + T4 / 863310000.0;

    // Convert to radians
    const Scalar D_rad = D * vulcan::constants::angle::deg2rad;
    const Scalar M_rad = M * vulcan::constants::angle::deg2rad;
    const Scalar Mp_rad = Mp * vulcan::constants::angle::deg2rad;
    const Scalar F_rad = F * vulcan::constants::angle::deg2rad;
    const Scalar Lp_rad = Lp * vulcan::constants::angle::deg2rad;

    // Longitude perturbations (simplified - main terms only)
    const Scalar dL = 6288774.0 * janus::sin(Mp_rad) +
                      1274027.0 * janus::sin(2.0 * D_rad - Mp_rad) +
                      658314.0 * janus::sin(2.0 * D_rad) +
                      213618.0 * janus::sin(2.0 * Mp_rad) -
                      185116.0 * janus::sin(M_rad) -
                      114332.0 * janus::sin(2.0 * F_rad);

    // Latitude perturbations (simplified)
    const Scalar dB = 5128122.0 * janus::sin(F_rad) +
                      280602.0 * janus::sin(Mp_rad + F_rad) +
                      277693.0 * janus::sin(Mp_rad - F_rad) +
                      173237.0 * janus::sin(2.0 * D_rad - F_rad);

    // Distance perturbations (simplified)
    const Scalar dR = -20905355.0 * janus::cos(Mp_rad) -
                      3699111.0 * janus::cos(2.0 * D_rad - Mp_rad) -
                      2955968.0 * janus::cos(2.0 * D_rad) -
                      569925.0 * janus::cos(2.0 * Mp_rad);

    // Ecliptic longitude and latitude (degrees)
    const Scalar lambda = Lp + dL / 1000000.0;
    const Scalar beta = dB / 1000000.0;

    // Distance (km, then convert to m)
    const Scalar dist_km = 385000.56 + dR / 1000.0;
    const Scalar dist = dist_km * 1000.0;

    // Convert to radians
    const Scalar lambda_rad = lambda * vulcan::constants::angle::deg2rad;
    const Scalar beta_rad = beta * vulcan::constants::angle::deg2rad;

    // Mean obliquity of the ecliptic
    const Scalar epsilon = 23.439291 - 0.0130042 * T;
    const Scalar epsilon_rad = epsilon * vulcan::constants::angle::deg2rad;

    // Ecliptic to equatorial transformation
    const Scalar cos_lambda = janus::cos(lambda_rad);
    const Scalar sin_lambda = janus::sin(lambda_rad);
    const Scalar cos_beta = janus::cos(beta_rad);
    const Scalar sin_beta = janus::sin(beta_rad);
    const Scalar cos_eps = janus::cos(epsilon_rad);
    const Scalar sin_eps = janus::sin(epsilon_rad);

    Vec3<Scalar> r_moon;
    r_moon(0) = dist * cos_beta * cos_lambda;
    r_moon(1) = dist * (cos_eps * cos_beta * sin_lambda - sin_eps * sin_beta);
    r_moon(2) = dist * (sin_eps * cos_beta * sin_lambda + cos_eps * sin_beta);

    return r_moon;
}

/**
 * @brief Sun position in ECEF
 *
 * @tparam Scalar double or casadi::MX
 * @param jd_tt Julian Date in Terrestrial Time
 * @return Sun position in ECEF [m]
 */
template <typename Scalar>
Vec3<Scalar> sun_position_ecef(const Scalar& jd_tt) {
    Vec3<Scalar> r_eci = sun_position_eci(jd_tt);

    // Compute Greenwich Mean Sidereal Time (GMST)
    const Scalar T = (jd_tt - 2451545.0) / 36525.0;
    const Scalar gmst_deg = 280.46061837 + 360.98564736629 * (jd_tt - 2451545.0) +
                            0.000387933 * T * T - T * T * T / 38710000.0;
    const Scalar gmst_rad = gmst_deg * vulcan::constants::angle::deg2rad;

    // Rotate from ECI to ECEF
    const Scalar cos_gmst = janus::cos(gmst_rad);
    const Scalar sin_gmst = janus::sin(gmst_rad);

    Vec3<Scalar> r_ecef;
    r_ecef(0) = cos_gmst * r_eci(0) + sin_gmst * r_eci(1);
    r_ecef(1) = -sin_gmst * r_eci(0) + cos_gmst * r_eci(1);
    r_ecef(2) = r_eci(2);

    return r_ecef;
}

/**
 * @brief Moon position in ECEF
 *
 * @tparam Scalar double or casadi::MX
 * @param jd_tt Julian Date in Terrestrial Time
 * @return Moon position in ECEF [m]
 */
template <typename Scalar>
Vec3<Scalar> moon_position_ecef(const Scalar& jd_tt) {
    Vec3<Scalar> r_eci = moon_position_eci(jd_tt);

    const Scalar T = (jd_tt - 2451545.0) / 36525.0;
    const Scalar gmst_deg = 280.46061837 + 360.98564736629 * (jd_tt - 2451545.0) +
                            0.000387933 * T * T - T * T * T / 38710000.0;
    const Scalar gmst_rad = gmst_deg * vulcan::constants::angle::deg2rad;

    const Scalar cos_gmst = janus::cos(gmst_rad);
    const Scalar sin_gmst = janus::sin(gmst_rad);

    Vec3<Scalar> r_ecef;
    r_ecef(0) = cos_gmst * r_eci(0) + sin_gmst * r_eci(1);
    r_ecef(1) = -sin_gmst * r_eci(0) + cos_gmst * r_eci(1);
    r_ecef(2) = r_eci(2);

    return r_ecef;
}

} // namespace vulcan::orbital::ephemeris::analytical
```

### 3.7 Umbrella Header (`Orbital.hpp`)

```cpp
// include/vulcan/orbital/Orbital.hpp
#pragma once

#include <vulcan/orbital/OrbitalTypes.hpp>
#include <vulcan/orbital/StateConversions.hpp>
#include <vulcan/orbital/AnomalyConversions.hpp>
#include <vulcan/orbital/OrbitalQuantities.hpp>
#include <vulcan/orbital/TransferMechanics.hpp>
#include <vulcan/orbital/AnalyticalEphemeris.hpp>
```

---

## 4. Testing Strategy

### 4.1 Test Categories

| Category | Purpose | Data Source |
|----------|---------|-------------|
| **Round-trip** | `cartesian → keplerian → cartesian` | Self-consistency |
| **Reference Values** | Known orbital parameters | Vallado textbook |
| **Ephemeris Accuracy** | Sun/Moon positions | JPL Horizons |
| **Symbolic Compatibility** | Autodiff works correctly | Finite difference |
| **Edge Cases** | Circular, equatorial, polar orbits | Analytical |

### 4.2 Test Examples

```cpp
// tests/orbital/test_state_conversions.cpp
TEST(StateConversions, RoundTrip_ISS) {
    // ISS-like orbit
    double mu = vulcan::constants::earth::mu;
    
    Vec3<double> r, v;
    r << -4453.783586, 5038.203756, -426.384456;  // km * 1000
    r *= 1000.0;  // to meters
    v << -3.829428, -2.943567, -5.611621;
    v *= 1000.0;  // to m/s
    
    auto oe = elements::cartesian_to_keplerian(r, v, mu);
    auto [r2, v2] = elements::keplerian_to_cartesian(oe, mu);
    
    EXPECT_NEAR((r - r2).norm(), 0.0, 1.0);  // Within 1 m
    EXPECT_NEAR((v - v2).norm(), 0.0, 1e-6);  // Within 1 mm/s
}

TEST(AnomalyConversions, KeplerEquation) {
    double M = 0.5;  // rad
    double e = 0.5;
    
    double E = anomaly::mean_to_eccentric(M, e);
    double M_check = E - e * std::sin(E);
    
    EXPECT_NEAR(M, M_check, 1e-12);
}

TEST(AnalyticalEphemeris, SunPosition_J2000) {
    double jd_j2000 = 2451545.0;
    
    Vec3<double> r_sun = ephemeris::analytical::sun_position_eci(jd_j2000);
    double dist = janus::norm(r_sun);
    
    // Sun should be ~1 AU away
    double AU = vulcan::orbital::constants::sun::AU;
    EXPECT_NEAR(dist / AU, 1.0, 0.02);  // Within 2%
}

TEST(AnalyticalEphemeris, MoonPosition_Approximate) {
    double jd = 2451545.0;
    
    Vec3<double> r_moon = ephemeris::analytical::moon_position_eci(jd);
    double dist = janus::norm(r_moon);
    
    // Moon ~384,000 km away
    double expected = vulcan::orbital::constants::moon::mean_distance;
    EXPECT_NEAR(dist, expected, expected * 0.1);  // Within 10%
}

TEST(AnalyticalEphemeris, SymbolicEvaluation) {
    auto jd = janus::sym("jd");
    auto r_sun = ephemeris::analytical::sun_position_eci(jd);
    
    double jd_val = 2451545.0;
    double x = janus::eval(r_sun(0), {{"jd", jd_val}});
    
    EXPECT_NE(x, 0.0);
}
```

---

## 5. Verification & Validation

### 5.1 Reference Sources

| Source | Data | Notes |
|--------|------|-------|
| **Vallado** | State conversion examples | "Fundamentals of Astrodynamics" |
| **JPL Horizons** | Ephemeris positions | https://ssd.jpl.nasa.gov/horizons/ |
| **STK/GMAT** | Cross-validation | Commercial/open tools |
| **Meeus** | Analytical formulas | "Astronomical Algorithms" |

### 5.2 Validation Cases

1. **ISS Orbit** - Compare computed elements against TLE-derived values
2. **GEO Satellite** - Verify circular equatorial orbit handling
3. **Molniya Orbit** - High eccentricity, high inclination
4. **Sun Position at Solstice** - Verify known geometry
5. **Moon Position at Full Moon** - Verify opposition geometry

---

## 6. Implementation Phases

### Phase 13.1: Core Types & Quantities (2 hours)
- [ ] Create `include/vulcan/orbital/` directory
- [ ] Implement `OrbitalTypes.hpp`
- [ ] Implement `OrbitalQuantities.hpp`
- [ ] Basic tests for period, velocity, energy

### Phase 13.2: State Conversions (3 hours)
- [ ] Implement `StateConversions.hpp`
- [ ] Implement `AnomalyConversions.hpp`
- [ ] Round-trip tests
- [ ] Reference value validation

### Phase 13.3: Transfer Mechanics (2 hours)
- [ ] Implement `TransferMechanics.hpp`
- [ ] Hohmann transfer tests
- [ ] Bielliptic transfer tests

### Phase 13.4: Analytical Ephemeris (3 hours)
- [ ] Implement `AnalyticalEphemeris.hpp`
- [ ] Sun position (Meeus algorithm)
- [ ] Moon position (Meeus algorithm)
- [ ] ECEF conversions
- [ ] Validate against JPL Horizons

### Phase 13.5: Symbolic Compatibility (1 hour)
- [ ] Verify all functions work with `casadi::MX`
- [ ] Test gradient computation
- [ ] Verify Kepler solver works symbolically

### Phase 13.6: Documentation & Examples (2 hours)
- [ ] Create `examples/orbital/orbit_elements_demo.cpp`
- [ ] Create `examples/orbital/ephemeris_demo.cpp`
- [ ] Update `docs/user_guides/`
- [ ] Add umbrella header to `vulcan.hpp`

---

## Appendix A: Ephemeris Accuracy Comparison

| Method | Sun Accuracy | Moon Accuracy | Data Required |
|--------|--------------|---------------|---------------|
| Analytical (Meeus) | ~0.01° | ~0.3° | None |
| JPL DE440 | <0.001" | <0.001" | Binary files |
| SPICE | <0.001" | <0.001" | Kernel files |

For trajectory optimization, analytical ephemeris is sufficient. High-precision navigation requires JPL ephemeris (future Phase 12 work).

---

*This implementation plan combines classical orbital mechanics primitives with analytical ephemeris for Sun and Moon positions, providing the foundation for both orbit analysis and third-body perturbation computations.*
