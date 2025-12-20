# Phase 12: Third-Body Gravity Implementation Plan

> **Purpose**: This document provides a comprehensive implementation plan for third-body gravitational perturbations (Sun, Moon, planets) in Vulcan, following the established Janus-compatible patterns.

> **Dependencies**: Phase 9 (Time Systems) must be complete - third-body effects require time-dependent ephemeris.

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

Implement third-body gravitational perturbations for high-fidelity orbit propagation:

| Component | Fidelity | Use Case | Symbolic Support |
|-----------|----------|----------|------------------|
| **Sun (Analytical)** | Medium | GEO, HEO, cislunar | Full |
| **Moon (Analytical)** | Medium | GEO, HEO, cislunar | Full |
| **Sun (JPL DE)** | Very High | Precision applications | Numeric only |
| **Moon (JPL DE)** | Very High | Precision applications | Numeric only |
| **Planets** | High | Interplanetary | Future |

### When Third-Body Effects Matter

| Orbit Type | Sun Effect | Moon Effect | Notes |
|------------|------------|-------------|-------|
| LEO (< 2000 km) | Negligible | Negligible | Earth J2 dominates |
| MEO (2000-35000 km) | Small | Small | Consider for long propagation |
| GEO (~36000 km) | Significant | Significant | Required for stationkeeping |
| HEO (Molniya) | Moderate | Moderate | Affects apogee |
| Cislunar | Dominant | Dominant | Primary perturbation |
| Interplanetary | Dominant | Small | Need planetary ephemeris |

### Third-Body Acceleration Magnitude (Order of Magnitude)

At GEO altitude (~42,000 km from Earth center):
- Earth gravity: ~0.22 m/s²
- Sun perturbation: ~5e-6 m/s²
- Moon perturbation: ~1e-5 m/s²

Small but cumulative over days/weeks of propagation.

---

## 2. Architecture

### Namespace Organization

Extending the `vulcan::gravity::` namespace:

```
vulcan::gravity::
├── earth::                      # Phase 6 (complete)
│   ├── point_mass::
│   ├── j2::
│   ├── j2j4::
│   └── spherical_harmonics::
│
└── thirdbody::                  # Phase 12 (this plan)
    ├── sun::
    │   ├── acceleration(r_sat, r_sun)           # Given sun position
    │   ├── acceleration(r_sat, jd_tt)           # Auto-compute sun position
    │   └── perturbation_ratio(r_sat, r_sun)     # Diagnostic
    ├── moon::
    │   ├── acceleration(r_sat, r_moon)
    │   ├── acceleration(r_sat, jd_tt)
    │   └── perturbation_ratio(r_sat, r_moon)
    ├── combined::
    │   ├── acceleration(r_sat, jd_tt)           # Sun + Moon
    │   └── acceleration(r_sat, jd_tt, bodies)   # Configurable
    └── ephemeris::
        ├── analytical::                         # Meeus algorithms
        │   ├── sun_position_eci(jd_tt)
        │   ├── moon_position_eci(jd_tt)
        │   ├── sun_position_ecef(jd_tt)
        │   └── moon_position_ecef(jd_tt)
        └── jpl::                                # DE440/441 (future)
            ├── load(filename)
            ├── body_position(body_id, jd_tt)
            └── body_velocity(body_id, jd_tt)
```

### File Structure

```
include/vulcan/gravity/thirdbody/
├── ThirdBodyTypes.hpp          # Constants, enums, shared types
├── ThirdBodyAcceleration.hpp   # Core perturbation equations
├── SunGravity.hpp              # Sun-specific functions
├── MoonGravity.hpp             # Moon-specific functions
├── CombinedThirdBody.hpp       # Convenience combined functions
├── AnalyticalEphemeris.hpp     # Meeus-based positions
├── JPLEphemeris.hpp            # DE440/441 reader (future)
└── ThirdBody.hpp               # Umbrella header

tests/gravity/thirdbody/
├── test_thirdbody_acceleration.cpp
├── test_sun_gravity.cpp
├── test_moon_gravity.cpp
├── test_analytical_ephemeris.cpp
└── test_combined_thirdbody.cpp

examples/gravity/
├── thirdbody_perturbation.cpp      # Effect on GEO satellite
├── cislunar_trajectory.cpp         # Moon gravity dominance
└── ephemeris_comparison.cpp        # Analytical vs reference
```

---

## 3. Implementation Details

### 3.1 Third-Body Constants (`ThirdBodyTypes.hpp`)

```cpp
// include/vulcan/gravity/thirdbody/ThirdBodyTypes.hpp
#pragma once

#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::gravity::thirdbody {

// =============================================================================
// Celestial Body Constants
// =============================================================================

namespace constants {

namespace sun {
/// Gravitational parameter [m³/s²]
inline constexpr double mu = 1.32712440018e20;

/// Mean radius [m]
inline constexpr double radius = 6.96e8;

/// Mean distance from Earth [m] (1 AU)
inline constexpr double AU = 1.495978707e11;
} // namespace sun

namespace moon {
/// Gravitational parameter [m³/s²]
inline constexpr double mu = 4.9028695e12;

/// Mean radius [m]
inline constexpr double radius = 1.7374e6;

/// Mean distance from Earth [m]
inline constexpr double mean_distance = 3.844e8;
} // namespace moon

namespace jupiter {
/// Gravitational parameter [m³/s²] (for interplanetary)
inline constexpr double mu = 1.26686534e17;
} // namespace jupiter

namespace venus {
/// Gravitational parameter [m³/s²]
inline constexpr double mu = 3.24858599e14;
} // namespace venus

namespace mars {
/// Gravitational parameter [m³/s²]
inline constexpr double mu = 4.28283100e13;
} // namespace mars

} // namespace constants

// =============================================================================
// Body Enumeration
// =============================================================================

/// Celestial bodies for third-body perturbations
enum class Body {
    Sun,
    Moon,
    Mercury,
    Venus,
    Mars,
    Jupiter,
    Saturn,
    Uranus,
    Neptune
};

/// Get gravitational parameter for a body
inline double get_mu(Body body) {
    switch (body) {
        case Body::Sun:     return constants::sun::mu;
        case Body::Moon:    return constants::moon::mu;
        case Body::Venus:   return constants::venus::mu;
        case Body::Mars:    return constants::mars::mu;
        case Body::Jupiter: return constants::jupiter::mu;
        default:            return 0.0;
    }
}

} // namespace vulcan::gravity::thirdbody
```

### 3.2 Core Third-Body Acceleration (`ThirdBodyAcceleration.hpp`)

The fundamental equation for third-body perturbation acceleration:

```
a_3b = μ_3b * [ (r_3b - r_sat)/|r_3b - r_sat|³ - r_3b/|r_3b|³ ]
```

where:
- `r_sat` = satellite position (ECI)
- `r_3b` = third body position (ECI)
- `μ_3b` = third body gravitational parameter

```cpp
// include/vulcan/gravity/thirdbody/ThirdBodyAcceleration.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/gravity/thirdbody/ThirdBodyTypes.hpp>

namespace vulcan::gravity::thirdbody {

/**
 * @brief Third-body gravitational acceleration (general form)
 *
 * Computes the perturbation acceleration on a satellite due to a
 * third body (Sun, Moon, planet). Uses the standard formulation
 * that accounts for both direct and indirect effects.
 *
 * The acceleration consists of two terms:
 * 1. Direct: Attraction of satellite toward third body
 * 2. Indirect: Acceleration of Earth toward third body (reference frame effect)
 *
 * @tparam Scalar double or casadi::MX
 * @param r_sat Satellite position in ECI [m]
 * @param r_body Third body position in ECI [m]
 * @param mu_body Gravitational parameter of third body [m³/s²]
 * @return Perturbation acceleration in ECI [m/s²]
 */
template <typename Scalar>
Vec3<Scalar> acceleration(const Vec3<Scalar>& r_sat,
                          const Vec3<Scalar>& r_body,
                          double mu_body) {
    // Vector from satellite to third body
    const Vec3<Scalar> r_rel = r_body - r_sat;

    // Distances
    const Scalar d_sat_body = janus::norm(r_rel);      // |r_body - r_sat|
    const Scalar d_body = janus::norm(r_body);          // |r_body|

    // Cubed distances
    const Scalar d_sat_body_cubed = d_sat_body * d_sat_body * d_sat_body;
    const Scalar d_body_cubed = d_body * d_body * d_body;

    // Third-body acceleration:
    // a = μ * [ (r_body - r_sat)/|r_body - r_sat|³ - r_body/|r_body|³ ]
    //       \_____________direct_______________/   \____indirect____/
    return mu_body * (r_rel / d_sat_body_cubed - r_body / d_body_cubed);
}

/**
 * @brief Third-body perturbation ratio
 *
 * Computes the ratio of third-body perturbation to central body gravity.
 * Useful for assessing when third-body effects become significant.
 *
 * Rule of thumb:
 * - Ratio < 1e-8: Negligible
 * - Ratio 1e-8 to 1e-6: Minor (long-term effects)
 * - Ratio > 1e-6: Significant (include in propagation)
 *
 * @tparam Scalar double or casadi::MX
 * @param r_sat Satellite position in ECI [m]
 * @param r_body Third body position in ECI [m]
 * @param mu_body Third body gravitational parameter [m³/s²]
 * @param mu_central Central body gravitational parameter [m³/s²]
 * @return Ratio |a_thirdbody| / |a_central|
 */
template <typename Scalar>
Scalar perturbation_ratio(const Vec3<Scalar>& r_sat,
                          const Vec3<Scalar>& r_body,
                          double mu_body,
                          double mu_central = vulcan::constants::earth::mu) {
    const Vec3<Scalar> a_3b = acceleration(r_sat, r_body, mu_body);
    const Scalar a_3b_mag = janus::norm(a_3b);

    const Scalar r_sat_mag = janus::norm(r_sat);
    const Scalar a_central_mag = mu_central / (r_sat_mag * r_sat_mag);

    return a_3b_mag / a_central_mag;
}

/**
 * @brief Battin's F(q) function for improved numerical accuracy
 *
 * For cases where satellite is far from Earth relative to the third body
 * distance, the standard formulation can suffer from numerical issues.
 * Battin's formulation provides better accuracy:
 *
 * a = -μ/|r_rel|³ * [r_sat + F(q) * r_body]
 *
 * where q = (r_sat · (r_sat - 2*r_body)) / |r_body|²
 *       F(q) = q * (3 + 3q + q²) / (1 + (1+q)^1.5)
 *
 * Reference: Battin, "An Introduction to the Mathematics and Methods
 *            of Astrodynamics" (1999), Section 8.3
 *
 * @tparam Scalar double or casadi::MX
 * @param r_sat Satellite position in ECI [m]
 * @param r_body Third body position in ECI [m]
 * @param mu_body Gravitational parameter of third body [m³/s²]
 * @return Perturbation acceleration in ECI [m/s²]
 */
template <typename Scalar>
Vec3<Scalar> acceleration_battin(const Vec3<Scalar>& r_sat,
                                  const Vec3<Scalar>& r_body,
                                  double mu_body) {
    const Vec3<Scalar> r_rel = r_body - r_sat;
    const Scalar d_rel = janus::norm(r_rel);
    const Scalar d_body_sq = janus::dot(r_body, r_body);

    // q parameter
    const Scalar q = janus::dot(r_sat, r_sat - 2.0 * r_body) / d_body_sq;

    // F(q) function - numerically stable for small q
    const Scalar q2 = q * q;
    const Scalar q3 = q2 * q;
    const Scalar numerator = q * (3.0 + 3.0 * q + q2);
    const Scalar denominator = 1.0 + janus::pow(1.0 + q, 1.5);
    const Scalar F_q = numerator / denominator;

    // Battin formulation
    const Scalar d_rel_cubed = d_rel * d_rel * d_rel;
    return -mu_body / d_rel_cubed * (r_sat + F_q * r_body);
}

} // namespace vulcan::gravity::thirdbody
```

### 3.3 Analytical Ephemeris (`AnalyticalEphemeris.hpp`)

Low-to-medium fidelity ephemeris using Meeus algorithms. No external data required.

```cpp
// include/vulcan/gravity/thirdbody/AnalyticalEphemeris.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/time/JulianDate.hpp>

namespace vulcan::gravity::thirdbody::ephemeris::analytical {

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
    const Scalar M_rad = M * constants::angle::deg2rad;

    // Equation of center (degrees)
    const Scalar C = (1.914602 - 0.004817 * T - 0.000014 * T * T) * janus::sin(M_rad) +
                     (0.019993 - 0.000101 * T) * janus::sin(2.0 * M_rad) +
                     0.000289 * janus::sin(3.0 * M_rad);

    // True longitude (degrees)
    const Scalar sun_lon = L0 + C;
    const Scalar sun_lon_rad = sun_lon * constants::angle::deg2rad;

    // True anomaly
    const Scalar v = M + C;
    const Scalar v_rad = v * constants::angle::deg2rad;

    // Distance in AU
    const Scalar R_AU = 1.000001018 * (1.0 - e * e) / (1.0 + e * janus::cos(v_rad));

    // Convert to meters
    const Scalar R = R_AU * thirdbody::constants::sun::AU;

    // Mean obliquity of the ecliptic (degrees)
    const Scalar epsilon = 23.439291 - 0.0130042 * T - 0.00000016 * T * T;
    const Scalar epsilon_rad = epsilon * constants::angle::deg2rad;

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
 * For higher accuracy, use JPL ephemeris.
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
    const Scalar D_rad = D * constants::angle::deg2rad;
    const Scalar M_rad = M * constants::angle::deg2rad;
    const Scalar Mp_rad = Mp * constants::angle::deg2rad;
    const Scalar F_rad = F * constants::angle::deg2rad;
    const Scalar Lp_rad = Lp * constants::angle::deg2rad;

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
    const Scalar lambda_rad = lambda * constants::angle::deg2rad;
    const Scalar beta_rad = beta * constants::angle::deg2rad;

    // Mean obliquity of the ecliptic
    const Scalar epsilon = 23.439291 - 0.0130042 * T;
    const Scalar epsilon_rad = epsilon * constants::angle::deg2rad;

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
 * Converts ECI sun position to ECEF using Earth rotation.
 *
 * @tparam Scalar double or casadi::MX
 * @param jd_tt Julian Date in Terrestrial Time
 * @return Sun position in ECEF [m]
 */
template <typename Scalar>
Vec3<Scalar> sun_position_ecef(const Scalar& jd_tt) {
    // Get ECI position
    Vec3<Scalar> r_eci = sun_position_eci(jd_tt);

    // Compute Greenwich Mean Sidereal Time (GMST)
    // Simplified formula - for full accuracy use IAU 2006 precession
    const Scalar T = (jd_tt - 2451545.0) / 36525.0;
    const Scalar gmst_deg = 280.46061837 + 360.98564736629 * (jd_tt - 2451545.0) +
                            0.000387933 * T * T - T * T * T / 38710000.0;
    const Scalar gmst_rad = gmst_deg * constants::angle::deg2rad;

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
 */
template <typename Scalar>
Vec3<Scalar> moon_position_ecef(const Scalar& jd_tt) {
    Vec3<Scalar> r_eci = moon_position_eci(jd_tt);

    const Scalar T = (jd_tt - 2451545.0) / 36525.0;
    const Scalar gmst_deg = 280.46061837 + 360.98564736629 * (jd_tt - 2451545.0) +
                            0.000387933 * T * T - T * T * T / 38710000.0;
    const Scalar gmst_rad = gmst_deg * constants::angle::deg2rad;

    const Scalar cos_gmst = janus::cos(gmst_rad);
    const Scalar sin_gmst = janus::sin(gmst_rad);

    Vec3<Scalar> r_ecef;
    r_ecef(0) = cos_gmst * r_eci(0) + sin_gmst * r_eci(1);
    r_ecef(1) = -sin_gmst * r_eci(0) + cos_gmst * r_eci(1);
    r_ecef(2) = r_eci(2);

    return r_ecef;
}

} // namespace vulcan::gravity::thirdbody::ephemeris::analytical
```

### 3.4 Sun Gravity (`SunGravity.hpp`)

```cpp
// include/vulcan/gravity/thirdbody/SunGravity.hpp
#pragma once

#include <vulcan/gravity/thirdbody/ThirdBodyAcceleration.hpp>
#include <vulcan/gravity/thirdbody/AnalyticalEphemeris.hpp>

namespace vulcan::gravity::thirdbody::sun {

/**
 * @brief Solar gravitational acceleration given sun position
 *
 * @tparam Scalar double or casadi::MX
 * @param r_sat Satellite position in ECI [m]
 * @param r_sun Sun position in ECI [m]
 * @return Perturbation acceleration in ECI [m/s²]
 */
template <typename Scalar>
Vec3<Scalar> acceleration(const Vec3<Scalar>& r_sat,
                          const Vec3<Scalar>& r_sun) {
    return thirdbody::acceleration(r_sat, r_sun, constants::sun::mu);
}

/**
 * @brief Solar gravitational acceleration at given time
 *
 * Automatically computes sun position using analytical ephemeris.
 *
 * @tparam Scalar double or casadi::MX
 * @param r_sat Satellite position in ECI [m]
 * @param jd_tt Julian Date in Terrestrial Time
 * @return Perturbation acceleration in ECI [m/s²]
 */
template <typename Scalar>
Vec3<Scalar> acceleration(const Vec3<Scalar>& r_sat,
                          const Scalar& jd_tt) {
    const Vec3<Scalar> r_sun = ephemeris::analytical::sun_position_eci(jd_tt);
    return acceleration(r_sat, r_sun);
}

/**
 * @brief Solar perturbation ratio (diagnostic)
 */
template <typename Scalar>
Scalar perturbation_ratio(const Vec3<Scalar>& r_sat,
                          const Vec3<Scalar>& r_sun) {
    return thirdbody::perturbation_ratio(r_sat, r_sun, constants::sun::mu);
}

} // namespace vulcan::gravity::thirdbody::sun
```

### 3.5 Moon Gravity (`MoonGravity.hpp`)

```cpp
// include/vulcan/gravity/thirdbody/MoonGravity.hpp
#pragma once

#include <vulcan/gravity/thirdbody/ThirdBodyAcceleration.hpp>
#include <vulcan/gravity/thirdbody/AnalyticalEphemeris.hpp>

namespace vulcan::gravity::thirdbody::moon {

/**
 * @brief Lunar gravitational acceleration given moon position
 *
 * @tparam Scalar double or casadi::MX
 * @param r_sat Satellite position in ECI [m]
 * @param r_moon Moon position in ECI [m]
 * @return Perturbation acceleration in ECI [m/s²]
 */
template <typename Scalar>
Vec3<Scalar> acceleration(const Vec3<Scalar>& r_sat,
                          const Vec3<Scalar>& r_moon) {
    return thirdbody::acceleration(r_sat, r_moon, constants::moon::mu);
}

/**
 * @brief Lunar gravitational acceleration at given time
 *
 * @tparam Scalar double or casadi::MX
 * @param r_sat Satellite position in ECI [m]
 * @param jd_tt Julian Date in Terrestrial Time
 * @return Perturbation acceleration in ECI [m/s²]
 */
template <typename Scalar>
Vec3<Scalar> acceleration(const Vec3<Scalar>& r_sat,
                          const Scalar& jd_tt) {
    const Vec3<Scalar> r_moon = ephemeris::analytical::moon_position_eci(jd_tt);
    return acceleration(r_sat, r_moon);
}

/**
 * @brief Lunar perturbation ratio (diagnostic)
 */
template <typename Scalar>
Scalar perturbation_ratio(const Vec3<Scalar>& r_sat,
                          const Vec3<Scalar>& r_moon) {
    return thirdbody::perturbation_ratio(r_sat, r_moon, constants::moon::mu);
}

} // namespace vulcan::gravity::thirdbody::moon
```

### 3.6 Combined Third-Body (`CombinedThirdBody.hpp`)

```cpp
// include/vulcan/gravity/thirdbody/CombinedThirdBody.hpp
#pragma once

#include <vulcan/gravity/thirdbody/SunGravity.hpp>
#include <vulcan/gravity/thirdbody/MoonGravity.hpp>

namespace vulcan::gravity::thirdbody::combined {

/**
 * @brief Combined Sun + Moon gravitational acceleration
 *
 * @tparam Scalar double or casadi::MX
 * @param r_sat Satellite position in ECI [m]
 * @param jd_tt Julian Date in Terrestrial Time
 * @param include_sun Include solar perturbation (default: true)
 * @param include_moon Include lunar perturbation (default: true)
 * @return Total third-body perturbation acceleration in ECI [m/s²]
 */
template <typename Scalar>
Vec3<Scalar> acceleration(const Vec3<Scalar>& r_sat,
                          const Scalar& jd_tt,
                          bool include_sun = true,
                          bool include_moon = true) {
    Vec3<Scalar> a_total = Vec3<Scalar>::Zero();

    if (include_sun) {
        a_total += sun::acceleration(r_sat, jd_tt);
    }

    if (include_moon) {
        a_total += moon::acceleration(r_sat, jd_tt);
    }

    return a_total;
}

/**
 * @brief Combined acceleration with pre-computed ephemeris
 *
 * More efficient when propagating - compute positions once per step.
 */
template <typename Scalar>
Vec3<Scalar> acceleration(const Vec3<Scalar>& r_sat,
                          const Vec3<Scalar>& r_sun,
                          const Vec3<Scalar>& r_moon) {
    return sun::acceleration(r_sat, r_sun) + moon::acceleration(r_sat, r_moon);
}

} // namespace vulcan::gravity::thirdbody::combined
```

### 3.7 Umbrella Header (`ThirdBody.hpp`)

```cpp
// include/vulcan/gravity/thirdbody/ThirdBody.hpp
#pragma once

#include <vulcan/gravity/thirdbody/ThirdBodyTypes.hpp>
#include <vulcan/gravity/thirdbody/ThirdBodyAcceleration.hpp>
#include <vulcan/gravity/thirdbody/AnalyticalEphemeris.hpp>
#include <vulcan/gravity/thirdbody/SunGravity.hpp>
#include <vulcan/gravity/thirdbody/MoonGravity.hpp>
#include <vulcan/gravity/thirdbody/CombinedThirdBody.hpp>
```

---

## 4. Testing Strategy

### 4.1 Test Categories

| Category | Purpose | Data Source |
|----------|---------|-------------|
| **Ephemeris Accuracy** | Verify Sun/Moon positions | JPL Horizons |
| **Acceleration Magnitude** | Order-of-magnitude checks | Analytical estimates |
| **Consistency** | Direct vs time-based APIs | Internal cross-check |
| **Symbolic Compatibility** | Verify autodiff works | Finite difference |
| **Edge Cases** | Eclipse, lunar proximity | Manual analysis |

### 4.2 Reference Data

Sun and Moon positions can be validated against JPL Horizons:
https://ssd.jpl.nasa.gov/horizons/

```cpp
// tests/gravity/thirdbody/test_analytical_ephemeris.cpp
#include <gtest/gtest.h>
#include <vulcan/gravity/thirdbody/AnalyticalEphemeris.hpp>

using namespace vulcan::gravity::thirdbody::ephemeris::analytical;

TEST(AnalyticalEphemeris, SunPosition_J2000) {
    // J2000.0 epoch: 2000-01-01 12:00:00 TT
    double jd_j2000 = 2451545.0;

    Vec3<double> r_sun = sun_position_eci(jd_j2000);
    double dist = janus::norm(r_sun);

    // Sun should be ~1 AU away
    double AU = vulcan::gravity::thirdbody::constants::sun::AU;
    EXPECT_NEAR(dist / AU, 1.0, 0.02);  // Within 2%

    // At J2000, Sun is roughly in -X direction (winter solstice was Dec 21)
    EXPECT_LT(r_sun(0), 0.0);
}

TEST(AnalyticalEphemeris, MoonPosition_Approximate) {
    double jd = 2451545.0;

    Vec3<double> r_moon = moon_position_eci(jd);
    double dist = janus::norm(r_moon);

    // Moon should be ~384,000 km away
    double expected_dist = vulcan::gravity::thirdbody::constants::moon::mean_distance;
    EXPECT_NEAR(dist, expected_dist, expected_dist * 0.1);  // Within 10%
}

TEST(AnalyticalEphemeris, SymbolicEvaluation) {
    auto jd = janus::sym("jd");

    auto r_sun = sun_position_eci(jd);

    // Should be able to evaluate
    double jd_val = 2451545.0;
    double x = janus::eval(r_sun(0), {{"jd", jd_val}});

    EXPECT_NE(x, 0.0);  // Non-zero position
}
```

### 4.3 Acceleration Tests

```cpp
// tests/gravity/thirdbody/test_thirdbody_acceleration.cpp
#include <gtest/gtest.h>
#include <vulcan/gravity/thirdbody/ThirdBody.hpp>

using namespace vulcan::gravity::thirdbody;

TEST(ThirdBodyAcceleration, SunAtGEO) {
    // GEO satellite position (equatorial, ~42,000 km)
    Vec3<double> r_sat;
    r_sat << 42164e3, 0.0, 0.0;

    // Sun at 1 AU in +X direction
    Vec3<double> r_sun;
    r_sun << constants::sun::AU, 0.0, 0.0;

    Vec3<double> a = sun::acceleration(r_sat, r_sun);
    double a_mag = janus::norm(a);

    // Expected: ~5e-6 m/s² at GEO
    EXPECT_GT(a_mag, 1e-7);
    EXPECT_LT(a_mag, 1e-4);
}

TEST(ThirdBodyAcceleration, MoonAtGEO) {
    Vec3<double> r_sat;
    r_sat << 42164e3, 0.0, 0.0;

    // Moon at mean distance in +X direction
    Vec3<double> r_moon;
    r_moon << constants::moon::mean_distance, 0.0, 0.0;

    Vec3<double> a = moon::acceleration(r_sat, r_moon);
    double a_mag = janus::norm(a);

    // Expected: ~1e-5 m/s² at GEO (Moon effect larger than Sun at GEO)
    EXPECT_GT(a_mag, 1e-7);
    EXPECT_LT(a_mag, 1e-4);
}

TEST(ThirdBodyAcceleration, NegligibleAtLEO) {
    // LEO satellite (400 km altitude)
    Vec3<double> r_sat;
    r_sat << vulcan::constants::earth::R_eq + 400e3, 0.0, 0.0;

    Vec3<double> r_sun;
    r_sun << constants::sun::AU, 0.0, 0.0;

    double ratio = sun::perturbation_ratio(r_sat, r_sun);

    // At LEO, third-body effects are << Earth gravity
    EXPECT_LT(ratio, 1e-6);
}

TEST(ThirdBodyAcceleration, SymbolicGradient) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");
    auto z = janus::sym("z");

    Vec3<janus::SymbolicScalar> r_sat;
    r_sat << x, y, z;

    Vec3<double> r_sun;
    r_sun << constants::sun::AU, 0.0, 0.0;

    auto a = sun::acceleration(r_sat, r_sun);

    // Compute Jacobian
    auto da_dx = janus::jacobian(a(0), x);

    // Evaluate at GEO
    double r_geo = 42164e3;
    double grad = janus::eval(da_dx, {{"x", r_geo}, {"y", 0.0}, {"z", 0.0}});

    // Should have non-zero gradient
    EXPECT_NE(grad, 0.0);
}
```

---

## 5. Verification & Validation

### 5.1 Reference Sources

| Source | Data | Notes |
|--------|------|-------|
| **JPL Horizons** | Ephemeris | Authoritative positions |
| **Montenbruck & Gill** | Algorithms | "Satellite Orbits" textbook |
| **Meeus** | Analytical formulas | "Astronomical Algorithms" |
| **STK/GMAT** | Cross-validation | Commercial/open tools |

### 5.2 Validation Cases

```
Test Case 1: GEO Stationkeeping
- Satellite at GEO (λ = 0°)
- Propagate for 1 month
- Compare drift with/without third-body

Test Case 2: Molniya Orbit
- Highly elliptical, 12-hour period
- Significant third-body effects at apogee
- Compare against GMAT propagation

Test Case 3: Lunar Transfer
- Trans-lunar injection
- Moon gravity becomes dominant
- Validate against published trajectories

Test Case 4: Solar Eclipse Geometry
- Verify Sun-Earth-Moon alignment
- Ephemeris consistency check
```

---

## 6. Implementation Phases

### Phase 12.1: Core Infrastructure (2-3 hours)
- [ ] Create `include/vulcan/gravity/thirdbody/` directory
- [ ] Implement `ThirdBodyTypes.hpp` (constants, enums)
- [ ] Implement `ThirdBodyAcceleration.hpp` (core equations)
- [ ] Basic unit tests for acceleration formula

### Phase 12.2: Analytical Ephemeris (3-4 hours)
- [ ] Implement `AnalyticalEphemeris.hpp`
- [ ] Sun position (Meeus algorithm)
- [ ] Moon position (Meeus algorithm)
- [ ] ECEF conversions
- [ ] Validate against JPL Horizons reference data

### Phase 12.3: Body-Specific Modules (2-3 hours)
- [ ] Implement `SunGravity.hpp`
- [ ] Implement `MoonGravity.hpp`
- [ ] Implement `CombinedThirdBody.hpp`
- [ ] Create tests for each module

### Phase 12.4: Symbolic Compatibility (1-2 hours)
- [ ] Verify all functions work with `casadi::MX`
- [ ] Test gradient computation
- [ ] Integration with `janus::Opti` for trajectory optimization

### Phase 12.5: Documentation & Examples (2-3 hours)
- [ ] Create `examples/gravity/thirdbody_perturbation.cpp`
- [ ] Create `examples/gravity/cislunar_trajectory.cpp`
- [ ] Update `docs/user_guides/gravity_models.md`
- [ ] Add to umbrella headers

### Phase 12.6: JPL Ephemeris (Future - 4-6 hours)
- [ ] Implement DE440/441 binary file reader
- [ ] Chebyshev interpolation for positions
- [ ] Velocity computation
- [ ] Caching for performance

---

## Appendix A: Third-Body Perturbation Theory

### Acceleration Derivation

For a satellite at position `r_sat` relative to Earth, and a third body at position `r_3b` relative to Earth, the perturbation acceleration is:

```
a_3b = -μ_3b * [ (r_sat - r_3b)/|r_sat - r_3b|³ + r_3b/|r_3b|³ ]
```

Rearranging with `r_rel = r_3b - r_sat`:

```
a_3b = μ_3b * [ r_rel/|r_rel|³ - r_3b/|r_3b|³ ]
        ↑              ↑
    "direct"      "indirect"
```

- **Direct term**: Attraction of satellite toward third body
- **Indirect term**: Accounts for Earth's acceleration toward third body (non-inertial frame effect)

### Order of Magnitude Estimates

At distance `d` from Earth, third-body acceleration scales as:

```
a_3b ≈ 2 * μ_3b * d / R³
```

where `R` is the distance from Earth to the third body.

For Sun at GEO:
```
a_sun ≈ 2 * 1.3e20 * 42e6 / (1.5e11)³ ≈ 3e-6 m/s²
```

For Moon at GEO:
```
a_moon ≈ 2 * 4.9e12 * 42e6 / (3.8e8)³ ≈ 7e-6 m/s²
```

---

## Appendix B: Ephemeris Accuracy Comparison

| Method | Sun Accuracy | Moon Accuracy | Data Required |
|--------|--------------|---------------|---------------|
| Analytical (Meeus) | ~0.01° | ~0.3° | None |
| JPL DE440 | <0.001" | <0.001" | Binary files |
| SPICE | <0.001" | <0.001" | Kernel files |

For most trajectory optimization applications, analytical ephemeris is sufficient. High-precision applications (navigation, conjunction analysis) require JPL ephemeris.

---

This implementation plan provides all context needed to implement third-body gravitational perturbations in Vulcan while maintaining Janus compatibility and clean separation from Earth gravity models.
