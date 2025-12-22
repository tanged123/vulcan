# Phase 11: Environmental Utilities Implementation Plan

> **Purpose**: Implement environmental utilities for space applications, including solar position (approximate ephemeris), eclipse detection, and magnetic field (dipole model). All implementations follow Vulcan's Janus-compatible templated patterns.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Architecture](#2-architecture)
3. [Implementation Details](#3-implementation-details)
4. [Testing Strategy](#4-testing-strategy)
5. [Verification Plan](#5-verification-plan)

---

## 1. Overview

### Goals

| Component | Purpose | Symbolic Support |
|-----------|---------|------------------|
| **Solar Position** | Approximate Sun ephemeris (position in ECI) | Full |
| **Eclipse Detection** | Cylindrical/conical shadow models | Full |
| **Magnetic Field** | Tilted dipole model for Earth | Full |

### Dependencies

- `vulcan/time/JulianDate.hpp` - Julian centuries since J2000
- `vulcan/core/Constants.hpp` - Physical constants
- `vulcan/core/VulcanTypes.hpp` - Vec3 type aliases
- `janus/janus.hpp` - Math dispatch and symbolic support

---

## 2. Architecture

### Namespace Organization

```
vulcan::environment::
├── solar::
│   ├── position_eci(jd)              // Sun position in ECI [m]
│   ├── unit_vector_eci(jd)           // Sun direction unit vector
│   ├── right_ascension(jd)           // Right ascension [rad]
│   ├── declination(jd)               // Declination [rad]
│   └── distance(jd)                  // Earth-Sun distance [m]
├── eclipse::
│   ├── shadow_function(r_sat, r_sun) // Shadow function (0=umbra, 1=sunlit)
│   ├── is_in_shadow(r_sat, r_sun)    // Boolean shadow check
│   └── illumination_factor(...)      // Penumbra transition (0-1)
└── magnetic::
    ├── field_eci(r_eci, jd)          // B-field vector in ECI [T]
    ├── field_ecef(r_ecef)            // B-field vector in ECEF [T]
    └── field_ned(lat, lon, alt)      // B-field in NED [T]
```

### File Structure

```
include/vulcan/environment/
├── SolarPosition.hpp       # Approximate solar ephemeris
├── Eclipse.hpp             # Shadow/eclipse detection  
├── MagneticField.hpp       # Dipole magnetic field model
├── EnvironmentTypes.hpp    # Shared types (if needed)
└── Environment.hpp         # Umbrella header

tests/environment/
├── test_solar_position.cpp
├── test_eclipse.cpp
└── test_magnetic_field.cpp

examples/environment/
└── environment_demo.cpp    # Combined demonstration
```

---

## 3. Implementation Details

### 3.1 Solar Position (`SolarPosition.hpp`)

Low-precision solar ephemeris using the algorithm from Vallado's "Fundamentals of Astrodynamics" (accurate to ~0.01° over several decades).

**Key equations:**
- Mean longitude: `λ_sun = 280.460° + 36000.771·T` (mod 360°)
- Mean anomaly: `M_sun = 357.5291° + 35999.05034·T`
- Ecliptic longitude: `λ_ec = λ_sun + 1.9148·sin(M) + 0.02·sin(2M)`
- Obliquity: `ε = 23.439° - 0.0130·T`
- Distance: `r = 1.00014 - 0.01671·cos(M) - 0.00014·cos(2M)` [AU]

```cpp
// include/vulcan/environment/SolarPosition.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/time/JulianDate.hpp>

namespace vulcan::environment::solar {

namespace detail {
    // Astronomical Unit in meters
    inline constexpr double AU = 149597870700.0;
    
    // Wrap angle to [0, 2π)
    template <typename Scalar>
    Scalar wrap_to_2pi(const Scalar& angle) {
        constexpr double two_pi = 2.0 * constants::angle::pi;
        // Use fmod-like behavior for symbolic compatibility
        return angle - two_pi * janus::floor(angle / two_pi);
    }
}

/**
 * @brief Compute Sun's right ascension and declination
 * 
 * Low-precision ephemeris from Vallado (accurate to ~0.01°)
 * 
 * @tparam Scalar double or casadi::MX
 * @param jd Julian Date (TDB or TT)
 * @return {right_ascension [rad], declination [rad]}
 */
template <typename Scalar>
std::pair<Scalar, Scalar> ra_dec(const Scalar& jd) {
    using janus::sin;
    using janus::cos;
    using janus::atan2;
    
    // Julian centuries since J2000.0
    const Scalar T = time::jd_to_j2000_centuries(jd);
    
    // Mean longitude of the Sun [deg -> rad]
    const Scalar lambda_M = (280.460 + 36000.771 * T) * constants::angle::deg2rad;
    
    // Mean anomaly [deg -> rad]
    const Scalar M = (357.5291092 + 35999.05034 * T) * constants::angle::deg2rad;
    
    // Ecliptic longitude [rad]
    const Scalar lambda_ec = lambda_M + 
        (1.914666471 * sin(M) + 0.019994643 * sin(2.0 * M)) * constants::angle::deg2rad;
    
    // Obliquity of the ecliptic [rad]
    const Scalar epsilon = (23.439291 - 0.0130042 * T) * constants::angle::deg2rad;
    
    // Right ascension and declination
    const Scalar sin_lambda = sin(lambda_ec);
    const Scalar cos_lambda = cos(lambda_ec);
    const Scalar cos_eps = cos(epsilon);
    const Scalar sin_eps = sin(epsilon);
    
    const Scalar ra = atan2(cos_eps * sin_lambda, cos_lambda);
    const Scalar dec = janus::asin(sin_eps * sin_lambda);
    
    return {ra, dec};
}

/**
 * @brief Earth-Sun distance
 * @param jd Julian Date
 * @return Distance [m]
 */
template <typename Scalar>
Scalar distance(const Scalar& jd) {
    const Scalar T = time::jd_to_j2000_centuries(jd);
    const Scalar M = (357.5291092 + 35999.05034 * T) * constants::angle::deg2rad;
    
    // Distance in AU
    const Scalar r_au = 1.000140612 - 0.016708617 * janus::cos(M) 
                        - 0.000139589 * janus::cos(2.0 * M);
    
    return r_au * detail::AU;
}

/**
 * @brief Sun position in ECI frame (J2000 equatorial)
 * @param jd Julian Date
 * @return Position vector [m]
 */
template <typename Scalar>
Vec3<Scalar> position_eci(const Scalar& jd) {
    auto [ra, dec] = ra_dec(jd);
    const Scalar r = distance(jd);
    
    const Scalar cos_dec = janus::cos(dec);
    
    Vec3<Scalar> pos;
    pos(0) = r * cos_dec * janus::cos(ra);
    pos(1) = r * cos_dec * janus::sin(ra);
    pos(2) = r * janus::sin(dec);
    
    return pos;
}

/**
 * @brief Sun unit direction vector in ECI
 */
template <typename Scalar>
Vec3<Scalar> unit_vector_eci(const Scalar& jd) {
    auto [ra, dec] = ra_dec(jd);
    const Scalar cos_dec = janus::cos(dec);
    
    Vec3<Scalar> u;
    u(0) = cos_dec * janus::cos(ra);
    u(1) = cos_dec * janus::sin(ra);
    u(2) = janus::sin(dec);
    
    return u;
}

} // namespace vulcan::environment::solar
```

---

### 3.2 Eclipse Detection (`Eclipse.hpp`)

Implements cylindrical and conical shadow models for eclipse detection.

**Cylindrical Model** (simplest): Satellite is in shadow if:
- Behind Earth relative to Sun
- Within Earth's shadow cylinder

**Conical Model** (more accurate): Accounts for umbra/penumbra geometry using Earth and Sun radii.

```cpp
// include/vulcan/environment/Eclipse.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::environment::eclipse {

namespace detail {
    // Solar radius [m]
    inline constexpr double R_sun = 696340000.0;
    
    // Mean Earth-Sun distance [m] (1 AU)
    inline constexpr double AU = 149597870700.0;
}

/**
 * @brief Cylindrical shadow function
 * 
 * Returns 0 when in complete shadow (umbra), 1 when fully sunlit.
 * Uses cylindrical approximation (Earth's shadow is a cylinder).
 * 
 * @tparam Scalar double or casadi::MX
 * @param r_sat Satellite position in ECI [m]
 * @param r_sun Sun position in ECI [m]
 * @param R_body Radius of shadowing body [m] (default: Earth)
 * @return Shadow function ν ∈ [0,1]
 */
template <typename Scalar>
Scalar shadow_cylindrical(const Vec3<Scalar>& r_sat,
                          const Vec3<Scalar>& r_sun,
                          double R_body = constants::earth::R_eq) {
    // Sun direction (unit vector from Earth to Sun)
    const Scalar r_sun_mag = janus::norm(r_sun);
    const Vec3<Scalar> s_hat = r_sun / r_sun_mag;
    
    // Projection of satellite onto sun direction
    const Scalar proj = r_sat.dot(s_hat);
    
    // Satellite is on sunlit side if projection is positive
    // (Sun is behind satellite relative to Earth)
    const Scalar on_sunlit_side = janus::where(proj > 0.0, Scalar(1.0), Scalar(0.0));
    
    // Perpendicular distance from shadow axis
    const Vec3<Scalar> r_perp = r_sat - proj * s_hat;
    const Scalar d_perp = janus::norm(r_perp);
    
    // In shadow if behind Earth AND within shadow cylinder
    const Scalar in_cylinder = janus::where(d_perp < R_body, Scalar(1.0), Scalar(0.0));
    const Scalar behind_earth = janus::where(proj < 0.0, Scalar(1.0), Scalar(0.0));
    
    // Shadow = 0 (in shadow) or 1 (sunlit)
    const Scalar in_shadow = behind_earth * in_cylinder;
    
    return 1.0 - in_shadow;
}

/**
 * @brief Conical shadow function with penumbra
 * 
 * More accurate model accounting for the finite size of the Sun.
 * Returns smooth transition through penumbra.
 * 
 * Based on Montenbruck & Gill "Satellite Orbits" Section 3.4
 * 
 * @param r_sat Satellite position [m]
 * @param r_sun Sun position [m]
 * @param R_body Shadowing body radius [m]
 * @param R_sun Sun radius [m]
 * @return Illumination factor ν ∈ [0,1]
 */
template <typename Scalar>
Scalar shadow_conical(const Vec3<Scalar>& r_sat,
                      const Vec3<Scalar>& r_sun,
                      double R_body = constants::earth::R_eq,
                      double R_sun = detail::R_sun) {
    // Vector from satellite to Sun
    const Vec3<Scalar> r_sat_to_sun = r_sun - r_sat;
    const Scalar s = janus::norm(r_sat_to_sun);
    
    // Apparent angular radii as seen from satellite
    const Scalar r_sat_mag = janus::norm(r_sat);
    const Scalar theta_body = janus::asin(R_body / r_sat_mag);  // Earth angular radius
    const Scalar theta_sun = janus::asin(R_sun / s);            // Sun angular radius
    
    // Angle between Earth center and Sun as seen from satellite
    // cos(θ) = -r_sat · r_sat_to_sun / (|r_sat| |r_sat_to_sun|)
    const Scalar cos_theta = -r_sat.dot(r_sat_to_sun) / (r_sat_mag * s);
    const Scalar theta = janus::acos(cos_theta);
    
    // Shadow geometry:
    // - Full sunlight: θ > θ_body + θ_sun
    // - Penumbra: |θ_body - θ_sun| < θ < θ_body + θ_sun
    // - Umbra (full eclipse): θ < |θ_body - θ_sun| AND θ_body > θ_sun
    
    const Scalar sum_angles = theta_body + theta_sun;
    const Scalar diff_angles = janus::abs(theta_body - theta_sun);
    
    // Fully sunlit
    const Scalar full_sun = janus::where(theta >= sum_angles, Scalar(1.0), Scalar(0.0));
    
    // Full umbra
    const Scalar full_umbra = janus::where(theta <= diff_angles, Scalar(0.0), Scalar(1.0));
    
    // Penumbra: linear interpolation (simple model)
    const Scalar penumbra_frac = (theta - diff_angles) / (sum_angles - diff_angles);
    const Scalar in_penumbra = janus::where(
        (theta > diff_angles) * (theta < sum_angles),
        penumbra_frac,
        Scalar(0.0)
    );
    
    // Combine
    return full_sun + (1.0 - full_sun) * (full_umbra * in_penumbra);
}

/**
 * @brief Simple binary eclipse check
 * @return true if satellite is in Earth's shadow
 */
template <typename Scalar>
Scalar is_in_shadow(const Vec3<Scalar>& r_sat,
                    const Vec3<Scalar>& r_sun,
                    double R_body = constants::earth::R_eq) {
    const Scalar nu = shadow_cylindrical(r_sat, r_sun, R_body);
    return janus::where(nu < 0.5, Scalar(1.0), Scalar(0.0));
}

} // namespace vulcan::environment::eclipse
```

---

### 3.3 Magnetic Field (`MagneticField.hpp`)

Implements Earth's magnetic field using a tilted dipole model. This is a first-order approximation suitable for many applications.

**Dipole Model:**
- Earth's magnetic dipole moment: ~7.94 × 10²² A·m²
- Dipole axis tilted ~11.5° from rotation axis
- Field strength at surface: ~25-65 μT

```cpp
// include/vulcan/environment/MagneticField.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::environment::magnetic {

namespace constants {
    /// Earth's magnetic dipole moment [T·m³]
    /// B0 * R_eq³ where B0 ≈ 3.12e-5 T at equator surface
    inline constexpr double dipole_moment = 7.94e22 * 4.0e-7 * constants::angle::pi;
    // Simplified: μ₀/(4π) * m ≈ 7.94e15 T·m³
    
    /// Reference magnetic field at equator surface [T]
    inline constexpr double B0 = 3.12e-5;  // ~31.2 μT
    
    /// Magnetic dipole tilt angle from rotation axis [rad]
    inline constexpr double dipole_tilt = 11.5 * constants::angle::deg2rad;
    
    /// Magnetic pole longitude (West is negative) [rad]
    inline constexpr double pole_longitude = -72.6 * constants::angle::deg2rad;
}

/**
 * @brief Centered dipole magnetic field in ECEF
 * 
 * Simplified model assuming dipole is aligned with rotation axis.
 * For applications where geomagnetic field orientation matters,
 * use the tilted_dipole variant.
 * 
 * B = (B0 * R³/r³) * [3(m̂·r̂)r̂ - m̂]
 * 
 * where m̂ is the dipole direction (aligned with Z for centered dipole)
 * 
 * @tparam Scalar double or casadi::MX
 * @param r_ecef Position in ECEF [m]
 * @param B0 Reference field at equator surface [T]
 * @param R Reference radius [m]
 * @return Magnetic field vector in ECEF [T]
 */
template <typename Scalar>
Vec3<Scalar> dipole_field_ecef(const Vec3<Scalar>& r_ecef,
                                double B0 = constants::B0,
                                double R = vulcan::constants::earth::R_eq) {
    const Scalar x = r_ecef(0);
    const Scalar y = r_ecef(1);
    const Scalar z = r_ecef(2);
    
    const Scalar r2 = x*x + y*y + z*z;
    const Scalar r = janus::sqrt(r2);
    const Scalar r5 = r2 * r2 * r;
    
    // Dipole field coefficient: B0 * R³ / r⁵
    const Scalar coeff = B0 * (R * R * R) / r5;
    
    // For centered dipole aligned with Z-axis:
    // B = coeff * [3z·r - r²·ẑ]
    // B_x = 3 * coeff * x * z
    // B_y = 3 * coeff * y * z  
    // B_z = coeff * (3z² - r²) = coeff * (2z² - x² - y²)
    
    Vec3<Scalar> B;
    B(0) = 3.0 * coeff * x * z;
    B(1) = 3.0 * coeff * y * z;
    B(2) = coeff * (2.0 * z * z - x * x - y * y);
    
    return B;
}

/**
 * @brief Field magnitude at given position
 * @return |B| in Tesla
 */
template <typename Scalar>
Scalar field_magnitude(const Vec3<Scalar>& r_ecef,
                       double B0 = constants::B0,
                       double R = vulcan::constants::earth::R_eq) {
    const Vec3<Scalar> B = dipole_field_ecef(r_ecef, B0, R);
    return janus::norm(B);
}

/**
 * @brief Magnetic field at given geodetic coordinates
 * 
 * Returns field in local NED (North-East-Down) frame.
 * Uses centered dipole model.
 * 
 * @param lat Geodetic latitude [rad]
 * @param lon Geodetic longitude [rad]  
 * @param alt Altitude above ellipsoid [m]
 * @return {B_north, B_east, B_down} in Tesla
 */
template <typename Scalar>
Vec3<Scalar> field_ned(const Scalar& lat, const Scalar& lon, const Scalar& alt,
                       double B0 = constants::B0,
                       double R = vulcan::constants::earth::R_eq) {
    // Spherical approximation for simplicity
    const Scalar r = R + alt;
    
    // Geocentric latitude (spherical approx)
    const Scalar phi = lat;
    
    // Radial and theta components of dipole field
    // B_r = -2 * B0 * (R/r)³ * sin(φ)
    // B_θ = B0 * (R/r)³ * cos(φ)
    const Scalar R_over_r_cubed = (R * R * R) / (r * r * r);
    
    const Scalar sin_phi = janus::sin(phi);
    const Scalar cos_phi = janus::cos(phi);
    
    const Scalar B_r = -2.0 * B0 * R_over_r_cubed * sin_phi;
    const Scalar B_theta = B0 * R_over_r_cubed * cos_phi;
    
    // Convert to NED:
    // North = -B_θ (θ increases southward in spherical coords)
    // East = 0 (axial symmetry for centered dipole)
    // Down = -B_r (down is negative radial)
    Vec3<Scalar> B_ned;
    B_ned(0) = -B_theta;  // North
    B_ned(1) = Scalar(0.0);  // East
    B_ned(2) = -B_r;      // Down
    
    return B_ned;
}

/**
 * @brief Total field intensity at surface for given latitude
 * 
 * |B| = B0 * sqrt(1 + 3sin²(lat))
 */
template <typename Scalar>
Scalar surface_intensity(const Scalar& lat, double B0 = constants::B0) {
    const Scalar sin_lat = janus::sin(lat);
    return B0 * janus::sqrt(1.0 + 3.0 * sin_lat * sin_lat);
}

} // namespace vulcan::environment::magnetic
```

---

### 3.4 Umbrella Header (`Environment.hpp`)

```cpp
// include/vulcan/environment/Environment.hpp
#pragma once

#include <vulcan/environment/SolarPosition.hpp>
#include <vulcan/environment/Eclipse.hpp>
#include <vulcan/environment/MagneticField.hpp>
```

---

## 4. Testing Strategy

### 4.1 Test Categories

| Category | Purpose | Validation Source |
|----------|---------|-------------------|
| Reference Values | Compare against known ephemeris | JPL Horizons / Vallado |
| Consistency | Cross-check between functions | Internal |
| Symbolic | Verify autodiff works | Finite difference |
| Edge Cases | Equinox, solstice, poles | Manual analysis |

### 4.2 Solar Position Tests

```cpp
// tests/environment/test_solar_position.cpp
#include <gtest/gtest.h>
#include <vulcan/environment/SolarPosition.hpp>
#include <vulcan/time/JulianDate.hpp>
#include <janus/janus.hpp>

using namespace vulcan::environment;

// Reference: J2000.0 epoch - Sun RA/Dec from JPL Horizons
TEST(SolarPosition, J2000Reference) {
    // J2000.0 = 2000-01-01 12:00:00 TT
    double jd = vulcan::time::calendar_to_jd(2000, 1, 1, 12, 0, 0.0);
    
    auto [ra, dec] = solar::ra_dec(jd);
    
    // Expected: RA ≈ 281.3° (18h 45m), Dec ≈ -23.0° (winter solstice)
    EXPECT_NEAR(ra * vulcan::constants::angle::rad2deg, 281.3, 0.5);
    EXPECT_NEAR(dec * vulcan::constants::angle::rad2deg, -23.0, 0.5);
}

// Vernal equinox: RA = 0, Dec = 0
TEST(SolarPosition, VernalEquinox2024) {
    // March 20, 2024 ~03:06 UTC
    double jd = vulcan::time::calendar_to_jd(2024, 3, 20, 3, 6, 0.0);
    
    auto [ra, dec] = solar::ra_dec(jd);
    
    // Near equinox, declination should be near zero
    EXPECT_NEAR(dec * vulcan::constants::angle::rad2deg, 0.0, 1.0);
}

// Distance check
TEST(SolarPosition, DistanceRange) {
    // Test over a year
    for (int month = 1; month <= 12; month++) {
        double jd = vulcan::time::calendar_to_jd(2024, month, 15, 12, 0, 0.0);
        double dist = solar::distance(jd);
        
        // Distance should be ~1 AU ± 2%
        double AU = 149597870700.0;
        EXPECT_GT(dist, 0.98 * AU);
        EXPECT_LT(dist, 1.02 * AU);
    }
}

// Symbolic evaluation
TEST(SolarPosition, SymbolicMode) {
    auto jd = janus::sym("jd");
    auto pos = solar::position_eci(jd);
    
    // Evaluate at J2000
    double jd_val = vulcan::time::calendar_to_jd(2000, 1, 1, 12, 0, 0.0);
    double x = janus::eval(pos(0), {{"jd", jd_val}});
    double y = janus::eval(pos(1), {{"jd", jd_val}});
    double z = janus::eval(pos(2), {{"jd", jd_val}});
    
    double r = std::sqrt(x*x + y*y + z*z);
    EXPECT_NEAR(r, 147.1e9, 1e9);  // ~147 million km in early January
}
```

### 4.3 Eclipse Tests

```cpp
// tests/environment/test_eclipse.cpp
TEST(Eclipse, SunlitSatellite) {
    // Satellite on sunlit side
    Vec3<double> r_sat;
    r_sat << 7000e3, 0.0, 0.0;  // 7000 km on X-axis
    
    Vec3<double> r_sun;
    r_sun << 1.5e11, 0.0, 0.0;  // Sun in +X direction
    
    double nu = eclipse::shadow_cylindrical(r_sat, r_sun);
    EXPECT_NEAR(nu, 1.0, 1e-6);  // Fully sunlit
}

TEST(Eclipse, SatelliteInShadow) {
    // Satellite behind Earth relative to Sun
    Vec3<double> r_sat;
    r_sat << -7000e3, 0.0, 0.0;  // Behind Earth
    
    Vec3<double> r_sun;
    r_sun << 1.5e11, 0.0, 0.0;  // Sun in +X direction
    
    double nu = eclipse::shadow_cylindrical(r_sat, r_sun);
    EXPECT_NEAR(nu, 0.0, 1e-6);  // In shadow
}
```

### 4.4 Magnetic Field Tests

```cpp
// tests/environment/test_magnetic_field.cpp
TEST(MagneticField, EquatorSurface) {
    Vec3<double> r_eq;
    r_eq << vulcan::constants::earth::R_eq, 0.0, 0.0;
    
    double B = magnetic::field_magnitude(r_eq);
    
    // Equatorial surface field ~31 μT
    EXPECT_NEAR(B * 1e6, 31.2, 2.0);  // μT
}

TEST(MagneticField, PoleSurface) {
    Vec3<double> r_pole;
    r_pole << 0.0, 0.0, vulcan::constants::earth::R_pol;
    
    double B = magnetic::field_magnitude(r_pole);
    
    // Polar surface field ~62 μT (2x equator for dipole)
    EXPECT_NEAR(B * 1e6, 62.0, 5.0);  // μT
}

TEST(MagneticField, FieldDecreaseWithAltitude) {
    // Field should decrease as r⁻³
    Vec3<double> r_surface, r_leo;
    r_surface << vulcan::constants::earth::R_eq, 0.0, 0.0;
    r_leo << vulcan::constants::earth::R_eq + 400e3, 0.0, 0.0;
    
    double B_surface = magnetic::field_magnitude(r_surface);
    double B_leo = magnetic::field_magnitude(r_leo);
    
    double ratio = B_leo / B_surface;
    double expected_ratio = std::pow(6378137.0 / 6778137.0, 3);
    
    EXPECT_NEAR(ratio, expected_ratio, 0.01);
}
```

---

## 5. Verification Plan

### Automated Tests

Run all tests with:
```bash
# In nix environment
./scripts/ci.sh

# Or step by step
./scripts/build.sh
./scripts/test.sh
```

Specific environment tests:
```bash
ctest --test-dir build -R environment -VV
```

### Verification Checklist

- [ ] `./scripts/build.sh` completes without errors
- [ ] `ctest --test-dir build -R environment` passes all tests
- [ ] Solar position at J2000 matches JPL Horizons within 0.5°
- [ ] Eclipse detection correctly identifies umbra/penumbra
- [ ] Magnetic field at equator surface ~31 μT
- [ ] Magnetic field at poles ~62 μT
- [ ] Symbolic mode works for all functions (autodiff)

---

## Proposed Changes

### [NEW] include/vulcan/environment/

#### [NEW] [SolarPosition.hpp](file:///home/tanged/sources/vulcan/include/vulcan/environment/SolarPosition.hpp)
Approximate solar ephemeris using low-precision algorithm from Vallado.

#### [NEW] [Eclipse.hpp](file:///home/tanged/sources/vulcan/include/vulcan/environment/Eclipse.hpp)  
Cylindrical and conical shadow models for eclipse detection.

#### [NEW] [MagneticField.hpp](file:///home/tanged/sources/vulcan/include/vulcan/environment/MagneticField.hpp)
Centered dipole magnetic field model.

#### [NEW] [Environment.hpp](file:///home/tanged/sources/vulcan/include/vulcan/environment/Environment.hpp)
Umbrella header including all environment modules.

---

### [MODIFY] include/vulcan/vulcan.hpp

Add include for environment module umbrella header.

---

### [NEW] tests/environment/

#### [NEW] [test_solar_position.cpp](file:///home/tanged/sources/vulcan/tests/environment/test_solar_position.cpp)
Tests for solar ephemeris accuracy and symbolic compatibility.

#### [NEW] [test_eclipse.cpp](file:///home/tanged/sources/vulcan/tests/environment/test_eclipse.cpp)
Tests for shadow function correctness.

#### [NEW] [test_magnetic_field.cpp](file:///home/tanged/sources/vulcan/tests/environment/test_magnetic_field.cpp)
Tests for dipole field accuracy and r⁻³ decay.

---

### [MODIFY] tests/CMakeLists.txt

Add `test_environment` executable with the new test files.

---

## References

1. Vallado, D.A. "Fundamentals of Astrodynamics and Applications", 4th Ed. - Solar position algorithm
2. Montenbruck, O. & Gill, E. "Satellite Orbits" - Eclipse geometry (Chapter 3.4)
3. Wertz, J.R. "Spacecraft Attitude Determination and Control" - Magnetic field models
