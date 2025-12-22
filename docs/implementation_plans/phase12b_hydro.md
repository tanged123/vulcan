# Phase 12b: Hydrodynamics Module Implementation Plan

> **Purpose**: Implement hydrodynamics utilities for subsea (AUV/ROV) and sea surface vehicle (USV/ship) simulation. Provides individual force components (6-DOF dynamics integration handled by Icarus sim).

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

Provide templated, symbolic-compatible hydrodynamic force components for underwater and surface vehicles:

| Component | Description | Symbolic Support |
|-----------|-------------|------------------|
| **Seawater Properties** | Density, viscosity, speed of sound vs depth | Full |
| **Hydrostatic Forces** | Buoyancy, metacentric height, restoring moments | Full |
| **Added Mass** | 6×6 added mass matrix for standard shapes | Full |
| **Hydrodynamic Damping** | Linear + quadratic damping force components | Full |
| **Current Models** | Ocean current velocity profiles | Full |
| **Sea State** | Pierson-Moskowitz / JONSWAP wave spectra | Full |
| **Wave Forces** | First-order wave forces on marine bodies | Full |

### Design Philosophy

- **Force Components Only**: This module provides hydrodynamic force/moment computation
- **6-DOF Integration**: Full dynamics (M·ν̇ + C·ν + D·ν + g = τ) handled by Icarus
- **Fossen Notation**: Follows standard marine vehicle notation from Fossen (2011)

---

## 2. Architecture

### Namespace Organization

```
vulcan::hydro::
├── seawater::
│   ├── density(depth, temp, salinity)     // UNESCO equation of state
│   ├── speed_of_sound(depth, temp, sal)   // Mackenzie equation
│   ├── viscosity(temp)                    // Dynamic viscosity [Pa·s]
│   └── pressure(depth)                    // Hydrostatic pressure
├── buoyancy::
│   ├── force(volume, density)             // Archimedes' principle
│   ├── restoring_force(W, B, r_g, r_b)    // 6-DOF restoring vector
│   └── metacentric_height(...)            // GM calculation
├── added_mass::
│   ├── sphere(radius, rho)                // Analytical: m_a = 2/3·π·ρ·r³
│   ├── ellipsoid(a, b, c, rho)            // Lamb's solution
│   └── diagonal_matrix(...)               // User-supplied coefficients
├── damping::
│   ├── linear(D_L, v)                     // D·ν term
│   ├── quadratic(D_Q, v)                  // D|ν|·ν term
│   └── combined(D_L, D_Q, v)              // D_L + D_Q|ν|
├── current::
│   ├── constant(Vc, direction)            // Uniform current
│   ├── depth_profile(Vc_surface, z)       // Exponential decay
│   └── relative_velocity(v_body, V_c)     // Body-relative flow
├── sea_state::
│   ├── SeaStateCode                       // World Meteorological Org codes
│   ├── from_code(ss)                      // Get Hs, Tp from sea state code
│   ├── beaufort_to_sea_state(bf)          // Beaufort scale conversion
│   └── wind_to_waves(U10, fetch)          // SMB empirical relationship
└── waves::
    ├── pierson_moskowitz(omega, Hs, Tp)   // Fully developed spectrum
    ├── jonswap(omega, Hs, Tp, gamma)      // Fetch-limited spectrum
    ├── wave_elevation(spectrum, t, x)     // Surface elevation η(x,t)
    ├── wave_kinematics(...)               // u, w, p under waves
    └── first_order_force(...)             // Froude-Krylov + diffraction
```

### File Structure

```
include/vulcan/hydrodynamics/
├── Seawater.hpp           # Water properties vs depth/temp
├── Buoyancy.hpp           # Hydrostatic forces and moments
├── AddedMass.hpp          # Added mass matrices for bodies
├── Damping.hpp            # Hydrodynamic damping models
├── Current.hpp            # Ocean current velocity models
├── SeaState.hpp           # Sea state codes and wave statistics
├── Waves.hpp              # Wave spectra and forces
└── Hydrodynamics.hpp      # Umbrella header

tests/hydrodynamics/
├── test_seawater.cpp
├── test_buoyancy.cpp
├── test_added_mass.cpp
├── test_damping.cpp
├── test_current.cpp
├── test_sea_state.cpp
└── test_waves.cpp

examples/hydrodynamics/
└── marine_vehicle_demo.cpp  # AUV/USV hydrodynamics demonstration
```

---

## 3. Implementation Details

### 3.1 Seawater Properties (`Seawater.hpp`)

UNESCO International Equation of State for seawater density, Mackenzie equation for speed of sound.

```cpp
// include/vulcan/hydrodynamics/Seawater.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>

namespace vulcan::hydro::seawater {

namespace constants {
/// Reference seawater density at surface [kg/m³]
inline constexpr double rho_0 = 1025.0;

/// Average ocean salinity [psu]
inline constexpr double salinity_avg = 35.0;

/// Average ocean temperature [°C]
inline constexpr double temp_avg = 15.0;
}

/**
 * @brief Seawater density (simplified UNESCO equation)
 * 
 * Simplified model: ρ = ρ₀ + 0.77·S + depth/200
 * 
 * @param depth Depth below surface [m] (positive down)
 * @param temp Temperature [°C]
 * @param salinity Salinity [psu] (parts per thousand)
 * @return Density [kg/m³]
 */
template <typename Scalar>
Scalar density(const Scalar& depth, 
               const Scalar& temp = Scalar(constants::temp_avg),
               const Scalar& salinity = Scalar(constants::salinity_avg)) {
    // Base density + salinity effect + pressure effect
    const Scalar rho_base = 1000.0 + 0.77 * salinity;
    const Scalar temp_correction = -0.15 * (temp - 15.0);
    const Scalar pressure_correction = depth / 200.0;
    
    return rho_base + temp_correction + pressure_correction;
}

/**
 * @brief Speed of sound in seawater (Mackenzie equation)
 * 
 * Mackenzie, K.V. (1981). Valid for 0-8000m depth.
 * 
 * @return Speed of sound [m/s]
 */
template <typename Scalar>
Scalar speed_of_sound(const Scalar& depth, const Scalar& temp, 
                      const Scalar& salinity) {
    // Mackenzie equation (simplified)
    const Scalar c = 1448.96 + 4.591 * temp - 0.05304 * temp * temp
                   + 1.340 * (salinity - 35.0)
                   + 0.0163 * depth;
    return c;
}

/**
 * @brief Hydrostatic pressure at depth
 * @return Pressure [Pa]
 */
template <typename Scalar>
Scalar pressure(const Scalar& depth, const Scalar& rho = Scalar(constants::rho_0)) {
    return rho * vulcan::constants::physics::g0 * depth;
}

/**
 * @brief Dynamic viscosity of seawater
 * @param temp Temperature [°C]
 * @return Dynamic viscosity [Pa·s]
 */
template <typename Scalar>
Scalar viscosity(const Scalar& temp) {
    // Approximate: μ ≈ 0.001 Pa·s at 20°C, varies with temp
    return 0.00108 - 0.000023 * (temp - 20.0);
}

} // namespace vulcan::hydro::seawater
```

---

### 3.2 Buoyancy (`Buoyancy.hpp`)

Hydrostatic forces and restoring moments.

```cpp
// include/vulcan/hydrodynamics/Buoyancy.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/hydrodynamics/Seawater.hpp>

namespace vulcan::hydro::buoyancy {

/**
 * @brief Buoyancy force magnitude
 * 
 * F_b = ρ·g·V (Archimedes' principle)
 * 
 * @param volume Displaced volume [m³]
 * @param density Fluid density [kg/m³]
 * @return Buoyancy force [N] (positive upward)
 */
template <typename Scalar>
Scalar force(const Scalar& volume, const Scalar& density) {
    return density * vulcan::constants::physics::g0 * volume;
}

/**
 * @brief 6-DOF restoring force/moment vector g(η)
 * 
 * Computes gravitational and buoyancy restoring forces/moments.
 * 
 * g(η) = [         (W - B)·sin(θ)          ]
 *        [        -(W - B)·cos(θ)·sin(φ)   ]
 *        [        -(W - B)·cos(θ)·cos(φ)   ]
 *        [  -(y_g·W - y_b·B)·cos(θ)·cos(φ) + (z_g·W - z_b·B)·cos(θ)·sin(φ)  ]
 *        [   (z_g·W - z_b·B)·sin(θ) + (x_g·W - x_b·B)·cos(θ)·cos(φ)  ]
 *        [  -(x_g·W - x_b·B)·cos(θ)·sin(φ) - (y_g·W - y_b·B)·sin(θ)  ]
 * 
 * @param W Weight [N]
 * @param B Buoyancy [N]
 * @param r_g Center of gravity in body frame [m]
 * @param r_b Center of buoyancy in body frame [m]
 * @param phi Roll angle [rad]
 * @param theta Pitch angle [rad]
 * @return 6-DOF restoring vector [N, N, N, Nm, Nm, Nm]
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 6, 1> restoring_forces(
    const Scalar& W, const Scalar& B,
    const Vec3<Scalar>& r_g, const Vec3<Scalar>& r_b,
    const Scalar& phi, const Scalar& theta) {
    
    using janus::sin;
    using janus::cos;
    
    const Scalar s_phi = sin(phi);
    const Scalar c_phi = cos(phi);
    const Scalar s_theta = sin(theta);
    const Scalar c_theta = cos(theta);
    
    Eigen::Matrix<Scalar, 6, 1> g;
    
    // Forces
    g(0) = (W - B) * s_theta;
    g(1) = -(W - B) * c_theta * s_phi;
    g(2) = -(W - B) * c_theta * c_phi;
    
    // Moments (simplified for small CG-CB offset)
    const Scalar x_g = r_g(0), y_g = r_g(1), z_g = r_g(2);
    const Scalar x_b = r_b(0), y_b = r_b(1), z_b = r_b(2);
    
    g(3) = -(y_g*W - y_b*B)*c_theta*c_phi + (z_g*W - z_b*B)*c_theta*s_phi;
    g(4) = (z_g*W - z_b*B)*s_theta + (x_g*W - x_b*B)*c_theta*c_phi;
    g(5) = -(x_g*W - x_b*B)*c_theta*s_phi - (y_g*W - y_b*B)*s_theta;
    
    return g;
}

/**
 * @brief Metacentric height for roll stability
 * 
 * GM = KB + BM - KG
 * 
 * @param KB Distance from keel to center of buoyancy [m]
 * @param BM Metacentric radius [m]
 * @param KG Distance from keel to center of gravity [m]
 * @return Metacentric height [m] (positive = stable)
 */
template <typename Scalar>
Scalar metacentric_height(const Scalar& KB, const Scalar& BM, const Scalar& KG) {
    return KB + BM - KG;
}

} // namespace vulcan::hydro::buoyancy
```

---

### 3.3 Added Mass (`AddedMass.hpp`)

Analytical added mass for standard shapes.

```cpp
// include/vulcan/hydrodynamics/AddedMass.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <Eigen/Dense>

namespace vulcan::hydro::added_mass {

/**
 * @brief Added mass for a sphere
 * 
 * m_a = (2/3)·π·ρ·r³ = 0.5·m_displaced
 * 
 * @param radius Sphere radius [m]
 * @param rho Fluid density [kg/m³]
 * @return Added mass for surge/sway/heave [kg]
 */
template <typename Scalar>
Scalar sphere(const Scalar& radius, const Scalar& rho) {
    const Scalar volume = (4.0 / 3.0) * constants::angle::pi * radius * radius * radius;
    return 0.5 * rho * volume;  // Half the displaced mass
}

/**
 * @brief Added mass coefficients for a prolate ellipsoid
 * 
 * Uses Lamb's k-factors for ellipsoid of revolution.
 * 
 * @param a Semi-major axis (length/2) [m]
 * @param b Semi-minor axis (diameter/2) [m]
 * @param rho Fluid density [kg/m³]
 * @return {m_surge, m_sway, m_heave} added masses [kg]
 */
template <typename Scalar>
Vec3<Scalar> ellipsoid(const Scalar& a, const Scalar& b, const Scalar& rho) {
    // Eccentricity
    const Scalar e = janus::sqrt(1.0 - (b*b)/(a*a));
    
    // Lamb's α₀ for surge (motion along major axis)
    const Scalar alpha_0 = (2.0 * (1.0 - e*e) / (e*e*e)) * 
                           (0.5 * janus::log((1.0 + e)/(1.0 - e)) - e);
    
    // Lamb's β₀ for sway/heave (motion perpendicular to major axis)
    const Scalar beta_0 = (1.0 / (e*e)) - 
                          ((1.0 - e*e) / (2.0*e*e*e)) * janus::log((1.0 + e)/(1.0 - e));
    
    // k-factors
    const Scalar k1 = alpha_0 / (2.0 - alpha_0);  // Surge
    const Scalar k2 = beta_0 / (2.0 - beta_0);    // Sway/heave
    
    // Displaced mass
    const Scalar m_disp = (4.0/3.0) * constants::angle::pi * a * b * b * rho;
    
    Vec3<Scalar> m_a;
    m_a << k1 * m_disp, k2 * m_disp, k2 * m_disp;
    
    return m_a;
}

/**
 * @brief 6×6 added mass matrix for a symmetric vehicle
 * 
 * Diagonal approximation for vehicles with symmetry planes.
 * 
 * @param m_x Added mass in surge [kg]
 * @param m_y Added mass in sway [kg]
 * @param m_z Added mass in heave [kg]
 * @param I_x Added inertia in roll [kg·m²]
 * @param I_y Added inertia in pitch [kg·m²]
 * @param I_z Added inertia in yaw [kg·m²]
 * @return 6×6 added mass matrix M_A
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 6, 6> diagonal_matrix(
    const Scalar& m_x, const Scalar& m_y, const Scalar& m_z,
    const Scalar& I_x, const Scalar& I_y, const Scalar& I_z) {
    
    Eigen::Matrix<Scalar, 6, 6> M_A = Eigen::Matrix<Scalar, 6, 6>::Zero();
    M_A(0, 0) = m_x;
    M_A(1, 1) = m_y;
    M_A(2, 2) = m_z;
    M_A(3, 3) = I_x;
    M_A(4, 4) = I_y;
    M_A(5, 5) = I_z;
    
    return M_A;
}

} // namespace vulcan::hydro::added_mass
```

---

### 3.4 Damping (`Damping.hpp`)

Linear and quadratic hydrodynamic damping.

```cpp
// include/vulcan/hydrodynamics/Damping.hpp
#pragma once

#include <janus/janus.hpp>
#include <Eigen/Dense>

namespace vulcan::hydro::damping {

/**
 * @brief Linear damping force
 * 
 * F_D = -D_L · ν
 * 
 * @param D_L 6×6 linear damping matrix [N/(m/s) or Nm/(rad/s)]
 * @param velocity 6-DOF velocity vector [m/s, rad/s]
 * @return Damping force/moment vector
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 6, 1> linear(
    const Eigen::Matrix<Scalar, 6, 6>& D_L,
    const Eigen::Matrix<Scalar, 6, 1>& velocity) {
    return -D_L * velocity;
}

/**
 * @brief Quadratic damping force
 * 
 * F_D = -D_Q · |ν| · ν  (element-wise)
 * 
 * @param D_Q 6×6 quadratic damping matrix [N/(m/s)² or Nm/(rad/s)²]
 * @param velocity 6-DOF velocity vector
 * @return Damping force/moment vector
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 6, 1> quadratic(
    const Eigen::Matrix<Scalar, 6, 6>& D_Q,
    const Eigen::Matrix<Scalar, 6, 1>& velocity) {
    
    // Element-wise |v| · v
    Eigen::Matrix<Scalar, 6, 1> v_abs_v;
    for (int i = 0; i < 6; ++i) {
        v_abs_v(i) = janus::abs(velocity(i)) * velocity(i);
    }
    
    return -D_Q * v_abs_v;
}

/**
 * @brief Combined linear + quadratic damping
 * 
 * F_D = -(D_L + D_Q·|ν|) · ν
 * 
 * @param D_L Linear damping matrix
 * @param D_Q Quadratic damping matrix
 * @param velocity 6-DOF velocity
 * @return Total damping force/moment
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 6, 1> combined(
    const Eigen::Matrix<Scalar, 6, 6>& D_L,
    const Eigen::Matrix<Scalar, 6, 6>& D_Q,
    const Eigen::Matrix<Scalar, 6, 1>& velocity) {
    return linear(D_L, velocity) + quadratic(D_Q, velocity);
}

/**
 * @brief Create diagonal damping matrix from coefficients
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 6, 6> diagonal_matrix(
    const Scalar& d_u, const Scalar& d_v, const Scalar& d_w,
    const Scalar& d_p, const Scalar& d_q, const Scalar& d_r) {
    
    Eigen::Matrix<Scalar, 6, 6> D = Eigen::Matrix<Scalar, 6, 6>::Zero();
    D(0, 0) = d_u;
    D(1, 1) = d_v;
    D(2, 2) = d_w;
    D(3, 3) = d_p;
    D(4, 4) = d_q;
    D(5, 5) = d_r;
    
    return D;
}

} // namespace vulcan::hydro::damping
```

---

### 3.5 Ocean Current (`Current.hpp`)

Ocean current velocity models.

```cpp
// include/vulcan/hydrodynamics/Current.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::hydro::current {

/**
 * @brief Constant ocean current
 * 
 * @param speed Current speed [m/s]
 * @param heading Current direction [rad] (0 = North, π/2 = East)
 * @return Current velocity in NED frame [m/s]
 */
template <typename Scalar>
Vec3<Scalar> constant(const Scalar& speed, const Scalar& heading) {
    Vec3<Scalar> V_c;
    V_c(0) = speed * janus::cos(heading);  // North
    V_c(1) = speed * janus::sin(heading);  // East
    V_c(2) = Scalar(0.0);                  // Down
    return V_c;
}

/**
 * @brief Depth-dependent current (exponential profile)
 * 
 * V(z) = V_surface · exp(-z / z_ref)
 * 
 * @param V_surface Surface current speed [m/s]
 * @param heading Current direction [rad]
 * @param depth Depth below surface [m] (positive down)
 * @param z_ref Reference depth for decay [m]
 * @return Current velocity in NED frame [m/s]
 */
template <typename Scalar>
Vec3<Scalar> depth_profile(const Scalar& V_surface, const Scalar& heading,
                           const Scalar& depth, const Scalar& z_ref) {
    const Scalar V = V_surface * janus::exp(-depth / z_ref);
    return constant(V, heading);
}

/**
 * @brief Relative velocity (body velocity minus current)
 * 
 * V_r = V_body - R_nb^T · V_c
 * 
 * @param V_body Velocity in body frame [m/s]
 * @param V_current Current in NED frame [m/s]
 * @param R_nb Rotation matrix from NED to body
 * @return Relative velocity in body frame [m/s]
 */
template <typename Scalar>
Vec3<Scalar> relative_velocity(const Vec3<Scalar>& V_body,
                               const Vec3<Scalar>& V_current,
                               const Eigen::Matrix<Scalar, 3, 3>& R_nb) {
    return V_body - R_nb.transpose() * V_current;
}

} // namespace vulcan::hydro::current
```

---

### 3.6 Sea State (`SeaState.hpp`)

World Meteorological Organization sea state codes and wave statistics.

```cpp
// include/vulcan/hydrodynamics/SeaState.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/VulcanTypes.hpp>
#include <array>

namespace vulcan::hydro::sea_state {

/**
 * @brief World Meteorological Organization (WMO) Sea State Codes
 * 
 * Douglas Sea Scale for swell waves.
 */
enum class SeaStateCode : int {
    SS0_CALM_GLASSY = 0,      // Hs = 0 m
    SS1_CALM_RIPPLED = 1,     // Hs = 0-0.1 m
    SS2_SMOOTH = 2,           // Hs = 0.1-0.5 m
    SS3_SLIGHT = 3,           // Hs = 0.5-1.25 m
    SS4_MODERATE = 4,         // Hs = 1.25-2.5 m
    SS5_ROUGH = 5,            // Hs = 2.5-4.0 m
    SS6_VERY_ROUGH = 6,       // Hs = 4.0-6.0 m
    SS7_HIGH = 7,             // Hs = 6.0-9.0 m
    SS8_VERY_HIGH = 8,        // Hs = 9.0-14.0 m
    SS9_PHENOMENAL = 9        // Hs > 14.0 m
};

/**
 * @brief Sea state parameters
 */
struct SeaStateParams {
    double Hs_min;      ///< Minimum significant wave height [m]
    double Hs_max;      ///< Maximum significant wave height [m]
    double Hs_typical;  ///< Typical significant wave height [m]
    double Tp_typical;  ///< Typical peak period [s]
};

/**
 * @brief Get wave parameters from sea state code
 */
inline SeaStateParams from_code(SeaStateCode code) {
    // Sea state parameters based on WMO Douglas Scale
    static const std::array<SeaStateParams, 10> params = {{
        {0.0,  0.0,  0.0,  0.0},    // SS0
        {0.0,  0.1,  0.05, 2.0},    // SS1
        {0.1,  0.5,  0.3,  4.0},    // SS2
        {0.5,  1.25, 0.9,  5.5},    // SS3
        {1.25, 2.5,  1.9,  7.0},    // SS4
        {2.5,  4.0,  3.2,  9.0},    // SS5
        {4.0,  6.0,  5.0,  11.0},   // SS6
        {6.0,  9.0,  7.5,  13.0},   // SS7
        {9.0,  14.0, 11.5, 16.0},   // SS8
        {14.0, 20.0, 17.0, 20.0}    // SS9
    }};
    return params[static_cast<int>(code)];
}

/**
 * @brief Convert Beaufort wind scale to sea state
 * 
 * Approximate mapping Beaufort → Douglas Sea State
 */
inline SeaStateCode beaufort_to_sea_state(int beaufort) {
    if (beaufort <= 0) return SeaStateCode::SS0_CALM_GLASSY;
    if (beaufort <= 1) return SeaStateCode::SS1_CALM_RIPPLED;
    if (beaufort <= 2) return SeaStateCode::SS2_SMOOTH;
    if (beaufort <= 3) return SeaStateCode::SS3_SLIGHT;
    if (beaufort <= 5) return SeaStateCode::SS4_MODERATE;
    if (beaufort <= 6) return SeaStateCode::SS5_ROUGH;
    if (beaufort <= 7) return SeaStateCode::SS6_VERY_ROUGH;
    if (beaufort <= 9) return SeaStateCode::SS7_HIGH;
    if (beaufort <= 11) return SeaStateCode::SS8_VERY_HIGH;
    return SeaStateCode::SS9_PHENOMENAL;
}

/**
 * @brief Significant wave height from wind (SMB empirical)
 * 
 * Sverdrup-Munk-Bretschneider empirical relationship for
 * fully developed seas.
 * 
 * @param U10 Wind speed at 10m height [m/s]
 * @return Significant wave height Hs [m]
 */
template <typename Scalar>
Scalar wind_to_Hs(const Scalar& U10) {
    // SMB formula: Hs = 0.0248 * U10^2
    return 0.0248 * U10 * U10;
}

/**
 * @brief Peak wave period from wind (SMB empirical)
 * 
 * @param U10 Wind speed at 10m height [m/s]
 * @return Peak period Tp [s]
 */
template <typename Scalar>
Scalar wind_to_Tp(const Scalar& U10) {
    // SMB formula: Tp = 0.729 * U10
    return 0.729 * U10;
}

} // namespace vulcan::hydro::sea_state
```

---

### 3.7 Waves (`Waves.hpp`)

Wave spectra and wave-induced forces.

```cpp
// include/vulcan/hydrodynamics/Waves.hpp
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::hydro::waves {

namespace constants {
/// Acceleration due to gravity [m/s²]
inline constexpr double g = vulcan::constants::physics::g0;
}

/**
 * @brief Pierson-Moskowitz wave spectrum
 * 
 * Fully developed sea spectrum. Valid for unlimited fetch conditions.
 * 
 * S(ω) = (α·g²/ω⁵) · exp(-β·(ωp/ω)⁴)
 * 
 * where α = 8.1e-3, β = 0.74, ωp = 2π/Tp
 * 
 * @tparam Scalar double or casadi::MX
 * @param omega Angular frequency [rad/s]
 * @param Hs Significant wave height [m]
 * @param Tp Peak period [s]
 * @return Spectral density S(ω) [m²·s/rad]
 */
template <typename Scalar>
Scalar pierson_moskowitz(const Scalar& omega, const Scalar& Hs, 
                         const Scalar& Tp) {
    const Scalar omega_p = 2.0 * constants::angle::pi / Tp;
    const Scalar omega_ratio = omega_p / omega;
    
    // PM parameters for Hs-Tp formulation
    const Scalar A = (5.0 / 16.0) * Hs * Hs * 
                     janus::pow(omega_p, Scalar(4.0));
    const Scalar B = 5.0 / 4.0;
    
    return (A / janus::pow(omega, Scalar(5.0))) * 
           janus::exp(-B * janus::pow(omega_ratio, Scalar(4.0)));
}

/**
 * @brief JONSWAP wave spectrum
 * 
 * Fetch-limited sea spectrum. Sharper peak than Pierson-Moskowitz.
 * 
 * S_J(ω) = S_PM(ω) · γ^α
 * 
 * where α = exp(-(ω - ωp)² / (2·σ²·ωp²))
 * 
 * @param omega Angular frequency [rad/s]
 * @param Hs Significant wave height [m]
 * @param Tp Peak period [s]
 * @param gamma Peak enhancement factor (default 3.3)
 * @return Spectral density S(ω) [m²·s/rad]
 */
template <typename Scalar>
Scalar jonswap(const Scalar& omega, const Scalar& Hs, const Scalar& Tp,
               const Scalar& gamma = Scalar(3.3)) {
    const Scalar omega_p = 2.0 * constants::angle::pi / Tp;
    
    // Sigma parameter (spectral width)
    const Scalar is_lower = omega <= omega_p;
    const Scalar sigma = janus::where(is_lower, Scalar(0.07), Scalar(0.09));
    
    // Peak enhancement exponent
    const Scalar delta_omega = omega - omega_p;
    const Scalar alpha = janus::exp(
        -(delta_omega * delta_omega) / 
        (2.0 * sigma * sigma * omega_p * omega_p)
    );
    
    // JONSWAP = PM × γ^α
    return pierson_moskowitz(omega, Hs, Tp) * janus::pow(gamma, alpha);
}

/**
 * @brief Wave elevation at a point (linear superposition)
 * 
 * η(x,t) = Σ aᵢ·cos(kᵢ·x - ωᵢ·t + φᵢ)
 * 
 * This is a single-component version for frequency-domain analysis.
 * 
 * @param a Wave amplitude [m]
 * @param k Wave number [1/m]
 * @param omega Angular frequency [rad/s]
 * @param x Horizontal position [m]
 * @param t Time [s]
 * @param phi Phase [rad]
 * @return Surface elevation [m]
 */
template <typename Scalar>
Scalar wave_elevation(const Scalar& a, const Scalar& k, const Scalar& omega,
                      const Scalar& x, const Scalar& t, 
                      const Scalar& phi = Scalar(0.0)) {
    return a * janus::cos(k * x - omega * t + phi);
}

/**
 * @brief Deep water dispersion relation
 * 
 * ω² = g·k  →  k = ω²/g
 * 
 * @param omega Angular frequency [rad/s]
 * @return Wave number k [1/m]
 */
template <typename Scalar>
Scalar wavenumber_deep(const Scalar& omega) {
    return omega * omega / constants::g;
}

/**
 * @brief Wave-induced horizontal velocity (Airy theory)
 * 
 * u(z,t) = a·ω·exp(k·z)·cos(k·x - ω·t + φ)
 * 
 * Valid for deep water (z < 0, positive upward from surface).
 * 
 * @param a Wave amplitude [m]
 * @param omega Angular frequency [rad/s]
 * @param k Wave number [1/m]
 * @param z Depth below surface [m] (negative values, 0 at surface)
 * @param x Horizontal position [m]
 * @param t Time [s]
 * @return Horizontal particle velocity [m/s]
 */
template <typename Scalar>
Scalar horizontal_velocity(const Scalar& a, const Scalar& omega, 
                           const Scalar& k, const Scalar& z,
                           const Scalar& x, const Scalar& t) {
    return a * omega * janus::exp(k * z) * janus::cos(k * x - omega * t);
}

/**
 * @brief Wave-induced vertical velocity (Airy theory)
 * 
 * w(z,t) = a·ω·exp(k·z)·sin(k·x - ω·t + φ)
 */
template <typename Scalar>
Scalar vertical_velocity(const Scalar& a, const Scalar& omega,
                         const Scalar& k, const Scalar& z,
                         const Scalar& x, const Scalar& t) {
    return a * omega * janus::exp(k * z) * janus::sin(k * x - omega * t);
}

/**
 * @brief First-order wave excitation force (heave direction)
 * 
 * Simplified Froude-Krylov force for a spherical body:
 * F_z = ρ·g·V·exp(k·z_c)·cos(ω·t)
 * 
 * where z_c is the center depth.
 * 
 * @param rho Fluid density [kg/m³]
 * @param volume Displaced volume [m³]
 * @param a Wave amplitude [m]
 * @param omega Angular frequency [rad/s]
 * @param k Wave number [1/m]
 * @param z_center Center depth [m] (negative below surface)
 * @param t Time [s]
 * @return Vertical wave force [N]
 */
template <typename Scalar>
Scalar heave_excitation_force(const Scalar& rho, const Scalar& volume,
                              const Scalar& a, const Scalar& omega,
                              const Scalar& k, const Scalar& z_center,
                              const Scalar& t) {
    const Scalar g = constants::g;
    return rho * g * volume * a * k * janus::exp(k * z_center) * 
           janus::cos(omega * t);
}

/**
 * @brief Wave drift force (second-order)
 * 
 * Mean drift force in wave direction:
 * F_drift = 0.5·ρ·g·L·a²
 * 
 * Simplified for a vertical wall/surface of width L.
 * 
 * @param rho Fluid density [kg/m³]
 * @param L Characteristic width [m]
 * @param a Wave amplitude [m]
 * @return Mean drift force [N]
 */
template <typename Scalar>
Scalar wave_drift_force(const Scalar& rho, const Scalar& L, const Scalar& a) {
    return 0.5 * rho * constants::g * L * a * a;
}

} // namespace vulcan::hydro::waves
```

---
## 4. Testing Strategy

### Test Categories

| Category | Purpose | Validation Source |
|----------|---------|-------------------|
| Seawater | Density, sound speed vs depth | UNESCO tables |
| Buoyancy | Archimedes validation | Simple geometry |
| Added Mass | Sphere/ellipsoid analytical | Lamb, Newman |
| Damping | Matrix operations | Manual calculation |
| Symbolic | Verify autodiff | Finite difference |

### Key Tests

```cpp
// tests/hydrodynamics/test_seawater.cpp
TEST(Seawater, SurfaceDensity) {
    double rho = vulcan::hydro::seawater::density(0.0, 15.0, 35.0);
    EXPECT_NEAR(rho, 1025.0, 1.0);  // ~1025 kg/m³
}

TEST(Seawater, DeepDensity) {
    // Density increases with depth (~1050 kg/m³ at 5000m)
    double rho_surface = vulcan::hydro::seawater::density(0.0);
    double rho_deep = vulcan::hydro::seawater::density(5000.0);
    EXPECT_GT(rho_deep, rho_surface);
}

// tests/hydrodynamics/test_added_mass.cpp
TEST(AddedMass, Sphere) {
    // Added mass of sphere = 0.5 × displaced mass
    double r = 1.0;
    double rho = 1025.0;
    double m_a = vulcan::hydro::added_mass::sphere(r, rho);
    double m_disp = rho * (4.0/3.0) * M_PI * r * r * r;
    EXPECT_NEAR(m_a, 0.5 * m_disp, 1.0);
}

// tests/hydrodynamics/test_buoyancy.cpp
TEST(Buoyancy, NeutralBuoyancy) {
    double W = 1000.0;  // Weight 1000 N
    double B = 1000.0;  // Buoyancy 1000 N
    Vec3<double> r_g, r_b;
    r_g << 0, 0, 0;
    r_b << 0, 0, 0;
    
    auto g = vulcan::hydro::buoyancy::restoring_forces(W, B, r_g, r_b, 0, 0);
    
    // Neutral buoyancy: no restoring force at level attitude
    EXPECT_NEAR(g(2), 0.0, 1e-6);  // Heave force
}
```

---

## 5. Verification Plan

### Automated Tests

```bash
# Run after implementation
./scripts/ci.sh

# Run just hydrodynamics tests
ctest --test-dir build -R hydrodynamics -VV
```

### Test Checklist

- [ ] Seawater density at surface ≈ 1025 kg/m³
- [ ] Seawater density increases with depth
- [ ] Speed of sound ≈ 1500 m/s at surface
- [ ] Sphere added mass = 0.5 × displaced mass
- [ ] Neutral buoyancy produces zero restoring force
- [ ] Positive buoyancy produces upward force
- [ ] Damping opposes velocity direction
- [ ] PM and JONSWAP spectra peak at correct frequency
- [ ] Wave kinematics decay with depth (Airy theory)
- [ ] Symbolic mode produces valid expressions

---

## Proposed Changes

### [NEW] include/vulcan/hydrodynamics/

| File | Description |
|------|-------------|
| [Seawater.hpp](file:///home/tanged/sources/vulcan/include/vulcan/hydrodynamics/Seawater.hpp) | UNESCO density, Mackenzie sound speed |
| [Buoyancy.hpp](file:///home/tanged/sources/vulcan/include/vulcan/hydrodynamics/Buoyancy.hpp) | Hydrostatic forces, restoring moments |
| [AddedMass.hpp](file:///home/tanged/sources/vulcan/include/vulcan/hydrodynamics/AddedMass.hpp) | Analytical added mass for shapes |
| [Damping.hpp](file:///home/tanged/sources/vulcan/include/vulcan/hydrodynamics/Damping.hpp) | Linear + quadratic damping |
| [Current.hpp](file:///home/tanged/sources/vulcan/include/vulcan/hydrodynamics/Current.hpp) | Ocean current models |
| [SeaState.hpp](file:///home/tanged/sources/vulcan/include/vulcan/hydrodynamics/SeaState.hpp) | WMO sea state codes, Beaufort scale |
| [Waves.hpp](file:///home/tanged/sources/vulcan/include/vulcan/hydrodynamics/Waves.hpp) | PM/JONSWAP spectra, wave forces |
| [Hydrodynamics.hpp](file:///home/tanged/sources/vulcan/include/vulcan/hydrodynamics/Hydrodynamics.hpp) | Umbrella header |

---

### [MODIFY] include/vulcan/vulcan.hpp

Add include for hydrodynamics umbrella header.

---

### [NEW] tests/hydrodynamics/

| File | Tests |
|------|-------|
| test_seawater.cpp | Density, sound speed, viscosity |
| test_buoyancy.cpp | Force, restoring moments, stability |
| test_added_mass.cpp | Sphere, ellipsoid, matrices |
| test_damping.cpp | Linear, quadratic, combined |
| test_current.cpp | Constant, depth profile, relative |
| test_sea_state.cpp | WMO codes, Beaufort mapping, SMB |
| test_waves.cpp | PM/JONSWAP spectra, wave kinematics |

---

### [MODIFY] tests/CMakeLists.txt

Add `test_hydrodynamics` executable.

---

### [NEW] examples/hydrodynamics/marine_vehicle_demo.cpp

Demo showing AUV/USV hydrodynamic properties, wave environment, and forces.

---

### [NEW] docs/user_guides/hydrodynamics.md

User documentation with API reference and examples.

---

## References

1. Fossen, T.I. "Handbook of Marine Craft Hydrodynamics and Motion Control" (2011)
2. Newman, J.N. "Marine Hydrodynamics" (1977)
3. UNESCO "Algorithms for computation of fundamental properties of seawater" (1983)
4. Mackenzie, K.V. "Nine-term equation for sound speed in the oceans" (1981)
5. Pierson, W.J. & Moskowitz, L. "A Proposed Spectral Form for Fully Developed Wind Seas" (1964)
6. Hasselmann, K. et al. "Measurements of wind-wave growth and swell decay during JONSWAP" (1973)
7. WMO "Manual on Codes - International Codes, Vol. I.1" - Sea State Codes
