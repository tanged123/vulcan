# Phase 5: Wind Models Implementation Plan

This phase implements atmospheric wind models for flight simulation and trajectory optimization in Vulcan. The models are designed for dual numeric/symbolic compatibility with Janus.

## Background

Wind models are essential for:
- Realistic flight simulation with atmospheric disturbances
- Trajectory optimization under uncertain wind conditions
- Vehicle stability and control analysis
- Gust load estimation for structural design

We will implement four models with increasing complexity:

1. **Constant Wind Field** - Uniform wind vector (baseline)
2. **Wind Shear Profiles** - Altitude-dependent wind (linear, exponential, power-law)
3. **Dryden Turbulence Model** - Rational PSD, easier to implement
4. **von Kármán Turbulence Model** - Irrational PSD, more accurate matching of atmospheric data

---

## User Review Required

> [!IMPORTANT]
> **Design Decision: Turbulence Model Implementation Approach**
> 
> Turbulence models use power spectral density (PSD) equations that require filtering white noise to produce time-correlated gusts. Since CasADi (symbolic backend) does not support stochastic processes directly, we have two options:
> 
> **Option A (Recommended)**: Implement PSD equations and transfer functions for **user-side integration** - user provides white noise input, we output shaped turbulence. This is fully symbolic-compatible.
> 
> **Option B**: Provide **numeric-only** turbulence generators using `<random>` that return sampled gust velocities. Not symbolic-compatible but simpler for users.
> 
> **Recommendation**: Option A for core library (symbolic-compatible forming filters), with optional numeric helpers for convenience.

> [!WARNING]
> **State-Based Turbulence**
> 
> The Dryden and von Kármán models are implemented as **state-space filters** (forming filters). This means:
> - Turbulence velocity depends on **previous state** and **white noise input**
> - The filter must be stepped in time (discrete integration)
> - User must manage state persistence between time steps
> 
> This is different from the stateless atmosphere models (temperature is purely a function of altitude).

> [!NOTE]
> **MIL-F-8785C / MIL-HDBK-1797 Compliance**
> 
> The turbulence intensity (σ) and scale length (L) parameters follow military specifications and vary with altitude, airspeed, and atmospheric conditions. We will provide convenience functions for MIL-spec parameter lookup.

---

## Proposed Changes

### Component 1: Wind Module Structure

Create a new `wind/` module following the existing atmosphere pattern.

#### [NEW] [include/vulcan/wind/](file:///home/tanged/sources/temp/vulcan/include/vulcan/wind/)

New directory for wind models.

---

### Component 2: Common Wind Types

#### [NEW] [WindTypes.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/wind/WindTypes.hpp)

Common types and utilities for all wind models.

```cpp
#pragma once

#include <janus/janus.hpp>

namespace vulcan::wind {

// ============================================================================
// Wind Vector Representation
// ============================================================================

/**
 * @brief 3D wind velocity in NED frame
 * 
 * Wind components represent the velocity of the air mass relative to the Earth.
 * Positive North = wind blowing from South to North
 * Positive East = wind blowing from West to East  
 * Positive Down = wind blowing upward (rare, usually near-zero)
 */
template <typename Scalar> 
struct WindVector {
    Scalar north;  ///< North component [m/s]
    Scalar east;   ///< East component [m/s]
    Scalar down;   ///< Down component [m/s]
    
    /// Magnitude in horizontal plane
    Scalar horizontal_speed() const {
        return janus::sqrt(north * north + east * east);
    }
    
    /// Total magnitude
    Scalar speed() const {
        return janus::sqrt(north * north + east * east + down * down);
    }
    
    /// Direction wind is coming FROM (meteorological convention)
    /// Returns angle in radians from North, clockwise positive
    Scalar direction_from() const {
        return janus::atan2(east, north);
    }
};

/**
 * @brief Turbulent gust velocities in body frame
 * 
 * Linear gust components in aircraft body-fixed frame (uvw convention).
 */
template <typename Scalar>
struct GustVelocity {
    Scalar u_g;  ///< Longitudinal gust [m/s] (along body x-axis)
    Scalar v_g;  ///< Lateral gust [m/s] (along body y-axis)  
    Scalar w_g;  ///< Vertical gust [m/s] (along body z-axis)
};

/**
 * @brief Angular gust rates in body frame
 * 
 * Angular velocity perturbations caused by spatial gradients of linear gusts.
 */
template <typename Scalar>
struct GustAngularRate {
    Scalar p_g;  ///< Roll gust rate [rad/s]
    Scalar q_g;  ///< Pitch gust rate [rad/s]
    Scalar r_g;  ///< Yaw gust rate [rad/s]
};

// ============================================================================
// Turbulence Parameters
// ============================================================================

/**
 * @brief Turbulence intensity and scale parameters (MIL-F-8785C)
 */
struct TurbulenceParams {
    double sigma_u;  ///< Longitudinal RMS intensity [m/s]
    double sigma_v;  ///< Lateral RMS intensity [m/s]
    double sigma_w;  ///< Vertical RMS intensity [m/s]
    double L_u;      ///< Longitudinal scale length [m]
    double L_v;      ///< Lateral scale length [m]
    double L_w;      ///< Vertical scale length [m]
};

/**
 * @brief Turbulence severity levels per MIL-HDBK-1797
 */
enum class TurbulenceSeverity {
    Light,     ///< σ_w ≈ 1 m/s at low altitude
    Moderate,  ///< σ_w ≈ 3 m/s at low altitude
    Severe     ///< σ_w ≈ 7 m/s at low altitude
};

/**
 * @brief Compute MIL-spec turbulence parameters for given conditions
 * 
 * @param altitude Altitude above ground level [m]
 * @param severity Turbulence severity level
 * @return TurbulenceParams with appropriate σ and L values
 */
TurbulenceParams mil_spec_params(double altitude, TurbulenceSeverity severity);

} // namespace vulcan::wind
```

---

### Component 3: Constant Wind Field

#### [NEW] [ConstantWind.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/wind/ConstantWind.hpp)

Simple constant wind vector - useful as baseline and for testing.

```cpp
#pragma once

#include <vulcan/wind/WindTypes.hpp>

namespace vulcan::constant_wind {

/**
 * @brief Create constant wind from NED components
 * 
 * @tparam Scalar double or casadi::MX
 * @param north North wind component [m/s]
 * @param east East wind component [m/s]
 * @param down Down wind component [m/s] (usually 0)
 * @return WindVector with constant components
 */
template <typename Scalar>
wind::WindVector<Scalar> from_ned(const Scalar& north, 
                                   const Scalar& east, 
                                   const Scalar& down = Scalar(0));

/**
 * @brief Create constant wind from speed and direction
 * 
 * Uses meteorological convention: direction is where wind comes FROM.
 * 
 * @tparam Scalar double or casadi::MX
 * @param speed Wind speed [m/s]
 * @param direction_from Direction wind is blowing FROM [rad], clockwise from North
 * @return WindVector in NED frame
 */
template <typename Scalar>
wind::WindVector<Scalar> from_speed_direction(const Scalar& speed,
                                               const Scalar& direction_from);

} // namespace vulcan::constant_wind
```

---

### Component 4: Wind Shear Profiles

#### [NEW] [WindShear.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/wind/WindShear.hpp)

Altitude-dependent wind profiles for boundary layer and low-level wind shear.

```cpp
#pragma once

#include <vulcan/wind/WindTypes.hpp>

namespace vulcan::wind_shear {

// ============================================================================
// Linear Wind Shear
// ============================================================================

/**
 * @brief Linear wind shear profile
 * 
 * Wind = W_0 + (dW/dh) * (h - h_0)
 * 
 * @tparam Scalar double or casadi::MX
 * @param altitude Altitude [m]
 * @param base_wind Wind at reference altitude [m/s]
 * @param base_altitude Reference altitude [m]
 * @param shear_rate Wind change per unit altitude [1/s], i.e., dV/dh
 * @return Wind speed at given altitude [m/s]
 */
template <typename Scalar>
Scalar linear(const Scalar& altitude,
              double base_wind,
              double base_altitude,
              double shear_rate);

/**
 * @brief Linear wind shear returning full wind vector
 */
template <typename Scalar>
wind::WindVector<Scalar> linear_vector(const Scalar& altitude,
                                        const wind::WindVector<double>& base_wind,
                                        double base_altitude,
                                        double shear_rate);

// ============================================================================
// Power Law Wind Shear (Atmospheric Boundary Layer)
// ============================================================================

/**
 * @brief Power-law wind profile (typical for atmospheric boundary layer)
 * 
 * V(h) = V_ref * (h / h_ref)^alpha
 * 
 * Common exponents:
 * - α = 1/7 (0.143) - unstable/daytime conditions
 * - α = 1/4 (0.25) - neutral conditions  
 * - α = 1/3 (0.33) - stable/nighttime conditions
 * 
 * @tparam Scalar double or casadi::MX
 * @param altitude Altitude above ground [m]
 * @param ref_wind Wind speed at reference height [m/s]
 * @param ref_altitude Reference height [m] (typically 10m)
 * @param exponent Power law exponent α (default: 1/7 for unstable)
 * @return Wind speed at altitude [m/s]
 */
template <typename Scalar>
Scalar power_law(const Scalar& altitude,
                 double ref_wind,
                 double ref_altitude = 10.0,
                 double exponent = 1.0/7.0);

// ============================================================================
// Logarithmic Wind Profile
// ============================================================================

/**
 * @brief Logarithmic wind profile (neutral atmospheric boundary layer)
 * 
 * V(h) = (u* / κ) * ln((h - d) / z_0)
 * 
 * Where:
 * - u* = friction velocity [m/s]
 * - κ = von Kármán constant (≈ 0.41)
 * - d = zero-plane displacement [m] (for vegetation/urban areas)
 * - z_0 = roughness length [m]
 * 
 * @tparam Scalar double or casadi::MX
 * @param altitude Altitude above ground [m]
 * @param friction_velocity Surface friction velocity u* [m/s]
 * @param roughness_length Surface roughness z_0 [m]
 * @param displacement Zero-plane displacement d [m] (default: 0)
 * @return Wind speed at altitude [m/s]
 */
template <typename Scalar>
Scalar logarithmic(const Scalar& altitude,
                   double friction_velocity,
                   double roughness_length,
                   double displacement = 0.0);

// ============================================================================
// Common Roughness Lengths
// ============================================================================

namespace roughness {
    inline constexpr double OPEN_WATER = 0.0002;    // [m]
    inline constexpr double OPEN_TERRAIN = 0.03;    // [m] - grass, few trees
    inline constexpr double RURAL = 0.1;            // [m] - scattered buildings
    inline constexpr double SUBURBAN = 0.5;         // [m] - low-rise urban
    inline constexpr double URBAN = 1.0;            // [m] - city center
} // namespace roughness

} // namespace vulcan::wind_shear
```

---

### Component 5: Dryden Turbulence Model

#### [NEW] [DrydenTurbulence.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/wind/DrydenTurbulence.hpp)

Dryden turbulence model using forming filters with rational transfer functions.

```cpp
#pragma once

#include <vulcan/wind/WindTypes.hpp>

namespace vulcan::dryden {

// ============================================================================
// Power Spectral Density Functions
// ============================================================================

/**
 * @brief Dryden longitudinal PSD
 * 
 * Φ_u(Ω) = σ_u² * (2L_u/π) * [1 / (1 + (L_u·Ω)²)]
 * 
 * @tparam Scalar double or casadi::MX
 * @param omega Spatial frequency Ω [rad/m]
 * @param sigma_u RMS turbulence intensity [m/s]
 * @param L_u Scale length [m]
 * @return PSD value [m²/s²/(rad/m)]
 */
template <typename Scalar>
Scalar psd_longitudinal(const Scalar& omega, double sigma_u, double L_u);

/**
 * @brief Dryden lateral/vertical PSD
 * 
 * Φ_v(Ω) = σ_v² * (L_v/π) * [(1 + 3(L_v·Ω)²) / (1 + (L_v·Ω)²)²]
 * 
 * Same form for vertical component with w subscripts.
 */
template <typename Scalar>
Scalar psd_lateral(const Scalar& omega, double sigma, double L);

// ============================================================================
// Forming Filter (State-Space Representation)  
// ============================================================================

/**
 * @brief Dryden forming filter state
 * 
 * Tracks internal filter states for time-domain simulation.
 * One state per filter (2 for longitudinal, 3 for lateral/vertical).
 */
template <typename Scalar>
struct FilterState {
    Scalar x_u;        ///< Longitudinal filter state
    Scalar x_v1;       ///< Lateral filter state 1
    Scalar x_v2;       ///< Lateral filter state 2
    Scalar x_w1;       ///< Vertical filter state 1
    Scalar x_w2;       ///< Vertical filter state 2
};

/**
 * @brief Compute forming filter coefficients
 * 
 * Discretizes the continuous-time forming filters for given
 * airspeed and time step.
 * 
 * @param params Turbulence parameters (σ and L values)
 * @param airspeed True airspeed V [m/s]
 * @param dt Time step [s]
 * @return Discretized filter coefficients
 */
struct FilterCoeffs {
    // Longitudinal (first-order filter)
    double a_u, b_u, c_u;
    
    // Lateral (second-order filter)  
    double a_v1, a_v2, b_v1, b_v2, c_v;
    
    // Vertical (second-order filter, same structure as lateral)
    double a_w1, a_w2, b_w1, b_w2, c_w;
};

FilterCoeffs compute_filter_coeffs(const wind::TurbulenceParams& params,
                                   double airspeed,
                                   double dt);

/**
 * @brief Step the Dryden forming filter
 * 
 * Updates filter state and computes gust velocities given white noise input.
 * 
 * @tparam Scalar double or casadi::MX
 * @param state Current filter state (updated in-place)
 * @param coeffs Pre-computed filter coefficients
 * @param noise_u White noise input for longitudinal channel
 * @param noise_v White noise input for lateral channel
 * @param noise_w White noise input for vertical channel
 * @return Gust velocity (u_g, v_g, w_g)
 */
template <typename Scalar>
wind::GustVelocity<Scalar> step(FilterState<Scalar>& state,
                                 const FilterCoeffs& coeffs,
                                 const Scalar& noise_u,
                                 const Scalar& noise_v,
                                 const Scalar& noise_w);

/**
 * @brief Initialize filter state to zero
 */
template <typename Scalar>
FilterState<Scalar> init_state();

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * @brief Compute all filter coefficients for MIL-spec conditions
 */
FilterCoeffs mil_spec_coeffs(double altitude,
                             wind::TurbulenceSeverity severity,
                             double airspeed,
                             double dt);

} // namespace vulcan::dryden
```

---

### Component 6: von Kármán Turbulence Model

#### [NEW] [VonKarmanTurbulence.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/wind/VonKarmanTurbulence.hpp)

von Kármán turbulence with approximated forming filters (more accurate than Dryden).

```cpp
#pragma once

#include <vulcan/wind/WindTypes.hpp>

namespace vulcan::von_karman {

// ============================================================================
// Power Spectral Density Functions
// ============================================================================

/**
 * @brief von Kármán longitudinal PSD
 * 
 * Φ_u(Ω) = σ_u² * (2L_u/π) * [1 / (1 + (1.339·L_u·Ω)²)^(5/6)]
 * 
 * Note: The irrational exponent (5/6) requires approximation for filter design.
 * 
 * @tparam Scalar double or casadi::MX
 * @param omega Spatial frequency Ω [rad/m]
 * @param sigma_u RMS turbulence intensity [m/s]
 * @param L_u Scale length [m]
 * @return PSD value
 */
template <typename Scalar>
Scalar psd_longitudinal(const Scalar& omega, double sigma_u, double L_u);

/**
 * @brief von Kármán lateral/vertical PSD
 * 
 * Φ_v(Ω) = σ_v² * (L_v/π) * [(1 + (8/3)(1.339·L_v·Ω)²) / (1 + (1.339·L_v·Ω)²)^(11/6)]
 */
template <typename Scalar>
Scalar psd_lateral(const Scalar& omega, double sigma, double L);

// ============================================================================
// Approximate Forming Filters
// ============================================================================

/**
 * @brief von Kármán filter state (higher order than Dryden)
 * 
 * Uses rational polynomial approximation of the irrational transfer function.
 * Third-order approximation provides good accuracy.
 */
template <typename Scalar>
struct FilterState {
    // Longitudinal (3rd order approximation)
    Scalar x_u[3];
    
    // Lateral (4th order approximation)
    Scalar x_v[4];
    
    // Vertical (4th order approximation)  
    Scalar x_w[4];
};

/**
 * @brief von Kármán forming filter coefficients
 * 
 * Uses rational polynomial approximation to the irrational transfer functions.
 * Coefficients are computed using Padé approximation or optimal fitting.
 */
struct FilterCoeffs {
    // State matrices for 3rd/4th order discrete systems
    // A_x: state transition, B_x: input, C_x: output, D_x: feedthrough
    double A_u[3][3], B_u[3], C_u[3], D_u;
    double A_v[4][4], B_v[4], C_v[4], D_v;
    double A_w[4][4], B_w[4], C_w[4], D_w;
};

FilterCoeffs compute_filter_coeffs(const wind::TurbulenceParams& params,
                                   double airspeed,
                                   double dt);

/**
 * @brief Step the von Kármán forming filter
 */
template <typename Scalar>
wind::GustVelocity<Scalar> step(FilterState<Scalar>& state,
                                 const FilterCoeffs& coeffs,
                                 const Scalar& noise_u,
                                 const Scalar& noise_v,
                                 const Scalar& noise_w);

/**
 * @brief Initialize filter state
 */
template <typename Scalar>
FilterState<Scalar> init_state();

} // namespace vulcan::von_karman
```

---

### Component 7: Tests

#### [NEW] [tests/wind/test_constant_wind.cpp](file:///home/tanged/sources/temp/vulcan/tests/wind/test_constant_wind.cpp)

```cpp
// Tests for ConstantWind and WindTypes
// - WindVector construction and accessors
// - Speed/direction calculations
// - Numeric and symbolic evaluation
```

#### [NEW] [tests/wind/test_wind_shear.cpp](file:///home/tanged/sources/temp/vulcan/tests/wind/test_wind_shear.cpp)

```cpp
// Tests for WindShear profiles
// - Linear shear at boundary conditions
// - Power law profile against reference values
// - Logarithmic profile with different roughness
// - Symbolic gradient computation (dV/dh)
```

#### [NEW] [tests/wind/test_dryden.cpp](file:///home/tanged/sources/temp/vulcan/tests/wind/test_dryden.cpp)

```cpp
// Tests for Dryden turbulence model
// - PSD function values against analytical
// - Filter coefficient computation
// - Filter step produces correct variance (statistical test)
// - MIL-spec parameter lookup
// - Symbolic filter step (graph generation)
```

#### [NEW] [tests/wind/test_von_karman.cpp](file:///home/tanged/sources/temp/vulcan/tests/wind/test_von_karman.cpp)

```cpp
// Tests for von Kármán turbulence model
// - PSD function values
// - Approximate filter accuracy vs ideal PSD
// - Statistical properties of output
```

---

### Component 8: CMake Updates

#### [MODIFY] [tests/CMakeLists.txt](file:///home/tanged/sources/temp/vulcan/tests/CMakeLists.txt)

Add wind test subdirectory:
```cmake
add_subdirectory(wind)
```

#### [NEW] [tests/wind/CMakeLists.txt](file:///home/tanged/sources/temp/vulcan/tests/wind/CMakeLists.txt)

```cmake
vulcan_add_test(test_constant_wind test_constant_wind.cpp)
vulcan_add_test(test_wind_shear test_wind_shear.cpp)
vulcan_add_test(test_dryden test_dryden.cpp)
vulcan_add_test(test_von_karman test_von_karman.cpp)
```

---

### Component 9: Example Application

#### [NEW] [examples/wind_simulation.cpp](file:///home/tanged/sources/temp/vulcan/examples/wind_simulation.cpp)

Demonstration of wind models:
- Constant wind + shear profile overlay
- Dryden turbulence time series generation
- Visualization of PSD vs theoretical curve
- Combined wind field for trajectory simulation

---

## Implementation Order

1. **WindTypes.hpp** - Common types (no dependencies)
2. **ConstantWind.hpp** + tests - Simple baseline
3. **WindShear.hpp** + tests - Altitude-dependent profiles
4. **DrydenTurbulence.hpp** + tests - Forming filters
5. **VonKarmanTurbulence.hpp** + tests - Higher-order approximation
6. **Example application**

---

## Verification Plan

### Automated Tests

All wind model tests will be integrated into the standard test suite:

```bash
# Full CI verification (includes wind tests)
./scripts/ci.sh

# Run only wind tests
cd build && ctest -R wind --output-on-failure
```

**Test Matrix:**

| Test File | Coverage |
|-----------|----------|
| `test_constant_wind.cpp` | WindVector construction, speed/direction math, NED↔speed-direction conversions, symbolic evaluation |
| `test_wind_shear.cpp` | Linear, power-law, logarithmic profiles; boundary conditions; symbolic gradients |
| `test_dryden.cpp` | PSD vs analytical; filter coefficients; statistical variance check (N=10000 samples); MIL-spec params |
| `test_von_karman.cpp` | PSD vs analytical; filter approximation error; statistical tests |

### Numeric Validation

Cross-reference implementations against:
- **MIL-F-8785C** - Reference parameter tables
- **MATLAB Aerospace Blockset** - Dryden/von Kármán reference implementations
- **NASA TN D-5882** - Original Dryden spectral formulation

### Statistical Verification

For turbulence models, verify:
1. **Zero mean**: `E[u_g] ≈ 0` over 10,000+ samples
2. **Correct variance**: `Var[u_g] ≈ σ_u²` within 5%
3. **PSD shape**: FFT of generated time series matches theoretical PSD

### Symbolic Verification

Each model must pass symbolic graph generation:
```cpp
auto alt = janus::sym("altitude");
auto wind = wind_shear::power_law(alt, 10.0, 10.0);
janus::Function f("wind_profile", {alt}, {wind});
// Verify gradient computation
auto dwind_dh = janus::jacobian(wind, alt);
```

### Manual Verification

After implementation, user can visually verify:

```bash
cd build && ./examples/wind_simulation
```

This will generate:
1. Plot of wind shear profile vs altitude
2. Time series of turbulence output
3. PSD of generated turbulence vs theoretical curve
