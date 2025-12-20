# Vulcan Wind Models User Guide

This guide covers the wind modeling utilities in Vulcan, providing atmospheric wind profiles and turbulence models for flight simulation and trajectory optimization.

## Quick Start

```cpp
#include <vulcan/wind/WindShear.hpp>
#include <vulcan/wind/DrydenTurbulence.hpp>

using namespace vulcan;

// Create a power-law wind shear profile
double alt = 100.0;  // meters
double wind = wind_shear::power_law(alt, 10.0, 10.0);  // 10 m/s at 10m ref

// Get MIL-spec turbulence parameters
auto params = wind::mil_spec_params(500.0, wind::TurbulenceSeverity::Moderate);
// params.sigma_u, params.L_u, etc.
```

## Wind Types

### WindVector - 3D Wind in NED Frame

```cpp
#include <vulcan/wind/WindTypes.hpp>

wind::WindVector<double> w{.north = 10.0, .east = 5.0, .down = 0.0};

double speed = w.speed();              // Total magnitude [m/s]
double horiz = w.horizontal_speed();   // Horizontal component
double dir = w.direction_from();       // Meteorological convention [rad]
```

### Turbulence Parameters (MIL-F-8785C)

```cpp
// Get MIL-spec parameters for given altitude and severity
auto params = wind::mil_spec_params(
    altitude,                           // AGL [m]
    wind::TurbulenceSeverity::Moderate  // Light, Moderate, or Severe
);

// Access σ (intensity) and L (scale length) for each axis
std::cout << "σ_u = " << params.sigma_u << " m/s\n";
std::cout << "L_u = " << params.L_u << " m\n";
```

## Constant Wind Field

Simple uniform wind - useful as baseline and for testing.

```cpp
#include <vulcan/wind/ConstantWind.hpp>

// From NED components
auto w = constant_wind::from_ned(10.0, 5.0, 0.0);

// From speed and direction (meteorological convention)
auto w = constant_wind::from_speed_direction(
    12.0,           // speed [m/s]
    M_PI / 4.0      // direction FROM [rad], clockwise from North
);
```

## Wind Shear Profiles

Altitude-dependent wind models for boundary layer effects.

### Power-Law Profile (Most Common)

```cpp
#include <vulcan/wind/WindShear.hpp>

// V(h) = V_ref * (h / h_ref)^α
double wind = wind_shear::power_law(
    altitude,       // Current altitude [m]
    10.0,           // Reference wind at h_ref [m/s]
    10.0,           // Reference height h_ref [m]
    wind_shear::exponent::NEUTRAL  // α = 0.25
);

// Available exponents:
// - exponent::UNSTABLE = 1/7 (daytime, strong heating)
// - exponent::NEUTRAL  = 1/4 (overcast, moderate wind)
// - exponent::STABLE   = 1/3 (nighttime, light wind)
```

### Logarithmic Profile (Boundary Layer Theory)

```cpp
// V(h) = (u* / κ) * ln((h - d) / z₀)
double wind = wind_shear::logarithmic(
    altitude,           // [m]
    0.5,                // friction velocity u* [m/s]
    roughness::OPEN_TERRAIN,  // z₀ = 0.03 m
    0.0                 // displacement height d [m]
);

// Available roughness lengths:
// - roughness::OPEN_WATER   = 0.0002 m
// - roughness::OPEN_TERRAIN = 0.03 m
// - roughness::RURAL        = 0.1 m
// - roughness::SUBURBAN     = 0.5 m
// - roughness::URBAN        = 1.0 m
```

### Linear Shear

```cpp
// V(h) = V_base + shear_rate * (h - h_base)
double wind = wind_shear::linear(
    altitude,           // Current altitude [m]
    10.0,               // Base wind [m/s]
    0.0,                // Base altitude [m]
    0.01                // Shear rate [1/s]
);
```

## Dryden Turbulence Model

The Dryden model uses **rational transfer functions** to filter white noise into realistic turbulence. Easier to implement than von Kármán.

### Power Spectral Density

```cpp
#include <vulcan/wind/DrydenTurbulence.hpp>

// Longitudinal PSD: Φ_u(Ω) = σ²·(2L/π) / [1 + (L·Ω)²]
double psd = dryden::psd_longitudinal(omega, sigma_u, L_u);

// Lateral/vertical PSD (different shape)
double psd_v = dryden::psd_lateral(omega, sigma_v, L_v);
```

### Forming Filter (Time-Domain Simulation)

```cpp
// 1. Initialize filter state
auto state = dryden::init_state<double>();

// 2. Compute filter coefficients for given conditions
auto coeffs = dryden::mil_spec_coeffs(
    altitude,                           // AGL [m]
    wind::TurbulenceSeverity::Moderate, // severity
    100.0,                              // airspeed [m/s]
    0.01                                // time step [s]
);

// 3. Step filter with white noise input
std::mt19937 rng(12345);
std::normal_distribution<double> noise(0.0, 1.0);

for (int i = 0; i < 1000; ++i) {
    auto gust = dryden::step(
        state, coeffs,
        noise(rng),  // longitudinal noise
        noise(rng),  // lateral noise  
        noise(rng)   // vertical noise
    );
    // Use gust.u_g, gust.v_g, gust.w_g
}
```

## von Kármán Turbulence Model

More accurate spectral match to atmospheric data, but uses **irrational exponents** (5/6, 11/6) requiring higher-order filter approximation.

### Power Spectral Density

```cpp
#include <vulcan/wind/VonKarmanTurbulence.hpp>

// Φ_u(Ω) = σ²·(2L/π) / [1 + (1.339·L·Ω)²]^(5/6)
double psd = von_karman::psd_longitudinal(omega, sigma_u, L_u);
```

### Forming Filter

Same API as Dryden, but uses 3rd/4th order filters internally:

```cpp
auto state = von_karman::init_state<double>();
auto coeffs = von_karman::mil_spec_coeffs(altitude, severity, airspeed, dt);

auto gust = von_karman::step(state, coeffs, noise_u, noise_v, noise_w);
```

## Symbolic Computation

All wind functions work with `janus::SymbolicScalar` for gradient-based optimization:

```cpp
using Scalar = janus::SymbolicScalar;

Scalar alt = janus::sym("altitude");
Scalar wind = wind_shear::power_law(alt, 10.0, 10.0);

// Compute gradient dV/dh symbolically
auto dv_dh = janus::jacobian(wind, alt);

// Create CasADi function for optimization
janus::Function f("wind_profile", {alt}, {wind, dv_dh});
```

### Optimization Example

Optimize aircraft climb profile considering wind shear:

```cpp
janus::Opti opti;

auto alt1 = opti.variable(500.0);  // First waypoint
auto alt2 = opti.variable(1000.0); // Second waypoint

// Objective: minimize fuel cost (function of headwind)
auto cost = mission_cost(alt1, alt2, ref_wind);
opti.minimize(cost);

// Constraints
opti.subject_to(alt1 >= 200.0);
opti.subject_to(alt2 >= alt1 + 100.0);

auto solution = opti.solve();
```

## Graph Visualization

Export computational graphs as interactive HTML:

```cpp
auto alt = janus::sym("altitude");
auto wind = wind_shear::power_law(alt, 10.0, 10.0, wind_shear::exponent::NEUTRAL);

janus::export_graph_html(wind, "graph_wind_profile", "Wind_Profile");
```

> [!TIP]
> **Interactive Examples** - Explore the computational graphs:
> - [🌬️ Power-Law Wind Profile](../examples/graph_wind_profile.html)

## API Reference

### WindTypes.hpp

| Type/Function | Description |
|---------------|-------------|
| `WindVector<Scalar>` | 3D wind in NED frame |
| `GustVelocity<Scalar>` | Turbulent gusts in body frame (u_g, v_g, w_g) |
| `TurbulenceParams<Scalar>` | σ and L values for each axis |
| `TurbulenceSeverity` | Enum: Light, Moderate, Severe |
| `mil_spec_params(alt, severity)` | MIL-F-8785C parameter lookup |

### ConstantWind.hpp

| Function | Description |
|----------|-------------|
| `from_ned(n, e, d)` | WindVector from NED components |
| `from_speed_direction(spd, dir)` | WindVector from speed and meteorological direction |

### WindShear.hpp

| Function | Description |
|----------|-------------|
| `linear(alt, base, base_alt, rate)` | Linear profile |
| `power_law(alt, ref, ref_alt, exp)` | Power-law profile |
| `logarithmic(alt, u_star, z0, d)` | Logarithmic profile |
| `friction_velocity_from_ref(...)` | Compute u* from reference wind |

### DrydenTurbulence.hpp

| Function | Description |
|----------|-------------|
| `psd_longitudinal(ω, σ, L)` | Longitudinal PSD |
| `psd_lateral(ω, σ, L)` | Lateral/vertical PSD |
| `init_state<Scalar>()` | Initialize filter state |
| `compute_filter_coeffs(params, V, dt)` | Discretize filters |
| `step(state, coeffs, n_u, n_v, n_w)` | Step filter, return gusts |
| `mil_spec_coeffs(alt, sev, V, dt)` | Convenience: params + discretize |

### VonKarmanTurbulence.hpp

| Function | Description |
|----------|-------------|
| `psd_longitudinal(ω, σ, L)` | von Kármán longitudinal PSD |
| `psd_lateral(ω, σ, L)` | von Kármán lateral/vertical PSD |
| `init_state<Scalar>()` | Initialize higher-order filter state |
| `compute_filter_coeffs(params, V, dt)` | Discretize 3rd/4th order filters |
| `step(state, coeffs, n_u, n_v, n_w)` | Step filter, return gusts |

## Example: Wind Optimization Demo

See `examples/wind/wind_optimization.cpp` for a complete demonstration including:
- Wind shear profile survey
- Fuel cost optimization with `janus::Opti`
- MIL-spec turbulence parameters
- Interactive graph export

```bash
# Build and run the demo
./scripts/build.sh
./build/examples/wind_optimization
```

The demo outputs:

```
Optimal climb profile:
  Waypoint 1: 1200.0m
  Waypoint 2: 1300.0m
  Total cost: 3204.38

Improvement over baseline: 1.03%
```
