# Aerodynamics Module

The `vulcan::aero` module provides fundamental aerodynamic calculations for flight mechanics and trajectory optimization. It is designed to work seamlessly with both numeric types (`double`) and symbolic types (`janus::SymbolicScalar`) for automatic differentiation.

## Overview

The module provides:
- **Fundamental Quantities**: Dynamic pressure, Mach number, Reynolds number, Airspeed.
- **Aerodynamic Angles**: Angle of attack (α) and sideslip angle (β).
- **Combined State**: A struct `AeroState` containing all key aerodynamic properties.

## Basic Usage

### Include
```cpp
#include <vulcan/vulcan.hpp>
// or
#include <vulcan/aerodynamics/Aerodynamics.hpp>
```

### Computing Individual Properties

All functions are templated on `Scalar` type.

```cpp
double rho = 1.225;   // kg/m^3
double V = 250.0;     // m/s
double a = 340.3;     // m/s (speed of sound)
double mu = 1.789e-5; // Pa*s (dynamic viscosity)
double L = 2.0;       // m (characteristic length)

// Dynamic Pressure (q)
double q = vulcan::aero::dynamic_pressure(rho, V); // 38281.25 Pa

// Mach Number (M)
double M = vulcan::aero::mach_number(V, a); // 0.734

// Reynolds Number (Re)
double Re = vulcan::aero::reynolds_number(rho, V, L, mu); // 3.42e7
```

### Computing Aerodynamic Angles

`aero_angles` computes angle of attack (α) and sideslip (β) from the body-frame velocity vector.

```cpp
vulcan::Vec3<double> v_body;
v_body << 200.0, 10.0, 20.0; // [vx, vy, vz]

auto angles = vulcan::aero::aero_angles(v_body);
double alpha = angles(0); // ~0.1 rad (atan2(vz, vx))
double beta = angles(1);  // ~0.05 rad (asin(vy / V))
```

### Combined Aerodynamic State

For efficiency, especially in trajectory optimization, use `aero_state` to compute everything at once. This effectively combines the atmosphere model outputs with flight conditions.

```cpp
// 1. Get atmosphere state (e.g. at 10km)
auto atm = vulcan::ussa1976::state(10000.0);

// 2. Define flight condition
vulcan::Vec3<double> v_body_10km;
v_body_10km << 250.0, 0.0, 0.0;
double char_len = 5.0;

// 3. Compute Aero State
auto state = vulcan::aero::aero_state(
    atm.density,
    atm.speed_of_sound,
    atm.dynamic_viscosity,
    v_body_10km,
    char_len
);

// Access results
std::cout << "Mach: " << state.mach << std::endl;
std::cout << "Dyn P: " << state.dynamic_pressure << std::endl;
```

## Symbolic Optimization

The entire module is Janus-compatible. You can use `janus::SymbolicScalar` to build computational graphs for optimization problems.

```cpp
// Define symbolic optimization variables
janus::SymbolicScalar alt = janus::sym("h");
janus::SymbolicScalar velocity = janus::sym("v");

// Compute atmosphere symbolically
auto atm = vulcan::ussa1976::state(alt);

// Compute dynamic pressure symbolically
auto q = vulcan::aero::dynamic_pressure(atm.density, velocity);

// 'q' is now a symbolic expression graph that depends on h and v
```

Note: `aero_angles` uses `janus::atan2` and `janus::asin`, which are fully differentiable and safe for symbolic use.
