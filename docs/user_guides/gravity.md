# Vulcan Gravity Models User Guide

This guide covers the gravity models in Vulcan, from simple point mass to spherical harmonic expansions, all with full symbolic compatibility for trajectory optimization.

## Quick Start

```cpp
#include <vulcan/gravity/Gravity.hpp>

using namespace vulcan;
using namespace vulcan::gravity;

// Position in ECEF [m]
Vec3<double> r;
r << 7000000.0, 0.0, 0.0;  // 7000 km from center

// Point mass gravity
auto g_simple = point_mass::acceleration(r);

// J2 oblate Earth (recommended for LEO/MEO)
auto g_j2 = j2::acceleration(r);

// Higher fidelity with J2-J4
auto g_j2j4 = j2j4::acceleration(r);
```

## Coordinate Convention

All gravity functions use the **ECEF (Earth-Centered, Earth-Fixed)** frame:

- **Position `r`**: Vector from Earth's center to your point, in meters
- **Acceleration `g`**: Points **toward Earth's center** (opposite to `r`)

### Direction from Potential

The gravitational acceleration is the **negative gradient of the potential**:

```
g = -∇U
```

| Quantity | Sign | Physical Meaning |
|----------|------|------------------|
| Potential `U` | Negative | Lower (more negative) = more bound |
| Acceleration `g` | Toward center | Always points "downhill" in potential |

### Example

```cpp
Vec3<double> r;
r << 7000000.0, 0.0, 0.0;  // On +X axis, 7000 km from center

auto g = j2::acceleration(r);
// g ≈ [-7.9, 0, 0] m/s²  ← Points toward center (negative X)

// For equations of motion: r̈ = g
// The acceleration IS the gravity, already correctly directed
```

> [!TIP]
> You don't need to compute `-∇U` yourself—the `acceleration()` functions return the correctly-directed vector.

## Available Models

| Model | Fidelity | Use Case | Function |
|-------|----------|----------|----------|
| **Point Mass** | Low | Quick estimates | `point_mass::acceleration()` |
| **J2** | Medium | LEO/MEO, 6-DOF sim | `j2::acceleration()` |
| **J2-J4** | High | Precise propagation | `j2j4::acceleration()` |
| **Spherical Harmonics** | Very High | High-precision | `spherical_harmonics::acceleration()` |

## Point Mass Gravity

The simplest model treating Earth as a uniform sphere:

```cpp
// g = -μ/r³ · r
auto g = point_mass::acceleration(r);

// Just the magnitude
double g_mag = point_mass::acceleration_magnitude(janus::norm(r));

// Gravitational potential (negative)
auto U = point_mass::potential(r);  // U = -μ/r
```

## J2 Oblate Earth Model

Accounts for Earth's equatorial bulge—the dominant perturbation for satellites:

```cpp
// With default Earth parameters
auto g = j2::acceleration(r);

// With custom parameters (e.g., for Moon)
auto g = j2::acceleration(r, mu_moon, J2_moon, R_eq_moon);

// Potential
auto U = j2::potential(r);
```

### J2 Effects

- **Equatorial gravity**: Slightly reduced (~9.78 m/s² vs ~9.83 at poles)
- **Nodal precession**: Orbital plane rotates over time
- **Apsidal precession**: Orbit ellipse rotates

## J2-J4 Higher Fidelity

Includes J3 (pear shape) and J4 (higher-order oblateness):

```cpp
auto g = j2j4::acceleration(r);

// With explicit coefficients
auto g = j2j4::acceleration(r, mu, J2, J3, J4, R_eq);

// Potential with all zonal harmonics
auto U = j2j4::potential(r);
```

> [!TIP]
> Use J2-J4 for orbit propagation requiring accuracy better than ~1 km over 24 hours.

## Spherical Harmonics

General expansion for highest fidelity:

```cpp
// With default Earth coefficients (J2, J3, J4)
auto g = spherical_harmonics::acceleration(r);

// With custom coefficients
spherical_harmonics::GravityCoefficients coeffs(10);  // Up to degree 10
// Set coeffs.C[n][m] and coeffs.S[n][m] as needed
auto g = spherical_harmonics::acceleration(r, coeffs);
```

### Legendre Polynomials

The implementation includes associated Legendre polynomial computation:

```cpp
// P_2,0(sin φ) = (3sin²φ - 1) / 2
double P20 = spherical_harmonics::legendre_Pnm(2, 0, sin_lat);

// Works symbolically too
auto P20_sym = spherical_harmonics::legendre_Pnm(2, 0, lat_symbolic);
```

## Symbolic Computation

All gravity models work with `janus::SymbolicScalar` for optimization:

```cpp
using Scalar = janus::SymbolicScalar;

Scalar x = janus::sym("x");
Scalar y = janus::sym("y");
Scalar z = janus::sym("z");

Vec3<Scalar> r;
r << x, y, z;

// Symbolic gravity expressions
auto g = j2::acceleration(r);

// Create CasADi function for optimization
janus::Function f("gravity", {x, y, z}, {g(0), g(1), g(2)});

// Evaluate numerically
auto result = f({7000000.0, 0.0, 0.0});
```

### Gravity Gradient (Jacobian)

Compute the Jacobian for orbit propagation and control:

```cpp
auto g = j2::acceleration(r);

// 3x3 gravity gradient tensor
auto J = janus::jacobian({g(0), g(1), g(2)}, {x, y, z});

// Create function for numerical evaluation
janus::Function f_grad("gravity_gradient", {x, y, z},
    {J(0,0), J(0,1), J(0,2),
     J(1,0), J(1,1), J(1,2),
     J(2,0), J(2,1), J(2,2)});
```

## Graph Visualization

Export computational graphs as interactive HTML:

```cpp
auto g = j2::acceleration(r);

// Export to HTML
janus::export_graph_html(g(0), "gravity_graph", "J2_Gravity_X");
```

> [!TIP]
> **Interactive Examples** - Explore the computational graphs:
> - [🔍 Point Mass Gravity](examples/graph_point_mass.html)
> - [🔍 J2 Gravity](examples/graph_j2_gravity.html)
> - [🔍 J2 Potential](examples/graph_j2_potential.html)
> - [🔍 Gravity Gradient](examples/graph_gravity_gradient.html)

## Physical Constants

All constants are in `vulcan::constants::earth`:

```cpp
constants::earth::mu     // 3.986004418e14 m³/s²
constants::earth::R_eq   // 6378137.0 m
constants::earth::R_pol  // 6356752.3 m
constants::earth::J2     // 1.08263e-3
constants::earth::J3     // -2.54e-6
constants::earth::J4     // -1.61e-6
```

## API Reference

### point_mass namespace

| Function | Description |
|----------|-------------|
| `acceleration(r)` | Gravitational acceleration g = -μ/r³ · r |
| `acceleration_magnitude(r_mag)` | \|g\| = μ/r² |
| `potential(r)` | U = -μ/r |

### j2 namespace

| Function | Description |
|----------|-------------|
| `acceleration(r, [mu, J2, R_eq])` | J2 oblate Earth gravity |
| `potential(r, [mu, J2, R_eq])` | J2 gravitational potential |

### j2j4 namespace

| Function | Description |
|----------|-------------|
| `acceleration(r, [mu, J2, J3, J4, R_eq])` | J2-J4 zonal harmonics |
| `potential(r, [mu, J2, J3, J4, R_eq])` | J2-J4 potential |

### spherical_harmonics namespace

| Function | Description |
|----------|-------------|
| `acceleration(r, [coeffs])` | General spherical harmonic acceleration |
| `potential(r, [coeffs])` | General spherical harmonic potential |
| `legendre_Pnm(n, m, x)` | Associated Legendre polynomial P_nm(x) |
| `GravityCoefficients(n_max)` | Coefficient container with default Earth values |

## Example: Complete Demo

See `examples/gravity/gravity_demo.cpp` for a comprehensive demonstration.

```bash
# Build and run
./scripts/build.sh
./build/examples/gravity_demo
```

Sample output:
```
=== Vulcan Gravity Models Demo ===

--- 1. Point Mass Gravity ---
ISS gravity (point mass): [-8.691234, 0.000000, 0.000000] m/s²
  Magnitude: 8.691234 m/s²

--- 2. J2 Gravity (Oblate Earth) ---
J2 effect: Polar gravity is 0.52% stronger than equatorial
...
```
