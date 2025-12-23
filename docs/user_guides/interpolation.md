# Table Interpolation Module

The `vulcan::Table1D`, `vulcan::TableND`, `vulcan::ScatteredTable1D`, and `vulcan::ScatteredTableND` classes provide lookup table interpolation for aerodynamic coefficients, atmospheric properties, and other data. They wrap Janus's interpolators for symbolic compatibility.

## Overview

| Class | Use Case |
|-------|----------|
| `Table1D` | Uniform 1D grid (altitude → temperature) |
| `TableND` | Uniform N-D grid (Mach, alpha → Cd) |
| `ScatteredTable1D` | Non-uniform 1D points (wind tunnel data) |
| `ScatteredTableND` | Unstructured N-D point cloud (CFD samples) |

## Basic Usage

### Include
```cpp
#include <vulcan/vulcan.hpp>
// or
#include <vulcan/core/TableInterpolator.hpp>
```

### 1D Gridded Interpolation

```cpp
using namespace vulcan;

// Altitude vs temperature table
janus::NumericVector alt(5), temp(5);
alt << 0, 5, 10, 15, 20;        // km
temp << 288, 256, 223, 217, 217; // K

Table1D atm_table(alt, temp);

// Query at any altitude
double T = atm_table(7.5);  // Interpolated temperature
```

### N-D Gridded Interpolation

```cpp
// 2D aero table: Mach × alpha → CL
janus::NumericVector mach(3), alpha(4);
mach << 0.6, 0.8, 1.0;
alpha << 0, 5, 10, 15;

// Values in Fortran order (first dimension varies fastest)
janus::NumericVector cl(12);
cl << 0.0, 0.1, 0.2,   // alpha=0, all Mach
     0.5, 0.6, 0.7,   // alpha=5, all Mach
     1.0, 1.1, 1.2,   // alpha=10
     1.4, 1.5, 1.6;   // alpha=15

TableND aero_table({mach, alpha}, cl);

// Query at (Mach=0.7, alpha=8°)
janus::NumericVector query(2);
query << 0.7, 8.0;
double lift = aero_table(query);
```

## Scattered Interpolation

For non-uniformly spaced or unstructured data (wind tunnel tests, CFD):

### 1D Scattered

```cpp
// Non-uniform test points
janus::NumericVector x(8), y(8);
x << 0.0, 0.3, 0.7, 1.2, 2.0, 2.8, 3.5, 4.0;
y << 0.0, 0.29, 0.64, 1.0, 0.91, 0.33, -0.35, -0.75;

ScatteredTable1D table(x, y);

double val = table(1.5);  // RBF-based interpolation
std::cout << "RMS error: " << table.reconstruction_error() << "\n";
```

### N-D Scattered

```cpp
// 2D wind tunnel data: (Mach, alpha) → CL
int n_tests = 20;
janus::NumericMatrix points(n_tests, 2);
janus::NumericVector values(n_tests);
// ... fill with measurements ...

ScatteredTableND table(points, values, 30);  // 30 = grid resolution

janus::NumericVector query(2);
query << 0.8, 5.0;
double cl = table(query);
```

### RBF Kernel Selection

```cpp
// Available kernels (default: ThinPlateSpline)
ScatteredTable1D tps(x, y, 50, RBFKernel::ThinPlateSpline);
ScatteredTable1D mq(x, y, 50, RBFKernel::Multiquadric);
ScatteredTable1D gauss(x, y, 50, RBFKernel::Gaussian);
ScatteredTable1D linear(x, y, 50, RBFKernel::Linear);
ScatteredTable1D cubic(x, y, 50, RBFKernel::Cubic);
```

## Symbolic Optimization

All table classes work with `janus::SymbolicScalar` for trajectory optimization:

```cpp
Table1D drag_table(mach_pts, cd_pts);

// Symbolic query
janus::SymbolicScalar mach_sym = janus::sym("mach");
janus::SymbolicScalar cd_sym = drag_table(mach_sym);

// Build differentiable function
janus::Function cd_fn("cd_lookup", {mach_sym}, {cd_sym});

// Use in optimization objective
janus::Opti opti;
auto mach = opti.variable();
opti.minimize(cd_fn(mach)[0]);
```

### Gradients Through Tables

```cpp
ScatteredTable1D table(x, y, 100);

auto x_sym = janus::sym("x");
auto y_sym = table(x_sym);
auto grad = janus::jacobian(y_sym, x_sym);

janus::Function df("df", {x_sym}, {grad});
double slope = df.eval(1.5)(0, 0);  // dy/dx at x=1.5
```

## Interpolation Methods

For gridded tables, specify the method:

```cpp
Table1D linear(x, y, janus::InterpolationMethod::Linear);   // C0 (default)
Table1D bspline(x, y, janus::InterpolationMethod::BSpline); // C2 smooth
```

| Method | Smoothness | Symbolic | Use Case |
|--------|------------|----------|----------|
| Linear | C0 | ✅ | Fast, general |
| BSpline | C2 | ✅ | Optimization (smooth gradients) |
| Hermite | C1 | ❌ | Animation, trajectories |
| Nearest | None | ❌ | Fast lookup |

## Data Layout: Fortran Order

For N-D tables, values must be in Fortran (column-major) order:

```
Grid: x = [0, 1], y = [0, 1, 2]

Values layout:
  y=0   y=1   y=2
x=0  a     b     c    →  values = [a, d, b, e, c, f]
x=1  d     e     f            (x varies fastest)
```

## See Also

- [Janus Interpolation Guide](../../reference/janus/docs/user_guides/interpolation.md)
- [Aerodynamics](aerodynamics.md) - Using tables for aero coefficients
- [Atmosphere](atmosphere.md) - Atmospheric property tables
