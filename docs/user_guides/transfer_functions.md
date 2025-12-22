# Transfer Functions Module

The `vulcan::tf` module provides stateless transfer function utilities for linear systems and nonlinear elements. All functions are symbolic-compatible for trajectory optimization.

## Overview

| Header | Description |
|--------|-------------|
| `FirstOrder.hpp` | First-order lag filters |
| `SecondOrder.hpp` | Second-order systems (ωn, ζ) |
| `Discretize.hpp` | ZOH, Tustin, Euler discretization |
| `Nonlinear.hpp` | Rate limiting, saturation, deadband, hysteresis |

## Design Philosophy

**All functions are stateless.** State management is the responsibility of the calling framework (e.g., Icarus for simulation). This keeps Vulcan focused on pure math utilities.

## First-Order Systems

```cpp
#include <vulcan/transfer_functions/TransferFunctions.hpp>
using namespace vulcan::tf;

// Compute discretized coefficients
auto c = first_order<double>(0.1, 1.0, 0.01);  // tau, K, dt

// Stateless step (caller manages state)
double y = 0.0;
y = first_order_step(c, y, 1.0);  // returns next y

// Analytical response
double y_at_t = first_order_response(0.1, 1.0, 0.3);  // tau, K, t
```

## Second-Order Systems

```cpp
auto c = second_order<double>(25.0, 0.7, 1.0, 0.01);  // ωn, ζ, K, dt

Eigen::Vector2d x = Eigen::Vector2d::Zero();
x = second_order_step(c, x, 1.0);     // x = [y, ẏ]
double y = second_order_output(c, x, 1.0);

// Characteristics
auto [tr, ts, Mp] = second_order_characteristics(25.0, 0.7);
```

## Nonlinear Elements

```cpp
// Rate limiting
double next = rate_limit(current, commanded, rate_max, dt);

// Saturation
double sat = saturate(value, min, max);

// Deadband
double db = deadband(input, width);

// Hysteresis
double hyst = hysteresis(input, last_output, width);
```

## Discretization

```cpp
Eigen::Matrix2d A_c, A_d;
Eigen::Vector2d B_c, B_d;

discretize_zoh<2>(A_c, B_c, dt, A_d, B_d);      // Zero-order hold
discretize_tustin<2>(A_c, B_c, dt, A_d, B_d);  // Bilinear
discretize_euler<2>(A_c, B_c, dt, A_d, B_d);    // Forward Euler
```

## Symbolic Optimization

All functions work with `casadi::MX`:

```cpp
auto c = first_order<casadi::MX>(0.1, 1.0, 0.01);
casadi::MX x = casadi::MX::sym("x");
casadi::MX u = casadi::MX::sym("u");
casadi::MX x_next = first_order_step(c, x, u);
// x_next is a symbolic expression for use in janus::Opti
```
