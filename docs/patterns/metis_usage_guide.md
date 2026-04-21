# Metis Usage Guide

> **Purpose**: This document provides comprehensive guidance for AI agents working with the Metis library. It documents all available modules, best practices, and conventions to prevent duplication of work.

---

## Table of Contents

1. [Documentation Resources](#documentation-resources)
2. [Best Practices](#best-practices)
3. [Type System](#type-system)
4. [Module Reference](#module-reference)
   - [Core Layer](#core-layer-metiscore)
   - [Math Layer](#math-layer-metismath)
   - [Optimization Layer](#optimization-layer-metisoptimization)
5. [Existing User Guides](#existing-user-guides)

---

## Documentation Resources

### Official Documentation

- **Doxygen API Docs**: [https://tanged123.github.io/metis/index.html](https://tanged123.github.io/metis/index.html)
- **Design Overview**: `docs/design_overview.md`
- **User Guides**: `docs/user_guides/` (10 comprehensive guides)

### Key Documentation Files

| File | Description |
|------|-------------|
| `docs/design_overview.md` | High-level architecture and design philosophy |
| `docs/user_guides/symbolic_computing.md` | Symbolic mode guide |
| `docs/user_guides/numeric_computing.md` | Numeric mode guide |
| `docs/user_guides/optimization.md` | Opti interface usage |
| `docs/user_guides/interpolation.md` | Interpolation utilities |
| `docs/user_guides/graph_visualization.md` | Computational graph visualization |
| `docs/user_guides/sparsity.md` | Sparsity pattern analysis |
| `docs/user_guides/collocation.md` | Direct collocation for trajectory optimization |
| `docs/user_guides/multiple_shooting.md` | Multiple shooting transcription |
| `docs/user_guides/transcription_methods.md` | Overview of trajectory optimization methods |

---

## Best Practices

### 1. Template-First Design (MANDATORY)

All physics/math functions **must** be templated on `Scalar`:

```cpp
// ✅ CORRECT
template <typename Scalar>
Scalar my_function(const Scalar& x) { ... }

// ❌ WRONG - Breaks symbolic mode
double my_function(double x) { ... }
```

### 2. Math Dispatch - Use `metis::` Namespace

**Always** use Metis math functions instead of `std::`:

```cpp
// ✅ CORRECT
metis::sin(x), metis::pow(x, 2), metis::sqrt(x), metis::exp(x)

// ❌ WRONG - Uses std, breaks symbolic tracing
std::sin(x), std::pow(x, 2), std::sqrt(x), std::exp(x)
```

### 3. Branching - Use `metis::where()`, Never `if/else`

```cpp
// ✅ CORRECT
Scalar result = metis::where(x > 0, x, -x);

// ❌ WRONG - MX can't evaluate to bool
if (x > 0) { result = x; } else { result = -x; }
```

For multi-way branching, use `metis::select()`:
```cpp
Scalar cd = metis::select(
    {mach < 0.3, mach < 0.8, mach < 1.2},
    {Scalar(0.02), Scalar(0.025), Scalar(0.05)},
    Scalar(0.03));  // default
```

### 4. Loops - Structural Bounds Only

```cpp
// ✅ CORRECT - Structural bound (known at trace time)
for (int i = 0; i < N; ++i) { ... }

// ❌ WRONG - Dynamic bound (breaks symbolic mode)
while (error > tolerance) { ... }
```

### 5. Type Aliases - Use Metis Native Types

```cpp
#include <metis/core/MetisTypes.hpp>

// Prefer these over raw Eigen types
metis::Vec3<Scalar>   // 3D vector
metis::Mat3<Scalar>   // 3x3 matrix
metis::VecX<Scalar>   // Dynamic vector
metis::MatX<Scalar>   // Dynamic matrix
```

---

## Type System

### Backend Types

| Type | Numeric Mode | Symbolic Mode |
|------|--------------|---------------|
| **Scalar** | `double` | `casadi::MX` |
| **Matrix** | `Eigen::MatrixXd` | `Eigen::Matrix<casadi::MX>` |
| **Vector** | `Eigen::VectorXd` | `Eigen::Matrix<casadi::MX, Dynamic, 1>` |

### Metis Type Aliases (`metis/core/MetisTypes.hpp`)

```cpp
// Symbolic types (for graph building)
metis::SymbolicScalar   // casadi::MX
metis::SymbolicMatrix   // Eigen::Matrix<casadi::MX, Dynamic, Dynamic>
metis::SymbolicVector   // Eigen::Matrix<casadi::MX, Dynamic, 1>

// Numeric types (for evaluation)
metis::NumericMatrix    // Eigen::MatrixXd
metis::NumericVector    // Eigen::VectorXd

// Fixed-size templated types
metis::Vec2<T>, metis::Vec3<T>, metis::Vec4<T>
metis::Mat2<T>, metis::Mat3<T>, metis::Mat4<T>
metis::VecX<T>, metis::MatX<T>, metis::RowVecX<T>

// Sparse types (numeric only)
metis::SparseMatrix     // Eigen::SparseMatrix<double>
metis::SparseTriplet    // Eigen::Triplet<double>
```

### Symbolic Variable Creation

```cpp
// Scalar
auto x = metis::sym("x");

// Matrix/Vector (returns MX)
auto M = metis::sym("M", rows, cols);

// Vector as SymbolicVector (Eigen container)
auto v = metis::sym_vector("v", size);
auto v = metis::sym_vec("v", size);  // Alias

// Get both SymbolicVector and underlying MX
auto [vec, mx] = metis::sym_vec_pair("state", 3);
```

### Conversion Utilities

```cpp
metis::to_mx(eigen_matrix)     // Eigen → CasADi MX
metis::to_eigen(casadi_mx)     // CasADi MX → Eigen
metis::as_mx(symbolic_vector)  // SymbolicVector → single MX
metis::as_vector(casadi_mx)    // MX → SymbolicVector
```

---

## Module Reference

### Core Layer (`metis/core/`)

| File | Description | Key Functions |
|------|-------------|---------------|
| `MetisTypes.hpp` | Type system and aliases | `sym()`, `sym_vec()`, `to_mx()`, `to_eigen()` |
| `MetisConcepts.hpp` | C++20 concepts for type constraints | `ScalarType`, `NumericScalar`, `SymbolicScalar` |
| `MetisError.hpp` | Custom exception types | `InvalidArgument`, `IntegrationError`, `InterpolationError` |
| `MetisIO.hpp` | I/O and graph visualization | `eval()`, `print()`, `to_dot()`, `graphviz()` |
| `Function.hpp` | CasADi function wrapper | `Function` class for compiled symbolic functions |
| `Sparsity.hpp` | Sparsity pattern analysis | `SparsityPattern`, `analyze_sparsity()`, `visualize_sparsity()` |

#### Key APIs in Core

```cpp
// Evaluation
double result = metis::eval(symbolic_expr, {{"x", 5.0}});
metis::NumericMatrix result = metis::eval(symbolic_matrix, args);

// Printing
metis::print(symbolic_expr);            // Pretty print
metis::print_expression(symbolic_expr); // Show graph structure

// Graph visualization
std::string dot = metis::to_dot(expr, "my_graph");
metis::graphviz(expr, "output.pdf");

// Function compilation
metis::Function f("f", {x}, {result});
auto output = f({5.0});
```

---

### Math Layer (`metis/math/`)

#### Arithmetic (`Arithmetic.hpp`)

Basic math operations with dual-backend support:

| Function | Description |
|----------|-------------|
| `abs(x)` | Absolute value |
| `sqrt(x)` | Square root |
| `pow(base, exp)` | Power function |
| `exp(x)` | Exponential e^x |
| `log(x)` | Natural logarithm |
| `log10(x)` | Base-10 logarithm |
| `sinh(x)`, `cosh(x)`, `tanh(x)` | Hyperbolic functions |
| `floor(x)`, `ceil(x)` | Rounding |
| `remainder(x, y)` | Modulo operation |
| `sign(x)` | Sign function |
| `copysign(mag, sgn)` | Copy sign |

All functions have scalar and matrix overloads.

---

#### Trigonometry (`Trig.hpp`)

| Function | Description |
|----------|-------------|
| `sin(x)`, `cos(x)`, `tan(x)` | Basic trig |
| `asin(x)`, `acos(x)`, `atan(x)` | Inverse trig |
| `atan2(y, x)` | Two-argument arctangent |

---

#### Logic (`Logic.hpp`)

Symbolic-compatible branching and comparisons:

| Function | Description |
|----------|-------------|
| `where(cond, if_true, if_false)` | Ternary selection |
| `select(conditions, values, default)` | Multi-way selection |
| `min(a, b)`, `max(a, b)` | Min/max |
| `clamp(val, low, high)` | Clamping |
| `lt(a, b)`, `gt(a, b)`, `le(a, b)`, `ge(a, b)`, `eq(a, b)`, `neq(a, b)` | Element-wise comparisons |
| `sigmoid_blend(x, val_low, val_high, sharpness)` | Smooth blending |
| `blend(x, val_low, val_high, center, sharpness)` | Centered blending |

---

#### Calculus (`Calculus.hpp`)

| Function | Description |
|----------|-------------|
| `gradient(f, x)` | Gradient of scalar function |
| `jacobian(f, x)` | Jacobian matrix |
| `hessian(f, x)` | Hessian matrix |
| `trapz(y, x)` | Trapezoidal integration |
| `diff(y, x)` | Numerical differentiation |
| `cumsum(y)` | Cumulative sum |

---

#### Autodiff (`AutoDiff.hpp`)

| Function | Description |
|----------|-------------|
| `jacobian({outputs}, {inputs})` | Multi-input/output Jacobian |
| `hessian(output, inputs)` | Hessian matrix |

---

#### Linear Algebra (`Linalg.hpp`)

| Function | Description |
|----------|-------------|
| `dot(a, b)` | Dot product |
| `cross(a, b)` | Cross product (3D) |
| `norm(v)` | Euclidean norm |
| `normalize(v)` | Unit vector |
| `trace(M)` | Matrix trace |
| `det(M)` | Determinant |
| `inv(M)` | Matrix inverse |
| `reshape(M, rows, cols)` | Reshape matrix |
| `eye<Scalar>(n)` | Identity matrix |
| `zeros<Scalar>(rows, cols)` | Zero matrix |
| `ones<Scalar>(rows, cols)` | Ones matrix |
| `block_diag(blocks)` | Block diagonal matrix |

---

#### Interpolation (`Interpolate.hpp`)

| Function | Description |
|----------|-------------|
| `interp1(x, xp, fp)` | 1D linear interpolation |
| `interp1(x, table, method)` | 1D with method selection |
| `interp_nd(point, table)` | N-dimensional interpolation |
| `InterpolatedModel::fit(x, y)` | Fit interpolation model |

Methods: `"linear"`, `"cubic"`, `"monotonic"`

---

#### Spacing (`Spacing.hpp`)

| Function | Description |
|----------|-------------|
| `linspace(start, end, n)` | Linear spacing |
| `cosspace(start, end, n)` | Cosine spacing (clusters at ends) |
| `sinspace(start, end, n)` | Sine spacing |
| `logspace(start, end, n)` | Logarithmic spacing |
| `geomspace(start, end, n)` | Geometric spacing |

---

#### Integration (`Integrate.hpp`)

ODE integration:

| Function | Description |
|----------|-------------|
| `quad(f, a, b)` | Definite integral (Gauss-Kronrod) |
| `solve_ivp(dynamics, x0, t_span)` | ODE IVP solver |

Integration methods: `"RK4"`, `"CVODES"`, `"IDAS"`

---

#### Discrete Integration (`IntegrateDiscrete.hpp`)

Symbolic-friendly discrete integration:

| Function | Description |
|----------|-------------|
| `integrate_discrete(y, x, method)` | Discrete integration |

Methods: `"rectangular"`, `"trapezoidal"`, `"simpson"`, `"cubic"`, `"squared_curvature"`

---

#### Finite Difference (`FiniteDifference.hpp`)

| Function | Description |
|----------|-------------|
| `finite_difference_coefficients(derivative_order, x, x0)` | FD coefficients |

---

#### Rotations (`Rotations.hpp`)

| Function | Description |
|----------|-------------|
| `rotation_2d(angle)` | 2D rotation matrix |
| `rotation_x(angle)` | Rotation about X axis |
| `rotation_y(angle)` | Rotation about Y axis |
| `rotation_z(angle)` | Rotation about Z axis |

---

#### Quaternion (`Quaternion.hpp`)

Full quaternion algebra class:

| Method | Description |
|--------|-------------|
| `Quaternion(w, x, y, z)` | Constructor |
| `Quaternion::from_axis_angle(axis, angle)` | From axis-angle |
| `Quaternion::from_dcm(dcm)` | From rotation matrix |
| `q.inverse()`, `q.conjugate()` | Inverse/conjugate |
| `q.normalize()` | Normalize |
| `q.rotate(v)` | Rotate vector |
| `q.to_dcm()` | Convert to DCM |
| `q.to_euler_zyx()` | Convert to Euler angles |
| `slerp(q1, q2, t)` | Spherical interpolation |

---

#### Root Finding (`RootFinding.hpp`)

| Function | Description |
|----------|-------------|
| `bisection(f, a, b)` | Bisection method |
| `newton(f, df, x0)` | Newton-Raphson |
| `find_root(f, x0)` | Automatic method selection |

---

#### Surrogate Models (`SurrogateModel.hpp`)

Smooth approximations:

| Function | Description |
|----------|-------------|
| `softmax(x, sharpness)` | Smooth maximum |
| `softmin(x, sharpness)` | Smooth minimum |
| `softabs(x, sharpness)` | Smooth absolute value |
| `sigmoid(x)` | Logistic sigmoid |
| `tanh_blend(x, low, high, center, width)` | Tanh blending |

---

### Optimization Layer (`metis/optimization/`)

#### Opti (`Opti.hpp`)

Main optimization interface:

```cpp
metis::Opti opti;

// Variables
auto x = opti.variable(1.0);                    // Scalar
auto v = opti.variable(3, 0.0);                 // Vector
auto x = opti.variable(1.0, {.category = "Wing", .freeze = true});

// Parameters (fixed between solves)
auto p = opti.parameter(5.0);

// Objective
opti.minimize(cost_function);
opti.maximize(profit_function);

// Constraints
opti.subject_to(x >= 0);
opti.subject_to(x * x + y * y <= 1);
opti.subject_to_bounds(x, lower, upper);  // Box constraints

// Solve
auto sol = opti.solve();
double x_val = sol.value(x);
metis::NumericVector v_val = sol.value_vector(v);
```

#### Solver Configuration (`OptiOptions.hpp`)

```cpp
opti.set_max_iterations(1000);
opti.set_tolerance(1e-8);
opti.set_option("print_level", 0);  // IPOPT options
opti.set_solver(metis::Solver::IPOPT);

// Check solver availability
if (metis::solver_available(metis::Solver::SNOPT)) { ... }
```

#### Solution (`OptiSol.hpp`)

```cpp
auto sol = opti.solve();

sol.value(x);           // Get scalar value
sol.value_vector(v);    // Get vector value
sol.objective_value();  // Optimal objective
sol.stats();            // Solver statistics
```

#### Parametric Sweep (`OptiSweep.hpp`)

```cpp
metis::OptiSweep sweep(opti);
auto results = sweep.run(parameter, values);
```

#### Trajectory Optimization

**Collocation** (`Collocation.hpp`):
```cpp
metis::DirectCollocation problem(N_segments, dynamics, t0, tf);
problem.set_state_bounds(lower, upper);
problem.set_control_bounds(u_lower, u_upper);
problem.set_boundary_conditions(x0, xf);
auto [sol, t, x, u] = problem.solve();
```

**Multiple Shooting** (`MultiShooting.hpp`):
```cpp
metis::MultipleShooting problem(N_segments, dynamics, t0, tf);
problem.set_initial_guess(t_init, x_init, u_init);
auto [sol, t, x, u] = problem.solve();
```

---

## Existing User Guides

Before implementing new functionality, check these existing guides to avoid duplication:

| Guide | File | Topics Covered |
|-------|------|----------------|
| Numeric Computing | `docs/user_guides/numeric_computing.md` | Numeric mode, evaluation, standard execution |
| Symbolic Computing | `docs/user_guides/symbolic_computing.md` | Symbolic mode, graph building, MX operations |
| Optimization | `docs/user_guides/optimization.md` | Opti interface, constraints, solving |
| Interpolation | `docs/user_guides/interpolation.md` | 1D/ND interp, methods, caching |
| Graph Visualization | `docs/user_guides/graph_visualization.md` | DOT export, Graphviz, debugging |
| Sparsity | `docs/user_guides/sparsity.md` | Sparsity patterns, Jacobian structure |
| Collocation | `docs/user_guides/collocation.md` | Direct collocation transcription |
| Multiple Shooting | `docs/user_guides/multiple_shooting.md` | Multiple shooting transcription |
| Transcription Methods | `docs/user_guides/transcription_methods.md` | Comparison of trajectory methods |
| Math Functions | `docs/user_guides/math_functions.md` | Overview of metis:: math functions |

---

## Pattern Documentation

- `docs/patterns/branching_logic.md` - Advanced `where()` and `select()` patterns
- `docs/patterns/loop_patterns.md` - Structural loops, while→for conversion
- (Additional patterns in `docs/patterns/`)

---

## Summary for Agents

### DO NOT Reimplement

The following functionality already exists in Metis:

- ✅ All basic math (`sin`, `cos`, `pow`, `exp`, `log`, `sqrt`, etc.)
- ✅ Linear algebra (`dot`, `cross`, `norm`, `inv`, `det`)
- ✅ Quaternion algebra and rotations
- ✅ Interpolation (1D, ND, multiple methods)
- ✅ ODE integration (`solve_ivp`, `quad`)
- ✅ Discrete integration (trapz, Simpson, etc.)
- ✅ Branching logic (`where`, `select`, `clamp`, `min`, `max`)
- ✅ Sparsity analysis
- ✅ Function compilation
- ✅ Optimization (`Opti`, IPOPT)
- ✅ Trajectory optimization (collocation, multiple shooting)

### When Building on Metis

1. **Import via** `#include <metis/metis.hpp>` (includes everything)
2. **Use Metis types** (`metis::Vec3<Scalar>`, `metis::SymbolicScalar`)
3. **Use Metis math** (`metis::sin`, not `std::sin`)
4. **Use Metis branching** (`metis::where`, not `if/else`)
5. **Template everything** on `Scalar`
6. **Test both modes** (numeric AND symbolic)
