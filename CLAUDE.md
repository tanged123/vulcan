# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Vulcan is an aerospace engineering utilities library built on the Janus framework. It provides model-agnostic simulation utilities (coordinate systems, atmospheric models, rotations, time systems) that work in both **numeric** (`double`) and **symbolic** (`casadi::MX`) computational modes.

## Build Commands

All scripts auto-enter the Nix environment if not already in one.

```bash
./scripts/build.sh          # Build the project
./scripts/build.sh --clean  # Clean rebuild
./scripts/test.sh           # Run all tests
./scripts/run_examples.sh   # Run all examples
./scripts/run_example.sh <name>  # Run a single example (e.g., ./scripts/run_example.sh euler_demo)
./scripts/coverage.sh       # Generate coverage report
./scripts/verify.sh         # Full verification (build + tests + examples)
nix develop                 # Enter dev environment manually
```

### Running a Single Test

```bash
# After building, run a specific test executable
./build/tests/test_coordinates
./build/tests/test_atmosphere
./build/tests/test_rotations
./build/tests/test_time

# Or use ctest with a filter
ctest --test-dir build -R "TestName"
```

## Critical Janus Compatibility Rules

**These rules are INVIOLABLE. Breaking them will cause symbolic mode to fail.**

### 1. Template-First Design
All engineering models MUST be templated on a generic `Scalar` type:
```cpp
// CORRECT
template <typename Scalar>
Scalar my_function(const Scalar& x);

// WRONG - breaks symbolic mode
double my_function(double x);
```

### 2. Math Dispatch - Use `janus::` Namespace
ALWAYS use `janus::` math functions instead of `std::`:
```cpp
// CORRECT
janus::sin(x), janus::pow(x, 2), janus::sqrt(x), janus::exp(x)

// WRONG - breaks symbolic tracing
std::sin(x), std::pow(x, 2)
```

### 3. Branching - Use `janus::where()`, NEVER `if/else`
```cpp
// CORRECT
Scalar result = janus::where(x > 0, x, -x);

// WRONG - MX cannot evaluate to bool
if (x > 0) { result = x; } else { result = -x; }
```

For multi-way branching, use `janus::select()`:
```cpp
Scalar cd = janus::select(
    {mach < 0.3, mach < 0.8, mach < 1.2},
    {Scalar(0.02), Scalar(0.025), Scalar(0.05)},
    Scalar(0.03));  // default
```

### 4. Loop Bounds - Must Be Structural
Loop bounds must be compile-time constants, not optimization variables:
```cpp
// CORRECT - structural bound
for (int i = 0; i < N; ++i) { ... }

// WRONG - dynamic bound breaks symbolic mode
while (error > tolerance) { ... }
```

## Type Aliases

Use Janus native types from `<janus/core/JanusTypes.hpp>`:
```cpp
janus::Vec3<Scalar>   // 3D vector
janus::Mat3<Scalar>   // 3x3 matrix
janus::VecX<Scalar>   // Dynamic vector
janus::MatX<Scalar>   // Dynamic matrix
```

## Architecture

- **`include/vulcan/core/`**: Constants, units, types, table interpolation
- **`include/vulcan/atmosphere/`**: US Standard Atmosphere 1976, exponential models
- **`include/vulcan/coordinates/`**: ECI/ECEF/NED/Body frames, geodetic conversions, transforms
- **`include/vulcan/rotations/`**: DCM utilities, all 12 Euler sequences, axis-angle, interpolation
- **`include/vulcan/time/`**: Julian dates, GPS/UTC/TAI time scales, leap seconds, epochs
- **`tests/`**: GoogleTest suite mirroring include structure
- **`examples/`**: Usage examples organized by module

The main umbrella header is `<vulcan/vulcan.hpp>` which includes all modules.

## Testing Requirements

Every new function must have tests for BOTH numeric AND symbolic modes:
```cpp
TEST(MyModule, NumericTest) {
    double result = my_function(1.0);
    EXPECT_NEAR(result, expected, tolerance);
}

TEST(MyModule, SymbolicTest) {
    auto x = janus::sym("x");
    auto result = my_function(x);
    // Verify symbolic expression works
    double evaluated = janus::eval(result, {{"x", 1.0}});
    EXPECT_NEAR(evaluated, expected, tolerance);
}
```

## Documentation

- **`docs/patterns/janus_usage_guide.md`**: Comprehensive Janus API reference
- **`docs/implementation_plans/vulcan_bootstrap_guide.md`**: Vulcan Bootstrap Guide
- **`docs/user_guides/`**: Walkthroughs for each module
- **`docs/implementation_plans/`**: Phase-based implementation plans
- **`docs/saved_work/`**: Context preservation for agent handover

## Key Janus APIs Available (Do Not Reimplement)

- Math: `sin`, `cos`, `pow`, `exp`, `log`, `sqrt`, `abs`, `atan2`
- Linear algebra: `dot`, `cross`, `norm`, `normalize`, `inv`, `det`, `trace`
- Quaternions: `janus::Quaternion<Scalar>` with full algebra
- Branching: `where`, `select`, `clamp`, `min`, `max`
- Calculus: `jacobian`, `gradient`, `hessian`
- Interpolation: `interp1`, `interp_nd`
- Optimization: `janus::Opti` for NLP problems
