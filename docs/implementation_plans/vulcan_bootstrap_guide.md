# Vulcan Repository Bootstrap Guide

> **Purpose**: This document provides comprehensive instructions for an AI agent to create the **Vulcan** engineering utilities repository. Vulcan contains model-agnostic aerospace simulation utilities (coordinate frames, rotations, atmospheric models, gravity models, earth models, etc.) that utilize the **Metis** math library as a dependency.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Understanding Metis's Dual Symbolic/Numeric Nature](#2-understanding-metiss-dual-symbolicnumeric-nature)
3. [Repository Structure](#3-repository-structure)
4. [Nix Setup](#4-nix-setup)
5. [CMake Configuration](#5-cmake-configuration)
6. [Scripts Directory](#6-scripts-directory)
7. [Documentation Standards](#7-documentation-standards)
8. [Examples Organization](#8-examples-organization)
9. [Testing Framework](#9-testing-framework)
10. [CI/CD Workflows](#10-cicd-workflows)
11. [Agent Rules (.cursorrules)](#11-agent-rules-cursorrules)
12. [Git Configuration](#12-git-configuration)
13. [Initial Module Roadmap](#13-initial-module-roadmap)
14. [Verification Checklist](#14-verification-checklist)

---

## 1. Project Overview

### What is Vulcan?

**Vulcan** is an engineering utilities library for aerospace simulation. Named after the Roman god of fire and forge (fitting for engineering/manufacturing), it provides:

- **Coordinate Frames & Geodetics**: Reference frames (ECI, ECEF, NED, Body), transformations, LLA↔ECEF, geodetic utilities, great circle/range-bearing calculations
- **Rotation Utilities**: DCM, Euler angles (all 12 sequences), axis-angle, quaternion conversions and composition
- **Atmospheric Models**: Standard atmosphere (US Standard 1976), NRLMSISE-00, exponential models
- **Wind Models**: Constant wind, wind shear profiles, turbulence (Dryden, von Kármán spectra), gust models
- **Earth/Gravity Models**: WGS84 ellipsoid, gravity models (J2, J4, spherical harmonics), magnetic field (dipole, IGRF)
- **Aerodynamic Utilities**: Dynamic pressure, Mach number, speed of sound, Reynolds number, AoA/sideslip from velocity vectors
- **Environmental Utilities**: Solar position, eclipse detection, illumination, albedo, thermal radiation
- **Time Systems**: Time conversions (UTC, TAI, TT, GPS, TDB), Julian dates, leap seconds
- **Sensor Noise Models**: Gaussian noise, random walk, bias instability, Markov processes, Allan variance
- **Physical Constants**: Consolidated library (standard gravity, gas constants, planetary data, WGS84 constants)
- **Unit Conversions**: SI, imperial, angular (deg/rad), and domain-specific conversions
- **Data I/O**: HDF5 telemetry logging, CSV export, standardized frame-by-frame data recording for 6-DOF simulations

### Key Design Principle: Metis Compatibility

All Vulcan utilities **MUST** be templated on a generic `Scalar` type to maintain compatibility with Metis's dual-backend system. This allows Vulcan models to work in both:

1. **Numeric Mode**: Fast execution with `double`
2. **Symbolic Mode**: Graph generation for optimization with `casadi::MX`

---

## 2. Understanding Metis's Dual Symbolic/Numeric Nature

> [!IMPORTANT]
> This section is **CRITICAL**. Every line of Vulcan code must respect these constraints.

### The Core Paradigm

Metis implements **Code Transformations** - write physics once, execute in two modes:

| Mode | Scalar Type | Matrix Type | Purpose |
|------|-------------|-------------|---------|
| **Numeric** | `double` | `Eigen::MatrixXd` | Fast simulation, real-time control |
| **Symbolic** | `casadi::MX` | `Eigen::Matrix<casadi::MX>` | Graph generation, optimization |

### The "Red Line" Rules

These rules are **INVIOLABLE**:

#### 1. Template-First Design
```cpp
// ✅ CORRECT: Templated on Scalar
template <typename Scalar>
Scalar compute_gravity(const Scalar& altitude, const Scalar& latitude);

// ❌ WRONG: Hardcoded types
double compute_gravity(double altitude, double latitude);
```

#### 2. Math Dispatch - Use `metis::` Namespace
```cpp
// ✅ CORRECT: Uses metis:: namespace
auto result = metis::sin(theta) * metis::pow(r, 2);

// ❌ WRONG: Uses std:: namespace (breaks symbolic mode)
auto result = std::sin(theta) * std::pow(r, 2);
```

#### 3. Branching - Use `metis::where()`, NEVER `if/else`
```cpp
// ✅ CORRECT: Symbolic-compatible branching
Scalar cd = metis::where(mach > 1.0, 0.5, 0.02);

// ❌ WRONG: Breaks symbolic graph (MX doesn't evaluate to bool)
Scalar cd;
if (mach > 1.0) {  // COMPILATION ERROR with MX!
    cd = 0.5;
} else {
    cd = 0.02;
}
```

#### 4. Loop Bounds - Must Be Structural (Not Scalar-Dependent)
```cpp
// ✅ CORRECT: Loop bound is a compile-time constant
for (int i = 0; i < NUM_TERMS; ++i) {
    result += coefficients[i] * metis::pow(x, i);
}

// ❌ WRONG: Loop bound depends on optimization variable
for (int i = 0; i < some_scalar_value; ++i) {  // FORBIDDEN!
    ...
}
```

### Metis Type Aliases (Use These!)

When using Metis types, prefer the native aliases:

```cpp
#include <metis/core/MetisTypes.hpp>

// Fixed-size vectors/matrices
metis::Vec2<Scalar>  // 2D column vector
metis::Vec3<Scalar>  // 3D column vector
metis::Vec4<Scalar>  // 4D column vector
metis::Mat2<Scalar>  // 2x2 matrix
metis::Mat3<Scalar>  // 3x3 matrix
metis::Mat4<Scalar>  // 4x4 matrix

// Dynamic types
metis::VecX<Scalar>  // Dynamic column vector
metis::MatX<Scalar>  // Dynamic matrix
metis::RowVecX<Scalar>  // Dynamic row vector
```

### Advanced Control Flow Patterns

Beyond the basic `metis::where()`, Vulcan implementations will need more sophisticated patterns for complex aerospace logic.

#### Multi-Way Branching with `metis::select()`

For switch-case style logic, use `metis::select()` instead of nested `where()`:

```cpp
// ❌ Hard to read with nested where()
Scalar cd = metis::where(mach < 0.3,
               Scalar(0.02),
               metis::where(mach < 0.8, 
                   Scalar(0.025),
                   metis::where(mach < 1.2, Scalar(0.05), Scalar(0.03))));

// ✅ Clean with select() - Conditions checked in order, first match wins
Scalar cd = metis::select(
    {mach < 0.3, mach < 0.8, mach < 1.2},    // conditions
    {Scalar(0.02), Scalar(0.025), Scalar(0.05)},  // values
    Scalar(0.03));  // default if none match
```

#### Complex Branch Calculations

When branches require multi-step calculations, use helper functions:

```cpp
// Helper functions for complex flow regimes
template <typename Scalar>
Scalar turbulent_skin_friction(const Scalar& re, const Scalar& mach) {
    auto cf = 0.074 / metis::pow(re, 0.2);
    auto compressibility = 1.0 + 0.144 * metis::pow(mach, 2.0);
    return cf * compressibility;
}

template <typename Scalar>
Scalar laminar_skin_friction(const Scalar& re) {
    return 1.328 / metis::sqrt(re);
}

// Use in branching - both branches are ALWAYS evaluated in symbolic mode!
template <typename Scalar>
Scalar skin_friction(const Scalar& re, const Scalar& mach) {
    return metis::where(re > 5e5,
                       turbulent_skin_friction(re, mach),
                       laminar_skin_friction(re));
}
```

> [!WARNING]
> Both branches of `metis::where()` are **always evaluated** in symbolic mode (that's how computational graphs work). The condition only selects which result to use. Avoid expensive computations in branches that might not be needed.

#### Compound Conditions

Combine multiple conditions using logical operators:

```cpp
template <typename Scalar>
Scalar apply_stall_correction(const Scalar& cl, const Scalar& alpha,
                               const Scalar& reynolds) {
    auto is_stalled = metis::abs(alpha) > 0.26;  // ~15 deg
    auto low_reynolds = reynolds < 1e5;
    
    // Combine conditions - both must be true
    return metis::where(is_stalled && low_reynolds,
                       cl * 0.7,   // Apply 30% reduction
                       cl);        // No correction
}
```

### Loop Patterns

#### The Structural vs Dynamic Distinction

| Loop Type | Numeric Mode | Symbolic Mode | Example |
|-----------|--------------|---------------|---------|
| **Structural** (bounds known at compile/trace time) | ✅ Works | ✅ Works | `for (int i = 0; i < 10; i++)` |
| **Dynamic** (bounds depend on runtime values) | ✅ Works | ❌ Fails | `while (error > tol)` |

#### Structural Loops (✅ Both Modes)

```cpp
// ✅ WORKS - Loop bound is structural (integer constant)
template <typename Scalar>
Scalar polynomial_eval(const Scalar& x, const std::vector<double>& coeffs) {
    Scalar result = 0.0;
    for (int i = 0; i < static_cast<int>(coeffs.size()); ++i) {
        result += coeffs[i] * metis::pow(x, static_cast<double>(i));
    }
    return result;
}
```

#### Converting While Loops to Fixed Iterations

```cpp
// ❌ DOESN'T WORK (symbolic) - condition is dynamic
while (error > tolerance) {
    x = update(x);
    error = compute_error(x);
}

// ✅ WORKS (both modes) - fixed iteration count
template <typename Scalar>
Scalar newton_iteration(const Scalar& x0, int max_iter) {
    Scalar x = x0;
    for (int i = 0; i < max_iter; ++i) {
        x = x - f(x) / df(x);  // Newton step
    }
    return x;
}
```

#### Replacing Break/Continue with Conditional Accumulation

```cpp
// ❌ DOESN'T WORK (symbolic) - break is dynamic
for (int i = 0; i < n; ++i) {
    if (values(i) > threshold) break;
    result += values(i);
}

// ✅ WORKS (both modes) - use where() for conditional logic
template <typename Scalar>
Scalar conditional_sum(const metis::VecX<Scalar>& values,
                       const Scalar& threshold) {
    Scalar sum = 0.0;
    auto exceeded = Scalar(0.0);  // Flag: have we exceeded threshold?
    
    for (int i = 0; i < values.size(); ++i) {
        auto should_add = (values(i) <= threshold) && (exceeded < 0.5);
        sum += metis::where(should_add, values(i), Scalar(0.0));
        exceeded = metis::where(values(i) > threshold, Scalar(1.0), exceeded);
    }
    return sum;
}
```

#### Hybrid Approach: Numeric Determines Structure, Symbolic Uses It

For simulations where iteration count depends on convergence:

```cpp
// Step 1: Run numeric simulation to determine iteration count
int determine_convergence_steps(double x0, double tolerance) {
    double x = x0, error = 1000.0;
    int steps = 0;
    while (error > tolerance && steps < 1000) {
        double x_new = physics_update(x);
        error = std::abs(x_new - x);
        x = x_new;
        ++steps;
    }
    return steps;
}

// Step 2: Use fixed count for symbolic version (can now differentiate!)
template <typename Scalar>
Scalar simulate_symbolic(const Scalar& x0, int n_steps) {
    Scalar x = x0;
    for (int i = 0; i < n_steps; ++i) {
        x = physics_update(x);
    }
    return x;
}
```

#### Aerospace Example: Gravity Harmonic Expansion

```cpp
template <typename Scalar>
Scalar gravity_potential(const Scalar& r, const Scalar& phi, int n_max) {
    Scalar potential = -mu / r;  // Point mass term
    
    // Harmonic expansion - structural loop (n_max is fixed at call time)
    for (int n = 2; n <= n_max; ++n) {
        for (int m = 0; m <= n; ++m) {
            auto P_nm = legendre(n, m, metis::sin(phi));
            auto factor = metis::pow(R_e / r, n);
            
            // Use where() for any conditional logic on symbolic values
            potential += factor * (C[n][m] * metis::cos(m * lambda)
                                 + S[n][m] * metis::sin(m * lambda)) * P_nm;
        }
    }
    return potential;
}
```

### Control Flow Quick Reference

| Pattern | C++ Standard | Metis Equivalent |
|---------|--------------|------------------|
| `if (x < 0) a else b` | `if/else` | `metis::where(x < 0, a, b)` |
| `switch (regime)` | `switch/case` | `metis::select({cond1, cond2}, {val1, val2}, default)` |
| `while (error > tol)` | `while` | Fixed iteration: `for (int i = 0; i < N; i++)` |
| `if (cond) break` | `break` | Conditional accumulation with `where()` |
| `if (cond) continue` | `continue` | Conditional update with `where()` |

### Namespace Organization (No Factories!)

> [!IMPORTANT]
> **Do NOT use factory patterns with virtual dispatch** in Vulcan. Runtime polymorphism breaks Metis symbolic tracing and prevents autodiff.

#### Why Factories Break Metis

```cpp
// ❌ BREAKS METIS - virtual calls can't be traced through
class AtmosphereModel {
    virtual double density(double alt) = 0;  // Virtual = no autodiff
};

std::unique_ptr<AtmosphereModel> model = AtmosphereFactory::create("standard");
model->density(altitude);  // Runtime dispatch breaks the graph!
```

#### The Correct Pattern: Namespace-Organized Free Functions

Instead of factories, use **explicit namespace selection**:

```cpp
// ✅ CORRECT - Fully traceable, zero overhead, autodiff works
vulcan::atmosphere::standard::density(altitude);
vulcan::atmosphere::exponential::density(altitude, scale_height);

vulcan::gravity::point_mass::acceleration(position);
vulcan::gravity::j2::acceleration(position, mu, J2, R_eq);

vulcan::wind::constant::velocity(position);
vulcan::wind::dryden::velocity(position, intensity, airspeed);
```

#### Namespace Hierarchy

```
vulcan::
├── atmosphere::
│   ├── standard::      density(), temperature(), pressure()
│   ├── exponential::   density(), scale_height()
│   └── nrlmsise::      density(), temperature() (if implemented)
├── gravity::
│   ├── point_mass::    acceleration(), potential()
│   ├── j2::            acceleration(), potential()
│   └── spherical::     acceleration(), potential() (takes n_max)
├── wind::
│   ├── constant::      velocity()
│   ├── shear::         velocity()
│   ├── dryden::        velocity(), update() (stateful)
│   └── vonkarman::     velocity(), update() (stateful)
├── coordinates::
│   ├── lla_to_ecef()
│   ├── ecef_to_lla()
│   ├── ecef_to_eci()
│   ├── ned_to_body()
│   └── geodetic::      latitude(), altitude(), range_bearing()
├── aero::
│   ├── mach()
│   ├── dynamic_pressure()
│   ├── reynolds()
│   ├── speed_of_sound()
│   └── angles::        alpha_beta_from_velocity()
├── time::
│   ├── utc_to_tai()
│   ├── tai_to_gps()
│   ├── julian_date()
│   └── leap_seconds()
├── noise::             (numeric-only, stateful RNG)
│   ├── gaussian::      sample()
│   ├── random_walk::   update()
│   └── markov::        update()
├── constants::
│   ├── earth::         mu, R_eq, J2, omega, ...
│   ├── wgs84::         a, f, e2, ...
│   └── physics::       c, G, k_B, ...
└── units::
    ├── deg_to_rad()
    ├── rad_to_deg()
    ├── ft_to_m()
    └── ...
```

#### Configuration via Parameters, Not Factories

Instead of `factory.create("spherical", n_terms)`, use parameters:

```cpp
// Gravity model complexity via parameter
auto accel = vulcan::gravity::spherical::acceleration(r, lat, lon, 20);  // 20 terms

// Atmosphere model with custom scale height
auto rho = vulcan::atmosphere::exponential::density(alt, 8500.0);  // scale height
```

#### When Runtime Selection is Needed (Numeric-Only)

If a simulation **truly needs** runtime model selection (config-file-driven), use explicit `switch` dispatch in **numeric mode only**:

```cpp
// Numeric-only dispatch (cannot be differentiated)
enum class AtmosphereType { Standard, Exponential };

double get_density(AtmosphereType type, double altitude) {
    switch (type) {
        case AtmosphereType::Standard:
            return vulcan::atmosphere::standard::density(altitude);
        case AtmosphereType::Exponential:
            return vulcan::atmosphere::exponential::density(altitude, 8500.0);
    }
}
```

For **optimization**, the user must select the model at trace-time (before building the computational graph).

---

## 3. Repository Structure

Create the following directory structure:

```plaintext
vulcan/
├── .agent/
│   └── workflows/          # Agent automation workflows
├── .github/
│   ├── ISSUE_TEMPLATE/     # Bug report / feature request templates
│   ├── hooks/              # Git hooks
│   │   └── pre-commit
│   └── workflows/          # GitHub Actions CI
│       ├── ci.yml
│       ├── coverage.yml
│       ├── docs.yml
│       └── format.yml
├── docs/
│   ├── design_overview.md  # High-level architecture
│   ├── design_reviews/     # Design decisions and reviews
│   ├── images/             # Documentation images
│   ├── implementation_plans/  # Phase-based implementation plans
│   ├── patterns/           # Code pattern documentation
│   ├── saved_work/         # Context preservation for agents
│   └── user_guides/        # User-facing documentation
│       ├── coordinate_systems.md
│       ├── atmosphere_models.md
│       ├── gravity_models.md
│       └── ...
├── examples/
│   ├── CMakeLists.txt
│   ├── intro/              # Getting started examples
│   ├── atmosphere/         # Atmospheric model examples
│   ├── gravity/            # Gravity model examples
│   ├── coordinates/        # Coordinate frame examples
│   └── integration/        # Integration with Metis optimization
├── include/
│   └── vulcan/
│       ├── vulcan.hpp      # Main entry point (includes all)
│       ├── core/           # Core types and concepts
│       │   ├── VulcanConcepts.hpp
│       │   ├── VulcanTypes.hpp
│       │   ├── Constants.hpp        # Physical constants library
│       │   └── Units.hpp            # Unit conversion utilities
│       ├── coordinates/    # Coordinate systems & geodetics
│       │   ├── Frames.hpp           # ECI, ECEF, NED, Body definitions
│       │   ├── Transformations.hpp  # Frame transformation matrices
│       │   ├── Geodetic.hpp         # LLA↔ECEF, geodetic utilities
│       │   ├── GreatCircle.hpp      # Range, bearing, waypoints
│       │   └── WGS84.hpp            # WGS84 ellipsoid constants
│       ├── atmosphere/     # Atmospheric models
│       │   ├── StandardAtmosphere.hpp
│       │   ├── Exponential.hpp
│       │   └── NRLMSISE.hpp
│       ├── wind/           # Wind models
│       │   ├── ConstantWind.hpp
│       │   ├── WindShear.hpp
│       │   └── Turbulence.hpp       # Dryden, von Kármán
│       ├── gravity/        # Gravity models
│       │   ├── PointMass.hpp
│       │   ├── J2.hpp
│       │   └── Spherical.hpp
│       ├── aero/           # Aerodynamic utilities
│       │   ├── FlowProperties.hpp   # Mach, dynamic pressure, Re
│       │   └── Angles.hpp           # AoA, sideslip from velocity
│       ├── environment/    # Environmental utilities
│       │   ├── SolarPosition.hpp
│       │   ├── Eclipse.hpp
│       │   ├── Illumination.hpp
│       │   └── MagneticField.hpp
│       ├── noise/          # Sensor noise models
│       │   ├── Gaussian.hpp
│       │   ├── RandomWalk.hpp
│       │   ├── BiasInstability.hpp
│       │   └── Markov.hpp
│       ├── time/           # Time systems
│       │   ├── TimeConversions.hpp  # UTC, TAI, TT, GPS, TDB
│       │   ├── JulianDate.hpp
│       │   └── LeapSeconds.hpp
│       └── io/             # Data I/O utilities
│           ├── HDF5Writer.hpp       # Frame-by-frame HDF5 logging
│           ├── HDF5Reader.hpp       # HDF5 data loading
│           ├── CSVExport.hpp        # HDF5 to CSV conversion
│           └── TelemetrySchema.hpp  # Standard 6-DOF data schema
├── logs/                   # Build/test logs (gitignored)
├── scripts/
│   ├── build.sh
│   ├── clean.sh
│   ├── ci.sh
│   ├── coverage.sh
│   ├── dev.sh
│   ├── examples.sh
│   ├── install-hooks.sh
│   ├── run_examples.sh
│   ├── test.sh
│   └── verify.sh
├── tests/
│   ├── CMakeLists.txt
│   ├── atmosphere/
│   ├── coordinates/
│   ├── gravity/
│   └── environment/
├── .clang-format
├── .cursorrules
├── .gitignore
├── CMakeLists.txt
├── Doxyfile
├── flake.lock
├── flake.nix
├── LICENSE
└── README.md
```

---

## 4. Nix Setup

### flake.nix

Create a Nix flake that brings in Metis as a dependency:

```nix
{
  description = "Vulcan: Aerospace Engineering Utilities";
  
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    treefmt-nix.url = "github:numtide/treefmt-nix";
    
    # Metis as a flake input
    metis = {
      url = "github:tanged123/metis";
      # Or for local development:
      # url = "path:/home/tanged/sources/metis";
    };
  };

  outputs = { self, nixpkgs, flake-utils, treefmt-nix, metis }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        stdenv = pkgs.llvmPackages_latest.stdenv;
        
        # Get metis package from input
        metisPackage = metis.packages.${system}.default;

        # Treefmt configuration
        treefmtEval = treefmt-nix.lib.evalModule pkgs {
          projectRootFile = "flake.nix";
          programs.nixfmt.enable = true;
          programs.clang-format.enable = true;
          programs.cmake-format.enable = true;
        };
      in
      {
        packages.default = stdenv.mkDerivation {
          pname = "vulcan";
          version = "0.1.0";
          src = ./.;

          nativeBuildInputs = [
            pkgs.cmake
            pkgs.ninja
            pkgs.pkg-config
          ];

          buildInputs = [
            pkgs.eigen
            pkgs.casadi
            pkgs.hdf5
            pkgs.highfive  # C++ HDF5 wrapper
            metisPackage
          ];

          cmakeFlags = [
            "-DENABLE_COVERAGE=OFF"
          ];
        };

        devShells.default = pkgs.mkShell.override { inherit stdenv; } {
          packages = with pkgs; [
            cmake
            ninja
            pkg-config
            eigen
            casadi
            hdf5
            highfive
            gtest
            clang-tools
            doxygen
            graphviz
            lcov
            llvmPackages_latest.llvm
          ] ++ [
            metisPackage
            treefmtEval.config.build.wrapper
          ];

          shellHook = ''
            export CMAKE_PREFIX_PATH=${pkgs.eigen}:${pkgs.casadi}:${pkgs.gtest}:${pkgs.hdf5}:${pkgs.highfive}:${metisPackage}
          '';
        };

        formatter = treefmtEval.config.build.wrapper;

        checks = {
          formatting = treefmtEval.config.build.check self;
        };
      }
    );
}
```

### Alternative: Local Metis Path

For development alongside Metis, use a local path input:

```nix
metis = {
  url = "path:/home/tanged/sources/metis";
};
```

---

## 5. CMake Configuration

### Root CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.20)
project(
  vulcan
  VERSION 0.1.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
find_package(Eigen3 3.4 REQUIRED)
find_package(casadi REQUIRED)
find_package(metis REQUIRED)  # Metis as external dependency
find_package(HDF5 REQUIRED COMPONENTS C CXX)
find_package(HighFive REQUIRED)  # C++ HDF5 wrapper

# --- Main Library ---
add_library(vulcan INTERFACE)
target_include_directories(
  vulcan INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                   $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# Link Dependencies
target_link_libraries(vulcan INTERFACE Eigen3::Eigen casadi metis::metis HighFive)

# --- Coverage ---
option(ENABLE_COVERAGE "Enable coverage reporting" OFF)
if(ENABLE_COVERAGE)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
    add_link_options(--coverage)
  else()
    message(WARNING "Coverage not supported for this compiler")
  endif()
endif()

# --- Install ---
include(GNUInstallDirs)

install(
  TARGETS vulcan
  EXPORT vulcanTargets
  INCLUDES
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
  EXPORT vulcanTargets
  FILE vulcanConfig.cmake
  NAMESPACE vulcan::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vulcan)

# --- Testing ---
enable_testing()
add_subdirectory(tests)

# --- Examples ---
add_subdirectory(examples)
```

### tests/CMakeLists.txt

```cmake
find_package(GTest REQUIRED)

# Core tests
add_executable(test_coordinates
    coordinates/test_frames.cpp
    coordinates/test_transformations.cpp
    coordinates/test_wgs84.cpp
)
target_link_libraries(test_coordinates PRIVATE vulcan GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(test_coordinates)

# Atmosphere tests
add_executable(test_atmosphere
    atmosphere/test_standard.cpp
    atmosphere/test_exponential.cpp
)
target_link_libraries(test_atmosphere PRIVATE vulcan GTest::gtest_main)
gtest_discover_tests(test_atmosphere)

# Gravity tests
add_executable(test_gravity
    gravity/test_point_mass.cpp
    gravity/test_j2.cpp
)
target_link_libraries(test_gravity PRIVATE vulcan GTest::gtest_main)
gtest_discover_tests(test_gravity)
```

### examples/CMakeLists.txt

```cmake
# =============================================================================
# Intro Examples
# =============================================================================
add_executable(getting_started intro/getting_started.cpp)
target_link_libraries(getting_started PRIVATE vulcan)

# =============================================================================
# Coordinate Examples
# =============================================================================
add_executable(frame_demo coordinates/frame_demo.cpp)
target_link_libraries(frame_demo PRIVATE vulcan)

# =============================================================================
# Atmosphere Examples
# =============================================================================
add_executable(atmosphere_profile atmosphere/atmosphere_profile.cpp)
target_link_libraries(atmosphere_profile PRIVATE vulcan)

# =============================================================================
# Gravity Examples
# =============================================================================
add_executable(gravity_comparison gravity/gravity_comparison.cpp)
target_link_libraries(gravity_comparison PRIVATE vulcan)

# =============================================================================
# Integration with Metis Optimization
# =============================================================================
add_executable(trajectory_opt integration/trajectory_optimization.cpp)
target_link_libraries(trajectory_opt PRIVATE vulcan)
```

---

## 6. Scripts Directory

Create the following scripts in `scripts/`:

### build.sh
```bash
#!/usr/bin/env bash
set -e

# Handle arguments
CLEAN=false

for arg in "$@"; do
    case $arg in
        --clean)
        CLEAN=true
        shift
        ;;
    esac
done

if [ "$CLEAN" = true ]; then
    echo "Clean build requested."
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    "$SCRIPT_DIR/clean.sh"
fi

# Create build directory if it doesn't exist or reconfigure
cmake -B build -G Ninja

# Build the project
ninja -C build
```

### clean.sh
```bash
#!/usr/bin/env bash
set -e
echo "Cleaning build directory..."
rm -rf build
echo "Clean complete."
```

### dev.sh
```bash
#!/usr/bin/env bash
# Enter the Vulcan development environment
nix develop
```

### test.sh
```bash
#!/usr/bin/env bash
set -e

# Ensure we have a build
if [ ! -d "build" ]; then
    echo "Build directory not found. Building..."
    ./scripts/build.sh
fi

# Rebuild to ensure latest changes
ninja -C build

# Run tests
mkdir -p logs
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="logs/tests_${TIMESTAMP}.log"

ctest --test-dir build -VV 2>&1 | tee "$LOG_FILE"

# Create symlink to latest
ln -sf "tests_${TIMESTAMP}.log" logs/tests.log
```

### ci.sh
```bash
#!/usr/bin/env bash
set -e

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Ensure logs directory exists
mkdir -p "$PROJECT_ROOT/logs"

# Create timestamp
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$PROJECT_ROOT/logs/ci_${TIMESTAMP}.log"

# Run build and test scripts inside the nix environment
echo "Running CI under Nix..."
cd "$PROJECT_ROOT"
nix develop --command bash -c "./scripts/build.sh --clean && ./scripts/test.sh" 2>&1 | tee "$LOG_FILE"

# Create symlink to latest
ln -sf "ci_${TIMESTAMP}.log" "$PROJECT_ROOT/logs/ci.log"

echo "CI Complete. Logs available at logs/ci_${TIMESTAMP}.log (symlinked to logs/ci.log)"
```

### verify.sh
```bash
#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

mkdir -p "$PROJECT_ROOT/logs"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$PROJECT_ROOT/logs/verify_${TIMESTAMP}.log"

echo "Starting full verification..." | tee "$LOG_FILE"

echo "=== Building ===" | tee -a "$LOG_FILE"
"$SCRIPT_DIR/build.sh" 2>&1 | tee -a "$LOG_FILE"

echo "=== Running Tests ===" | tee -a "$LOG_FILE"
"$SCRIPT_DIR/test.sh" 2>&1 | tee -a "$LOG_FILE"

echo "=== Running Examples ===" | tee -a "$LOG_FILE"
"$SCRIPT_DIR/run_examples.sh" 2>&1 | tee -a "$LOG_FILE"

ln -sf "verify_${TIMESTAMP}.log" "$PROJECT_ROOT/logs/verify.log"
echo "Verification complete! Logs at logs/verify_${TIMESTAMP}.log"
```

### install-hooks.sh
```bash
#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
HOOK_DIR="$PROJECT_ROOT/.git/hooks"
SOURCE_HOOK="$PROJECT_ROOT/.github/hooks/pre-commit"

if [ ! -d "$HOOK_DIR" ]; then
    echo "Error: .git/hooks directory not found. Are you in a git repository?"
    exit 1
fi

if [ ! -f "$SOURCE_HOOK" ]; then
    echo "Error: Pre-commit hook not found at $SOURCE_HOOK"
    exit 1
fi

cp "$SOURCE_HOOK" "$HOOK_DIR/pre-commit"
chmod +x "$HOOK_DIR/pre-commit"

echo "✅ Pre-commit hook installed successfully!"
echo "   Your code will be auto-formatted before each commit."
```

---

## 7. Documentation Standards

### Required Documentation Files

#### docs/design_overview.md
```markdown
# Vulcan Design Overview

## 1. Project Mission

Vulcan is an aerospace engineering utilities library built on the Metis framework...

## 2. Core Modules

### A. Coordinate Systems
...

### B. Atmospheric Models
...
```

#### docs/user_guides/
Create focused guides for each module, similar to Metis's pattern:
- `coordinate_systems.md`
- `atmosphere_models.md` 
- `gravity_models.md`
- `time_systems.md`

#### docs/implementation_plans/
Store phase-based implementation plans here for agent context preservation.

#### docs/saved_work/
Store context and progress notes for agent handover.

---

## 8. Examples Organization

Examples should be organized by module and demonstrate both **numeric** and **symbolic** usage:

### Example Pattern (atmosphere/atmosphere_profile.cpp)
```cpp
#include <vulcan/vulcan.hpp>
#include <iostream>

// Templated model function - works in both modes
template <typename Scalar>
Scalar air_density(const Scalar& altitude) {
    return vulcan::standard_atmosphere::density(altitude);
}

int main() {
    // ========================================
    // 1. Numeric Mode: Fast evaluation
    // ========================================
    std::cout << "=== Numeric Mode ===" << std::endl;
    double alt = 10000.0;  // 10 km
    double rho = air_density(alt);
    std::cout << "Density at 10km: " << rho << " kg/m^3" << std::endl;

    // ========================================
    // 2. Symbolic Mode: For optimization
    // ========================================
    std::cout << "\n=== Symbolic Mode ===" << std::endl;
    metis::Opti opti;
    auto h = opti.variable(5000.0);  // Altitude as decision variable
    
    auto rho_sym = air_density(h);
    
    // Example: Minimize flight at altitude with minimum density
    opti.minimize(rho_sym);
    opti.subject_to_bounds(h, 0.0, 50000.0);
    
    auto sol = opti.solve();
    std::cout << "Optimal altitude: " << sol.value(h) << " m" << std::endl;
    
    return 0;
}
```

---

## 9. Testing Framework

### Test Pattern

Every Vulcan module must have tests for both numeric AND symbolic modes:

```cpp
#include <gtest/gtest.h>
#include <vulcan/atmosphere/StandardAtmosphere.hpp>
#include <metis/metis.hpp>

// ============================================
// Numeric Tests
// ============================================
TEST(StandardAtmosphere, SeaLevelDensity) {
    double rho = vulcan::standard_atmosphere::density(0.0);
    EXPECT_NEAR(rho, 1.225, 1e-3);
}

TEST(StandardAtmosphere, TropopauseDensity) {
    double rho = vulcan::standard_atmosphere::density(11000.0);
    EXPECT_NEAR(rho, 0.3639, 1e-3);
}

// ============================================
// Symbolic Tests (Graph Generation)
// ============================================
TEST(StandardAtmosphere, SymbolicEvaluation) {
    auto alt = metis::sym("altitude");
    auto rho = vulcan::standard_atmosphere::density(alt);
    
    // Verify symbolic expression was created
    EXPECT_FALSE(rho.is_constant());
    
    // Evaluate symbolically-created function
    double result = metis::eval(rho, {{"altitude", 0.0}});
    EXPECT_NEAR(result, 1.225, 1e-3);
}

TEST(StandardAtmosphere, SymbolicGradient) {
    auto alt = metis::sym("altitude");
    auto rho = vulcan::standard_atmosphere::density(alt);
    
    // Verify derivatives exist
    auto drho_dalt = metis::jacobian(rho, alt);
    
    // Density should decrease with altitude
    double grad = metis::eval(drho_dalt, {{"altitude", 5000.0}});
    EXPECT_LT(grad, 0.0);
}
```

---

## 10. CI/CD Workflows

### .github/workflows/ci.yml
```yaml
name: Vulcan CI

on: push

jobs:
  build-and-test:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v4
    
    - name: Install Nix
      uses: cachix/install-nix-action@v20
      with:
        nix_path: nixpkgs=channel:nixos-unstable

    - name: Build & Test (Flake Check)
      run: ./scripts/ci.sh
```

### .github/workflows/format.yml
```yaml
name: Format Check

on: push

jobs:
  format:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
          
      - uses: DeterminateSystems/nix-installer-action@main
      
      - uses: DeterminateSystems/magic-nix-cache-action@main

      - name: Check formatting
        run: |
          if ! nix fmt -- --fail-on-change; then
            echo ""
            echo "❌ Code is not formatted correctly!"
            echo "💡 Run 'nix fmt' locally to fix formatting"
            echo "💡 Or run './scripts/install-hooks.sh' to auto-format on every commit"
            exit 1
          fi
          echo "✅ All files are properly formatted!"
```

### .github/workflows/coverage.yml
```yaml
name: Code Coverage

on: push

jobs:
  coverage:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v4

    - name: Install Nix
      uses: cachix/install-nix-action@v20
      with:
        nix_path: nixpkgs=channel:nixos-unstable

    - name: Run Coverage Script
      run: nix develop --command ./scripts/coverage.sh

    - name: Upload to Codecov
      uses: codecov/codecov-action@v5
      with:
        token: ${{ secrets.CODECOV_TOKEN }}
        files: ./build/coverage/coverage_clean.info
        fail_ci_if_error: true
```

---

## 11. Agent Rules (.cursorrules)

Create `.cursorrules` at the repository root:

```markdown
# Agent Ruleset: Vulcan Project

You are an advanced AI coding assistant working on **Vulcan**, an aerospace engineering utilities library built on the Metis framework. Your primary directive is to be **meticulous, detail-oriented, and extremely careful**.

## Global Behavioral Rules

1.  **Safety First**: You must NEVER "nuke" a repository. Do not delete large portions of code or directories without explicit, confirmed instructions and a valid backup plan.
2.  **Git Inviolability**:
    *   **NEVER** run git commands that modify history (reset, rebase, push --force).
    *   **NEVER** commit or push changes automatically unless explicitly part of a requested workflow.
    *   **ALWAYS** leave git state management to the user unless specifically asked to stage/commit.
    *   **Respect .gitignore**: Do not add files that should be ignored.
3.  **Meticulousness**:
    *   Read all provided context before generating code.
    *   Double-check types, templates, and constraints.
    *   When refactoring, ensure no functionality is lost.
    *   Prefer clarity and correctness over brevity.
4.  **No Hallucinations**: Do not invent APIs. If you are unsure about a Metis or Vulcan API, search the codebase first.
5.  **Context Preservation**:
    *   **Documentation First**: You must CONSTANTLY create and update documentation inside `docs/` to maintain context between agents and resets.
    *   **Artifacts**: Create implementation plans, TODO lists, and architectural notes in `docs/` (e.g., `docs/implementation_plans/`, `docs/TODO.md`).
    *   **Handover**: Assume your memory will be wiped after this session. Write down your plan and progress so the next agent can resume seamlessly.

## Vulcan-Specific Rules (CRITICAL)

### 1. Metis Compatibility (The "Red Line")
*   **Template-First**: ALL engineering models MUST be templated on a generic `Scalar` type.
*   **Dual-Backend Compatibility**: Code must compile and run correctly for both:
    *   **Numeric Mode**: `double` / `Eigen::MatrixXd`
    *   **Symbolic Mode**: `casadi::MX` / `Eigen::Matrix<casadi::MX>`

### 2. Math & Control Flow (MANDATORY)
*   **Math Dispatch**: ALWAYS use `metis::` namespace for math operations (e.g., `metis::sin`, `metis::pow`) instead of `std::`.
*   **Branching**:
    *   **NEVER** use standard C++ `if/else` on `Scalar` types.
    *   **ALWAYS** use `metis::where(condition, true_val, false_val)` for branching logic involving scalars.
*   **Loops**:
    *   Standard `for` loops are allowed ONLY if bounds are structural (integers/constants), not optimization variables.

### 3. Coding Style & Standards
*   **Language Standard**: C++20.
*   **Formatting**: Adhere to `treefmt` (clang-format) rules.
*   **Testing**: Write GoogleTest cases for all new functionality. Ensure tests run for both numeric and symbolic backends.

### 4. Project Structure to Respect
*   `include/vulcan/core/`: Core types and concepts.
*   `include/vulcan/coordinates/`: Coordinate frame utilities.
*   `include/vulcan/atmosphere/`: Atmospheric models.
*   `include/vulcan/gravity/`: Gravity models.
*   `include/vulcan/environment/`: Environmental utilities.
*   `tests/`: Test suite (mirroring include structure).
*   `docs/`: Context and planning documentation (CRITICAL).

## Workflow Commands
*   Build: `./scripts/build.sh`
*   Test: `./scripts/test.sh`
*   Dev: `./scripts/dev.sh`
*   Full Verify: `./scripts/verify.sh`
```

---

## 12. Git Configuration

### .gitignore
```gitignore
# Prerequisites
*.d

# Compiled Object files
*.slo
*.lo
*.o
*.obj

# Precompiled Headers
*.gch
*.pch

# Linker files
*.ilk

# Debugger Files
*.pdb

# Compiled Dynamic libraries
*.so
*.dylib
*.dll

# Fortran module files
*.mod
*.smod

# Compiled Static libraries
*.lai
*.la
*.a
*.lib

# Executables
*.exe
*.out
*.app

# debug information files
*.dwo

# logs
logs/

# Build Directories
build/
bin/
lib/
cmake-build-*/

# Temporary Files
*.swp
*.swo
*~
.DS_Store
*.log
*.tmp
.cache/
*.pdf
*.png
*.dot
*.json

# Preserve doc images
!docs/images/*.png
```

### .clang-format
```yaml
BasedOnStyle: LLVM
IndentWidth: 4
```

---

## 13. Initial Module Roadmap

### Phase 1: Core Infrastructure
- [x] Set up repository structure
- [x] Configure Nix flake with Metis dependency
- [x] Configure CMake
- [x] Set up CI/CD workflows
- [x] Create initial documentation

### Phase 2: Atmospheric Models
- [x] US Standard Atmosphere 1976
- [x] Exponential atmosphere
- [x] Custom tabulated atmosphere

### Phase 3: Coordinate Systems
- [x] WGS84 ellipsoid constants
- [x] ECI/ECEF frames
- [x] NED/ENU local frames
- [x] Body frame definitions
- [x] Transformation matrices (templated)

### Phase 4: Rotations
- [x] DCM utilities (extend Metis quaternions)
- [x] Euler angle sequences (all 12)
- [x] Axis-angle conversions
- [x] Rotation composition

### Phase 5: Wind Models
- [x] Constant wind field
- [x] Wind shear profiles (linear, exponential)
- [x] Dryden turbulence model
- [x] von Kármán turbulence model

### Phase 6: Gravity Models
- [x] Point mass gravity
- [x] J2 perturbation
- [x] J2-J4 perturbations
- [x] Spherical harmonic expansion (future)

### Phase 7: Aerodynamic Utilities
- [x] Dynamic pressure computation
- [x] Mach number / speed of sound
- [x] Reynolds number
- [x] AoA/sideslip from body velocity

### Phase 8: Sensor Noise Models
- [x] Gaussian white noise
- [x] Random walk (integrated white noise)
- [x] Bias instability
- [x] First-order Markov processes
- [x] Allan variance parameterized noise

### Phase 9: Time Systems
- [x] Julian date utilities
- [x] UTC ↔ TAI conversions
- [x] GPS time
- [x] Leap second handling

### Phase 10: Data I/O
- [ ] HDF5Writer (frame-by-frame logging)
- [ ] HDF5Reader (post-simulation analysis)
- [ ] CSVExport (HDF5 → CSV conversion)
- [ ] Adaptable telemetry schema (arbitrary signals out of the simulation)

### Phase 11: Environmental Utilities
- [ ] Solar position (approximate ephemeris)
- [ ] Eclipse detection
- [ ] Magnetic field (dipole model)

---

## 14. Verification Checklist

Before considering the repository "bootstrapped", verify:

- [ ] `nix develop` enters the development shell without errors
- [ ] `./scripts/build.sh` compiles without errors
- [ ] `./scripts/test.sh` runs and passes basic tests
- [ ] `nix fmt` passes without changes
- [ ] CI workflows trigger on push
- [ ] A simple example compiles and runs:
  - Numeric mode evaluation works
  - Symbolic mode graph generation works
  - Integration with `metis::Opti` works

---

## Appendix A: Example Starter Header

Here's an example of a properly structured Vulcan header:

```cpp
// include/vulcan/atmosphere/StandardAtmosphere.hpp
#pragma once

#include <metis/metis.hpp>
#include <vulcan/core/Constants.hpp>

namespace vulcan::standard_atmosphere {

/**
 * @brief US Standard Atmosphere 1976 - Temperature
 * 
 * @tparam Scalar Type for scalar operations (double or casadi::MX)
 * @param altitude Geometric altitude [m]
 * @return Temperature [K]
 */
template <typename Scalar>
Scalar temperature(const Scalar& altitude) {
    // Constants (structural - OK to use as-is)
    constexpr double T0 = 288.15;     // Sea level temperature [K]
    constexpr double L = 0.0065;       // Lapse rate [K/m]
    constexpr double h_tropopause = 11000.0;  // Tropopause altitude [m]
    
    // Use metis::where for branching (MANDATORY for symbolic compatibility)
    auto in_troposphere = (altitude < h_tropopause);
    
    // Troposphere: linear decrease
    Scalar T_troposphere = T0 - L * altitude;
    
    // Stratosphere: isothermal at tropopause temperature
    Scalar T_stratosphere = T0 - L * h_tropopause;
    
    return metis::where(in_troposphere, T_troposphere, T_stratosphere);
}

/**
 * @brief US Standard Atmosphere 1976 - Density
 */
template <typename Scalar>
Scalar density(const Scalar& altitude) {
    constexpr double rho0 = 1.225;    // Sea level density [kg/m^3]
    constexpr double g0 = 9.80665;    // Standard gravity [m/s^2]
    constexpr double M = 0.0289644;   // Molar mass of air [kg/mol]
    constexpr double R = 8.31447;     // Gas constant [J/(mol·K)]
    
    Scalar T = temperature(altitude);
    
    // Barometric formula (simplified for troposphere)
    Scalar exponent = g0 * M / (R * 0.0065);
    return rho0 * metis::pow(T / 288.15, exponent - 1.0);
}

}  // namespace vulcan::standard_atmosphere
```

---

This document should provide all the context needed to bootstrap the Vulcan repository with proper structure, tooling, and Metis integration.
