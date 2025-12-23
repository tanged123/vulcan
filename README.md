# Vulcan 🔥

**Aerospace Engineering Utilities Built on Janus**

[![Documentation](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://tanged123.github.io/vulcan/)
[![CI](https://github.com/tanged123/vulcan/actions/workflows/ci.yml/badge.svg)](https://github.com/tanged123/vulcan/actions/workflows/ci.yml)
[![Format](https://github.com/tanged123/vulcan/actions/workflows/format.yml/badge.svg)](https://github.com/tanged123/vulcan/actions/workflows/format.yml)
[![codecov](https://codecov.io/github/tanged123/vulcan/graph/badge.svg?token=0DSF7KK8W7)](https://codecov.io/github/tanged123/vulcan)

Vulcan is an aerospace engineering utilities library that provides model-agnostic simulation utilities for coordinate systems, atmospheric models, gravity models, and more. Built on the [Janus](https://github.com/tanged123/janus) math library, Vulcan utilities work seamlessly in both **numeric** and **symbolic** computational modes.

## Features

- 🌍 **Coordinate Systems**: ECI, ECEF, NED, Body frames; geodetic utilities
- 🌤️ **Atmospheric Models**: US Standard Atmosphere 1976, exponential models
- 🌑 **Gravity Models**: Point mass, J2/J4 perturbations, spherical harmonics
- 🛰️ **Orbital Mechanics**: Keplerian elements, state vector propagation, anomaly conversions
- 📐 **Geometry**: Primitives (Sphere, Cylinder, Cone, Box) with symbolic support
- 📡 **Sensors**: IMU noise models (Random Walk, Bias Instability), Gaussian noise, Markov processes
- 🎲 **RNG**: Reproducible random number generation with stream splitting
- 💾 **Data I/O**: HDF5 reading/writing, telemetry schemas, CSV export
- 💨 **Wind Models**: Constant wind, wind shear (linear/power-law/log), Dryden & von Kármán turbulence
- ✈️ **Aerodynamics**: Dynamic pressure, Mach, Reynolds number, angle of attack/sideslip
- 🎮 **Transfer Functions**: Transfer functions (1st/2nd order), discretization, PID control
- 📉 **Estimation**: Kalman Filter, EKF, UKF, estimation utilities
- 🚀 **Propulsion**: Rocket, Electric, Air-breathing, Altitude-compensated thrust
- 🎯 **Dynamics**: 6DOF rigid body, 5DOF guided, 3DOF point mass, 1/2DOF oscillators, fuel slosh, rail launch
- ⚖️ **Mass Properties**: Aggregation, parallel axis theorem, shape primitives, validation
- ⏱️ **Time Systems**: UTC, TAI, GPS, TT, TDB; Julian date conversions; leap seconds
- 🔄 **Rotations**: Quaternions, DCMs, all 12 Euler sequences, axis-angle, SLERP
- 🌌 **Environment**: Space environment constants, solar flux (placeholder)
- 📊 **Units & Constants**: SI/imperial conversions, WGS84, Earth parameters

> **Note**: Vulcan uses **SI units** throughout (meters, kilograms, seconds, radians) unless explicitly stated otherwise.

## Quick Start

### Prerequisites

- [Nix](https://nixos.org/download.html) package manager (recommended)
- Or: CMake 3.20+, Clang/GCC with C++20, Eigen3, CasADi, Janus

### Building

```bash
# Enter dev environment
nix develop

# Build
./scripts/build.sh

# Run tests
./scripts/test.sh

# Run examples
./scripts/run_examples.sh
```

### Example Usage

```cpp
#include <vulcan/vulcan.hpp>

// 1. Pure Physics Model (State-Free)
template <typename Scalar>
Scalar flight_loads(Scalar alt, Scalar vel) {
    // Composition: Atmosphere + Aerodynamics modules
    Scalar rho = vulcan::ussa1976::density(alt);
    return vulcan::aero::dynamic_pressure(rho, vel);
}

int main() {
    // 2. Numeric Mode (Simulation)
    double q = flight_loads(10000.0, 500.0); 
    
    // 3. Symbolic Mode (Optimization)
    auto h = janus::sym("h");
    auto v = janus::sym("v");
    auto q_sym = flight_loads(h, v);
    
    // Automatic differentiation
    auto dq_dh = janus::jacobian(q_sym, h); 
}
```

## Directory Structure

```
vulcan/
├── docs/
│   ├── implementation_plans/ # Design documents
│   └── user_guides/          # Module documentation
├── examples/
│   ├── aerodynamics/       # Aero calculations demo
│   ├── atmosphere/         # Atmospheric model usage
│   ├── coordinates/        # Coordinate frame transformations
│   ├── dynamics/           # Dynamics demo
│   ├── environment/        # Space environment
│   ├── geodetic/           # Geodetic conversions
│   ├── geometry/           # Geometric primitives
│   ├── gravity/            # Gravity models demo
│   ├── intro/              # Getting started
│   ├── io/                 # HDF5 and telemetry I/O
│   ├── mass/               # Mass properties, aggregation, inertia
│   ├── orbital/            # Orbital mechanics & optimization
│   ├── propulsion/         # Propulsion models demo
│   ├── rotations/          # Rotation and attitude examples
│   ├── sensors/            # Sensor noise simulation
│   ├── time/               # Time systems and Julian dates
│   ├── transfer_functions/ # Transfer functions demo
│   └── wind/               # Wind model optimization
├── include/vulcan/
│   ├── aerodynamics/       # Dynamic pressure, Mach, Reynolds, AoA
│   ├── atmosphere/         # US76, Exponential atmosphere
│   ├── coordinates/        # ECEF, LLA, NED, body frames
│   ├── core/               # Types, constants, interpolation
│   ├── environment/        # Space environment utilities
│   ├── estimation/         # Kalman filters (Linear, EKF, UKF)
│   ├── geodetic/           # Geodesic utils
│   ├── geometry/           # Geometric primitives
│   ├── gravity/            # Point mass, J2/J4, spherical harmonics
│   ├── io/                 # HDF5, CSV, Signal, Telemetry
│   ├── dynamics/           # 6DOF, point mass, guided, oscillators, slosh
│   ├── mass/               # Mass properties, aggregation, inertia
│   ├── orbital/            # Keplerian, anomaly, ephemeris
│   ├── propulsion/         # Rocket, electric, air-breathing
│   ├── rng/                # Random number generation
│   ├── rotations/          # Quaternions, Euler, DCM, axis-angle
│   ├── sensors/            # Noise models (Allan variance, etc.)
│   ├── time/               # GPS, UTC, TAI, TT, TDB, Julian dates
│   ├── transfer_functions/ # Dynamics, discretization, PID
│   ├── wind/               # Shear profiles, Dryden, von Kármán
│   └── vulcan.hpp          # Main umbrella header
├── scripts/                # Build, test, and dev utilities
├── tests/                  # GoogleTest suite mirroring include/
└── reference/              # Reference data and lookups
```

## The Janus Paradigm

Vulcan follows Janus's dual-backend design. All models are templated on a `Scalar` type:

| Mode | Scalar Type | Purpose |
|------|-------------|---------|
| **Numeric** | `double` | Fast simulation, real-time control |
| **Symbolic** | `casadi::MX` | Graph generation, optimization |

### Critical Rules

```cpp
// ✅ Use janus:: namespace for math
auto result = janus::sin(theta) * janus::pow(r, 2);

// ✅ Use janus::where() for branching
Scalar cd = janus::where(mach > 1.0, 0.5, 0.02);

// ❌ NEVER use std:: math or if/else on Scalars
```

## Architecture: State-Free

**Vulcan is explicitly STATE FREE.** 

It does **not** manage simulation state, time integration, or object lifecycles. It is a library of **pure** physics models and utilities.

- **No classes storing `dt` or `time`**: You provide the state, Vulcan calculates the derivative.
- **No internal integrators**: You choose the integrator (RK4, discrete stepping, collocation).
- **Just Physics**: Header-only libraries defining the equations of motion and constitutive laws.

This design is critical for symbolic optimization, where the entire simulation must be unrolled into a single computational graph without side effects.


## License

MIT License - See [LICENSE](LICENSE) for details.
