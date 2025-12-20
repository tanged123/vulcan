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
- 🌑 **Gravity Models**: Point mass, J2/J4 perturbations
- ✈️ **Aerodynamic Utilities**: Mach, dynamic pressure, Reynolds number
- 📐 **Unit Conversions**: SI, imperial, angular
- 📊 **Physical Constants**: WGS84, Earth parameters, atmospheric constants

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

// Templated function works in both modes
template <typename Scalar>
Scalar air_density(const Scalar& altitude) {
    return vulcan::standard_atmosphere::density(altitude);
}

int main() {
    // Numeric mode
    double rho = air_density(10000.0);  // 10 km
    
    // Symbolic mode
    auto h = janus::sym("h");
    auto rho_sym = air_density(h);
    auto drho_dh = janus::jacobian(rho_sym, h);  // Automatic differentiation!
}
```

## Directory Structure

```
vulcan/
├── docs/
│   └── user_guides/        # Walkthroughs and guides
│       ├── atmosphere_models_walkthrough.md
│       ├── coordinate_systems_walkthrough.md
│       ├── rotations.md
│       └── time_systems.md
├── examples/
│   ├── atmosphere/         # Atmospheric model usage
│   ├── coordinates/        # Coordinate frame transformations
│   ├── intro/              # Getting started
│   ├── rotations/          # Rotation and attitude examples
│   └── time/               # Time systems and Julian dates
├── include/vulcan/
│   ├── atmosphere/         # Atmospheric models (US76, Exponential)
│   ├── coordinates/        # ECEF, LLA, NED transformations
│   ├── core/               # Math utilities and constants
│   ├── rotations/          # Quaternions, Euler angles, DCMs
│   ├── time/               # GPS, UTC, TAI, Julian Dates
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

## License

MIT License - See [LICENSE](LICENSE) for details.
