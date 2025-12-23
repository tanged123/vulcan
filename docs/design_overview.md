# Vulcan Design Overview

## 1. Project Mission

**Vulcan** is an aerospace engineering utilities library built on the [Janus](https://github.com/tanged123/janus) framework. Named after the Roman god of fire and forge, Vulcan provides model-agnostic simulation utilities that work seamlessly in both numeric and symbolic computational modes.

### Core Capabilities

- **Coordinate Systems & Geodetics**: ECI, ECEF, NED, CDA, Body frames; LLA↔ECEF conversions
- **Atmospheric Models**: US Standard Atmosphere 1976, exponential models
- **Gravity Models**: Point mass, J2/J4 perturbations, spherical harmonics
- **Orbital Mechanics**: Keplerian elements, propagators, anomaly conversions
- **Aerodynamic Utilities**: Mach, dynamic pressure, Reynolds number
- **Propulsion**: Rockets, electric, air-breathing, altitude compensation
- **Dynamics**: 6DOF rigid body, 5DOF guided, 3DOF point mass, oscillators, fuel slosh, rail launch
- **Mass Properties**: Aggregation, parallel axis theorem, shape primitives
- **Control & Dynamics**: Transfer functions, PID, discretization
- **Estimation**: Kalman Filters (Linear, EKF, UKF)
- **Time Systems**: High-precision time conversions (TAI, UTC, GPS, TT, TDB)
- **Geometry**: Symbolic-compatible geometric primitives
- **Unit Conversions**: SI, imperial, angular
- **Physical Constants**: Consolidated library (WGS84, Earth parameters)

### Key Design Principle: Janus Compatibility

All Vulcan utilities are **templated on a generic `Scalar` type** to maintain compatibility with Janus's dual-backend system:

| Mode | Scalar Type | Purpose |
|------|-------------|---------|
| **Numeric** | `double` | Fast simulation, real-time control |
| **Symbolic** | `casadi::MX` | Graph generation, optimization |

---

## 2. Architecture

```
vulcan/
├── include/vulcan/
│   ├── core/           # Constants, Utils, Types
│   ├── atmosphere/     # Atmospheric models
│   ├── coordinates/    # Coordinate systems
│   ├── dynamics/       # 6DOF, point mass, guided, oscillators, slosh
│   ├── mass/           # Mass properties, aggregation, inertia
│   ├── orbital/        # Orbital mechanics
│   ├── gravity/        # Gravity models
│   └── ...
├── tests/              # GoogleTest suite
├── examples/           # Usage examples
└── docs/               # Documentation
```

### Stateless Philosophy

Vulcan adheres to a strict **State-Free** architecture. It is NOT a simulation engine; it is a library of physics models.

| Concept | Definition |
| :--- | :--- |
| **Model** | A stateless unit of **Physics**. A pure function or helper class (usually from **Vulcan**) that performs a standard calculation given inputs (e.g., `density(h)`, `forces(state)`). |
| **State** | A data structure (struct) defined by the **User** that holds the variables evolving in time. |
| **Integrator** | An external algorithm (RK4, etc.) that advances the **State** using derivatives computed by the **Model**. |

**Why?**
- **Symbolic Compatibility**: Internal state variables (like `dt` counters) break symbolic graph unrolling.
- **Flexibility**: The user owns the memory layout (Arrays vs Structs vs Eigen).
- **Correctness**: Physics doesn't change; state does. Separating them prevents "magic" side effects.

---

---

## 3. The "Red Line" Rules

These constraints ensure Janus compatibility:

1. **Template-First Design**: All models templated on `Scalar`
2. **Math Dispatch**: Use `janus::` namespace (`janus::sin`, `janus::pow`)
3. **Branching**: Use `janus::where()`, never `if/else` on Scalars
4. **Loop Bounds**: Must be structural (compile-time constants)

---

## 4. Current Status

### Implemented Modules ✅

- **Core**: Constants, Units, Interpolation, Error Handling
- **Coordinates**: ECEF, LLA, NED, Body frames, Transforms
- **Rotations**: Quaternions, DCMs, Euler angles, SLERP
- **Atmosphere**: US Standard Atmosphere 1976
- **Gravity**: Point Mass, J2/J4, Spherical Harmonics
- **Orbital**: Keplerian elements, propagators, anomalies
- **Time**: Time scales, Julian dates, Leap seconds
- **Wind**: Shear models, Turbulence (Dryden, Von Kármán)
- **Aerodynamics**: Dynamic pressure, Mach, Reynolds #
- **Sensors**: IMU noise models (Random Walk, Bias Instability)
- **Propulsion**: Rocket, Electric, Air-breathing, Altitude-compensation
- **Control**: Transfer functions, Discretization, PID
- **Estimation**: Kalman Filter, EKF, UKF
- **Geometry**: Basic primitives (Sphere, Cone, Cylinder)
- **RNG**: Reproducible random number generation
- **I/O**: HDF5, CSV, Telemetry support
- **Dynamics**: 6DOF rigid body, 5DOF guided, 3DOF point mass, oscillators, slosh, rail launch
- **Mass Properties**: Aggregation, parallel axis theorem, shape factories

### In Refinement 🚧

- **Environment**: Solar flux models, space weather
