# Vulcan Design Overview

## 1. Project Mission

**Vulcan** is an aerospace engineering utilities library built on the [Janus](https://github.com/tanged123/janus) framework. Named after the Roman god of fire and forge, Vulcan provides model-agnostic simulation utilities that work seamlessly in both numeric and symbolic computational modes.

### Core Capabilities

- **Coordinate Systems & Geodetics**: ECI, ECEF, NED, Body frames; LLA↔ECEF conversions
- **Atmospheric Models**: US Standard Atmosphere 1976, exponential models
- **Gravity Models**: Point mass, J2/J4 perturbations
- **Aerodynamic Utilities**: Mach, dynamic pressure, Reynolds number
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
│   ├── core/           # Constants, Units, Types
│   ├── atmosphere/     # Atmospheric models
│   ├── coordinates/    # Coordinate systems
│   ├── gravity/        # Gravity models
│   └── ...
├── tests/              # GoogleTest suite
├── examples/           # Usage examples
└── docs/               # Documentation
```

---

## 3. The "Red Line" Rules

These constraints ensure Janus compatibility:

1. **Template-First Design**: All models templated on `Scalar`
2. **Math Dispatch**: Use `janus::` namespace (`janus::sin`, `janus::pow`)
3. **Branching**: Use `janus::where()`, never `if/else` on Scalars
4. **Loop Bounds**: Must be structural (compile-time constants)

---

## 4. Current Status

### Phase 1: Core Infrastructure ✅
- Nix flake with Janus dependency
- CMake configuration
- CI/CD workflows
- Core constants and units

### Future Phases
- Phase 2: Coordinate Systems
- Phase 3: Rotations
- Phase 4: Atmospheric Models (extended)
- Phase 5: Wind Models
- Phase 6: Gravity Models
- Phase 7-11: Advanced utilities
