# Phase 2 Component 2: USSA1976 Atmosphere Detailed Plan

## Goal

Refactor the US Standard Atmosphere 1976 module to:
1. Return all atmospheric properties in a single evaluation (struct-based)
2. Remove legacy analytical model (`standard_atmosphere`)
3. Rename `vulcan::us76` → `vulcan::ussa1976`
4. Improve API ergonomics for trajectory optimization

## User Review Required

> [!IMPORTANT]
> **API Change**: Should we keep individual property functions alongside the struct-based `state()` function?
> ```cpp
> // Option A: Both available (flexible but more code)
> auto T = vulcan::ussa1976::temperature(alt);
> auto state = vulcan::ussa1976::state(alt);
> 
> // Option B: Only struct-based (simpler, encourages batch queries)
> auto state = vulcan::ussa1976::state(alt);
> ```

> [!WARNING]
> **Breaking Change**: Removing `vulcan::standard_atmosphere` namespace entirely. Any code using this will need to migrate to `vulcan::ussa1976`.

---

## Reference Implementation

See [StandardAtmos.m](file:///home/tanged/sources/vulcan/reference/aerospace-toolbox/U.S.%20Standard%20Atmosphere%201976%20Model/StandardAtmos.m) for full COESA model:

| Property | Symbol | Units | Notes |
|----------|--------|-------|-------|
| Geometric altitude | Z | m | Input |
| Geopotential altitude | H | m | Derived |
| Kinetic temperature | T | K | Table or layer equations |
| Molecular temperature | T_M | K | Defined 0-86 km only |
| Pressure | P | Pa | Barometric equations |
| Density | ρ | kg/m³ | Ideal gas law |
| Speed of sound | c | m/s | √(γRT_M/M) |
| Gravity | g | m/s² | g₀(r₀/(r₀+Z))² |
| Dynamic viscosity | μ | Pa·s | Sutherland formula |
| Kinematic viscosity | ν | m²/s | μ/ρ |

---

## Proposed Changes

### [DELETE] [StandardAtmosphere.hpp](file:///home/tanged/sources/vulcan/include/vulcan/atmosphere/StandardAtmosphere.hpp)

Remove legacy analytical model (`vulcan::standard_atmosphere` namespace).

---

### [MODIFY] [US76Atmosphere.hpp](file:///home/tanged/sources/vulcan/include/vulcan/atmosphere/US76Atmosphere.hpp) → **USSA1976.hpp**

Rename file and namespace from `us76` → `ussa1976`.

#### New AtmosphericState Struct

```cpp
namespace vulcan::ussa1976 {

/// All atmospheric properties at a given altitude
template<typename Scalar>
struct AtmosphericState {
    Scalar temperature;         // K
    Scalar pressure;            // Pa  
    Scalar density;             // kg/m³
    Scalar speed_of_sound;      // m/s
    Scalar gravity;             // m/s²
    Scalar dynamic_viscosity;   // Pa·s (optional, 0-20km only)
    Scalar kinematic_viscosity; // m²/s (optional, 0-20km only)
};

/// Evaluate all properties at once
template<typename Scalar>
AtmosphericState<Scalar> state(const Scalar& altitude_m);

// Individual accessors (keep for convenience)
template<typename Scalar> Scalar temperature(const Scalar& altitude_m);
template<typename Scalar> Scalar pressure(const Scalar& altitude_m);
// ... etc

} // namespace vulcan::ussa1976
```

#### Implementation Notes

- `state()` builds a single result struct from table lookups
- For symbolic: each table lookup creates one graph node (efficient)
- Viscosity only populated for altitudes with data (0-20 km)

---

### [MODIFY] Tests

#### [MODIFY] [test_standard.cpp](file:///home/tanged/sources/vulcan/tests/atmosphere/test_standard.cpp)

- Remove all `StandardAtmosphere.*` tests (8 tests)
- Keep `US76Atmosphere.*` tests
- Rename `US76Atmosphere` → `USSA1976`
- Add tests for `state()` struct-based API

---

## Future Enhancement (Not This PR)

For a fully analytical COESA model (reference: `AtmosphericLayers.m`, `CalcTemp.m`):

**Geopotential Layers (0-86 km):**
| Layer | H_b (km') | L_M (K/km') | T_M (K) |
|-------|-----------|-------------|---------|
| 0 | 0 | -6.5 | 288.15 |
| 1 | 11 | 0.0 | 216.65 |
| 2 | 20 | +1.0 | 216.65 |
| 3 | 32 | +2.8 | 228.65 |
| 4 | 47 | 0.0 | 270.65 |
| 5 | 51 | -2.8 | 270.65 |
| 6 | 71 | -2.0 | 214.65 |
| 7 | 84.852 | — | 186.95 |

**Geometric Layers (86-1000 km):**
- Layer 8: 86-91 km, isothermal (T = 186.87 K)
- Layer 9: 91-110 km, elliptical profile (T_c = 263.19 K)
- Layer 10: 110-120 km, linear (L = 12 K/km)
- Layer 11+: 120-1000 km, exponential approach to T_∞ = 1000 K

This would eliminate table lookup overhead but requires complex `janus::where` branching for all 12+ layers.

---

## Verification Plan

### Automated Tests

```bash
# Build and run all tests
./scripts/ci.sh

# Or run atmosphere tests only
cd build && ctest -R atmosphere --output-on-failure
```

**Test coverage after refactor:**

| Test | Coverage |
|------|----------|
| `USSA1976.SeaLevelValues` | Validates T, P, ρ, c, g at z=0 |
| `USSA1976.At10km` | Mid-troposphere validation |
| `USSA1976.At50km` | Stratopause validation |
| `USSA1976.Interpolation` | Non-grid point accuracy |
| `USSA1976.SymbolicEvaluation` | CasADi graph generation |
| `USSA1976.SymbolicGradient` | Autodiff correctness |
| `USSA1976.BatchQuery` | Vector evaluation |
| `USSA1976.StateStruct` | **NEW**: Validates `state()` returns all fields |

### Manual Verification

Run atmosphere profile example and compare output to reference data:
```bash
cd build && ./examples/atmosphere_profile
```
