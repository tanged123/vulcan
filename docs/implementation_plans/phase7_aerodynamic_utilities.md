# Phase 7: Aerodynamic Utilities Implementation Plan

## Overview

Phase 7 adds aerodynamic utility functions to Vulcan. The **aero module owns all core calculations**:
- Dynamic pressure, Mach number, Reynolds number
- Angle of attack / sideslip (moved from BodyFrames)

**Module dependencies**:
```mermaid
graph LR
    Atmosphere["atmosphere/"] -->|speed_of_sound, viscosity| Aero["aerodynamics/"]
    Aero -->|aero_angles| BodyFrames["coordinates/BodyFrames.hpp"]
```

---

## Proposed Changes

### Aerodynamics Module (New)

#### [NEW] [Aerodynamics.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/aerodynamics/Aerodynamics.hpp)

Core aerodynamic calculations, templated for Janus compatibility:

```cpp
namespace vulcan::aero {

// =============================================================================
// Fundamental Quantities
// =============================================================================

/// Dynamic pressure: q = 0.5 * rho * V²  [Pa]
template <typename Scalar>
Scalar dynamic_pressure(const Scalar& density, const Scalar& velocity);

/// Mach number: M = V / a  [-]
template <typename Scalar>
Scalar mach_number(const Scalar& velocity, const Scalar& speed_of_sound);

/// Reynolds number: Re = rho * V * L / mu  [-]
template <typename Scalar>
Scalar reynolds_number(const Scalar& density, const Scalar& velocity,
                       const Scalar& length, const Scalar& dynamic_viscosity);

/// Airspeed from ground velocity and wind: ||V_ground - V_wind||  [m/s]
template <typename Scalar>
Scalar airspeed(const Vec3<Scalar>& v_ground, const Vec3<Scalar>& v_wind);

// =============================================================================
// Aerodynamic Angles (MOVED FROM BodyFrames.hpp)
// =============================================================================

/// Angle of attack and sideslip from body-frame velocity
/// @return [alpha, beta] in radians
template <typename Scalar>
Vec2<Scalar> aero_angles(const Vec3<Scalar>& velocity_body);

// =============================================================================
// Combined State
// =============================================================================

template <typename Scalar>
struct AeroState {
    Scalar dynamic_pressure;  // [Pa]
    Scalar mach;              // [-]
    Scalar reynolds;          // [-]
    Scalar airspeed;          // [m/s]
    Scalar alpha;             // [rad]
    Scalar beta;              // [rad]
};

/// Compute complete aerodynamic state
template <typename Scalar>
AeroState<Scalar> aero_state(const Scalar& density, const Scalar& speed_of_sound,
                             const Scalar& viscosity, const Vec3<Scalar>& velocity_body,
                             const Scalar& char_length);

} // namespace vulcan::aero
```

---

### BodyFrames Update

#### [MODIFY] [BodyFrames.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/coordinates/BodyFrames.hpp)

**Change**: Remove `aero_angles` implementation, replace with import from aerodynamics module.

```diff
+#include <vulcan/aerodynamics/Aerodynamics.hpp>

-/// Compute aerodynamic angles from velocity in body frame
-/// ...
-template <typename Scalar>
-Vec2<Scalar> aero_angles(const Vec3<Scalar> &velocity_body) {
-    // ... full implementation ...
-}

+// Re-export from aerodynamics module for backwards compatibility
+using vulcan::aero::aero_angles;
```

---

### Atmosphere Module (No Changes)

The atmosphere module already provides:
- `speed_of_sound(altitude)` - used by `mach_number()`
- `dynamic_viscosity(altitude)` - used by `reynolds_number()`

These are consumed by the aero module as **inputs**, not dependencies.

---

### vulcan.hpp

#### [MODIFY] [vulcan.hpp](file:///home/tanged/sources/temp/vulcan/include/vulcan/vulcan.hpp)

Add aerodynamics include:
```cpp
#include <vulcan/aerodynamics/Aerodynamics.hpp>
```

---

### Tests

#### [NEW] [test_aerodynamics.cpp](file:///home/tanged/sources/temp/vulcan/tests/aerodynamics/test_aerodynamics.cpp)

| Test Case | Validation |
|-----------|------------|
| Dynamic pressure @ SL, 100 m/s | 6125 Pa |
| Mach 1 @ SL | V ≈ 340 m/s |
| Reynolds @ SL, 100 m/s, L=1m | 6.85×10⁶ |
| AoA/sideslip | Matches expected geometry |
| Symbolic mode | Graph generation works |

#### [MODIFY] [CMakeLists.txt](file:///home/tanged/sources/temp/vulcan/tests/CMakeLists.txt)

Add `test_aerodynamics` executable.

---

## Verification Plan

```bash
./scripts/ci.sh  # Build + test
```

All existing BodyFrames tests should continue to pass (backward compatibility via re-export).
