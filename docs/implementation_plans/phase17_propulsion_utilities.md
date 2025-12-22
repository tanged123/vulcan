# Phase 17: Propulsion Utilities

Symbolic-compatible propulsion modeling utilities for trajectory optimization and simulation.

## Scope

| Category | Included | Notes |
|----------|----------|-------|
| Rocket fundamentals | ✅ | Thrust, Isp, mass flow, delta-v |
| Altitude compensation | ✅ | Pressure-dependent thrust |
| Air-breathing | ✅ | TSFC, altitude/Mach tables |
| Electric propulsion | ✅ | Power-limited, high-Isp |
| Throttle models | ❌ | Icarus component model |
| Staging | ❌ | Structural concern (Icarus) |
| Tank sizing | ❌ | Design tool, not runtime |

---

## Proposed API

### Rocket Propulsion (`include/vulcan/propulsion/Rocket.hpp`)

```cpp
namespace vulcan::propulsion::rocket {

/// Thrust from mass flow and exhaust velocity: F = ṁ * Ve
template <typename Scalar>
Scalar thrust_from_mdot(const Scalar& mdot, const Scalar& Ve);

/// Exhaust velocity from Isp: Ve = Isp * g0
template <typename Scalar>
Scalar exhaust_velocity(const Scalar& Isp, double g0 = 9.80665);

/// Specific impulse from thrust and mass flow: Isp = F / (ṁ * g0)
template <typename Scalar>
Scalar specific_impulse(const Scalar& thrust, const Scalar& mdot,
                        double g0 = 9.80665);

/// Tsiolkovsky rocket equation: Δv = Ve * ln(m0 / mf)
template <typename Scalar>
Scalar delta_v(const Scalar& Ve, const Scalar& m0, const Scalar& mf);

/// Propellant mass for given delta-v: mp = m0 * (1 - exp(-Δv/Ve))
template <typename Scalar>
Scalar propellant_mass(const Scalar& delta_v, const Scalar& m0,
                       const Scalar& Ve);

/// Mass flow rate: ṁ = F / Ve
template <typename Scalar>
Scalar mass_flow_rate(const Scalar& thrust, const Scalar& Ve);

/// Burn time: t_burn = mp / ṁ
template <typename Scalar>
Scalar burn_time(const Scalar& propellant_mass, const Scalar& mdot);

}
```

### Altitude-Compensated Thrust (`include/vulcan/propulsion/AltitudeThrust.hpp`)

```cpp
namespace vulcan::propulsion {

/// Thrust variation with altitude (nozzle expansion)
/// F = F_vac - (P_atm - P_e) * A_e
template <typename Scalar>
Scalar altitude_thrust(const Scalar& F_vac, const Scalar& P_atm,
                       double P_exit, double A_exit);

/// Thrust coefficient from chamber pressure
/// C_F = F / (P_c * A_t)
template <typename Scalar>
Scalar thrust_coefficient(const Scalar& thrust, const Scalar& P_chamber,
                          double A_throat);

}
```

### Air-Breathing Propulsion (`include/vulcan/propulsion/AirBreathing.hpp`)

```cpp
namespace vulcan::propulsion::air_breathing {

/// Fuel consumption: ṁ_fuel = TSFC * F
template <typename Scalar>
Scalar fuel_flow_rate(const Scalar& thrust, const Scalar& TSFC);

/// Range factor (Breguet): R = (V/TSFC) * (L/D) * ln(W0/W1)
template <typename Scalar>
Scalar breguet_range(const Scalar& velocity, const Scalar& TSFC,
                     const Scalar& L_D, const Scalar& W0, const Scalar& W1);

/// Endurance: E = (1/TSFC) * (L/D) * ln(W0/W1)  
template <typename Scalar>
Scalar breguet_endurance(const Scalar& TSFC, const Scalar& L_D,
                         const Scalar& W0, const Scalar& W1);

}
```

### Electric Propulsion (`include/vulcan/propulsion/Electric.hpp`)

```cpp
namespace vulcan::propulsion::electric {

/// Power-limited thrust: P = F * Ve / (2 * η)
template <typename Scalar>
Scalar thrust_from_power(const Scalar& power, const Scalar& Ve,
                         const Scalar& efficiency);

/// Mass flow for electric: ṁ = 2 * P * η / Ve²
template <typename Scalar>
Scalar mass_flow_from_power(const Scalar& power, const Scalar& Ve,
                            const Scalar& efficiency);

/// Characteristic velocity: c* = sqrt(2 * η * P / ṁ)
template <typename Scalar>
Scalar characteristic_velocity(const Scalar& power, const Scalar& efficiency,
                               const Scalar& mdot);

}
```

---

## File Structure

```
include/vulcan/propulsion/
├── Propulsion.hpp          # Aggregate header
├── PropulsionTypes.hpp     # Common types/structs
├── Rocket.hpp              # Rocket fundamentals
├── AltitudeThrust.hpp      # Pressure-dependent thrust
├── AirBreathing.hpp        # TSFC-based models
└── Electric.hpp            # Power-limited EP

tests/propulsion/
├── test_rocket.cpp
├── test_altitude_thrust.cpp
├── test_air_breathing.cpp
└── test_electric.cpp

docs/user_guides/
└── propulsion.md

examples/propulsion/
└── propulsion_demo.cpp
```

---

## Verification Plan

### Automated Tests
```bash
./scripts/ci.sh  # Build + all tests
cd build && ctest -R propulsion -V
```

### Reference Values
| Test | Source |
|------|--------|
| Rocket equation | Sutton & Biblarz, RPE 9th ed |
| Thrust coefficient | NASA SP-125 |
| Breguet range | Anderson, Aircraft Performance |
| Electric propulsion | Goebel & Katz, Ion Propulsion |

### Symbolic Validation
- Instantiate all functions with `casadi::MX`
- Verify autodiff gradients match finite differences

---

## Design Decisions

1. **No state management** — Propulsion functions are pure; Icarus handles engine on/off, staging, fuel depletion

2. **Constants as defaults** — `g0 = 9.80665` as default parameter, overridable for other planets

3. **Scalar-first API** — All functions templated on `Scalar`, using `janus::` math for symbolic compatibility

4. **No table interpolation here** — Complex thrust tables use `TableInterpolator` from core; propulsion module provides the physics
