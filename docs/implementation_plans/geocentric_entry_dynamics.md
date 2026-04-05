# Geocentric 3-DOF Entry Dynamics for Hypersonic Trajectory Optimization

Stateless geocentric spherical-coordinate equations of motion for atmospheric entry trajectory optimization.

## Background

The existing `PointMass.hpp` provides 3-DOF dynamics in **Cartesian** frames (inertial/ECEF). Hypersonic trajectory optimization universally uses a different formulation: the **geocentric spherical** (LLA-based) 3-DOF point-mass EOMs with state vector `(h, theta, phi, V, gamma, psi)`. This is the canonical form used by all SOTA methods:

- Sequential Convex Programming (SCvx) for real-time entry guidance
- Pseudospectral methods (GPOPS-II, Radau collocation)
- Direct collocation with CasADi/IPOPT
- hp-adaptive mesh refinement schemes

The formulation follows **Vinh (1981)** "Optimal Trajectories in Atmospheric Flight" and **Betts (2010)** "Practical Methods for Optimal Control", the two standard references for this field. Recent work (2023-2025) from AIAA and Aerospace Science & Technology journals confirms this remains the standard 3-DOF formulation for hypersonic glide vehicle trajectory optimization.

### Why Not Just Use the Cartesian PointMass?

1. **State minimality**: 6 scalars vs 6-vector (position) + 6-vector (velocity) = 12 Cartesian states. Fewer NLP variables = faster optimization.
2. **Natural constraint expression**: Path constraints (heating, dynamic pressure, g-load) and boundary conditions (altitude, downrange) are directly expressed in these coordinates.
3. **Literature compatibility**: Every published entry trajectory paper uses this formulation. Results can be directly compared.
4. **Physical intuition**: Flight path angle and heading have direct aerodynamic meaning.

---

## Equations of Motion Reference

### State Vector

| Symbol | Name | Units | Description |
|--------|------|-------|-------------|
| h | Altitude | m | Height above reference sphere |
| theta | Longitude | rad | Geocentric longitude |
| phi | Latitude | rad | Geocentric latitude |
| V | Velocity | m/s | Earth-relative speed magnitude |
| gamma | Flight path angle | rad | Angle from local horizontal (+up) |
| psi | Heading angle | rad | Clockwise from North |

### Controls

| Symbol | Name | Units |
|--------|------|-------|
| alpha | Angle of attack | rad |
| sigma | Bank angle | rad |

### Non-Rotating Spherical Earth (Vinh 1981, Ch. 1)

Kinematic equations:

$$\dot{h} = V \sin\gamma$$

$$\dot{\theta} = \frac{V \cos\gamma \sin\psi}{r \cos\phi}$$

$$\dot{\phi} = \frac{V \cos\gamma \cos\psi}{r}$$

Dynamic equations:

$$\dot{V} = -\frac{D}{m} + \frac{T\cos\alpha}{m} - g\sin\gamma$$

$$\dot{\gamma} = \frac{L\cos\sigma + T\sin\alpha}{mV} - \left(\frac{g}{V} - \frac{V}{r}\right)\cos\gamma$$

$$\dot{\psi} = \frac{L\sin\sigma}{mV\cos\gamma} - \frac{V}{r}\cos\gamma\sin\psi\tan\phi$$

Where:
- $r = R_{\text{body}} + h$ (radial distance from center)
- $g = \mu / r^2$ (inverse-square gravity)
- $L = \frac{1}{2}\rho V^2 S C_L$ (lift), $D = \frac{1}{2}\rho V^2 S C_D$ (drag)
- For unpowered glide, $T = 0$ and thrust terms vanish

### Rotating Earth Extension (Vinh 1981, Ch. 4, Eq. 4.18-4.20)

Add to the non-rotating equations:

$$\dot{V} \mathrel{+}= \omega^2 r \cos\phi \left[\sin\gamma\cos\phi - \cos\gamma\sin\phi\cos\psi\right]$$

$$\dot{\gamma} \mathrel{+}= \frac{\omega^2 r}{V}\cos\phi\left[\cos\gamma\cos\phi + \sin\gamma\sin\phi\cos\psi\right] + 2\omega\cos\phi\sin\psi$$

$$\dot{\psi} \mathrel{+}= \frac{\omega^2 r}{V\cos\gamma}\sin\phi\cos\phi\sin\psi + 2\omega\left(\tan\gamma\cos\phi\cos\psi - \sin\phi\right)$$

The centripetal terms ($\omega^2 r$) provide "centripetal relief" reducing effective gravity. The Coriolis terms ($2\omega$) deflect the trajectory. These are significant for long-range hypersonic glide (>1000 km downrange).

### Path Constraints (Standard in Trajectory Optimization)

| Constraint | Formula | Typical Bound |
|-----------|---------|---------------|
| Dynamic pressure | $q = \frac{1}{2}\rho V^2$ | $q \leq q_{\max}$ (structural) |
| Stagnation heating | $\dot{Q} = k_q \sqrt{\rho} \cdot V^3$ | $\dot{Q} \leq \dot{Q}_{\max}$ (TPS) |
| Load factor | $n = \sqrt{L^2 + D^2} / (mg_0)$ | $n \leq n_{\max}$ (structural/crew) |

Chapman's approximation (1959, NACA TN 4150) for stagnation-point convective heating:
- $k_q \approx 1.7415 \times 10^{-4}$ for Shuttle-class nose radius (~1m)
- Units: $[\text{W}/(\text{m}^2 \cdot (\text{kg/m}^3)^{0.5} \cdot (\text{m/s})^3)]$

### Known Singularities

1. $\dot{\theta}$: Contains $1/\cos\phi$ -- singular at poles ($\phi = \pm\pi/2$)
2. $\dot{\psi}$: Contains $1/\cos\gamma$ -- singular at vertical flight ($\gamma = \pm\pi/2$)
3. $\dot{\psi}$: Contains $\tan\phi$ -- singular at poles

These are **coordinate singularities**, not physical. Entry trajectories never pass over the poles or reach vertical flight, so they are not problematic for trajectory optimization. Document them in the header but do not add regularization (it distorts gradients).

---

## Scope

| Feature | Included | Notes |
|---------|----------|-------|
| State/derivative structs | Yes | `GeocentricState<Scalar>`, `GeocentricDerivatives<Scalar>` |
| Non-rotating EOMs | Yes | Standard Vinh/Betts formulation with optional thrust |
| Rotating Earth EOMs | Yes | Centripetal + Coriolis corrections |
| Aero force helpers | Yes | `lift_force`, `drag_force` (convenience wrappers) |
| Gravity helper | Yes | `gravity_inverse_square(r, mu)` |
| Path constraints | Yes | Dynamic pressure, Chapman heating, load factor |
| State conversions | Yes | To ECEF position, NED velocity, specific energy |
| Oblate Earth (J2) | No | Negligible for atmospheric entry; can add later |
| Mass variation (mdot) | No | Add as a 7th state equation if needed later |

---

## File Plan

### New Files

| File | Purpose |
|------|---------|
| `include/vulcan/dynamics/GeocentricEntry.hpp` | All EOMs, structs, helpers, path constraints |
| `tests/dynamics/test_geocentric_entry.cpp` | Numeric + symbolic tests |
| `examples/dynamics/geocentric_entry_demo.cpp` | Forward integration + symbolic graph + Opti trajectory optimization |

### Modified Files

| File | Change |
|------|--------|
| `include/vulcan/dynamics/Dynamics.hpp` | Add `#include <vulcan/dynamics/GeocentricEntry.hpp>` |
| `tests/CMakeLists.txt` (line ~219) | Add `dynamics/test_geocentric_entry.cpp` to `test_dynamics` sources |
| `examples/CMakeLists.txt` (after line ~153) | Add `geocentric_entry_demo` executable |

---

## Implementation Details: `GeocentricEntry.hpp`

All in `namespace vulcan::dynamics`. Header-only. Templated on `Scalar`.

### Structs

```cpp
template <typename Scalar>
struct GeocentricState {
    Scalar h;      ///< Altitude above reference sphere [m]
    Scalar theta;  ///< Longitude [rad]
    Scalar phi;    ///< Geocentric latitude [rad]
    Scalar V;      ///< Earth-relative velocity magnitude [m/s]
    Scalar gamma;  ///< Flight path angle [rad] (positive = climbing)
    Scalar psi;    ///< Heading angle [rad] (clockwise from North)
};

template <typename Scalar>
struct GeocentricDerivatives {
    Scalar h_dot;      ///< Altitude rate [m/s]
    Scalar theta_dot;  ///< Longitude rate [rad/s]
    Scalar phi_dot;    ///< Latitude rate [rad/s]
    Scalar V_dot;      ///< Acceleration along velocity [m/s^2]
    Scalar gamma_dot;  ///< Flight path angle rate [rad/s]
    Scalar psi_dot;    ///< Heading angle rate [rad/s]
};
```

Pattern: Mirrors `RigidBodyState`/`RigidBodyDerivatives` in `include/vulcan/dynamics/RigidBodyTypes.hpp`.

### Function Signatures

**Aerodynamic helpers:**
```cpp
template <typename Scalar>
Scalar lift_force(const Scalar &density, const Scalar &velocity,
                  const Scalar &S_ref, const Scalar &C_L);

template <typename Scalar>
Scalar drag_force(const Scalar &density, const Scalar &velocity,
                  const Scalar &S_ref, const Scalar &C_D);
```

**Gravity:**
```cpp
template <typename Scalar>
Scalar gravity_inverse_square(const Scalar &r, double mu = constants::earth::mu);
```

**Non-rotating EOMs:**
```cpp
template <typename Scalar>
GeocentricDerivatives<Scalar> geocentric_entry_derivatives(
    const GeocentricState<Scalar> &state,
    const Scalar &lift, const Scalar &drag, const Scalar &mass,
    const Scalar &sigma,
    const Scalar &thrust = Scalar(0), const Scalar &alpha = Scalar(0),
    double R_body = constants::earth::R_mean, double mu = constants::earth::mu);
```

**Rotating Earth EOMs:**
```cpp
template <typename Scalar>
GeocentricDerivatives<Scalar> geocentric_entry_derivatives_rotating(
    const GeocentricState<Scalar> &state,
    const Scalar &lift, const Scalar &drag, const Scalar &mass,
    const Scalar &sigma,
    const Scalar &thrust = Scalar(0), const Scalar &alpha = Scalar(0),
    double R_body = constants::earth::R_mean, double mu = constants::earth::mu,
    double omega = constants::earth::omega);
```

Design: Rotating version calls non-rotating internally and adds correction terms (no code duplication).

**Path constraints:**
```cpp
template <typename Scalar>
Scalar entry_dynamic_pressure(const Scalar &density, const Scalar &velocity);

template <typename Scalar>
Scalar entry_heating_rate(const Scalar &density, const Scalar &velocity,
                          double k_q = 1.7415e-4);

template <typename Scalar>
Scalar entry_load_factor(const Scalar &lift, const Scalar &drag,
                         const Scalar &mass, double g0 = constants::physics::g0);
```

**Conversions:**
```cpp
template <typename Scalar>
Vec3<Scalar> geocentric_state_to_ecef_position(
    const GeocentricState<Scalar> &state, double R_body = constants::earth::R_mean);

template <typename Scalar>
Vec3<Scalar> geocentric_state_to_velocity_ned(const GeocentricState<Scalar> &state);

template <typename Scalar>
Scalar geocentric_specific_energy(
    const GeocentricState<Scalar> &state,
    double R_body = constants::earth::R_mean, double mu = constants::earth::mu);
```

### Design Decisions

1. **L and D as inputs, not rho/CL/CD**: Keeps dynamics decoupled from atmosphere/aero models. User computes forces externally. Same pattern as `Guided5Dof.hpp`.
2. **Thrust decomposition**: `T*cos(alpha)` along velocity, `T*sin(alpha)` normal. Standard bank-to-turn convention where bank rotates only the lift vector.
3. **Default parameters**: `R_body`, `mu`, `omega` default to Earth values from `Constants.hpp`. Allows use with other bodies.
4. **`entry_` prefix on path constraints**: Avoids collision with `vulcan::aero::dynamic_pressure` in `Aerodynamics.hpp`.

### Constants Used (from `include/vulcan/core/Constants.hpp`)

- `constants::earth::R_mean` (6371008.8 m)
- `constants::earth::mu` (3.986004418e14 m^3/s^2)
- `constants::earth::omega` (7.2921159e-5 rad/s)
- `constants::physics::g0` (9.80665 m/s^2)

---

## Test Plan: `test_geocentric_entry.cpp`

Pattern: Follows `tests/dynamics/test_point_mass.cpp` exactly -- open `vulcan::dynamics` namespace, GTest, both numeric and symbolic.

### Kinematic Tests

| Test | Setup | Validation |
|------|-------|------------|
| LevelFlightEquator | h=100km, phi=0, gamma=0, psi=pi/2 (East) | h_dot=0, theta_dot=V/r, phi_dot=0 |
| VerticalClimb | gamma=pi/2 | h_dot=V, theta_dot=0, phi_dot=0 |
| NorthboundFlight | gamma=0, psi=0 | phi_dot=V/r, theta_dot=0 |

### Dynamic Tests

| Test | Setup | Validation |
|------|-------|------------|
| DragOnlyDeceleration | L=0, T=0, gamma=0 | V_dot = -D/m |
| BallisticFreefall | L=0, D=0, T=0, gamma=-pi/2 | V_dot = g (accelerating down) |
| CircularOrbitEquilibrium | V=sqrt(mu/r), gamma=0, L=0, D=0, T=0 | V_dot~0, gamma_dot~0 |
| BankTurnEffect | L>0, sigma=pi/4, gamma=0 | psi_dot > 0 (turning right) |
| ThrustDecomposition | T>0, alpha>0 | V_dot increases, gamma_dot gets pull-up |

### Rotating Earth Tests

| Test | Validation |
|------|------------|
| ZeroOmega_MatchesNonRotating | rotating(omega=0) == non-rotating exactly |
| EquatorialEastbound | Centripetal relief: rotating.V_dot > non_rotating.V_dot |

### Path Constraint Tests

| Test | Setup | Expected |
|------|-------|----------|
| DynamicPressure_SeaLevel | rho=1.225, V=100 | q=6125 Pa |
| HeatingRate | Known rho, V | Verify against Chapman formula |
| LoadFactor | L=10000, D=5000, m=1000 | n = sqrt(1.25e8)/(1000*9.80665) |

### Conversion Tests

| Test | Validation |
|------|------------|
| StateToECEF | Round-trip: state -> ECEF -> back via atan2/asin |
| VelocityNED_Magnitude | norm(v_ned) == V |

### Symbolic Tests

| Test | Method |
|------|--------|
| SymbolicDerivatives_NonRotating | Build janus::Function, evaluate at concrete values, compare to numeric |
| SymbolicDerivatives_Rotating | Same for rotating version |
| SymbolicPathConstraints | Verify sqrt/pow trace through CasADi |
| SymbolicJacobian | Compute `janus::jacobian` of EOMs w.r.t. state; verify differentiability |

---

## Example: `geocentric_entry_demo.cpp`

Three demonstrations:

### 1. Numeric Forward Integration (Unpowered Shuttle-Class Entry)

- Initial: h=120km, V=7500 m/s, gamma=-1 deg, phi=0, theta=0, psi=pi/4
- Exponential atmosphere for density lookup at each step
- Simple drag polar: C_D = C_D0 + k*C_L^2, fixed alpha -> fixed C_L, C_D
- Euler integration with dt=1s, ~600 steps
- Print state + path constraints (q, Q_dot, n) every 10 seconds
- Shows deceleration, heating peak, eventual pull-out

### 2. Symbolic Graph Construction

- Create symbolic state + control variables
- Call `geocentric_entry_derivatives_rotating` symbolically
- Build `janus::Function` wrapping dynamics + all path constraints
- Compute state Jacobian via `janus::jacobian`
- Print Jacobian dimensions and evaluate at a flight condition
- Demonstrates the EOMs are fully CasADi-differentiable

### 3. Minimum-Heating Entry Optimization (janus::Opti)

- Decision variables: bank angle profile sigma(t) over N discrete nodes
- State variables: (h, theta, phi, V, gamma, psi) at each node
- Objective: minimize peak heating rate (or integrated heat load)
- Equality constraints: trapezoidal/Hermite-Simpson collocation of the EOMs
- Inequality constraints: q <= q_max, n <= n_max, h >= 0
- Boundary conditions: fixed initial state, final altitude/velocity targets
- Solve with `janus::Opti` (IPOPT backend)
- Print optimal trajectory and bank angle profile

This demonstrates the complete pipeline these EOMs are designed for.

---

## Build Integration

### `include/vulcan/dynamics/Dynamics.hpp`
```cpp
// Add one line:
#include <vulcan/dynamics/GeocentricEntry.hpp>
```

### `tests/CMakeLists.txt` (add to test_dynamics target, line ~219)
```cmake
    dynamics/test_geocentric_entry.cpp    # <-- ADD
```

### `examples/CMakeLists.txt` (add after dynamics_demo section, ~line 153)
```cmake
# =============================================================================
# Geocentric Entry Dynamics Examples
# =============================================================================
add_executable(geocentric_entry_demo dynamics/geocentric_entry_demo.cpp)
target_link_libraries(geocentric_entry_demo PRIVATE vulcan)
target_precompile_headers(geocentric_entry_demo REUSE_FROM getting_started)
```

---

## Verification

```bash
# Build
./scripts/build.sh

# Run dynamics tests only
ctest --test-dir build -R test_dynamics

# Run the example
./scripts/run_example.sh geocentric_entry_demo

# Full suite (should all pass with no regressions)
./scripts/verify.sh
```

---

## References

1. Vinh, N.X. (1981). *Optimal Trajectories in Atmospheric Flight*. Elsevier. Ch. 1 & 4.
2. Betts, J.T. (2010). *Practical Methods for Optimal Control and Estimation Using Nonlinear Programming*, 2nd Ed. SIAM.
3. Chapman, D.R. (1959). "An Approximate Analytical Method for Studying Entry into Planetary Atmospheres". NACA TN 4150.
4. Lu, P. (1993). "Analytical Solutions to Constrained Hypersonic Flight Trajectories". *J. Guidance, Control, and Dynamics*.
5. Wang et al. (2023). "High-Accuracy 3-DoF Hypersonic Reentry Guidance via Sequential Convex Programming". AIAA SciTech.
6. Zhang et al. (2024). "Entry Guidance for Hypersonic Glide Vehicles via Two-Phase hp-Adaptive Sequential Convex Programming". *Aerospace*.
