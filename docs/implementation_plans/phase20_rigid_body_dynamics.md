# Phase 20: Rigid Body Dynamics

Stateless 6DOF equations of motion utilities for trajectory optimization and simulation.

## Background

The 6DOF equations of motion describe rigid body translational and rotational dynamics. By extracting the *pure physics math* into Vulcan, we enable:

1. **Direct use in trajectory optimization** — No need to instantiate an Icarus `Component` to get dynamics
2. **Guaranteed physics consistency** — `Simulator<double>` and `janus::Opti<MX>` use the exact same code
3. **Clean separation of concerns** — Vulcan provides the math; Icarus manages state and signals

See: [Icarus IDOA Section 9 - Vulcan Integration](file:///home/tanged/sources/icarus/icarus_data_oriented_architecture.md#9-vulcan-integration-engineering-library)

---

## Equations of Motion Reference

### Translational Dynamics (TAOS Reference)

From **TAOS User's Manual Section 2.2** (see [taos_eom.md](file:///home/tanged/sources/vulcan/reference/taos_eom.md)):

> [!IMPORTANT]
> **Inertial Frame (Eq 2-97):**
> $$\vec{a}_{I} = \frac{\Sigma\vec{F}}{m}$$

> **Earth-Relative Frame (Eq 2-105):**
> $$\vec{a}_{\oplus} = \frac{\Sigma\vec{F}}{m} - 2(\vec{\omega}_{\oplus} \times \vec{V}_{\oplus}) - \vec{\omega}_{\oplus} \times (\vec{\omega}_{\oplus} \times \vec{r})$$

Where:
- $2(\vec{\omega}_{\oplus} \times \vec{V}_{\oplus})$ is the **Coriolis acceleration**
- $\vec{\omega}_{\oplus} \times (\vec{\omega}_{\oplus} \times \vec{r})$ is the **Centrifugal acceleration**

**Implementation:** For body-frame formulation commonly used in aircraft/missile simulation, we transform to:

$$\dot{\vec{v}}_B = \frac{\Sigma\vec{F}_B}{m} - \vec{\omega}_B \times \vec{v}_B$$

Where $\vec{\omega}_B \times \vec{v}_B$ accounts for the transport term when velocity is expressed in a rotating body frame.

### Rotational Dynamics (Euler's Equations)

From **Classical Mechanics** (Goldstein, 3rd Ed., Chapter 5):

> [!IMPORTANT]
> **Euler's Equations in Body Frame:**
> $$\mathbf{I} \dot{\vec{\omega}}_B = \vec{M}_B - \vec{\omega}_B \times (\mathbf{I} \vec{\omega}_B)$$

Expanded for principal axes:
$$I_{xx} \dot{\omega}_x = M_x - (I_{zz} - I_{yy}) \omega_y \omega_z$$
$$I_{yy} \dot{\omega}_y = M_y - (I_{xx} - I_{zz}) \omega_z \omega_x$$
$$I_{zz} \dot{\omega}_z = M_z - (I_{yy} - I_{xx}) \omega_x \omega_y$$

**Derivation:** The angular momentum $\vec{H} = \mathbf{I}\vec{\omega}$. Newton's second law for rotation states $\vec{M} = \frac{d\vec{H}}{dt}$. When expressed in a body-fixed rotating frame with angular velocity $\vec{\omega}$, the transport theorem gives:

$$\vec{M} = \frac{d\vec{H}}{dt}\bigg|_{body} + \vec{\omega} \times \vec{H}$$

Substituting $\vec{H} = \mathbf{I}\vec{\omega}$ (with constant body-fixed inertia) yields Euler's equations.

### Quaternion Kinematics

From **Quaternion-Based Flight Control** (Shuster, 1993):

$$\dot{q} = \frac{1}{2} q \otimes \omega_{quat}$$

Where $\omega_{quat} = (0, \omega_x, \omega_y, \omega_z)$ is the angular velocity expressed as a pure quaternion.

**Implementation:** Already available as `vulcan::quaternion_rate_from_omega()` in [RotationKinematics.hpp](file:///home/tanged/sources/vulcan/include/vulcan/rotations/RotationKinematics.hpp).

---

## Scope

| Category | Included | Notes |
|----------|----------|-------|
| Translational dynamics | ✅ | $\dot{v} = F/m - \omega \times v$ (body frame) |
| Rotational dynamics | ✅ | Euler's equations $I\dot{\omega} + \omega \times I\omega = M$ |
| Quaternion kinematics | ✅ | $\dot{q} = \frac{1}{2} q \otimes \omega$ |
| Mass properties | ✅ | Mass, inertia tensor struct (input, not internal state) |
| State/derivative structs | ✅ | `RigidBodyState`, `RigidBodyDerivatives` |
| State vector packing | ❌ | Icarus integrator concern |
| Component wiring | ❌ | Icarus signal/backplane concern |

> [!NOTE]
> **Mass Properties Philosophy:** Mass properties (`MassProperties<Scalar>`) are passed as an **input argument** to dynamics functions, not stored internally. This enables:
> - Variable mass (from propulsion): Pass current mass at each timestep
> - Time-varying inertia: Pass updated inertia tensor as fuel depletes  
> - Optimization variables: Use `casadi::MX` for mass as an optimization parameter
>
> The caller (Icarus component or trajectory optimizer) is responsible for providing the **current** mass properties. This is cleaner than baking in mass-rate equations because the caller knows where mass properties come from (propulsion model, lookup table, symbolic parameter, etc.).

> [!CAUTION]
> **Agent Implementation Note:** The `MassProperties::full()` factory applies NEGATIVE signs to off-diagonals internally. Do NOT double-negate when testing.

---

## Proposed API

### State & Derivative Types (`include/vulcan/dynamics/RigidBodyTypes.hpp`)

```cpp
namespace vulcan::dynamics {

/// Mass properties for a rigid body
/// 
/// This is passed AS INPUT to dynamics functions, not stored.
/// The caller provides the current mass properties at each timestep,
/// enabling variable mass vehicles (rockets, aircraft with fuel burn).
///
/// The inertia tensor is a full 3x3 SYMMETRIC matrix:
///   | Ixx  -Ixy  -Ixz |
///   |-Ixy   Iyy  -Iyz |
///   |-Ixz  -Iyz   Izz |
///
/// Sign convention: Products of inertia (Ixy, Ixz, Iyz) use the POSITIVE
/// convention, i.e., Ixy = ∫ xy dm. The negative signs appear in the tensor.
template <typename Scalar>
struct MassProperties {
    Scalar mass;              ///< Current mass [kg]
    Mat3<Scalar> inertia;     ///< Full inertia tensor in body frame [kg·m²]
    Vec3<Scalar> cg_offset;   ///< CG offset from geometric center (optional) [m]
    
    /// Construct from mass only (assumes zero CG offset, identity inertia)
    static MassProperties<Scalar> from_mass(const Scalar& m) {
        return MassProperties<Scalar>{
            .mass = m,
            .inertia = Mat3<Scalar>::Identity() * m,  // Point mass at origin
            .cg_offset = Vec3<Scalar>::Zero()
        };
    }
    
    /// Construct for diagonal inertia tensor (principal axes aligned with body)
    static MassProperties<Scalar> diagonal(
        const Scalar& m, 
        const Scalar& Ixx, const Scalar& Iyy, const Scalar& Izz) {
        Mat3<Scalar> I = Mat3<Scalar>::Zero();
        I(0,0) = Ixx; I(1,1) = Iyy; I(2,2) = Izz;
        return MassProperties<Scalar>{
            .mass = m,
            .inertia = I,
            .cg_offset = Vec3<Scalar>::Zero()
        };
    }
    
    /// Construct with full inertia tensor (6 unique components)
    /// Products of inertia use POSITIVE convention: Ixy = ∫ xy dm
    static MassProperties<Scalar> full(
        const Scalar& m,
        const Scalar& Ixx, const Scalar& Iyy, const Scalar& Izz,
        const Scalar& Ixy, const Scalar& Ixz, const Scalar& Iyz) {
        Mat3<Scalar> I;
        I(0,0) = Ixx;   I(0,1) = -Ixy;  I(0,2) = -Ixz;
        I(1,0) = -Ixy;  I(1,1) = Iyy;   I(1,2) = -Iyz;
        I(2,0) = -Ixz;  I(2,1) = -Iyz;  I(2,2) = Izz;
        return MassProperties<Scalar>{
            .mass = m,
            .inertia = I,
            .cg_offset = Vec3<Scalar>::Zero()
        };
    }
};

/// Rigid body state in body-fixed frame
/// Position/velocity relative to an implicit reference frame (e.g., ECI/ECEF)
template <typename Scalar>
struct RigidBodyState {
    Vec3<Scalar> position;             ///< Position in reference frame [m]
    Vec3<Scalar> velocity_body;        ///< Velocity in body frame [m/s]
    janus::Quaternion<Scalar> attitude; ///< Body-to-reference quaternion
    Vec3<Scalar> omega_body;           ///< Angular velocity in body frame [rad/s]
};

/// Time derivatives of rigid body state
template <typename Scalar>
struct RigidBodyDerivatives {
    Vec3<Scalar> position_dot;         ///< Velocity in reference frame [m/s]
    Vec3<Scalar> velocity_dot;         ///< Acceleration in body frame [m/s²]
    janus::Quaternion<Scalar> attitude_dot; ///< Quaternion rate
    Vec3<Scalar> omega_dot;            ///< Angular acceleration in body frame [rad/s²]
};

}
```

### Core Dynamics (`include/vulcan/dynamics/RigidBody.hpp`)

```cpp
namespace vulcan::dynamics {

/// Compute full 6DOF rigid body derivatives
///
/// Implements the standard 6DOF equations of motion:
/// - **Translational (TAOS Eq 2-105 in body frame):**
///   $$\dot{v}_B = F_B/m - \omega_B \times v_B$$
/// - **Rotational (Euler's equations, Goldstein Ch.5):**
///   $$I \dot{\omega}_B = M_B - \omega_B \times (I \omega_B)$$
/// - **Kinematics (Shuster 1993):**
///   $$\dot{q} = \frac{1}{2} q \otimes (0, \omega_B)$$
///
/// This is a PURE FUNCTION with no internal state.
///
/// @tparam Scalar Scalar type (double or SymbolicScalar)
/// @param state Current rigid body state
/// @param force_body Sum of forces in body frame [N]
/// @param moment_body Sum of moments about CG in body frame [N·m]
/// @param mass_props Current mass and inertia properties (variable mass supported)
/// @return State derivatives
template <typename Scalar>
RigidBodyDerivatives<Scalar> compute_6dof_derivatives(
    const RigidBodyState<Scalar>& state,
    const Vec3<Scalar>& force_body,
    const Vec3<Scalar>& moment_body,
    const MassProperties<Scalar>& mass_props);

/// Translational dynamics only (F = ma in body frame with transport term)
///
/// $$\dot{v}_B = F_B/m - \omega_B \times v_B$$
///
/// Reference: TAOS Methods (2-105), body-frame form
///
/// @param velocity_body Current velocity in body frame [m/s]
/// @param omega_body Angular velocity in body frame [rad/s]
/// @param force_body Sum of forces in body frame [N]
/// @param mass Current body mass [kg]
/// @return Acceleration in body frame [m/s²]
template <typename Scalar>
Vec3<Scalar> translational_dynamics(
    const Vec3<Scalar>& velocity_body,
    const Vec3<Scalar>& omega_body,
    const Vec3<Scalar>& force_body,
    const Scalar& mass);

/// Rotational dynamics only (Euler's equations)
///
/// $$I \dot{\omega}_B = M_B - \omega_B \times (I \omega_B)$$
///
/// Reference: Goldstein, Classical Mechanics, 3rd Ed., Ch.5
///
/// @param omega_body Angular velocity in body frame [rad/s]
/// @param moment_body Sum of moments in body frame [N·m]
/// @param inertia Inertia tensor in body frame [kg·m²]
/// @return Angular acceleration in body frame [rad/s²]
template <typename Scalar>
Vec3<Scalar> rotational_dynamics(
    const Vec3<Scalar>& omega_body,
    const Vec3<Scalar>& moment_body,
    const Mat3<Scalar>& inertia);

/// Transform velocity from body frame to reference frame
///
/// v_ref = R_body_to_ref * v_body
///
/// @param velocity_body Velocity in body frame [m/s]
/// @param attitude Body-to-reference quaternion
/// @return Velocity in reference frame [m/s]
template <typename Scalar>
Vec3<Scalar> velocity_to_reference_frame(
    const Vec3<Scalar>& velocity_body,
    const janus::Quaternion<Scalar>& attitude);

/// Transform velocity from reference frame to body frame
///
/// v_body = R_ref_to_body * v_ref
///
/// @param velocity_ref Velocity in reference frame [m/s]
/// @param attitude Body-to-reference quaternion
/// @return Velocity in body frame [m/s]
template <typename Scalar>
Vec3<Scalar> velocity_to_body_frame(
    const Vec3<Scalar>& velocity_ref,
    const janus::Quaternion<Scalar>& attitude);

/// Compute Earth-relative acceleration with Coriolis and centrifugal terms
///
/// For ECEF integration, includes fictitious forces:
/// $$\dot{v}_{\oplus} = F/m - 2(\omega_{\oplus} \times v_{\oplus}) 
///                    - \omega_{\oplus} \times (\omega_{\oplus} \times r)$$
///
/// Reference: TAOS Methods Eq 2-105
///
/// @param position Position in ECEF [m]
/// @param velocity_ecef Velocity in ECEF [m/s]
/// @param force_ecef Force in ECEF [N]
/// @param mass Mass [kg]
/// @param omega_earth Earth rotation vector [rad/s] (typically 7.2921159e-5 * z_hat)
/// @return Acceleration in ECEF [m/s²]
template <typename Scalar>
Vec3<Scalar> translational_dynamics_ecef(
    const Vec3<Scalar>& position,
    const Vec3<Scalar>& velocity_ecef,
    const Vec3<Scalar>& force_ecef,
    const Scalar& mass,
    const Vec3<Scalar>& omega_earth);

}
```

---

## Dependencies

This module depends on existing Vulcan utilities:

| Dependency | From | Purpose |
|------------|------|---------|
| `Vec3<Scalar>`, `Mat3<Scalar>` | `vulcan/core/VulcanTypes.hpp` | Vector/matrix types |
| `janus::Quaternion<Scalar>` | `janus/math/Quaternion.hpp` | Attitude representation |
| `quaternion_rate_from_omega()` | `vulcan/rotations/RotationKinematics.hpp` | Quaternion kinematics |
| `janus::cross()` | `janus/linalg/Vec.hpp` | Cross product |

---

## File Structure

```
include/vulcan/dynamics/
├── Dynamics.hpp            # Aggregate header
├── RigidBodyTypes.hpp      # MassProperties, State, Derivatives structs
└── RigidBody.hpp           # Core 6DOF dynamics functions

tests/dynamics/
├── test_rigid_body.cpp     # Core dynamics tests

docs/user_guides/
└── dynamics.md

examples/dynamics/
└── dynamics_demo.cpp
```

---

## Pre-Implementation Verification

Before starting implementation, verify dependencies exist:

- [ ] `janus::Quaternion<Scalar>` exists and is templated (`janus/math/Quaternion.hpp`)
- [ ] `quaternion_rate_from_omega()` signature in `vulcan/rotations/RotationKinematics.hpp`
- [ ] `janus::cross()` works for `Vec3<casadi::MX>` (`janus/linalg/Vec.hpp`)
- [ ] `Mat3<Scalar>` alias exists in `vulcan/core/VulcanTypes.hpp`

---

## Implementation Checklist

- [ ] Create `include/vulcan/dynamics/RigidBodyTypes.hpp`
  - [ ] `MassProperties<Scalar>` struct with `from_mass()`, `diagonal()`, `full()` factories
  - [ ] `RigidBodyState<Scalar>` struct
  - [ ] `RigidBodyDerivatives<Scalar>` struct
- [ ] Create `include/vulcan/dynamics/RigidBody.hpp`
  - [ ] `translational_dynamics()`
  - [ ] `rotational_dynamics()`
  - [ ] `compute_6dof_derivatives()`
  - [ ] `velocity_to_reference_frame()`
  - [ ] `velocity_to_body_frame()`
  - [ ] `translational_dynamics_ecef()`
- [ ] Create `include/vulcan/dynamics/Dynamics.hpp` (aggregate header)
- [ ] Update `include/vulcan/vulcan.hpp` to include dynamics module
- [ ] Create `tests/dynamics/test_rigid_body.cpp`
- [ ] Update `tests/CMakeLists.txt` to add dynamics test
- [ ] Create `examples/dynamics/dynamics_demo.cpp`
- [ ] Create `docs/user_guides/dynamics.md`
```

---

## Verification Plan

### Automated Tests

```bash
# Build and run all tests
./scripts/ci.sh

# Run dynamics tests specifically
cd build && ctest -R dynamics -V
```

### Test Cases with Concrete Values

#### 1. Torque-Free Symmetric Gyroscope
- **Setup:** `Ixx = Iyy = 10.0` kg·m², `Izz = 5.0` kg·m², `ω = (0.1, 0.2, 0.5)` rad/s, `M = (0,0,0)`
- **Expected:** `ω_dot_z = 0` (spin axis torque-free), `|ω|` magnitude conserved
- **Tolerance:** `1e-10`

#### 2. Euler's Equations Component Check
- **Setup:** `Ixx = 2.0`, `Iyy = 3.0`, `Izz = 4.0` kg·m², `ω = (1, 2, 3)` rad/s, `M = (0,0,0)`
- **Expected:**
  - `ω_dot_x = -(Izz - Iyy) * ωy * ωz / Ixx = -(4-3)*2*3/2 = -3.0` rad/s²
  - `ω_dot_y = -(Ixx - Izz) * ωz * ωx / Iyy = -(2-4)*3*1/3 = +2.0` rad/s²
  - `ω_dot_z = -(Iyy - Ixx) * ωx * ωy / Izz = -(3-2)*1*2/4 = -0.5` rad/s²
- **Tolerance:** `1e-12`

#### 3. Pure Translational (No Rotation)
- **Setup:** `v_body = (100, 0, 0)` m/s, `ω = (0, 0, 0)` rad/s, `F = (500, 0, 0)` N, `m = 100` kg
- **Expected:** `v_dot = F/m = (5, 0, 0)` m/s²
- **Tolerance:** `1e-12`

#### 4. Transport Term (Spinning Body)
- **Setup:** `v_body = (10, 0, 0)` m/s, `ω = (0, 0, 0.1)` rad/s, `F = (0, 0, 0)` N, `m = 1` kg
- **Expected:** `v_dot = -ω × v = -(0, 0, 0.1) × (10, 0, 0) = (0, 1, 0)` m/s²
- **Tolerance:** `1e-12`

#### 5. ECEF Coriolis + Centrifugal
- **Setup:** Position at equator `r = (6.371e6, 0, 0)` m, velocity `v = (0, 100, 0)` m/s, `F = 0`, `ω_earth = (0, 0, 7.2921159e-5)` rad/s
- **Expected Coriolis:** `2 * (0, 0, 7.29e-5) × (0, 100, 0) = (0.01458, 0, 0)` m/s² (outward)
- **Expected Centrifugal:** `ω × (ω × r) ≈ (0.0339, 0, 0)` m/s² (outward)
- **Tolerance:** `1e-6` (due to Earth rotation rate precision)

#### 6. Quaternion Kinematics Consistency
- **Setup:** Identity quaternion `q = (1, 0, 0, 0)`, `ω = (0.1, 0, 0)` rad/s
- **Expected:** `q_dot = 0.5 * q ⊗ (0, 0.1, 0, 0) = (0, 0.05, 0, 0)`
- **Tolerance:** `1e-12`

### Symbolic Validation Pattern

```cpp
// Copy-paste this test pattern for symbolic validation
TEST(RigidBodySymbolic, Instantiation) {
    using MX = casadi::MX;
    
    // Create symbolic mass properties
    auto m = MX::sym("m");
    auto Ixx = MX::sym("Ixx");
    auto Iyy = MX::sym("Iyy");
    auto Izz = MX::sym("Izz");
    auto mass_props = vulcan::dynamics::MassProperties<MX>::diagonal(m, Ixx, Iyy, Izz);
    
    // Create symbolic state
    vulcan::dynamics::RigidBodyState<MX> state{
        .position = Vec3<MX>::Zero(),
        .velocity_body = Vec3<MX>{MX::sym("vx"), MX::sym("vy"), MX::sym("vz")},
        .attitude = janus::Quaternion<MX>::identity(),
        .omega_body = Vec3<MX>{MX::sym("wx"), MX::sym("wy"), MX::sym("wz")}
    };
    
    // Create symbolic inputs
    Vec3<MX> force{MX::sym("Fx"), MX::sym("Fy"), MX::sym("Fz")};
    Vec3<MX> moment{MX::sym("Mx"), MX::sym("My"), MX::sym("Mz")};
    
    // This should NOT throw — graph builds successfully
    auto derivs = vulcan::dynamics::compute_6dof_derivatives<MX>(
        state, force, moment, mass_props);
    
    // Verify outputs are symbolic (non-empty graph)
    EXPECT_FALSE(derivs.velocity_dot(0).is_empty());
    EXPECT_FALSE(derivs.omega_dot(0).is_empty());
}
```

### Integration Test (with Janus ODE solver)

```cpp
// Example: Free-falling rotating body
auto derivs_fn = [&mass_props](auto t, const auto& state) {
    auto parsed = unpack_state(state);  // Helper to unpack vector
    Vec3<Scalar> gravity = {0, 0, -9.81};
    Vec3<Scalar> force_body = attitude.inverse().rotate(gravity) * mass_props.mass;
    Vec3<Scalar> moment_body = Vec3<Scalar>::Zero();
    auto d = vulcan::dynamics::compute_6dof_derivatives(
        parsed, force_body, moment_body, mass_props);
    return pack_derivatives(d);  // Helper to pack into vector
};
auto [t_out, x_out] = janus::solve_ivp(derivs_fn, t_span, x0, janus::RK4);
```

---

## Design Decisions

1. **Body-frame velocity** — Velocity stored in body frame matches aerospace convention and simplifies force summation (thrust, aero in body frame)

2. **Quaternion for attitude** — Avoids gimbal lock, singular-free representation; aligns with Janus `Quaternion<Scalar>` class

3. **No state packing/unpacking** — That's Icarus' job; Vulcan just provides the math with clean struct types

4. **Stateless functions** — No internal state; all dependencies passed as arguments; enables direct use in optimizers

5. **Separate translational/rotational** — Allows using just one when the other is fixed (e.g., attitude-only control problem)

6. **Mass properties as input** — Unlike the original plan's `VariableMass.hpp` extension, mass properties are simply passed as an argument. This allows:
   - Rockets: Propulsion component computes mass rate, passes current mass
   - Aircraft: Fuel burn model provides updated mass at each timestep  
   - Optimization: Mass can be a CasADi variable for optimal fuel scheduling

7. **ECEF helper** — Added `translational_dynamics_ecef()` to directly implement TAOS Eq 2-105 for Earth-relative trajectory simulation

---

## Icarus Integration Pattern

After this module is implemented, an Icarus `RigidBody6DOF` component would look like:

```cpp
template <typename Scalar>
void RigidBody6DOF::Step(Scalar t, Scalar dt) {
    // 1. Gather inputs from signals (mass comes from propulsion component!)
    Vec3<Scalar> F = *input_force_;
    Vec3<Scalar> M = *input_moment_;
    
    // 2. Get current mass properties from mass properties component
    //    (This handles variable mass naturally!)
    vulcan::dynamics::MassProperties<Scalar> mass_props{
        .mass = *input_mass_,
        .inertia = *input_inertia_,
        .cg_offset = *input_cg_offset_
    };
    
    // 3. Call Vulcan for pure physics (stateless)
    auto derivs = vulcan::dynamics::compute_6dof_derivatives(
        current_state_, F, M, mass_props);
    
    // 4. Write derivatives to state output (integrator handles the rest)
    *state_dot_position_ = derivs.position_dot;
    *state_dot_velocity_ = derivs.velocity_dot;
    // ... etc
}
```

For trajectory optimization (without Icarus):

```cpp
// Direct use in janus::Opti — mass can be a symbolic variable!
auto mass = opti.variable();  // Mass as optimization variable
opti.subject_to(mass >= dry_mass);

vulcan::dynamics::MassProperties<MX> props{
    .mass = mass,
    .inertia = I_body,  // Could also be state-dependent
    .cg_offset = cg
};

auto derivs = vulcan::dynamics::compute_6dof_derivatives<MX>(
    state_vars, F_symbolic, M_symbolic, props);
opti.subject_to(v_next == v + derivs.velocity_dot * dt);
```
