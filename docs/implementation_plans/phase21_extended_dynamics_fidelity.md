# Phase 21: Extended Dynamics Fidelity

Multi-fidelity dynamics models from 1-DOF oscillators to flexible body, plus actuator/sensor dynamics.

## Background

Phase 20 established the reference 6-DOF rigid body dynamics. This phase extends the dynamics module with:

1. **Primitive oscillators** — 1-DOF and 2-DOF building blocks
2. **Fuel slosh** — Pendulum and spring-mass models
3. **Lower/higher fidelity** — 3-DOF point mass to flexible body
4. **Actuator/sensor dynamics** — Physical response with nonlinearities
5. **Aerodynamic lag** — Unsteady aero effects
6. **Advanced models** — Tethered systems, articulated bodies, wheels

---

## Fidelity Hierarchy

| DOF | Model | States | Use Case |
|-----|-------|--------|----------|
| **1** | Oscillator | 2 | Fuel slosh, linear rail, torsion spring |
| **2** | Planar/Pendulum | 4 | 2D trajectories, spherical pendulum |
| **3** | Point mass | 6 | Trajectory optimization |
| **5** | Guided | 8-10 | Missiles, simple aircraft |
| **6** | Rigid body | 13 | Standard simulation (Phase 20) |
| **6+N** | Flex | 13+2N | Launch vehicles, spacecraft |
| **Composite** | Mix | Variable | Slosh + flex + actuators + rigid |

---

## Proposed API

### Phase 21a: Primitive Oscillators

#### `Oscillator1Dof.hpp`

```cpp
namespace vulcan::dynamics {

template <typename Scalar>
struct Oscillator1DofState {
    Scalar x;      ///< Displacement [m] or angle [rad]
    Scalar x_dot;  ///< Velocity [m/s] or angular rate [rad/s]
};

/// ẍ + 2ζωẋ + ω²x = F/m
template <typename Scalar>
Oscillator1DofDerivatives<Scalar> compute_1dof_oscillator(
    const Oscillator1DofState<Scalar>& state,
    const Scalar& omega_n,
    const Scalar& zeta,
    const Scalar& force,
    const Scalar& mass);

}
```

#### `RailLaunch.hpp`

Rail-constrained 1-DOF motion for rocket/missile launch from rail or tube.

**Depends on:** `coordinates/LocalFrames.hpp` for `local_rail()` frame.

```cpp
namespace vulcan::dynamics {

/// Rail launch state (1-DOF constrained to rail axis)
template <typename Scalar>
struct RailLaunchState {
    Scalar s;       ///< Position along rail [m]
    Scalar s_dot;   ///< Velocity along rail [m/s]
};

/// Rail launch parameters
template <typename Scalar>
struct RailParams {
    Vec3<Scalar> origin;       ///< Rail start point in reference frame [m]
    Vec3<Scalar> direction;    ///< Unit vector along rail (from local_rail())
    Scalar rail_length;        ///< Total rail length [m]
    Scalar friction_coeff;     ///< Rail friction coefficient
    Scalar elevation;          ///< Rail elevation angle [rad]
    Scalar azimuth;            ///< Rail azimuth from North [rad]
};

/// Rail launch dynamics
///
/// Constrains vehicle to 1-DOF motion along rail axis until rail departure.
/// Forces perpendicular to rail are reacted by rail (not integrated).
/// 
/// s̈ = (F·d̂ - μ|F_perp|·sign(ṡ)) / m - g·sin(elev)
///
/// @return {derivatives, on_rail (bool), rail_reaction_force}
template <typename Scalar>
std::tuple<RailLaunchDerivatives<Scalar>, Scalar, Vec3<Scalar>>
compute_rail_launch_dynamics(
    const RailLaunchState<Scalar>& state,
    const RailParams<Scalar>& params,
    const Vec3<Scalar>& force_body,
    const Scalar& mass,
    const Scalar& gravity);

/// Convert rail state to full 6-DOF state (for post-departure)
template <typename Scalar>
RigidBodyState<Scalar> rail_to_6dof_state(
    const RailLaunchState<Scalar>& rail_state,
    const RailParams<Scalar>& params,
    const janus::Quaternion<Scalar>& body_attitude);

}
```

**Coordinate Frame Addition (`coordinates/LocalFrames.hpp`):**

```cpp
/// Create Rail Launch frame
///
/// Rail-aligned coordinate frame for launch dynamics:
/// - X-axis: Along rail (uprange direction)
/// - Y-axis: Horizontal, perpendicular to rail
/// - Z-axis: Completes right-hand system (normal to launch plane)
///
/// @param lla_origin Launch site geodetic position
/// @param azimuth Launch azimuth from North [rad]
/// @param elevation Rail elevation angle from horizontal [rad]
/// @return Rail frame expressed in ECEF
template <typename Scalar>
CoordinateFrame<Scalar> local_rail(
    const LLA<Scalar>& lla_origin,
    const Scalar& azimuth,
    const Scalar& elevation,
    const EarthModel& m = EarthModel::WGS84());

/// Convenience: create from ECEF position
template <typename Scalar>
CoordinateFrame<Scalar> local_rail_at(
    const Vec3<Scalar>& r_ecef,
    const Scalar& azimuth,
    const Scalar& elevation,
    const EarthModel& m = EarthModel::WGS84());
```

#### `Planar2Dof.hpp`

```cpp
namespace vulcan::dynamics {

/// 2-DOF planar point mass
template <typename Scalar>
Planar2DofDerivatives<Scalar> compute_2dof_planar(
    const Planar2DofState<Scalar>& state,
    const Vec2<Scalar>& force,
    const Scalar& mass);

/// Spherical pendulum (2-DOF on sphere)
template <typename Scalar>
SphericalPendulumDerivatives<Scalar> compute_spherical_pendulum(
    const SphericalPendulumState<Scalar>& state,
    const Scalar& length,
    const Scalar& gravity,
    const Scalar& damping_ratio);

}
```

---

### Phase 21b: Fuel Slosh (`Slosh.hpp`)

```cpp
namespace vulcan::dynamics {

/// Pendulum slosh (NASA SP-8009)
/// Returns {derivatives, reaction_force, reaction_moment}
template <typename Scalar>
std::tuple<PendulumSloshDerivatives<Scalar>, Vec3<Scalar>, Vec3<Scalar>>
compute_pendulum_slosh(
    const PendulumSloshState<Scalar>& state,
    const PendulumSloshParams<Scalar>& params,
    const Vec3<Scalar>& accel_body);

/// Spring-mass slosh (3-DOF per tank)
template <typename Scalar>
std::tuple<SpringMassSloshDerivatives<Scalar>, Vec3<Scalar>, Vec3<Scalar>>
compute_spring_mass_slosh(
    const SpringMassSloshState<Scalar>& state,
    const SpringMassSloshParams<Scalar>& params,
    const Vec3<Scalar>& accel_body);

}
```

---

### Phase 21c: Point Mass & Guided

#### `PointMass.hpp`

```cpp
template <typename Scalar>
PointMassDerivatives<Scalar> compute_3dof_derivatives(
    const PointMassState<Scalar>& state,
    const Vec3<Scalar>& force,
    const Scalar& mass);

template <typename Scalar>
PointMassDerivatives<Scalar> compute_3dof_derivatives_ecef(
    const PointMassState<Scalar>& state,
    const Vec3<Scalar>& force_ecef,
    const Scalar& mass,
    const Vec3<Scalar>& omega_earth);
```

#### `Guided5Dof.hpp`

```cpp
/// 5-DOF with transfer function attitude response
template <typename Scalar>
Guided5DofDerivatives<Scalar> compute_5dof_derivatives(
    const Guided5DofState<Scalar>& state,
    const Vec3<Scalar>& force_body,
    const Scalar& mass,
    const Scalar& alpha_cmd, const Scalar& beta_cmd,
    const Scalar& wn, const Scalar& zeta);
```

---

### Phase 21d: Flexible Body

#### `FlexBody.hpp`

```cpp
/// N-mode flexible dynamics: η̈ + 2ζωη̇ + ω²η = Φᵀ F/m
template <typename Scalar, int NumModes>
FlexModeDerivatives<Scalar, NumModes> compute_flex_derivatives(
    const FlexModeState<Scalar, NumModes>& state,
    const FlexModeParams<Scalar, NumModes>& params,
    const Vec3<Scalar>& force_body);
```

#### `FlexRigidBody.hpp`

```cpp
/// Coupled rigid + flex (6+2N DOF)
template <typename Scalar, int NumModes>
FlexRigidBodyDerivatives<Scalar, NumModes> compute_flex_6dof_derivatives(
    const FlexRigidBodyState<Scalar, NumModes>& state,
    const Vec3<Scalar>& force_body,
    const Vec3<Scalar>& moment_body,
    const MassProperties<Scalar>& mass_props,
    const FlexModeParams<Scalar, NumModes>& flex_params);
```

---

### Phase 21e: Actuator & Sensor Dynamics

Uses `transfer_functions/` internally but adds physics constraints.

#### `ActuatorDynamics.hpp`

```cpp
namespace vulcan::dynamics {

template <typename Scalar>
struct ActuatorState {
    Scalar position;   ///< Current position [rad] or [m]
    Scalar rate;       ///< Current rate
};

template <typename Scalar>
struct ActuatorParams {
    Scalar omega_n;        ///< Natural frequency [rad/s]
    Scalar zeta;           ///< Damping ratio
    Scalar rate_limit;     ///< Max rate [rad/s] or [m/s]
    Scalar pos_min;        ///< Position limit min
    Scalar pos_max;        ///< Position limit max
    Scalar backlash;       ///< Deadband [rad] or [m]
};

/// Actuator with 2nd order dynamics + rate/position limits
template <typename Scalar>
ActuatorDerivatives<Scalar> compute_actuator_dynamics(
    const ActuatorState<Scalar>& state,
    const Scalar& command,
    const ActuatorParams<Scalar>& params);

/// Multi-axis actuator (e.g., gimbal)
template <typename Scalar, int NumAxes>
MultiActuatorDerivatives<Scalar, NumAxes> compute_multi_actuator(
    const MultiActuatorState<Scalar, NumAxes>& state,
    const Eigen::Vector<Scalar, NumAxes>& commands,
    const MultiActuatorParams<Scalar, NumAxes>& params);

}
```

#### `SensorDynamics.hpp`

```cpp
namespace vulcan::dynamics {

/// Sensor with 1st order lag
template <typename Scalar>
struct SensorLagState {
    Scalar output;  ///< Current sensor output
};

template <typename Scalar>
SensorLagDerivatives<Scalar> compute_sensor_lag(
    const SensorLagState<Scalar>& state,
    const Scalar& true_value,
    const Scalar& time_constant);

/// Gyro dynamics (2nd order + quantization)
template <typename Scalar>
struct GyroState {
    Vec3<Scalar> output;
    Vec3<Scalar> output_rate;
};

template <typename Scalar>
GyroDerivatives<Scalar> compute_gyro_dynamics(
    const GyroState<Scalar>& state,
    const Vec3<Scalar>& true_omega,
    const Scalar& omega_n,
    const Scalar& zeta);

}
```

---

### Phase 21f: Aerodynamic Lag

#### `AeroLag.hpp`

```cpp
namespace vulcan::dynamics {

/// 1st order lag on aerodynamic coefficients
/// τ·Ċ + C = C_steady
template <typename Scalar>
struct AeroLagState {
    Scalar Cl;   ///< Current lift coefficient
    Scalar Cd;   ///< Current drag coefficient
    Scalar Cm;   ///< Current pitching moment
};

template <typename Scalar>
AeroLagDerivatives<Scalar> compute_aero_lag(
    const AeroLagState<Scalar>& state,
    const Scalar& Cl_steady,
    const Scalar& Cd_steady,
    const Scalar& Cm_steady,
    const Scalar& tau);  ///< Time constant [s]

/// Unsteady aerodynamics with Theodorsen function
/// For flutter analysis, rotorcraft
template <typename Scalar>
UnsteadyAeroDerivatives<Scalar> compute_unsteady_aero(
    const UnsteadyAeroState<Scalar>& state,
    const Scalar& alpha,
    const Scalar& alpha_dot,
    const Scalar& velocity,
    const Scalar& chord);

}
```

---

### Phase 21g: Advanced Multi-Body

#### `Tethered.hpp`

```cpp
namespace vulcan::dynamics {

/// Tethered body constraint
/// Maintains distance L between two points
template <typename Scalar>
struct TetherState {
    Vec3<Scalar> r1, v1;  ///< End 1 position/velocity
    Vec3<Scalar> r2, v2;  ///< End 2 position/velocity
    Scalar tension;       ///< Tether tension [N]
};

/// Returns derivatives + constraint force on each body
template <typename Scalar>
std::tuple<TetherDerivatives<Scalar>, Vec3<Scalar>, Vec3<Scalar>>
compute_tether_dynamics(
    const TetherState<Scalar>& state,
    const Scalar& length,
    const Scalar& stiffness,  ///< Linear spring if slightly elastic
    const Scalar& damping,
    const Vec3<Scalar>& F1_ext,
    const Vec3<Scalar>& F2_ext,
    const Scalar& m1,
    const Scalar& m2);

}
```

#### `Articulated.hpp`

```cpp
namespace vulcan::dynamics {

/// Single revolute joint
template <typename Scalar>
struct JointState {
    Scalar angle;
    Scalar angle_dot;
};

/// Articulated body segment
template <typename Scalar>
ArticulatedDerivatives<Scalar> compute_joint_dynamics(
    const JointState<Scalar>& joint,
    const Vec3<Scalar>& parent_accel,
    const Vec3<Scalar>& parent_omega,
    const Scalar& torque,
    const Scalar& joint_inertia,
    const Scalar& joint_damping);

}
```

#### `WheelDynamics.hpp`

```cpp
namespace vulcan::dynamics {

/// Wheel with ground contact
template <typename Scalar>
struct WheelState {
    Scalar omega;         ///< Wheel angular velocity [rad/s]
    Scalar slip_ratio;    ///< Longitudinal slip
};

/// Returns wheel derivatives + tire force
template <typename Scalar>
std::pair<WheelDerivatives<Scalar>, Vec3<Scalar>>
compute_wheel_dynamics(
    const WheelState<Scalar>& state,
    const Scalar& brake_torque,
    const Scalar& drive_torque,
    const Scalar& normal_force,
    const Scalar& wheel_inertia,
    const Scalar& radius,
    const Scalar& mu);  ///< Friction coefficient

}
```

---

## File Structure

```
include/vulcan/dynamics/
├── Dynamics.hpp            # Aggregate header
├── RigidBodyTypes.hpp      # (existing)
├── RigidBody.hpp           # (existing)
│
├── Oscillator1Dof.hpp      # 21a
├── RailLaunch.hpp          # 21a (rail-constrained launch)
├── Planar2Dof.hpp          # 21a
│
├── Slosh.hpp               # 21b
│
├── PointMass.hpp           # 21c
├── Guided5Dof.hpp          # 21c
│
├── FlexBody.hpp            # 21d
├── FlexRigidBody.hpp       # 21d
│
├── ActuatorDynamics.hpp    # 21e
├── SensorDynamics.hpp      # 21e
│
├── AeroLag.hpp             # 21f
│
├── Tethered.hpp            # 21g
├── Articulated.hpp         # 21g
└── WheelDynamics.hpp       # 21g
```

---

## Implementation Phases

### Phase 21a: Primitive Oscillators & Constrained Motion
- [ ] `Oscillator1Dof.hpp`
- [ ] `RailLaunch.hpp` (rail-constrained 1-DOF launch)
- [ ] `Planar2Dof.hpp` (planar + spherical pendulum)
- [ ] Tests

### Phase 21b: Fuel Slosh
- [ ] `Slosh.hpp` (pendulum + spring-mass)
- [ ] Reaction force computation
- [ ] NASA SP-8009 validation

### Phase 21c: Point Mass & Guided
- [ ] `PointMass.hpp` (3-DOF)
- [ ] `Guided5Dof.hpp` (5-DOF with TF attitude)

### Phase 21d: Flexible Body
- [ ] `FlexBody.hpp` (modal dynamics)
- [ ] `FlexRigidBody.hpp` (coupled)

### Phase 21e: Actuator & Sensor Dynamics
- [ ] `ActuatorDynamics.hpp` (2nd order + limits)
- [ ] `SensorDynamics.hpp` (lag + gyro)
- [ ] Integration with `transfer_functions/`

### Phase 21f: Aerodynamic Lag
- [ ] `AeroLag.hpp` (1st order coefficient lag)
- [ ] Unsteady aero (Theodorsen)

### Phase 21g: Advanced Multi-Body
- [ ] `Tethered.hpp`
- [ ] `Articulated.hpp`
- [ ] `WheelDynamics.hpp`

---

## Verification

```bash
./scripts/ci.sh

# By subsystem
cd build && ctest -R "oscillator|rail|slosh|point_mass|flex|actuator|aero|tether|wheel" -V
```
