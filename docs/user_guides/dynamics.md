# Rigid Body Dynamics

The `vulcan::dynamics` module provides stateless 6DOF equations of motion utilities for trajectory optimization and simulation. These functions are designed to work with both numeric (`double`) and symbolic (`casadi::MX`) types, enabling direct integration with Janus optimization.

## Key Features

- **Stateless design** — Pure functions with no internal state
- **Dual-backend compatible** — Works with `double` and `casadi::MX`
- **Variable mass support** — Mass properties passed as input, not stored
- **Physics-anchored** — Derived from TAOS, Goldstein, and Shuster references

## Quick Start

```cpp
#include <vulcan/dynamics/Dynamics.hpp>

using namespace vulcan::dynamics;

// Define mass properties
auto mass_props = MassProperties<double>::diagonal(
    100.0,  // mass [kg]
    10.0, 20.0, 30.0  // Ixx, Iyy, Izz [kg·m²]
);

// Define current state
RigidBodyState<double> state{
    .position = {0, 0, 1000},        // [m]
    .velocity_body = {50, 0, 0},     // [m/s]
    .attitude = janus::Quaternion<double>(),  // identity
    .omega_body = {0, 0, 1}          // [rad/s]
};

// Forces and moments in body frame
Vec3<double> force{0, 0, -981};   // gravity
Vec3<double> moment{0, 0, 0};

// Compute derivatives
auto derivs = compute_6dof_derivatives(state, force, moment, mass_props);
// derivs.position_dot, velocity_dot, attitude_dot, omega_dot
```

## Core Types

### MassProperties

```cpp
// Point mass
auto props = MassProperties<double>::from_mass(100.0);

// Diagonal inertia (principal axes aligned)
auto props = MassProperties<double>::diagonal(m, Ixx, Iyy, Izz);

// Full inertia tensor (6 unique components)
auto props = MassProperties<double>::full(m, Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
```

> [!NOTE]
> Products of inertia (Ixy, Ixz, Iyz) use **positive convention** (Ixy = ∫xy dm). The `full()` factory applies the negative signs internally.

### RigidBodyState / RigidBodyDerivatives

State and derivative structures for 6DOF integration:

| Field | State | Derivative |
|-------|-------|------------|
| Position | `position` [m] | `position_dot` [m/s] |
| Velocity | `velocity_body` [m/s] | `velocity_dot` [m/s²] |
| Attitude | `attitude` (quaternion) | `attitude_dot` |
| Angular velocity | `omega_body` [rad/s] | `omega_dot` [rad/s²] |

## Core Functions

### Full 6DOF Dynamics

```cpp
auto derivs = compute_6dof_derivatives(state, force_body, moment_body, mass_props);
```

Implements:
- **Translational**: `v̇_B = F_B/m - ω_B × v_B`
- **Rotational (Euler's)**: `I ω̇_B = M_B - ω_B × (I ω_B)`
- **Kinematics**: `q̇ = ½ q ⊗ (0, ω_B)`

### Individual Components

```cpp
// Translational only
auto v_dot = translational_dynamics(v_body, omega_body, force_body, mass);

// Rotational only (Euler's equations)
auto omega_dot = rotational_dynamics(omega_body, moment_body, inertia);
```

### Frame Transformations

```cpp
// Body → Reference frame
auto v_ref = velocity_to_reference_frame(v_body, attitude);

// Reference → Body frame
auto v_body = velocity_to_body_frame(v_ref, attitude);
```

### ECEF Dynamics

For Earth-relative integration with Coriolis and centrifugal effects:

```cpp
Vec3<double> omega_earth{0, 0, 7.2921159e-5};  // Earth rotation

auto a_ecef = translational_dynamics_ecef(
    position, velocity_ecef, force_ecef, mass, omega_earth);
```

Implements: `a_⊕ = F/m - 2(ω_⊕ × v_⊕) - ω_⊕ × (ω_⊕ × r)`

## Symbolic Usage (Trajectory Optimization)
 
All functions work with `casadi::MX` for Janus optimization, allowing you to solve optimal control problems directly using the dynamics equations.

### Example: Max Sustained Turn Rate

The following example demonstrates how to use `janus::Opti` to find the optimal control inputs (Lift, Bank, Thrust) for a guided vehicle to maximize its turn rate while maintaining altitude and speed.

```cpp
using MX = casadi::MX;
janus::Opti opti;

// 1. Define Variables
auto lift   = opti.variable(10000.0); // Lift [N]
auto bank   = opti.variable(0.5);     // Bank angle [rad]
auto thrust = opti.variable(5000.0);  // Thrust [N]

// 2. Define Parameters & State
auto velocity = MX(300.0);     // 300 m/s
auto gamma    = MX(0.0);       // Level flight
auto mass     = MX(1000.0);    // 1000 kg
auto weight   = mass * 9.81;

// 3. Define Constraints
// Maintain Altitude: Vertical lift component must equal weight
// L * cos(phi) = W
opti.subject_to(lift * janus::cos(bank) == weight);

// Maintain Speed: Thrust must equal Drag
// T = D (assuming small angle of attack for drag calc)
// CD = CD0 + k * CL^2
auto rho = 0.736; // Density at 5km
auto S = 20.0;    // Wing area
auto q = 0.5 * rho * velocity * velocity;
auto CL = lift / (q * S);
auto CD = 0.02 + 0.05 * CL * CL;
auto drag = q * S * CD;

opti.subject_to(thrust == drag);

// Physical Limits
opti.subject_to(thrust <= 30000.0);
opti.subject_to(bank >= 0.0);
opti.subject_to(bank <= 1.5708); // Max 90 deg

// 4. Define Objective
// Maximize Turn Rate: chi_dot = (L * sin(phi)) / (m * v * cos(gamma))
auto chi_dot = vulcan::dynamics::chi_dot_btt(lift, mass, velocity, gamma, bank);

opti.minimize(-chi_dot); // Minimize negative turn rate

// 5. Solve
auto sol = opti.solve();
std::cout << "Optimal Bank: " << sol.value(bank) * 180.0 / M_PI << " deg\n";
```
 
### General Symbolic Workflow
 
```cpp
using MX = casadi::MX;
 
auto mass = opti.variable();
auto mass_props = MassProperties<MX>::diagonal(mass, Ixx, Iyy, Izz);
 
RigidBodyState<MX> state{
    .position = r_sym,
    .velocity_body = v_sym,
    .attitude = q_sym,
    .omega_body = omega_sym
};
 
auto derivs = compute_6dof_derivatives<MX>(state, F_sym, M_sym, mass_props);
opti.subject_to(v_next == v + derivs.velocity_dot * dt);
```

## Variable Mass

Mass properties are passed as **input** to dynamics functions, enabling:

- **Rockets**: Update mass from propulsion model each timestep
- **Aircraft**: Track fuel burn
- **Optimization**: Use mass as a symbolic variable

```cpp
// Variable mass example
for (double t = 0; t < t_final; t += dt) {
    double current_mass = initial_mass - mdot * t;
    auto props = MassProperties<double>::from_mass(current_mass);
    auto derivs = compute_6dof_derivatives(state, F, M, props);
    // ... integrate
}
```

## References

- **Translational Dynamics**: TAOS User's Manual, Section 2.2, Eq. 2-105
- **Rotational Dynamics**: Goldstein, Classical Mechanics, 3rd Ed., Chapter 5
- **Quaternion Kinematics**: Shuster (1993)

## See Also

- [Coordinate Transforms](coordinates.md) — ECEF/ECI conversions
- [Rotation Kinematics](rotations.md) — Quaternion rate utilities
- [Propulsion](propulsion.md) — Thrust and mass rate models
