# Phase 23: Comprehensive Demo Examples

Showcase Vulcan's dual numeric/symbolic capabilities through non-trivial aerospace optimization problems.

## Background

Vulcan's current examples demonstrate individual modules but lack comprehensive demos that:
1. **Integrate multiple modules** — Combine atmosphere, gravity, dynamics, aerodynamics, etc.
2. **Showcase the dual-backend paradigm** — Same physics model for numeric simulation AND symbolic optimization
3. **Produce complex computational graphs** — 100+ nodes, automatic differentiation through deep physics
4. **Solve real engineering problems** — Practical aerospace scenarios with non-trivial constraints

### Existing Pattern (from `time_optimization.cpp`, `orbital_optimization.cpp`, `wind_optimization.cpp`):

```cpp
// 1. Define templated physics model
template <typename Scalar>
Scalar physics_model(const Scalar& x, ...) {
    // Use vulcan:: and janus:: throughout
}

// 2. Numeric evaluation survey
for (auto x : range) {
    double result = physics_model<double>(x, ...);
}

// 3. Symbolic optimization
janus::Opti opti;
auto x = opti.variable(initial_guess);
auto cost = physics_model<casadi::MX>(x, ...);
opti.minimize(cost);
opti.subject_to(...);
auto sol = opti.solve();

// 4. Graph export
janus::export_graph_html(cost, "graph_name", "Title");
```

---

## Proposed Demos

### Demo 1: Rocket Ascent Trajectory Optimization (`comprehensive_ascent.cpp`)

**Problem:** Optimize the thrust-vector-controlled ascent of a rocket from launch to orbital insertion, minimizing fuel consumption while respecting atmospheric, structural, and thermal constraints.

**Modules Integrated:**
- `dynamics/RigidBody.hpp` — 6DOF equations of motion  
- `dynamics/PointMass.hpp` — 3DOF for fast coasting arcs
- `atmosphere/StandardAtmosphere.hpp` — Density, temperature, pressure
- `aerodynamics/Aerodynamics.hpp` — Dynamic pressure, Mach number
- `gravity/PointMass.hpp`, `gravity/J2.hpp` — Earth gravity with J2
- `propulsion/Rocket.hpp` — Thrust, Isp, mass flow
- `mass/MassProperties.hpp` — Variable mass tracking
- `coordinates/ECEF.hpp`, `coordinates/NED.hpp` — Frame transformations

**Physics Model:**

```cpp
template <typename Scalar>
AscentDerivatives<Scalar> ascent_dynamics(
    const AscentState<Scalar>& state,
    const AscentControls<Scalar>& controls,
    const AscentParams& params) 
{
    // 1. Atmosphere
    auto [rho, T, p] = vulcan::standard_atmosphere::properties(state.altitude);
    
    // 2. Aerodynamics
    auto mach = vulcan::aerodynamics::mach_number(state.velocity, state.altitude);
    auto q = vulcan::aerodynamics::dynamic_pressure(rho, state.velocity);
    auto cd = vulcan::aerodynamics::drag_coefficient_supersonic(mach);  // Simplified
    auto drag = q * params.Sref * cd;
    
    // 3. Gravity (J2)
    auto g_ecef = vulcan::gravity::j2_acceleration(state.r_ecef);
    
    // 4. Propulsion
    auto [thrust, mdot] = vulcan::propulsion::rocket_thrust(
        params.Isp_vac, params.Isp_sl, params.mdot_max, p, controls.throttle);
    
    // 5. Forces
    Vec3<Scalar> F_thrust = controls.gimbal_quat.rotate(Vec3<Scalar>{thrust, 0, 0});
    Vec3<Scalar> F_aero = -drag * velocity_direction(state.velocity);
    
    // 6. Mass properties (variable)
    auto mass_props = vulcan::mass::MassProperties<Scalar>::solid_cylinder(
        state.mass, params.radius, params.length);
    
    // 7. Dynamics
    return vulcan::dynamics::compute_6dof_derivatives(
        state.rigid_body, F_thrust + F_aero, moment, mass_props);
}
```

**Optimization Problem:**
- **Objective:** Minimize fuel (maximize final mass)
- **Constraints:**
  - Max Q ≤ 40 kPa
  - Max load factor ≤ 5g
  - Final altitude = 185 km
  - Final velocity = orbital
  - Gimbal angle limits

**Demo Structure:**
1. Numeric trajectory simulation (forward integration)
2. Parameter sweep (gravity turn angle vs fuel)
3. Symbolic optimization of gravity turn initiation time
4. Graph export of dynamics pipeline

---

### Demo 2: Hypersonic Glide Vehicle Energy Management (`hypersonic_reentry.cpp`)

**Problem:** Optimize the bank angle schedule of a hypersonic glide vehicle during atmospheric reentry to maximize cross-range while respecting thermal and structural limits.

**Modules Integrated:**
- `dynamics/Guided5Dof.hpp` — Bank-to-turn dynamics
- `atmosphere/ExponentialAtmosphere.hpp` — High altitude exponential model
- `aerodynamics/Aerodynamics.hpp` — Hypersonic L/D
- `gravity/PointMass.hpp` — Spherical Earth
- `coordinates/ECEF.hpp`, `coordinates/LLA.hpp` — Geodetic conversions
- `transfer_functions/FirstOrder.hpp` — Actuator lag

**Physics Model:**

```cpp
template <typename Scalar>
ReentryDerivatives<Scalar> reentry_dynamics(
    const ReentryState<Scalar>& state,  // [r, V, gamma, chi, lat, lon]
    const Scalar& bank_cmd,
    const ReentryParams& params) 
{
    // 1. Atmosphere (exponential for high altitude)
    Scalar rho = vulcan::exponential_atmosphere::density(state.altitude);
    
    // 2. Hypersonic aero (constant L/D approximation)
    Scalar q = vulcan::aerodynamics::dynamic_pressure(rho, state.velocity);
    Scalar L = q * params.Sref * params.CL;
    Scalar D = q * params.Sref * params.CD;
    
    // 3. Bank angle with actuator lag
    auto [bank, bank_dot] = vulcan::transfer_functions::first_order(
        state.bank, bank_cmd, params.bank_tau);
    
    // 4. 5-DOF dynamics
    auto [v_dot, gamma_dot, chi_dot] = vulcan::dynamics::btt_derivatives(
        state.velocity, state.gamma, state.chi, bank,
        L, D, state.mass, g, state.altitude);
    
    // 5. Position kinematics (spherical Earth)
    Scalar lat_dot = state.velocity * janus::cos(state.gamma) * janus::cos(state.chi) / (R + state.altitude);
    Scalar lon_dot = state.velocity * janus::cos(state.gamma) * janus::sin(state.chi) / 
                     ((R + state.altitude) * janus::cos(state.latitude));
    
    return {v_dot, gamma_dot, chi_dot, lat_dot, lon_dot, bank_dot};
}
```

**Optimization Problem:**
- **Objective:** Maximize cross-range (final longitude difference)
- **Constraints:**
  - Stagnation point heating rate ≤ 2 MW/m²
  - Dynamic pressure ≤ 50 kPa
  - Load factor ≤ 2.5g
  - Bank angle ±80°
  - Final altitude = 30 km (pullout)

**Demo Structure:**
1. Reference equilibrium glide trajectory (numeric)
2. Heating rate profile comparison
3. Bank reversal optimization (timing of bank sign changes)
4. Export thermal/load constraint graphs

---

### Demo 3: Multi-Stage Separation Dynamics (`stage_separation.cpp`)

**Problem:** Optimize the separation sequence of a two-stage rocket to minimize clearance time while ensuring safe separation distance.

**Modules Integrated:**
- `dynamics/RigidBody.hpp` — 6DOF for both stages
- `dynamics/RigidBodyTypes.hpp` — State packing
- `mass/MassProperties.hpp` — Stage mass aggregation
- `propulsion/Rocket.hpp` — First stage shutdown, second stage ignition
- `atmosphere/StandardAtmosphere.hpp` — Residual aerodynamics
- `geometry/Cylinder.hpp` — Collision detection geometry
- `rotations/Quaternion.hpp` — Attitude representation

**Physics Model:**

```cpp
template <typename Scalar>
SeparationDerivatives<Scalar> separation_dynamics(
    const TwoBodyState<Scalar>& state,  // 6DOF × 2 bodies
    const SeparationControls<Scalar>& controls,
    const SeparationParams& params)
{
    // 1. Mass properties for each stage
    auto mp1 = vulcan::mass::MassProperties<Scalar>::solid_cylinder(
        params.m1, params.r1, params.h1);
    auto mp2 = vulcan::mass::MassProperties<Scalar>::solid_cylinder(
        params.m2, params.r2, params.h2);
    
    // 2. Separation spring force
    Vec3<Scalar> rel_pos = state.body2.position - state.body1.position;
    Scalar dist = janus::norm(rel_pos);
    Vec3<Scalar> F_spring = controls.spring_force * rel_pos / dist;
    
    // 3. Retro rockets on first stage
    Vec3<Scalar> F_retro = state.body1.attitude.inverse().rotate(
        Vec3<Scalar>{-controls.retro_thrust, 0, 0});
    
    // 4. Aerodynamic interference (simplified)
    Scalar rho = vulcan::standard_atmosphere::density(state.altitude);
    Vec3<Scalar> F_aero1 = separation_aero(state.body1, rho, params);
    Vec3<Scalar> F_aero2 = separation_aero(state.body2, rho, params);
    
    // 5. Dynamics for each body
    auto deriv1 = vulcan::dynamics::compute_6dof_derivatives(
        state.body1, F_aero1 + F_retro - F_spring, M1, mp1);
    auto deriv2 = vulcan::dynamics::compute_6dof_derivatives(
        state.body2, F_aero2 + F_spring + F_thrust2, M2, mp2);
    
    return {deriv1, deriv2};
}
```

**Optimization Problem:**
- **Objective:** Minimize time to reach 10m separation
- **Variables:** Spring force, retro thrust, delay timing
- **Constraints:**
  - Minimum separation velocity > 2 m/s
  - No collision (distance > sum of radii)
  - Attitude rates bounded
  - Second stage ignition after safe distance

**Demo Structure:**
1. Nominal separation simulation (numeric)
2. Monte Carlo analysis (with variations)
3. Robust separation timing optimization
4. Collision probability computation

---

### Demo 4: Fuel-Optimal Lunar Transfer (`lunar_transfer.cpp`)

**Problem:** Design a fuel-optimal trajectory from LEO to lunar orbit insertion, using Vulcan's orbital mechanics and gravity models.

**Modules Integrated:**
- `orbital/Keplerian.hpp` — Orbital elements, state propagation
- `orbital/Transfer.hpp` — Hohmann, bielliptic, Lambert
- `orbital/Ephemeris.hpp` — Moon position
- `gravity/PointMass.hpp` — Earth and Moon gravity (patched conic)
- `time/Epoch.hpp` — Time conversions, TDB
- `coordinates/ECEF.hpp`, `coordinates/ECI.hpp` — Frame transforms

**Physics Model:**

```cpp
template <typename Scalar>
TransferCost<Scalar> compute_lunar_transfer_cost(
    const Scalar& departure_jd,
    const Scalar& tof,  // Time of flight
    const LunarTransferParams& params)
{
    // 1. Departure state (LEO)
    auto r_dep = params.r_leo;  // Position on reference orbit
    auto v_circ = vulcan::orbital::circular_velocity(r_dep);
    
    // 2. Arrival epoch and Moon state
    Scalar arrival_jd = departure_jd + tof / 86400.0;
    auto r_moon = vulcan::ephemeris::analytical::moon_position_eci(arrival_jd);
    
    // 3. Lambert solver for transfer arc
    auto [v1, v2] = vulcan::orbital::lambert_solve(
        params.r_leo_vec, r_moon, tof);
    
    // 4. Delta-V at departure
    Scalar dv1 = janus::norm(v1 - params.v_leo_vec);
    
    // 5. Moon-relative arrival velocity
    auto v_moon = vulcan::ephemeris::analytical::moon_velocity_eci(arrival_jd);
    auto v_rel = v2 - v_moon;
    
    // 6. LOI burn (circularize at Moon)
    Scalar v_inf = janus::norm(v_rel);
    Scalar dv2 = vulcan::orbital::hyperbolic_to_circular(
        v_inf, params.r_lunar_orbit, vulcan::constants::moon::mu);
    
    return {dv1, dv2, dv1 + dv2};
}
```

**Optimization Problem:**
- **Objective:** Minimize total ΔV
- **Variables:** Departure date, time of flight, parking orbit phase
- **Constraints:**
  - Transfer time 2.5-5 days
  - Arrival at favorable lunar phase
  - Within launch window

**Demo Structure:**
1. Porkchop plot generation (numeric sweep)
2. Multi-start optimization for global minimum
3. Sensitivity analysis (departure date tolerance)
4. Compute C3 and TLI requirements

---

### Demo 5: Slosh-Coupled Roll Control (`slosh_stability.cpp`)

**Problem:** Design a roll control system for a rocket with coupled fuel slosh dynamics, demonstrating the slosh models integrated with rigid body dynamics.

**Modules Integrated:**
- `dynamics/RigidBody.hpp` — 6DOF rocket body
- `dynamics/Slosh.hpp` — Pendulum slosh model
- `dynamics/Oscillator1Dof.hpp` — Engine gimbal dynamics
- `mass/MassProperties.hpp` — Time-varying mass
- `transfer_functions/SecondOrder.hpp` — Autopilot
- `propulsion/Rocket.hpp` — Thrust

**Physics Model:**

```cpp
template <typename Scalar, int NumTanks = 2>
CoupledDynamics<Scalar> slosh_coupled_dynamics(
    const RocketState<Scalar>& rocket,
    const std::array<SloshState<Scalar>, NumTanks>& slosh,
    const GimbalState<Scalar>& gimbal,
    const Scalar& gimbal_cmd,
    const SloshRocketParams& params)
{
    // 1. Total mass properties (rigid + slosh contribution)
    auto mp_rigid = vulcan::mass::MassProperties<Scalar>::solid_cylinder(
        params.m_dry + params.m_fuel, params.radius, params.length);
    
    auto [slosh_derivs, slosh_force, slosh_moment] = 
        vulcan::dynamics::compute_pendulum_slosh(
            slosh[0], params.slosh_params[0], rocket.accel_body);
    
    // Add slosh reaction to rigid body
    Vec3<Scalar> moment_total = thrust_moment + slosh_moment;
    
    // 2. Gimbal dynamics (second order lag)
    auto gimbal_derivs = vulcan::dynamics::compute_actuator_dynamics(
        gimbal, gimbal_cmd, params.gimbal_params);
    
    // 3. Rigid body with slosh coupling
    auto rocket_derivs = vulcan::dynamics::compute_6dof_derivatives(
        rocket.rigid_body, force, moment_total, mp_rigid);
    
    return {rocket_derivs, slosh_derivs, gimbal_derivs};
}
```

**Optimization Problem:**
- **Objective:** Minimize roll error integral
- **Variables:** Autopilot gains (Kp, Kd, anti-slosh filters)
- **Constraints:**
  - Slosh amplitude bounded
  - Gimbal rate limits
  - Stability margin (symbolic if possible)

**Demo Structure:**
1. Open-loop slosh response (numeric)
2. Closed-loop step response comparison
3. Gain optimization for slosh suppression
4. Bode plot generation (linearized model export)

---

## File Structure

```
examples/
├── comprehensive/           # NEW
│   ├── comprehensive_ascent.cpp     # Demo 1
│   ├── hypersonic_reentry.cpp       # Demo 2
│   ├── stage_separation.cpp         # Demo 3
│   ├── lunar_transfer.cpp           # Demo 4
│   └── slosh_stability.cpp          # Demo 5
```

---

## Implementation Checklist

### Demo 1: Rocket Ascent (`comprehensive_ascent.cpp`)
- [ ] Templated `ascent_dynamics()` function
- [ ] Numeric forward integration with RK4
- [ ] Parameter sweep for gravity turn angle
- [ ] `janus::Opti` for gravity turn timing optimization
- [ ] Graph export for drag, thrust, and acceleration chains

### Demo 2: Hypersonic Reentry (`hypersonic_reentry.cpp`)
- [ ] Templated `reentry_dynamics()` function
- [ ] Equilibrium glide reference trajectory
- [ ] Heating rate computation
- [ ] Bank angle schedule optimization
- [ ] Thermal and load factor constraint graphs

### Demo 3: Stage Separation (`stage_separation.cpp`)
- [ ] Templated `separation_dynamics()` for 2 rigid bodies
- [ ] Collision avoidance constraint
- [ ] Spring/retro thrust optimization
- [ ] Monte Carlo clearance analysis

### Demo 4: Lunar Transfer (`lunar_transfer.cpp`)
- [ ] Templated Lambert solver with Moon ephemeris
- [ ] Porkchop plot generation
- [ ] Multi-start optimization
- [ ] C3 and TLI ΔV analysis

### Demo 5: Slosh Stability (`slosh_stability.cpp`)
- [ ] Coupled rigid-slosh dynamics
- [ ] Autopilot gain tuning
- [ ] Slosh suppression demonstration
- [ ] Linearized model export for Bode analysis

### Build System
- [ ] Create `examples/comprehensive/` directory
- [ ] Update `examples/CMakeLists.txt` with new subdirectory
- [ ] Create `examples/comprehensive/CMakeLists.txt`

---

## Verification Plan

### Build Verification
```bash
./scripts/ci.sh
cd build && ctest -R comprehensive -V
```

### Manual Verification
- [ ] All demos compile and run without errors
- [ ] All demos produce HTML graph exports
- [ ] Optimization results are physically reasonable
- [ ] Console output is clean and informative

### Expected Graph Complexity
- Demo 1: ~150+ nodes (atmosphere + aero + gravity + propulsion + dynamics)
- Demo 2: ~100+ nodes (reentry dynamics + constraints)
- Demo 3: ~200+ nodes (dual rigid body + separation physics)
- Demo 4: ~80+ nodes (Lambert + ephemeris + orbital mechanics)
- Demo 5: ~120+ nodes (slosh + gimbal + rigid body coupling)

---

## Design Decisions

1. **Single templated physics function** — Each demo defines ONE templated function that encapsulates all physics, enabling both numeric simulation and symbolic optimization.

2. **Consistent structure** — All demos follow the pattern: (1) Numeric survey, (2) Symbolic optimization, (3) Graph export.

3. **Real-world relevance** — Problems are based on actual aerospace engineering analyses (NASA references, textbook scenarios).

4. **Progressive complexity** — Demos increase in module integration complexity, showcasing Vulcan's composability.

5. **Educational value** — Each demo includes comments explaining the physics and engineering rationale.

---

## References

- SMAD (Space Mission Analysis and Design) — Trajectory and propulsion formulas
- Vinh, "Flight Mechanics of High-Performance Aircraft" — Hypersonic glide equations
- NASA SP-8009 — Slosh model parameters
- Battin, "Astronautical Guidance" — Lambert problem formulation
