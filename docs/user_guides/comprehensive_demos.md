# Vulcan Comprehensive Demo Guide

This guide details the comprehensive engineering examples included in `examples/comprehensive/`. These demos showcase how to combine Vulcan's modules (Dynamics, Aerodynamics, Orbital, Propulsion) with Janus's dual-mode numeric/symbolic architecture to solve complex aerospace problems.

## Overview of Demos

| Demo | File | Key Features |
| :--- | :--- | :--- |
| **Rocket Ascent** | `comprehensive_ascent.cpp` | trajectory optimization, variable mass, aerodynamic heating constraint, custom constraints |
| **Hypersonic Reentry** | `hypersonic_reentry.cpp` | bank angle modulation, thermal constraints, cross-range maximization, USSA-76 atmosphere |
| **Lunar Transfer** | `lunar_transfer.cpp` | porkchop plots, orbital mechanics, analytical ephemeris, delta-v budgeting |
| **Slosh Stability** | `slosh_stability.cpp` | coupled rigid-flexible dynamics, frequency domain filtering, autopilot gain optimization |

## 1. Rocket Ascent Trajectory (`comprehensive_ascent.cpp`)

**Goal**: Maximize payload to orbit by optimizing the pitch profile.

### Physics
- **Propulsion**: Tsiolkovsky rocket equation with gravity losses.
- **Aerodynamics**: Drag ($C_d$) and Max-Q constraints.
- **Environment**: Exponential atmosphere model.

### Key Concepts
- **Hybrid Optimization**: Uses a numeric sweep to find a good initial guess for the pitchover altitude, then symbolically optimizes the parameters.
- **Graph Export**: detailed plots of Altitude, Velocity, and Dynamic Pressure.

### Usage
```bash
./build/examples/comprehensive/comprehensive_ascent
```

## 2. Hypersonic Reentry (`hypersonic_reentry.cpp`)

**Goal**: Maximize cross-range for a reentering vehicle by modulating bank angle.

### Physics
- **Dynamics**: 3-DOF guided point mass equations (Gamma-dot, Chi-dot).
- **Thermal**: Stagnation point heating rate scaling with $\sqrt{\rho} v^3$.
- **Atmosphere**: `vulcan::atmosphere::ussa1976` for realistic density profiles.

### Key Concepts
- **Path Constraints**: Constrains maximum heating rate and dynamic pressure during the trajectory.
- **Symbolic Integration**: Demonstrates integrating complex equations of motion symbolically for gradient-based optimization.

### Usage
```bash
./build/examples/comprehensive/hypersonic_reentry
```

## 3. Lunar Transfer (`lunar_transfer.cpp`)

**Goal**: Find the minimum Delta-V transfer to the Moon for a given launch window.

### Physics
- **Orbital Mechanics**: Patched conics approximation.
- **Transfer**: Hohmann transfer approximation modified for non-coplanar/elliptical targets.
- **Ephemeris**: Analytical breakdown of Moon's position relative to Earth.

### Key Concepts
- **Porkchop Plot**: Generates specific Delta-V data across a grid of departure dates and flight times to visualize launch windows.
- **Units**: extensive use of `vulcan::orbital` constants and conversion factors.

### Usage
```bash
./build/examples/comprehensive/lunar_transfer
```

## 4. Slosh-Coupled Roll Control (`slosh_stability.cpp`)

**Goal**: Design a roll autopilot that is stable despite fuel sloshing.

### Physics
- **Coupled Dynamics**:
  - Rigid body roll dynamics ($I \ddot{\phi} = M$)
  - Pendulum slosh model ($\ddot{\theta} + 2\zeta\omega\dot{\theta} + \omega^2\theta = -\ddot{\phi}/r$)
  - Interaction moments ($M_{slosh} = m r a_{lat}$)

### Key Concepts
- **Symbolic Control Design**: Optimizes PID gains ($K_p, K_d$) to minimize settling time and slosh excitation.
- **Integral Squared Error (ISE)**: Uses a smooth integral objective function for stable symbolic gradients.
- **Frequency Domain**: Demonstrates handling of resonance modes in a time-domain simulation.

### Usage
```bash
./build/examples/comprehensive/slosh_stability
```

---

## Running the Demos

All demos can be built and run using the standard project scripts:

1.  **Build**:
    ```bash
    ./scripts/build.sh
    ```
2.  **Run Specific Demo**:
    ```bash
    ./build/examples/comprehensive/slosh_stability
    ```
3.  **View Graphs**:
    After processing, open the generated HTML files in your browser (e.g., `graph_ascent_trajectory.html`).
