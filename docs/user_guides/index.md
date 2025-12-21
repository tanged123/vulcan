# Vulcan User Guides

Welcome to the Vulcan user guides. These documents provide detailed usage instructions for each module with code examples, API references, and links to interactive demos.

## Modules

| Guide | Description |
|-------|-------------|
| [Atmosphere](atmosphere.md) | US Standard Atmosphere 1976 - temperature, pressure, density |
| [Coordinates](coordinates.md) | ECEF, LLA, NED frames and geodetic conversions |
| [Rotations](rotations.md) | Euler angles, quaternions, DCMs, axis-angle |
| [Time](time.md) | Epochs, time scales (TAI, TT, UTC, GPS), leap seconds |
| [Wind](wind.md) | Wind shear, Dryden and von Kármán turbulence models |
| [Gravity](gravity.md) | Point mass, J2, J2-J4 gravity models |
| [Aerodynamics](aerodynamics.md) | Dynamic pressure, Mach, Reynolds number |
| [Sensors](sensors.md) | IMU noise models - Allan variance, Markov, random walk |
| [RNG](rng.md) | Random number generation utilities for Monte Carlo |

## Quick Links

### Examples

All examples are in the `examples/` directory:

```bash
./scripts/build.sh
./build/examples/atmosphere_profile
./build/examples/coordinate_demo
./build/examples/rotations_demo
./build/examples/time_optimization
./build/examples/wind_optimization
./build/examples/gravity_demo
./build/examples/aerodynamics_demo
./build/examples/sensor_noise_demo
```

### Interactive Graphs

Explore symbolic computational graphs in your browser:

- [Wind Profile (Power-Law)](../examples/graph_wind_profile.html)
- [DCM Element R[0,0]](../examples/graph_dcm_00.html)
- [LLA to ECEF Conversion](../examples/graph_lla_to_ecef_x.html)
- [Solar Elevation Model](../examples/graph_solar_elevation.html)

## Getting Started

If you're new to Vulcan, start with the [Getting Started example](../../examples/intro/getting_started.cpp) which demonstrates:

1. Basic atmospheric queries
2. Coordinate transformations
3. Symbolic computation with Janus

```cpp
#include <vulcan/vulcan.hpp>

// All Vulcan modules available via single include
auto atm = vulcan::ussa1976::state(10000.0);
auto wind = vulcan::wind_shear::power_law(100.0, 10.0, 10.0);
```
