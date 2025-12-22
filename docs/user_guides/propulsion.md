# Propulsion Utilities

The `vulcan/propulsion` module provides fundamental propulsion model utilities for rocketry, air-breathing, and electric propulsion systems. These functions are designed to be stateless, functional, and compatible with both numeric (`double`) and symbolic (`casadi::MX`) types for trajectory optimization.

## Features

- **Rocket Propulsion**: Tsiolkovsky rocket equation, thrust/Isp relations.
- **Altitude Compensation**: Adjusting thrust for ambient pressure.
- **Air-Breathing**: Breguet range/endurance, TSFC-based fuel flow.
- **Electric Propulsion**: Power-limited thrust, mass flow, and efficiency relations.

## Usage

Include the main header:

```cpp
#include <vulcan/propulsion/Propulsion.hpp>
```

### Rocket Fundamentals

Calculate Delta-V or required propellant:

```cpp
using namespace vulcan::propulsion::rocket;

double Isp = 300.0;
double m0 = 1000.0;
double mf = 100.0;
double Ve = exhaust_velocity(Isp); // Uses standard g0

// Calculate Delta-V
double dV = delta_v(Ve, m0, mf);

// Calculate required propellant for a maneuver
double mp_needed = propellant_mass(500.0, m0, Ve); // for 500 m/s dV
```

### Altitude Compensation

Adjust thrust based on altitude (pressure):

```cpp
using namespace vulcan::propulsion;

double F_vac = 50000.0; // 50 kN vacuum thrust
double P_atm = 101325.0; // Sea Level
double A_exit = 0.5; // Nozzle exit area

double F_sl = altitude_thrust(F_vac, P_atm, 0.0, A_exit);
```

### Air-Breathing

Estimate range using Breguet equation:

```cpp
using namespace vulcan::propulsion::air_breathing;

double V = 250.0;
double TSFC = 1.0e-4; // 1/s
double L_D = 15.0;
double W0 = 50000.0;
double W1 = 40000.0;

// Returns range in meters (if TSFC is 1/s)
double range = breguet_range(V, TSFC, L_D, W0, W1);
```

### Electric Propulsion

Model power-limited thrusters (e.g., Ion, Hall effect):

```cpp
using namespace vulcan::propulsion::electric;

double Power = 5000.0; // 5 kW
double Ve = 30000.0; // 30 km/s
double efficiency = 0.6;

double Thrust = thrust_from_power(Power, Ve, efficiency);
double mdot = mass_flow_from_power(Power, Ve, efficiency);
```

## Symbolic Compatibility

All functions are templated on `Scalar` and use `janus::` math functions, making them compatible with `casadi::MX` for optimization problems constructed via Janus.

```cpp
casadi::MX m_sym = casadi::MX::sym("m");
casadi::MX F = thrust_from_mdot(mdot_sym, Ve_sym);
```
