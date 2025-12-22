# Vulcan Orbital Mechanics User Guide

This guide covers orbital mechanics utilities in Vulcan: state conversions, Kepler solvers, transfer calculations, and Sun/Moon ephemeris—all with full symbolic compatibility.

## Quick Start

```cpp
#include <vulcan/orbital/Orbital.hpp>

using namespace vulcan;
using namespace vulcan::orbital;

// Orbital period for ISS
double T = quantities::period(6778.0e3);  // ~92 minutes

// Hohmann transfer LEO to GEO
auto [dv1, dv2] = transfer::hohmann_delta_v(6678.0e3, 42164.0e3);

// Sun position
auto r_sun = ephemeris::analytical::sun_position_eci(jd);
```

## Orbital Quantities

Basic two-body orbital computations:

```cpp
double r = constants::earth::R_eq + 400.0e3;  // 400 km altitude

// Period (Kepler's third law)
double T = quantities::period(7000.0e3);  // ~92 minutes for a = 7000 km

// Vis-viva equation
double v = quantities::velocity(r_peri, a);  // Velocity at given radius

// Circular orbit velocity
double v_circ = quantities::circular_velocity(r);  // ~7.67 km/s at 400 km

// Escape velocity (√2 × circular)
double v_esc = quantities::escape_velocity(r);

// Specific orbital energy
double E = quantities::energy(a);  // Negative for bound orbits

// Mean motion
double n = quantities::mean_motion(a);  // rad/s
```

## Anomaly Conversions

Solve Kepler's equation and convert between anomaly types:

```cpp
double e = 0.5;  // Eccentricity
double M = 1.0;  // Mean anomaly [rad]

// Kepler's equation: M = E - e·sin(E)
double E = anomaly::mean_to_eccentric(M, e);

// Eccentric to True anomaly
double nu = anomaly::eccentric_to_true(E, e);

// Inverse conversions
double E2 = anomaly::true_to_eccentric(nu, e);
double M2 = anomaly::eccentric_to_mean(E2, e);

// Direct conversions
double nu = anomaly::mean_to_true(M, e);
double M = anomaly::true_to_mean(nu, e);
```

> [!TIP]
> The Kepler solver uses Newton-Raphson iteration and converges to machine precision for all eccentricities < 1.

## State Conversions

Transform between Cartesian and Keplerian elements:

```cpp
// Cartesian state (ECI)
Vec3<double> r, v;
r << 6678.0e3, 0.0, 0.0;
v << 0.0, 7.73e3, 0.0;

// Convert to Keplerian elements
auto oe = elements::cartesian_to_keplerian(r, v);
// Returns: OrbitalElements{a, e, i, Omega, omega, nu}

std::cout << "Semi-major axis: " << oe.a << " m\n";
std::cout << "Eccentricity: " << oe.e << "\n";
std::cout << "Inclination: " << oe.i * 180/M_PI << "°\n";

// Convert back to Cartesian
auto [r2, v2] = elements::keplerian_to_cartesian(oe);
```

### OrbitalElements Structure

```cpp
template <typename Scalar>
struct OrbitalElements {
    Scalar a;      // Semi-major axis [m]
    Scalar e;      // Eccentricity [-]
    Scalar i;      // Inclination [rad]
    Scalar Omega;  // RAAN [rad]
    Scalar omega;  // Argument of periapsis [rad]
    Scalar nu;     // True anomaly [rad]
};
```

## Transfer Mechanics

Compute Δv for orbital maneuvers:

### Hohmann Transfer

```cpp
double r_leo = 6678.0e3;   // 300 km altitude
double r_geo = 42164.0e3;  // GEO

auto [dv1, dv2] = transfer::hohmann_delta_v(r_leo, r_geo);
double total_dv = transfer::hohmann_total_delta_v(r_leo, r_geo);
double t_transfer = transfer::hohmann_transfer_time(r_leo, r_geo);

std::cout << "Δv₁ = " << dv1/1000 << " km/s\n";  // ~2.4 km/s
std::cout << "Δv₂ = " << dv2/1000 << " km/s\n";  // ~1.5 km/s
std::cout << "Transfer time = " << t_transfer/3600 << " hours\n";  // ~5 hr
```

### Bielliptic Transfer

More efficient than Hohmann for r₂/r₁ > 11.94:

```cpp
double r_b = 200000.0e3;  // Intermediate apoapsis

auto [dv1, dv2, dv3] = transfer::bielliptic_delta_v(r1, r2, r_b);
double total = dv1 + dv2 + dv3;
```

### Plane Change

```cpp
double v = quantities::circular_velocity(r_geo);
double delta_i = 28.5 * M_PI / 180;  // Cape Canaveral to equatorial

double dv = transfer::plane_change_delta_v(v, delta_i);
// Plane change at GEO is expensive: ~1.8 km/s for 28.5°
```

### Combined Maneuver

```cpp
// Single impulse for altitude + plane change
double dv = transfer::combined_maneuver_delta_v(v1, v2, delta_i);
```

## Analytical Ephemeris

Sun and Moon positions using Meeus/Vallado algorithms:

```cpp
double jd = 2451545.0;  // J2000 epoch

// Sun position in ECI (J2000 equatorial)
auto r_sun = ephemeris::analytical::sun_position_eci(jd);
double dist_au = janus::norm(r_sun) / constants::sun::AU;  // ~1.0 AU

// Moon position in ECI
auto r_moon = ephemeris::analytical::moon_position_eci(jd);
double dist_km = janus::norm(r_moon) / 1000;  // ~384,000 km

// ECEF positions (for ground-based applications)
auto r_sun_ecef = ephemeris::analytical::sun_position_ecef(jd);
auto r_moon_ecef = ephemeris::analytical::moon_position_ecef(jd);

// Sun right ascension and declination
auto [ra, dec] = ephemeris::analytical::sun_ra_dec(jd);

// Sun distance only
double r = ephemeris::analytical::sun_distance(jd);

// Unit vector toward Sun
auto u_sun = ephemeris::analytical::sun_unit_vector_eci(jd);
```

### Ephemeris Accuracy

| Body | Position Error | Valid Range |
|------|----------------|-------------|
| Sun  | ~0.01° (~1 arcmin) | 1950–2050 |
| Moon | ~0.3° (~10 km) | 1950–2050 |

> [!NOTE]
> For higher precision applications (navigation, conjunction analysis), use JPL DE ephemeris (not yet implemented).

## Celestial Body Constants

```cpp
vulcan::constants::sun::AU           // 149597870700.0 m (exact IAU)
vulcan::constants::sun::mu           // 1.32712440018e20 m³/s²
vulcan::constants::sun::radius       // 6.96e8 m

vulcan::constants::moon::mu          // 4.9028695e12 m³/s²
vulcan::constants::moon::radius      // 1.7374e6 m
vulcan::constants::moon::mean_distance  // 3.844e8 m
```

## Symbolic Computation

All functions work with `janus::SymbolicScalar` for optimization:

```cpp
auto a = janus::sym("a");
auto e = janus::sym("e");

// Symbolic period
auto T = quantities::period(a);

// Symbolic Kepler solver (uses fixed iterations for autodiff)
auto M = janus::sym("M");
auto E = anomaly::mean_to_eccentric(M, e);

// Create CasADi function
janus::Function f("kepler", {M, e}, {E});
auto result = f({1.0, 0.5});
```

### Symbolic State Conversions

```cpp
OrbitalElements<janus::SymbolicScalar> oe;
oe.a = janus::sym("a");
oe.e = janus::sym("e");
oe.i = janus::sym("i");
oe.Omega = janus::sym("Omega");
oe.omega = janus::sym("omega");
oe.nu = janus::sym("nu");

auto [r, v] = elements::keplerian_to_cartesian(oe);
// r and v are now symbolic expressions
```

## API Reference

### quantities namespace

| Function | Description |
|----------|-------------|
| `period(a, [mu])` | Orbital period [s] |
| `velocity(r, a, [mu])` | Vis-viva velocity [m/s] |
| `circular_velocity(r, [mu])` | Circular orbit velocity |
| `escape_velocity(r, [mu])` | Escape velocity |
| `energy(a, [mu])` | Specific orbital energy [J/kg] |
| `mean_motion(a, [mu])` | Mean motion [rad/s] |

### anomaly namespace

| Function | Description |
|----------|-------------|
| `mean_to_eccentric(M, e)` | Solve Kepler's equation |
| `eccentric_to_true(E, e)` | E → ν conversion |
| `true_to_eccentric(nu, e)` | ν → E conversion |
| `eccentric_to_mean(E, e)` | E → M conversion |
| `mean_to_true(M, e)` | M → ν direct |
| `true_to_mean(nu, e)` | ν → M direct |

### elements namespace

| Function | Description |
|----------|-------------|
| `cartesian_to_keplerian(r, v, [mu])` | State → Elements |
| `keplerian_to_cartesian(oe, [mu])` | Elements → State |

### transfer namespace

| Function | Description |
|----------|-------------|
| `hohmann_delta_v(r1, r2, [mu])` | Returns (Δv₁, Δv₂) |
| `hohmann_total_delta_v(r1, r2, [mu])` | Total Hohmann Δv |
| `hohmann_transfer_time(r1, r2, [mu])` | Transfer time [s] |
| `bielliptic_delta_v(r1, r2, rb, [mu])` | Returns (Δv₁, Δv₂, Δv₃) |
| `plane_change_delta_v(v, delta_i)` | Inclination change Δv |
| `combined_maneuver_delta_v(v1, v2, delta_i)` | Combined Δv |

### ephemeris::analytical namespace

| Function | Description |
|----------|-------------|
| `sun_position_eci(jd)` | Sun position [m] |
| `sun_distance(jd)` | Earth-Sun distance [m] |
| `sun_ra_dec(jd)` | Sun RA and Dec [rad] |
| `sun_unit_vector_eci(jd)` | Unit vector to Sun |
| `sun_position_ecef(jd)` | Sun in ECEF [m] |
| `moon_position_eci(jd)` | Moon position [m] |
| `moon_distance(jd)` | Earth-Moon distance [m] |
| `moon_position_ecef(jd)` | Moon in ECEF [m] |

## Example

See `examples/orbital/orbital_demo.cpp` and `examples/orbital/orbital_optimization.cpp` for a comprehensive demonstration:

```bash
./scripts/build.sh
./build/examples/orbital_demo
```
