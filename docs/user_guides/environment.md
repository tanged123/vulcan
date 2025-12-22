# Environment Utilities

The environment module provides space environment models for satellite and mission analysis:

- **Solar Position** - Approximate Sun ephemeris (RA/Dec, distance, ECI position)
- **Eclipse Detection** - Cylindrical and conical shadow models
- **Magnetic Field** - Centered dipole geomagnetic field model

All functions are templated for both numeric (`double`) and symbolic (`casadi::MX`) evaluation.

---

## Quick Start

```cpp
#include <vulcan/environment/Environment.hpp>

using namespace vulcan::environment;

// Solar position at J2000.0
double jd = vulcan::time::calendar_to_jd(2000, 1, 1, 12, 0, 0.0);
auto [ra, dec] = solar::ra_dec(jd);
auto sun_pos = solar::position_eci(jd);

// Eclipse check
Vec3<double> r_sat, r_sun;
r_sat << -7000e3, 0.0, 0.0;  // Behind Earth
r_sun << 1.5e11, 0.0, 0.0;   // Sun at 1 AU
double illumination = eclipse::shadow_cylindrical(r_sat, r_sun);  // 0.0 = shadow

// Magnetic field
double B = magnetic::field_magnitude(r_sat);  // [Tesla]
```

---

## API Reference

### Solar Position (`vulcan::environment::solar`)

| Function | Description |
|----------|-------------|
| `ra_dec(jd)` | Sun right ascension and declination [rad] |
| `right_ascension(jd)` | Sun right ascension [rad] |
| `declination(jd)` | Sun declination [rad] |
| `distance(jd)` | Earth-Sun distance [m] |
| `position_eci(jd)` | Sun position vector in ECI [m] |
| `unit_vector_eci(jd)` | Sun direction unit vector in ECI |

**Algorithm**: Low-precision ephemeris from Vallado's "Fundamentals of Astrodynamics" (~0.01° accuracy over decades).

#### Example: Seasonal Variation

```cpp
// Summer solstice 2024
double jd = vulcan::time::calendar_to_jd(2024, 6, 21, 12, 0, 0.0);
auto [ra, dec] = solar::ra_dec(jd);

// Declination at summer solstice ≈ +23.44°
std::cout << "Declination: " << dec * 180/M_PI << "°\n";
```

---

### Eclipse Detection (`vulcan::environment::eclipse`)

| Function | Description |
|----------|-------------|
| `shadow_cylindrical(r_sat, r_sun)` | Cylindrical shadow model (0=shadow, 1=sunlit) |
| `shadow_conical(r_sat, r_sun)` | Conical model with penumbra transition |
| `is_in_shadow(r_sat, r_sun)` | Returns 1.0 if in shadow |
| `is_sunlit(r_sat, r_sun)` | Returns 1.0 if sunlit |

All functions accept optional `R_body` parameter for custom shadow-casting body radius.

#### Shadow Models

**Cylindrical Model** (fastest):
- Simple shadow cylinder behind Earth
- Good for quick checks and optimization

**Conical Model** (accurate):
- Accounts for finite Sun size
- Includes penumbra transition (0-1 range)
- Based on Montenbruck & Gill "Satellite Orbits"

```cpp
Vec3<double> r_sat, r_sun;
r_sat << -7000e3, 100e3, 0.0;  // Near shadow edge
r_sun << 1.5e11, 0.0, 0.0;

double nu_cyl = eclipse::shadow_cylindrical(r_sat, r_sun);  // Binary
double nu_con = eclipse::shadow_conical(r_sat, r_sun);      // Smooth transition
```

---

### Magnetic Field (`vulcan::environment::magnetic`)

| Function | Description |
|----------|-------------|
| `dipole_field_ecef(r_ecef)` | B-field vector in ECEF [T] |
| `field_magnitude(r_ecef)` | Field magnitude [T] |
| `field_ned(lat, lon, alt)` | B-field in NED frame [T] |
| `surface_intensity(lat)` | Surface field intensity [T] |
| `inclination(lat)` | Magnetic inclination angle [rad] |

#### Dipole Model

The centered dipole model approximates Earth's magnetic field:

- **B₀ ≈ 31.2 μT** at equator surface
- **~62 μT** at poles (2× equator)
- Field decays as **r⁻³** with altitude

```cpp
// Field at 400 km LEO altitude
Vec3<double> r;
r << vulcan::constants::earth::R_eq + 400e3, 0.0, 0.0;
double B = magnetic::field_magnitude(r);  // ~25 μT

// NED components at 45° latitude
auto B_ned = magnetic::field_ned(45.0 * deg2rad, 0.0, 0.0);
// B_ned[0] = North, B_ned[1] = East, B_ned[2] = Down
```

> [!NOTE]
> This is a simplified centered dipole model. For high-accuracy geomagnetic applications (navigation, attitude control), consider IGRF or WMM models.

---

## Symbolic Mode

All environment functions work with Janus symbolic types for automatic differentiation:

```cpp
auto jd = janus::sym("jd");
auto dec = solar::declination(jd);
auto ddec_djd = janus::jacobian(dec, jd);  // Rate of change

janus::Function f("dec_rate", {jd}, {ddec_djd});
auto result = f({2451545.0});  // Evaluate at J2000
```

### Example: Eclipse Optimization

Use shadow function in trajectory optimization:

```cpp
auto x = janus::sym("x");
auto y = janus::sym("y");
auto z = janus::sym("z");

Vec3<janus::SymbolicScalar> r_sat, r_sun;
r_sat << x, y, z;
r_sun << janus::SymbolicScalar(1.5e11), 
         janus::SymbolicScalar(0.0), 
         janus::SymbolicScalar(0.0);

auto nu = eclipse::shadow_cylindrical(r_sat, r_sun);

// Use nu in cost function: maximize illumination time
// CasADi will compute gradients automatically
```

---

## Constants

Available in `vulcan::environment::solar::constants`, `eclipse::constants`, `magnetic::constants`:

| Constant | Value | Description |
|----------|-------|-------------|
| `solar::constants::AU` | 1.496×10¹¹ m | Astronomical Unit |
| `eclipse::constants::R_sun` | 6.963×10⁸ m | Solar radius |
| `magnetic::constants::B0` | 3.12×10⁻⁵ T | Equatorial surface field |
| `magnetic::constants::dipole_tilt` | 11.5° | Dipole tilt angle |

---

## See Also

- [Time Systems](time.md) - Julian Date utilities for solar ephemeris
- [Coordinates](coordinates.md) - ECI/ECEF frame transformations
- [Gravity](gravity.md) - Gravitational models
