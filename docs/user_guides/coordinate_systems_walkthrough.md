# Vulcan Coordinate Systems Walkthrough

This guide provides a comprehensive walkthrough of the Vulcan Coordinate Systems module, demonstrating how to define Earth models, perform geodetic conversions, work with local and body frames, and calculate flight path angles. It follows the structure of `examples/coordinates/coordinate_demo.cpp`.

## 1. Earth Models

Vulcan provides standard Earth models, such as WGS84, which define the physical properties of the planet (ellipsoid parameters).

```cpp
#include <vulcan/vulcan.hpp>

// ...

auto wgs84 = vulcan::EarthModel::WGS84();
// Access properties:
// wgs84.a (semi-major axis)
// wgs84.b (semi-minor axis)
// wgs84.f (flattening)
// wgs84.omega (angular velocity)
```

## 2. Geodetic Conversions (LLA <-> ECEF)

Vulcan allows seamless conversion between Geodetic coordinates (Latitude, Longitude, Altitude) and Earth-Centered Earth-Fixed (ECEF) Cartesian coordinates.

### Defining a Location
Locations can be defined using `LLA<Scalar>`:

```cpp
using namespace vulcan;

double lon_deg = -77.0367;
double lat_deg = 38.8951;
double alt_m = 100.0;

LLA<double> lla_dc = {
    lon_deg * constants::angle::deg2rad,
    lat_deg * constants::angle::deg2rad,
    alt_m
};
```

### Conversions
Use `lla_to_ecef` and `ecef_to_lla` for conversions. The algorithms (like Vermeille's method) are robust and high-precision.

```cpp
// LLA to ECEF
Vec3<double> r_ecef = lla_to_ecef(lla_dc);

// ECEF to LLA (Round-trip)
LLA<double> lla_back = ecef_to_lla(r_ecef);
```

You can also convert ECEF to Spherical coordinates:

```cpp
Spherical<double> sph = ecef_to_spherical(r_ecef);
```

## 3. Local Tangent Frames (NED, ENU)

Local tangent planes are essential for navigation and vehicle dynamics relative to the Earth's surface.

### North-East-Down (NED)
Common in aerospace, the NED frame is defined by a reference geodetic position.

```cpp
auto ned = CoordinateFrame<double>::ned(lla_dc.lon, lla_dc.lat);

// Inspect basis vectors in ECEF frame
// ned.x_axis() -> points North
// ned.y_axis() -> points East
// ned.z_axis() -> points Down
```

### Transforming Vectors
Transform vectors between the global ECEF frame and the local NED frame:

```cpp
Vec3<double> v_ecef = ...;
// Transform ECEF velocity to NED
Vec3<double> v_ned = ned.from_ecef(v_ecef);

// Transform NED velocity back to ECEF
Vec3<double> v_ecef_back = ned.to_ecef(v_ned);
```

## 4. Earth-Centered Inertial (ECI) Frame

The ECI frame is an inertial frame used for orbital mechanics. Vulcan supports time-based rotation models to define the ECI frame relative to ECEF.

```cpp
auto rotation = ConstantOmegaRotation::from_wgs84();
double time_s = ...;

// Create ECI frame at a specific time (GMST)
auto eci = CoordinateFrame<double>::eci(rotation.gmst(time_s));
```

## 5. Body Frames and Euler Angles

The Body frame is rigidly attached to the vehicle. Vulcan uses standard 3-2-1 (Yaw-Pitch-Roll) Euler angles to define orientation relative to a navigation frame (like NED).

### Defining Body Orientation
```cpp
double yaw = ...;
double pitch = ...;
double roll = ...;

// Create body frame from Euler angles relative to NED
auto body = body_from_euler(ned, yaw, pitch, roll);
```

### Extracting Euler Angles
You can recover Euler angles from a body frame:

```cpp
Vec3<double> euler = euler_from_body(body, ned);
// euler(0) = yaw, euler(1) = pitch, euler(2) = roll
```

## 6. Flight Path Angles

Calculates the flight path angle ($\gamma$) and heading angle ($\psi$) from a velocity vector in the local frame.

```cpp
Vec3<double> v_ned = ...;
auto fpa = flight_path_angles(v_ned);
// fpa(0) = gamma (flight path angle)
// fpa(1) = psi (heading)
```

## 7. Aerodynamic Angles

Calculates angle of attack ($\alpha$) and sideslip angle ($\beta$) from the velocity vector in the body frame.

```cpp
Vec3<double> v_body = ...;
auto aero = aero_angles(v_body);
// aero(0) = alpha
// aero(1) = beta
```

## 8. Non-Inertial Accelerations

When working in rotating frames (like ECEF), you often need to account for Coriolis and Centrifugal accelerations.

```cpp
Vec3<double> r_ecef = ...;
Vec3<double> v_ecef = ...;

// Calculate individual components or total
auto a_coriolis = coriolis_acceleration(v_ecef);
auto a_centrifugal = centrifugal_acceleration(r_ecef);
auto a_total = coriolis_centrifugal(r_ecef, v_ecef);
```

## 9. Symbolic Coordinate Operations

All coordinate transformations in Vulcan are templated on `Scalar`, making them fully compatible with Janus's symbolic engine. This enables automatic differentiation and code generation for complex GNC algorithms.

### Symbolic Computation Example

You can use `janus::SymbolicScalar` (which aliases `casadi::MX`) to build computational graphs.

```cpp
// Define symbolic inputs
SymbolicScalar sym_x = sym("x");
SymbolicScalar sym_y = sym("y");
SymbolicScalar sym_z = sym("z");

Vec3<SymbolicScalar> sym_r_ecef;
sym_r_ecef << sym_x, sym_y, sym_z;

// Perform symbolic conversion (builds the graph)
LLA<SymbolicScalar> sym_lla = ecef_to_lla(sym_r_ecef);

// Create a callable function
janus::Function f("ecef_to_lla", 
                  {sym_x, sym_y, sym_z}, 
                  {sym_lla.lon, sym_lla.lat, sym_lla.alt});
```

### Visualizing the Algorithm

Janus can visualize the resulting computational graph. Below is the graph for the **Vermeille algorithm** used in `ecef_to_lla`, showing the flow from Cartesian inputs (bottom) to Geodetic outputs (top).

> [!TIP]
> **Interactive Examples** - Explore the computational graphs:
> 
> - [🔍 LLA to ECEF (X coordinate)](examples/graph_lla_to_ecef_x.html)
> - [🔍 ECEF to LLA (Vermeille algorithm)](examples/graph_ecef_to_lla_lat.html)


## Summary

The Vulcan Coordinates module provides a rigorous, type-safe (via `Scalar` templates for Janus compatibility), and comprehensive set of tools for aerospace positioning and navigation.
