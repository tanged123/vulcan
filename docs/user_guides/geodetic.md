# Geodetic Utilities

Vulcan provides geodetic utilities for accurate distance and bearing calculations on the WGS84 ellipsoid, as well as a trajectory-relative CDA (Cross-range, Down-range, Altitude) coordinate frame.

## Quick Start

```cpp
#include <vulcan/vulcan.hpp>
using namespace vulcan;
using namespace vulcan::geodetic;

// Define locations
LLA<double> london(-0.1278_deg, 51.5074_deg, 0.0);
LLA<double> nyc(-74.0060_deg, 40.7128_deg, 0.0);

// Distance (Vincenty formula, accurate to 0.5mm)
double distance = great_circle_distance(london, nyc);  // ~5,585 km

// Initial bearing (forward azimuth)
double bearing = initial_bearing(london, nyc);  // ~288° (WNW)
```

## Distance Functions

### Haversine (Fast, Spherical)

```cpp
double d = haversine_distance(lla1, lla2);
```

- Uses spherical approximation (mean Earth radius)
- Fast computation, error < 0.3% for short distances
- Best for: Quick estimates, short distances (< 100 km)

### Vincenty (Accurate, Ellipsoidal)

```cpp
double d = great_circle_distance(lla1, lla2);
```

- Uses WGS84 ellipsoid
- Accurate to approximately 0.5 mm
- 20 fixed iterations (symbolic-compatible)
- Best for: Navigation, surveying, long distances

## Bearing Functions

### Initial Bearing

```cpp
double bearing = initial_bearing(start, end);  // radians, [0, 2π)
```

The forward azimuth at the start point, measured clockwise from North.

### Final Bearing

```cpp
double bearing = final_bearing(start, end);  // radians, [0, 2π)
```

The azimuth you'd be traveling when arriving at the destination.

> **Note**: On a great-circle route, initial and final bearings differ due to Earth's curvature.

## Destination Point (Direct Problem)

Given a start point, bearing, and distance, find the destination:

```cpp
auto dest = destination_point(start, bearing, distance);
// dest.lat, dest.lon, dest.alt
```

## Visibility & Horizon

### Horizon Distance

```cpp
double d = horizon_distance(altitude);  // meters
```

Line-of-sight distance to the geometric horizon from a given altitude.

| Altitude | Horizon |
|----------|---------|
| 1.7 m (eye level) | 4.7 km |
| 10 km (aircraft) | 357 km |
| 400 km (ISS) | 2,294 km |

### Visibility Check

```cpp
double vis = is_visible(observer_lla, target_lla);
// vis > 0: visible, vis <= 0: occluded by Earth
```

Pure geometric check (no terrain).

## Ray-Ellipsoid Intersection

```cpp
auto result = ray_ellipsoid_intersection(origin_ecef, direction);
if (result.hit > 0) {
    // result.point_near, result.point_far
    // result.t_near, result.t_far
}
```

Useful for terrain line-of-sight and satellite ground track calculations.

## CDA Frame (Trajectory-Relative)

The CDA (Cross-range, Down-range, Altitude) frame is useful for trajectory analysis:

- **D** (X-axis): Down-range, along the launch bearing
- **C** (Y-axis): Cross-range, perpendicular (right-hand)
- **A** (Z-axis): Altitude, up from surface

### Create CDA Frame

```cpp
// From bearing
auto frame = local_cda(lla_origin, bearing);

// From ECEF position
auto frame = local_cda_at(r_ecef, bearing);
```

### Coordinate Conversions

```cpp
// ECEF to CDA
Vec3<double> cda = ecef_to_cda(r_ecef, lla_ref, bearing);
// cda(0) = down-range, cda(1) = cross-range, cda(2) = altitude

// CDA to ECEF
Vec3<double> ecef = cda_to_ecef(cda, lla_ref, bearing);
```

### Example: Launch Trajectory

```cpp
// Launch site
LLA<double> ksc(-80.60_deg, 28.57_deg, 0.0);
double azimuth = 90.0_deg;  // Due East

// Point 100 km downrange, 10 km crossrange, 50 km altitude
Vec3<double> trajectory_cda;
trajectory_cda << 100000, 10000, 50000;

// Convert to geographic coordinates
Vec3<double> ecef = cda_to_ecef(trajectory_cda, ksc, azimuth);
LLA<double> lla = ecef_to_lla(ecef);
```

## Symbolic Mode

All geodetic functions are templated and work with CasADi symbolic types:

```cpp
auto lat = casadi::MX::sym("lat");
auto lon = casadi::MX::sym("lon");
auto bearing = casadi::MX::sym("bearing");

LLA<casadi::MX> pos(lon, lat, casadi::MX(0));
auto dest = destination_point(pos, bearing, casadi::MX(10000));

// Build optimizable function
casadi::Function f("dest", {lat, lon, bearing}, {dest.lat, dest.lon});
```

## API Reference

### Distance & Bearing

| Function | Description |
|----------|-------------|
| `haversine_distance(lla1, lla2)` | Spherical distance [m] |
| `great_circle_distance(lla1, lla2)` | Ellipsoidal distance (Vincenty) [m] |
| `initial_bearing(lla1, lla2)` | Forward azimuth [rad] |
| `final_bearing(lla1, lla2)` | Arrival azimuth [rad] |
| `destination_point(lla, bearing, dist)` | Endpoint of geodesic |

### Visibility

| Function | Description |
|----------|-------------|
| `horizon_distance(altitude)` | LOS distance to horizon [m] |
| `is_visible(observer, target)` | Visibility check (>0 = visible) |
| `ray_ellipsoid_intersection(origin, dir)` | Ray-surface intersection |

### CDA Frame

| Function | Description |
|----------|-------------|
| `local_cda(lla, bearing)` | CDA frame from LLA |
| `local_cda_at(ecef, bearing)` | CDA frame from ECEF |
| `ecef_to_cda(ecef, lla_ref, bearing)` | ECEF → CDA |
| `cda_to_ecef(cda, lla_ref, bearing)` | CDA → ECEF |

## See Also

- [Coordinates User Guide](coordinates.md) — LLA, ECEF, NED/ENU frames
- [Example: geodetic_demo.cpp](../examples/geodetic/geodetic_demo.cpp)
