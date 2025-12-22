# Geometry Primitives Module

The `vulcan::geometry` module provides spatial computation utilities for guidance, visibility, and sensor field-of-view calculations. It is designed to work seamlessly with both numeric (`double`) and symbolic (`janus::SymbolicScalar`) types for automatic differentiation in trajectory optimization.

## Overview

The module provides:
- **Line-of-Sight**: Slant range, azimuth/elevation angles, and angular rates.
- **Ray Intersections**: Sphere, plane, and cone (FOV) intersection tests.
- **Projections**: Project to plane and ground track point.

## Basic Usage

### Include
```cpp
#include <vulcan/vulcan.hpp>
// or
#include <vulcan/geometry/Geometry.hpp>
```

### Line-of-Sight Computations

```cpp
using namespace vulcan::geometry;

Vec3<double> observer(0, 0, 0);
Vec3<double> target(10000, 5000, -2000);  // NED frame

// Distance between two points
double range = slant_range(observer, target);  // ~11.36 km

// Azimuth and elevation angles
Vec2<double> angles = los_angles(observer, target);
double azimuth = angles(0);    // rad, from +X axis
double elevation = angles(1);  // rad, from XY plane
```

### LOS Angular Rates (Target Tracking)

For tracking scenarios where both observer and target are moving:

```cpp
Vec3<double> r_obs(0, 0, 0);
Vec3<double> v_obs(0, 0, 0);      // Stationary radar
Vec3<double> r_tgt(10000, 0, 0);
Vec3<double> v_tgt(0, 200, 0);    // Target moving at 200 m/s

Vec2<double> rates = los_rate(r_obs, v_obs, r_tgt, v_tgt);
double az_rate = rates(0);   // rad/s
double el_rate = rates(1);   // rad/s
```

## Ray Intersections

### Ray-Sphere Intersection

Useful for Earth intercept calculations or collision detection:

```cpp
Vec3<double> origin(0, 0, -7000000);   // Satellite position
Vec3<double> direction(0, 0, 1);       // Looking toward Earth
Vec3<double> center(0, 0, 0);          // Earth center
double radius = 6371000;               // Earth radius

double t = ray_sphere_intersection(origin, direction, center, radius);

if (t > 0) {
    Vec3<double> impact = origin + t * direction;
}
```

### Ray-Plane Intersection

For ground targeting or terrain intersection:

```cpp
Vec3<double> origin(0, 0, -5000);       // Aircraft at 5km altitude
Vec3<double> direction(0.707, 0, 0.707); // 45° dive
direction.normalize();

Vec3<double> ground_normal(0, 0, -1);   // NED: up
Vec3<double> ground_point(0, 0, 0);

double t = ray_plane_intersection(origin, direction, ground_normal, ground_point);
```

### Sensor FOV Check (Point in Cone)

Check if a target is within a sensor's field of view:

```cpp
Vec3<double> sensor_pos(0, 0, 0);
Vec3<double> sensor_axis(1, 0, 0);  // Boresight direction (normalized)
double half_angle = 15.0 * M_PI / 180.0;  // ±15° FOV

Vec3<double> target(100, 10, 0);

// Returns 1.0 if inside, 0.0 if outside (Scalar for symbolic compat)
double inside = point_in_cone(target, sensor_pos, sensor_axis, half_angle);

if (inside > 0.5) {
    // Target is visible
}
```

## Projections

### Project to Plane

```cpp
Vec3<double> point(100, 200, 500);
Vec3<double> plane_normal(0, 0, 1);  // Horizontal plane
Vec3<double> plane_point(0, 0, 0);

Vec3<double> projected = project_to_plane(point, plane_normal, plane_point);
// Result: (100, 200, 0)
```

### Ground Track Point

Project an orbital position to the Earth's surface:

```cpp
// Satellite in ECEF
Vec3<double> r_ecef = lla_to_ecef(LLA<double>(lon, lat, 420000));

// Get ground track (LLA with alt=0)
LLA<double> ground = ground_track_point(r_ecef);

// Or get ECEF coordinates directly
Vec3<double> ground_ecef = ground_track_ecef(r_ecef);
```

## Symbolic Optimization

All functions are Janus-compatible for trajectory optimization:

```cpp
using MX = janus::SymbolicScalar;

// Symbolic positions
MX ox = sym("ox"), oy = sym("oy"), oz = sym("oz");
MX tx = sym("tx"), ty = sym("ty"), tz = sym("tz");

Vec3<MX> observer, target;
observer << ox, oy, oz;
target << tx, ty, tz;

// Symbolic slant range
MX range = slant_range(observer, target);

// Build differentiable function
janus::Function f("range", {ox, oy, oz, tx, ty, tz}, {range});

// Evaluate with gradients
auto result = f({0, 0, 0, 3, 4, 0});      // = 5.0
auto jacobian = f.jacobian({0, 0, 0, 3, 4, 0});  // [0, 0, 0, 0.6, 0.8, 0]
```

### Example: FOV Optimization

```cpp
MX sensor_az = sym("az");  // Sensor azimuth to optimize

// Compute if target is in FOV
Vec3<MX> axis;
axis << janus::cos(sensor_az), janus::sin(sensor_az), MX(0);

MX visible = point_in_cone(target_position, sensor_pos, axis, fov_half);

// Use in optimization cost function
```

## Coordinate Frame Notes

- **LOS Angles**: Azimuth is measured from +X axis, positive clockwise. Elevation is measured from XY plane, positive upward.
- **NED Convention**: Negative Z is "up", so targets above the observer have negative Z components.
- **Ground Track**: Uses the same ellipsoid model as `ecef_to_lla` (WGS84 by default).

## See Also

- [Geodetic Utilities](geodetic.md) - Great circle distance, bearings
- [Coordinates](coordinates.md) - Frame transformations
- [Orbital](orbital.md) - Orbital mechanics
