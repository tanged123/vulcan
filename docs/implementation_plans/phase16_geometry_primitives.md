# Phase 16: Geometry Primitives Implementation Plan

## Overview

Phase 16 adds spatial computation utilities for guidance, visibility, and sensor FOV calculations. This is a small, focused module of stateless geometric functions.

**Scope**: Line-of-sight, ray intersections, and projections.

---

## Proposed Changes

### Geometry Module (New)

#### [NEW] [Geometry.hpp](file:///home/tanged/sources/vulcan/include/vulcan/geometry/Geometry.hpp)

All functions live in `vulcan::geometry`, templated on `Scalar` for Janus compatibility.

```cpp
namespace vulcan::geometry {

// =============================================================================
// Line-of-Sight
// =============================================================================

/// Compute azimuth and elevation angles from observer to target
/// @param r_observer Observer position [m] (any frame)
/// @param r_target   Target position [m] (same frame)
/// @return [azimuth, elevation] in radians
template <typename Scalar>
Vec2<Scalar> los_angles(const Vec3<Scalar>& r_observer, const Vec3<Scalar>& r_target);

/// Line-of-sight angular rate
/// @param r_obs, v_obs  Observer position and velocity
/// @param r_tgt, v_tgt  Target position and velocity
/// @return [azimuth_rate, elevation_rate] in rad/s
template <typename Scalar>
Vec2<Scalar> los_rate(const Vec3<Scalar>& r_obs, const Vec3<Scalar>& v_obs,
                      const Vec3<Scalar>& r_tgt, const Vec3<Scalar>& v_tgt);

/// Slant range (distance) between two points
template <typename Scalar>
Scalar slant_range(const Vec3<Scalar>& r_observer, const Vec3<Scalar>& r_target);

// =============================================================================
// Ray Intersections
// =============================================================================

/// Ray-sphere intersection
/// @param origin    Ray origin
/// @param direction Ray direction (should be normalized)
/// @param center    Sphere center
/// @param radius    Sphere radius
/// @return Distance to intersection (negative if no intersection)
template <typename Scalar>
Scalar ray_sphere_intersection(const Vec3<Scalar>& origin,
                               const Vec3<Scalar>& direction,
                               const Vec3<Scalar>& center,
                               const Scalar& radius);

/// Ray-plane intersection
/// @param origin      Ray origin
/// @param direction   Ray direction (should be normalized)
/// @param plane_normal Plane normal vector
/// @param plane_point  A point on the plane
/// @return Distance to intersection (negative if no intersection or parallel)
template <typename Scalar>
Scalar ray_plane_intersection(const Vec3<Scalar>& origin,
                              const Vec3<Scalar>& direction,
                              const Vec3<Scalar>& plane_normal,
                              const Vec3<Scalar>& plane_point);

/// Check if a point lies within a cone (e.g., sensor FOV)
/// @param point      Point to test
/// @param apex       Cone apex (sensor location)
/// @param axis       Cone axis direction (should be normalized)
/// @param half_angle Cone half-angle in radians
/// @return true if point is inside the cone
template <typename Scalar>
Scalar point_in_cone(const Vec3<Scalar>& point,
                     const Vec3<Scalar>& apex,
                     const Vec3<Scalar>& axis,
                     const Scalar& half_angle);

// =============================================================================
// Projections
// =============================================================================

/// Project a point onto a plane
/// @param point        Point to project
/// @param plane_normal Plane normal (should be normalized)
/// @param plane_point  A point on the plane
/// @return Projected point on the plane
template <typename Scalar>
Vec3<Scalar> project_to_plane(const Vec3<Scalar>& point,
                              const Vec3<Scalar>& plane_normal,
                              const Vec3<Scalar>& plane_point);

/// Project ECEF position to ellipsoid surface (ground track)
/// @param r_ecef Position in ECEF [m]
/// @return LLA coordinates of ground track point [lat, lon, 0]
template <typename Scalar>
Vec3<Scalar> ground_track_point(const Vec3<Scalar>& r_ecef);

} // namespace vulcan::geometry
```

---

### vulcan.hpp

#### [MODIFY] [vulcan.hpp](file:///home/tanged/sources/vulcan/include/vulcan/vulcan.hpp)

Add geometry include:
```cpp
#include <vulcan/geometry/Geometry.hpp>
```

---

### Tests

#### [NEW] [test_geometry.cpp](file:///home/tanged/sources/vulcan/tests/geometry/test_geometry.cpp)

| Test Case | Validation |
|-----------|------------|
| `slant_range` | Simple 3-4-5 triangle distance |
| `los_angles` | Known azimuth/elevation geometry |
| `los_rate` | Moving target case |
| `ray_sphere_intersection` | Hit and miss cases |
| `ray_plane_intersection` | Varying angles + parallel ray |
| `point_in_cone` | Inside, outside, on boundary |
| `project_to_plane` | Verify projection math |
| `ground_track_point` | Round-trip with `ecef_to_lla` |
| Symbolic mode | Graph generation for all functions |

#### [MODIFY] [CMakeLists.txt](file:///home/tanged/sources/vulcan/tests/CMakeLists.txt)

Add `test_geometry` executable under `geometry/` subdirectory.

---

## Implementation Notes

1. **LOS Angles Convention**: Use NED frame convention (azimuth from North, elevation positive up).
2. **Return Types**: `point_in_cone` returns `Scalar` (0.0/1.0) rather than `bool` for symbolic compatibility via `janus::where`.
3. **No New Dependencies**: Uses existing `Vec3`, `janus::` math functions.
4. **Normalization**: Direction vectors are assumed normalized; document this precondition.

---

## Verification Plan

### Automated Tests
```bash
./scripts/ci.sh  # Build + test all
```

### Manual Verification
- Verify symbolic graph generation for each function
- Cross-check `ground_track_point` against `ecef_to_lla` reference values
