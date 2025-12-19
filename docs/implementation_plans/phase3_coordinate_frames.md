# Phase 3: Coordinate Frames Implementation Plan

Implement a robust, 6-DOF coordinate transformation engine for Vulcan based on the TAOS specification. All code will be templated on `Scalar` for Janus symbolic/numeric dual-backend compatibility.

## User Review Required

> [!IMPORTANT]
> **Hub-and-Spoke vs Direct Transformation**
> 
> TAOS recommends the "Unit Vector Projection Method" where all transformations route through ECFC as the hub. This is O(2N) conversions instead of O(N²) direct matrix pairs. This is the proposed approach.

> [!IMPORTANT]
> **Frame Representation: Basis Vectors vs DCM**
> 
> Store frames as three unit basis vectors (expressed in ECFC) rather than full DCMs. Transformations use dot products for projections. This is more memory-efficient and matches TAOS exactly.

> [!WARNING]
> **Geodetic Latitude Iteration**
> 
> ECFC → Geodetic conversion requires an iterative solver (TAOS Eqs 2-34 to 2-41). For symbolic mode, fixed iteration count (max 25) with early exit for numeric. Symbolic graphs will contain all iterations.

> [!CAUTION]
> **Pole and Origin Singularities**
> 
> Longitude is undefined at poles (xy_dist ≈ 0) and at Earth's center (||r|| ≈ 0). These must be handled explicitly with `janus::where` to avoid NaN propagation.

---

## Proposed Changes

### Component 1: Earth Model

Configurable ellipsoid parameters with WGS84 as default.

#### [NEW] [EarthModel.hpp](file:///home/tanged/sources/vulcan/include/vulcan/coordinates/EarthModel.hpp)

```cpp
namespace vulcan {

/// Ellipsoid parameters for Earth models
struct EarthModel {
    double a;        // Semi-major axis (equatorial radius) [m]
    double f;        // Flattening
    double omega;    // Angular velocity [rad/s]
    double mu;       // Gravitational parameter [m³/s²]
    
    // Derived quantities (computed at construction)
    double b;        // Semi-minor axis (polar radius)
    double e2;       // First eccentricity squared
    double e_prime2; // Second eccentricity squared
    
    static constexpr EarthModel WGS84();
    static constexpr EarthModel WGS72();
    static constexpr EarthModel Spherical();
};

} // namespace vulcan
```

**Key design decisions:**
- Use `constexpr` factory functions for standard models
- Derived quantities computed once at construction
- Not templated (constants are always `double`)

---

### Component 2: Coordinate Frame Class

Generic frame representation storing basis vectors in ECFC.

#### [NEW] [CoordinateFrame.hpp](file:///home/tanged/sources/vulcan/include/vulcan/coordinates/CoordinateFrame.hpp)

```cpp
namespace vulcan {

/// Coordinate frame defined by orthonormal basis vectors expressed in ECFC
template<typename Scalar>
struct CoordinateFrame {
    Vec3<Scalar> x_axis;  // Unit X basis vector in ECFC
    Vec3<Scalar> y_axis;  // Unit Y basis vector in ECFC
    Vec3<Scalar> z_axis;  // Unit Z basis vector in ECFC
    Vec3<Scalar> origin;  // Frame origin in ECFC (Earth center or vehicle CM)
    
    /// Project vector FROM ECFC TO this frame
    Vec3<Scalar> from_ecfc(const Vec3<Scalar>& vec_ecfc) const;
    
    /// Project vector FROM this frame TO ECFC
    Vec3<Scalar> to_ecfc(const Vec3<Scalar>& vec_local) const;
    
    /// Compose rotation matrix as column vectors [x_axis, y_axis, z_axis]
    Mat3<Scalar> dcm() const;
    
    /// Validate orthonormality
    Scalar is_valid(double tol = 1e-9) const;
    
    // --- Static Factories ---
    static CoordinateFrame ecfc();  // Identity frame
    static CoordinateFrame ecic(Scalar time, double omega = constants::wgs84::omega);
    static CoordinateFrame geocentric(Scalar lon, Scalar lat_gc);
    static CoordinateFrame geodetic(Scalar lon, Scalar lat_gd);
    static CoordinateFrame from_euler(const CoordinateFrame& ref, Scalar yaw, Scalar pitch, Scalar roll);
};

} // namespace vulcan
```

**Projection implementation (from TAOS):**
```cpp
template<typename Scalar>
Vec3<Scalar> CoordinateFrame<Scalar>::from_ecfc(const Vec3<Scalar>& v) const {
    return Vec3<Scalar>(
        janus::dot(v, x_axis),
        janus::dot(v, y_axis),
        janus::dot(v, z_axis)
    );
}

template<typename Scalar>
Vec3<Scalar> CoordinateFrame<Scalar>::to_ecfc(const Vec3<Scalar>& v) const {
    return x_axis * v(0) + y_axis * v(1) + z_axis * v(2);
}
```

---

### Component 3: Geodetic Utilities

Robust ECFC ↔ Geodetic conversions with pole/center handling.

#### [NEW] [Geodetic.hpp](file:///home/tanged/sources/vulcan/include/vulcan/coordinates/Geodetic.hpp)

```cpp
namespace vulcan {

/// Geodetic coordinates (longitude, latitude, altitude)
template<typename Scalar>
struct GeodeticCoord {
    Scalar lon;   // Longitude [rad]
    Scalar lat;   // Geodetic latitude [rad]
    Scalar alt;   // Altitude above ellipsoid [m]
};

/// Convert ECFC position to geodetic coordinates
/// Uses TAOS iterative algorithm (max 25 iterations, tol 1e-8)
template<typename Scalar>
GeodeticCoord<Scalar> ecfc_to_geodetic(const Vec3<Scalar>& r_ecfc, 
                                        const EarthModel& model = EarthModel::WGS84());

/// Convert geodetic coordinates to ECFC position
template<typename Scalar>
Vec3<Scalar> geodetic_to_ecfc(const GeodeticCoord<Scalar>& geo,
                               const EarthModel& model = EarthModel::WGS84());

/// Geocentric coordinates (spherical)
template<typename Scalar>
struct GeocentricCoord {
    Scalar lon;      // Longitude [rad]
    Scalar lat_gc;   // Geocentric latitude [rad]
    Scalar radius;   // Distance from Earth center [m]
};

template<typename Scalar>
GeocentricCoord<Scalar> ecfc_to_geocentric(const Vec3<Scalar>& r_ecfc);

template<typename Scalar>
Vec3<Scalar> geocentric_to_ecfc(const GeocentricCoord<Scalar>& geo);

} // namespace vulcan
```

**TAOS Iterative Algorithm for Geodetic Latitude:**
```cpp
// Initial estimate assuming spherical Earth
Scalar z_intercept = model.a * model.e2 * (r_ecfc(2) / janus::norm(r_ecfc));

// Iteration loop (fixed count for symbolic compatibility)
for (int i = 0; i < 25; ++i) {
    Scalar z_d = r_ecfc(2) + z_intercept;
    Scalar N_plus_h = janus::sqrt(xy_dist_sq + z_d * z_d);
    Scalar sin_lat = z_d / N_plus_h;
    Scalar N = model.a / janus::sqrt(one - model.e2 * sin_lat * sin_lat);
    Scalar z_new = N * model.e2 * sin_lat;
    
    // For numeric: early exit if converged
    // For symbolic: all iterations execute (graph contains full loop)
    z_intercept = z_new;
}
```

**Pole handling:**
```cpp
Scalar is_pole = xy_dist < eps;
lon = janus::where(is_pole, Scalar(0.0), janus::atan2(r_y, r_x));
```

---

### Component 4: Local Horizon Frames

NED/ENU reference frames based on geodetic or geocentric latitude.

#### [NEW] [LocalFrames.hpp](file:///home/tanged/sources/vulcan/include/vulcan/coordinates/LocalFrames.hpp)

```cpp
namespace vulcan {

/// Local Geodetic Horizon (LGH) frame - NED convention
/// X: North (tangent to meridian), Y: East, Z: Down (ellipsoid normal)
template<typename Scalar>
CoordinateFrame<Scalar> local_geodetic_horizon(Scalar lon, Scalar lat_gd);

/// Local Geocentric Horizon frame - same as LGH but using geocentric latitude
/// X: North, Y: East, Z: Down (opposite to radius vector)
template<typename Scalar>
CoordinateFrame<Scalar> local_geocentric_horizon(Scalar lon, Scalar lat_gc);

/// ENU frame (alternate convention)
/// X: East, Y: North, Z: Up
template<typename Scalar>
CoordinateFrame<Scalar> local_enu(Scalar lon, Scalar lat_gd);

} // namespace vulcan
```

**TAOS basis vector construction (geodetic):**
```cpp
// Using geodetic latitude δ_gd and longitude λ
Scalar sin_lat = janus::sin(lat_gd);
Scalar cos_lat = janus::cos(lat_gd);
Scalar sin_lon = janus::sin(lon);
Scalar cos_lon = janus::cos(lon);

Vec3<Scalar> x_north;  // Points North
x_north << -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat;

Vec3<Scalar> y_east;   // Points East
y_east << -sin_lon, cos_lon, Scalar(0.0);

Vec3<Scalar> z_down;   // Points Down (normal to ellipsoid)
z_down << -cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat;
```

---

### Component 5: Body & Aerodynamic Frames

Vehicle-centric frames for flight dynamics.

#### [NEW] [BodyFrames.hpp](file:///home/tanged/sources/vulcan/include/vulcan/coordinates/BodyFrames.hpp)

```cpp
namespace vulcan {

/// Create body-fixed frame from Euler angles relative to LGH
/// Sequence: Yaw (Z) → Pitch (Y') → Roll (X'')
template<typename Scalar>
CoordinateFrame<Scalar> body_frame_from_euler(
    const CoordinateFrame<Scalar>& lgh,
    Scalar yaw, Scalar pitch, Scalar roll);

/// Extract Euler angles from body frame (with gimbal lock handling)
template<typename Scalar>
Vec3<Scalar> euler_from_body_frame(
    const CoordinateFrame<Scalar>& body,
    const CoordinateFrame<Scalar>& lgh);

/// Velocity frame - X aligned with Earth-relative velocity
template<typename Scalar>
CoordinateFrame<Scalar> velocity_frame(
    const Vec3<Scalar>& velocity_ecfc,
    const CoordinateFrame<Scalar>& lgh);

/// Flight path angles (gamma = vertical FPA, psi = heading)
template<typename Scalar>
Vec2<Scalar> flight_path_angles(
    const Vec3<Scalar>& velocity,
    const CoordinateFrame<Scalar>& lgh);

/// Aerodynamic angles (alpha, beta)
template<typename Scalar>
Vec2<Scalar> aero_angles(
    const Vec3<Scalar>& velocity_body,
    const CoordinateFrame<Scalar>& body);

/// Total angle of attack (preferred for high-angle dynamics)
template<typename Scalar>
Vec2<Scalar> total_aoa(
    const Vec3<Scalar>& velocity_wind,
    const CoordinateFrame<Scalar>& body);

} // namespace vulcan
```

**Gimbal lock handling for Euler extraction:**
```cpp
// Check if pitch ≈ ±90° (gimbal lock)
Scalar is_gimbal_lock = janus::abs(x_body_dot_z_lgh - one) < eps 
                      | janus::abs(x_body_dot_z_lgh + one) < eps;

// At gimbal lock: set yaw = 0, roll = 0 (arbitrary but consistent)
yaw = janus::where(is_gimbal_lock, Scalar(0.0), normal_yaw);
```

---

### Component 6: Transformation Engine

Central hub for coordinate transformations.

#### [NEW] [Transforms.hpp](file:///home/tanged/sources/vulcan/include/vulcan/coordinates/Transforms.hpp)

```cpp
namespace vulcan {

/// Transform vector between any two frames (via ECFC hub)
template<typename Scalar>
Vec3<Scalar> transform_vector(
    const Vec3<Scalar>& vec,
    const CoordinateFrame<Scalar>& from,
    const CoordinateFrame<Scalar>& to);

/// Transform position (accounts for origin offset)
template<typename Scalar>
Vec3<Scalar> transform_position(
    const Vec3<Scalar>& pos,
    const CoordinateFrame<Scalar>& from,
    const CoordinateFrame<Scalar>& to);

/// Transform velocity with Coriolis correction (rotating → inertial)
template<typename Scalar>
Vec3<Scalar> transform_velocity_rotating(
    const Vec3<Scalar>& vel_rotating,
    const Vec3<Scalar>& pos_ecfc,
    const EarthModel& model = EarthModel::WGS84());

/// ECFC → ECIC transformation at given time
template<typename Scalar>
Vec3<Scalar> ecfc_to_ecic(
    const Vec3<Scalar>& vec_ecfc, 
    Scalar time,
    Scalar t0 = 0.0,
    Scalar omega0 = 0.0,
    const EarthModel& model = EarthModel::WGS84());

} // namespace vulcan
```

---

### Component 7: Main Header Update

#### [MODIFY] [vulcan.hpp](file:///home/tanged/sources/vulcan/include/vulcan/vulcan.hpp)

Uncomment and add coordinate system includes:
```cpp
// Coordinate systems
#include <vulcan/coordinates/EarthModel.hpp>
#include <vulcan/coordinates/CoordinateFrame.hpp>
#include <vulcan/coordinates/Geodetic.hpp>
#include <vulcan/coordinates/LocalFrames.hpp>
#include <vulcan/coordinates/BodyFrames.hpp>
#include <vulcan/coordinates/Transforms.hpp>
```

---

### Component 8: Header Directory

#### [NEW] include/vulcan/coordinates/

Create the directory containing all coordinate headers.

---

## Verification Plan

### Automated Tests

All tests run via the Vulcan CI script:

```bash
# Full CI (dev shell + build + test)
./scripts/ci.sh

# Run only coordinate tests
cd build && ctest -R coordinates --output-on-failure
```

#### [MODIFY] [test_frames.cpp](file:///home/tanged/sources/vulcan/tests/coordinates/test_frames.cpp)

Expand existing scaffold with comprehensive tests:

| Test Case | Description | Reference |
|-----------|-------------|-----------|
| `EarthModel_WGS84Constants` | Verify WGS84 a, f, e² values | TAOS Table 2-1 |
| `Geodetic_SeaLevelEquator` | LLA(0,0,0) → ECFC → LLA roundtrip | |
| `Geodetic_NorthPole` | Pole singularity handling | TAOS Eq 2-34 |
| `Geodetic_SouthPole` | Pole singularity handling | |
| `Geodetic_Dateline` | λ=±180° consistency | |
| `Geodetic_HighAltitude` | Geostationary orbit altitude | |
| `Geodetic_Iteration` | Convergence within 25 iterations | TAOS Eq 2-41 |
| `ECIC_ZeroTime` | ECIC = ECFC at t=0 | TAOS Eq 2-16 |
| `ECIC_QuarterDay` | 90° rotation after ~6 hours | |
| `LocalHorizon_Equator` | NED at equator/prime meridian | TAOS Eqs 2-26 |
| `LocalHorizon_Pole` | Degenerate case handling | |
| `BodyFrame_Identity` | Zero Euler angles = LGH | |
| `BodyFrame_GimbalLock` | Pitch = ±90° handling | TAOS Eq 2-54 |
| `VelocityFrame_Horizontal` | γ=0 case | |
| `VelocityFrame_Vertical` | γ=±90° case | |
| `Symbolic_Geodetic` | Symbolic mode builds graph | |
| `Symbolic_Transform` | Symbolic transform chain | |

**Test template:**
```cpp
TEST(CoordinateFrames, Geodetic_SeaLevelEquator) {
    using namespace vulcan;
    
    // Sea level at equator on prime meridian
    GeodeticCoord<double> geo{0.0, 0.0, 0.0};  // (lon, lat, alt)
    
    // Convert to ECFC
    Vec3<double> ecfc = geodetic_to_ecfc(geo);
    
    // Should be at (R_eq, 0, 0)
    EXPECT_NEAR(ecfc(0), constants::wgs84::a, 1.0);
    EXPECT_NEAR(ecfc(1), 0.0, 1.0);
    EXPECT_NEAR(ecfc(2), 0.0, 1.0);
    
    // Round-trip back
    auto geo_back = ecfc_to_geodetic(ecfc);
    EXPECT_NEAR(geo_back.lon, 0.0, 1e-10);
    EXPECT_NEAR(geo_back.lat, 0.0, 1e-10);
    EXPECT_NEAR(geo_back.alt, 0.0, 1.0);
}
```

### Symbolic Mode Validation

```cpp
TEST(CoordinateFrames, Symbolic_Geodetic) {
    using namespace vulcan;
    using Scalar = janus::SymbolicScalar;
    
    // Create symbolic position
    auto r = janus::sym_vec<3>("r");
    
    // Build symbolic geodetic conversion graph
    auto geo = ecfc_to_geodetic<Scalar>(r);
    
    // Verify graph was built (not evaluated)
    EXPECT_TRUE(janus::is_symbolic(geo.lat));
    
    // Substitute numeric values and evaluate
    janus::Function f({r}, {geo.lon, geo.lat, geo.alt});
    auto result = f.evaluate({{6378137.0, 0.0, 0.0}});
    
    EXPECT_NEAR(result[0](0, 0), 0.0, 1e-10);  // lon
    EXPECT_NEAR(result[1](0, 0), 0.0, 1e-10);  // lat
    EXPECT_NEAR(result[2](0, 0), 0.0, 1.0);    // alt
}
```

### Manual Verification

Create an example program for visual validation:

```bash
cd build && ./examples/coordinate_demo
```

**Expected output format:**
```
=== Vulcan Coordinate Frames Demo ===

Test Position: Washington DC (38.9°N, 77.0°W, 100m)
  Geodetic:  lon=-77.0°, lat=38.9°, alt=100m
  ECFC:      x=1107.4km, y=-4843.3km, z=3988.8km
  Geocentric: lon=-77.0°, lat_gc=38.7°, r=6370.9km

Local NED Frame at Washington DC:
  X (North): [...]
  Y (East):  [...]
  Z (Down):  [...]

Round-trip error: < 1e-8 m
```

### Reference Validation

Compare against TAOS reference values (from documentation):
- Equatorial point: (6378137, 0, 0) → (0°, 0°, 0m)
- Polar point: (0, 0, 6356752) → (0°, 90°, 0m)
- Geostationary: r ≈ 42164km, lat ≈ 0°
