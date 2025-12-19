# Phase 3: Coordinate Frames — Unified Implementation Plan

A robust 6-DOF coordinate transformation engine for Vulcan with milliarcsec-level accuracy. All code templated on `Scalar` for Janus symbolic/numeric compatibility. Phased approach enables incremental delivery.

---

## User Review Required

> [!IMPORTANT]
> **Hub Frame: ECEF (Earth-Centered Earth-Fixed)**
> 
> All transformations route through ECEF as the hub. This is O(2N) conversions instead of O(N²) direct matrix pairs. ECEF is natural for atmospheric/launch simulations; EOMs require Coriolis/centrifugal terms but are mathematically equivalent to ECI.

> [!IMPORTANT]
> **Geodetic Conversion: Vermeille's Closed-Form Algorithm**
> 
> Using Vermeille (2004) instead of TAOS iterative method. Benefits:
> - Truly non-iterative (no 25-iteration loop)
> - Works at poles, origin, and space
> - Symbolic-friendly with no loops to unroll

> [!IMPORTANT]
> **Frame Representation: Basis Vectors in ECEF**
> 
> Frames stored as three orthonormal unit vectors (expressed in ECEF) rather than full DCMs. Transformations use dot products for projections. Memory-efficient and matches the TAOS specification.

> [!WARNING]
> **Pole and Origin Singularities**
> 
> Longitude is undefined at poles (xy_dist ≈ 0) and at Earth's center (||r|| ≈ 0). These are handled via `janus::where` for NaN-safe symbolic graphs.

> [!NOTE]
> **Phased SOFA Integration**
> 
> Phase 3a uses `ConstantOmegaRotation` (simple ωt). Phase 3b adds time infrastructure (Epoch, LeapSeconds). Phase 3c wraps SOFA for IAU 2006/2000A precession-nutation (~mas accuracy). API remains stable throughout.

---

## Phase Overview

| Phase | Scope | Effort | Dependencies |
|-------|-------|--------|--------------|
| **3a** | Core Coordinates | ~3 days | None |
| **3b** | Time Infrastructure | ~5 days | None (pure C++) |
| **3c** | SOFA Integration | ~2 weeks | SOFA C library (via Nix) |

---

## Phase 3a: Core Coordinates

### Component 1: Earth Model

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

/// Pluggable Earth rotation model (upgraded in Phase 3c)
struct EarthRotationModel {
    virtual ~EarthRotationModel() = default;
    virtual double gmst(double t_seconds) const = 0;
};

struct ConstantOmegaRotation : EarthRotationModel {
    double omega;      // [rad/s]
    double theta0;     // Initial angle at t=0 [rad]
    double gmst(double t) const override { return theta0 + omega * t; }
};

} // namespace vulcan
```

**Key decisions:**
- `constexpr` factory functions for standard models
- Derived quantities computed once at construction
- Not templated (constants are always `double`)
- `EarthRotationModel` abstraction for SOFA upgrade path

---

### Component 2: Coordinate Frame Class

#### [NEW] [CoordinateFrame.hpp](file:///home/tanged/sources/vulcan/include/vulcan/coordinates/CoordinateFrame.hpp)

```cpp
namespace vulcan {

/// Coordinate frame defined by orthonormal basis vectors expressed in ECEF
template<typename Scalar>
struct CoordinateFrame {
    Vec3<Scalar> x_axis;  // Unit X basis vector in ECEF
    Vec3<Scalar> y_axis;  // Unit Y basis vector in ECEF
    Vec3<Scalar> z_axis;  // Unit Z basis vector in ECEF
    Vec3<Scalar> origin;  // Frame origin in ECEF
    
    /// Project vector FROM ECEF TO this frame
    Vec3<Scalar> from_ecef(const Vec3<Scalar>& v) const {
        return Vec3<Scalar>(
            janus::dot(v, x_axis),
            janus::dot(v, y_axis),
            janus::dot(v, z_axis)
        );
    }
    
    /// Project vector FROM this frame TO ECEF
    Vec3<Scalar> to_ecef(const Vec3<Scalar>& v) const {
        return x_axis * v(0) + y_axis * v(1) + z_axis * v(2);
    }
    
    /// Compose DCM as column vectors [x_axis | y_axis | z_axis]
    Mat3<Scalar> dcm() const;
    
    /// Validate orthonormality
    bool is_valid(double tol = 1e-9) const;
    
    // --- Static Factories ---
    static CoordinateFrame ecef();  // Identity frame
    static CoordinateFrame eci(Scalar gmst);
    static CoordinateFrame ned(Scalar lon, Scalar lat);
    static CoordinateFrame enu(Scalar lon, Scalar lat);
};

} // namespace vulcan
```

---

### Component 3: Geodetic Utilities (Vermeille Algorithm)

#### [NEW] [Geodetic.hpp](file:///home/tanged/sources/vulcan/include/vulcan/coordinates/Geodetic.hpp)

```cpp
namespace vulcan {

/// Geodetic coordinates (longitude, latitude, altitude)
template<typename Scalar>
struct LLA {
    Scalar lon;  // Longitude [rad]
    Scalar lat;  // Geodetic latitude [rad]
    Scalar alt;  // Altitude above ellipsoid [m]
};

/// ECEF → LLA using Vermeille (2004) closed-form algorithm
template<typename Scalar>
LLA<Scalar> ecef_to_lla(const Vec3<Scalar>& r, 
                         const EarthModel& m = EarthModel::WGS84());

/// LLA → ECEF (closed-form)
template<typename Scalar>
Vec3<Scalar> lla_to_ecef(const LLA<Scalar>& lla, 
                          const EarthModel& m = EarthModel::WGS84());

/// Geocentric (spherical) coordinates
template<typename Scalar>
struct Spherical {
    Scalar lon;      // Longitude [rad]
    Scalar lat_gc;   // Geocentric latitude [rad]
    Scalar radius;   // Distance from Earth center [m]
};

template<typename Scalar>
Spherical<Scalar> ecef_to_spherical(const Vec3<Scalar>& r);

template<typename Scalar>
Vec3<Scalar> spherical_to_ecef(const Spherical<Scalar>& geo);

} // namespace vulcan
```

**Vermeille Algorithm (2004) — Key Equations:**
```cpp
// Closed-form ECEF to geodetic — no iteration
Scalar p = (x*x + y*y) / (a*a);
Scalar q = (1 - e2) * z*z / (a*a);
Scalar r = (p + q - e4) / 6;
Scalar s = e4 * p * q / (4 * r*r*r);
Scalar t = janus::pow(1 + s + janus::sqrt(s * (2 + s)), 1.0/3.0);
Scalar u = r * (1 + t + 1/t);
Scalar v = janus::sqrt(u*u + e4*q);
Scalar w = e2 * (u + v - q) / (2*v);
Scalar k = janus::sqrt(u + v + w*w) - w;
Scalar D = k * janus::sqrt(x*x + y*y) / (k + e2);
lat = 2 * janus::atan2(z, D + janus::sqrt(D*D + z*z));
alt = (k + e2 - 1) / k * janus::sqrt(D*D + z*z);
lon = janus::atan2(y, x);
```

**Pole handling:**
```cpp
Scalar is_pole = janus::sqrt(x*x + y*y) < eps;
lon = janus::where(is_pole, Scalar(0.0), janus::atan2(y, x));
```

---

### Component 4: Local Frames & Body Frames

#### [NEW] [LocalFrames.hpp](file:///home/tanged/sources/vulcan/include/vulcan/coordinates/LocalFrames.hpp)

```cpp
namespace vulcan {

/// Local Geodetic Horizon (NED) — North, East, Down
template<typename Scalar>
CoordinateFrame<Scalar> local_ned(Scalar lon, Scalar lat_gd);

/// Local Horizon (ENU) — East, North, Up
template<typename Scalar>
CoordinateFrame<Scalar> local_enu(Scalar lon, Scalar lat_gd);

/// Local Geocentric Horizon — uses geocentric latitude
template<typename Scalar>
CoordinateFrame<Scalar> local_geocentric(Scalar lon, Scalar lat_gc);

} // namespace vulcan
```

#### [NEW] [BodyFrames.hpp](file:///home/tanged/sources/vulcan/include/vulcan/coordinates/BodyFrames.hpp)

```cpp
namespace vulcan {

/// Body-fixed frame from Euler angles relative to NED
/// Sequence: Yaw (Z) → Pitch (Y') → Roll (X'')
template<typename Scalar>
CoordinateFrame<Scalar> body_from_euler(
    const CoordinateFrame<Scalar>& ned,
    Scalar yaw, Scalar pitch, Scalar roll);

/// Extract Euler angles from body frame (with gimbal lock handling)
template<typename Scalar>
Vec3<Scalar> euler_from_body(
    const CoordinateFrame<Scalar>& body,
    const CoordinateFrame<Scalar>& ned);

/// Velocity frame — X aligned with Earth-relative velocity
template<typename Scalar>
CoordinateFrame<Scalar> velocity_frame(
    const Vec3<Scalar>& velocity_ecef,
    const CoordinateFrame<Scalar>& ned);

/// Flight path angles: gamma (vertical FPA), psi (heading)
template<typename Scalar>
Vec2<Scalar> flight_path_angles(
    const Vec3<Scalar>& velocity,
    const CoordinateFrame<Scalar>& ned);

/// Aerodynamic angles: alpha (angle of attack), beta (sideslip)
template<typename Scalar>
Vec2<Scalar> aero_angles(
    const Vec3<Scalar>& velocity_body,
    const CoordinateFrame<Scalar>& body);

} // namespace vulcan
```

---

### Component 5: Transformation Engine

#### [NEW] [Transforms.hpp](file:///home/tanged/sources/vulcan/include/vulcan/coordinates/Transforms.hpp)

```cpp
namespace vulcan {

/// Transform vector between any two frames (via ECEF hub)
template<typename Scalar>
Vec3<Scalar> transform_vector(
    const Vec3<Scalar>& v,
    const CoordinateFrame<Scalar>& from,
    const CoordinateFrame<Scalar>& to);

/// Transform position (accounts for origin offset)
template<typename Scalar>
Vec3<Scalar> transform_position(
    const Vec3<Scalar>& pos,
    const CoordinateFrame<Scalar>& from,
    const CoordinateFrame<Scalar>& to);

/// ECEF→ECI velocity transform (adds ω × r term)
template<typename Scalar>
Vec3<Scalar> velocity_ecef_to_eci(
    const Vec3<Scalar>& v_ecef,
    const Vec3<Scalar>& r_ecef,
    const EarthModel& m = EarthModel::WGS84());

/// Non-inertial accelerations for ECEF EOMs
/// a_ecef = a_inertial - 2(ω × v) - ω × (ω × r)
template<typename Scalar>
Vec3<Scalar> coriolis_centrifugal(
    const Vec3<Scalar>& r_ecef,
    const Vec3<Scalar>& v_ecef,
    const EarthModel& m = EarthModel::WGS84());

} // namespace vulcan
```

---

### Component 6: Main Header Update

#### [MODIFY] [vulcan.hpp](file:///home/tanged/sources/vulcan/include/vulcan/vulcan.hpp)

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

## Phase 3b: Time Infrastructure

### Component 7: Julian Date Utilities

#### [NEW] [JulianDate.hpp](file:///home/tanged/sources/vulcan/include/vulcan/time/JulianDate.hpp)

```cpp
namespace vulcan {

double calendar_to_jd(int y, int m, int d, int h, int min, double sec);
void jd_to_calendar(double jd, int& y, int& m, int& d, int& h, int& min, double& sec);
double jd_to_mjd(double jd);
double mjd_to_jd(double mjd);

} // namespace vulcan
```

---

### Component 8: Leap Seconds

#### [NEW] [LeapSeconds.hpp](file:///home/tanged/sources/vulcan/include/vulcan/time/LeapSeconds.hpp)

```cpp
namespace vulcan {

/// Leap second table (IERS Bulletin C)
int leap_seconds_at(double tai_jd);  // TAI - UTC
double utc_to_tai(double utc_jd);
double tai_to_utc(double tai_jd);

} // namespace vulcan
```

---

### Component 9: Epoch Class

#### [NEW] [Epoch.hpp](file:///home/tanged/sources/vulcan/include/vulcan/time/Epoch.hpp)

```cpp
namespace vulcan {

enum class TimeScale { UTC, TAI, TT, TDB, GPS };

class Epoch {
    double tai_seconds_;  // TAI seconds since J2000.0
public:
    static Epoch from_utc(int y, int m, int d, int h, int min, double sec);
    static Epoch from_jd(double jd, TimeScale scale = TimeScale::TT);
    static Epoch j2000();
    
    double to_jd(TimeScale scale) const;
    double to_mjd(TimeScale scale) const;
    double seconds_since_j2000(TimeScale scale) const;
    
    Epoch operator+(double seconds) const;
    double operator-(const Epoch& other) const;
};

} // namespace vulcan
```

---

## Phase 3c: SOFA Integration

### Component 10: SOFA Wrapper

#### [NEW] [SOFA.hpp](file:///home/tanged/sources/vulcan/include/vulcan/orientation/SOFA.hpp)

```cpp
namespace vulcan {

/// Earth Rotation Angle (ERA) — modern replacement for GMST
double era(double ut1_jd);

/// Precession-Nutation (IAU 2006/2000A)
Mat3<double> precession_nutation_matrix(double tt_jd);

/// Full ITRF ↔ GCRF transformation (mas accuracy)
Mat3<double> itrf_to_gcrf(const Epoch& t);
Mat3<double> gcrf_to_itrf(const Epoch& t);

/// SOFA-backed rotation model
struct SOFARotation : EarthRotationModel {
    Mat3<double> ecef_to_eci(const Epoch& t) const;
};

} // namespace vulcan
```

### Required SOFA Routines

| Routine | Purpose |
|---------|---------|
| `iauEra00` | Earth Rotation Angle |
| `iauPnm06a` | Precession-nutation matrix |
| `iauC2t06a` | Celestial-to-terrestrial |
| `iauDat` | Delta AT (leap seconds) |

---

## Reference Frames Summary

| Frame | Description | Accuracy |
|-------|-------------|----------|
| **GCRF** | Geocentric Celestial (≈ICRF) | mas |
| **ITRF** | International Terrestrial (≈ECEF) | mas |
| **TEME** | TLE propagation frame | arcsec |
| **TOD/MOD** | True/Mean of Date | arcsec |

---

## Verification Plan

### Phase 3a Tests

```bash
./scripts/ci.sh && cd build && ctest -R coordinates
```

| Test | Description | Reference |
|------|-------------|-----------|
| `WGS84_Constants` | Verify a, f, e² values | TAOS Table 2-1 |
| `LLA_Equator` | LLA(0,0,0) ↔ ECEF roundtrip | |
| `LLA_NorthPole` | Pole singularity handling | |
| `LLA_SouthPole` | Pole singularity handling | |
| `LLA_Dateline` | λ=±180° consistency | |
| `LLA_HighAlt` | Geostationary orbit altitude | |
| `Vermeille_Accuracy` | Compare against reference values | |
| `ECI_ZeroTime` | ECI = ECEF at t=0 | |
| `ECI_QuarterDay` | 90° rotation after ~6 hours | |
| `NED_Consistency` | NED frame validation | |
| `Body_Identity` | Zero Euler angles = NED | |
| `Body_GimbalLock` | Pitch = ±90° handling | |
| `Symbolic_LLA` | Symbolic graph builds | |
| `Symbolic_Transform` | Symbolic transform chain | |
| `Coriolis_Centrifugal` | Non-inertial terms validation | |

### Phase 3b Tests

| Test | Description |
|------|-------------|
| `JD_Roundtrip` | Calendar ↔ JD ↔ MJD |
| `LeapSeconds_Table` | Validate against IERS |
| `Epoch_TimeScales` | UTC/TAI/TT conversions |

### Phase 3c Tests

| Test | Description |
|------|-------------|
| `SOFA_TestVectors` | Compare against SOFA library tests |
| `ITRF_GCRF` | ITRF↔GCRF at known epochs |
| `JPL_Horizons` | Validate against JPL ephemerides |

### Reference Validation Points

```
Equatorial:    (6378137, 0, 0) m      → (0°, 0°, 0m)
North Pole:    (0, 0, 6356752) m      → (0°, 90°, 0m)
Geostationary: r ≈ 42164 km           → lat ≈ 0°
```

### Example Program

```bash
cd build && ./examples/coordinate_demo
```

Expected output:
```
=== Vulcan Coordinate Frames Demo ===

Test Position: Washington DC (38.9°N, 77.0°W, 100m)
  LLA:        lon=-77.0°, lat=38.9°, alt=100m
  ECEF:       x=1107.4km, y=-4843.3km, z=3988.8km
  Spherical:  lon=-77.0°, lat_gc=38.7°, r=6370.9km

Local NED Frame at Washington DC:
  X (North): [...]
  Y (East):  [...]
  Z (Down):  [...]

Round-trip error: < 1e-10 m
```

---

## File Summary

### Phase 3a Files

| Type | Path |
|------|------|
| [NEW] | `include/vulcan/coordinates/EarthModel.hpp` |
| [NEW] | `include/vulcan/coordinates/CoordinateFrame.hpp` |
| [NEW] | `include/vulcan/coordinates/Geodetic.hpp` |
| [NEW] | `include/vulcan/coordinates/LocalFrames.hpp` |
| [NEW] | `include/vulcan/coordinates/BodyFrames.hpp` |
| [NEW] | `include/vulcan/coordinates/Transforms.hpp` |
| [MODIFY] | `include/vulcan/vulcan.hpp` |
| [NEW] | `tests/coordinates/test_geodetic.cpp` |
| [NEW] | `tests/coordinates/test_frames.cpp` |
| [NEW] | `tests/coordinates/test_transforms.cpp` |
| [NEW] | `examples/coordinate_demo.cpp` |

### Phase 3b Files

| Type | Path |
|------|------|
| [NEW] | `include/vulcan/time/JulianDate.hpp` |
| [NEW] | `include/vulcan/time/LeapSeconds.hpp` |
| [NEW] | `include/vulcan/time/Epoch.hpp` |
| [NEW] | `tests/time/test_julian.cpp` |
| [NEW] | `tests/time/test_epoch.cpp` |

### Phase 3c Files

| Type | Path |
|------|------|
| [NEW] | `include/vulcan/orientation/SOFA.hpp` |
| [MODIFY] | `flake.nix` (add SOFA dependency) |
| [NEW] | `tests/orientation/test_sofa.cpp` |
