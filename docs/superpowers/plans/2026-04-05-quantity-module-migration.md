# Quantity Module Migration Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Migrate all Vulcan public APIs to use `Quantity<unit, Scalar>` — no mixing of raw `Scalar` and `Quantity` in public interfaces.

**Architecture:** Each module follows the same pattern: (1) change function signatures to accept/return `Quantity`, (2) unwrap with `.value()` at the top of each function body, (3) keep internal Janus math unchanged, (4) rewrap return values. Existing tests are updated to construct Quantities. All existing test assertions remain valid — only the construction/access syntax changes.

**Tech Stack:** C++20, Vulcan `Quantity<unit, Scalar>` (from `vulcan/quantity/`), GoogleTest, Nix/Ninja

**Spec:** `docs/specs/2026-04-05-quantity-type-design.md`

**CRITICAL convention:** Always use Janus APIs (`janus::SymbolicScalar`, `janus::sym()`, `janus::Function`), never raw CasADi types.

---

## Migration Pattern (applies to every task)

Every function migration follows this recipe:

```cpp
// BEFORE
template <typename Scalar>
Scalar temperature(const Scalar& altitude) {
    // ... math using altitude ...
    return result;
}

// AFTER
template <typename Scalar>
Quantity<K, Scalar> temperature(Quantity<m, Scalar> altitude) {
    auto h = altitude.value();    // unwrap to raw Scalar
    // ... same math using h ...
    return Quantity<K, Scalar>{result};  // rewrap
}
```

Every test migration follows this recipe:

```cpp
// BEFORE
EXPECT_NEAR(temperature(10000.0), 223.15, 0.1);

// AFTER
EXPECT_NEAR(temperature(Quantity<m>(10000.0)).value(), 223.15, 0.1);
```

---

## File Structure

**Modified headers (signatures change):**
- `include/vulcan/core/Constants.hpp` — all constants become `Quantity<unit>`
- `include/vulcan/atmosphere/USSA1976.hpp` — functions + `AtmosphericState` struct
- `include/vulcan/coordinates/Geodetic.hpp` — `LLA`, `Spherical`, conversion functions
- `include/vulcan/coordinates/FrameLocal.hpp` — local frame functions
- `include/vulcan/coordinates/FramePrimitives.hpp` — `CoordinateFrame` struct
- `include/vulcan/rotations/EulerSequences.hpp` — Euler angle functions
- `include/vulcan/rotations/DCMUtils.hpp` — DCM utilities
- `include/vulcan/rotations/AxisAngle.hpp` — axis-angle conversions
- `include/vulcan/rotations/RotationKinematics.hpp` — angular velocity functions
- `include/vulcan/rotations/Interpolation.hpp` — slerp, squad
- `include/vulcan/time/Epoch.hpp` — Epoch class
- `include/vulcan/gravity/PointMass.hpp` — gravity acceleration
- `include/vulcan/gravity/J2.hpp` — J2 perturbation
- `include/vulcan/dynamics/PointMass.hpp` — 3-DOF dynamics
- `include/vulcan/dynamics/RigidBody.hpp` + `RigidBodyTypes.hpp` — 6-DOF dynamics
- `include/vulcan/propulsion/Rocket.hpp` — thrust, Isp, delta-v

**Modified tests (construction syntax changes):**
- `tests/core/test_constants.cpp`
- `tests/atmosphere/test_standard.cpp`
- `tests/coordinates/test_geodetic.cpp`, `test_frames.cpp`, `test_transforms.cpp`, etc.
- `tests/rotations/test_euler_sequences.cpp`, `test_dcm_utils.cpp`, `test_axis_angle.cpp`
- `tests/time/test_time.cpp`
- `tests/gravity/test_point_mass.cpp`, `test_j2.cpp`
- `tests/dynamics/test_point_mass.cpp`, `test_rigid_body.cpp`
- `tests/propulsion/test_rocket.cpp`

**Deprecated (not deleted yet — Icarus still depends on them):**
- `include/vulcan/core/Units.hpp` — old conversion functions (`deg_to_rad`, `ft_to_m`, etc.)

---

### Task 1: Constants Migration

**Files:**
- Modify: `include/vulcan/core/Constants.hpp`
- Modify: `tests/core/test_constants.cpp`

All `inline constexpr double` constants become `inline constexpr Quantity<unit>` values.

- [ ] **Step 1: Read the current Constants.hpp and test_constants.cpp**

Read both files to understand the current structure and test patterns.

- [ ] **Step 2: Add Quantity include and migrate earth constants**

Add `#include <vulcan/quantity/Quantity.hpp>` and `#include <vulcan/quantity/Units.hpp>` to Constants.hpp. Add `using namespace vulcan::units;` inside the constants namespace (or use fully qualified `vulcan::units::m`).

Migrate `constants::earth`:
```cpp
namespace earth {
    inline constexpr Quantity<m * m * m / (s * s)> mu{3.986004418e14};
    inline constexpr Quantity<m>     R_eq{6378137.0};
    inline constexpr Quantity<m>     R_pol{6356752.3142};
    inline constexpr Quantity<m>     R_mean{6371008.8};
    inline constexpr Quantity<dimensionless> f{1.0 / 298.257223563};
    inline constexpr Quantity<dimensionless> J2{1.08263e-3};
    inline constexpr Quantity<dimensionless> J3{-2.54e-6};
    inline constexpr Quantity<dimensionless> J4{-1.61e-6};
    inline constexpr Quantity<rad / s> omega{7.2921159e-5};
}
```

- [ ] **Step 3: Migrate remaining constant namespaces**

`constants::wgs84` — same units as earth.

`constants::atmosphere`:
```cpp
namespace atmosphere {
    inline constexpr Quantity<K>   T0{288.15};
    inline constexpr Quantity<Pa>  P0{101325.0};
    inline constexpr Quantity<kg / (m * m * m)> rho0{1.225};
    inline constexpr Quantity<K / m> L{0.0065};
    inline constexpr Quantity<m>   h_tropopause{11000.0};
    inline constexpr Quantity<kg / (m * m * m / (s * s * K))> M{0.0289644};  // or keep as double if unit is complex
    inline constexpr Quantity<J / (kg * K)> R_air{287.05287};  // verify unit algebra
    inline constexpr Quantity<dimensionless> gamma{1.4};
}
```

Note: Some constants have complex compound units (kg/mol, J/(mol·K), W/(m²·K⁴)). If the mp-units type algebra for these is too complex, keep them as raw `double` with a comment explaining why. Don't block the migration on edge-case unit types.

`constants::physics`:
```cpp
namespace physics {
    inline constexpr Quantity<mps>  c{299792458.0};
    inline constexpr Quantity<m / (s * s)> g0{9.80665};
    // G, k_B, sigma — complex units, keep as double if needed
}
```

`constants::sun`, `constants::moon` — mu as `Quantity<m*m*m/(s*s)>`, radii as `Quantity<m>`, etc.

`constants::angle` — `pi` becomes `Quantity<rad>`, `deg2rad`/`rad2deg`/`arcsec2rad` can be kept as dimensionless doubles (they're conversion factors, now superseded by `Quantity::in<>()`).

- [ ] **Step 4: Update test_constants.cpp**

Change all test accesses to use `.value()`:
```cpp
// Before
EXPECT_NEAR(earth::R_eq, 6378137.0, 1.0);
// After
EXPECT_NEAR(earth::R_eq.value(), 6378137.0, 1.0);
```

- [ ] **Step 5: Build and test**

Run: `./scripts/build.sh && ./scripts/test.sh`
Expected: All tests pass. Some downstream modules that reference constants may break — fix compilation errors by adding `.value()` where constants are used in raw-Scalar math contexts.

**Important:** This task may cause cascading compilation failures in other modules that use these constants with raw Scalar math. Fix these by adding `.value()` at the usage sites as a temporary bridge. The proper Quantity migration of each module comes in later tasks.

- [ ] **Step 6: Commit**

```bash
git commit -m "feat(quantity): migrate constants to Quantity types"
```

---

### Task 2: Atmosphere (USSA1976) Migration

**Files:**
- Modify: `include/vulcan/atmosphere/USSA1976.hpp`
- Modify: `tests/atmosphere/test_standard.cpp`

- [ ] **Step 1: Read current USSA1976.hpp and test_standard.cpp**

Understand the current function signatures and test patterns.

- [ ] **Step 2: Migrate AtmosphericState struct**

```cpp
template <typename Scalar> struct AtmosphericState {
    Quantity<K, Scalar>                   temperature;
    Quantity<Pa, Scalar>                  pressure;
    Quantity<kg / (m * m * m), Scalar>    density;
    Quantity<mps, Scalar>                 speed_of_sound;
    Quantity<m / (s * s), Scalar>         gravity;
    Quantity<Pa * s, Scalar>              dynamic_viscosity;
};
```

- [ ] **Step 3: Migrate individual property functions**

Each function takes `Quantity<m, Scalar> altitude` and returns the appropriate typed quantity:

```cpp
template <typename Scalar>
Quantity<K, Scalar> temperature(Quantity<m, Scalar> altitude);

template <typename Scalar>
Quantity<Pa, Scalar> pressure(Quantity<m, Scalar> altitude);

template <typename Scalar>
Quantity<kg / (m * m * m), Scalar> density(Quantity<m, Scalar> altitude);

template <typename Scalar>
Quantity<mps, Scalar> speed_of_sound(Quantity<m, Scalar> altitude);

template <typename Scalar>
Quantity<m / (s * s), Scalar> gravity(Quantity<m, Scalar> altitude);

template <typename Scalar>
Quantity<Pa * s, Scalar> dynamic_viscosity(Quantity<m, Scalar> altitude);

template <typename Scalar>
AtmosphericState<Scalar> state(Quantity<m, Scalar> altitude);
```

Inside each function body: `auto h = altitude.value();` at the top, keep all math, wrap return.

- [ ] **Step 4: Update tests**

```cpp
// Before
auto T = vulcan::ussa1976::temperature(10000.0);
EXPECT_NEAR(T, 223.15, 0.5);

// After
auto T = vulcan::ussa1976::temperature(Quantity<m>(10000.0));
EXPECT_NEAR(T.value(), 223.15, 0.5);
```

- [ ] **Step 5: Build and test**

Run: `./scripts/build.sh && ./scripts/test.sh`
Expected: All atmosphere tests pass.

- [ ] **Step 6: Commit**

```bash
git commit -m "feat(quantity): migrate atmosphere (USSA1976) to Quantity types"
```

---

### Task 3: Coordinates — Geodetic + EarthModel

**Files:**
- Modify: `include/vulcan/coordinates/Geodetic.hpp`
- Modify: `include/vulcan/coordinates/EarthModel.hpp`
- Modify: `tests/coordinates/test_geodetic.cpp`

- [ ] **Step 1: Read current Geodetic.hpp, EarthModel.hpp, and test_geodetic.cpp**

- [ ] **Step 2: Migrate LLA and Spherical structs**

```cpp
template <typename Scalar> struct LLA {
    Quantity<rad, Scalar> lon;
    Quantity<rad, Scalar> lat;
    Quantity<m, Scalar>   alt;
};

template <typename Scalar> struct Spherical {
    Quantity<rad, Scalar> lon;
    Quantity<rad, Scalar> lat_gc;
    Quantity<m, Scalar>   radius;
};
```

- [ ] **Step 3: Migrate EarthModel struct**

EarthModel is `double`-only (not templated). Migrate fields to Quantity:
```cpp
struct EarthModel {
    Quantity<m>              a;
    Quantity<dimensionless>  f;
    Quantity<rad / s>        omega;
    Quantity<m * m * m / (s * s)> mu;
    Quantity<m>              b;       // computed
    Quantity<dimensionless>  e2;      // computed
    Quantity<dimensionless>  e_prime2; // computed
};
```

- [ ] **Step 4: Migrate geodetic conversion functions**

```cpp
template <typename Scalar>
LLA<Scalar> ecef_to_lla(const Vec3<Quantity<m, Scalar>> &r,
                        const EarthModel &model = EarthModel::WGS84());

template <typename Scalar>
Vec3<Quantity<m, Scalar>> lla_to_ecef(const LLA<Scalar> &lla,
                                      const EarthModel &model = EarthModel::WGS84());

template <typename Scalar>
Quantity<rad, Scalar> geodetic_to_geocentric_lat(Quantity<rad, Scalar> lat_gd,
                                                 const EarthModel &model = EarthModel::WGS84());

template <typename Scalar>
Quantity<m, Scalar> radius_of_curvature_N(Quantity<rad, Scalar> lat,
                                          const EarthModel &model = EarthModel::WGS84());
// ... etc for all geodetic functions
```

Note: `Vec3<Quantity<m, Scalar>>` requires the Eigen NumTraits from Task 6 Phase 1. This is already in place.

- [ ] **Step 5: Update tests**

- [ ] **Step 6: Build and test**

Run: `./scripts/build.sh && ./scripts/test.sh`

- [ ] **Step 7: Commit**

```bash
git commit -m "feat(quantity): migrate geodetic/earth model to Quantity types"
```

---

### Task 4: Coordinates — Local Frames

**Files:**
- Modify: `include/vulcan/coordinates/FrameLocal.hpp`
- Modify: `include/vulcan/coordinates/FramePrimitives.hpp`
- Modify: `tests/coordinates/test_frames.cpp` and other frame tests

- [ ] **Step 1: Read current FrameLocal.hpp, FramePrimitives.hpp, and frame tests**

- [ ] **Step 2: Migrate CoordinateFrame struct**

```cpp
template <typename Scalar> struct CoordinateFrame {
    Vec3<Quantity<dimensionless, Scalar>> x_axis;  // unit vector
    Vec3<Quantity<dimensionless, Scalar>> y_axis;
    Vec3<Quantity<dimensionless, Scalar>> z_axis;
    Vec3<Quantity<m, Scalar>> origin;               // position in ECEF
};
```

**Important consideration:** If `Vec3<Quantity<dimensionless, Scalar>>` creates Eigen issues (e.g., dot product of dimensionless × dimensionless should give dimensionless, which needs to work with Eigen's internal ops), it may be simpler to keep the unit vectors as `Vec3<Scalar>` since they are genuinely dimensionless. Use judgment here — if Eigen NumTraits handles dimensionless cleanly, use Quantity. If it creates friction, keep axes as raw Scalar and only type the origin.

- [ ] **Step 3: Migrate local frame functions**

All angle parameters become `Quantity<rad, Scalar>`, position vectors become `Vec3<Quantity<m, Scalar>>`:

```cpp
template <typename Scalar>
CoordinateFrame<Scalar> local_ned(Quantity<rad, Scalar> lon,
                                   Quantity<rad, Scalar> lat_gd);

template <typename Scalar>
CoordinateFrame<Scalar> local_ned_at(const Vec3<Quantity<m, Scalar>> &r_ecef,
                                      const EarthModel &m = EarthModel::WGS84());

template <typename Scalar>
CoordinateFrame<Scalar> local_rail(const LLA<Scalar> &lla_origin,
                                    Quantity<rad, Scalar> azimuth,
                                    Quantity<rad, Scalar> elevation,
                                    const EarthModel &m = EarthModel::WGS84());
// ... etc
```

- [ ] **Step 4: Update tests, build, test, commit**

```bash
git commit -m "feat(quantity): migrate local frames to Quantity types"
```

---

### Task 5: Rotations — Euler Sequences

**Files:**
- Modify: `include/vulcan/rotations/EulerSequences.hpp`
- Modify: `tests/rotations/test_euler_sequences.cpp`

- [ ] **Step 1: Read current files**

- [ ] **Step 2: Migrate function signatures**

All angle parameters → `Quantity<rad, Scalar>`. DCMs remain `Mat3<Scalar>` (or `Mat3<Quantity<dimensionless, Scalar>>` if Eigen handles it). Quaternions remain `janus::Quaternion<Scalar>`.

```cpp
template <typename Scalar>
Mat3<Scalar> dcm_from_euler(Quantity<rad, Scalar> e1,
                            Quantity<rad, Scalar> e2,
                            Quantity<rad, Scalar> e3,
                            EulerSequence seq);

template <typename Scalar>
janus::Quaternion<Scalar> quaternion_from_euler(Quantity<rad, Scalar> e1,
                                                Quantity<rad, Scalar> e2,
                                                Quantity<rad, Scalar> e3,
                                                EulerSequence seq);

template <typename Scalar>
Vec3<Quantity<rad, Scalar>> euler_from_dcm(const Mat3<Scalar> &R,
                                           EulerSequence seq);

template <typename Scalar>
Vec3<Quantity<rad, Scalar>> euler_from_quaternion(const janus::Quaternion<Scalar> &q,
                                                  EulerSequence seq);
```

Note: `Vec3<Quantity<rad, Scalar>>` for euler angle returns is the correct type — all three components are angles in radians.

**Decision point for DCMs/Quaternions:** The spec says "all Vulcan APIs return Quantities, using `dimensionless` for DCMs/quaternions." However, if `Mat3<Quantity<dimensionless, Scalar>>` causes Eigen issues in practice (matrix multiplication, inverse, etc.), keep DCMs and quaternions as raw `Mat3<Scalar>` and `janus::Quaternion<Scalar>`. Document the decision. The key safety win is on the angle parameters, not the DCM internals.

- [ ] **Step 3: Migrate function bodies**

Pattern: unwrap angles with `.value()`, keep all matrix math, wrap return angles.

- [ ] **Step 4: Update tests, build, test, commit**

```bash
git commit -m "feat(quantity): migrate Euler sequences to Quantity types"
```

---

### Task 6: Rotations — DCMUtils + AxisAngle

**Files:**
- Modify: `include/vulcan/rotations/DCMUtils.hpp`
- Modify: `include/vulcan/rotations/AxisAngle.hpp`
- Modify: `tests/rotations/test_dcm_utils.cpp`, `tests/rotations/test_axis_angle.cpp`

- [ ] **Step 1: Read current files**

- [ ] **Step 2: Migrate DCMUtils**

Functions with angle parameters:
```cpp
template <typename Scalar>
Mat3<Scalar> dcm_from_small_angle(const Vec3<Quantity<rad, Scalar>> &theta);

template <typename Scalar>
Vec3<Quantity<rad, Scalar>> small_angle_from_dcm(const Mat3<Scalar> &R);

template <typename Scalar>
Mat3<Scalar> dcm_principal_axis(Quantity<rad, Scalar> theta, int axis);
```

Functions without unit parameters (skew, unskew, compose_dcm, relative_dcm, is_valid_dcm) — keep as-is if they only deal with dimensionless matrices/vectors. Migrate if parameters have physical units.

- [ ] **Step 3: Migrate AxisAngle**

```cpp
template <typename Scalar>
janus::Quaternion<Scalar> quaternion_from_axis_angle(
    const Vec3<Quantity<dimensionless, Scalar>> &axis,
    Quantity<rad, Scalar> angle);

template <typename Scalar>
janus::Quaternion<Scalar> quaternion_from_rotation_vector(
    const Vec3<Quantity<rad, Scalar>> &rot_vec);

template <typename Scalar>
Mat3<Scalar> dcm_from_axis_angle(
    const Vec3<Quantity<dimensionless, Scalar>> &axis,
    Quantity<rad, Scalar> angle);

template <typename Scalar>
std::pair<Vec3<Quantity<dimensionless, Scalar>>, Quantity<rad, Scalar>>
axis_angle_from_quaternion(const janus::Quaternion<Scalar> &q);

template <typename Scalar>
Vec3<Quantity<rad, Scalar>> rotation_vector_from_quaternion(
    const janus::Quaternion<Scalar> &q);
// ... etc
```

- [ ] **Step 4: Update tests, build, test, commit**

```bash
git commit -m "feat(quantity): migrate DCM utils and axis-angle to Quantity types"
```

---

### Task 7: Rotations — Kinematics + Interpolation

**Files:**
- Modify: `include/vulcan/rotations/RotationKinematics.hpp`
- Modify: `include/vulcan/rotations/Interpolation.hpp`
- Modify: relevant test files

- [ ] **Step 1: Read current files**

- [ ] **Step 2: Migrate RotationKinematics**

Angular velocity parameters → `Vec3<Quantity<rad_s, Scalar>>`:

```cpp
template <typename Scalar>
Vec3<Quantity<rad_s, Scalar>> omega_from_quaternion_rate(
    const janus::Quaternion<Scalar> &q,
    const janus::Quaternion<Scalar> &q_dot);

template <typename Scalar>
janus::Quaternion<Scalar> quaternion_rate_from_omega(
    const janus::Quaternion<Scalar> &q,
    const Vec3<Quantity<rad_s, Scalar>> &omega_body);

template <typename Scalar>
Vec3<Quantity<rad, Scalar>> rotation_error(
    const janus::Quaternion<Scalar> &q_actual,
    const janus::Quaternion<Scalar> &q_desired);
```

- [ ] **Step 3: Migrate Interpolation**

`slerp` and `squad` — the `t` parameter is dimensionless (interpolation factor 0-1):

```cpp
template <typename Scalar>
janus::Quaternion<Scalar> slerp(const janus::Quaternion<Scalar> &q0,
                                const janus::Quaternion<Scalar> &q1,
                                Quantity<dimensionless, Scalar> t);
```

`quat_exp`/`quat_log` — rotation vectors in radians:
```cpp
template <typename Scalar>
janus::Quaternion<Scalar> quat_exp(const Vec3<Quantity<rad, Scalar>> &v);

template <typename Scalar>
Vec3<Quantity<rad, Scalar>> quat_log(const janus::Quaternion<Scalar> &q);
```

- [ ] **Step 4: Update tests, build, test, commit**

```bash
git commit -m "feat(quantity): migrate rotation kinematics and interpolation to Quantity types"
```

---

### Task 8: Time — Epoch

**Files:**
- Modify: `include/vulcan/time/Epoch.hpp`
- Modify: `tests/time/test_time.cpp`

- [ ] **Step 1: Read current Epoch.hpp and test_time.cpp**

- [ ] **Step 2: Migrate Epoch internal storage and arithmetic**

```cpp
template <typename Scalar>
class Epoch {
    Quantity<s, Scalar> tai_seconds_;  // internal storage

public:
    explicit Epoch(Quantity<s, Scalar> tai_seconds, int delta_at = 37);

    // Factory methods take Quantity<s> or keep Julian Date as raw Scalar
    // (Julian Dates are dimensionless numbers, not seconds)
    static Epoch from_tai_seconds(Quantity<s, Scalar> tai_sec, int delta_at = 37);

    // Accessors
    [[nodiscard]] Quantity<s, Scalar> tai_seconds() const;
    [[nodiscard]] Quantity<s, Scalar> tt_seconds() const;

    // Julian Date accessors — return raw Scalar (JD is dimensionless)
    [[nodiscard]] Scalar jd_tai() const;
    // ... etc

    // Arithmetic
    Epoch operator+(Quantity<s, Scalar> dt) const;
    Epoch operator-(Quantity<s, Scalar> dt) const;
    Quantity<s, Scalar> operator-(const Epoch &other) const;
};
```

**Important:** Julian Dates are dimensionless numbers (days since epoch), not "seconds" or "days" in a typed sense. Keep JD accessors returning raw `Scalar`. Only `tai_seconds`, `tt_seconds`, arithmetic use `Quantity<s>`.

- [ ] **Step 3: Update tests, build, test**

- [ ] **Step 4: Commit**

```bash
git commit -m "feat(quantity): migrate Epoch to Quantity types"
```

---

### Task 9: Gravity Models

**Files:**
- Modify: `include/vulcan/gravity/PointMass.hpp`
- Modify: `include/vulcan/gravity/J2.hpp`
- Modify: `tests/gravity/test_point_mass.cpp`, `tests/gravity/test_j2.cpp`

- [ ] **Step 1: Read current files**

- [ ] **Step 2: Migrate gravity functions**

```cpp
namespace vulcan::gravity::point_mass {

template <typename Scalar>
Vec3<Quantity<m / (s * s), Scalar>> acceleration(
    const Vec3<Quantity<m, Scalar>> &r_ecef,
    Quantity<m * m * m / (s * s)> mu = constants::earth::mu);

template <typename Scalar>
Quantity<m * m / (s * s), Scalar> potential(
    const Vec3<Quantity<m, Scalar>> &r_ecef,
    Quantity<m * m * m / (s * s)> mu = constants::earth::mu);
}
```

Note: The `mu` default parameter now comes from the migrated constants (Task 1), which is already a `Quantity`.

- [ ] **Step 3: Update tests, build, test, commit**

```bash
git commit -m "feat(quantity): migrate gravity models to Quantity types"
```

---

### Task 10: Dynamics — PointMass

**Files:**
- Modify: `include/vulcan/dynamics/PointMass.hpp`
- Modify: `tests/dynamics/test_point_mass.cpp`

- [ ] **Step 1: Read current files**

- [ ] **Step 2: Migrate 3-DOF dynamics functions**

```cpp
template <typename Scalar>
Vec3<Quantity<m / (s * s), Scalar>> point_mass_acceleration(
    const Vec3<Quantity<N, Scalar>> &force,
    Quantity<kg, Scalar> mass);

template <typename Scalar>
Quantity<mps, Scalar> speed(const Vec3<Quantity<mps, Scalar>> &velocity);

template <typename Scalar>
Quantity<rad, Scalar> flight_path_angle(const Vec3<Quantity<mps, Scalar>> &velocity);

template <typename Scalar>
Quantity<rad, Scalar> heading_angle(const Vec3<Quantity<mps, Scalar>> &velocity);

template <typename Scalar>
Vec3<Quantity<m / (s * s), Scalar>> point_mass_acceleration_ecef(
    const Vec3<Quantity<m, Scalar>> &position,
    const Vec3<Quantity<mps, Scalar>> &velocity,
    const Vec3<Quantity<N, Scalar>> &force,
    Quantity<kg, Scalar> mass,
    const Vec3<Quantity<rad_s, Scalar>> &omega_earth);
```

- [ ] **Step 3: Update tests, build, test, commit**

```bash
git commit -m "feat(quantity): migrate point mass dynamics to Quantity types"
```

---

### Task 11: Dynamics — RigidBody

**Files:**
- Modify: `include/vulcan/dynamics/RigidBodyTypes.hpp`
- Modify: `include/vulcan/dynamics/RigidBody.hpp`
- Modify: `tests/dynamics/test_rigid_body.cpp`

- [ ] **Step 1: Read current files**

- [ ] **Step 2: Migrate state/derivative/mass structs**

```cpp
template <typename Scalar> struct RigidBodyState {
    Vec3<Quantity<m, Scalar>>    position;
    Vec3<Quantity<mps, Scalar>>  velocity_body;
    janus::Quaternion<Scalar>    attitude;       // dimensionless
    Vec3<Quantity<rad_s, Scalar>> omega_body;
};

template <typename Scalar> struct RigidBodyDerivatives {
    Vec3<Quantity<mps, Scalar>>            position_dot;
    Vec3<Quantity<m / (s * s), Scalar>>    velocity_dot;
    janus::Quaternion<Scalar>              attitude_dot;   // dimensionless rate
    Vec3<Quantity<rad / (s * s), Scalar>>  omega_dot;
};

template <typename Scalar> struct MassProperties {
    Quantity<kg, Scalar>                mass;
    Mat3<Scalar>                        inertia;  // kg·m² — keep raw if unit type is complex
};
```

- [ ] **Step 3: Migrate dynamics functions**

```cpp
template <typename Scalar>
Vec3<Quantity<m / (s * s), Scalar>> translational_dynamics(
    const Vec3<Quantity<mps, Scalar>> &velocity_body,
    const Vec3<Quantity<rad_s, Scalar>> &omega_body,
    const Vec3<Quantity<N, Scalar>> &force_body,
    Quantity<kg, Scalar> mass);

template <typename Scalar>
Vec3<Quantity<rad / (s * s), Scalar>> rotational_dynamics(
    const Vec3<Quantity<rad_s, Scalar>> &omega_body,
    const Vec3<Quantity<N * m, Scalar>> &moment_body,
    const Mat3<Scalar> &inertia);  // inertia stays raw if unit type is complex
```

- [ ] **Step 4: Update tests, build, test, commit**

```bash
git commit -m "feat(quantity): migrate rigid body dynamics to Quantity types"
```

---

### Task 12: Propulsion — Rocket

**Files:**
- Modify: `include/vulcan/propulsion/Rocket.hpp`
- Modify: `tests/propulsion/test_rocket.cpp`

- [ ] **Step 1: Read current files**

- [ ] **Step 2: Migrate propulsion functions**

```cpp
namespace vulcan::propulsion::rocket {

template <typename Scalar>
Quantity<N, Scalar> thrust_from_mdot(Quantity<kg / s, Scalar> mdot,
                                     Quantity<mps, Scalar> Ve);

template <typename Scalar>
Quantity<mps, Scalar> exhaust_velocity(Quantity<s, Scalar> Isp,
                                       Quantity<m / (s * s)> g0 = constants::physics::g0);

template <typename Scalar>
Quantity<s, Scalar> specific_impulse(Quantity<N, Scalar> thrust,
                                     Quantity<kg / s, Scalar> mdot,
                                     Quantity<m / (s * s)> g0 = constants::physics::g0);

template <typename Scalar>
Quantity<mps, Scalar> delta_v(Quantity<mps, Scalar> Ve,
                               Quantity<kg, Scalar> m0,
                               Quantity<kg, Scalar> mf);

template <typename Scalar>
Quantity<kg, Scalar> propellant_mass(Quantity<mps, Scalar> delta_v,
                                     Quantity<kg, Scalar> m0,
                                     Quantity<mps, Scalar> Ve);

template <typename Scalar>
Quantity<kg / s, Scalar> mass_flow_rate(Quantity<N, Scalar> thrust,
                                        Quantity<mps, Scalar> Ve);

template <typename Scalar>
Quantity<s, Scalar> burn_time(Quantity<kg, Scalar> propellant_mass,
                               Quantity<kg / s, Scalar> mdot);
}
```

- [ ] **Step 3: Update tests, build, test, commit**

```bash
git commit -m "feat(quantity): migrate rocket propulsion to Quantity types"
```

---

### Task 13: Full Regression + Deprecation Markers

**Files:**
- Modify: `include/vulcan/core/Units.hpp` — add deprecation markers
- No new files

- [ ] **Step 1: Run full verification**

Run: `./scripts/verify.sh`
Expected: All tests pass, all examples build.

- [ ] **Step 2: Add deprecation markers to old Units.hpp**

Add `[[deprecated("Use Quantity<unit>.in<target>() instead")]]` to each conversion function in `include/vulcan/core/Units.hpp`:

```cpp
template <typename Scalar>
[[deprecated("Use Quantity<deg>(val).in<rad>() instead")]]
constexpr Scalar deg_to_rad(const Scalar &deg) { ... }
```

- [ ] **Step 3: Build and verify deprecation warnings appear**

Run: `./scripts/build.sh`
Expected: Deprecation warnings for any code still using old conversion functions. No errors.

- [ ] **Step 4: Commit**

```bash
git commit -m "chore(quantity): deprecate old unit conversion functions"
```

---

## Task Dependency Order

```
Task 1 (Constants) → all other tasks depend on this
Task 2 (Atmosphere) → independent after Task 1
Task 3 (Geodetic) → independent after Task 1
Task 4 (Local Frames) → depends on Task 3 (uses LLA, EarthModel)
Task 5 (Euler Sequences) → independent after Task 1
Task 6 (DCM + AxisAngle) → independent after Task 1
Task 7 (Kinematics + Interp) → depends on Tasks 5-6
Task 8 (Epoch) → independent after Task 1
Task 9 (Gravity) → depends on Task 1 (uses constants as defaults)
Task 10 (PointMass Dynamics) → depends on Tasks 3, 5 (uses coordinates, rotations)
Task 11 (RigidBody) → depends on Tasks 5, 10
Task 12 (Propulsion) → depends on Task 1
Task 13 (Regression) → depends on all
```

**Parallelizable after Task 1:** Tasks 2, 3, 5, 6, 8, 9, 12 can all run in parallel.
