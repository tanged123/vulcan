# Phase 4: Rotations — Comprehensive Rotation Library

A unified, robust rotation library for Vulcan extending Janus quaternion support with all 12 Euler angle sequences, DCM utilities, axis-angle conversions, and rotation composition. All code templated on `Scalar` for Janus symbolic/numeric compatibility.

---

## User Review Required

> [!IMPORTANT]
> **Euler Angle Sequence Naming Convention**
> 
> We adopt the **intrinsic rotation** convention with **axis labels** (not angles):
> - `ZYX` = Yaw-Pitch-Roll (aerospace standard) — rotate about Z, then new Y, then new X
> - `XYZ` = Roll-Pitch-Yaw (robotics)
> - `ZXZ`, `XYX`, etc. = Proper Euler angles
> 
> The sequence string describes the rotation order: first axis → last axis.

> [!IMPORTANT]
> **DCM Convention: Active vs Passive**
> 
> Vulcan follows **active rotation** convention:
> - DCM columns are the rotated frame's basis vectors expressed in the reference frame
> - `v_ref = DCM * v_body` transforms body-fixed vectors to reference frame
> - This matches Janus `Quaternion::to_rotation_matrix()` output

> [!WARNING]
> **Gimbal Lock Handling**
> 
> All Euler angle extractions handle gimbal lock via `janus::where` for symbolic compatibility:
> - Near singularity: set one DOF to zero and compute the other from remaining DCM elements
> - Different sequences have singularities at different pitch angles (±90° for Tait-Bryan, 0°/180° for proper Euler)

> [!NOTE]
> **Design Philosophy: Extend, Don't Duplicate**
> 
> Priority is to extend Janus APIs and re-export useful functions rather than duplicate. 
> New Vulcan utilities should add value beyond what Janus provides (e.g., all 12 sequences instead of just ZYX).

---

## Existing Capabilities Audit

### Janus Provides (No Changes Needed)

| Component | API | Notes |
|-----------|-----|-------|
| `Quaternion.hpp` | `from_euler(roll, pitch, yaw)` | ZYX intrinsic only |
| `Quaternion.hpp` | `to_euler()` | Returns [roll, pitch, yaw] for ZYX |
| `Quaternion.hpp` | `from_axis_angle(axis, angle)` | Auto-normalizes axis |
| `Quaternion.hpp` | `from_rotation_vector(rot_vec)` | axis × angle representation |
| `Quaternion.hpp` | `from_rotation_matrix(R)` | Shepperd's method (numeric), trace method (symbolic) |
| `Quaternion.hpp` | `to_rotation_matrix()` | 3×3 DCM |
| `Quaternion.hpp` | `rotate(v)` | Efficient `v + 2q_vec×(q_vec×v + q_w*v)` |
| `Quaternion.hpp` | `operator*`, `conjugate`, `inverse`, `normalized` | Standard operations |
| `Rotations.hpp` | `rotation_matrix_from_euler_angles(r,p,y)` | ZYX sequence DCM |
| `Rotations.hpp` | `rotation_matrix_3d(theta, axis)` | Principal axis (0=X, 1=Y, 2=Z) |
| `Rotations.hpp` | `is_valid_rotation_matrix(R, tol)` | Orthonormality check |
| `janus::slerp` | `slerp(q0, q1, t)` | Full fidelity with shortest path |

### Vulcan Provides (Already Implemented)

| File | API | Notes |
|------|-----|-------|
| `QuaternionUtils.hpp` | `compose_rotations(q1, q2)` | q2 * q1 composition |
| `QuaternionUtils.hpp` | `relative_rotation(q_from, q_to)` | Frame-to-frame rotation |
| `QuaternionUtils.hpp` | `omega_from_quaternion_rate(q, q_dot)` | Angular velocity extraction |
| `QuaternionUtils.hpp` | `quaternion_rate_from_omega(q, omega)` | Kinematic equation |
| `QuaternionUtils.hpp` | `rotation_vector_from_quaternion(q)` | With singularity handling |
| `BodyFrames.hpp` | `body_from_euler(ned, yaw, pitch, roll)` | ZYX via quaternion |
| `BodyFrames.hpp` | `euler_from_body(body, ned)` | ZYX with gimbal lock |
| `BodyFrames.hpp` | `body_from_quaternion(ned, q)` | Quaternion-based construction |
| `BodyFrames.hpp` | `quaternion_from_body(body, ned)` | DCM→quaternion via frame |
| `CoordinateFrame.hpp` | `dcm()`, `dcm_inverse()` | DCM composition/extraction |
| `CoordinateFrame.hpp` | `quaternion()`, `from_quaternion()` | Quaternion interface |

---

## Proposed Changes

### New File: Rotations.hpp (Vulcan Extension)

#### [NEW] [Rotations.hpp](file:///home/tanged/sources/vulcan/include/vulcan/rotations/Rotations.hpp)

Central rotation utilities header extending Janus capabilities.

```cpp
namespace vulcan {

// =============================================================================
// Euler Angle Sequences (All 12)
// =============================================================================

/// Enumeration of all 12 Euler angle sequences
enum class EulerSequence {
    // Tait-Bryan (3 distinct axes)
    XYZ, XZY, YXZ, YZX, ZXY, ZYX,
    // Proper Euler (symmetric axes)  
    XYX, XZX, YXY, YZY, ZXZ, ZYZ
};

/// Create quaternion from Euler angles with specified sequence
/// @param e1 First rotation angle [rad]
/// @param e2 Second rotation angle [rad]
/// @param e3 Third rotation angle [rad]
/// @param seq Euler angle sequence
template<typename Scalar>
janus::Quaternion<Scalar> quaternion_from_euler(Scalar e1, Scalar e2, Scalar e3,
                                                 EulerSequence seq);

/// Create DCM from Euler angles with specified sequence
template<typename Scalar>
Mat3<Scalar> dcm_from_euler(Scalar e1, Scalar e2, Scalar e3,
                            EulerSequence seq);

/// Extract Euler angles from quaternion for specified sequence
/// @return [e1, e2, e3] angles [rad]
template<typename Scalar>
Vec3<Scalar> euler_from_quaternion(const janus::Quaternion<Scalar>& q,
                                    EulerSequence seq);

/// Extract Euler angles from DCM for specified sequence
template<typename Scalar>
Vec3<Scalar> euler_from_dcm(const Mat3<Scalar>& R, EulerSequence seq);

// =============================================================================
// DCM Utilities
// =============================================================================

/// Compose two DCMs: R_combined = R2 * R1 (apply R1 first, then R2)
template<typename Scalar>
Mat3<Scalar> compose_dcm(const Mat3<Scalar>& R1, const Mat3<Scalar>& R2);

/// Relative rotation DCM: R_rel transforms from frame A to frame B
/// R_rel = R_B * R_A^T
template<typename Scalar>
Mat3<Scalar> relative_dcm(const Mat3<Scalar>& R_A, const Mat3<Scalar>& R_B);

/// Small-angle DCM approximation: R ≈ I + [θ]×
/// Valid for |θ| << 1 rad
template<typename Scalar>
Mat3<Scalar> dcm_from_small_angle(const Vec3<Scalar>& theta);

/// Extract small-angle vector from near-identity DCM
/// θ ≈ [R32-R23, R13-R31, R21-R12] / 2
template<typename Scalar>
Vec3<Scalar> small_angle_from_dcm(const Mat3<Scalar>& R);

/// Skew-symmetric matrix from vector: [v]× such that [v]× * u = v × u
template<typename Scalar>
Mat3<Scalar> skew(const Vec3<Scalar>& v);

/// Extract vector from skew-symmetric matrix
template<typename Scalar>
Vec3<Scalar> unskew(const Mat3<Scalar>& S);

// =============================================================================
// Axis-Angle Utilities
// =============================================================================

/// Convert axis-angle to quaternion (wrapper for janus API)
template<typename Scalar>
janus::Quaternion<Scalar> quaternion_from_axis_angle(const Vec3<Scalar>& axis,
                                                      Scalar angle);

/// Convert axis-angle to DCM (Rodrigues' formula)
template<typename Scalar>
Mat3<Scalar> dcm_from_axis_angle(const Vec3<Scalar>& axis, Scalar angle);

/// Extract axis and angle from quaternion
/// @return Pair of (axis, angle) where axis is unit vector
template<typename Scalar>
std::pair<Vec3<Scalar>, Scalar> axis_angle_from_quaternion(
    const janus::Quaternion<Scalar>& q);

/// Extract axis and angle from DCM
template<typename Scalar>
std::pair<Vec3<Scalar>, Scalar> axis_angle_from_dcm(const Mat3<Scalar>& R);

// =============================================================================
// Principal Rotation Vector
// =============================================================================

/// Convert rotation vector (axis × angle) to quaternion
template<typename Scalar>
janus::Quaternion<Scalar> quaternion_from_rotation_vector(
    const Vec3<Scalar>& rot_vec);

/// Convert rotation vector to DCM
template<typename Scalar>
Mat3<Scalar> dcm_from_rotation_vector(const Vec3<Scalar>& rot_vec);

/// Extract rotation vector from quaternion
template<typename Scalar>
Vec3<Scalar> rotation_vector_from_quaternion(
    const janus::Quaternion<Scalar>& q);

/// Extract rotation vector from DCM
template<typename Scalar>
Vec3<Scalar> rotation_vector_from_dcm(const Mat3<Scalar>& R);

// =============================================================================
// Rotation Composition
// =============================================================================

/// Compose quaternion rotations: result = q2 * q1 (apply q1 first)
template<typename Scalar>
janus::Quaternion<Scalar> compose_rotations(
    const janus::Quaternion<Scalar>& q1,
    const janus::Quaternion<Scalar>& q2);

/// Relative rotation quaternion from frame A to frame B
template<typename Scalar>
janus::Quaternion<Scalar> relative_rotation(
    const janus::Quaternion<Scalar>& q_from,
    const janus::Quaternion<Scalar>& q_to);

/// Rotation "difference" as rotation vector: log(q_to * q_from^-1)
template<typename Scalar>
Vec3<Scalar> rotation_error(const janus::Quaternion<Scalar>& q_actual,
                            const janus::Quaternion<Scalar>& q_desired);

// =============================================================================
// Angular Velocity Kinematics
// =============================================================================

/// Compute angular velocity from quaternion rate (body frame)
template<typename Scalar>
Vec3<Scalar> omega_from_quaternion_rate(const janus::Quaternion<Scalar>& q,
                                         const janus::Quaternion<Scalar>& q_dot);

/// Compute quaternion rate from angular velocity (body frame omega)
template<typename Scalar>
janus::Quaternion<Scalar> quaternion_rate_from_omega(
    const janus::Quaternion<Scalar>& q,
    const Vec3<Scalar>& omega_body);

/// Compute angular velocity from DCM rate: ω = unskew(R^T * R_dot)
template<typename Scalar>
Vec3<Scalar> omega_from_dcm_rate(const Mat3<Scalar>& R,
                                  const Mat3<Scalar>& R_dot);

// =============================================================================
// Interpolation
// =============================================================================

/// Re-export janus::slerp for convenience
using janus::slerp;

/// Squad (Spherical Quadratic Interpolation) for smooth quaternion curves
template<typename Scalar>
janus::Quaternion<Scalar> squad(const janus::Quaternion<Scalar>& q0,
                                 const janus::Quaternion<Scalar>& q1,
                                 const janus::Quaternion<Scalar>& s0,
                                 const janus::Quaternion<Scalar>& s1,
                                 Scalar t);

/// Compute Squad control point for smooth interpolation
template<typename Scalar>
janus::Quaternion<Scalar> squad_control_point(
    const janus::Quaternion<Scalar>& q_prev,
    const janus::Quaternion<Scalar>& q_curr,
    const janus::Quaternion<Scalar>& q_next);

} // namespace vulcan
```

---

### Implementation Details: Euler Sequences

#### Core Algorithm: Axis Index Mapping

```cpp
// Map EulerSequence to axis indices [0=X, 1=Y, 2=Z]
constexpr std::array<int, 3> euler_axes(EulerSequence seq) {
    switch (seq) {
        case EulerSequence::XYZ: return {0, 1, 2};
        case EulerSequence::XZY: return {0, 2, 1};
        case EulerSequence::YXZ: return {1, 0, 2};
        case EulerSequence::YZX: return {1, 2, 0};
        case EulerSequence::ZXY: return {2, 0, 1};
        case EulerSequence::ZYX: return {2, 1, 0};  // Aerospace standard
        case EulerSequence::XYX: return {0, 1, 0};
        case EulerSequence::XZX: return {0, 2, 0};
        case EulerSequence::YXY: return {1, 0, 1};
        case EulerSequence::YZY: return {1, 2, 1};
        case EulerSequence::ZXZ: return {2, 0, 2};
        case EulerSequence::ZYZ: return {2, 1, 2};
    }
}

// Check if sequence is proper Euler (symmetric axes)
constexpr bool is_proper_euler(EulerSequence seq) {
    auto axes = euler_axes(seq);
    return axes[0] == axes[2];
}
```

#### DCM from Euler (Generic Implementation)

```cpp
template<typename Scalar>
Mat3<Scalar> dcm_from_euler(Scalar e1, Scalar e2, Scalar e3, EulerSequence seq) {
    auto axes = euler_axes(seq);
    
    // R = R_axis[2](e3) * R_axis[1](e2) * R_axis[0](e1)
    // Using intrinsic rotations: compose right-to-left
    Mat3<Scalar> R1 = janus::rotation_matrix_3d(e1, axes[0]);
    Mat3<Scalar> R2 = janus::rotation_matrix_3d(e2, axes[1]);
    Mat3<Scalar> R3 = janus::rotation_matrix_3d(e3, axes[2]);
    
    return R3 * R2 * R1;
}
```

#### Euler from DCM (with Gimbal Lock Handling)

For **Tait-Bryan sequences** (e.g., ZYX):
- Singularity at pitch = ±90° (cos(pitch) = 0)
- Convention: set roll = 0 at singularity

For **Proper Euler sequences** (e.g., ZXZ):
- Singularity at nutation = 0° or 180° (sin(nutation) = 0)
- Convention: set precession = 0 at singularity

```cpp
template<typename Scalar>  
Vec3<Scalar> euler_from_dcm_zyx(const Mat3<Scalar>& R) {
    // ZYX: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    // R[2,0] = -sin(pitch)
    // R[0,0] = cos(yaw)*cos(pitch), R[1,0] = sin(yaw)*cos(pitch)
    // R[2,1] = cos(pitch)*sin(roll), R[2,2] = cos(pitch)*cos(roll)
    
    Scalar sin_pitch = -R(2, 0);
    Scalar pitch = janus::asin(sin_pitch);
    
    Scalar eps = Scalar(1e-6);
    Scalar cos_pitch = janus::sqrt(Scalar(1) - sin_pitch * sin_pitch);
    Scalar is_gimbal = cos_pitch < eps;
    
    // Normal case
    Scalar yaw_n = janus::atan2(R(1, 0), R(0, 0));
    Scalar roll_n = janus::atan2(R(2, 1), R(2, 2));
    
    // Gimbal lock: roll = 0, yaw from other elements
    Scalar roll_g = Scalar(0);
    // At pitch = +90°: R[0,1] = -sin(yaw-roll), R[1,1] = cos(yaw-roll)
    // With roll=0: yaw = atan2(-R[0,1], R[1,1])
    Scalar yaw_g = janus::atan2(-R(0, 1), R(1, 1));
    
    Vec3<Scalar> euler;
    euler(0) = janus::where(is_gimbal, yaw_g, yaw_n);    // yaw
    euler(1) = pitch;                                     // pitch
    euler(2) = janus::where(is_gimbal, roll_g, roll_n);  // roll
    
    return euler;
}
```

---

### File Organization

#### Directory Structure

```
include/vulcan/rotations/
├── Rotations.hpp         # Main header (includes all)
├── EulerSequences.hpp    # All 12 Euler sequences
├── DCMUtils.hpp          # DCM utilities (skew, compose, etc.)
├── AxisAngle.hpp         # Axis-angle and rotation vector
├── RotationKinematics.hpp # Angular velocity, rates
└── Interpolation.hpp     # Squad interpolation

tests/rotations/
├── test_euler_sequences.cpp
├── test_dcm_utils.cpp
├── test_axis_angle.cpp
├── test_rotation_kinematics.cpp
└── test_interpolation.cpp
```

---

### Changes to Existing Files

#### [MODIFY] [vulcan.hpp](file:///home/tanged/sources/vulcan/include/vulcan/vulcan.hpp)

Add rotation module includes:

```diff
 // Coordinate systems
 #include <vulcan/coordinates/EarthModel.hpp>
 #include <vulcan/coordinates/CoordinateFrame.hpp>
 #include <vulcan/coordinates/Geodetic.hpp>
 #include <vulcan/coordinates/LocalFrames.hpp>
 #include <vulcan/coordinates/BodyFrames.hpp>
+#include <vulcan/coordinates/QuaternionUtils.hpp>
 #include <vulcan/coordinates/Transforms.hpp>
+
+// Rotations
+#include <vulcan/rotations/Rotations.hpp>
```

#### [MODIFY] [QuaternionUtils.hpp](file:///home/tanged/sources/vulcan/include/vulcan/coordinates/QuaternionUtils.hpp)

Keep existing content but add deprecation notes for functions moving to `Rotations.hpp`:

```cpp
/// @deprecated Use vulcan::compose_rotations from <vulcan/rotations/Rotations.hpp>
```

Actually, better approach: keep `QuaternionUtils.hpp` as-is for backward compatibility and have `Rotations.hpp` re-export its functions. This follows the "extend, don't break" philosophy.

---

## Verification Plan

### Unit Tests

| Test Suite | Test Cases |
|------------|------------|
| `test_euler_sequences` | All 12 sequences: roundtrip (euler→DCM→euler), roundtrip (euler→quat→euler), identity rotation, 90° principal rotations, gimbal lock cases |
| `test_dcm_utils` | `compose_dcm` associativity, `relative_dcm` correctness, `skew`/`unskew` roundtrip, small-angle approximation accuracy |
| `test_axis_angle` | Zero angle → identity, 180° rotation, roundtrip via quaternion and DCM, axis normalization |
| `test_rotation_kinematics` | Quaternion rate ↔ omega consistency, constant omega integration |
| `test_interpolation` | Slerp endpoints, slerp midpoint, Squad smoothness |

### Reference Validation

```cpp
// ZYX (aerospace standard) - matches existing Janus
// Verify against rotation_matrix_from_euler_angles
EXPECT_NEAR(dcm_from_euler(roll, pitch, yaw, EulerSequence::ZYX),
            janus::rotation_matrix_from_euler_angles(roll, pitch, yaw), 1e-12);

// XYZ (robotics) - common alternative
// R = Rx(e1) * Ry(e2) * Rz(e3)

// ZXZ (classical mechanics / orbit mechanics)
// R = Rz(precession) * Rx(nutation) * Rz(spin)
```

### Symbolic Compatibility Tests

```cpp
TEST(SymbolicRotations, EulerSequenceDCM) {
    using Scalar = janus::SymbolicScalar;
    
    Scalar e1 = janus::make_symbol("e1");
    Scalar e2 = janus::make_symbol("e2");  
    Scalar e3 = janus::make_symbol("e3");
    
    auto R = vulcan::dcm_from_euler(e1, e2, e3, vulcan::EulerSequence::ZYX);
    
    // Verify it can build symbolic graph
    janus::Function f({e1, e2, e3}, janus::flatten(R));
    
    // Evaluate numerically and compare
    auto result = f(0.1, 0.2, 0.3);
    auto expected = janus::rotation_matrix_from_euler_angles(0.1, 0.2, 0.3);
    // Compare...
}
```

---

## Implementation Priority

### Phase 4a: Core Euler Sequences (Priority: High)

| Task | Effort | Dependencies |
|------|--------|--------------|
| `EulerSequence` enum and axis mapping | 1 hour | None |
| `dcm_from_euler` for all 12 sequences | 2 hours | `rotation_matrix_3d` |
| `quaternion_from_euler` for all 12 sequences | 1 hour | `dcm_from_euler` |
| `euler_from_dcm` for Tait-Bryan (6) | 3 hours | Gimbal lock handling |
| `euler_from_dcm` for Proper Euler (6) | 2 hours | Different singularity |
| Tests for all sequences | 4 hours | Above |

### Phase 4b: DCM Utilities (Priority: Medium)

| Task | Effort | Dependencies |
|------|--------|--------------|
| `skew` / `unskew` | 30 min | None |
| `compose_dcm` / `relative_dcm` | 30 min | None |
| `dcm_from_small_angle` / `small_angle_from_dcm` | 1 hour | `skew` |
| Tests | 2 hours | Above |

### Phase 4c: Axis-Angle & Rotation Vector (Priority: Medium)

| Task | Effort | Dependencies |
|------|--------|--------------|
| `dcm_from_axis_angle` (Rodrigues) | 1 hour | `skew` |
| `axis_angle_from_dcm` | 1.5 hours | Singularity handling |
| `rotation_vector_from_dcm` | 30 min | `axis_angle_from_dcm` |
| Tests | 2 hours | Above |

### Phase 4d: Rotation Composition & Kinematics (Priority: Medium)

| Task | Effort | Dependencies |
|------|--------|--------------|
| Move/re-export from `QuaternionUtils.hpp` | 1 hour | None |
| `rotation_error` | 30 min | `rotation_vector_from_quaternion` |
| `omega_from_dcm_rate` | 30 min | `unskew` |
| Tests | 1 hour | Above |

### Phase 4e: Advanced Interpolation (Priority: Low)

| Task | Effort | Dependencies |
|------|--------|--------------|
| `squad` | 2 hours | `slerp` |
| `squad_control_point` | 1 hour | Quaternion log/exp |
| Tests | 1 hour | Above |

---

## File Summary

| Type | Path |
|------|------|
| [NEW] | `include/vulcan/rotations/Rotations.hpp` |
| [NEW] | `include/vulcan/rotations/EulerSequences.hpp` |
| [NEW] | `include/vulcan/rotations/DCMUtils.hpp` |
| [NEW] | `include/vulcan/rotations/AxisAngle.hpp` |
| [NEW] | `include/vulcan/rotations/RotationKinematics.hpp` |
| [NEW] | `include/vulcan/rotations/Interpolation.hpp` |
| [MODIFY] | `include/vulcan/vulcan.hpp` |
| [NEW] | `tests/rotations/test_euler_sequences.cpp` |
| [NEW] | `tests/rotations/test_dcm_utils.cpp` |
| [NEW] | `tests/rotations/test_axis_angle.cpp` |
| [NEW] | `tests/rotations/test_rotation_kinematics.cpp` |
| [NEW] | `tests/rotations/test_interpolation.cpp` |

---

## Refactoring Opportunities

After the new rotations API is implemented, the following existing files can benefit from refactoring:

### High Priority

| File | Opportunity |
|------|-------------|
| [BodyFrames.hpp](file:///home/tanged/sources/vulcan/include/vulcan/coordinates/BodyFrames.hpp) | `euler_from_body()` (lines 123-181) can delegate to `euler_from_dcm(dcm, EulerSequence::ZYX)` |
| [QuaternionUtils.hpp](file:///home/tanged/sources/vulcan/include/vulcan/coordinates/QuaternionUtils.hpp) | `rotation_vector_from_quaternion()`, `quaternion_from_axis_angle()` can re-export from `Rotations.hpp` |

### Medium Priority

| File | Opportunity |
|------|-------------|
| [CoordinateFrame.hpp](file:///home/tanged/sources/vulcan/include/vulcan/coordinates/CoordinateFrame.hpp) | Add `from_euler()` factory accepting any `EulerSequence` |
| [Transforms.hpp](file:///home/tanged/sources/vulcan/include/vulcan/coordinates/Transforms.hpp) | Use `skew()` utility in transport rate calculations |
| [test_transforms.cpp](file:///home/tanged/sources/vulcan/tests/coordinates/test_transforms.cpp) | Add parameterized tests for all 12 Euler sequences |
| [coordinate_demo.cpp](file:///home/tanged/sources/vulcan/examples/coordinates/coordinate_demo.cpp) | Demonstrate other Euler sequences (e.g., ZXZ for orbit mechanics) |

> [!NOTE]
> All refactoring is **optional** and should be done **after** the core rotations API is implemented.

---

## Design Decisions Log

### Decision 1: Separate `rotations/` Directory vs Extend `coordinates/`

**Chosen**: New `include/vulcan/rotations/` directory

**Rationale**: 
- Rotations are mathematically separate from coordinate frames
- Keeps coordinate files focused on geodesy and frames
- Easier to find rotation-specific utilities
- Mirrors Janus structure (`janus/math/Rotations.hpp`, `janus/math/Quaternion.hpp`)

### Decision 2: Enum vs Template for Euler Sequences

**Chosen**: `EulerSequence` enum with runtime dispatch

**Rationale**:
- Cleaner user API: `dcm_from_euler(r, p, y, EulerSequence::ZYX)`
- Enum is more discoverable than template tags
- Runtime dispatch has negligible performance cost
- Can still use `if constexpr` internally for optimization

### Decision 3: Keep QuaternionUtils.hpp vs Merge into Rotations

**Chosen**: Keep separate, re-export in Rotations.hpp

**Rationale**:
- Backward compatibility with existing code
- QuaternionUtils focuses on Vulcan-specific quaternion extensions
- Rotations.hpp is the new comprehensive entry point
- Clear migration path: use either, both work
