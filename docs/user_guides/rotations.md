# Vulcan Rotations Library User Guide

This guide covers the unified rotations library in Vulcan, providing comprehensive utilities for 3D rotation representations including all 12 Euler angle sequences, quaternions, DCMs, and axis-angle conversions.

## Quick Start

```cpp
#include <vulcan/rotations/Rotations.hpp>

using namespace vulcan;

// Create a rotation from aerospace-standard Euler angles (yaw-pitch-roll)
double yaw = 0.5, pitch = 0.3, roll = 0.1;  // radians
auto R = dcm_from_euler(yaw, pitch, roll, EulerSequence::ZYX);
auto q = quaternion_from_euler(yaw, pitch, roll, EulerSequence::ZYX);

// Extract angles back
auto euler = euler_from_dcm(R, EulerSequence::ZYX);  // [yaw, pitch, roll]
```

## Euler Angle Sequences

Vulcan supports all 12 Euler angle sequences, organized into two categories:

### Tait-Bryan Sequences (3 distinct axes)

| Sequence | Convention | Common Use |
|----------|------------|------------|
| `ZYX` | Yaw-Pitch-Roll | Aerospace (default) |
| `XYZ` | Roll-Pitch-Yaw | Robotics |
| `YXZ` | | Computer graphics |
| `ZXY` | | |
| `XZY` | | |
| `YZX` | | |

### Proper Euler Sequences (symmetric axes)

| Sequence | Convention | Common Use |
|----------|------------|------------|
| `ZXZ` | Precession-Nutation-Spin | Classical mechanics, orbital |
| `ZYZ` | | Robotics (wrist) |
| `XYX` | | |
| `XZX` | | |
| `YXY` | | |
| `YZY` | | |

### Creating Rotations from Euler Angles

```cpp
// Aerospace: ZYX (yaw, pitch, roll)
auto R = dcm_from_euler(yaw, pitch, roll, EulerSequence::ZYX);

// Robotics: XYZ (roll, pitch, yaw)  
auto R = dcm_from_euler(roll, pitch, yaw, EulerSequence::XYZ);

// Classical mechanics: ZXZ (precession, nutation, spin)
auto R = dcm_from_euler(precession, nutation, spin, EulerSequence::ZXZ);
```

### Extracting Euler Angles

```cpp
// From DCM
Vec3<double> euler = euler_from_dcm(R, EulerSequence::ZYX);
// euler(0) = yaw, euler(1) = pitch, euler(2) = roll

// From quaternion
Vec3<double> euler = euler_from_quaternion(q, EulerSequence::ZYX);
```

### Gimbal Lock Handling

All Euler angle extractions handle gimbal lock robustly using `janus::where` for symbolic compatibility:

- **Tait-Bryan**: At ±90° pitch, roll is set to 0
- **Proper Euler**: At 0° or 180° nutation, spin is set to 0

```cpp
// This works even at gimbal lock
double pitch = M_PI / 2.0;  // 90 degrees
auto R = dcm_from_euler(0.5, pitch, 0.3, EulerSequence::ZYX);
auto euler = euler_from_dcm(R, EulerSequence::ZYX);
// euler(2) will be 0 (roll undefined at gimbal lock)
```

## DCM Utilities

### Skew-Symmetric Matrix

```cpp
Vec3<double> omega;
omega << 0.1, 0.2, 0.3;

// Create skew matrix: [ω]× such that [ω]× * v = ω × v
Mat3<double> S = skew(omega);

// Extract vector back
Vec3<double> omega_back = unskew(S);
```

### DCM Composition

```cpp
// Compose rotations: R_combined = R2 * R1 (apply R1 first, then R2)
auto R_combined = compose_dcm(R1, R2);

// Compute relative DCM from frame A to frame B
auto R_rel = relative_dcm(R_A, R_B);  // v_B = R_rel * v_A
```

### Small-Angle Approximation

For linearized analysis and estimation:

```cpp
Vec3<double> theta;  // Small rotation angles (< 0.1 rad)
theta << 0.01, -0.02, 0.015;

// First-order approximation: R ≈ I + [θ]×
auto R_approx = dcm_from_small_angle(theta);

// Extract small angles from near-identity DCM
auto theta_back = small_angle_from_dcm(R_approx);
```

### DCM Validation

```cpp
if (is_valid_dcm(R)) {
    // Matrix is a valid rotation (det ≈ 1, orthonormal)
}
```

## Axis-Angle and Rotation Vectors

### Creating Rotations

```cpp
// From axis and angle
Vec3<double> axis;
axis << 0.0, 0.0, 1.0;  // Z-axis
double angle = M_PI / 4.0;  // 45 degrees

auto q = quaternion_from_axis_angle(axis, angle);
auto R = dcm_from_axis_angle(axis, angle);  // Uses Rodrigues' formula

// From rotation vector (axis * angle)
Vec3<double> rot_vec;
rot_vec << 0.3, -0.2, 0.5;  // Combined axis-angle representation

auto q = quaternion_from_rotation_vector(rot_vec);
auto R = dcm_from_rotation_vector(rot_vec);
```

### Extracting Axis-Angle

```cpp
// From quaternion
auto [axis, angle] = axis_angle_from_quaternion(q);
Vec3<double> rot_vec = rotation_vector_from_quaternion(q);

// From DCM
auto [axis, angle] = axis_angle_from_dcm(R);
Vec3<double> rot_vec = rotation_vector_from_dcm(R);
```

## Rotation Kinematics

### Quaternion Rate ↔ Angular Velocity

```cpp
// Given quaternion and angular velocity, compute quaternion rate
Vec3<double> omega_body;  // Angular velocity in body frame [rad/s]
omega_body << 0.1, 0.05, 0.02;

auto q_dot = quaternion_rate_from_omega(q, omega_body);

// Given quaternion and its derivative, extract angular velocity
Vec3<double> omega = omega_from_quaternion_rate(q, q_dot);
```

### DCM Rate ↔ Angular Velocity

```cpp
// Compute DCM rate from angular velocity
Mat3<double> R_dot = dcm_rate_from_omega(R, omega_body);

// Extract angular velocity from DCM and its rate
Vec3<double> omega = omega_from_dcm_rate(R, R_dot);
```

### Rotation Composition and Error

```cpp
// Compose quaternions: result = q2 * q1 (apply q1 first)
auto q_combined = compose_rotations(q1, q2);

// Compute relative rotation
auto q_rel = relative_rotation(q_from, q_to);

// Compute rotation error as rotation vector (useful for control)
Vec3<double> error = rotation_error(q_actual, q_desired);
// For P-control: torque = K * error
```

## Interpolation

### Slerp (Spherical Linear Interpolation)

```cpp
// Interpolate between two orientations
auto q_interp = slerp(q0, q1, t);  // t ∈ [0, 1]

// Example: smooth transition
for (double t = 0.0; t <= 1.0; t += 0.1) {
    auto q = slerp(q_start, q_end, t);
    // Use interpolated orientation
}
```

### Squad (Spherical Quadrangle Interpolation)

For C1-continuous rotation curves through waypoints:

```cpp
// Given waypoints q0, q1, q2, q3...
// Compute control points for segment [q1, q2]
auto s1 = squad_control_point(q0, q1, q2);
auto s2 = squad_control_point(q1, q2, q3);

// Interpolate with C1 continuity
auto q_interp = squad(q1, q2, s1, s2, t);
```

## Symbolic Computation

All functions are templated and work with `janus::SymbolicScalar` for optimization:

```cpp
using Scalar = janus::SymbolicScalar;

Scalar yaw = janus::sym("yaw");
Scalar pitch = janus::sym("pitch");
Scalar roll = janus::sym("roll");

// Create symbolic DCM
auto R = dcm_from_euler(yaw, pitch, roll, EulerSequence::ZYX);

// R(0,0) is now a symbolic expression: cos(yaw)*cos(pitch)
std::cout << R(0, 0) << std::endl;  // Prints symbolic expression

// Create CasADi function for optimization
janus::Function f("attitude_dcm", {yaw, pitch, roll}, 
                  {R(0,0), R(1,0), R(2,0)});
```

## Integration with CoordinateFrame

The rotations library integrates with Vulcan's coordinate frame system:

```cpp
#include <vulcan/coordinates/BodyFrames.hpp>

// Create body frame from Euler angles
auto body = body_from_euler(ned, yaw, pitch, roll);

// Extract Euler angles (uses euler_from_dcm internally)
Vec3<double> euler = euler_from_body(body, ned);
// euler(0) = yaw, euler(1) = pitch, euler(2) = roll
```

## Graph Visualization

Export computational graphs as interactive HTML for debugging and documentation:

```cpp
using Scalar = janus::SymbolicScalar;

Scalar yaw = janus::sym("yaw");
Scalar pitch = janus::sym("pitch");
Scalar roll = janus::sym("roll");

auto R = dcm_from_euler(yaw, pitch, roll, EulerSequence::ZYX);

// Export as interactive HTML - opens in any browser
janus::export_graph_html(R(0, 0), "graph_dcm_r00", "DCM_R00_ZYX");
// Creates: graph_dcm_r00.html
```

> [!TIP]
> **Interactive Examples** - Explore the computational graphs:
> - [🔍 DCM R[0,0] = cos(yaw)·cos(pitch)](../examples/graph_dcm_00.html)
> - [🔍 DCM R[2,0] = -sin(pitch)](../examples/graph_dcm_20.html)
> - [🔍 Rodrigues Formula](../examples/graph_rodrigues.html)
> - [🔍 LLA to ECEF (X coordinate)](../examples/graph_lla_to_ecef_x.html)
> - [🔍 ECEF to LLA (Vermeille algorithm)](../examples/graph_ecef_to_lla_lat.html)

The generated HTML files are self-contained and can be:
- Opened in any web browser
- Shared for documentation
- Used to visualize complex symbolic expressions


## API Reference

### EulerSequences.hpp

| Function | Description |
|----------|-------------|
| `dcm_from_euler(e1, e2, e3, seq)` | Create DCM from Euler angles |
| `quaternion_from_euler(e1, e2, e3, seq)` | Create quaternion from Euler angles |
| `euler_from_dcm(R, seq)` | Extract Euler angles from DCM |
| `euler_from_quaternion(q, seq)` | Extract Euler angles from quaternion |
| `euler_axes(seq)` | Get axis indices [0=X, 1=Y, 2=Z] |
| `is_proper_euler(seq)` | Check if sequence is proper Euler |
| `euler_sequence_name(seq)` | Get sequence name as string |

### DCMUtils.hpp

| Function | Description |
|----------|-------------|
| `skew(v)` | Create skew-symmetric matrix from vector |
| `unskew(S)` | Extract vector from skew-symmetric matrix |
| `compose_dcm(R1, R2)` | Compose DCMs: R2 * R1 |
| `relative_dcm(R_A, R_B)` | Relative DCM from A to B |
| `dcm_from_small_angle(theta)` | First-order DCM approximation |
| `small_angle_from_dcm(R)` | Extract small angles from near-identity DCM |
| `is_valid_dcm(R)` | Check if matrix is valid rotation |
| `dcm_principal_axis(theta, axis)` | DCM for principal axis rotation |

### AxisAngle.hpp

| Function | Description |
|----------|-------------|
| `quaternion_from_axis_angle(axis, angle)` | Quaternion from axis-angle |
| `quaternion_from_rotation_vector(rot_vec)` | Quaternion from rotation vector |
| `dcm_from_axis_angle(axis, angle)` | DCM via Rodrigues' formula |
| `dcm_from_rotation_vector(rot_vec)` | DCM from rotation vector |
| `axis_angle_from_quaternion(q)` | Extract axis and angle from quaternion |
| `axis_angle_from_dcm(R)` | Extract axis and angle from DCM |
| `rotation_vector_from_quaternion(q)` | Rotation vector from quaternion |
| `rotation_vector_from_dcm(R)` | Rotation vector from DCM |

### RotationKinematics.hpp

| Function | Description |
|----------|-------------|
| `omega_from_quaternion_rate(q, q_dot)` | Angular velocity from quaternion rate |
| `quaternion_rate_from_omega(q, omega)` | Quaternion rate from angular velocity |
| `omega_from_dcm_rate(R, R_dot)` | Angular velocity from DCM rate |
| `dcm_rate_from_omega(R, omega)` | DCM rate from angular velocity |
| `compose_rotations(q1, q2)` | Compose quaternion rotations |
| `relative_rotation(q_from, q_to)` | Relative rotation quaternion |
| `rotation_error(q_actual, q_desired)` | Rotation error as rotation vector |

### Interpolation.hpp

| Function | Description |
|----------|-------------|
| `slerp(q0, q1, t)` | Spherical linear interpolation |
| `quat_exp(v)` | Quaternion exponential |
| `quat_log(q)` | Quaternion logarithm |
| `squad_control_point(q_prev, q_curr, q_next)` | Squad control point |
| `squad(q0, q1, s0, s1, t)` | Squad interpolation |

## Example: Complete Demo

See `examples/rotations/rotations_demo.cpp` for a comprehensive demonstration of all features.

```bash
# Build and run the demo
./scripts/build.sh
./build/examples/rotations_demo
```
