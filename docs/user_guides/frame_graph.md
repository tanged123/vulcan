# Frame Graph

This guide covers Vulcan's frame-tree API for extensible coordinate transforms:

- `FrameRegistry` stores frame topology.
- `TransformProvider` defines child/parent math per edge.
- `FrameContext<Scalar>` ties topology and providers together.

## Quick Start

```cpp
#include <vulcan/vulcan.hpp>

using namespace vulcan;

FrameContext<double> ctx;
ctx.set_ecef(ConstantOmegaRotation::from_wgs84(), 3600.0);
ctx.set_ned(-77.0367 * constants::angle::deg2rad,
            38.8951 * constants::angle::deg2rad);
ctx.set_body_euler(45.0 * constants::angle::deg2rad,
                   5.0 * constants::angle::deg2rad,
                   10.0 * constants::angle::deg2rad);

Vec3<double> v_body;
v_body << 100.0, 0.0, 0.0;

Vec3<double> v_ned = ctx.transform(v_body, FRAME_BODY, FRAME_NED);
Vec3<double> v_eci = ctx.transform(v_body, FRAME_BODY, FRAME_ECI);
```

## Built-In Hierarchy

Default tree (`FrameRegistry::default_aerospace()`):

- `ECI` (root)
- `ECEF` child of `ECI`
- `NED`, `ENU`, `Geocentric`, `Rail`, `CDA` children of `ECEF`
- `Body` child of `NED`
- `Wind`, `Stability` children of `Body`

## Custom Frames

```cpp
auto sensor_q =
    janus::Quaternion<double>::from_euler(0.0, 10.0 * constants::angle::deg2rad, 0.0);
auto sensor_id = ctx.add_frame(
    "Sensor",
    FRAME_BODY,
    std::make_shared<QuaternionProvider<double>>(sensor_q));

Vec3<double> v_sensor;
v_sensor << 1.0, 0.0, 0.0;
auto v_ned_from_sensor = ctx.transform(v_sensor, sensor_id, FRAME_NED);
```

## Symbolic Usage

`FrameContext` is templated and works with `SymbolicScalar`:

```cpp
FrameContext<SymbolicScalar> sym_ctx;
auto gmst = sym("gmst");
auto lon = sym("lon");
auto lat = sym("lat");
auto yaw = sym("yaw");
auto pitch = sym("pitch");
auto roll = sym("roll");

sym_ctx.set_ecef(gmst);
sym_ctx.set_ned(lon, lat);
sym_ctx.set_body_euler(yaw, pitch, roll);
```

## Backward Compatibility

Existing APIs remain unchanged:

- `CoordinateFrame<Scalar>`
- `transform_vector(...)`
- `body_from_euler(...)`
- `lla_to_ecef(...)` / `ecef_to_lla(...)`

Use `FrameContext` when you need automatic multi-hop traversal or custom frame registration.
