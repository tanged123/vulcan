# Data I/O

The Vulcan Data I/O module provides a comprehensive telemetry system for recording simulation data, post-processing analysis, and real-time streaming to the Hermes middleware.

## Overview

The I/O module consists of:

| Component | Purpose |
|-----------|---------|
| `TelemetrySchema` | Defines signal names, types, and lifecycle |
| `Frame` | Holds a single timestep of data |
| `HDF5Writer` | Writes frames to HDF5 files |
| `HDF5Reader` | Reads HDF5 files for analysis |
| `CSVExport` | Exports HDF5 data to CSV |
| `FrameSerializer` | Binary serialization for real-time streaming |

## Quick Start

```cpp
#include <vulcan/vulcan.hpp>

using namespace vulcan::io;

// 1. Define schema
TelemetrySchema schema;
schema.add_vec3("position", SignalLifecycle::Dynamic, "m")
      .add_vec3("velocity", SignalLifecycle::Dynamic, "m/s")
      .add_int32("phase", SignalLifecycle::Dynamic)
      .add_double("mass", SignalLifecycle::Static, "kg");

// 2. Record simulation data
HDF5Writer writer("sim.h5", schema);
for (double t = 0; t < 10.0; t += 0.01) {
    Frame frame(schema);
    frame.set_time(t);
    frame.set("position", compute_position(t));
    frame.set("velocity", compute_velocity(t));
    frame.set("phase", int32_t{1});
    frame.set("mass", 1000.0);
    writer.write_frame(frame);
}

// 3. Analyze results
HDF5Reader reader("sim.h5");
auto positions = reader.read_vec3("position");
auto times = reader.times();
```

## Telemetry Schema

### Signal Types

All signals are stored as 8-byte aligned values:

| Type | Wire Size | Description |
|------|-----------|-------------|
| `Double` | 8 bytes | 64-bit floating point |
| `Int32` | 8 bytes | 32-bit integer (padded) |
| `Int64` | 8 bytes | 64-bit integer |

### Signal Lifecycle

| Lifecycle | Description | Streaming Behavior |
|-----------|-------------|-------------------|
| `Dynamic` | Changes every frame | Sent at 60Hz |
| `Static` | Set once at startup | Sent once in handshake |

### Adding Signals

```cpp
TelemetrySchema schema;

// Scalar signals
schema.add_double("altitude", SignalLifecycle::Dynamic, "m");
schema.add_int32("phase", SignalLifecycle::Dynamic);
schema.add_int64("timestamp", SignalLifecycle::Static);

// Vector convenience (expands to .x, .y, .z)
schema.add_vec3("position", SignalLifecycle::Dynamic, "m");
// Creates: position.x, position.y, position.z

// Quaternion convenience (expands to .w, .x, .y, .z)
schema.add_quat("attitude");
// Creates: attitude.w, attitude.x, attitude.y, attitude.z

// Chained API
schema.add_vec3("pos")
      .add_vec3("vel")
      .add_quat("att")
      .add_double("mass", SignalLifecycle::Static, "kg");
```

### Schema Introspection

```cpp
std::cout << "Signal count: " << schema.signal_count() << "\n";
std::cout << "Frame size: " << schema.frame_size_bytes() << " bytes\n";

// Check if signal exists
if (schema.has_signal("position.x")) {
    const auto& sig = schema.signal("position.x");
    std::cout << "Type: " << (sig.type == SignalType::Double ? "double" : "int") << "\n";
    std::cout << "Unit: " << sig.unit << "\n";
}

// Get dynamic vs static signals
for (const auto& sig : schema.dynamic_signals()) {
    std::cout << sig.name << " (dynamic)\n";
}
```

## Frame

The `Frame` class holds one timestep of telemetry data:

```cpp
Frame frame(schema);

// Set timestamp
frame.set_time(1.234);

// Set scalar signals
frame.set("altitude", 10000.0);
frame.set("phase", int32_t{2});

// Set vector signals (convenience API)
frame.set("position", Eigen::Vector3d(1000, 2000, 3000));
frame.set("velocity", Eigen::Vector3d(100, 200, 300));

// Set quaternion
frame.set("attitude", Eigen::Quaterniond::Identity());

// Get values back
double alt = frame.get_double("altitude");
int32_t phase = frame.get_int32("phase");
Eigen::Vector3d pos = frame.get_vec3("position");
Eigen::Quaterniond att = frame.get_quat("attitude");

// Clear all values
frame.clear();
```

## HDF5 Writer

Records frames to HDF5 files with chunked, extensible datasets:

```cpp
HDF5Writer writer("telemetry.h5", schema);

// Write individual frames
for (size_t i = 0; i < 10000; ++i) {
    Frame frame(schema);
    frame.set_time(i * 0.001);
    // ... populate frame ...
    writer.write_frame(frame);
}

// Check progress
std::cout << "Frames written: " << writer.frame_count() << "\n";

// Flush to disk periodically (optional)
writer.flush();

// Close file (automatically called on destruction)
writer.close();
```

### HDF5 File Structure

```
/time                    # (N,) float64 - timestamps
/signals/
    position.x           # (N,) float64
    position.y           # (N,) float64
    position.z           # (N,) float64
    phase                # (N,) int32
    mass                 # (N,) float64
/metadata/
    schema               # JSON string attribute
    created_at           # ISO 8601 timestamp
```

## HDF5 Reader

Reads HDF5 files for post-simulation analysis:

```cpp
HDF5Reader reader("telemetry.h5");

// Basic info
std::cout << "Frames: " << reader.frame_count() << "\n";

// Read timestamps
auto times = reader.times();

// Read scalar signals
auto altitudes = reader.read_double("altitude");
auto phases = reader.read_int32("phase");

// Read vector signals (convenience)
auto positions = reader.read_vec3("position");  // Returns vector<Eigen::Vector3d>
auto attitudes = reader.read_quat("attitude");  // Returns vector<Eigen::Quaterniond>

// Read a slice (for large files)
auto slice = reader.read_double("altitude", 1000, 100);  // 100 samples starting at index 1000

// List all signals
for (const auto& name : reader.signal_names()) {
    std::cout << name << "\n";
}

// Access embedded schema
auto schema = reader.schema();
```

## CSV Export

Export HDF5 data to CSV for external tools:

```cpp
// Simple export
export_to_csv("telemetry.h5", "output.csv");

// With options
CSVExportOptions options;
options.delimiter = ';';           // Custom delimiter
options.precision = 6;             // Decimal places
options.include_header = true;     // First row has column names
options.signals = {"position.x", "position.y", "position.z"};  // Subset

export_to_csv("telemetry.h5", "positions.csv", options);

// Export to stream
HDF5Reader reader("telemetry.h5");
std::stringstream ss;
export_to_csv(reader, ss, options);
```

## Binary Serialization (Hermes Integration)

The `FrameSerializer` provides efficient binary serialization for real-time streaming to the Hermes middleware:

```cpp
FrameSerializer serializer(schema);

// Check sizes
std::cout << "Dynamic frame: " << serializer.dynamic_size_bytes() << " bytes\n";
std::cout << "Static data: " << serializer.static_size_bytes() << " bytes\n";

// Serialize for streaming
Frame frame(schema);
frame.set_time(1.0);
// ... populate frame ...

// Get binary data (points to internal buffer)
std::span<const std::byte> dynamic_bytes = serializer.serialize(frame);
std::span<const std::byte> static_bytes = serializer.serialize_statics(frame);

// Send to Hermes lock-free buffer
hermes_buffer.write(dynamic_bytes.data(), dynamic_bytes.size());

// Deserialize (on receiver side)
Frame received(schema);
serializer.deserialize_statics(static_bytes, received);  // Once at handshake
serializer.deserialize(dynamic_bytes, received);          // Every frame
```

### Wire Format

All values are little-endian, 8-byte aligned:

```
Dynamic Frame:
+------------------+------------------+------------------+
| time (8 bytes)   | signal_0 (8B)    | signal_1 (8B)... |
+------------------+------------------+------------------+

Static Frame:
+------------------+------------------+
| static_0 (8B)    | static_1 (8B)... |
+------------------+------------------+
```

## JSON Schema (Protocol Negotiation)

The schema can be serialized to JSON for protocol negotiation:

```cpp
std::string json = schema.to_json();
// Send to Hermes for handshake

// Parse JSON back
auto schema2 = TelemetrySchema::from_json(json);
```

Example JSON output:
```json
{
  "signals": [
    {"name": "position.x", "type": "double", "lifecycle": "dynamic", "unit": "m"},
    {"name": "position.y", "type": "double", "lifecycle": "dynamic", "unit": "m"},
    {"name": "position.z", "type": "double", "lifecycle": "dynamic", "unit": "m"},
    {"name": "phase", "type": "int32", "lifecycle": "dynamic", "semantic": "enum"},
    {"name": "mass", "type": "double", "lifecycle": "static", "unit": "kg"}
  ]
}
```

## Example

See the complete demo: [telemetry_demo.cpp](file:///home/tanged/sources/vulcan/examples/io/telemetry_demo.cpp)

```bash
# Build and run
./scripts/build.sh
./build/examples/telemetry_demo
```
