# Phase 10: Data I/O Implementation Plan

This module provides telemetry infrastructure for both **post-processing** (HDF5/CSV) and **real-time streaming** (serialization for Hermes middleware).

## Design Philosophy: Scalar-Only Signals

> [!IMPORTANT]
> **All signals are scalar values.** Vectors and matrices are represented as multiple named scalars with a naming convention (e.g., `position.x`, `position.y`, `position.z`).

This aligns with:
- The Icarus Protocol's `double[]` wire format with explicit offsets
- Daedalus UI per-signal widgets and plots
- Cleaner static/dynamic lifecycle management
- Simpler HDF5 column-per-signal structure

**Convenience API expands vectors:**
```cpp
// User writes:
frame.set("position", Eigen::Vector3d(1, 2, 3));

// Expands to:
frame.set("position.x", 1.0);
frame.set("position.y", 2.0);
frame.set("position.z", 3.0);
```

---

## Proposed Changes

### I/O Module (`include/vulcan/io/`)

#### [NEW] [TelemetrySchema.hpp](file:///home/tanged/sources/vulcan/include/vulcan/io/TelemetrySchema.hpp)

```cpp
namespace vulcan::io {

/// Signal data type (all scalar)
enum class SignalType { 
    Double,  // 8 bytes
    Int32,   // 4 bytes (padded to 8)
    Int64    // 8 bytes
};

/// Signal lifecycle
enum class SignalLifecycle { Static, Dynamic };

/// Signal descriptor
struct SignalDescriptor {
    std::string name;
    SignalType type;
    SignalLifecycle lifecycle;
    std::string unit;
    std::string semantic;  // Optional: "boolean", "enum", etc.
    size_t offset;         // Byte offset in frame buffer
};

/// Telemetry schema builder
class TelemetrySchema {
public:
    // Core signals (scalar only)
    TelemetrySchema& add_double(const std::string& name, 
                                 SignalLifecycle lifecycle = SignalLifecycle::Dynamic,
                                 const std::string& unit = "");
    TelemetrySchema& add_int32(const std::string& name,
                                SignalLifecycle lifecycle = SignalLifecycle::Dynamic,
                                const std::string& semantic = "");
    TelemetrySchema& add_int64(const std::string& name,
                                SignalLifecycle lifecycle = SignalLifecycle::Dynamic);
    
    // Convenience: expands to name.x, name.y, name.z
    TelemetrySchema& add_vec3(const std::string& name,
                               SignalLifecycle lifecycle = SignalLifecycle::Dynamic,
                               const std::string& unit = "");
    
    // Convenience: expands to name.x, name.y, name.z, name.w
    TelemetrySchema& add_quat(const std::string& name,
                               SignalLifecycle lifecycle = SignalLifecycle::Dynamic);
    
    // Access
    const std::vector<SignalDescriptor>& signals() const;
    std::vector<SignalDescriptor> dynamic_signals() const;
    std::vector<SignalDescriptor> static_signals() const;
    
    size_t offset(const std::string& name) const;
    size_t frame_size_bytes() const;
    size_t dynamic_frame_size_bytes() const;  // For streaming
    
    // Serialization
    std::string to_json() const;
    static TelemetrySchema from_json(const std::string& json);
    
    void validate() const;  // Throws on duplicate names
};

}  // namespace vulcan::io
```

---

#### [NEW] [Frame.hpp](file:///home/tanged/sources/vulcan/include/vulcan/io/Frame.hpp)

```cpp
namespace vulcan::io {

/// Single timestep of telemetry data (scalar values only)
class Frame {
public:
    explicit Frame(const TelemetrySchema& schema);
    
    // Time
    void set_time(double t);
    double time() const;
    
    // Scalar setters
    void set(const std::string& signal, double value);
    void set(const std::string& signal, int32_t value);
    void set(const std::string& signal, int64_t value);
    
    // Convenience: expands to signal.x, signal.y, signal.z
    void set(const std::string& signal, const Eigen::Vector3d& v);
    void set(const std::string& signal, const Eigen::Vector4d& v);  // For quaternions
    
    // Scalar getters
    double get_double(const std::string& signal) const;
    int32_t get_int32(const std::string& signal) const;
    int64_t get_int64(const std::string& signal) const;
    
    // Convenience getters
    Eigen::Vector3d get_vec3(const std::string& signal) const;
    Eigen::Vector4d get_quat(const std::string& signal) const;
    
    // Raw access (for serialization)
    const std::byte* data() const;
    std::byte* data();
    size_t size_bytes() const;
    
    void clear();
    
private:
    const TelemetrySchema* schema_;
    double time_;
    std::vector<std::byte> buffer_;  // 8-byte aligned slots
};

}  // namespace vulcan::io
```

---

#### [NEW] [FrameSerializer.hpp](file:///home/tanged/sources/vulcan/include/vulcan/io/FrameSerializer.hpp)

Binary serialization for Hermes (streaming only dynamic signals):

```cpp
namespace vulcan::io {

class FrameSerializer {
public:
    explicit FrameSerializer(const TelemetrySchema& schema);
    
    /// Serialize frame (dynamic signals only) to binary
    std::span<const std::byte> serialize(const Frame& frame);
    
    /// Serialize static signals (sent once in schema handshake)
    std::span<const std::byte> serialize_statics(const Frame& frame);
    
    /// Deserialize
    void deserialize(std::span<const std::byte> data, Frame& frame);
    
    size_t dynamic_size_bytes() const;
    size_t static_size_bytes() const;

private:
    TelemetrySchema schema_;
    std::vector<std::byte> buffer_;
};

}  // namespace vulcan::io
```

---

#### [NEW] [HDF5Writer.hpp](file:///home/tanged/sources/vulcan/include/vulcan/io/HDF5Writer.hpp)

```cpp
namespace vulcan::io {

class HDF5Writer {
public:
    explicit HDF5Writer(const std::string& filename, const TelemetrySchema& schema);
    ~HDF5Writer();
    
    void write_frame(const Frame& frame);
    size_t frame_count() const;
    void flush();
    void close();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace vulcan::io
```

**HDF5 Structure** (one dataset per signal):
```
/time                    # (N,) float64
/signals/
    position.x           # (N,) float64
    position.y           # (N,) float64
    position.z           # (N,) float64
    gnc.phase            # (N,) int32
    ...
/metadata/
    schema               # JSON string attribute
    created_at           # ISO 8601 timestamp
```

---

#### [NEW] [HDF5Reader.hpp](file:///home/tanged/sources/vulcan/include/vulcan/io/HDF5Reader.hpp)

```cpp
namespace vulcan::io {

class HDF5Reader {
public:
    explicit HDF5Reader(const std::string& filename);
    ~HDF5Reader();
    
    TelemetrySchema schema() const;
    size_t frame_count() const;
    
    std::vector<double> times() const;
    std::vector<double> read_double(const std::string& signal) const;
    std::vector<int32_t> read_int32(const std::string& signal) const;
    std::vector<int64_t> read_int64(const std::string& signal) const;
    
    // Convenience
    std::vector<Eigen::Vector3d> read_vec3(const std::string& signal) const;
    
    std::vector<std::string> signal_names() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace vulcan::io
```

---

#### [NEW] [CSVExport.hpp](file:///home/tanged/sources/vulcan/include/vulcan/io/CSVExport.hpp)

```cpp
namespace vulcan::io {

struct CSVExportOptions {
    char delimiter = ',';
    int precision = 15;
    bool include_header = true;
    std::vector<std::string> signals;  // empty = all
};

void export_to_csv(const std::string& hdf5_path,
                   const std::string& csv_path,
                   const CSVExportOptions& options = {});

}  // namespace vulcan::io
```

---

### [MODIFY] [vulcan.hpp](file:///home/tanged/sources/vulcan/include/vulcan/vulcan.hpp)

```cpp
// Data I/O
#include <vulcan/io/TelemetrySchema.hpp>
#include <vulcan/io/Frame.hpp>
#include <vulcan/io/FrameSerializer.hpp>
#include <vulcan/io/HDF5Writer.hpp>
#include <vulkan/io/HDF5Reader.hpp>
#include <vulcan/io/CSVExport.hpp>
```

---

## Verification Plan

### Automated Tests (`tests/io/`)

| Test File | Coverage |
|-----------|----------|
| `test_telemetry_schema.cpp` | Schema building, JSON round-trip, offset calculation |
| `test_frame.cpp` | Scalar/vector set/get, buffer alignment |
| `test_frame_serializer.cpp` | Binary serialize/deserialize, static/dynamic split |
| `test_hdf5_roundtrip.cpp` | Write/read all signal types |
| `test_csv_export.cpp` | Export with options |

**Run:** `./scripts/ci.sh` (builds and runs all tests)

### Manual Verification

1. Create `examples/io/telemetry_demo.cpp` that:
   - Defines a 6-DOF schema
   - Writes 1000 frames to HDF5
   - Reads back and verifies
   - Exports to CSV
   
2. Run example: `./build/examples/telemetry_demo`

---

## Design Summary

| Decision | Choice |
|----------|--------|
| Signal types | `double` (8), `int32` (4+4 pad), `int64` (8) |
| Vector/matrix support | Convenience API expands to scalars |
| Lifecycle | Static (schema-only) vs Dynamic (streaming) |
| Wire format | 8-byte aligned slots |
| HDF5 structure | One dataset per signal |
| Boolean | `int32` with `semantic: "boolean"` |
