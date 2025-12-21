# Vulcan Bootstrap Part 2: Extended Utilities

**Companion To:** `vulcan_bootstrap.md` (Phases 1-11)
**Scope:** Additional stateless physics utilities and Hermes support libraries.

---

## Design Principle

Vulcan provides **stateless computations only**. Functions take inputs and return outputs with no side effects, no time evolution, and no component lifecycle. Anything with state belongs in Icarus.

| Vulcan | Icarus |
|--------|--------|
| `compute_dynamic_pressure(rho, V)` | `AeroComponent` that uses it |
| `lla_to_ecef(lat, lon, alt)` | `VehicleState` that stores position |
| `keplerian_to_cartesian(oe)` | `OrbitPropagator` component |
| `great_circle_distance(p1, p2)` | `WaypointGuidance` component |

---

## Phase 12: Geodetic Utilities

Pure geometric computations on the WGS84 ellipsoid.

### Functions
- [ ] `great_circle_distance(lla1, lla2)` — Vincenty formula (accurate to 0.5mm)
- [ ] `initial_bearing(lla1, lla2)` — Azimuth from point 1 to point 2
- [ ] `final_bearing(lla1, lla2)` — Azimuth arriving at point 2
- [ ] `destination_point(lla, bearing, distance)` — Direct geodesic problem
- [ ] `horizon_distance(altitude)` — Line-of-sight to geometric horizon
- [ ] `ray_ellipsoid_intersection(origin, direction)` — For terrain/visibility
- [ ] `is_visible(lla_observer, lla_target)` — Line-of-sight check (no terrain)

### Notes
- All angles in radians internally, provide degree wrappers.
- Consider Haversine as a fast approximate option for short distances.

---

## Phase 13: Orbital Mechanics Primitives

Classical two-body computations. No propagation (that's an Icarus component).

### State Conversions
- [ ] `cartesian_to_keplerian(r, v, mu)` → `OrbitalElements`
- [ ] `keplerian_to_cartesian(oe, mu)` → `(r, v)`
- [ ] `mean_to_eccentric_anomaly(M, e)` — Kepler's equation solver
- [ ] `eccentric_to_true_anomaly(E, e)`
- [ ] `true_to_eccentric_anomaly(nu, e)`

### Orbital Quantities
- [ ] `orbital_period(a, mu)`
- [ ] `orbital_velocity(r, a, mu)` — Vis-viva equation
- [ ] `orbital_energy(a, mu)`
- [ ] `escape_velocity(r, mu)`
- [ ] `circular_velocity(r, mu)`

### Transfer Computations
- [ ] `hohmann_delta_v(r1, r2, mu)` — Returns (dv1, dv2)
- [ ] `bielliptic_delta_v(r1, r2, r_transfer, mu)`
- [ ] `plane_change_delta_v(v, delta_inclination)`

### Structs
```cpp
struct OrbitalElements {
    double a;    // Semi-major axis [m]
    double e;    // Eccentricity [-]
    double i;    // Inclination [rad]
    double Omega; // RAAN [rad]
    double omega; // Argument of periapsis [rad]
    double nu;   // True anomaly [rad]
};
```

---

## Phase 14: Physical Constants

Centralized, well-referenced constants.

### Universal Constants (CODATA 2018)
- [ ] `SPEED_OF_LIGHT` — 299792458 m/s (exact)
- [ ] `GRAVITATIONAL_CONSTANT` — 6.67430e-11 m³/(kg·s²)
- [ ] `BOLTZMANN_CONSTANT` — 1.380649e-23 J/K (exact)
- [ ] `STEFAN_BOLTZMANN` — 5.670374e-8 W/(m²·K⁴)

### Planetary Constants
- [ ] `EARTH_MU` — 3.986004418e14 m³/s²
- [ ] `EARTH_RADIUS_EQUATORIAL` — 6378137.0 m (WGS84)
- [ ] `EARTH_RADIUS_POLAR` — 6356752.3142 m
- [ ] `EARTH_FLATTENING` — 1/298.257223563
- [ ] `EARTH_J2` — 1.08263e-3
- [ ] `EARTH_ROTATION_RATE` — 7.2921159e-5 rad/s

### Standard Atmosphere
- [ ] `SEA_LEVEL_PRESSURE` — 101325 Pa
- [ ] `SEA_LEVEL_TEMPERATURE` — 288.15 K
- [ ] `SEA_LEVEL_DENSITY` — 1.225 kg/m³
- [ ] `GAS_CONSTANT_AIR` — 287.05287 J/(kg·K)
- [ ] `GAMMA_AIR` — 1.4

### Implementation Notes
- Use `constexpr` for compile-time evaluation.
- Group in namespaces: `vulcan::constants::earth`, `vulcan::constants::physics`.
- Add Doxygen comments with source references.

---

## Phase 15: Units & Validation

### Angle Utilities
- [ ] `deg_to_rad(deg)` / `rad_to_deg(rad)`
- [ ] `wrap_to_pi(angle)` — Wrap to [-π, π]
- [ ] `wrap_to_2pi(angle)` — Wrap to [0, 2π]
- [ ] `wrap_to_180(angle)` — Wrap to [-180, 180] degrees

### Common Conversions
- [ ] `feet_to_meters(ft)` / `meters_to_feet(m)`
- [ ] `knots_to_mps(kts)` / `mps_to_knots(mps)`
- [ ] `nmi_to_meters(nmi)` / `meters_to_nmi(m)`
- [ ] `psi_to_pascals(psi)` / `pascals_to_psi(pa)`
- [ ] `fahrenheit_to_kelvin(f)` / `kelvin_to_fahrenheit(k)`

### Validation Helpers
- [ ] `is_finite(x)` — Not NaN or Inf
- [ ] `is_in_range(x, min, max)`
- [ ] `clamp(x, min, max)`
- [ ] `assert_finite(x, name)` — Throws with context if invalid
- [ ] `assert_positive(x, name)`
- [ ] `assert_unit_quaternion(q, tolerance)`

---

## Phase 16: Geometry Primitives

Spatial computations for guidance and visibility.

### Line-of-Sight
- [ ] `los_angles(r_observer, r_target)` — Azimuth and elevation
- [ ] `los_rate(r, v, r_target, v_target)` — Angular rate of LOS
- [ ] `slant_range(r_observer, r_target)` — Distance between points

### Intersections
- [ ] `ray_sphere_intersection(origin, direction, center, radius)`
- [ ] `ray_plane_intersection(origin, direction, plane_normal, plane_point)`
- [ ] `point_in_cone(point, apex, axis, half_angle)` — Sensor FOV check

### Projections
- [ ] `project_to_plane(point, plane_normal, plane_point)`
- [ ] `ground_track_point(r_ecef)` — Project to ellipsoid surface

---

## Phase 17: Hermes Protocol Utilities

Shared between Hermes and Daedalus. Consider placing in `common/` directory.

### CRC & Hashing
- [ ] `crc32(data, length)` — For `schema_hash`
- [ ] `crc32_combine(crc1, crc2, len2)` — Incremental computation

### Binary Serialization
- [ ] `write_le<T>(buffer, value)` — Little-endian write
- [ ] `read_le<T>(buffer)` — Little-endian read
- [ ] `write_be<T>(buffer, value)` — Big-endian write
- [ ] `read_be<T>(buffer)` — Big-endian read

### Packet Structures
```cpp
struct TelemetryHeader {
    uint32_t sequence;
    uint64_t sim_time_ns;
    uint32_t schema_hash;
    uint32_t flags;
    uint32_t data_len;

    static constexpr size_t SIZE = 24;

    void serialize(uint8_t* buffer) const;
    static TelemetryHeader deserialize(const uint8_t* buffer);
};
```

### Protocol Constants
```cpp
namespace protocol {
    constexpr uint8_t MAGIC[8] = {'I','C','A','R','E','C','0','1'};
    constexpr uint16_t VERSION = 1;
    constexpr uint16_t DEFAULT_WS_PORT = 8765;
    constexpr uint16_t DEFAULT_UDP_PORT = 8766;

    // Flag bits
    constexpr uint32_t FLAG_PAUSED    = 1 << 0;
    constexpr uint32_t FLAG_RECORDING = 1 << 1;
}
```

---

## Phase 18: Hermes Threading Utilities

Lock-free primitives for physics→I/O thread communication.

### SPSC Ring Buffer
- [ ] `SPSCRingBuffer<T, N>` — Single-producer, single-consumer
- [ ] `push(item)` → `bool` — Non-blocking enqueue
- [ ] `pop()` → `std::optional<T>` — Non-blocking dequeue
- [ ] `size()` — Approximate count

### Latest-Value Buffer
- [ ] `LatestValue<T>` — Writer overwrites, reader gets latest
- [ ] `write(value)` — Always succeeds
- [ ] `read()` → `T` — Returns most recent value
- [ ] Useful for telemetry snapshots (I/O thread reads latest state)

### Implementation Notes
- Use `std::atomic` with appropriate memory orderings.
- Avoid false sharing with cache-line padding.
- Consider `folly::ProducerConsumerQueue` as reference.

---

## Phase 19: Hermes Timing Utilities

Precision timing for real-time simulation.

### Rate Limiter
```cpp
class RateLimiter {
public:
    explicit RateLimiter(double hz);
    void wait();  // Sleeps until next tick
    double drift_ns() const;  // Accumulated timing error
    void reset();
};
```

### High-Resolution Sleep
- [ ] `precise_sleep_ns(nanoseconds)` — Uses `clock_nanosleep` on Linux
- [ ] `spin_sleep_ns(nanoseconds)` — Busy-wait for sub-ms precision

### Timing Measurement
- [ ] `Stopwatch` class — Start/stop/lap timing
- [ ] `measure_jitter(samples)` — Statistical jitter analysis
- [ ] `TimingStats` — Mean, stddev, max of timing intervals

---

## Phase 20: Recording Format Utilities

Reader/writer for `.icarec` files.

### Writer
```cpp
class IcarecWriter {
public:
    IcarecWriter(const std::string& path, const std::string& schema_json);
    void write_packet(const TelemetryHeader& header, const uint8_t* data);
    void close();

private:
    std::ofstream file_;
};
```

### Reader
```cpp
class IcarecReader {
public:
    explicit IcarecReader(const std::string& path);

    std::string schema_json() const;
    size_t packet_count() const;

    bool read_next(TelemetryHeader& header, std::vector<uint8_t>& data);
    bool seek_to_time(uint64_t sim_time_ns);  // Requires index
    void rewind();

private:
    std::ifstream file_;
    std::vector<size_t> packet_offsets_;  // Index for seeking
};
```

### Index Builder
- [ ] `build_index(path)` — Creates `.icarec.idx` for fast seeking
- [ ] Index format: `[sim_time_ns, file_offset]` pairs

---

## Directory Structure Suggestion

```
vulcan/
├── include/vulcan/
│   ├── geodetic.hpp      # Phase 12
│   ├── orbital.hpp       # Phase 13
│   ├── constants.hpp     # Phase 14
│   ├── units.hpp         # Phase 15
│   └── geometry.hpp      # Phase 16
└── src/
    └── ...

common/                   # Shared by Hermes + Daedalus
├── include/common/
│   ├── protocol.hpp      # Phase 17
│   ├── threading.hpp     # Phase 18
│   ├── timing.hpp        # Phase 19
│   └── icarec.hpp        # Phase 20
└── src/
    └── ...
```

---

## Testing Strategy

### Vulcan Physics (Phases 12-16)
- **Reference comparisons:** Validate against STK, GMAT, or Vallado textbook examples.
- **Round-trip tests:** `lla_to_ecef(ecef_to_lla(x)) ≈ x`
- **Edge cases:** Poles, date line, zero eccentricity, circular orbits.

### Hermes Utilities (Phases 17-20)
- **Fuzz testing:** Malformed packets, truncated files.
- **Stress testing:** Ring buffer under high load, timing under CPU pressure.
- **Determinism:** Same recording plays back identically every time.
