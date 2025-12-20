# Vulcan Time Systems User Guide

This guide covers time infrastructure for aerospace simulations: time scales, Julian dates, leap seconds, and the unified `Epoch` class.

---

## Quick Start

```cpp
#include <vulcan/vulcan.hpp>
using namespace vulcan::time;

// Create an epoch from UTC calendar
auto epoch = NumericEpoch::from_utc(2024, 7, 15, 12, 30, 0.0);

// Access different time scales
std::cout << epoch.to_iso_string();  // "2024-07-15T12:30:00Z"
std::cout << epoch.jd_tt();          // Julian Date in TT
std::cout << epoch.gps_week();       // GPS week number

// Time arithmetic
auto later = epoch + 3600.0;         // Add 1 hour
double dt = later - epoch;           // Difference in seconds
```

---

## Time Scales

Vulcan supports six time scales:

| Scale | Name | Use Case |
|-------|------|----------|
| **TAI** | International Atomic Time | Internal storage (continuous) |
| **UTC** | Coordinated Universal Time | Human I/O, civil time |
| **TT** | Terrestrial Time | Orbit propagation |
| **TDB** | Barycentric Dynamical Time | Solar system dynamics |
| **GPS** | GPS Time | Navigation, GPS receivers |
| **UT1** | Universal Time 1 | Earth rotation (requires IERS) |

### Relationships

```
TT  = TAI + 32.184s        (exact)
GPS = TAI - 19s            (exact)
UTC = TAI - leap_seconds   (table lookup)
TDB ≈ TT + periodic terms  (~1.7ms amplitude)
```

### Conversions

```cpp
// Fixed-offset conversions (templated for symbolic)
auto tt_jd = tai_to_tt(tai_jd);
auto gps_jd = tai_to_gps(tai_jd);

// Leap-second conversions (numeric with auto-lookup)
auto tai_jd = utc_to_tai(utc_jd);

// Leap-second conversions (templated with explicit delta_at)
auto tai_jd = utc_to_tai(utc_jd, 37);  // 37 leap seconds
```

---

## Julian Dates

### Calendar ↔ JD

```cpp
// Calendar to Julian Date (Vallado algorithm)
double jd = calendar_to_jd(2024, 7, 15, 12, 30, 45.0);

// Julian Date to calendar
auto [year, month, day, hour, min, sec] = jd_to_calendar(jd);
```

### Reference Epochs

| Constant | Value | Definition |
|----------|-------|------------|
| `JD_J2000` | 2451545.0 | 2000-01-01 12:00:00 TT |
| `JD_GPS_EPOCH` | 2444244.5 | 1980-01-06 00:00:00 UTC |
| `JD_UNIX_EPOCH` | 2440587.5 | 1970-01-01 00:00:00 UTC |

### MJD and J2000 Utilities

```cpp
// Modified Julian Date
double mjd = jd_to_mjd(jd);
double jd_back = mjd_to_jd(mjd);

// Seconds since J2000.0
double sec = jd_to_j2000_seconds(jd);

// Julian centuries since J2000.0 (for precession models)
double T = jd_to_j2000_centuries(jd);
```

---

## Epoch Class

The `Epoch<Scalar>` class provides unified time representation, templated for both numeric (`double`) and symbolic (`casadi::MX`) use.

### Construction

```cpp
// From UTC calendar (numeric only)
auto epoch = NumericEpoch::from_utc(2024, 7, 15, 12, 0, 0.0);

// From JD in various scales
auto epoch = NumericEpoch::from_jd_tai(jd_tai);
auto epoch = NumericEpoch::from_jd_tt(jd_tt);
auto epoch = NumericEpoch::from_jd_gps(jd_gps);

// From TAI seconds since J2000.0 (works with symbolic)
auto epoch = SymbolicEpoch::from_tai_seconds(t_sym);

// From GPS week/seconds
auto epoch = NumericEpoch::from_gps_week(2323, 158000.0);
```

### Accessors

```cpp
// Julian Dates
epoch.jd_tai();   // JD in TAI
epoch.jd_tt();    // JD in TT
epoch.jd_gps();   // JD in GPS
epoch.jd_utc();   // JD in UTC
epoch.jd_tdb();   // JD in TDB

// Seconds
epoch.tai_seconds();  // TAI seconds since J2000.0
epoch.tt_seconds();   // TT seconds since J2000.0
epoch.centuries_tt(); // Julian centuries since J2000.0 TT

// GPS
epoch.gps_week();           // GPS week number
epoch.gps_seconds_of_week(); // Seconds into current week

// Display (numeric only)
epoch.to_iso_string();    // "2024-07-15T12:30:00Z"
epoch.to_utc_calendar();  // tuple of (y, m, d, h, min, sec)
```

### Arithmetic

```cpp
auto epoch2 = epoch1 + 3600.0;  // Add 1 hour
auto epoch3 = epoch1 - 86400.0; // Subtract 1 day
double dt = epoch2 - epoch1;    // Difference in seconds

epoch1 += 60.0;  // In-place add
epoch1 -= 60.0;  // In-place subtract
```

---

## Leap Seconds

### Lookup Functions

```cpp
// Get TAI - UTC for a given date
int delta_at = leap_seconds_at_utc(utc_jd);  // Returns 37 for 2024
int delta_at = leap_seconds_at_tai(tai_jd);

// Check for leap second instant
bool is_leap = is_leap_second(2016, 12, 31, 23, 59, 60.0);
```

### Symbolic Leap Second Lookup

For optimization over time spans crossing leap second boundaries:

```cpp
// Uses janus::Interpolator for smooth, differentiable approximation
template <typename Scalar>
Scalar leap_seconds_symbolic(const Scalar& utc_jd);

// Fully symbolic UTC ↔ TAI
template <typename Scalar>
Scalar utc_to_tai_symbolic(const Scalar& utc_jd);
```

---

## GPS Time Utilities

```cpp
// GPS week calculations
int full_week = full_gps_week(week_mod_1024, 2);  // Handle rollover

// GPS week/seconds ↔ JD
double utc_jd = gps_week_to_utc_jd(week, sow, delta_at);
auto [week, sow] = utc_jd_to_gps_week_auto(utc_jd);

// GPS-UTC offset
int offset = gps_utc_offset(utc_jd);  // Returns 18 for 2024 (37 - 19)
```

---

## Symbolic Mode & Optimization

### Creating Symbolic Epochs

```cpp
#include <janus/janus.hpp>

// Symbolic time offset
auto dt = janus::sym("dt");

// Symbolic epoch from numeric base + symbolic offset
double base_sec = base_epoch.tai_seconds();
auto sym_epoch = SymbolicEpoch::from_tai_seconds(base_sec + dt);

// All accessors return symbolic expressions
auto jd_tt = sym_epoch.jd_tt();      // CasADi MX
auto T = sym_epoch.centuries_tt();   // CasADi MX
```

### Optimization with janus::Opti

```cpp
janus::Opti opti;

// Decision variable: time offset [seconds]
auto dt = opti.variable(0.0);

// Symbolic epoch
auto epoch = SymbolicEpoch::from_tai_seconds(base_tai + dt);

// Objective using time-dependent computation
auto objective = compute_objective(epoch);
opti.minimize(objective);

// Constraints
opti.subject_to(dt >= -12.0 * 3600.0);
opti.subject_to(dt <= 24.0 * 3600.0);

// Solve
auto solution = opti.solve();
double optimal_dt = solution.value(dt);
```

---

## Constants Reference

```cpp
namespace vulcan::constants::time {
    // Epochs
    JD_J2000         // 2451545.0
    MJD_J2000        // 51544.5
    JD_GPS_EPOCH     // 2444244.5
    JD_UNIX_EPOCH    // 2440587.5
    
    // Offsets
    TT_TAI_OFFSET    // 32.184 s
    GPS_TAI_OFFSET   // -19.0 s
    TAI_GPS_OFFSET   // 19.0 s
    
    // Conversion factors
    SECONDS_PER_DAY     // 86400.0
    SECONDS_PER_WEEK    // 604800.0
    SECONDS_PER_CENTURY // 86400 * 36525
    DAYS_PER_CENTURY    // 36525.0
}
```

---

## Example: Satellite Observation Optimization

See [time_optimization.cpp](file:///home/tanged/sources/temp/vulcan/examples/time/time_optimization.cpp) for a complete example demonstrating:

- Symbolic `Epoch` in optimization
- Time-dependent visibility computation
- `janus::Opti` with time constraints
- Interactive graph visualization

---

## Graph Visualization

Export symbolic time expressions as interactive HTML graphs:

```cpp
auto t = janus::sym("t");
auto epoch = SymbolicEpoch::from_tai_seconds(t);

// Export TAI → JD TT conversion graph
janus::export_graph_html(epoch.jd_tt(), "graph_jd_tt", "JD_TT_Conversion");

// Export visibility objective (complex graph)
auto visibility = visibility_metric(epoch, sat_period);
janus::export_graph_html(visibility, "graph_visibility", "Visibility_Objective");
```

> [!TIP]
> **Interactive Graphs** - Open the exported HTML files in any browser to:
> - 🔍 Click nodes to see full expressions
> - 🔗 Hover to highlight connected operations
> - 📊 Explore the computational flow from inputs to outputs

Example graphs generated by the [time optimization example](file:///home/tanged/sources/temp/vulcan/examples/time/time_optimization.cpp):

> - [🔍 JD TT](examples/graph_jd_tt.html)
> - [🔍 Julian centuries since J2000.0](examples/graph_centuries_tt.html)
> - [🔍 Visibility objective](examples/graph_visibility.html)
> - [🔍 Solar elevation model](examples/graph_solar_elevation.html)
---

## Example: Multi-Month Mission Planning

See [mission_planning.cpp](file:///home/tanged/sources/temp/vulcan/examples/time/mission_planning.cpp) for a complete example demonstrating:

- **Symbolic leap second interpolation** via `janus::Interpolator`
- Multi-month optimization across 6-month mission windows
- Ground station visibility with seasonal effects

### Symbolic Leap Second Lookup

```cpp
auto utc_jd = janus::sym("utc_jd");

// Smooth, differentiable leap second lookup
auto delta_at = leap_seconds_symbolic(utc_jd);

// Fully symbolic UTC → TAI conversion
auto tai_jd = utc_to_tai_symbolic(utc_jd);
```

> [!IMPORTANT]
> **Why Symbolic Leap Seconds?**
>
> Standard leap second lookup uses table search (non-differentiable).
> For optimization spanning leap second boundaries, `leap_seconds_symbolic()`
> uses `janus::Interpolator` with linear interpolation for smooth gradients.

### Mission Optimization Output

```
Optimal mission window:
  Start: 2024-07-01
  End:   2024-12-28
  Solver iterations: 27

Start Month | Visibility Score | Season
------------|------------------|--------
          1 |           0.9266 | Winter
          3 |           0.7865 | Spring
          5 |           0.8257 | Spring
          7 |           0.0000 | Summer
```

### Exported Graphs

| Graph | Description |
|-------|-------------|
| `graph_leap_seconds.html` | Leap second interpolator |
| `graph_utc_to_tai.html` | UTC→TAI symbolic conversion |
| `graph_ground_station.html` | Ground station visibility |
