# Phase 2: Atmospheric Models Implementation Plan

This phase focuses on implementing robust, vetted atmospheric models for Vulcan using table-based interpolation backed by Janus's dual symbolic/numeric architecture.

## Background

The current `StandardAtmosphere.hpp` uses an analytical 2-layer model (troposphere + stratosphere) that:
- Only covers altitudes up to ~20km accurately
- Uses simplified piecewise formulas with `janus::where` for branching
- Lacks validation against vetted reference data

We have access to comprehensive US Standard Atmosphere 1976 reference data in [`atmos_temp.txt`](file:///home/tanged/sources/vulcan/reference/atmos_temp.txt) covering 0-1000 km with columns:
- **Z** (geometric altitude, km), **H** (geopotential altitude, km)
- **T** (temperature, K), **p** (pressure, Pa), **ρ** (density, kg/m³)
- **c** (speed of sound, m/s), **g** (gravitational acceleration, m/s²)

---

## User Review Required

> [!IMPORTANT]
> **Design Decision: Multiple Atmosphere Model APIs**
> 
> I propose both **function-level access** (like current API) and **class-based access** (more flexible):
> ```cpp
> // Function API (simple use case)
> auto rho = vulcan::us76::density(altitude);
> 
> // Class API (for custom atmospheres or caching)
> vulcan::TabulatedAtmosphere atm("path/to/data.tsv");
> auto rho = atm.density(altitude);
> ```

> [!WARNING]
> **Table Data Baked Into Header vs External File**
> 
> Option A: Embed US76 data as `constexpr` arrays in header (no file I/O, always available)
> Option B: Load from external file at runtime (flexible but requires file system access)
> 
> **Recommendation**: Option A for US76 standard (always available), Option B for custom atmospheres.

---

## Proposed Changes

### Component 1: Table Interpolation Wrapper

A Vulcan-specific wrapper around the Janus `Interpolator` class for atmospheric property lookup. The Janus `Interpolator` is a unified N-dimensional interpolator where 1D is simply the N=1 case.

#### [NEW] [TableInterpolator.hpp](file:///home/tanged/sources/vulcan/include/vulcan/core/TableInterpolator.hpp)

```cpp
// Thin wrapper around janus::Interpolator with:
// - Template on Scalar for symbolic compatibility
// - Unified interface for 1D through N-D interpolation
// - Input validation and bound clamping
// - Caching for repeated queries in trajectory optimization

namespace vulcan {

/// 1D table interpolation (most common for atmospheric lookups)
template<typename Scalar>
class Table1D {
    janus::Interpolator interp_;
public:
    Table1D(const janus::NumericVector& x, const janus::NumericVector& y,
            janus::InterpolationMethod method = janus::InterpolationMethod::Linear);
    
    // Single point query
    Scalar operator()(const Scalar& x) const;
    
    // Batch query
    template<typename Derived>
    janus::JanusVector<Scalar> operator()(const Eigen::MatrixBase<Derived>& x) const;
};

/// N-D table interpolation (for multi-variate lookups)
template<typename Scalar, int NDims = Eigen::Dynamic>
class TableND {
    janus::Interpolator interp_;
    int m_dims;
public:
    TableND(const std::vector<janus::NumericVector>& grid_points,
            const janus::NumericVector& values,  // Fortran order (column-major)
            janus::InterpolationMethod method = janus::InterpolationMethod::Linear);
    
    // Single N-D point query
    Scalar operator()(const janus::JanusVector<Scalar>& x) const;
    
    // Batch query (matrix of points)
    template<typename Derived>
    janus::JanusVector<Scalar> operator()(const Eigen::MatrixBase<Derived>& x) const;
    
    int dims() const { return m_dims; }
};

} // namespace vulcan
```

**Janus API reference** (from [Interpolate.hpp](file:///home/tanged/sources/vulcan/reference/janus/include/janus/math/Interpolate.hpp)):
- `janus::Interpolator(x, y, method)` — 1D convenience constructor
- `janus::Interpolator(points_vec, values, method)` — N-D constructor
- `interp(scalar)` — 1D scalar query
- `interp(vector)` — N-D point query
- `interp(matrix)` — Batch query (auto-detects shape)

---

### Component 2: US Standard Atmosphere 1976 (Table-Based)

Replace current analytical model with vetted table interpolation.

#### [MODIFY] [StandardAtmosphere.hpp](file:///home/tanged/sources/vulcan/include/vulcan/atmosphere/StandardAtmosphere.hpp)

- Rename existing functions to `legacy::temperature()` etc. (deprecated)
- Add namespace `vulcan::us76` with table-based implementation
- Embed reference data as `constexpr` arrays

```cpp
namespace vulcan::us76 {
namespace detail {
    // Embedded US76 data (0-1000km, 5km increments = 201 points)
    inline constexpr std::array<double, 201> altitude_km = {...};
    inline constexpr std::array<double, 201> temperature_K = {...};
    inline constexpr std::array<double, 201> pressure_Pa = {...};
    inline constexpr std::array<double, 201> density_kgm3 = {...};
    inline constexpr std::array<double, 201> speed_of_sound_ms = {...};
    inline constexpr std::array<double, 201> gravity_ms2 = {...};
}

template<typename Scalar> Scalar temperature(const Scalar& altitude_m);
template<typename Scalar> Scalar pressure(const Scalar& altitude_m);
template<typename Scalar> Scalar density(const Scalar& altitude_m);
template<typename Scalar> Scalar speed_of_sound(const Scalar& altitude_m);
template<typename Scalar> Scalar gravity(const Scalar& altitude_m);
}
```

---

### Component 3: Exponential Atmosphere Model

Simple exponential atmosphere for quick estimates and validation.

#### [NEW] [ExponentialAtmosphere.hpp](file:///home/tanged/sources/vulcan/include/vulcan/atmosphere/ExponentialAtmosphere.hpp)

```cpp
namespace vulcan::exponential_atmosphere {
    // ρ = ρ₀ * exp(-h / H_scale)
    // Default scale height H ≈ 8.5 km
    template<typename Scalar>
    Scalar density(const Scalar& altitude, double scale_height = 8500.0);
    
    template<typename Scalar>
    Scalar pressure(const Scalar& altitude, double scale_height = 8500.0);
}
```

---

### Component 4: Custom Tabulated Atmosphere

Class-based interface for user-provided atmospheric data.

#### [NEW] [TabulatedAtmosphere.hpp](file:///home/tanged/sources/vulcan/include/vulcan/atmosphere/TabulatedAtmosphere.hpp)

```cpp
namespace vulcan {
class TabulatedAtmosphere {
    janus::Interp1D temp_interp_, pres_interp_, dens_interp_;
    janus::Interp1D sos_interp_, grav_interp_;
    double min_alt_, max_alt_;
public:
    // Construct from vectors (altitude in meters)
    TabulatedAtmosphere(const Eigen::VectorXd& altitude,
                        const Eigen::VectorXd& temperature,
                        const Eigen::VectorXd& pressure,
                        const Eigen::VectorXd& density,
                        janus::InterpolationMethod method = janus::InterpolationMethod::Linear);
    
    template<typename Scalar> Scalar temperature(const Scalar& alt) const;
    template<typename Scalar> Scalar pressure(const Scalar& alt) const;
    template<typename Scalar> Scalar density(const Scalar& alt) const;
    // ... etc
};
}
```

---

### Component 5: Tests

#### [MODIFY] [test_standard.cpp](file:///home/tanged/sources/vulcan/tests/atmosphere/test_standard.cpp)

Update existing tests to use new `us76::` namespace and add validation against reference data.

#### [NEW] [test_exponential.cpp](file:///home/tanged/sources/vulcan/tests/atmosphere/test_exponential.cpp)

Test exponential atmosphere model.

#### [NEW] [test_tabulated.cpp](file:///home/tanged/sources/vulcan/tests/atmosphere/test_tabulated.cpp)

Test custom tabulated atmosphere class.

---

## Verification Plan

### Automated Tests

All tests run via the standard Vulcan test script:

```bash
# Build and run all tests
./scripts/ci.sh

# Or test only atmosphere tests
cd build && ctest -R atmosphere --output-on-failure
```

**Test coverage:**

| Test File | Coverage |
|-----------|----------|
| `test_standard.cpp` | US76 table vs reference data, symbolic differentiation |
| `test_exponential.cpp` | Exponential decay, boundary conditions |
| `test_tabulated.cpp` | Custom data loading, interpolation accuracy |

### Validation Against Reference

The US76 implementation will be validated against the reference table data:
- **Sea level**: T=288.15K, P=101325Pa, ρ=1.225 kg/m³
- **At 10km**: T≈223K, P≈26500Pa, ρ≈0.414 kg/m³
- **At 50km**: T≈270K, P≈80Pa, ρ≈0.001 kg/m³

### Manual Verification

Run the updated example to visually verify atmosphere profiles:

```bash
cd build && ./examples/atmosphere_profile
```

Compare output table against reference data.
