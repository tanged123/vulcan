# Phase 22: Formalized Mass Properties

A comprehensive, symbolic-compatible mass properties implementation for Vulcan, supporting arbitrary definitions of mass, CG, moments and products of inertia, along with aggregation capabilities for use by the Icarus mass aggregator.

## Background

### Current State

Mass properties currently exist in several places:
- **`dynamics/RigidBodyTypes.hpp`**: Basic `MassProperties<Scalar>` struct with mass, inertia tensor, CG offset
- **`examples/dynamics/rigid_body_6dof.cpp`**: Usage patterns for 6DOF dynamics
- **`dynamics/Guided5Dof.hpp`** and **`dynamics/PointMass.hpp`**: Scattered mass/weight usage

### Problem Statement

1. **No aggregation support** — Cannot combine multiple component mass properties
2. **Limited construction options** — Only `from_mass()`, `diagonal()`, `full()` factories
3. **No frame transformation** — Cannot express mass properties about different reference points
4. **No validation** — No checks for physically possible inertia tensors

### Reference Design

[AeroSandbox `MassProperties`](file:///home/tanged/sources/vulcan/reference/AeroSandbox/aerosandbox/weights/mass_properties.py) provides excellent patterns:
- **Operator overloading (`+`, `-`, `*`)** for aggregation
- **Parallel axis theorem** for inertia about arbitrary points
- **Physical validation** for checking if tensor is realizable
- **Properties/accessors** for computed values

---

## Scope

| Category | Included | Notes |
|----------|----------|-------|
| Core `MassProperties<Scalar>` | ✅ | Extended struct with full capabilities |
| Factory constructors | ✅ | Point mass, solid shapes, from CAD |
| Operator overloading | ✅ | `+` (aggregate), `*` (scale), `-` (subtract) |
| Parallel axis theorem | ✅ | Transform inertia to arbitrary point |
| Physical validation | ✅ | Check eigenvalue constraints |
| Principal axes | ✅ | Compute principal moments and axes |
| Icarus integration | ❌ | Future phase (mass aggregator service) |

> [!NOTE]
> **Design Philosophy:** Mass properties remain a **value type** passed to dynamics functions. This phase formalizes the type and adds utilities; the Icarus "mass aggregator" will be a separate component pattern that uses these Vulcan utilities.

---

## Proposed API

### Core Structure (`include/vulcan/dynamics/MassProperties.hpp`)

```cpp
namespace vulcan::dynamics {

/// Full mass properties for a rigid body component
///
/// Inertia tensor uses the STANDARD mathematical convention:
///   | Ixx  Ixy  Ixz |
///   | Ixy  Iyy  Iyz |
///   | Ixz  Iyz  Izz |
///
/// where Ixy = -∫xy dm (negative product of inertia)
///
/// This differs from some CAD tools (SolidWorks, NX) which use the opposite sign.
///
/// Reference: https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
template <typename Scalar>
struct MassProperties {
    // --- Core Data ---
    Scalar mass;           ///< Total mass [kg]
    Vec3<Scalar> cg;       ///< Center of gravity position [m]
    Mat3<Scalar> inertia;  ///< Inertia tensor about CG [kg·m²]
    
    // ==========================================================================
    // Factory Constructors
    // ==========================================================================
    
    /// Create a point mass at a specified location
    static MassProperties<Scalar> point_mass(
        const Scalar& m, 
        const Vec3<Scalar>& position);
    
    /// Create from diagonal inertia (principal axes aligned with body)
    static MassProperties<Scalar> diagonal(
        const Scalar& m,
        const Vec3<Scalar>& cg,
        const Scalar& Ixx, const Scalar& Iyy, const Scalar& Izz);
    
    /// Create from full 6-component inertia tensor
    /// Products of inertia use POSITIVE convention: Ixy_input = ∫xy dm
    /// The function applies negation internally
    static MassProperties<Scalar> from_components(
        const Scalar& m,
        const Vec3<Scalar>& cg,
        const Scalar& Ixx, const Scalar& Iyy, const Scalar& Izz,
        const Scalar& Ixy, const Scalar& Ixz, const Scalar& Iyz);
    
    /// Create solid sphere
    static MassProperties<Scalar> solid_sphere(
        const Scalar& m, 
        const Scalar& radius,
        const Vec3<Scalar>& cg = Vec3<Scalar>::Zero());
    
    /// Create solid cylinder (axis along Z)
    static MassProperties<Scalar> solid_cylinder(
        const Scalar& m,
        const Scalar& radius,
        const Scalar& length,
        const Vec3<Scalar>& cg = Vec3<Scalar>::Zero());
    
    /// Create solid rectangular prism
    static MassProperties<Scalar> solid_box(
        const Scalar& m,
        const Scalar& dx, const Scalar& dy, const Scalar& dz,
        const Vec3<Scalar>& cg = Vec3<Scalar>::Zero());
    
    // ==========================================================================
    // Parallel Axis Theorem
    // ==========================================================================
    
    /// Get inertia tensor about an arbitrary point
    ///
    /// Uses the parallel axis theorem:
    /// J = I + m * (|r|² * Identity - r ⊗ r)
    ///
    /// where r is the vector from the new point to the CG.
    Mat3<Scalar> inertia_about_point(const Vec3<Scalar>& point) const;
    
    /// Get 6 unique components of inertia about a point (Jxx, Jyy, Jzz, Jxy, Jxz, Jyz)
    void inertia_components_about_point(
        const Vec3<Scalar>& point,
        Scalar& Jxx, Scalar& Jyy, Scalar& Jzz,
        Scalar& Jxy, Scalar& Jxz, Scalar& Jyz) const;
    
    // ==========================================================================
    // Physical Validation (numeric only)
    // ==========================================================================
    
    /// Check if this represents physically realizable mass properties
    ///
    /// Conditions checked:
    /// 1. mass > 0
    /// 2. Principal moments > 0
    /// 3. Triangle inequality: I1 + I2 >= I3 (for all permutations)
    ///
    /// @return true if physically possible
    bool is_physically_valid() const;
    
    /// Check if this is effectively a point mass (zero inertia)
    bool is_point_mass() const;
    
    // ==========================================================================
    // Principal Axes (numeric only)
    // ==========================================================================
    
    /// Compute principal moments of inertia (eigenvalues)
    Vec3<Scalar> principal_moments() const;
    
    /// Compute principal axes rotation matrix (eigenvectors)
    /// Columns are the principal axes
    Mat3<Scalar> principal_axes() const;
    
    // ==========================================================================
    // Aggregation Operators
    // ==========================================================================
    
    /// Add two mass properties (aggregation)
    ///
    /// Computes combined CG and inertia using parallel axis theorem.
    MassProperties<Scalar> operator+(const MassProperties<Scalar>& other) const;
    
    /// Subtract mass properties (hole removal)
    MassProperties<Scalar> operator-(const MassProperties<Scalar>& other) const;
    
    /// Scale mass properties by a factor
    MassProperties<Scalar> operator*(const Scalar& factor) const;
    
    /// Divide mass properties by a factor
    MassProperties<Scalar> operator/(const Scalar& factor) const;
    
    /// In-place addition
    MassProperties<Scalar>& operator+=(const MassProperties<Scalar>& other);
};

/// Left-multiplication: factor * props
template <typename Scalar>
MassProperties<Scalar> operator*(const Scalar& factor, const MassProperties<Scalar>& props);

// =============================================================================
// Free Functions for Aggregation
// =============================================================================

/// Aggregate a collection of mass properties
///
/// Computes combined CG and inertia tensor about combined CG.
/// Equivalent to sum() in Python: combined = mass_props_list[0] + mass_props_list[1] + ...
///
/// @param components Vector of component mass properties
/// @return Combined mass properties
template <typename Scalar>
MassProperties<Scalar> aggregate_mass_properties(
    const std::vector<MassProperties<Scalar>>& components);

/// Transform mass properties to a new coordinate frame
///
/// @param props Original mass properties
/// @param rotation Rotation from old frame to new frame
/// @param translation Translation from old origin to new origin (in old frame)
/// @return Mass properties in new frame
template <typename Scalar>
MassProperties<Scalar> transform_mass_properties(
    const MassProperties<Scalar>& props,
    const Mat3<Scalar>& rotation,
    const Vec3<Scalar>& translation);

} // namespace vulcan::dynamics
```

---

## Migration from RigidBodyTypes.hpp

> [!IMPORTANT]
> **Backward Compatibility:** The existing `MassProperties` struct in `RigidBodyTypes.hpp` will be **replaced** by the new implementation. The existing factory methods (`from_mass()`, `diagonal()`, `full()`) will be preserved via thin wrappers.

### Mapping

| Old API | New API |
|---------|---------|
| `MassProperties::from_mass(m)` | `MassProperties::point_mass(m, Vec3::Zero())` |
| `MassProperties::diagonal(m, Ixx, Iyy, Izz)` | `MassProperties::diagonal(m, Vec3::Zero(), Ixx, Iyy, Izz)` |
| `MassProperties::full(m, Ixx, Iyy, Izz, Ixy, Ixz, Iyz)` | `MassProperties::from_components(m, Vec3::Zero(), ...)` |
| `props.cg_offset` | `props.cg` |

### Compatibility Wrappers

```cpp
// Deprecated wrappers in MassProperties.hpp for backward compatibility
static MassProperties<Scalar> from_mass(const Scalar& m) {
    return point_mass(m, Vec3<Scalar>::Zero());
}

// Original diagonal with 4 args preserved
static MassProperties<Scalar> diagonal(
    const Scalar& m, const Scalar& Ixx, const Scalar& Iyy, const Scalar& Izz) {
    return diagonal(m, Vec3<Scalar>::Zero(), Ixx, Iyy, Izz);
}
```

---

## File Structure

```
include/vulcan/dynamics/
├── Dynamics.hpp            # Aggregate header (updated)
├── MassProperties.hpp      # NEW: Full mass properties implementation
├── RigidBodyTypes.hpp      # MODIFIED: remove MassProperties, include MassProperties.hpp
├── RigidBody.hpp           # No change (uses MassProperties)
└── ...

tests/dynamics/
├── test_mass_properties.cpp  # NEW: Comprehensive mass properties tests
├── test_rigid_body.cpp       # May need minor updates for API changes

docs/user_guides/
└── mass_properties.md        # NEW: User guide
```

---

## Implementation Checklist

### Core Implementation
- [ ] Create `include/vulcan/dynamics/MassProperties.hpp`
  - [ ] Core struct with mass, cg, inertia
  - [ ] Factory: `point_mass()`, `diagonal()`, `from_components()`
  - [ ] Factory: `solid_sphere()`, `solid_cylinder()`, `solid_box()`
  - [ ] Parallel axis theorem: `inertia_about_point()`
  - [ ] Aggregation operators: `+`, `-`, `*`, `/`
  - [ ] Free functions: `aggregate_mass_properties()`, `transform_mass_properties()`
  - [ ] Validation: `is_physically_valid()`, `is_point_mass()` (numeric only)
  - [ ] Principal axes: `principal_moments()`, `principal_axes()` (numeric only)
  - [ ] Backward compat: `from_mass()`, old 4-arg `diagonal()`, `full()`

### Integration
- [ ] Update `RigidBodyTypes.hpp` to include `MassProperties.hpp`, remove old struct
- [ ] Update `Dynamics.hpp` to include `MassProperties.hpp`
- [ ] Verify existing tests pass with new implementation

### Testing
- [ ] Create `tests/dynamics/test_mass_properties.cpp`
  - [ ] Factory constructor tests
  - [ ] Aggregation operator tests with known values
  - [ ] Parallel axis theorem validation
  - [ ] Symbolic compatibility tests
- [ ] Update `tests/CMakeLists.txt`

### Documentation
- [ ] Create `docs/user_guides/mass_properties.md`

---

## Verification Plan

### Automated Tests

```bash
# Build and run all tests
./scripts/ci.sh

# Run mass properties tests specifically
cd build && ctest -R mass_properties -V
```

### Test Cases with Concrete Values

#### 1. Point Mass Factory
- **Setup:** `m = 10 kg`, `position = (1, 2, 3) m`
- **Expected:** 
  - `cg = (1, 2, 3)` 
  - `inertia = zeros(3,3)` (point mass has zero inertia about its own CG)
- **Tolerance:** Exact

#### 2. Solid Sphere Factory
- **Setup:** `m = 100 kg`, `radius = 2 m`, `cg = (0, 0, 0)`
- **Expected:** `I = (2/5) * m * r² = 160 kg·m²` on all diagonals
- **Tolerance:** `1e-12`

#### 3. Aggregation - Two Point Masses
- **Setup:** 
  - `mp1`: 1 kg at (0, 0, 0)
  - `mp2`: 1 kg at (2, 0, 0)
- **Expected combined:**
  - `mass = 2 kg`
  - `cg = (1, 0, 0)` (midpoint)
  - `Iyy = Izz = 1*1² + 1*1² = 2 kg·m²` (each mass is 1m from combined CG)
- **Tolerance:** `1e-12`

#### 4. Parallel Axis Theorem
- **Setup:** Point mass 1 kg at origin, query inertia about point (1, 0, 0)
- **Expected:** `Iyy = Izz = 1 * 1² = 1 kg·m²`, `Ixx = 0`
- **Tolerance:** `1e-12`

#### 5. Aggregation - Asymmetric Body
- **Setup:** 
  - `mp1`: 2 kg at (0, 0, 0)
  - `mp2`: 1 kg at (3, 0, 0)
- **Expected combined:**
  - `mass = 3 kg`
  - `cg = (2*0 + 1*3)/3 = (1, 0, 0)`
  - Inertia about combined CG via parallel axis: each point mass contributes
- **Tolerance:** `1e-12`

#### 6. Subtraction - Hole Removal
- **Setup:** 
  - Solid: 10 kg at (0, 0, 0)
  - Hole: 2 kg at (1, 0, 0)
  - Result: 8 kg at (-0.25, 0, 0)
- **Tolerance:** `1e-12`

#### 7. Physical Validity Check
- **Setup:** Create impossible inertia tensor violating triangle inequality
- **Expected:** `is_physically_valid() = false`

#### 8. Backward Compatibility
- **Setup:** Use old API `MassProperties::diagonal(100.0, 10.0, 20.0, 30.0)`
- **Expected:** Works identically to before

### Symbolic Compatibility Test

```cpp
TEST(MassPropertiesSymbolic, Aggregation) {
    using MX = casadi::MX;
    
    auto m1 = janus::sym("m1");
    auto m2 = janus::sym("m2");
    auto x1 = janus::sym("x1");
    auto x2 = janus::sym("x2");
    
    auto mp1 = vulcan::dynamics::MassProperties<MX>::point_mass(
        m1, Vec3<MX>{x1, MX(0), MX(0)});
    auto mp2 = vulcan::dynamics::MassProperties<MX>::point_mass(
        m2, Vec3<MX>{x2, MX(0), MX(0)});
    
    auto combined = mp1 + mp2;
    
    // Build a function and evaluate
    janus::Function f("aggregate", {m1, m2, x1, x2}, 
                      {combined.mass, combined.cg(0)});
    
    auto result = f({1.0, 1.0, 0.0, 2.0});
    EXPECT_NEAR(result[0](0,0), 2.0, 1e-12);  // mass
    EXPECT_NEAR(result[1](0,0), 1.0, 1e-12);  // cg_x = midpoint
}
```

---

## Design Decisions

1. **`cg` instead of `cg_offset`** — Clearer semantics; position in the coordinate frame, not offset from some implicit "geometric center"

2. **Operator overloading for aggregation** — Intuitive syntax: `total = wing + fuselage + tail`. This matches AeroSandbox's proven pattern.

3. **Numeric-only validation and eigenvalue functions** — `is_physically_valid()` and `principal_axes()` only compile for `double` since they require eigenvalue decomposition not available for `casadi::MX`. Use `if constexpr` or SFINAE.

4. **Products of inertia sign convention** — Store the STANDARD tensor (negative products) internally, but accept POSITIVE products in `from_components()` and negate internally. This matches the existing `full()` behavior.

5. **Inertia about CG** — Always store inertia tensor about the component's own CG. Use parallel axis theorem when aggregating or querying about other points.

6. **No state** — Remains a pure value type. Aggregation creates new instances; doesn't mutate.

---

## Future Work: Icarus Mass Aggregator

This phase provides the **Vulcan utilities**. The Icarus mass aggregator pattern (Phase 23) will:

```cpp
// Icarus MassAggregator component (future phase)
template <typename Scalar>
class MassAggregator : public Component<Scalar> {
    // Inputs: vector of mass properties from child components
    std::vector<vulcan::dynamics::MassProperties<Scalar>*> component_inputs_;
    
    // Output: aggregated mass properties
    vulcan::dynamics::MassProperties<Scalar> total_;
    
    void Step(Scalar t, Scalar dt) override {
        std::vector<vulcan::dynamics::MassProperties<Scalar>> props;
        for (auto* input : component_inputs_) {
            props.push_back(*input);
        }
        total_ = vulcan::dynamics::aggregate_mass_properties(props);
    }
};
```

This separation ensures Vulcan remains library-level without Icarus dependencies.

---

## References

- [AeroSandbox MassProperties](file:///home/tanged/sources/vulcan/reference/AeroSandbox/aerosandbox/weights/mass_properties.py)
- [Phase 20: Rigid Body Dynamics](file:///home/tanged/sources/vulcan/docs/implementation_plans/phase20_rigid_body_dynamics.md)
- [Wikipedia: Moment of Inertia Tensor](https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor)
- [Parallel Axis Theorem](https://en.wikipedia.org/wiki/Parallel_axis_theorem#Tensor_generalization)
