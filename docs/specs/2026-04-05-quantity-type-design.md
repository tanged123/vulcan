# Quantity Type Design Spec

**Date:** 2026-04-05
**Status:** Draft
**Scope:** Vulcan core type + downstream Icarus migration

## Summary

Introduce `Quantity<unit, Scalar>` — a type that fuses mp-units compile-time dimensional analysis with Janus's dual-mode numeric/symbolic dispatch. This gives the entire simulation stack (Vulcan, Icarus, Hermes, Daedalus) hard-typed physical quantities that prevent unit mismatches at compile time, with zero runtime overhead.

## Motivation

Today, every function in the stack communicates units through naming conventions and documentation:

```cpp
// What unit is altitude? Metres? Feet? Who knows without reading docs.
template <typename Scalar>
Scalar temperature(const Scalar& altitude);
```

Vulcan provides ~40 hand-rolled conversion functions (`deg_to_rad`, `ft_to_m`, etc.) that developers must remember to call. The type system does not enforce correctness — a temperature in Celsius compiles just fine where Kelvin is expected.

## Design

### Approach: Hybrid wrapper (mp-units internally, Vulcan-owned API)

mp-units provides the compile-time dimensional type algebra (what unit does `m / s` produce?). Vulcan wraps it in a class that controls the developer-facing API, guarantees CasADi-safe arithmetic, and insulates downstream code from mp-units internals.

### Core Type

**File:** `vulcan/quantity/Quantity.hpp`

```cpp
namespace vulcan {

template <auto Unit, JanusScalar Rep = double>
class Quantity {
    mp_units::quantity<Unit, Rep> q_;

public:
    using rep = Rep;
    static constexpr auto unit = Unit;

    // --- Construction ---
    constexpr Quantity() : q_{} {}
    constexpr explicit Quantity(Rep v) : q_(v * mp_units::reference<Unit>{}) {}
    constexpr explicit Quantity(mp_units::quantity<Unit, Rep> q) : q_(q) {}

    // --- Unwrap ---
    // Returns raw Scalar in the declared unit
    constexpr Rep value() const;

    // --- Access internal mp_units::quantity (for math integration) ---
    constexpr auto raw() const { return q_; }

    // --- Unit conversion ---
    // alt.in<foot>() -> Quantity<foot, Rep>
    template <auto ToUnit>
    constexpr Quantity<ToUnit, Rep> in() const;

    // --- String representation ---
    // Quantity<m>(10000.0).to_string() -> "10000 m"
    std::string to_string() const;

    // --- Same-unit arithmetic ---
    friend constexpr Quantity operator+(Quantity a, Quantity b);
    friend constexpr Quantity operator-(Quantity a, Quantity b);
    constexpr Quantity operator-() const;
    constexpr Quantity& operator+=(Quantity other);
    constexpr Quantity& operator-=(Quantity other);

    // --- Cross-unit multiplication/division ---
    // Result unit computed by mp-units at compile time
    template <auto U2, JanusScalar R2>
    friend constexpr auto operator*(Quantity a, Quantity<U2, R2> b);
    template <auto U2, JanusScalar R2>
    friend constexpr auto operator/(Quantity a, Quantity<U2, R2> b);

    // --- Scalar multiplication (dimensionless scaling) ---
    friend constexpr Quantity operator*(Rep s, Quantity a);
    friend constexpr Quantity operator*(Quantity a, Rep s);
    friend constexpr Quantity operator/(Quantity a, Rep s);

    // --- Comparisons (return Rep-native types for janus::where compat) ---
    friend constexpr auto operator<(Quantity a, Quantity b);
    friend constexpr auto operator>(Quantity a, Quantity b);
    friend constexpr auto operator<=(Quantity a, Quantity b);
    friend constexpr auto operator>=(Quantity a, Quantity b);
    friend constexpr auto operator==(Quantity a, Quantity b);
    friend constexpr auto operator!=(Quantity a, Quantity b);

    // --- Implicit conversion for dimensionless quantities ---
    // Quantity<dimensionless, Scalar> -> Scalar (bridge to raw Janus math)
    constexpr operator Rep() const requires(Unit == mp_units::one);
};

} // namespace vulcan
```

**Key design decisions:**

- **One type for everything.** No point/difference distinction. One type means one concept to learn, one set of rules. Absolute temperatures, displacements, forces, and altitudes are all `Quantity<unit, Scalar>`. Affine correctness (preventing `epoch + epoch`) is handled by domain classes like `Epoch`, not the quantity type system.
- **Explicit constructor from raw `Rep`.** `Quantity<m>(10.0)` works, `Quantity<m> x = 10.0` does not. Prevents accidental untyped construction.
- **`.value()` returns in the declared unit.** `Quantity<ft>(1000.0).value()` returns `1000.0`, not `304.8`. No surprises. Convert first if you need SI: `.in<m>().value()`.
- **Comparisons return `Rep`-native types.** When `Rep = casadi::MX`, `operator<` returns `MX` (a symbolic predicate), flowing directly into `janus::where()`.
- **Implicit conversion to `Rep` for dimensionless quantities only.** `Quantity<dimensionless, Scalar>` (Mach number, drag coefficient, trig results) converts to raw `Scalar` automatically, bridging cleanly into raw Janus math like `janus::exp()`.
- **`.to_string()` for debugging and telemetry.** Delegates to mp-units formatting. Useful for spdlog output, Daedalus display, Hermes metadata, and test failure messages. Only available when `Rep = double` (symbolic MX has no meaningful string form for the value, though the unit string is still available via a static method).

### Unit Vocabulary

**File:** `vulcan/core/Units.hpp` (replaces current conversion-function file)

Curated aliases for units aerospace developers actually use. Developers write `Quantity<m>`, not `Quantity<mp_units::si::metre>`.

```cpp
namespace vulcan::units {

// --- Base SI ---
inline constexpr auto m   = mp_units::si::metre;
inline constexpr auto kg  = mp_units::si::kilogram;
inline constexpr auto s   = mp_units::si::second;
inline constexpr auto K   = mp_units::si::kelvin;
inline constexpr auto rad = mp_units::si::radian;

// --- Derived SI ---
inline constexpr auto N   = mp_units::si::newton;
inline constexpr auto Pa  = mp_units::si::pascal;
inline constexpr auto J   = mp_units::si::joule;
inline constexpr auto W   = mp_units::si::watt;
inline constexpr auto Hz  = mp_units::si::hertz;

// --- Aerospace imperial/customary ---
inline constexpr auto ft  = mp_units::usc::foot;
inline constexpr auto nm  = mp_units::nautical_mile;
inline constexpr auto kts = mp_units::knot;
inline constexpr auto lbm = mp_units::usc::pound;
inline constexpr auto lbf = mp_units::usc::pound_force;
inline constexpr auto deg = mp_units::non_si::degree;
inline constexpr auto slug = /* custom: 14.5939 kg */;
inline constexpr auto psi  = /* custom: lbf / in^2 */;

// --- Compound ---
inline constexpr auto mps   = m / s;       // metres per second
inline constexpr auto fps   = ft / s;      // feet per second
inline constexpr auto rad_s = rad / s;     // radians per second

// --- Dimensionless ---
inline constexpr auto dimensionless = mp_units::one;

// --- Long-form aliases for readability ---
inline constexpr auto metre            = m;
inline constexpr auto kilogram         = kg;
inline constexpr auto second           = s;
inline constexpr auto kelvin           = K;
inline constexpr auto radian           = rad;
inline constexpr auto newton           = N;
inline constexpr auto foot             = ft;
inline constexpr auto nautical_mile    = nm;
inline constexpr auto knot             = kts;
inline constexpr auto degrees          = deg;
inline constexpr auto metres_per_second = mps;

} // namespace vulcan::units
```

Custom units not in mp-units (slug, psi, slug-ft^2, etc.) are defined using mp-units' custom unit machinery.

### Janus Math Integration

**File:** `vulcan/core/QuantityMath.hpp`

Three tiers:

**Tier 1 — Operators (free, no Vulcan code):**
`+`, `-`, `*`, `/`, `<`, `>`, `<=`, `>=`, `==`, `!=` are defined on `Quantity` and delegate to `Rep`. CasADi traces through transparently. No wrapping needed.

**Tier 2 — Dimension-preserving functions (mechanical thin wrappers):**

```cpp
namespace vulcan {

template <auto U, JanusScalar Rep>
Quantity<U, Rep> abs(Quantity<U, Rep> x) {
    return Quantity<U, Rep>{janus::abs(x.value())};
}
// Same pattern: min, max, clamp, where, floor, ceil, round, sign

template <typename Cond, auto U, JanusScalar Rep>
Quantity<U, Rep> where(const Cond& cond,
                       Quantity<U, Rep> if_true,
                       Quantity<U, Rep> if_false) {
    return Quantity<U, Rep>{janus::where(cond, if_true.value(), if_false.value())};
}

} // namespace vulcan
```

**Tier 3 — Dimension-changing functions (~8 overloads encoding real physics):**

```cpp
namespace vulcan {

// Trig: radians -> dimensionless
template <JanusScalar Rep>
Quantity<dimensionless, Rep> sin(Quantity<rad, Rep> a) {
    return Quantity<dimensionless, Rep>{janus::sin(a.value())};
}
template <JanusScalar Rep>
Quantity<dimensionless, Rep> cos(Quantity<rad, Rep> a) {
    return Quantity<dimensionless, Rep>{janus::cos(a.value())};
}
template <JanusScalar Rep>
Quantity<dimensionless, Rep> tan(Quantity<rad, Rep> a) {
    return Quantity<dimensionless, Rep>{janus::tan(a.value())};
}

// Inverse trig: dimensionless -> radians
template <JanusScalar Rep>
Quantity<rad, Rep> asin(Quantity<dimensionless, Rep> x) {
    return Quantity<rad, Rep>{janus::asin(x.value())};
}
template <JanusScalar Rep>
Quantity<rad, Rep> acos(Quantity<dimensionless, Rep> x) {
    return Quantity<rad, Rep>{janus::acos(x.value())};
}
template <JanusScalar Rep>
Quantity<rad, Rep> atan2(Quantity<dimensionless, Rep> y,
                         Quantity<dimensionless, Rep> x) {
    return Quantity<rad, Rep>{janus::atan2(y.value(), x.value())};
}

// atan2 for same-unit pairs (metres/metres -> radians)
template <auto U, JanusScalar Rep>
Quantity<rad, Rep> atan2(Quantity<U, Rep> y, Quantity<U, Rep> x) {
    return Quantity<rad, Rep>{janus::atan2(y.value(), x.value())};
}

// sqrt: dimensional halving (m^2 -> m)
template <auto U, JanusScalar Rep>
auto sqrt(Quantity<U, Rep> x) {
    auto result_q = mp_units::sqrt(x.raw());
    using result_unit = decltype(result_q)::unit;
    return Quantity<result_unit, Rep>{janus::sqrt(x.value())};
}

// Angle wrapping
template <JanusScalar Rep>
Quantity<rad, Rep> wrap_to_pi(Quantity<rad, Rep> a);
template <JanusScalar Rep>
Quantity<rad, Rep> wrap_to_2pi(Quantity<rad, Rep> a);

} // namespace vulcan
```

**Everything else** (`exp`, `log`, `sinh`, `pow`, etc.) operates on dimensionless values. Since `Quantity<dimensionless, Scalar>` implicitly converts to `Scalar`, these work naturally: `janus::exp(mach_number)` compiles without `.value()`.

### Eigen Integration

**File:** `vulcan/core/QuantityEigen.hpp`

Generic `NumTraits` specialization — one definition covers all unit types.

```cpp
namespace Eigen {

template <auto Unit, typename Rep>
struct NumTraits<vulcan::Quantity<Unit, Rep>> : NumTraits<Rep> {
    using Real       = vulcan::Quantity<Unit, Rep>;
    using NonInteger = vulcan::Quantity<Unit, Rep>;
    using Nested     = vulcan::Quantity<Unit, Rep>;
    using Literal    = vulcan::Quantity<Unit, Rep>;

    enum {
        IsComplex             = 0,
        IsInteger             = 0,
        IsSigned              = NumTraits<Rep>::IsSigned,
        RequireInitialization = 1,
        ReadCost              = NumTraits<Rep>::ReadCost,
        AddCost               = NumTraits<Rep>::AddCost,
        MulCost               = NumTraits<Rep>::MulCost
    };

    static inline Real epsilon()         { return Real{NumTraits<Rep>::epsilon()}; }
    static inline Real dummy_precision() { return Real{NumTraits<Rep>::dummy_precision()}; }
    static inline Real highest()         { return Real{NumTraits<Rep>::highest()}; }
    static inline Real lowest()          { return Real{NumTraits<Rep>::lowest()}; }
};

} // namespace Eigen
```

**What this enables:**

```cpp
// Homogeneous physics vectors
Vec3<Quantity<m, Scalar>>   position;
Vec3<Quantity<mps, Scalar>> velocity;
Vec3<Quantity<N, Scalar>>   force;

// Vector arithmetic preserves units
auto displacement = pos_b - pos_a;          // Vec3<Quantity<m>>
auto impulse = force * Quantity<s>(dt);     // Vec3<Quantity<N*s>>
auto work = force.dot(displacement);        // Quantity<N*m> = Quantity<J>
auto torque = arm.cross(force);             // Quantity<m> x Quantity<N> = Quantity<N*m>

// DCM (dimensionless) rotating position preserves units
Mat3<Quantity<dimensionless, Scalar>> R = dcm_from_euler(...);
Vec3<Quantity<m, Scalar>> pos_body = R * pos_ecef;  // dimensionless * m -> m
```

**Requires proof-of-concept verification:**
- `Eigen::norm()` chaining through Quantity's operators (may need custom overload)
- `Eigen::dot()` and `cross()` with cross-unit results
- CasADi MX as Rep flowing through Eigen operations with Quantity wrappers

### Vulcan Module Migration

All Vulcan public APIs adopt Quantity. No mixing of raw `Scalar` and `Quantity` in public interfaces.

**Atmosphere:**

```cpp
// Before
template <typename Scalar>
Scalar temperature(const Scalar& altitude);

// After
template <typename Scalar>
Quantity<K, Scalar> temperature(Quantity<m, Scalar> altitude);
```

**Coordinates:**

```cpp
// Before
template <typename Scalar>
struct LLA {
    Scalar lat, lon, alt;  // radians? degrees? metres? feet?
};

// After
template <typename Scalar>
struct LLA {
    Quantity<rad, Scalar> lat;
    Quantity<rad, Scalar> lon;
    Quantity<m, Scalar>   alt;
};
```

**Rotations:**

```cpp
// Before
template <typename Scalar>
Mat3<Scalar> dcm_from_euler(const Scalar& e1, const Scalar& e2,
                            const Scalar& e3, EulerSequence seq);

// After
template <typename Scalar>
Mat3<Quantity<dimensionless, Scalar>> dcm_from_euler(
    Quantity<rad, Scalar> e1, Quantity<rad, Scalar> e2,
    Quantity<rad, Scalar> e3, EulerSequence seq);
```

**Time:**

```cpp
// Before
template <typename Scalar>
class Epoch {
    Scalar tai_seconds_;
    Epoch operator+(Scalar dt) const;
    Scalar operator-(const Epoch& other) const;
};

// After
template <typename Scalar>
class Epoch {
    Quantity<s, Scalar> tai_seconds_;

    Epoch operator+(Quantity<s, Scalar> dt) const;           // epoch + duration = epoch
    Quantity<s, Scalar> operator-(const Epoch& other) const; // epoch - epoch = duration
    // Epoch + Epoch = not defined (Epoch is its own class, not a Quantity)
};
```

Note: `Epoch` remains its own class because it carries additional state (delta_at, time scale conversions) and provides domain-specific methods (`.jd_tai()`, `.to_iso_string()`). Its arithmetic correctness (epoch + epoch doesn't compile) comes from being a distinct class, not from the quantity type system.

**Dynamics / Propulsion:**

```cpp
template <typename Scalar>
Quantity<N, Scalar> thrust_from_mdot(Quantity<kg/s, Scalar> mdot,
                                     Quantity<mps, Scalar> Ve);
```

**Constants become typed:**

```cpp
namespace vulcan::constants::earth {
    inline constexpr Quantity<m>             R_eq{6378137.0};
    inline constexpr Quantity<rad/s>         omega{7.2921159e-5};
    inline constexpr Quantity<m*m*m/(s*s)>   mu{3.986004418e14};
}
namespace vulcan::constants::physics {
    inline constexpr Quantity<m/(s*s)> g0{9.80665};
}
namespace vulcan::constants::atmosphere {
    inline constexpr Quantity<K>   T0{288.15};
    inline constexpr Quantity<Pa>  P0{101325.0};
    inline constexpr Quantity<K/m> L{0.0065};
}
```

**Deprecation:** The current `vulcan::units` conversion functions (`deg_to_rad`, `ft_to_m`, etc.) are deprecated. Conversion is now `quantity.in<target_unit>()`. Removed once Icarus migration is complete.

**Internal migration pattern** for each function body:

1. Parameters arrive as `Quantity<unit, Scalar>`
2. Unwrap to raw `Scalar` with `.value()` at the top
3. Internal math unchanged (raw Janus operations)
4. Rewrap return values as `Quantity<unit, Scalar>`

### Icarus Downstream Migration

**Component signal registration:**

```cpp
// Before
register_input("total_force.x", ..., "N", ...);
register_state_vec3("position", ..., "m", ...);

// After — unit metadata derived from type
register_input<Quantity<N, Scalar>>("total_force.x", ...);
register_state_vec3<Quantity<m, Scalar>>("position", ...);
```

**Configuration boundary (the most common change — 12 of 23 conversion sites):**

```cpp
// Before
lla.lat = lla_vec(0) * vulcan::constants::angle::deg2rad;

// After
lla.lat = Quantity<deg>(lla_vec(0)).in<rad>();
```

**Integrator boundary (one pack/unpack seam per component):**

```cpp
// Pack typed state -> raw integrator vector
VecX<Scalar> pack_state() const {
    VecX<Scalar> x(13);
    x.segment(0, 3)  = position.unaryExpr([](auto q) { return q.value(); });
    x.segment(3, 3)  = velocity.unaryExpr([](auto q) { return q.value(); });
    x.segment(6, 4)  = quaternion.coeffs().unaryExpr([](auto q) { return q.value(); });
    x.segment(10, 3) = omega.unaryExpr([](auto q) { return q.value(); });
    return x;
}

// Unpack raw integrator vector -> typed state
void unpack_state(const VecX<Scalar>& x) {
    position = x.segment(0, 3).unaryExpr([](auto v) { return Quantity<m>(v); });
    velocity = x.segment(3, 3).unaryExpr([](auto v) { return Quantity<mps>(v); });
    // ...
}
```

**Inter-component structs:**

```cpp
template <typename Scalar>
struct AeroForces {
    Vec3<Quantity<N, Scalar>>     force;
    Vec3<Quantity<N * m, Scalar>> moment;
};
```

**Migration scope:**

| Area | Files | Change |
|------|-------|--------|
| Component signals/state | ~7 | Signature types |
| Config parsing (Stage) | ~4 | `deg2rad` -> `Quantity<deg>().in<rad>()` |
| Integrator pack/unpack | ~3 | One boundary per component |
| Inter-component structs | ~5 | Fields become typed |
| Tests | ~8 | Construct Quantities instead of raw doubles |

**Unchanged:**
- Integrator internals (stays `VecX<Scalar>`)
- YAML config file format (still human-friendly numbers)
- Signal dictionary / Hermes WebSocket protocol (can auto-derive unit strings from Quantity types)

## Future Architecture

The `Quantity` type is designed to support two future extensions without breaking changes. These are NOT in scope for this implementation but the type architecture must not preclude them.

### Reference Frame Typing (Future Phase)

After unit errors, frame errors are the most common source of aerospace bugs. A future extension adds a reference frame as an additional template parameter on vectors:

```cpp
// Future — NOT in this spec
template <typename T, typename Frame = void>
using FrameVec3 = /* frame-tagged Vec3 */;

FrameVec3<Quantity<m, Scalar>, ECEF> position_ecef;
FrameVec3<Quantity<m, Scalar>, Body> position_body;
auto bad = position_ecef + position_body;          // COMPILE ERROR: frame mismatch
auto good = R_ecef_body * position_ecef;           // transforms frame, preserves units
```

**What this spec must preserve:** `Quantity` itself has no frame awareness. Frame typing lives on the vector/container level, not the scalar level. The current design (scalar-level Quantity, Eigen vectors of Quantities) is fully compatible with a future frame-tagged vector wrapper.

### Uncertainty Propagation (Future Phase)

Monte Carlo and covariance analysis need uncertainty tracking. A future `Uncertain<double>` type can slot into the `Rep` parameter:

```cpp
// Future — NOT in this spec
using UncertainScalar = Uncertain<double>;  // value + variance

Quantity<m, UncertainScalar> alt{10000.0, /*sigma=*/5.0};
auto T = temperature(alt);  // uncertainty propagates through Rep arithmetic
// T.value().mean() = 223.15, T.value().sigma() = 0.0325
```

**What this spec must preserve:** `Quantity` is parameterized on `Rep`, and all arithmetic delegates to `Rep`. An `Uncertain<double>` that provides the same arithmetic operators as `double` will work as a drop-in `Rep` without modifying Quantity. The `JanusScalar` concept constraint on `Rep` may need relaxing in the future (to `JanusScalar || UncertainScalar`), but the Quantity type itself requires no changes.

## Implementation Scope

**Phase 1 (this spec):** Quantity type, unit vocabulary, Janus math integration, Eigen integration, Vulcan module migration, proof-of-concept. All within the Vulcan repo.

**Phase 2 (separate spec/plan):** Icarus downstream migration. Depends on Phase 1 being complete and verified. The Icarus section above defines the migration pattern but is not in-scope for Phase 1 implementation.

## Proof-of-Concept (Before Full Implementation)

Before committing to the full migration, verify in an isolated test:

1. **CasADi arithmetic:** `Quantity<m, MX> + Quantity<m, MX>` traces correctly
2. **CasADi through operators:** `Quantity<m, MX> * Quantity<m, MX>` produces `Quantity<m*m, MX>` with valid graph
3. **Eigen NumTraits:** `Vec3<Quantity<m, double>>` compiles, addition/subtraction works
4. **Eigen dot/cross:** `Vec3<Quantity<m>> . Vec3<Quantity<N>>` produces `Quantity<N*m>`
5. **Dimensionless bridge:** `Quantity<dimensionless, MX>` implicitly converts to `MX`
6. **Unit conversion:** `Quantity<ft>(1000.0).in<m>().value()` returns `304.8`
7. **Compile errors:** `Quantity<m> + Quantity<K>` fails to compile
8. **Symbolic trig:** `vulcan::sin(Quantity<rad, MX>(...))` traces correctly
9. **String formatting:** `Quantity<m>(10000.0).to_string()` returns `"10000 m"`

## Dependencies

- **mp-units** (v2.x, C++20) — added to Vulcan's `flake.nix` and `CMakeLists.txt`
- **Janus** — unchanged
- **Eigen** — unchanged (NumTraits specialization in Vulcan)

## Runtime Overhead

Zero. `Quantity` contains exactly one field (the `Rep`). All unit information is compile-time template parameters, erased completely by the compiler. At any optimization level >= `-O1`, the generated machine code is identical to hand-written raw `Scalar` operations.

## File Structure

```
vulcan/include/vulcan/
  quantity/
    Quantity.hpp          # Core Quantity<unit, Scalar> type
    QuantityMath.hpp      # Janus math integration (~8 dimension-changing + ~8 preserving)
    QuantityEigen.hpp     # Eigen NumTraits specialization
    QuantityFormat.hpp    # .to_string() implementation, mp-units formatting bridge
    Units.hpp             # Unit vocabulary (replaces current conversion functions)
  core/
    Constants.hpp         # Updated with typed Quantity constants
    VulcanTypes.hpp       # Updated type aliases incorporating Quantity
  atmosphere/             # Migrated to Quantity signatures
  coordinates/            # Migrated to Quantity signatures
  rotations/              # Migrated to Quantity signatures
  time/                   # Migrated to Quantity signatures
  dynamics/               # Migrated to Quantity signatures
  propulsion/             # Migrated to Quantity signatures
```
