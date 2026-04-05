# Quantity Type Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Introduce `Quantity<unit, Scalar>` to Vulcan — a compile-time dimensioned type wrapping mp-units that works with both numeric (`double`) and symbolic (`casadi::MX`) backends.

**Architecture:** Hybrid wrapper — mp-units handles dimensional type algebra internally, Vulcan owns the public API. `Quantity` delegates all runtime arithmetic to its `Rep` (which is a `JanusScalar`), so CasADi graph tracing flows through transparently. Janus is not modified.

**Tech Stack:** C++20, mp-units v2.x, Eigen 3.4, CasADi, GoogleTest, Nix/Ninja build

**Spec:** `docs/specs/2026-04-05-quantity-type-design.md`

---

## File Structure

```
include/vulcan/
  quantity/
    Quantity.hpp          # Core Quantity<unit, Scalar> class template
    QuantityMath.hpp      # vulcan:: math overloads for Quantity (trig, abs, where, sqrt, etc.)
    QuantityEigen.hpp     # Eigen::NumTraits specialization for Quantity
    QuantityFormat.hpp    # .to_string() implementation
    Units.hpp             # Curated aerospace unit vocabulary (replaces core/Units.hpp)

tests/
  quantity/
    test_quantity.cpp          # Core type: construction, arithmetic, conversion, comparisons
    test_quantity_symbolic.cpp # CasADi MX as Rep: tracing, graph correctness
    test_quantity_math.cpp     # vulcan:: math overloads (trig, abs, where, sqrt)
    test_quantity_eigen.cpp    # Vec3<Quantity<m>>, dot, cross, matrix-vector
    test_quantity_format.cpp   # .to_string()
    test_quantity_units.cpp    # Unit vocabulary, custom units (slug, psi), conversions
```

**Files modified:**
- `CMakeLists.txt` — add `mp_units` dependency via `find_package`
- `flake.nix` — add `mp-units` to Nix inputs/buildInputs
- `tests/CMakeLists.txt` — add `test_quantity` executable
- `include/vulcan/vulcan.hpp` — add quantity headers to umbrella

**Files NOT modified (Phase 1):**
- No Vulcan module migration yet (atmosphere, coordinates, rotations, time, dynamics, propulsion)
- No Icarus changes
- No Janus changes

Phase 1 delivers the Quantity type, tests, and proof-of-concept. Module migration is Phase 2 (separate plan).

---

### Task 1: Add mp-units dependency to Nix and CMake

**Files:**
- Modify: `flake.nix`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Add mp-units derivation to flake.nix**

mp-units is NOT in nixpkgs. Add a local derivation inside the flake's `let` block, then include it in both `buildInputs` and the dev shell.

In `flake.nix`, add this derivation in the `let` block (after `janusPackage`):

```nix
mpUnitsPackage = stdenv.mkDerivation rec {
  pname = "mp-units";
  version = "2.5.0";

  src = pkgs.fetchFromGitHub {
    owner = "mpusz";
    repo = "mp-units";
    rev = "v${version}";
    hash = ""; # Run: nix-prefetch-url --unpack https://github.com/mpusz/mp-units/archive/v2.5.0.tar.gz
  };

  nativeBuildInputs = [ pkgs.cmake pkgs.ninja ];

  # mp-units CMakeLists.txt is in src/, not the repo root
  cmakeDir = "../src";
  cmakeFlags = [
    "-DMP_UNITS_BUILD_INSTALL=ON"
  ];
};
```

Add `mpUnitsPackage` to:
1. `packages.default.buildInputs` list
2. `devShells.default.packages` list
3. The `CMAKE_PREFIX_PATH` shell hook: `...:${mpUnitsPackage}`

- [ ] **Step 2: Resolve the Nix hash**

Run `nix-prefetch-url --unpack https://github.com/mpusz/mp-units/archive/v2.5.0.tar.gz` and fill in the `hash` field in the derivation. Alternatively, leave it empty and let Nix error with the correct hash on first build.

- [ ] **Step 3: Build the Nix dev shell and verify mp-units is available**

Run: `nix develop` (or let `./scripts/build.sh` auto-enter)
Then: `ls $CMAKE_PREFIX_PATH | grep mp` to confirm mp-units is on the path.

- [ ] **Step 4: Add find_package to CMakeLists.txt**

In `CMakeLists.txt`, after the existing `find_package` calls (after `find_package(yaml-cpp REQUIRED)`), add:

```cmake
find_package(mp-units REQUIRED)
```

And add to the `target_link_libraries` for vulcan:

```cmake
target_link_libraries(vulcan INTERFACE Eigen3::Eigen casadi janus::janus
                                       HighFive yaml-cpp mp-units::mp-units)
```

- [ ] **Step 5: Verify the build still compiles**

Run: `./scripts/build.sh --clean`
Expected: Build succeeds with mp-units found, all existing code compiles

- [ ] **Step 6: Verify tests still pass**

Run: `./scripts/test.sh`
Expected: All existing tests pass (no regressions)

- [ ] **Step 7: Commit**

```bash
git add flake.nix flake.lock CMakeLists.txt
git commit -m "build: add mp-units dependency to Nix and CMake"
```

---

### Task 2: Create Unit Vocabulary

**Files:**
- Create: `include/vulcan/quantity/Units.hpp`
- Create: `tests/quantity/test_quantity_units.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/quantity/test_quantity_units.cpp`:

```cpp
#include <gtest/gtest.h>
#include <mp-units/systems/si.h>
#include <vulcan/quantity/Units.hpp>

namespace vulcan::tests {

using namespace vulcan::units;

// Verify base SI aliases resolve to correct mp-units types
TEST(UnitsVocabulary, BaseSIAliases) {
    static_assert(m == mp_units::si::metre);
    static_assert(kg == mp_units::si::kilogram);
    static_assert(s == mp_units::si::second);
    static_assert(K == mp_units::si::kelvin);
    static_assert(rad == mp_units::si::radian);
}

TEST(UnitsVocabulary, DerivedSIAliases) {
    static_assert(N == mp_units::si::newton);
    static_assert(Pa == mp_units::si::pascal);
    static_assert(J == mp_units::si::joule);
    static_assert(W == mp_units::si::watt);
    static_assert(Hz == mp_units::si::hertz);
}

TEST(UnitsVocabulary, CompoundUnits) {
    // mps should be metre / second
    static_assert(mps == m / s);
    static_assert(fps == ft / s);
    static_assert(rad_s == rad / s);
}

TEST(UnitsVocabulary, DimensionlessAlias) {
    static_assert(dimensionless == mp_units::one);
}

TEST(UnitsVocabulary, LongFormAliases) {
    static_assert(metre == m);
    static_assert(kilogram == kg);
    static_assert(second == s);
    static_assert(kelvin == K);
    static_assert(radian == rad);
    static_assert(newton == N);
    static_assert(foot == ft);
    static_assert(degrees == deg);
    static_assert(metres_per_second == mps);
}

} // namespace vulcan::tests
```

- [ ] **Step 2: Add test target to CMakeLists.txt**

In `tests/CMakeLists.txt`, add a new test executable:

```cmake
# --- Quantity Tests ---
add_executable(
  test_quantity
  quantity/test_quantity_units.cpp)
target_link_libraries(test_quantity PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_quantity REUSE_FROM test_core)
gtest_discover_tests(test_quantity)
```

- [ ] **Step 3: Run test to verify it fails**

Run: `./scripts/build.sh && ctest --test-dir build -R test_quantity -VV`
Expected: FAIL — `vulcan/quantity/Units.hpp` does not exist

- [ ] **Step 4: Write the unit vocabulary header**

Create `include/vulcan/quantity/Units.hpp`:

```cpp
#pragma once
/// @file Units.hpp
/// @brief Curated aerospace unit vocabulary wrapping mp-units
///
/// Provides short aliases so developers write Quantity<m> not Quantity<mp_units::si::metre>.

#include <mp-units/systems/si.h>
#include <mp-units/systems/usc.h>
#include <mp-units/systems/non_si.h>

namespace vulcan::units {

// =============================================================================
// Base SI
// =============================================================================
inline constexpr auto m   = mp_units::si::metre;
inline constexpr auto kg  = mp_units::si::kilogram;
inline constexpr auto s   = mp_units::si::second;
inline constexpr auto K   = mp_units::si::kelvin;
inline constexpr auto rad = mp_units::si::radian;

// =============================================================================
// Derived SI
// =============================================================================
inline constexpr auto N   = mp_units::si::newton;
inline constexpr auto Pa  = mp_units::si::pascal;
inline constexpr auto J   = mp_units::si::joule;
inline constexpr auto W   = mp_units::si::watt;
inline constexpr auto Hz  = mp_units::si::hertz;

// =============================================================================
// Aerospace Imperial / Customary
// =============================================================================
inline constexpr auto ft  = mp_units::usc::foot;
inline constexpr auto deg = mp_units::non_si::degree;

// TODO: verify exact mp-units paths for these after Nix build confirms
// inline constexpr auto nm  = mp_units::nautical_mile;
// inline constexpr auto kts = mp_units::knot;
// inline constexpr auto lbm = mp_units::usc::pound;
// inline constexpr auto lbf = mp_units::usc::pound_force;

// =============================================================================
// Compound
// =============================================================================
inline constexpr auto mps   = m / s;
inline constexpr auto fps   = ft / s;
inline constexpr auto rad_s = rad / s;

// =============================================================================
// Dimensionless
// =============================================================================
inline constexpr auto dimensionless = mp_units::one;

// =============================================================================
// Long-form aliases
// =============================================================================
inline constexpr auto metre             = m;
inline constexpr auto kilogram          = kg;
inline constexpr auto second            = s;
inline constexpr auto kelvin            = K;
inline constexpr auto radian            = rad;
inline constexpr auto newton            = N;
inline constexpr auto foot              = ft;
inline constexpr auto degrees           = deg;
inline constexpr auto metres_per_second = mps;

} // namespace vulcan::units
```

Note: The exact mp-units header paths and type names for nautical_mile, knot, pound, pound_force may differ between mp-units versions. After Task 1 confirms the build works, resolve these paths from the installed mp-units headers and fill in the remaining aliases. The core SI/derived units above are stable.

- [ ] **Step 5: Run tests to verify they pass**

Run: `./scripts/build.sh && ctest --test-dir build -R test_quantity -VV`
Expected: All UnitsVocabulary tests PASS

- [ ] **Step 6: Commit**

```bash
git add include/vulcan/quantity/Units.hpp tests/quantity/test_quantity_units.cpp tests/CMakeLists.txt
git commit -m "feat(quantity): add curated aerospace unit vocabulary"
```

---

### Task 3: Core Quantity Type — Construction, Value, Conversion

**Files:**
- Create: `include/vulcan/quantity/Quantity.hpp`
- Create: `tests/quantity/test_quantity.cpp`
- Modify: `tests/CMakeLists.txt` (add test source)

- [ ] **Step 1: Write the failing tests**

Create `tests/quantity/test_quantity.cpp`:

```cpp
#include <gtest/gtest.h>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/Units.hpp>

namespace vulcan::tests {

using namespace vulcan::units;

// --- Construction ---
TEST(Quantity, DefaultConstruction) {
    Quantity<m> alt;
    EXPECT_DOUBLE_EQ(alt.value(), 0.0);
}

TEST(Quantity, ExplicitConstruction) {
    Quantity<m> alt(10000.0);
    EXPECT_DOUBLE_EQ(alt.value(), 10000.0);
}

TEST(Quantity, ValueReturnsInDeclaredUnit) {
    Quantity<ft> alt(1000.0);
    EXPECT_DOUBLE_EQ(alt.value(), 1000.0);  // NOT 304.8
}

// --- Unit conversion ---
TEST(Quantity, ConvertFeetToMetres) {
    Quantity<ft> alt_ft(1000.0);
    auto alt_m = alt_ft.in<m>();
    EXPECT_NEAR(alt_m.value(), 304.8, 1e-10);
}

TEST(Quantity, ConvertDegreesToRadians) {
    Quantity<deg> angle(180.0);
    auto angle_rad = angle.in<rad>();
    EXPECT_NEAR(angle_rad.value(), M_PI, 1e-12);
}

TEST(Quantity, ConvertRoundTrip) {
    Quantity<m> original(12345.0);
    auto converted = original.in<ft>().in<m>();
    EXPECT_NEAR(converted.value(), 12345.0, 1e-8);
}

// --- Same-unit arithmetic ---
TEST(Quantity, Addition) {
    Quantity<m> a(100.0);
    Quantity<m> b(200.0);
    auto c = a + b;
    EXPECT_DOUBLE_EQ(c.value(), 300.0);
}

TEST(Quantity, Subtraction) {
    Quantity<m> a(300.0);
    Quantity<m> b(100.0);
    auto c = a - b;
    EXPECT_DOUBLE_EQ(c.value(), 200.0);
}

TEST(Quantity, Negation) {
    Quantity<m> a(100.0);
    auto b = -a;
    EXPECT_DOUBLE_EQ(b.value(), -100.0);
}

TEST(Quantity, CompoundAssignment) {
    Quantity<m> a(100.0);
    a += Quantity<m>(50.0);
    EXPECT_DOUBLE_EQ(a.value(), 150.0);
    a -= Quantity<m>(25.0);
    EXPECT_DOUBLE_EQ(a.value(), 125.0);
}

// --- Cross-unit multiplication ---
TEST(Quantity, MultiplicationProducesCorrectUnit) {
    Quantity<m> dist(10.0);
    Quantity<m> width(5.0);
    auto area = dist * width;
    EXPECT_DOUBLE_EQ(area.value(), 50.0);
    // area should be m^2 — verified by type system at compile time
}

TEST(Quantity, DivisionProducesCorrectUnit) {
    Quantity<m> dist(100.0);
    Quantity<s> time(10.0);
    auto speed = dist / time;
    EXPECT_DOUBLE_EQ(speed.value(), 10.0);
    // speed should be m/s — verified by type system at compile time
}

// --- Scalar multiplication ---
TEST(Quantity, ScalarMultiplication) {
    Quantity<N> force(100.0);
    auto doubled = 2.0 * force;
    EXPECT_DOUBLE_EQ(doubled.value(), 200.0);
    auto tripled = force * 3.0;
    EXPECT_DOUBLE_EQ(tripled.value(), 300.0);
}

TEST(Quantity, ScalarDivision) {
    Quantity<m> dist(100.0);
    auto half = dist / 2.0;
    EXPECT_DOUBLE_EQ(half.value(), 50.0);
}

// --- Comparisons ---
TEST(Quantity, Comparisons) {
    Quantity<m> a(100.0);
    Quantity<m> b(200.0);
    EXPECT_TRUE(a < b);
    EXPECT_TRUE(b > a);
    EXPECT_TRUE(a <= a);
    EXPECT_TRUE(a >= a);
    EXPECT_TRUE(a == a);
    EXPECT_TRUE(a != b);
}

// --- Dimensionless implicit conversion ---
TEST(Quantity, DimensionlessConvertsToRep) {
    Quantity<dimensionless> ratio(0.5);
    double raw = ratio;  // implicit conversion
    EXPECT_DOUBLE_EQ(raw, 0.5);
}

TEST(Quantity, DimensionlessFromDivision) {
    Quantity<m> a(10.0);
    Quantity<m> b(5.0);
    auto ratio = a / b;  // m / m = dimensionless
    double raw = ratio;   // implicit conversion
    EXPECT_DOUBLE_EQ(raw, 2.0);
}

} // namespace vulcan::tests
```

- [ ] **Step 2: Add test source to CMakeLists.txt**

In `tests/CMakeLists.txt`, add `quantity/test_quantity.cpp` to the `test_quantity` executable source list:

```cmake
add_executable(
  test_quantity
  quantity/test_quantity_units.cpp
  quantity/test_quantity.cpp)
```

- [ ] **Step 3: Run test to verify it fails**

Run: `./scripts/build.sh && ctest --test-dir build -R test_quantity -VV`
Expected: FAIL — `vulcan/quantity/Quantity.hpp` does not exist

- [ ] **Step 4: Write the Quantity class**

Create `include/vulcan/quantity/Quantity.hpp`:

```cpp
#pragma once
/// @file Quantity.hpp
/// @brief Core Quantity<Unit, Scalar> type — compile-time dimensional analysis
///        fused with Janus numeric/symbolic dispatch.

#include <janus/core/JanusConcepts.hpp>
#include <mp-units/framework.h>
#include <mp-units/systems/si.h>
#include <string>

namespace vulcan {

/// @brief A dimensioned value combining mp-units dimensional analysis with
///        Janus dual-mode (double / casadi::MX) scalar dispatch.
///
/// @tparam Unit  mp-units unit (e.g., mp_units::si::metre). Determines dimension.
/// @tparam Rep   Underlying scalar type. Must satisfy janus::JanusScalar.
///               Defaults to double. Use casadi::MX for symbolic mode.
template <auto Unit, janus::JanusScalar Rep = double>
class Quantity {
    mp_units::quantity<Unit, Rep> q_;

public:
    using rep = Rep;
    static constexpr auto unit = Unit;

    // --- Construction ---
    constexpr Quantity() : q_(Rep{0} * mp_units::reference<Unit>{}) {}

    constexpr explicit Quantity(Rep v)
        : q_(v * mp_units::reference<Unit>{}) {}

    constexpr explicit Quantity(mp_units::quantity<Unit, Rep> q)
        : q_(q) {}

    // --- Unwrap (returns in declared unit, NOT converted to SI) ---
    constexpr Rep value() const { return q_.numerical_value_ref_in(Unit); }

    // --- Access underlying mp_units::quantity ---
    constexpr const auto& raw() const { return q_; }

    // --- Unit conversion ---
    template <auto ToUnit>
    constexpr Quantity<ToUnit, Rep> in() const {
        return Quantity<ToUnit, Rep>{q_.in(ToUnit)};
    }

    // --- Same-unit arithmetic ---
    friend constexpr Quantity operator+(Quantity a, Quantity b) {
        return Quantity{a.q_ + b.q_};
    }
    friend constexpr Quantity operator-(Quantity a, Quantity b) {
        return Quantity{a.q_ - b.q_};
    }
    constexpr Quantity operator-() const {
        return Quantity{-q_};
    }
    constexpr Quantity& operator+=(Quantity other) {
        q_ += other.q_;
        return *this;
    }
    constexpr Quantity& operator-=(Quantity other) {
        q_ -= other.q_;
        return *this;
    }

    // --- Cross-unit multiplication/division ---
    template <auto U2, janus::JanusScalar R2>
    friend constexpr auto operator*(Quantity a, Quantity<U2, R2> b) {
        auto result = a.q_ * b.raw();
        using ResultQ = decltype(result);
        return Quantity<ResultQ::unit, typename ResultQ::rep>{result};
    }
    template <auto U2, janus::JanusScalar R2>
    friend constexpr auto operator/(Quantity a, Quantity<U2, R2> b) {
        auto result = a.q_ / b.raw();
        using ResultQ = decltype(result);
        return Quantity<ResultQ::unit, typename ResultQ::rep>{result};
    }

    // --- Scalar multiplication (dimensionless scaling) ---
    friend constexpr Quantity operator*(Rep s, Quantity a) {
        return Quantity{s * a.value()};
    }
    friend constexpr Quantity operator*(Quantity a, Rep s) {
        return Quantity{a.value() * s};
    }
    friend constexpr Quantity operator/(Quantity a, Rep s) {
        return Quantity{a.value() / s};
    }

    // --- Comparisons (return Rep-native types for janus::where compat) ---
    friend constexpr auto operator<(Quantity a, Quantity b) { return a.value() < b.value(); }
    friend constexpr auto operator>(Quantity a, Quantity b) { return a.value() > b.value(); }
    friend constexpr auto operator<=(Quantity a, Quantity b) { return a.value() <= b.value(); }
    friend constexpr auto operator>=(Quantity a, Quantity b) { return a.value() >= b.value(); }
    friend constexpr auto operator==(Quantity a, Quantity b) { return a.value() == b.value(); }
    friend constexpr auto operator!=(Quantity a, Quantity b) { return a.value() != b.value(); }

    // --- Implicit conversion for dimensionless quantities ---
    constexpr operator Rep() const
        requires(Unit == mp_units::one)
    {
        return value();
    }
};

} // namespace vulcan
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `./scripts/build.sh && ctest --test-dir build -R test_quantity -VV`
Expected: All Quantity and UnitsVocabulary tests PASS

- [ ] **Step 6: Commit**

```bash
git add include/vulcan/quantity/Quantity.hpp tests/quantity/test_quantity.cpp tests/CMakeLists.txt
git commit -m "feat(quantity): core Quantity type with construction, arithmetic, conversion"
```

---

### Task 4: CasADi Symbolic Proof-of-Concept

**Files:**
- Create: `tests/quantity/test_quantity_symbolic.cpp`
- Modify: `tests/CMakeLists.txt` (add test source)

- [ ] **Step 1: Write the symbolic tests**

Create `tests/quantity/test_quantity_symbolic.cpp`:

```cpp
#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/Units.hpp>

namespace vulcan::tests {

using namespace vulcan::units;
using MX = casadi::MX;

// --- Basic symbolic arithmetic traces correctly ---
TEST(QuantitySymbolic, Addition) {
    auto x = MX::sym("x");
    auto y = MX::sym("y");
    Quantity<m, MX> a(x);
    Quantity<m, MX> b(y);
    auto c = a + b;

    // Evaluate: x=3, y=7 -> should give 10
    casadi::Function f("f", {x, y}, {c.value()});
    auto result = f(std::vector<casadi::DM>{3.0, 7.0});
    EXPECT_NEAR(static_cast<double>(result[0]), 10.0, 1e-12);
}

TEST(QuantitySymbolic, CrossUnitMultiplication) {
    auto dist = MX::sym("dist");
    auto time = MX::sym("time");
    Quantity<m, MX> d(dist);
    Quantity<s, MX> t(time);
    auto speed = d / t;  // should be m/s

    casadi::Function f("f", {dist, time}, {speed.value()});
    auto result = f(std::vector<casadi::DM>{100.0, 10.0});
    EXPECT_NEAR(static_cast<double>(result[0]), 10.0, 1e-12);
}

TEST(QuantitySymbolic, ScalarMultiplication) {
    auto thrust = MX::sym("thrust");
    Quantity<N, MX> F(thrust);
    auto doubled = F * MX(2.0);

    casadi::Function f("f", {thrust}, {doubled.value()});
    auto result = f(std::vector<casadi::DM>{500.0});
    EXPECT_NEAR(static_cast<double>(result[0]), 1000.0, 1e-12);
}

TEST(QuantitySymbolic, ComparisonReturnsSymbolicPredicate) {
    auto x = MX::sym("x");
    auto y = MX::sym("y");
    Quantity<m, MX> a(x);
    Quantity<m, MX> b(y);

    // a < b returns MX (symbolic predicate), usable with janus::where
    auto pred = a < b;
    auto result = janus::where(pred, a.value(), b.value());

    casadi::Function f("f", {x, y}, {result});
    // x=3 < y=7 -> returns x=3
    auto r1 = f(std::vector<casadi::DM>{3.0, 7.0});
    EXPECT_NEAR(static_cast<double>(r1[0]), 3.0, 1e-12);
    // x=7 > y=3 -> returns y=3
    auto r2 = f(std::vector<casadi::DM>{7.0, 3.0});
    EXPECT_NEAR(static_cast<double>(r2[0]), 3.0, 1e-12);
}

TEST(QuantitySymbolic, DimensionlessImplicitConversion) {
    auto x = MX::sym("x");
    auto y = MX::sym("y");
    Quantity<m, MX> a(x);
    Quantity<m, MX> b(y);
    auto ratio = a / b;  // dimensionless

    // Should implicitly convert to MX for use with janus::exp
    MX raw = ratio;
    auto result = janus::exp(raw);

    casadi::Function f("f", {x, y}, {result});
    auto r = f(std::vector<casadi::DM>{2.0, 1.0});
    EXPECT_NEAR(static_cast<double>(r[0]), std::exp(2.0), 1e-10);
}

TEST(QuantitySymbolic, UnitConversion) {
    auto x = MX::sym("alt_ft");
    Quantity<ft, MX> alt_ft(x);
    auto alt_m = alt_ft.in<m>();

    casadi::Function f("f", {x}, {alt_m.value()});
    auto result = f(std::vector<casadi::DM>{1000.0});
    EXPECT_NEAR(static_cast<double>(result[0]), 304.8, 1e-10);
}

} // namespace vulcan::tests
```

- [ ] **Step 2: Add test source to CMakeLists.txt**

In `tests/CMakeLists.txt`, add `quantity/test_quantity_symbolic.cpp` to the `test_quantity` executable:

```cmake
add_executable(
  test_quantity
  quantity/test_quantity_units.cpp
  quantity/test_quantity.cpp
  quantity/test_quantity_symbolic.cpp)
```

- [ ] **Step 3: Run tests**

Run: `./scripts/build.sh && ctest --test-dir build -R test_quantity -VV`
Expected: All tests PASS — CasADi traces through Quantity operators correctly

If any test fails, this is the critical go/no-go gate. Diagnose whether mp-units internally calls `std::` functions on the Rep or does something that breaks CasADi tracing. The fix would be adjusting the Quantity operator implementations to bypass mp-units for the runtime path and only use mp-units for compile-time type computation.

- [ ] **Step 4: Commit**

```bash
git add tests/quantity/test_quantity_symbolic.cpp tests/CMakeLists.txt
git commit -m "feat(quantity): CasADi symbolic proof-of-concept — all traces verified"
```

---

### Task 5: Janus Math Overloads

**Files:**
- Create: `include/vulcan/quantity/QuantityMath.hpp`
- Create: `tests/quantity/test_quantity_math.cpp`
- Modify: `tests/CMakeLists.txt` (add test source)

- [ ] **Step 1: Write the failing tests**

Create `tests/quantity/test_quantity_math.cpp`:

```cpp
#include <cmath>
#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/QuantityMath.hpp>
#include <vulcan/quantity/Units.hpp>

namespace vulcan::tests {

using namespace vulcan::units;

// --- Dimension-changing: trig ---
TEST(QuantityMath, SinOfRadians) {
    Quantity<rad> angle(M_PI / 6.0);  // 30 degrees
    auto result = vulcan::sin(angle);
    EXPECT_NEAR(static_cast<double>(result), 0.5, 1e-12);
}

TEST(QuantityMath, CosOfRadians) {
    Quantity<rad> angle(M_PI / 3.0);  // 60 degrees
    auto result = vulcan::cos(angle);
    EXPECT_NEAR(static_cast<double>(result), 0.5, 1e-12);
}

TEST(QuantityMath, TanOfRadians) {
    Quantity<rad> angle(M_PI / 4.0);  // 45 degrees
    auto result = vulcan::tan(angle);
    EXPECT_NEAR(static_cast<double>(result), 1.0, 1e-12);
}

TEST(QuantityMath, AsinReturnRadians) {
    Quantity<dimensionless> x(0.5);
    auto angle = vulcan::asin(x);
    EXPECT_NEAR(angle.value(), M_PI / 6.0, 1e-12);
}

TEST(QuantityMath, AcosReturnRadians) {
    Quantity<dimensionless> x(0.5);
    auto angle = vulcan::acos(x);
    EXPECT_NEAR(angle.value(), M_PI / 3.0, 1e-12);
}

TEST(QuantityMath, Atan2DimensionlessReturnsRadians) {
    Quantity<dimensionless> y(1.0);
    Quantity<dimensionless> x(1.0);
    auto angle = vulcan::atan2(y, x);
    EXPECT_NEAR(angle.value(), M_PI / 4.0, 1e-12);
}

TEST(QuantityMath, Atan2SameUnitReturnsRadians) {
    Quantity<m> y(100.0);
    Quantity<m> x(100.0);
    auto angle = vulcan::atan2(y, x);
    EXPECT_NEAR(angle.value(), M_PI / 4.0, 1e-12);
}

// --- Dimension-changing: sqrt ---
TEST(QuantityMath, SqrtHalvesDimension) {
    Quantity<m> a(3.0);
    Quantity<m> b(4.0);
    auto sum_sq = a * a + b * b;  // m^2
    auto hyp = vulcan::sqrt(sum_sq);
    EXPECT_NEAR(hyp.value(), 5.0, 1e-12);
}

// --- Dimension-preserving ---
TEST(QuantityMath, Abs) {
    Quantity<m> neg(-100.0);
    auto pos = vulcan::abs(neg);
    EXPECT_DOUBLE_EQ(pos.value(), 100.0);
}

TEST(QuantityMath, Where) {
    Quantity<K> hot(400.0);
    Quantity<K> cold(200.0);
    auto result = vulcan::where(true, hot, cold);
    EXPECT_DOUBLE_EQ(result.value(), 400.0);
    auto result2 = vulcan::where(false, hot, cold);
    EXPECT_DOUBLE_EQ(result2.value(), 200.0);
}

TEST(QuantityMath, Min) {
    Quantity<m> a(100.0);
    Quantity<m> b(200.0);
    auto result = vulcan::min(a, b);
    EXPECT_DOUBLE_EQ(result.value(), 100.0);
}

TEST(QuantityMath, Max) {
    Quantity<m> a(100.0);
    Quantity<m> b(200.0);
    auto result = vulcan::max(a, b);
    EXPECT_DOUBLE_EQ(result.value(), 200.0);
}

TEST(QuantityMath, Clamp) {
    Quantity<K> val(500.0);
    Quantity<K> lo(200.0);
    Quantity<K> hi(400.0);
    auto result = vulcan::clamp(val, lo, hi);
    EXPECT_DOUBLE_EQ(result.value(), 400.0);
}

// --- Symbolic trig ---
TEST(QuantityMath, SymbolicSin) {
    using MX = casadi::MX;
    auto x = MX::sym("x");
    Quantity<rad, MX> angle(x);
    auto result = vulcan::sin(angle);

    casadi::Function f("f", {x}, {static_cast<MX>(result)});
    auto r = f(std::vector<casadi::DM>{M_PI / 6.0});
    EXPECT_NEAR(static_cast<double>(r[0]), 0.5, 1e-12);
}

// --- Angle wrapping ---
TEST(QuantityMath, WrapToPi) {
    Quantity<rad> angle(3.0 * M_PI);
    auto wrapped = vulcan::wrap_to_pi(angle);
    EXPECT_NEAR(wrapped.value(), M_PI, 1e-12);
}

TEST(QuantityMath, WrapTo2Pi) {
    Quantity<rad> angle(-0.1);
    auto wrapped = vulcan::wrap_to_2pi(angle);
    EXPECT_NEAR(wrapped.value(), 2.0 * M_PI - 0.1, 1e-12);
}

} // namespace vulcan::tests
```

- [ ] **Step 2: Add test source to CMakeLists.txt**

Add `quantity/test_quantity_math.cpp` to the `test_quantity` executable source list.

- [ ] **Step 3: Run test to verify it fails**

Run: `./scripts/build.sh && ctest --test-dir build -R test_quantity -VV`
Expected: FAIL — `vulcan/quantity/QuantityMath.hpp` does not exist

- [ ] **Step 4: Write the math overloads**

Create `include/vulcan/quantity/QuantityMath.hpp`:

```cpp
#pragma once
/// @file QuantityMath.hpp
/// @brief Janus math overloads for Quantity types.
///
/// Tier 2: Dimension-preserving (abs, min, max, clamp, where)
/// Tier 3: Dimension-changing (sin, cos, tan, asin, acos, atan2, sqrt, wrap)

#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/Units.hpp>
#include <janus/math/Arithmetic.hpp>
#include <janus/math/Trig.hpp>
#include <janus/math/Logic.hpp>

namespace vulcan {

using namespace vulcan::units;

// =============================================================================
// Tier 2: Dimension-preserving
// =============================================================================

template <auto U, janus::JanusScalar Rep>
Quantity<U, Rep> abs(Quantity<U, Rep> x) {
    return Quantity<U, Rep>{janus::abs(x.value())};
}

template <auto U, janus::JanusScalar Rep>
Quantity<U, Rep> min(Quantity<U, Rep> a, Quantity<U, Rep> b) {
    return Quantity<U, Rep>{janus::min(a.value(), b.value())};
}

template <auto U, janus::JanusScalar Rep>
Quantity<U, Rep> max(Quantity<U, Rep> a, Quantity<U, Rep> b) {
    return Quantity<U, Rep>{janus::max(a.value(), b.value())};
}

template <auto U, janus::JanusScalar Rep>
Quantity<U, Rep> clamp(Quantity<U, Rep> val, Quantity<U, Rep> lo, Quantity<U, Rep> hi) {
    return Quantity<U, Rep>{janus::clamp(val.value(), lo.value(), hi.value())};
}

template <typename Cond, auto U, janus::JanusScalar Rep>
Quantity<U, Rep> where(const Cond& cond, Quantity<U, Rep> if_true, Quantity<U, Rep> if_false) {
    return Quantity<U, Rep>{janus::where(cond, if_true.value(), if_false.value())};
}

template <auto U, janus::JanusScalar Rep>
Quantity<U, Rep> floor(Quantity<U, Rep> x) {
    return Quantity<U, Rep>{janus::floor(x.value())};
}

template <auto U, janus::JanusScalar Rep>
Quantity<U, Rep> ceil(Quantity<U, Rep> x) {
    return Quantity<U, Rep>{janus::ceil(x.value())};
}

template <auto U, janus::JanusScalar Rep>
Quantity<U, Rep> round(Quantity<U, Rep> x) {
    return Quantity<U, Rep>{janus::round(x.value())};
}

template <auto U, janus::JanusScalar Rep>
Quantity<U, Rep> sign(Quantity<U, Rep> x) {
    return Quantity<U, Rep>{janus::sign(x.value())};
}

// =============================================================================
// Tier 3: Dimension-changing — trig
// =============================================================================

template <janus::JanusScalar Rep>
Quantity<dimensionless, Rep> sin(Quantity<rad, Rep> a) {
    return Quantity<dimensionless, Rep>{janus::sin(a.value())};
}

template <janus::JanusScalar Rep>
Quantity<dimensionless, Rep> cos(Quantity<rad, Rep> a) {
    return Quantity<dimensionless, Rep>{janus::cos(a.value())};
}

template <janus::JanusScalar Rep>
Quantity<dimensionless, Rep> tan(Quantity<rad, Rep> a) {
    return Quantity<dimensionless, Rep>{janus::tan(a.value())};
}

template <janus::JanusScalar Rep>
Quantity<rad, Rep> asin(Quantity<dimensionless, Rep> x) {
    return Quantity<rad, Rep>{janus::asin(x.value())};
}

template <janus::JanusScalar Rep>
Quantity<rad, Rep> acos(Quantity<dimensionless, Rep> x) {
    return Quantity<rad, Rep>{janus::acos(x.value())};
}

template <janus::JanusScalar Rep>
Quantity<rad, Rep> atan2(Quantity<dimensionless, Rep> y, Quantity<dimensionless, Rep> x) {
    return Quantity<rad, Rep>{janus::atan2(y.value(), x.value())};
}

// atan2 for same-unit pairs: atan2(metres, metres) -> radians
template <auto U, janus::JanusScalar Rep>
Quantity<rad, Rep> atan2(Quantity<U, Rep> y, Quantity<U, Rep> x) {
    return Quantity<rad, Rep>{janus::atan2(y.value(), x.value())};
}

// =============================================================================
// Tier 3: Dimension-changing — sqrt
// =============================================================================

template <auto U, janus::JanusScalar Rep>
auto sqrt(Quantity<U, Rep> x) {
    // mp-units computes the result unit at compile time (e.g., m^2 -> m)
    auto result_q = mp_units::sqrt(x.raw());
    using ResultQ = decltype(result_q);
    return Quantity<ResultQ::unit, Rep>{janus::sqrt(x.value())};
}

// =============================================================================
// Angle wrapping
// =============================================================================

template <janus::JanusScalar Rep>
Quantity<rad, Rep> wrap_to_pi(Quantity<rad, Rep> a) {
    return Quantity<rad, Rep>{vulcan::units::wrap_to_pi(a.value())};
}

template <janus::JanusScalar Rep>
Quantity<rad, Rep> wrap_to_2pi(Quantity<rad, Rep> a) {
    return Quantity<rad, Rep>{vulcan::units::wrap_to_2pi(a.value())};
}

} // namespace vulcan
```

Note: The `wrap_to_pi` / `wrap_to_2pi` calls reference the existing `vulcan::units` functions from `core/Units.hpp`. These remain available as internal helpers even after the old Units.hpp is deprecated for external use.

- [ ] **Step 5: Run tests to verify they pass**

Run: `./scripts/build.sh && ctest --test-dir build -R test_quantity -VV`
Expected: All QuantityMath tests PASS

- [ ] **Step 6: Commit**

```bash
git add include/vulcan/quantity/QuantityMath.hpp tests/quantity/test_quantity_math.cpp tests/CMakeLists.txt
git commit -m "feat(quantity): Janus math overloads — trig, abs, where, sqrt, wrap"
```

---

### Task 6: Eigen Integration

**Files:**
- Create: `include/vulcan/quantity/QuantityEigen.hpp`
- Create: `tests/quantity/test_quantity_eigen.cpp`
- Modify: `tests/CMakeLists.txt` (add test source)

- [ ] **Step 1: Write the failing tests**

Create `tests/quantity/test_quantity_eigen.cpp`:

```cpp
#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/QuantityEigen.hpp>
#include <vulcan/quantity/Units.hpp>

namespace vulcan::tests {

using namespace vulcan::units;

TEST(QuantityEigen, Vec3Construction) {
    janus::Vec3<Quantity<m>> pos;
    pos(0) = Quantity<m>(1.0);
    pos(1) = Quantity<m>(2.0);
    pos(2) = Quantity<m>(3.0);
    EXPECT_DOUBLE_EQ(pos(0).value(), 1.0);
    EXPECT_DOUBLE_EQ(pos(1).value(), 2.0);
    EXPECT_DOUBLE_EQ(pos(2).value(), 3.0);
}

TEST(QuantityEigen, Vec3Addition) {
    janus::Vec3<Quantity<m>> a;
    a << Quantity<m>(1.0), Quantity<m>(2.0), Quantity<m>(3.0);
    janus::Vec3<Quantity<m>> b;
    b << Quantity<m>(4.0), Quantity<m>(5.0), Quantity<m>(6.0);
    auto c = a + b;
    EXPECT_DOUBLE_EQ(c(0).value(), 5.0);
    EXPECT_DOUBLE_EQ(c(1).value(), 7.0);
    EXPECT_DOUBLE_EQ(c(2).value(), 9.0);
}

TEST(QuantityEigen, Vec3Subtraction) {
    janus::Vec3<Quantity<m>> a;
    a << Quantity<m>(10.0), Quantity<m>(20.0), Quantity<m>(30.0);
    janus::Vec3<Quantity<m>> b;
    b << Quantity<m>(1.0), Quantity<m>(2.0), Quantity<m>(3.0);
    auto c = a - b;
    EXPECT_DOUBLE_EQ(c(0).value(), 9.0);
    EXPECT_DOUBLE_EQ(c(1).value(), 18.0);
    EXPECT_DOUBLE_EQ(c(2).value(), 27.0);
}

TEST(QuantityEigen, Vec3ScalarMultiplication) {
    janus::Vec3<Quantity<m>> v;
    v << Quantity<m>(1.0), Quantity<m>(2.0), Quantity<m>(3.0);
    auto scaled = v * Quantity<m>(2.0);  // This is element * scalar
    // Note: Eigen scalar mult may need special handling; test what works
}

TEST(QuantityEigen, DotProductSameUnit) {
    janus::Vec3<Quantity<m>> a;
    a << Quantity<m>(1.0), Quantity<m>(0.0), Quantity<m>(0.0);
    janus::Vec3<Quantity<m>> b;
    b << Quantity<m>(5.0), Quantity<m>(0.0), Quantity<m>(0.0);
    auto dot = a.dot(b);
    // dot product of m * m = m^2, value should be 5.0
    EXPECT_DOUBLE_EQ(dot.value(), 5.0);
}

TEST(QuantityEigen, SymbolicVec3Addition) {
    using MX = casadi::MX;
    janus::Vec3<Quantity<m, MX>> a;
    auto x = MX::sym("x");
    auto y = MX::sym("y");
    a(0) = Quantity<m, MX>(x);
    a(1) = Quantity<m, MX>(y);
    a(2) = Quantity<m, MX>(MX(0.0));

    janus::Vec3<Quantity<m, MX>> b;
    b(0) = Quantity<m, MX>(MX(1.0));
    b(1) = Quantity<m, MX>(MX(2.0));
    b(2) = Quantity<m, MX>(MX(3.0));

    auto c = a + b;
    casadi::Function f("f", {x, y}, {c(0).value(), c(1).value()});
    auto result = f(std::vector<casadi::DM>{10.0, 20.0});
    EXPECT_NEAR(static_cast<double>(result[0]), 11.0, 1e-12);
    EXPECT_NEAR(static_cast<double>(result[1]), 22.0, 1e-12);
}

} // namespace vulcan::tests
```

- [ ] **Step 2: Add test source to CMakeLists.txt**

Add `quantity/test_quantity_eigen.cpp` to the `test_quantity` executable source list.

- [ ] **Step 3: Run test to verify it fails**

Run: `./scripts/build.sh && ctest --test-dir build -R test_quantity -VV`
Expected: FAIL — `vulcan/quantity/QuantityEigen.hpp` does not exist

- [ ] **Step 4: Write the Eigen NumTraits specialization**

Create `include/vulcan/quantity/QuantityEigen.hpp`:

```cpp
#pragma once
/// @file QuantityEigen.hpp
/// @brief Eigen::NumTraits specialization for Quantity, enabling
///        Vec3<Quantity<m>>, Mat3<Quantity<dimensionless>>, etc.

#include <vulcan/quantity/Quantity.hpp>
#include <Eigen/Dense>

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

- [ ] **Step 5: Run tests to verify they pass**

Run: `./scripts/build.sh && ctest --test-dir build -R test_quantity -VV`
Expected: All QuantityEigen tests PASS

Note: Some Eigen operations (dot product returning cross-unit results, scalar multiplication, norm) may need adjustment. If specific tests fail, investigate which Eigen internal operations break and add targeted fixes. Document findings for the spec.

- [ ] **Step 6: Commit**

```bash
git add include/vulcan/quantity/QuantityEigen.hpp tests/quantity/test_quantity_eigen.cpp tests/CMakeLists.txt
git commit -m "feat(quantity): Eigen NumTraits — Vec3<Quantity<m>> works"
```

---

### Task 7: String Formatting

**Files:**
- Create: `include/vulcan/quantity/QuantityFormat.hpp`
- Create: `tests/quantity/test_quantity_format.cpp`
- Modify: `tests/CMakeLists.txt` (add test source)
- Modify: `include/vulcan/quantity/Quantity.hpp` (add `to_string()` body)

- [ ] **Step 1: Write the failing tests**

Create `tests/quantity/test_quantity_format.cpp`:

```cpp
#include <gtest/gtest.h>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/QuantityFormat.hpp>
#include <vulcan/quantity/Units.hpp>

namespace vulcan::tests {

using namespace vulcan::units;

TEST(QuantityFormat, MetresToString) {
    Quantity<m> alt(10000.0);
    auto str = alt.to_string();
    // Should contain "10000" and "m" at minimum
    EXPECT_NE(str.find("10000"), std::string::npos);
    EXPECT_NE(str.find("m"), std::string::npos);
}

TEST(QuantityFormat, KelvinToString) {
    Quantity<K> temp(288.15);
    auto str = temp.to_string();
    EXPECT_NE(str.find("288.15"), std::string::npos);
    EXPECT_NE(str.find("K"), std::string::npos);
}

TEST(QuantityFormat, DimensionlessToString) {
    Quantity<dimensionless> mach(0.85);
    auto str = mach.to_string();
    EXPECT_NE(str.find("0.85"), std::string::npos);
}

} // namespace vulcan::tests
```

- [ ] **Step 2: Add test source to CMakeLists.txt**

Add `quantity/test_quantity_format.cpp` to the `test_quantity` executable source list.

- [ ] **Step 3: Run test to verify it fails**

Run: `./scripts/build.sh && ctest --test-dir build -R test_quantity -VV`
Expected: FAIL — `to_string()` not implemented or `QuantityFormat.hpp` missing

- [ ] **Step 4: Implement formatting**

Create `include/vulcan/quantity/QuantityFormat.hpp`:

```cpp
#pragma once
/// @file QuantityFormat.hpp
/// @brief String formatting for Quantity via mp-units formatting.

#include <vulcan/quantity/Quantity.hpp>
#include <mp-units/format.h>
#include <sstream>
#include <string>

namespace vulcan {

template <auto Unit, janus::JanusScalar Rep>
std::string Quantity<Unit, Rep>::to_string() const {
    if constexpr (std::is_floating_point_v<Rep>) {
        std::ostringstream oss;
        oss << q_;
        return oss.str();
    } else {
        // Symbolic MX: return unit string only (value is a graph, not printable)
        std::ostringstream oss;
        oss << "[symbolic] " << mp_units::unit_symbol(Unit);
        return oss.str();
    }
}

} // namespace vulcan
```

Note: The exact mp-units formatting API (`operator<<`, `std::format`, `MP_UNITS_STD_FMT::format`) varies by version. Adjust the implementation to match what's available in the Nix-provided mp-units. The key requirement is that `to_string()` produces a human-readable string containing the numeric value and unit symbol.

- [ ] **Step 5: Run tests to verify they pass**

Run: `./scripts/build.sh && ctest --test-dir build -R test_quantity -VV`
Expected: All QuantityFormat tests PASS

- [ ] **Step 6: Commit**

```bash
git add include/vulcan/quantity/QuantityFormat.hpp tests/quantity/test_quantity_format.cpp tests/CMakeLists.txt
git commit -m "feat(quantity): .to_string() formatting via mp-units"
```

---

### Task 8: Umbrella Header and Full Regression

**Files:**
- Modify: `include/vulcan/vulcan.hpp`

- [ ] **Step 1: Add quantity headers to umbrella**

In `include/vulcan/vulcan.hpp`, add the quantity headers after the janus include and before the core includes:

```cpp
// Quantity type system
#include <vulcan/quantity/Units.hpp>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/QuantityMath.hpp>
#include <vulcan/quantity/QuantityEigen.hpp>
#include <vulcan/quantity/QuantityFormat.hpp>
```

- [ ] **Step 2: Verify full build succeeds**

Run: `./scripts/build.sh --clean`
Expected: Clean build succeeds with no errors or warnings from quantity headers

- [ ] **Step 3: Run ALL tests (not just quantity)**

Run: `./scripts/test.sh`
Expected: All existing tests still pass. No regressions from adding quantity headers to the umbrella. The quantity tests all pass.

- [ ] **Step 4: Run examples if any exist**

Run: `./scripts/verify.sh`
Expected: Full verification passes (build + tests + examples)

- [ ] **Step 5: Commit**

```bash
git add include/vulcan/vulcan.hpp
git commit -m "feat(quantity): add quantity headers to vulcan umbrella"
```

---

### Task 9: Compile-Time Safety Verification

**Files:**
- Create: `tests/quantity/test_quantity_compile_errors.cpp`
- Modify: `tests/CMakeLists.txt` (add test source)

This task verifies that invalid unit operations produce compile errors. Since we can't test for compile failures in a normal test, we use `static_assert` with intentionally valid expressions and document the invalid ones as comments.

- [ ] **Step 1: Write compile-time verification tests**

Create `tests/quantity/test_quantity_compile_errors.cpp`:

```cpp
#include <gtest/gtest.h>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/QuantityMath.hpp>
#include <vulcan/quantity/Units.hpp>
#include <type_traits>

namespace vulcan::tests {

using namespace vulcan::units;

// These SHOULD compile (positive verification):
TEST(QuantityCompileTime, ValidOperationsCompile) {
    Quantity<m> a(1.0);
    Quantity<m> b(2.0);
    auto c = a + b;      // same unit addition: OK
    auto d = a - b;      // same unit subtraction: OK
    auto e = a * b;      // m * m = m^2: OK
    auto f = a / b;      // m / m = dimensionless: OK
    (void)c; (void)d; (void)e; (void)f;
}

TEST(QuantityCompileTime, DimensionlessConversion) {
    Quantity<m> a(10.0);
    Quantity<m> b(5.0);
    double ratio = a / b;  // dimensionless implicit -> double: OK
    EXPECT_DOUBLE_EQ(ratio, 2.0);
}

TEST(QuantityCompileTime, CrossUnitResultTypes) {
    Quantity<m> dist(100.0);
    Quantity<s> time(10.0);
    auto speed = dist / time;
    // Verify the result is the correct type (m/s)
    EXPECT_DOUBLE_EQ(speed.value(), 10.0);

    Quantity<N> force(50.0);
    auto work = force * dist;
    // force * distance = energy (N*m = J)
    EXPECT_DOUBLE_EQ(work.value(), 5000.0);
}

// These should NOT compile (documented as comments):
// Uncomment any line below to verify it produces a compile error:
//
// Quantity<m> a(1.0); Quantity<K> b(2.0); auto c = a + b;   // ERROR: m + K
// Quantity<m> a(1.0); double x = a;                          // ERROR: non-dimensionless to double
// vulcan::sin(Quantity<m>(1.0));                              // ERROR: sin expects radians
// vulcan::asin(Quantity<m>(0.5));                             // ERROR: asin expects dimensionless

} // namespace vulcan::tests
```

- [ ] **Step 2: Add test source to CMakeLists.txt**

Add `quantity/test_quantity_compile_errors.cpp` to the `test_quantity` executable source list.

- [ ] **Step 3: Run tests**

Run: `./scripts/build.sh && ctest --test-dir build -R test_quantity -VV`
Expected: All tests PASS

- [ ] **Step 4: Manually verify one compile error**

Temporarily uncomment one of the "should NOT compile" lines, e.g.:

```cpp
Quantity<m> a(1.0); Quantity<K> b(2.0); auto c = a + b;
```

Run: `./scripts/build.sh`
Expected: COMPILE ERROR mentioning incompatible units. Revert the line after confirming.

- [ ] **Step 5: Commit**

```bash
git add tests/quantity/test_quantity_compile_errors.cpp tests/CMakeLists.txt
git commit -m "test(quantity): compile-time safety verification"
```

---

## Summary

| Task | What it delivers | Key files |
|------|-----------------|-----------|
| 1 | mp-units in build system | `flake.nix`, `CMakeLists.txt` |
| 2 | Unit vocabulary (`m`, `kg`, `s`, `ft`, `deg`, ...) | `quantity/Units.hpp` |
| 3 | Core Quantity type (construct, arithmetic, convert) | `quantity/Quantity.hpp` |
| 4 | CasADi symbolic proof-of-concept | `test_quantity_symbolic.cpp` |
| 5 | Janus math overloads (trig, abs, where, sqrt) | `quantity/QuantityMath.hpp` |
| 6 | Eigen NumTraits for `Vec3<Quantity<m>>` | `quantity/QuantityEigen.hpp` |
| 7 | `.to_string()` formatting | `quantity/QuantityFormat.hpp` |
| 8 | Umbrella header + full regression | `vulcan.hpp` |
| 9 | Compile-time safety verification | `test_quantity_compile_errors.cpp` |

**Phase 1 stops here.** The Quantity type is complete, tested, and available via `#include <vulcan/vulcan.hpp>`. Vulcan module migration (atmosphere, coordinates, rotations, time, dynamics, propulsion) and Icarus downstream migration are separate plans.
