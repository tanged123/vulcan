// Tests for vulcan::Quantity math overloads (QuantityMath.hpp)
//
// Covers Tier 2 (dimension-preserving) and Tier 3 (dimension-changing)
// functions with both numeric (double) and symbolic (janus::SymbolicScalar)
// representations.
#include <gtest/gtest.h>

#include <vulcan/quantity/QuantityMath.hpp>

#include <janus/core/Function.hpp>
#include <janus/core/JanusTypes.hpp>

#include <cmath>

namespace vu = vulcan::units;
using Sym = janus::SymbolicScalar;

static constexpr double PI = vulcan::constants::angle::pi.value();

// ============================================================================
// Tier 2 — Dimension-preserving (numeric)
// ============================================================================

TEST(QuantityMath, AbsNumeric) {
    vulcan::Quantity<vu::m> neg{-100.0};
    auto result = vulcan::abs(neg);
    EXPECT_NEAR(result.value(), 100.0, 1e-12);
}

TEST(QuantityMath, FloorNumeric) {
    vulcan::Quantity<vu::m> q{3.7};
    auto result = vulcan::floor(q);
    EXPECT_NEAR(result.value(), 3.0, 1e-12);
}

TEST(QuantityMath, CeilNumeric) {
    vulcan::Quantity<vu::m> q{3.2};
    auto result = vulcan::ceil(q);
    EXPECT_NEAR(result.value(), 4.0, 1e-12);
}

TEST(QuantityMath, RoundNumeric) {
    vulcan::Quantity<vu::m> q{3.5};
    auto result = vulcan::round(q);
    EXPECT_NEAR(result.value(), 4.0, 1e-12);
}

TEST(QuantityMath, SignNumeric) {
    vulcan::Quantity<vu::m> pos{42.0};
    vulcan::Quantity<vu::m> neg{-7.0};
    EXPECT_NEAR(vulcan::sign(pos).value(), 1.0, 1e-12);
    EXPECT_NEAR(vulcan::sign(neg).value(), -1.0, 1e-12);
}

TEST(QuantityMath, MinNumeric) {
    vulcan::Quantity<vu::m> a{3.0};
    vulcan::Quantity<vu::m> b{7.0};
    auto result = vulcan::min(a, b);
    EXPECT_NEAR(result.value(), 3.0, 1e-12);
}

TEST(QuantityMath, MaxNumeric) {
    vulcan::Quantity<vu::m> a{3.0};
    vulcan::Quantity<vu::m> b{7.0};
    auto result = vulcan::max(a, b);
    EXPECT_NEAR(result.value(), 7.0, 1e-12);
}

TEST(QuantityMath, ClampNumeric) {
    vulcan::Quantity<vu::m> val{15.0};
    vulcan::Quantity<vu::m> lo{0.0};
    vulcan::Quantity<vu::m> hi{10.0};
    auto result = vulcan::clamp(val, lo, hi);
    EXPECT_NEAR(result.value(), 10.0, 1e-12);

    vulcan::Quantity<vu::m> val2{-5.0};
    auto result2 = vulcan::clamp(val2, lo, hi);
    EXPECT_NEAR(result2.value(), 0.0, 1e-12);
}

TEST(QuantityMath, WhereNumeric) {
    vulcan::Quantity<vu::m> a{3.0};
    vulcan::Quantity<vu::m> b{7.0};
    auto result = vulcan::where(true, a, b);
    EXPECT_NEAR(result.value(), 3.0, 1e-12);

    auto result2 = vulcan::where(false, a, b);
    EXPECT_NEAR(result2.value(), 7.0, 1e-12);
}

// ============================================================================
// Tier 3 — Trig (numeric)
// ============================================================================

TEST(QuantityMath, SinNumeric) {
    vulcan::Quantity<vu::rad> angle{PI / 6.0}; // 30 degrees
    auto result = vulcan::sin(angle);
    EXPECT_NEAR(static_cast<double>(result), 0.5, 1e-12);
}

TEST(QuantityMath, CosNumeric) {
    vulcan::Quantity<vu::rad> angle{PI / 3.0}; // 60 degrees
    auto result = vulcan::cos(angle);
    EXPECT_NEAR(static_cast<double>(result), 0.5, 1e-12);
}

TEST(QuantityMath, TanNumeric) {
    vulcan::Quantity<vu::rad> angle{PI / 4.0}; // 45 degrees
    auto result = vulcan::tan(angle);
    EXPECT_NEAR(static_cast<double>(result), 1.0, 1e-12);
}

TEST(QuantityMath, AsinNumeric) {
    vulcan::Quantity<vu::dimensionless> val{0.5};
    auto result = vulcan::asin(val);
    EXPECT_NEAR(result.value(), PI / 6.0, 1e-12);
}

TEST(QuantityMath, AcosNumeric) {
    vulcan::Quantity<vu::dimensionless> val{0.5};
    auto result = vulcan::acos(val);
    EXPECT_NEAR(result.value(), PI / 3.0, 1e-12);
}

TEST(QuantityMath, Atan2DimensionlessNumeric) {
    vulcan::Quantity<vu::dimensionless> y{1.0};
    vulcan::Quantity<vu::dimensionless> x{1.0};
    auto result = vulcan::atan2(y, x);
    EXPECT_NEAR(result.value(), PI / 4.0, 1e-12);
}

TEST(QuantityMath, Atan2SameUnitNumeric) {
    vulcan::Quantity<vu::m> y{100.0};
    vulcan::Quantity<vu::m> x{100.0};
    auto result = vulcan::atan2(y, x);
    EXPECT_NEAR(result.value(), PI / 4.0, 1e-12);
}

// ============================================================================
// Tier 3 — sqrt (numeric, uses mp-units unit algebra)
// ============================================================================

TEST(QuantityMath, SqrtM2ToM) {
    // sqrt(3^2 + 4^2) = sqrt(25) = 5
    using m2 = decltype(vulcan::units::m * vulcan::units::m);
    constexpr auto m2_unit =
        decltype(mp_units::quantity<vu::m, double>{1.0, vu::m} *
                 mp_units::quantity<vu::m, double>{1.0, vu::m})::unit;

    vulcan::Quantity<m2_unit> area{25.0};
    auto side = vulcan::sqrt(area);
    EXPECT_NEAR(side.value(), 5.0, 1e-12);
}

// ============================================================================
// Tier 3 — wrap functions (numeric)
// ============================================================================

TEST(QuantityMath, WrapTo2PiNumeric) {
    vulcan::Quantity<vu::rad> angle{-0.1};
    auto result = vulcan::wrap_to_2pi(angle);
    EXPECT_NEAR(result.value(), 2.0 * PI - 0.1, 1e-12);
}

TEST(QuantityMath, WrapTo2PiZero) {
    vulcan::Quantity<vu::rad> angle{0.0};
    auto result = vulcan::wrap_to_2pi(angle);
    EXPECT_NEAR(result.value(), 0.0, 1e-12);
}

TEST(QuantityMath, WrapTo2Pi_2Pi) {
    vulcan::Quantity<vu::rad> angle{2.0 * PI};
    auto result = vulcan::wrap_to_2pi(angle);
    EXPECT_NEAR(result.value(), 0.0, 1e-12);
}

TEST(QuantityMath, WrapToPiNumeric) {
    // wrap_to_pi(0.0) == 0.0
    vulcan::Quantity<vu::rad> a{0.0};
    EXPECT_NEAR(vulcan::wrap_to_pi(a).value(), 0.0, 1e-12);

    // wrap_to_pi(0.1) == 0.1
    vulcan::Quantity<vu::rad> b{0.1};
    EXPECT_NEAR(vulcan::wrap_to_pi(b).value(), 0.1, 1e-12);

    // wrap_to_pi(pi - 0.1) == pi - 0.1
    vulcan::Quantity<vu::rad> c{PI - 0.1};
    EXPECT_NEAR(vulcan::wrap_to_pi(c).value(), PI - 0.1, 1e-12);

    // wrap_to_pi(pi) == -pi  (from the implementation: wrap_to_2pi(2pi) = 0 ->
    // 0 - pi = -pi)
    vulcan::Quantity<vu::rad> d{PI};
    EXPECT_NEAR(vulcan::wrap_to_pi(d).value(), -PI, 1e-12);

    // wrap_to_pi(-pi) == -pi
    vulcan::Quantity<vu::rad> e{-PI};
    EXPECT_NEAR(vulcan::wrap_to_pi(e).value(), -PI, 1e-12);

    // wrap_to_pi(pi + 0.1) == -pi + 0.1
    vulcan::Quantity<vu::rad> f{PI + 0.1};
    EXPECT_NEAR(vulcan::wrap_to_pi(f).value(), -PI + 0.1, 1e-12);

    // wrap_to_pi(3*pi) == -pi
    vulcan::Quantity<vu::rad> g{3.0 * PI};
    EXPECT_NEAR(vulcan::wrap_to_pi(g).value(), -PI, 1e-12);
}

// ============================================================================
// Symbolic tests
// ============================================================================

TEST(QuantityMath, SinSymbolic) {
    auto x = janus::sym("x");
    vulcan::Quantity<vu::rad, Sym> angle{x};

    auto result = vulcan::sin(angle);

    janus::Function f("sin_test", {x}, {result.value()});
    auto out = f.eval(PI / 6.0);
    EXPECT_NEAR(out(0, 0), 0.5, 1e-12);
}

TEST(QuantityMath, WhereSymbolic) {
    auto a_sym = janus::sym("a");
    auto b_sym = janus::sym("b");

    vulcan::Quantity<vu::m, Sym> a{a_sym};
    vulcan::Quantity<vu::m, Sym> b{b_sym};

    // Condition: a < b
    auto cond = a < b;
    auto result = vulcan::where(cond, a, b);

    janus::Function f("where_test", {a_sym, b_sym}, {result.value()});

    // a < b -> select a
    auto r1 = f.eval(3.0, 7.0);
    EXPECT_NEAR(r1(0, 0), 3.0, 1e-12);

    // a >= b -> select b
    auto r2 = f.eval(9.0, 4.0);
    EXPECT_NEAR(r2(0, 0), 4.0, 1e-12);
}
