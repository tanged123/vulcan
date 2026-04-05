// CasADi symbolic proof-of-concept for vulcan::Quantity with SymbolicScalar
//
// This is the critical go/no-go gate: every test verifies that Quantity
// arithmetic, comparison, conversion, and dimensionless collapse trace
// through the symbolic graph and evaluate to correct numerics.
//
// Uses Janus APIs (janus::sym, janus::Function, janus::SymbolicScalar)
// — never raw CasADi types directly.
#include <gtest/gtest.h>

#include <vulcan/quantity/Quantity.hpp>

#include <janus/core/Function.hpp>
#include <janus/core/JanusTypes.hpp>
#include <janus/math/Arithmetic.hpp>
#include <janus/math/Logic.hpp>

namespace vu = vulcan::units;
using Sym = janus::SymbolicScalar;

// =============================================================================
// 1. Symbolic addition: Quantity<m, Sym> + Quantity<m, Sym>
// =============================================================================

TEST(QuantitySymbolic, Addition) {
    auto x = janus::sym("x");
    auto y = janus::sym("y");

    vulcan::Quantity<vu::m, Sym> qx{x};
    vulcan::Quantity<vu::m, Sym> qy{y};

    auto qz = qx + qy;

    janus::Function f("add", {x, y}, {qz.value()});
    auto result = f.eval(3.0, 7.0);
    EXPECT_NEAR(result(0, 0), 10.0, 1e-12);
}

// =============================================================================
// 2. Cross-unit division: Quantity<m, Sym> / Quantity<s, Sym> → m/s
// =============================================================================

TEST(QuantitySymbolic, CrossUnitDivision) {
    auto d = janus::sym("d");
    auto t = janus::sym("t");

    vulcan::Quantity<vu::m, Sym> dist{d};
    vulcan::Quantity<vu::s, Sym> time{t};

    auto vel = dist / time;

    janus::Function f("div", {d, t}, {vel.value()});
    auto result = f.eval(100.0, 10.0);
    EXPECT_NEAR(result(0, 0), 10.0, 1e-12);
}

// =============================================================================
// 3. Scalar multiplication: Quantity<N, Sym> * Sym(2.0)
// =============================================================================

TEST(QuantitySymbolic, ScalarMultiply) {
    auto force_sym = janus::sym("F");

    vulcan::Quantity<vu::N, Sym> force{force_sym};
    auto doubled = force * Sym(2.0);

    janus::Function f("smul", {force_sym}, {doubled.value()});
    auto result = f.eval(5.0);
    EXPECT_NEAR(result(0, 0), 10.0, 1e-12);
}

// =============================================================================
// 4. Comparison + janus::where: a < b → symbolic predicate
// =============================================================================

TEST(QuantitySymbolic, ComparisonWithWhere) {
    auto a_sym = janus::sym("a");
    auto b_sym = janus::sym("b");

    vulcan::Quantity<vu::m, Sym> a{a_sym};
    vulcan::Quantity<vu::m, Sym> b{b_sym};

    auto cond = a < b;
    Sym selected = janus::where(cond, a.value(), b.value());

    janus::Function f("sel", {a_sym, b_sym}, {selected});

    // a < b → select a
    auto r1 = f.eval(3.0, 7.0);
    EXPECT_NEAR(r1(0, 0), 3.0, 1e-12);

    // a >= b → select b
    auto r2 = f.eval(9.0, 4.0);
    EXPECT_NEAR(r2(0, 0), 4.0, 1e-12);
}

// =============================================================================
// 5. Dimensionless implicit conversion → janus::exp()
// =============================================================================

TEST(QuantitySymbolic, DimensionlessImplicitToExp) {
    auto a_sym = janus::sym("a");
    auto b_sym = janus::sym("b");

    vulcan::Quantity<vu::m, Sym> a{a_sym};
    vulcan::Quantity<vu::m, Sym> b{b_sym};

    // m / m → dimensionless → implicit conversion to Sym
    auto ratio = a / b;
    Sym ratio_sym = ratio;

    // Feed dimensionless into janus::exp
    Sym result_expr = janus::exp(ratio_sym);

    janus::Function f("dimless_exp", {a_sym, b_sym}, {result_expr});
    auto result = f.eval(2.0, 1.0);
    EXPECT_NEAR(result(0, 0), std::exp(2.0), 1e-10);
}

// =============================================================================
// 6. Unit conversion symbolic: Quantity<ft, Sym>(x).in<m>()
// =============================================================================

TEST(QuantitySymbolic, UnitConversionFtToM) {
    auto x = janus::sym("x");

    vulcan::Quantity<vu::ft, Sym> alt_ft{x};
    auto alt_m = alt_ft.template in<vu::m>();

    janus::Function f("ft2m", {x}, {alt_m.value()});
    auto result = f.eval(1000.0);
    EXPECT_NEAR(result(0, 0), 304.8, 1e-6);
}
