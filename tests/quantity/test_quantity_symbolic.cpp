// CasADi symbolic proof-of-concept for vulcan::Quantity<Unit, casadi::MX>
//
// This is the critical go/no-go gate: every test here verifies that
// Quantity arithmetic, comparison, conversion, and dimensionless collapse
// trace through CasADi's symbolic graph and evaluate to correct numerics.
//
// Key finding: mp_units::quantity<Unit, MX> cannot be instantiated because
// casadi::MX::operator== returns MX (not bool), violating
// std::equality_comparable. The Quantity class uses a partial specialization
// that stores MX directly and uses mp-units only for compile-time unit algebra
// and conversion factors.
#include <gtest/gtest.h>

#include <vulcan/quantity/Quantity.hpp>

#include <casadi/casadi.hpp>
#include <janus/math/Arithmetic.hpp>
#include <janus/math/Logic.hpp>

namespace vu = vulcan::units;
using MX = casadi::MX;

// Helper: evaluate a single-output CasADi function and return as double
static double eval1(casadi::Function &f, const std::vector<casadi::DM> &args) {
    std::vector<casadi::DM> result = f(args);
    return static_cast<double>(result.at(0));
}

// =============================================================================
// 1. Symbolic addition: Quantity<m,MX> + Quantity<m,MX>
// =============================================================================

TEST(QuantitySymbolic, Addition) {
    MX x = MX::sym("x");
    MX y = MX::sym("y");

    vulcan::Quantity<vu::m, MX> qx{x};
    vulcan::Quantity<vu::m, MX> qy{y};

    auto qz = qx + qy;
    MX z = qz.value();

    // Build CasADi Function and evaluate
    casadi::Function f("add", {x, y}, {z});
    EXPECT_NEAR(eval1(f, {casadi::DM(3.0), casadi::DM(7.0)}), 10.0, 1e-12);
}

// =============================================================================
// 2. Cross-unit division: Quantity<m,MX> / Quantity<s,MX>  -->  m/s
// =============================================================================

TEST(QuantitySymbolic, CrossUnitDivision) {
    MX d = MX::sym("d");
    MX t = MX::sym("t");

    vulcan::Quantity<vu::m, MX> dist{d};
    vulcan::Quantity<vu::s, MX> time{t};

    auto vel = dist / time; // should produce m/s result
    MX v = vel.value();

    casadi::Function f("div", {d, t}, {v});
    EXPECT_NEAR(eval1(f, {casadi::DM(100.0), casadi::DM(10.0)}), 10.0, 1e-12);
}

// =============================================================================
// 3. Scalar multiplication: Quantity<N,MX> * MX(2.0)
// =============================================================================

TEST(QuantitySymbolic, ScalarMultiply) {
    MX force_sym = MX::sym("F");

    vulcan::Quantity<vu::N, MX> force{force_sym};
    auto doubled = force * MX(2.0);
    MX result_expr = doubled.value();

    casadi::Function f("smul", {force_sym}, {result_expr});
    EXPECT_NEAR(eval1(f, {casadi::DM(5.0)}), 10.0, 1e-12);
}

// =============================================================================
// 4. Comparison + janus::where:  a < b  -->  MX predicate
// =============================================================================

TEST(QuantitySymbolic, ComparisonWithWhere) {
    MX a_sym = MX::sym("a");
    MX b_sym = MX::sym("b");

    vulcan::Quantity<vu::m, MX> a{a_sym};
    vulcan::Quantity<vu::m, MX> b{b_sym};

    // a < b produces an MX predicate (not bool)
    auto cond = a < b;
    MX selected = janus::where(cond, a.value(), b.value());

    casadi::Function f("sel", {a_sym, b_sym}, {selected});

    // Case 1: a < b  -->  select a
    EXPECT_NEAR(eval1(f, {casadi::DM(3.0), casadi::DM(7.0)}), 3.0, 1e-12);

    // Case 2: a >= b  -->  select b
    EXPECT_NEAR(eval1(f, {casadi::DM(9.0), casadi::DM(4.0)}), 4.0, 1e-12);
}

// =============================================================================
// 5. Dimensionless implicit conversion  -->  janus::exp()
// =============================================================================

TEST(QuantitySymbolic, DimensionlessImplicitToExp) {
    MX a_sym = MX::sym("a");
    MX b_sym = MX::sym("b");

    vulcan::Quantity<vu::m, MX> a{a_sym};
    vulcan::Quantity<vu::m, MX> b{b_sym};

    // m / m  -->  dimensionless  -->  implicit conversion to MX
    auto ratio = a / b;
    MX ratio_mx = ratio; // implicit conversion via operator Rep()

    // Feed dimensionless MX into janus::exp
    MX result_expr = janus::exp(ratio_mx);

    casadi::Function f("dimless_exp", {a_sym, b_sym}, {result_expr});

    // exp(2.0/1.0) = exp(2)
    EXPECT_NEAR(eval1(f, {casadi::DM(2.0), casadi::DM(1.0)}), std::exp(2.0),
                1e-10);
}

// =============================================================================
// 6. Unit conversion symbolic: Quantity<ft, MX>(x).in<m>()
// =============================================================================

TEST(QuantitySymbolic, UnitConversionFtToM) {
    MX x = MX::sym("x");

    vulcan::Quantity<vu::ft, MX> alt_ft{x};
    auto alt_m = alt_ft.template in<vu::m>();
    MX result_expr = alt_m.value();

    casadi::Function f("ft2m", {x}, {result_expr});

    // 1000 ft * 0.3048 = 304.8 m
    EXPECT_NEAR(eval1(f, {casadi::DM(1000.0)}), 304.8, 1e-6);
}
