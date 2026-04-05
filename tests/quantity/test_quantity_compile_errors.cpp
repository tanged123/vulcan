// Compile-time safety verification for vulcan::Quantity<Unit, Rep>.
//
// Positive tests verify that valid operations compile and produce correct
// results. Commented-out lines document operations that SHOULD fail to compile,
// serving as a specification of the type system's safety guarantees.
#include <gtest/gtest.h>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/QuantityMath.hpp>
#include <vulcan/quantity/Units.hpp>

namespace vulcan::tests {
using namespace vulcan::units;

// =============================================================================
// Valid operations that SHOULD compile
// =============================================================================

TEST(QuantityCompileTime, ValidSameUnitArithmetic) {
    Quantity<m> a(1.0), b(2.0);
    auto c = a + b; // same unit addition: OK
    auto d = a - b; // same unit subtraction: OK
    (void)c;
    (void)d;
}

TEST(QuantityCompileTime, ValidCrossUnit) {
    Quantity<m> dist(100.0);
    Quantity<s> time(10.0);
    auto speed = dist / time; // m/s: OK
    (void)speed;

    Quantity<N> force(50.0);
    auto work = force * dist; // N*m = J: OK
    (void)work;
}

TEST(QuantityCompileTime, DimensionlessConversion) {
    Quantity<m> a(10.0), b(5.0);
    double ratio = a / b; // dimensionless -> double: OK
    EXPECT_DOUBLE_EQ(ratio, 2.0);
}

TEST(QuantityCompileTime, CrossUnitResultTypes) {
    Quantity<m> dist(100.0);
    Quantity<s> time(10.0);
    auto speed = dist / time;
    EXPECT_DOUBLE_EQ(speed.value(), 10.0);
}

TEST(QuantityCompileTime, ValidScalarMultiplication) {
    Quantity<m> dist(5.0);
    auto doubled = dist * 2.0;
    auto tripled = 3.0 * dist;
    EXPECT_DOUBLE_EQ(doubled.value(), 10.0);
    EXPECT_DOUBLE_EQ(tripled.value(), 15.0);
}

TEST(QuantityCompileTime, ValidScalarDivision) {
    Quantity<m> dist(10.0);
    auto half = dist / 2.0;
    EXPECT_DOUBLE_EQ(half.value(), 5.0);
}

TEST(QuantityCompileTime, ValidTrigOnRadians) {
    Quantity<rad> angle(0.5);
    auto sine = vulcan::sin(angle);
    auto cosine = vulcan::cos(angle);
    auto tangent = vulcan::tan(angle);
    (void)sine;
    (void)cosine;
    (void)tangent;
}

TEST(QuantityCompileTime, ValidInverseTrigOnDimensionless) {
    Quantity<dimensionless> x(0.5);
    auto angle_asin = vulcan::asin(x);
    auto angle_acos = vulcan::acos(x);
    (void)angle_asin;
    (void)angle_acos;
}

TEST(QuantityCompileTime, ValidAtan2SameUnit) {
    Quantity<m> y(3.0), x(4.0);
    auto angle = vulcan::atan2(y, x);
    (void)angle;
}

TEST(QuantityCompileTime, ValidUnitConversion) {
    Quantity<ft> alt_ft(1000.0);
    auto alt_m = alt_ft.in<m>();
    EXPECT_NEAR(alt_m.value(), 304.8, 1e-6);
}

TEST(QuantityCompileTime, ValidCompoundUnitArithmetic) {
    Quantity<mps> v1(10.0), v2(5.0);
    auto sum = v1 + v2;
    EXPECT_DOUBLE_EQ(sum.value(), 15.0);
}

TEST(QuantityCompileTime, ValidNegation) {
    Quantity<K> temp(300.0);
    auto neg = -temp;
    EXPECT_DOUBLE_EQ(neg.value(), -300.0);
}

TEST(QuantityCompileTime, ValidComparisonOperators) {
    Quantity<m> a(1.0), b(2.0);
    EXPECT_TRUE(a < b);
    EXPECT_TRUE(b > a);
    EXPECT_TRUE(a <= b);
    EXPECT_TRUE(b >= a);
    EXPECT_TRUE(a != b);
    EXPECT_FALSE(a == b);
}

// =============================================================================
// Operations that should NOT compile -- documented as comments.
// Uncomment any single line to verify it produces a compile error.
// =============================================================================

// --- Mismatched-unit addition/subtraction ---
// Quantity<m> a(1.0); Quantity<K> b(2.0); auto c = a + b;   // ERROR: m + K
// Quantity<m> a(1.0); Quantity<s> b(2.0); auto c = a - b;   // ERROR: m - s
// Quantity<ft> a(1.0); Quantity<K> b(2.0); auto c = a + b;  // ERROR: ft + K

// --- Implicit conversion of non-dimensionless to double ---
// Quantity<m> a(1.0); double x = a;                          // ERROR: m is not
// dimensionless Quantity<K> t(300.0); double x = t;                        //
// ERROR: K is not dimensionless Quantity<rad> a(1.0); double x = a; // ERROR:
// rad is not dimensionless

// --- Trig functions require radians ---
// vulcan::sin(Quantity<m>(1.0));                              // ERROR: sin
// expects rad, not m vulcan::cos(Quantity<K>(1.0)); // ERROR: cos expects rad,
// not K vulcan::tan(Quantity<deg>(1.0));                            // ERROR:
// tan expects rad, not deg

// --- Inverse trig functions require dimensionless ---
// vulcan::asin(Quantity<m>(0.5));                             // ERROR: asin
// expects dimensionless vulcan::acos(Quantity<rad>(0.5)); // ERROR: acos
// expects dimensionless

} // namespace vulcan::tests
