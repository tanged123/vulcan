// Tests for vulcan::Quantity<Unit, Rep>
#include <cmath>
#include <gtest/gtest.h>
#include <vulcan/quantity/Quantity.hpp>

namespace vu = vulcan::units;

// =============================================================================
// Construction
// =============================================================================

TEST(Quantity, DefaultConstructionIsZero) {
    vulcan::Quantity<vu::m> q;
    EXPECT_DOUBLE_EQ(q.value(), 0.0);
}

TEST(Quantity, ExplicitConstruction) {
    vulcan::Quantity<vu::m> q{42.0};
    EXPECT_DOUBLE_EQ(q.value(), 42.0);
}

TEST(Quantity, ValueReturnsInDeclaredUnit) {
    // 1000 ft should remain 1000 when queried — not converted to metres.
    vulcan::Quantity<vu::ft> altitude{1000.0};
    EXPECT_DOUBLE_EQ(altitude.value(), 1000.0);
}

// =============================================================================
// Unit conversion with .in<>()
// =============================================================================

TEST(Quantity, ConvertFeetToMetres) {
    vulcan::Quantity<vu::ft> alt_ft{1000.0};
    auto alt_m = alt_ft.in<vu::m>();
    EXPECT_NEAR(alt_m.value(), 304.8, 1e-6);
}

TEST(Quantity, ConvertDegreesToRadians) {
    vulcan::Quantity<vu::deg> angle_deg{180.0};
    auto angle_rad = angle_deg.in<vu::rad>();
    EXPECT_NEAR(angle_rad.value(), M_PI, 1e-10);
}

TEST(Quantity, RoundTripConversion) {
    vulcan::Quantity<vu::m> original{123.456};
    auto in_ft = original.in<vu::ft>();
    auto back = in_ft.in<vu::m>();
    EXPECT_NEAR(back.value(), 123.456, 1e-9);
}

// =============================================================================
// Same-unit arithmetic
// =============================================================================

TEST(Quantity, Addition) {
    vulcan::Quantity<vu::m> a{10.0};
    vulcan::Quantity<vu::m> b{20.0};
    auto c = a + b;
    EXPECT_DOUBLE_EQ(c.value(), 30.0);
}

TEST(Quantity, Subtraction) {
    vulcan::Quantity<vu::m> a{30.0};
    vulcan::Quantity<vu::m> b{12.0};
    auto c = a - b;
    EXPECT_DOUBLE_EQ(c.value(), 18.0);
}

TEST(Quantity, UnaryNegation) {
    vulcan::Quantity<vu::m> a{5.0};
    auto b = -a;
    EXPECT_DOUBLE_EQ(b.value(), -5.0);
}

TEST(Quantity, PlusEquals) {
    vulcan::Quantity<vu::m> a{10.0};
    vulcan::Quantity<vu::m> b{5.0};
    a += b;
    EXPECT_DOUBLE_EQ(a.value(), 15.0);
}

TEST(Quantity, MinusEquals) {
    vulcan::Quantity<vu::m> a{10.0};
    vulcan::Quantity<vu::m> b{3.0};
    a -= b;
    EXPECT_DOUBLE_EQ(a.value(), 7.0);
}

// =============================================================================
// Cross-unit multiplication and division
// =============================================================================

TEST(Quantity, CrossUnitMultiplication) {
    vulcan::Quantity<vu::m> length{3.0};
    vulcan::Quantity<vu::m> width{4.0};
    auto area = length * width;
    // Result should be in m^2
    EXPECT_DOUBLE_EQ(area.value(), 12.0);
}

TEST(Quantity, CrossUnitDivision) {
    vulcan::Quantity<vu::m> distance{100.0};
    vulcan::Quantity<vu::s> time{10.0};
    auto speed = distance / time;
    // Result should be in m/s
    EXPECT_DOUBLE_EQ(speed.value(), 10.0);
}

// =============================================================================
// Scalar multiplication and division
// =============================================================================

TEST(Quantity, ScalarMultiplyRight) {
    vulcan::Quantity<vu::m> a{5.0};
    auto b = a * 3.0;
    EXPECT_DOUBLE_EQ(b.value(), 15.0);
}

TEST(Quantity, ScalarMultiplyLeft) {
    vulcan::Quantity<vu::m> a{5.0};
    auto b = 3.0 * a;
    EXPECT_DOUBLE_EQ(b.value(), 15.0);
}

TEST(Quantity, ScalarDivide) {
    vulcan::Quantity<vu::m> a{15.0};
    auto b = a / 3.0;
    EXPECT_DOUBLE_EQ(b.value(), 5.0);
}

// =============================================================================
// Comparisons
// =============================================================================

TEST(Quantity, EqualityTrue) {
    vulcan::Quantity<vu::m> a{5.0};
    vulcan::Quantity<vu::m> b{5.0};
    EXPECT_TRUE(a == b);
}

TEST(Quantity, EqualityFalse) {
    vulcan::Quantity<vu::m> a{5.0};
    vulcan::Quantity<vu::m> b{6.0};
    EXPECT_FALSE(a == b);
}

TEST(Quantity, Inequality) {
    vulcan::Quantity<vu::m> a{5.0};
    vulcan::Quantity<vu::m> b{6.0};
    EXPECT_TRUE(a != b);
}

TEST(Quantity, LessThan) {
    vulcan::Quantity<vu::m> a{5.0};
    vulcan::Quantity<vu::m> b{6.0};
    EXPECT_TRUE(a < b);
    EXPECT_FALSE(b < a);
}

TEST(Quantity, GreaterThan) {
    vulcan::Quantity<vu::m> a{6.0};
    vulcan::Quantity<vu::m> b{5.0};
    EXPECT_TRUE(a > b);
    EXPECT_FALSE(b > a);
}

TEST(Quantity, LessOrEqual) {
    vulcan::Quantity<vu::m> a{5.0};
    vulcan::Quantity<vu::m> b{5.0};
    vulcan::Quantity<vu::m> c{6.0};
    EXPECT_TRUE(a <= b);
    EXPECT_TRUE(a <= c);
    EXPECT_FALSE(c <= a);
}

TEST(Quantity, GreaterOrEqual) {
    vulcan::Quantity<vu::m> a{5.0};
    vulcan::Quantity<vu::m> b{5.0};
    vulcan::Quantity<vu::m> c{4.0};
    EXPECT_TRUE(a >= b);
    EXPECT_TRUE(a >= c);
    EXPECT_FALSE(c >= a);
}

// =============================================================================
// Dimensionless
// =============================================================================

TEST(Quantity, DimensionlessImplicitConversion) {
    vulcan::Quantity<vu::dimensionless> ratio{0.85};
    double val = ratio; // implicit conversion
    EXPECT_DOUBLE_EQ(val, 0.85);
}

TEST(Quantity, DimensionlessFromDivision) {
    vulcan::Quantity<vu::m> a{10.0};
    vulcan::Quantity<vu::m> b{5.0};
    auto ratio = a / b;
    // m / m should produce a dimensionless quantity
    double val = ratio.value();
    EXPECT_DOUBLE_EQ(val, 2.0);
}
