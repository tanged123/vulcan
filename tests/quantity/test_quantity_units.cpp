// Tests for vulcan::units vocabulary — compile-time verification via
// static_assert
#include <gtest/gtest.h>
#include <vulcan/quantity/Units.hpp>

namespace vu = vulcan::units;

// =============================================================================
// Base SI
// =============================================================================
static_assert(vu::m == mp_units::si::metre);
static_assert(vu::kg == mp_units::si::kilogram);
static_assert(vu::s == mp_units::si::second);
static_assert(vu::K == mp_units::si::kelvin);
static_assert(vu::rad == mp_units::si::radian);

// =============================================================================
// Derived SI
// =============================================================================
static_assert(vu::N == mp_units::si::newton);
static_assert(vu::Pa == mp_units::si::pascal);
static_assert(vu::J == mp_units::si::joule);
static_assert(vu::W == mp_units::si::watt);
static_assert(vu::Hz == mp_units::si::hertz);

// =============================================================================
// Aerospace / non-SI
// =============================================================================
static_assert(vu::ft == mp_units::international::foot);
static_assert(vu::deg == mp_units::non_si::degree);
static_assert(vu::nmi == mp_units::international::nautical_mile);
static_assert(vu::kn == mp_units::international::knot);
static_assert(vu::lb == mp_units::international::pound);
static_assert(vu::lbf == mp_units::international::pound_force);

// =============================================================================
// Dimensionless
// =============================================================================
static_assert(vu::dimensionless == mp_units::one);

// =============================================================================
// Long-form aliases
// =============================================================================
static_assert(vu::metre == vu::m);
static_assert(vu::kilogram == vu::kg);
static_assert(vu::second == vu::s);
static_assert(vu::kelvin == vu::K);
static_assert(vu::radian == vu::rad);
static_assert(vu::newton == vu::N);
static_assert(vu::pascal == vu::Pa);
static_assert(vu::joule == vu::J);
static_assert(vu::watt == vu::W);
static_assert(vu::hertz == vu::Hz);
static_assert(vu::foot == vu::ft);
static_assert(vu::degree == vu::deg);
static_assert(vu::knot == vu::kn);

// A trivial runtime test so GTest registers at least one test from this file.
TEST(QuantityUnits, StaticAssertsPass) {
    SUCCEED() << "All unit vocabulary static_asserts passed";
}
