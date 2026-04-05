// Tests for Eigen NumTraits specialization of vulcan::Quantity
//
// Verifies that Vec3<Quantity<m>> (and symbolic variants) work correctly
// with basic element access and same-unit arithmetic.
#include <gtest/gtest.h>

#include <vulcan/quantity/QuantityEigen.hpp>

#include <janus/core/Function.hpp>
#include <janus/core/JanusTypes.hpp>

namespace vu = vulcan::units;

using QM = vulcan::Quantity<vu::m>;
using QFt = vulcan::Quantity<vu::ft>;

// =============================================================================
// 1. Vec3<Quantity<m>> construction and element access
// =============================================================================

TEST(QuantityEigen, Vec3ConstructionAndAccess) {
    janus::Vec3<QM> v;
    v(0) = QM{1.0};
    v(1) = QM{2.0};
    v(2) = QM{3.0};

    EXPECT_DOUBLE_EQ(v(0).value(), 1.0);
    EXPECT_DOUBLE_EQ(v(1).value(), 2.0);
    EXPECT_DOUBLE_EQ(v(2).value(), 3.0);
}

// =============================================================================
// 2. Vec3<Quantity<m>> addition
// =============================================================================

TEST(QuantityEigen, Vec3Addition) {
    janus::Vec3<QM> a;
    a(0) = QM{1.0};
    a(1) = QM{2.0};
    a(2) = QM{3.0};

    janus::Vec3<QM> b;
    b(0) = QM{10.0};
    b(1) = QM{20.0};
    b(2) = QM{30.0};

    janus::Vec3<QM> c = a + b;

    EXPECT_DOUBLE_EQ(c(0).value(), 11.0);
    EXPECT_DOUBLE_EQ(c(1).value(), 22.0);
    EXPECT_DOUBLE_EQ(c(2).value(), 33.0);
}

// =============================================================================
// 3. Vec3<Quantity<m>> subtraction
// =============================================================================

TEST(QuantityEigen, Vec3Subtraction) {
    janus::Vec3<QM> a;
    a(0) = QM{10.0};
    a(1) = QM{20.0};
    a(2) = QM{30.0};

    janus::Vec3<QM> b;
    b(0) = QM{1.0};
    b(1) = QM{2.0};
    b(2) = QM{3.0};

    janus::Vec3<QM> c = a - b;

    EXPECT_DOUBLE_EQ(c(0).value(), 9.0);
    EXPECT_DOUBLE_EQ(c(1).value(), 18.0);
    EXPECT_DOUBLE_EQ(c(2).value(), 27.0);
}

// =============================================================================
// 4. Symbolic Vec3<Quantity<m, SymbolicScalar>> addition
// =============================================================================

TEST(QuantityEigen, SymbolicVec3Addition) {
    using Sym = janus::SymbolicScalar;
    using QMS = vulcan::Quantity<vu::m, Sym>;

    // Create 6 symbolic scalars for two 3-vectors
    auto ax = janus::sym("ax");
    auto ay = janus::sym("ay");
    auto az = janus::sym("az");
    auto bx = janus::sym("bx");
    auto by = janus::sym("by");
    auto bz = janus::sym("bz");

    janus::Vec3<QMS> a;
    a(0) = QMS{ax};
    a(1) = QMS{ay};
    a(2) = QMS{az};

    janus::Vec3<QMS> b;
    b(0) = QMS{bx};
    b(1) = QMS{by};
    b(2) = QMS{bz};

    janus::Vec3<QMS> c = a + b;

    // Evaluate each component through janus::Function
    janus::Function fx("fx", {ax, ay, az, bx, by, bz}, {c(0).value()});
    janus::Function fy("fy", {ax, ay, az, bx, by, bz}, {c(1).value()});
    janus::Function fz("fz", {ax, ay, az, bx, by, bz}, {c(2).value()});

    auto rx = fx.eval(1.0, 2.0, 3.0, 10.0, 20.0, 30.0);
    auto ry = fy.eval(1.0, 2.0, 3.0, 10.0, 20.0, 30.0);
    auto rz = fz.eval(1.0, 2.0, 3.0, 10.0, 20.0, 30.0);

    EXPECT_NEAR(rx(0, 0), 11.0, 1e-12);
    EXPECT_NEAR(ry(0, 0), 22.0, 1e-12);
    EXPECT_NEAR(rz(0, 0), 33.0, 1e-12);
}
