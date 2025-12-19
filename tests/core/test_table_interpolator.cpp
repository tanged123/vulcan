// Tests for vulcan::Table1D and vulcan::TableND
#include <gtest/gtest.h>
#include <vulcan/core/TableInterpolator.hpp>

#include <janus/core/Function.hpp>
#include <janus/core/JanusTypes.hpp>
#include <janus/math/AutoDiff.hpp>

// ============================================================================
// Table1D Numeric Tests
// ============================================================================

TEST(Table1DTest, LinearInterpolation) {
    janus::NumericVector x(5), y(5);
    x << 0.0, 1.0, 2.0, 3.0, 4.0;
    y << 0.0, 1.0, 4.0, 9.0, 16.0; // y = x^2

    vulcan::Table1D table(x, y);

    EXPECT_TRUE(table.valid());
    EXPECT_EQ(table.method(), janus::InterpolationMethod::Linear);

    // Exact grid points
    EXPECT_DOUBLE_EQ(table(0.0), 0.0);
    EXPECT_DOUBLE_EQ(table(2.0), 4.0);
    EXPECT_DOUBLE_EQ(table(4.0), 16.0);

    // Interpolated points
    EXPECT_NEAR(table(0.5), 0.5, 1e-10); // Linear between (0,0) and (1,1)
    EXPECT_NEAR(table(1.5), 2.5, 1e-10); // Linear between (1,1) and (2,4)
}

TEST(Table1DTest, BoundsClamping) {
    janus::NumericVector x(3), y(3);
    x << 0.0, 1.0, 2.0;
    y << 10.0, 20.0, 30.0;

    vulcan::Table1D table(x, y);

    // Values outside grid should clamp
    EXPECT_DOUBLE_EQ(table(-1.0), 10.0); // Clamps to min
    EXPECT_DOUBLE_EQ(table(5.0), 30.0);  // Clamps to max
}

TEST(Table1DTest, BatchQuery) {
    janus::NumericVector x(3), y(3);
    x << 0.0, 1.0, 2.0;
    y << 0.0, 10.0, 20.0;

    vulcan::Table1D table(x, y);

    janus::NumericVector queries(4);
    queries << 0.0, 0.5, 1.0, 1.5;

    auto results = table(queries);

    EXPECT_EQ(results.size(), 4);
    EXPECT_NEAR(results(0), 0.0, 1e-10);
    EXPECT_NEAR(results(1), 5.0, 1e-10);
    EXPECT_NEAR(results(2), 10.0, 1e-10);
    EXPECT_NEAR(results(3), 15.0, 1e-10);
}

TEST(Table1DTest, BSplineInterpolation) {
    janus::NumericVector x(5), y(5);
    x << 0.0, 1.0, 2.0, 3.0, 4.0;
    y << 0.0, 1.0, 4.0, 9.0, 16.0;

    vulcan::Table1D table(x, y, janus::InterpolationMethod::BSpline);

    EXPECT_TRUE(table.valid());
    EXPECT_EQ(table.method(), janus::InterpolationMethod::BSpline);

    // BSpline is smoother but may not pass exactly through points
    EXPECT_NEAR(table(2.0), 4.0, 0.5); // Allow some deviation
}

// ============================================================================
// Table1D Symbolic Tests
// ============================================================================

TEST(Table1DTest, SymbolicEvaluation) {
    janus::NumericVector x(3), y(3);
    x << 0.0, 5.0, 10.0;
    y << 288.0, 256.0, 223.0; // Temperature lookup

    vulcan::Table1D table(x, y);

    // Create symbolic variable
    janus::SymbolicScalar alt = janus::SymbolicScalar::sym("alt");

    // Evaluate symbolically
    janus::SymbolicScalar temp = table(alt);

    // Create function for numeric evaluation
    janus::Function f("temp_lookup", {alt}, {temp});

    // Test at grid points - f.eval() returns first output as matrix
    janus::NumericMatrix result = f.eval(2.5);
    EXPECT_NEAR(result(0, 0), 272.0, 1e-6); // Midpoint between 288 and 256
}

TEST(Table1DTest, SymbolicDerivative) {
    janus::NumericVector x(3), y(3);
    x << 0.0, 1.0, 2.0;
    y << 0.0, 10.0, 20.0; // Linear: dy/dx = 10

    vulcan::Table1D table(x, y);

    janus::SymbolicScalar query = janus::SymbolicScalar::sym("x");
    janus::SymbolicScalar result = table(query);

    // Compute Jacobian using scalar API
    janus::SymbolicScalar jac = janus::jacobian(result, query);

    janus::Function deriv("derivative", {query}, {jac});

    // Derivative should be constant = 10 for linear interp
    janus::NumericMatrix grad = deriv.eval(0.5);
    EXPECT_NEAR(grad(0, 0), 10.0, 1e-6);
}

// ============================================================================
// TableND Numeric Tests
// ============================================================================

TEST(TableNDTest, TwoDimensional) {
    // 2D grid: x = [0, 1], y = [0, 1]
    janus::NumericVector x_pts(2), y_pts(2);
    x_pts << 0.0, 1.0;
    y_pts << 0.0, 1.0;

    // z = x + y (Fortran order: z00, z10, z01, z11)
    janus::NumericVector values(4);
    values << 0.0, 1.0, 1.0, 2.0; // z(0,0), z(1,0), z(0,1), z(1,1)

    vulcan::TableND table({x_pts, y_pts}, values);

    EXPECT_TRUE(table.valid());
    EXPECT_EQ(table.dims(), 2);

    // Query at corners
    janus::NumericVector q1(2), q2(2), q3(2);
    q1 << 0.0, 0.0;
    q2 << 1.0, 1.0;
    q3 << 0.5, 0.5;

    EXPECT_NEAR(table(q1), 0.0, 1e-10);
    EXPECT_NEAR(table(q2), 2.0, 1e-10);
    EXPECT_NEAR(table(q3), 1.0, 1e-10); // Center: average of all
}

TEST(TableNDTest, ThreeDimensional) {
    // 3D grid: 2x2x2
    janus::NumericVector x(2), y(2), z(2);
    x << 0.0, 1.0;
    y << 0.0, 1.0;
    z << 0.0, 1.0;

    // v = x + y + z (8 values in Fortran order)
    janus::NumericVector values(8);
    values << 0.0, 1.0, 1.0, 2.0, 1.0, 2.0, 2.0, 3.0;

    vulcan::TableND table({x, y, z}, values);

    EXPECT_TRUE(table.valid());
    EXPECT_EQ(table.dims(), 3);

    janus::NumericVector query(3);
    query << 1.0, 1.0, 1.0;
    EXPECT_NEAR(table(query), 3.0, 1e-10);

    query << 0.5, 0.5, 0.5;
    EXPECT_NEAR(table(query), 1.5, 1e-10); // Center
}

TEST(TableNDTest, SymbolicEvaluationND) {
    janus::NumericVector x_pts(2), y_pts(2);
    x_pts << 0.0, 10.0;
    y_pts << 0.0, 10.0;

    janus::NumericVector values(4);
    values << 0.0, 10.0, 10.0, 20.0; // z = x + y

    vulcan::TableND table({x_pts, y_pts}, values);

    // Symbolic query
    janus::SymbolicScalar sx = janus::SymbolicScalar::sym("x");
    janus::SymbolicScalar sy = janus::SymbolicScalar::sym("y");
    janus::SymbolicVector query(2);
    query << sx, sy;

    janus::SymbolicScalar result = table(query);

    // Create function
    janus::Function f("interp2d", {sx, sy}, {result});

    // Evaluate at (5.0, 5.0)
    janus::NumericMatrix output = f.eval(5.0, 5.0);

    EXPECT_NEAR(output(0, 0), 10.0, 1e-6);
}
