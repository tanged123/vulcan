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

// ============================================================================
// ScatteredTable1D Numeric Tests
// ============================================================================

TEST(ScatteredTable1DTest, BasicInterpolation) {
    // Scattered data from y = x^2
    janus::NumericVector x(10);
    janus::NumericVector y(10);

    // Non-uniform spacing
    x << 0.0, 0.3, 0.7, 1.2, 1.8, 2.5, 3.1, 3.6, 4.2, 5.0;
    for (int i = 0; i < 10; ++i) {
        y(i) = x(i) * x(i);
    }

    vulcan::ScatteredTable1D table(x, y);

    EXPECT_TRUE(table.valid());
    EXPECT_EQ(table.dims(), 1);

    // Test at known points (RBF has some approximation error)
    EXPECT_NEAR(table(2.0), 4.0, 0.5); // Should be close to x^2 = 4
    EXPECT_NEAR(table(3.0), 9.0, 0.5); // Should be close to x^2 = 9
}

TEST(ScatteredTable1DTest, LinearData) {
    // Perfect linear data - should interpolate well
    janus::NumericVector x(5), y(5);
    x << 0.0, 1.0, 2.0, 3.0, 4.0;
    y << 0.0, 2.0, 4.0, 6.0, 8.0; // y = 2x

    vulcan::ScatteredTable1D table(x, y);

    EXPECT_NEAR(table(1.5), 3.0, 0.1);
    EXPECT_NEAR(table(2.5), 5.0, 0.1);
}

TEST(ScatteredTable1DTest, ReconstructionError) {
    janus::NumericVector x(20), y(20);

    for (int i = 0; i < 20; ++i) {
        x(i) = static_cast<double>(i) * 0.5;
        y(i) = std::sin(x(i));
    }

    vulcan::ScatteredTable1D table(x, y, 100); // High resolution

    // Reconstruction error should be small for smooth function
    EXPECT_LT(table.reconstruction_error(), 0.1);
}

TEST(ScatteredTable1DTest, DifferentKernels) {
    janus::NumericVector x(10), y(10);
    for (int i = 0; i < 10; ++i) {
        x(i) = static_cast<double>(i);
        y(i) = std::sin(x(i));
    }

    // Test different RBF kernel types
    vulcan::ScatteredTable1D tps(x, y, 50, vulcan::RBFKernel::ThinPlateSpline);
    EXPECT_TRUE(tps.valid());

    vulcan::ScatteredTable1D mq(x, y, 50, vulcan::RBFKernel::Multiquadric);
    EXPECT_TRUE(mq.valid());

    vulcan::ScatteredTable1D gauss(x, y, 50, vulcan::RBFKernel::Gaussian);
    EXPECT_TRUE(gauss.valid());

    vulcan::ScatteredTable1D linear(x, y, 50, vulcan::RBFKernel::Linear);
    EXPECT_TRUE(linear.valid());
}

// ============================================================================
// ScatteredTable1D Symbolic Tests
// ============================================================================

TEST(ScatteredTable1DTest, SymbolicEvaluation) {
    janus::NumericVector x(10), y(10);
    for (int i = 0; i < 10; ++i) {
        x(i) = static_cast<double>(i);
        y(i) = x(i) * x(i);
    }

    vulcan::ScatteredTable1D table(x, y, 50);

    // Symbolic query
    janus::SymbolicScalar sym_x = janus::SymbolicScalar::sym("x");
    janus::SymbolicScalar result = table(sym_x);

    janus::Function f("test", {sym_x}, {result});

    // Evaluate at x=3
    janus::NumericMatrix output = f.eval(3.0);
    EXPECT_NEAR(output(0, 0), 9.0, 0.5); // Should be close to 3² = 9
}

TEST(ScatteredTable1DTest, SymbolicGradient) {
    janus::NumericVector x(10), y(10);
    for (int i = 0; i < 10; ++i) {
        x(i) = static_cast<double>(i);
        y(i) = x(i) * x(i); // y = x²
    }

    vulcan::ScatteredTable1D table(x, y, 100);

    janus::SymbolicScalar sym_x = janus::SymbolicScalar::sym("x");
    janus::SymbolicScalar result = table(sym_x);

    // Jacobian scalar API
    janus::SymbolicScalar jac = janus::jacobian(result, sym_x);
    janus::Function df("derivative", {sym_x}, {jac});

    // For y=x², dy/dx = 2x, so at x=3, gradient ≈ 6
    janus::NumericMatrix grad = df.eval(3.0);
    EXPECT_NEAR(grad(0, 0), 6.0, 1.0);
}

// ============================================================================
// ScatteredTableND Numeric Tests
// ============================================================================

TEST(ScatteredTableNDTest, Basic2D) {
    // 2D scattered data: z = x + y
    int n = 25;
    janus::NumericMatrix points(n, 2);
    janus::NumericVector values(n);

    int idx = 0;
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            double xi = static_cast<double>(i);
            double yi = static_cast<double>(j);
            points(idx, 0) = xi;
            points(idx, 1) = yi;
            values(idx) = xi + yi;
            ++idx;
        }
    }

    vulcan::ScatteredTableND table(points, values, 30);

    EXPECT_TRUE(table.valid());
    EXPECT_EQ(table.dims(), 2);

    // Test at midpoint
    janus::NumericVector query(2);
    query << 2.5, 2.5;
    EXPECT_NEAR(table(query), 5.0, 1.0); // Some RBF approximation error
}

TEST(ScatteredTableNDTest, ReconstructionError) {
    int n = 16;
    janus::NumericMatrix points(n, 2);
    janus::NumericVector values(n);

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            int idx = i * 4 + j;
            points(idx, 0) = static_cast<double>(i);
            points(idx, 1) = static_cast<double>(j);
            values(idx) = points(idx, 0) + points(idx, 1);
        }
    }

    vulcan::ScatteredTableND table(points, values, 30);

    // RBF has approximation error; for small grids error can be larger
    EXPECT_LT(table.reconstruction_error(), 2.0);
}

// ============================================================================
// ScatteredTableND Symbolic Tests
// ============================================================================

TEST(ScatteredTableNDTest, SymbolicEvaluation2D) {
    int n = 25;
    janus::NumericMatrix points(n, 2);
    janus::NumericVector values(n);

    int idx = 0;
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            points(idx, 0) = static_cast<double>(i);
            points(idx, 1) = static_cast<double>(j);
            values(idx) = points(idx, 0) + points(idx, 1);
            ++idx;
        }
    }

    vulcan::ScatteredTableND table(points, values, 30);

    // Symbolic 2D query
    janus::SymbolicScalar sx = janus::SymbolicScalar::sym("x");
    janus::SymbolicScalar sy = janus::SymbolicScalar::sym("y");
    janus::SymbolicVector query(2);
    query << sx, sy;

    janus::SymbolicScalar result = table(query);
    janus::Function f("interp2d", {sx, sy}, {result});

    // Evaluate at (2, 3)
    janus::NumericMatrix output = f.eval(2.0, 3.0);
    EXPECT_NEAR(output(0, 0), 5.0, 1.0); // 2 + 3 = 5 (RBF tolerance)
}
