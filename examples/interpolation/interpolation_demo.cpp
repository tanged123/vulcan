// Table Interpolation Demo
// Demonstrates gridded (Table1D, TableND) and scattered (ScatteredTable1D,
// ScatteredTableND) interpolation
#include <vulcan/vulcan.hpp>

#include <iomanip>
#include <iostream>

using namespace vulcan;

int main() {
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "=== Vulcan Table Interpolation Demo ===\n\n";

    // =========================================================================
    // 1. Table1D - Simple 1D gridded interpolation
    // =========================================================================
    std::cout << "--- Table1D (Altitude vs Temperature) ---\n";
    {
        janus::NumericVector alt(5), temp(5);
        alt << 0, 5, 10, 15, 20;         // km
        temp << 288, 256, 223, 217, 217; // K

        Table1D table(alt, temp);

        std::cout << "Grid: alt = [0, 5, 10, 15, 20] km\n";
        std::cout << "Values: temp = [288, 256, 223, 217, 217] K\n\n";

        // Query at various altitudes
        for (double h : {2.5, 7.5, 12.0}) {
            std::cout << "  T(" << h << " km) = " << table(h) << " K\n";
        }
        std::cout << "\n";
    }

    // =========================================================================
    // 2. TableND - 2D gridded interpolation (aero coefficients)
    // =========================================================================
    std::cout << "--- TableND (Mach x Alpha -> CL) ---\n";
    {
        janus::NumericVector mach(3), alpha(4);
        mach << 0.6, 0.8, 1.0;
        alpha << 0, 5, 10, 15;

        // CL values in Fortran order (Mach varies fastest)
        janus::NumericVector cl(12);
        cl << 0.0, 0.05, 0.10, // alpha=0
            0.5, 0.55, 0.60,   // alpha=5
            1.0, 1.05, 1.10,   // alpha=10
            1.4, 1.45, 1.50;   // alpha=15

        TableND table({mach, alpha}, cl);

        std::cout << "Grid: Mach = [0.6, 0.8, 1.0], Alpha = [0, 5, 10, 15]\n";
        std::cout << "Dimensions: " << table.dims() << "\n\n";

        // Query at test points
        janus::NumericVector q1(2), q2(2);
        q1 << 0.7, 7.5;
        q2 << 0.9, 12.0;

        std::cout << "  CL(M=0.7, α=7.5°) = " << table(q1) << "\n";
        std::cout << "  CL(M=0.9, α=12.0°) = " << table(q2) << "\n\n";
    }

    // =========================================================================
    // 3. ScatteredTable1D - Non-uniform 1D data
    // =========================================================================
    std::cout << "--- ScatteredTable1D (Non-uniform Points) ---\n";
    {
        // Non-uniformly spaced data (like wind tunnel test points)
        janus::NumericVector x(8), y(8);
        x << 0.0, 0.3, 0.7, 1.2, 2.0, 2.8, 3.5, 4.0;
        for (int i = 0; i < 8; ++i) {
            y(i) = std::sin(x(i)); // Sample sin(x)
        }

        ScatteredTable1D table(x, y, 50);

        std::cout << "Data points: " << x.transpose() << "\n";
        std::cout << "RMS reconstruction error: "
                  << table.reconstruction_error() << "\n\n";

        // Query at test points
        for (double t : {0.5, 1.5, 2.5}) {
            double interp = table(t);
            double exact = std::sin(t);
            std::cout << "  f(" << t << ") = " << interp << " (exact: " << exact
                      << ")\n";
        }
        std::cout << "\n";
    }

    // =========================================================================
    // 4. ScatteredTableND - 2D unstructured data
    // =========================================================================
    std::cout << "--- ScatteredTableND (2D Scattered Points) ---\n";
    {
        // 2D scattered data: z = x + y
        int n = 25;
        janus::NumericMatrix points(n, 2);
        janus::NumericVector values(n);

        // 5x5 grid pattern
        int idx = 0;
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                double x = static_cast<double>(i);
                double y = static_cast<double>(j);
                points(idx, 0) = x;
                points(idx, 1) = y;
                values(idx) = x + y; // z = x + y
                ++idx;
            }
        }

        ScatteredTableND table(points, values, 30);

        std::cout << "Function: z = x + y  (25 scattered samples)\n";
        std::cout << "Dimensions: " << table.dims() << "\n";
        std::cout << "Reconstruction error: " << table.reconstruction_error()
                  << "\n\n";

        // Query at center point
        janus::NumericVector q(2);
        q << 2.5, 2.5;
        double result = table(q);
        double expected = 5.0; // 2.5 + 2.5
        std::cout << "  f(2.5, 2.5) = " << result << " (expected: " << expected
                  << ")\n\n";
    }

    // =========================================================================
    // 5. Symbolic Mode - Optimization-ready
    // =========================================================================
    std::cout << "--- Symbolic Mode ---\n";
    {
        // Create a simple table
        janus::NumericVector x(5), y(5);
        x << 0, 1, 2, 3, 4;
        y << 0, 1, 4, 9, 16; // y = x²

        Table1D table(x, y);

        // Symbolic query
        SymbolicScalar x_sym = sym("x");
        SymbolicScalar y_sym = table(x_sym);

        // Build function
        janus::Function f("table_lookup", {x_sym}, {y_sym});

        // Evaluate
        auto result = f(1.5);
        std::cout << "table(1.5) = " << double(result[0](0, 0)) << "\n";

        // Compute gradient
        SymbolicScalar grad = janus::jacobian(y_sym, x_sym);
        janus::Function df("table_derivative", {x_sym}, {grad});

        auto slope = df(1.5);
        std::cout << "d/dx table(1.5) = " << double(slope[0](0, 0)) << "\n";
    }

    std::cout << "\n=== Demo Complete ===\n";
    return 0;
}
