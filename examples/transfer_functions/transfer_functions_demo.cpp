/**
 * @file transfer_functions_demo.cpp
 * @brief Demonstrates the vulcan::tf module for transfer functions.
 *
 * Shows stateless first/second order systems, discretization, and nonlinear
 * elements.
 */

#include <iomanip>
#include <iostream>
#include <vulcan/transfer_functions/TransferFunctions.hpp>

using namespace vulcan::tf;

int main() {
    std::cout << "=== Transfer Functions Demo ===\n\n";

    // First-order system
    auto fo = first_order<double>(0.1, 1.0, 0.01);
    std::cout << "First-order: tau=0.1s, K=1.0, dt=10ms\n";
    std::cout << "  Coefficients: a=" << fo.a << ", b=" << fo.b << "\n\n";

    // Simulate step response (stateless - caller manages state)
    double y = 0.0;
    for (int i = 0; i < 100; ++i) {
        y = first_order_step(fo, y, 1.0);
    }
    std::cout << "  After 1s step: y = " << y << "\n\n";

    // Second-order system
    auto so = second_order<double>(25.0, 0.7, 1.0, 0.001);
    auto [tr, ts, Mp] = second_order_characteristics(25.0, 0.7);
    std::cout << "Second-order: ωn=25rad/s, ζ=0.7\n";
    std::cout << "  Rise time: " << tr * 1000 << "ms, Settling: " << ts * 1000
              << "ms\n";
    std::cout << "  Overshoot: " << Mp << "%\n\n";

    // Nonlinear elements
    std::cout << "Nonlinear functions (stateless):\n";
    std::cout << "  rate_limit(0, 100, 10, 0.1) = "
              << rate_limit(0.0, 100.0, 10.0, 0.1) << "\n";
    std::cout << "  saturate(50, -30, 30) = " << saturate(50.0, -30.0, 30.0)
              << "\n";
    std::cout << "  deadband(0.5, 1.0) = " << deadband(0.5, 1.0) << "\n";
    std::cout << "  deadband(2.0, 1.0) = " << deadband(2.0, 1.0) << "\n\n";

    std::cout << "All functions are symbolic-compatible for janus::Opti.\n";

    return 0;
}
