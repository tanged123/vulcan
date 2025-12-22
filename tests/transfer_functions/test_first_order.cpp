#include <cmath>
#include <gtest/gtest.h>
#include <vulcan/transfer_functions/FirstOrder.hpp>

using namespace vulcan::tf;

TEST(FirstOrderTest, CoefficientComputation) {
    auto c = first_order<double>(0.1, 1.0, 0.01);

    EXPECT_DOUBLE_EQ(c.tau, 0.1);
    EXPECT_DOUBLE_EQ(c.K, 1.0);
    EXPECT_DOUBLE_EQ(c.dt, 0.01);

    double expected_a = std::exp(-0.01 / 0.1);
    double expected_b = 1.0 * (1.0 - expected_a);

    EXPECT_NEAR(c.a, expected_a, 1e-10);
    EXPECT_NEAR(c.b, expected_b, 1e-10);
}

TEST(FirstOrderTest, StepToSteadyState) {
    auto c = first_order<double>(0.1, 1.0, 0.001);

    double state = 0.0;
    double input = 1.0;

    int steps = static_cast<int>(10.0 * 0.1 / 0.001);
    for (int i = 0; i < steps; ++i) {
        state = first_order_step(c, state, input);
    }

    EXPECT_NEAR(state, 1.0, 0.001);
}

TEST(FirstOrderTest, AnalyticalResponse) {
    double tau = 0.2;
    double K = 1.5;

    EXPECT_NEAR(first_order_response(tau, K, 0.0), 0.0, 1e-10);

    double at_tau = first_order_response(tau, K, tau);
    double expected = K * (1.0 - std::exp(-1.0));
    EXPECT_NEAR(at_tau, expected, 1e-10);
}

TEST(FirstOrderTest, DCGain) {
    auto c = first_order<double>(0.05, 2.5, 0.001);

    double state = 0.0;
    for (int i = 0; i < 1000; ++i) {
        state = first_order_step(c, state, 1.0);
    }
    EXPECT_NEAR(state, 2.5, 0.01);
}

TEST(FirstOrderTest, SymbolicInstantiation) {
    auto c = first_order<casadi::MX>(0.1, 1.0, 0.01);

    casadi::MX state = casadi::MX::sym("x");
    casadi::MX input = casadi::MX::sym("u");
    casadi::MX next = first_order_step(c, state, input);

    EXPECT_EQ(next.n_dep(), 2);
}
