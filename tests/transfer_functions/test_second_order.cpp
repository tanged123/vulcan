#include <cmath>
#include <gtest/gtest.h>
#include <vulcan/transfer_functions/SecondOrder.hpp>

using namespace vulcan::tf;

TEST(SecondOrderTest, UnderdampedCoeffs) {
    auto c = second_order<double>(10.0, 0.5, 1.0, 0.001);
    EXPECT_DOUBLE_EQ(c.omega_n, 10.0);
    EXPECT_DOUBLE_EQ(c.zeta, 0.5);
    EXPECT_FALSE(std::isnan(c.A(0, 0)));
}

TEST(SecondOrderTest, CriticallyDampedCoeffs) {
    auto c = second_order<double>(10.0, 1.0, 1.0, 0.001);
    EXPECT_DOUBLE_EQ(c.zeta, 1.0);
    EXPECT_FALSE(std::isnan(c.A(0, 0)));
}

TEST(SecondOrderTest, OverdampedCoeffs) {
    auto c = second_order<double>(10.0, 1.5, 1.0, 0.001);
    EXPECT_DOUBLE_EQ(c.zeta, 1.5);
    EXPECT_FALSE(std::isnan(c.A(0, 0)));
}

TEST(SecondOrderTest, StepToSteadyState) {
    auto c = second_order<double>(20.0, 0.7, 1.0, 0.0001);
    Eigen::Vector2d state = Eigen::Vector2d::Zero();

    for (int i = 0; i < 20000; ++i) {
        state = second_order_step(c, state, 1.0);
    }

    EXPECT_NEAR(second_order_output(c, state, 1.0), 1.0, 0.001);
}

TEST(SecondOrderTest, Overshoot) {
    auto c = second_order<double>(50.0, 0.3, 1.0, 0.0001);
    Eigen::Vector2d state = Eigen::Vector2d::Zero();
    double max_out = 0.0;

    for (int i = 0; i < 10000; ++i) {
        state = second_order_step(c, state, 1.0);
        double y = second_order_output(c, state, 1.0);
        if (y > max_out)
            max_out = y;
    }

    double expected_overshoot = std::exp(-M_PI * 0.3 / std::sqrt(1.0 - 0.09));
    EXPECT_NEAR(max_out, 1.0 + expected_overshoot, 0.02);
}

TEST(SecondOrderTest, Characteristics) {
    auto [tr, ts, Mp] = second_order_characteristics(10.0, 0.7);
    EXPECT_NEAR(tr, (1.8 - 0.6 * 0.7) / 10.0, 1e-10);
    EXPECT_NEAR(ts, 4.0 / (0.7 * 10.0), 1e-10);
}

TEST(SecondOrderTest, SymbolicInstantiation) {
    auto c = second_order<casadi::MX>(20.0, 0.7, 1.0, 0.01);

    Eigen::Matrix<casadi::MX, 2, 1> state;
    state(0) = casadi::MX::sym("y");
    state(1) = casadi::MX::sym("ydot");
    casadi::MX input = casadi::MX::sym("u");

    auto next = second_order_step(c, state, input);
    EXPECT_GT(next(0).n_dep(), 0);
}
