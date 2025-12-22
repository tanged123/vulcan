#include <gtest/gtest.h>
#include <vulcan/transfer_functions/Nonlinear.hpp>

using namespace vulcan::tf;

// Rate Limiting
TEST(NonlinearTest, RateLimitBasic) {
    double result = rate_limit(0.0, 100.0, 10.0, 0.1);
    EXPECT_NEAR(result, 1.0, 1e-10);
}

TEST(NonlinearTest, RateLimitWithinBound) {
    double result = rate_limit(5.0, 5.5, 10.0, 0.1);
    EXPECT_NEAR(result, 5.5, 1e-10);
}

// Saturation
TEST(NonlinearTest, SaturateWithinLimits) {
    EXPECT_DOUBLE_EQ(saturate(5.0, 0.0, 10.0), 5.0);
}

TEST(NonlinearTest, SaturateAboveMax) {
    EXPECT_DOUBLE_EQ(saturate(15.0, 0.0, 10.0), 10.0);
}

TEST(NonlinearTest, SaturateBelowMin) {
    EXPECT_DOUBLE_EQ(saturate(-5.0, 0.0, 10.0), 0.0);
}

// Deadband
TEST(NonlinearTest, DeadbandWithin) {
    EXPECT_DOUBLE_EQ(deadband(0.5, 1.0), 0.0);
    EXPECT_DOUBLE_EQ(deadband(-0.5, 1.0), 0.0);
}

TEST(NonlinearTest, DeadbandAbove) {
    EXPECT_NEAR(deadband(2.0, 1.0), 1.0, 1e-10);
    EXPECT_NEAR(deadband(-2.0, 1.0), -1.0, 1e-10);
}

// Hysteresis
TEST(NonlinearTest, HysteresisWithinBand) {
    EXPECT_DOUBLE_EQ(hysteresis(5.5, 5.0, 1.0), 5.0);
}

TEST(NonlinearTest, HysteresisAboveBand) {
    EXPECT_DOUBLE_EQ(hysteresis(7.0, 5.0, 1.0), 6.0);
}

TEST(NonlinearTest, HysteresisBelowBand) {
    EXPECT_DOUBLE_EQ(hysteresis(3.0, 5.0, 1.0), 4.0);
}

// Symbolic
TEST(NonlinearTest, SymbolicRateLimit) {
    casadi::MX curr = casadi::MX::sym("curr");
    casadi::MX cmd = casadi::MX::sym("cmd");
    casadi::MX result = rate_limit(curr, cmd, 10.0, 0.1);
    EXPECT_EQ(result.n_dep(), 2);
}

TEST(NonlinearTest, SymbolicDeadband) {
    casadi::MX input = casadi::MX::sym("input");
    casadi::MX result = deadband(input, 0.5);
    EXPECT_GT(result.n_dep(), 0);
}
