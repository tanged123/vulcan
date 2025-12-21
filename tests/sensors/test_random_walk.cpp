// Random Walk Noise Tests
#include <cmath>
#include <gtest/gtest.h>
#include <random>
#include <vulcan/sensors/RandomWalk.hpp>

// =============================================================================
// State Initialization
// =============================================================================

TEST(RandomWalk, InitState) {
    auto state = vulcan::random_walk::init_state<double>();
    EXPECT_DOUBLE_EQ(state.value, 0.0);
}

// =============================================================================
// Coefficient Computation
// =============================================================================

TEST(RandomWalk, ComputeCoeffs) {
    double K = 1e-6; // Rate random walk coefficient
    double dt = 0.01;

    auto coeffs = vulcan::random_walk::compute_coeffs(K, dt);

    // gain = K * sqrt(dt) = 1e-6 * 0.1 = 1e-7
    EXPECT_DOUBLE_EQ(coeffs.gain, K * std::sqrt(dt));
}

// =============================================================================
// Step Function
// =============================================================================

TEST(RandomWalk, StepAccumulates) {
    auto state = vulcan::random_walk::init_state<double>();
    auto coeffs = vulcan::random_walk::compute_coeffs(1.0, 1.0);

    // With K=1-and dt=1, gain = 1, so output equals noise input
    double out1 = vulcan::random_walk::step(state, coeffs, 0.5);
    EXPECT_DOUBLE_EQ(out1, 0.5);

    double out2 = vulcan::random_walk::step(state, coeffs, 0.3);
    EXPECT_DOUBLE_EQ(out2, 0.8);

    double out3 = vulcan::random_walk::step(state, coeffs, -0.2);
    EXPECT_DOUBLE_EQ(out3, 0.6);
}

TEST(RandomWalk, StepWithInlineCoeffs) {
    auto state = vulcan::random_walk::init_state<double>();
    double K = 2.0;
    double dt = 0.25; // gain = 2 * 0.5 = 1

    double out = vulcan::random_walk::step(state, K, dt, 1.0);
    EXPECT_DOUBLE_EQ(out, 1.0);
}

TEST(RandomWalk, ZeroNoise) {
    auto state = vulcan::random_walk::init_state<double>();
    auto coeffs = vulcan::random_walk::compute_coeffs(1e-5, 0.01);

    // Zero noise input should not change state
    double out1 = vulcan::random_walk::step(state, coeffs, 0.0);
    EXPECT_DOUBLE_EQ(out1, 0.0);

    double out2 = vulcan::random_walk::step(state, coeffs, 0.0);
    EXPECT_DOUBLE_EQ(out2, 0.0);
}

// =============================================================================
// Analysis Utilities
// =============================================================================

TEST(RandomWalk, ExpectedVariance) {
    double K = 1e-5;
    double t = 100.0; // 100 seconds

    double var = vulcan::random_walk::expected_variance(K, t);

    // Expected: K² * t = 1e-10 * 100 = 1e-8
    EXPECT_DOUBLE_EQ(var, 1e-8);
}

TEST(RandomWalk, ExpectedStddev) {
    double K = 1e-5;
    double t = 100.0;

    double stddev = vulcan::random_walk::expected_stddev(K, t);

    // Expected: K * sqrt(t) = 1e-5 * 10 = 1e-4
    EXPECT_DOUBLE_EQ(stddev, 1e-4);
}

// =============================================================================
// Statistical Verification
// =============================================================================

TEST(RandomWalk, VarianceGrowth) {
    // Verify variance grows linearly with time
    std::mt19937 rng(42);
    std::normal_distribution<double> dist(0.0, 1.0);

    double K = 0.01;  // Rate random walk
    double dt = 0.01; // 100 Hz
    int num_steps = 1000;
    int num_realizations = 1000;

    double sum_sq = 0.0;
    for (int r = 0; r < num_realizations; ++r) {
        auto state = vulcan::random_walk::init_state<double>();
        auto coeffs = vulcan::random_walk::compute_coeffs(K, dt);

        for (int i = 0; i < num_steps; ++i) {
            vulcan::random_walk::step(state, coeffs, dist(rng));
        }
        sum_sq += state.value * state.value;
    }

    double empirical_variance = sum_sq / num_realizations;
    double t = num_steps * dt;
    double expected_variance = K * K * t;

    // Should be within 10% due to finite sample size
    EXPECT_NEAR(empirical_variance, expected_variance, 0.1 * expected_variance);
}

// =============================================================================
// Symbolic Mode Compatibility
// =============================================================================

TEST(RandomWalk, SymbolicCompatibility) {
    auto x0 = janus::sym("x0");
    auto state = vulcan::random_walk::State<janus::SymbolicScalar>{.value = x0};
    auto coeffs = vulcan::random_walk::compute_coeffs(1.0, 1.0);
    auto noise = janus::sym("w");

    auto output = vulcan::random_walk::step(state, coeffs, noise);

    // Should create a symbolic expression: x0 + gain * w
    EXPECT_FALSE(output.is_constant());

    // Create function and evaluate
    janus::Function f("random_walk_step", {x0, noise}, {output});
    auto result = f({0.5, 0.3});
    EXPECT_DOUBLE_EQ(result[0](0, 0), 0.8);
}
