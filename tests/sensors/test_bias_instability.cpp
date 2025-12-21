// Bias Instability Tests
#include <cmath>
#include <gtest/gtest.h>
#include <random>
#include <vulcan/sensors/BiasInstability.hpp>

// =============================================================================
// State Initialization
// =============================================================================

TEST(BiasInstability, InitState) {
    auto state = vulcan::bias_instability::init_state<double>();
    EXPECT_DOUBLE_EQ(state.bias, 0.0);
}

TEST(BiasInstability, InitStateWithValue) {
    auto state = vulcan::bias_instability::init_state<double>(0.5);
    EXPECT_DOUBLE_EQ(state.bias, 0.5);
}

// =============================================================================
// Coefficient Computation
// =============================================================================

TEST(BiasInstability, ComputeCoeffs) {
    double sigma_b = 1e-5;
    double tau = 100.0;
    double dt = 0.01;

    auto coeffs = vulcan::bias_instability::compute_coeffs(sigma_b, tau, dt);

    // a = exp(-dt/tau) = exp(-0.0001) ≈ 0.9999
    EXPECT_NEAR(coeffs.a, std::exp(-0.0001), 1e-10);

    // b = sigma_b * sqrt(1 - a²)
    double expected_b = sigma_b * std::sqrt(1.0 - coeffs.a * coeffs.a);
    EXPECT_DOUBLE_EQ(coeffs.b, expected_b);
}

TEST(BiasInstability, CoeffsLongTau) {
    // Very long correlation time (typical for navigation grade)
    double sigma_b = 4.8e-8;
    double tau = 3600.0; // 1 hour
    double dt = 0.01;

    auto coeffs = vulcan::bias_instability::compute_coeffs(sigma_b, tau, dt);

    // For very long tau, a should be very close to 1
    // exp(-0.01/3600) = exp(-2.78e-6) ≈ 0.999997
    EXPECT_NEAR(coeffs.a, std::exp(-dt / tau), 1e-10);
    EXPECT_GT(coeffs.a, 0.99999);
}

// =============================================================================
// Step Function
// =============================================================================

TEST(BiasInstability, StepMarkovDynamics) {
    auto state = vulcan::bias_instability::init_state<double>(1.0);
    auto coeffs = vulcan::bias_instability::compute_coeffs(0.1, 10.0, 1.0);

    // With zero noise, bias should decay
    double out1 = vulcan::bias_instability::step(state, coeffs, 0.0);
    EXPECT_LT(out1, 1.0);

    // Should continue decaying
    double out2 = vulcan::bias_instability::step(state, coeffs, 0.0);
    EXPECT_LT(out2, out1);
}

TEST(BiasInstability, StepWithInlineCoeffs) {
    auto state = vulcan::bias_instability::init_state<double>();
    double sigma_b = 0.1;
    double tau = 10.0;
    double dt = 0.5;

    double out = vulcan::bias_instability::step(state, sigma_b, tau, dt, 1.0);

    // Should have non-zero output
    EXPECT_NE(out, 0.0);
}

// =============================================================================
// Analysis Utilities
// =============================================================================

TEST(BiasInstability, SteadyStateVariance) {
    double sigma_b = 2.5e-5;
    double var = vulcan::bias_instability::steady_state_variance(sigma_b);
    EXPECT_DOUBLE_EQ(var, sigma_b * sigma_b);
}

TEST(BiasInstability, TimeToFraction) {
    double tau = 100.0;
    double fraction = 0.632; // ~1 - e^(-1)

    double t = vulcan::bias_instability::time_to_fraction(tau, fraction);

    // For 63.2%, time should be approximately tau/2 * 1 = 50s
    // Actually: t = -tau/2 * ln(1-0.632) = -50 * ln(0.368) ≈ 50
    EXPECT_NEAR(t, 50.0, 0.5);
}

// =============================================================================
// Statistical Verification
// =============================================================================

TEST(BiasInstability, SteadyStateVarianceEmpirical) {
    // Verify process reaches steady-state variance
    std::mt19937 rng(42);
    std::normal_distribution<double> dist(0.0, 1.0);

    double sigma_b = 0.01;
    double tau = 10.0;
    double dt = 0.01;

    // Run for many tau periods to reach steady state
    int warmup_steps = static_cast<int>(10 * tau / dt);
    int sample_steps = 10000;
    int num_realizations = 100;

    double sum_sq = 0.0;
    for (int r = 0; r < num_realizations; ++r) {
        auto state = vulcan::bias_instability::init_state<double>();
        auto coeffs =
            vulcan::bias_instability::compute_coeffs(sigma_b, tau, dt);

        // Warmup
        for (int i = 0; i < warmup_steps; ++i) {
            vulcan::bias_instability::step(state, coeffs, dist(rng));
        }

        // Sample
        for (int i = 0; i < sample_steps; ++i) {
            double val =
                vulcan::bias_instability::step(state, coeffs, dist(rng));
            sum_sq += val * val;
        }
    }

    double empirical_variance = sum_sq / (num_realizations * sample_steps);
    double expected_variance = sigma_b * sigma_b;

    // Should be within 5% of expected
    EXPECT_NEAR(empirical_variance, expected_variance,
                0.05 * expected_variance);
}

// =============================================================================
// Symbolic Mode Compatibility
// =============================================================================

TEST(BiasInstability, SymbolicCompatibility) {
    auto initial = janus::sym("b0");
    auto state =
        vulcan::bias_instability::State<janus::SymbolicScalar>{.bias = initial};
    auto coeffs = vulcan::bias_instability::compute_coeffs(0.1, 10.0, 1.0);
    auto noise = janus::sym("w");

    auto output = vulcan::bias_instability::step(state, coeffs, noise);

    // Should create symbolic expression: a*b0 + b*w
    EXPECT_FALSE(output.is_constant());

    // Create function and evaluate
    janus::Function f("bias_step", {initial, noise}, {output});
    auto result = f({1.0, 0.0}); // Zero noise = just decay
    EXPECT_DOUBLE_EQ(result[0](0, 0), coeffs.a);
}
