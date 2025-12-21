// First-Order Markov Process Tests
#include <cmath>
#include <gtest/gtest.h>
#include <random>
#include <vulcan/sensors/MarkovProcess.hpp>

// =============================================================================
// State Initialization
// =============================================================================

TEST(MarkovProcess, InitState) {
    auto state = vulcan::markov::init_state<double>();
    EXPECT_DOUBLE_EQ(state.x, 0.0);
}

TEST(MarkovProcess, InitStateWithValue) {
    auto state = vulcan::markov::init_state<double>(1.5);
    EXPECT_DOUBLE_EQ(state.x, 1.5);
}

// =============================================================================
// Discretization
// =============================================================================

TEST(MarkovProcess, Discretize) {
    double tau = 10.0;
    double sigma = 0.5;
    double dt = 0.1;

    auto coeffs = vulcan::markov::discretize(tau, sigma, dt);

    // phi = exp(-dt/tau) = exp(-0.01) ≈ 0.99
    EXPECT_NEAR(coeffs.phi, std::exp(-0.01), 1e-10);

    // q = sigma * sqrt(1 - phi²)
    double expected_q = sigma * std::sqrt(1.0 - coeffs.phi * coeffs.phi);
    EXPECT_DOUBLE_EQ(coeffs.q, expected_q);
}

TEST(MarkovProcess, DiscretizePreservesVariance) {
    // Discretization should preserve steady-state variance at any dt
    double tau = 5.0;
    double sigma = 0.1;

    double dt_small = 0.001;
    double dt_large = 0.5;

    auto coeffs_small = vulcan::markov::discretize(tau, sigma, dt_small);
    auto coeffs_large = vulcan::markov::discretize(tau, sigma, dt_large);

    // For any dt, steady-state variance = sigma²
    // The discretization should be designed so this holds
    // Verify: q² / (1 - phi²) = sigma²
    double var_small = coeffs_small.q * coeffs_small.q /
                       (1.0 - coeffs_small.phi * coeffs_small.phi);
    double var_large = coeffs_large.q * coeffs_large.q /
                       (1.0 - coeffs_large.phi * coeffs_large.phi);

    EXPECT_NEAR(var_small, sigma * sigma, 1e-10);
    EXPECT_NEAR(var_large, sigma * sigma, 1e-10);
}

// =============================================================================
// Continuous-Time Parameters
// =============================================================================

TEST(MarkovProcess, ProcessNoisePSD) {
    double tau = 10.0;
    double sigma = 0.5;

    double S = vulcan::markov::process_noise_psd(tau, sigma);

    // S = 2 * sigma² / tau = 2 * 0.25 / 10 = 0.05
    EXPECT_DOUBLE_EQ(S, 0.05);
}

TEST(MarkovProcess, ProcessNoiseIntensity) {
    double tau = 10.0;
    double sigma = 0.5;

    double qc = vulcan::markov::process_noise_intensity(tau, sigma);

    // qc = sqrt(2/tau) * sigma = sqrt(0.2) * 0.5
    EXPECT_DOUBLE_EQ(qc, std::sqrt(0.2) * 0.5);
}

// =============================================================================
// Step Function
// =============================================================================

TEST(MarkovProcess, StepDecay) {
    auto state = vulcan::markov::init_state<double>(1.0);
    auto coeffs = vulcan::markov::discretize(10.0, 0.1, 1.0);

    // With zero noise, state should decay
    double out1 = vulcan::markov::step(state, coeffs, 0.0);
    EXPECT_LT(out1, 1.0);
    EXPECT_NEAR(out1, std::exp(-0.1), 1e-10);
}

TEST(MarkovProcess, StepWithInlineParams) {
    auto state = vulcan::markov::init_state<double>();
    double out = vulcan::markov::step(state, 10.0, 0.5, 0.1, 1.0);
    EXPECT_NE(out, 0.0);
}

// =============================================================================
// Analysis Utilities
// =============================================================================

TEST(MarkovProcess, Autocorrelation) {
    double tau = 10.0;
    double sigma = 0.5;

    // At lag 0
    double R0 = vulcan::markov::autocorrelation(tau, sigma, 0.0);
    EXPECT_DOUBLE_EQ(R0, sigma * sigma);

    // At lag = tau
    double R_tau = vulcan::markov::autocorrelation(tau, sigma, tau);
    EXPECT_NEAR(R_tau, sigma * sigma * std::exp(-1.0), 1e-10);

    // Should be symmetric
    double R_neg = vulcan::markov::autocorrelation(tau, sigma, -5.0);
    double R_pos = vulcan::markov::autocorrelation(tau, sigma, 5.0);
    EXPECT_DOUBLE_EQ(R_neg, R_pos);
}

TEST(MarkovProcess, PSDAtFrequency) {
    double tau = 10.0;
    double sigma = 0.5;

    // At DC (f=0)
    double S0 = vulcan::markov::psd_at_frequency(tau, sigma, 0.0);
    // S(0) = sigma² * 2 * tau = 0.25 * 20 = 5
    EXPECT_DOUBLE_EQ(S0, 5.0);

    // At corner frequency f_c = 1/(2*pi*tau)
    double fc = 1.0 / (2.0 * M_PI * tau);
    double S_fc = vulcan::markov::psd_at_frequency(tau, sigma, fc);
    // S(fc) = S(0) / 2
    EXPECT_NEAR(S_fc, S0 / 2.0, 1e-10);
}

// =============================================================================
// Statistical Verification
// =============================================================================

TEST(MarkovProcess, SteadyStateVarianceEmpirical) {
    std::mt19937 rng(42);
    std::normal_distribution<double> dist(0.0, 1.0);

    double tau = 5.0;
    double sigma = 0.2;
    double dt = 0.01;

    int warmup_steps = static_cast<int>(20 * tau / dt);
    int sample_steps = 1000;
    int num_realizations = 500; // Use ensemble averaging

    auto coeffs = vulcan::markov::discretize(tau, sigma, dt);

    double sum_sq = 0.0;
    for (int r = 0; r < num_realizations; ++r) {
        auto state = vulcan::markov::init_state<double>();

        // Warmup
        for (int i = 0; i < warmup_steps; ++i) {
            vulcan::markov::step(state, coeffs, dist(rng));
        }

        // Sample
        for (int i = 0; i < sample_steps; ++i) {
            double val = vulcan::markov::step(state, coeffs, dist(rng));
            sum_sq += val * val;
        }
    }

    double empirical_variance = sum_sq / (num_realizations * sample_steps);

    // Variance should be sigma² (using 10% tolerance for statistical variation)
    EXPECT_NEAR(empirical_variance, sigma * sigma, 0.10 * sigma * sigma);
}

TEST(MarkovProcess, CorrelationTimeEmpirical) {
    std::mt19937 rng(42);
    std::normal_distribution<double> dist(0.0, 1.0);

    double tau = 10.0;
    double sigma = 0.5;
    double dt = 0.1;

    int num_steps = 100000;
    int lag_steps = static_cast<int>(tau / dt); // Lag of 1 tau

    auto state = vulcan::markov::init_state<double>();
    auto coeffs = vulcan::markov::discretize(tau, sigma, dt);

    std::vector<double> samples(num_steps);
    for (int i = 0; i < num_steps; ++i) {
        samples[i] = vulcan::markov::step(state, coeffs, dist(rng));
    }

    // Compute autocorrelation at lag tau
    double sum_xy = 0.0;
    double sum_xx = 0.0;
    int count = num_steps - lag_steps;
    for (int i = 0; i < count; ++i) {
        sum_xy += samples[i] * samples[i + lag_steps];
        sum_xx += samples[i] * samples[i];
    }

    double R_lag = sum_xy / count;
    double R_0 = sum_xx / count;

    // At lag tau, R(tau)/R(0) = exp(-1) ≈ 0.368
    double rho = R_lag / R_0;
    EXPECT_NEAR(rho, std::exp(-1.0), 0.05);
}

// =============================================================================
// Symbolic Mode Compatibility
// =============================================================================

TEST(MarkovProcess, SymbolicCompatibility) {
    auto x0 = janus::sym("x0");
    auto state = vulcan::markov::State<janus::SymbolicScalar>{.x = x0};
    auto coeffs = vulcan::markov::discretize(10.0, 0.5, 0.1);
    auto noise = janus::sym("w");

    auto output = vulcan::markov::step(state, coeffs, noise);

    // Should create symbolic expression: phi*x0 + q*w
    EXPECT_FALSE(output.is_constant());

    // Create function and evaluate with zero noise = pure decay
    janus::Function f("markov_step", {x0, noise}, {output});
    auto result = f({1.0, 0.0});
    EXPECT_NEAR(result[0](0, 0), coeffs.phi, 1e-10);
}
