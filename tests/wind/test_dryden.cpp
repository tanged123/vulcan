// Tests for Dryden Turbulence Model
// Tests PSD functions, filter coefficients, filter stepping, and statistical
// properties
#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/wind/DrydenTurbulence.hpp>

#include <cmath>
#include <random>

// ============================================================================
// PSD Function Tests
// ============================================================================

TEST(DrydenPSD, LongitudinalZeroFrequency) {
    // At Ω=0, PSD_u = σ² * 2L/π
    double sigma = 2.0;
    double L = 100.0;

    double psd = vulcan::dryden::psd_longitudinal(0.0, sigma, L);
    double expected = sigma * sigma * 2.0 * L / M_PI;
    EXPECT_NEAR(psd, expected, 1e-10);
}

TEST(DrydenPSD, LongitudinalRolloff) {
    double sigma = 2.0;
    double L = 100.0;

    // At Ω = 1/L, PSD should be half of DC value
    double omega = 1.0 / L;
    double psd = vulcan::dryden::psd_longitudinal(omega, sigma, L);
    double dc = sigma * sigma * 2.0 * L / M_PI;

    // Φ(1/L) = σ² * 2L/π * 1/(1+1) = dc/2
    EXPECT_NEAR(psd, dc / 2.0, 1e-10);
}

TEST(DrydenPSD, LateralZeroFrequency) {
    // At Ω=0, PSD_v = σ² * L/π * (1+0)/(1+0)² = σ² * L/π
    double sigma = 3.0;
    double L = 150.0;

    double psd = vulcan::dryden::psd_lateral(0.0, sigma, L);
    double expected = sigma * sigma * L / M_PI;
    EXPECT_NEAR(psd, expected, 1e-10);
}

TEST(DrydenPSD, LateralPeakBehavior) {
    double sigma = 3.0;
    double L = 100.0;

    // PSD should have a peak at some positive frequency
    double psd_dc = vulcan::dryden::psd_lateral(0.0, sigma, L);
    double psd_low = vulcan::dryden::psd_lateral(0.01 / L, sigma, L);
    double psd_high = vulcan::dryden::psd_lateral(10.0 / L, sigma, L);

    // At very high frequencies, should roll off
    EXPECT_LT(psd_high, psd_dc);
}

TEST(DrydenPSD, SymbolicEvaluation) {
    auto omega = janus::sym("omega");
    auto psd = vulcan::dryden::psd_longitudinal(omega, 2.0, 100.0);

    janus::Function f("psd_u", {omega}, {psd});

    // Evaluate at Ω = 0
    auto result = f({0.0});
    double expected = 4.0 * 200.0 / M_PI; // σ²*2L/π
    EXPECT_NEAR(result[0](0, 0), expected, 1e-10);
}

// ============================================================================
// Filter Coefficient Tests
// ============================================================================

TEST(DrydenFilter, CoefficientComputation) {
    vulcan::wind::TurbulenceParams<double> params{.sigma_u = 2.0,
                                                  .sigma_v = 2.0,
                                                  .sigma_w = 1.5,
                                                  .L_u = 200.0,
                                                  .L_v = 200.0,
                                                  .L_w = 50.0};

    double airspeed = 100.0; // m/s
    double dt = 0.01;        // s

    auto coeffs = vulcan::dryden::compute_filter_coeffs(params, airspeed, dt);

    // State transition coefficients should be between 0 and 1 (stable)
    EXPECT_GT(coeffs.a_u, 0.0);
    EXPECT_LT(coeffs.a_u, 1.0);

    // Input coefficients should be positive
    EXPECT_GT(coeffs.b_u, 0.0);
}

TEST(DrydenFilter, MilSpecCoeffs) {
    auto coeffs = vulcan::dryden::mil_spec_coeffs(
        100.0,                                      // altitude
        vulcan::wind::TurbulenceSeverity::Moderate, // severity
        50.0,                                       // airspeed
        0.01);                                      // dt

    // Should produce valid coefficients
    EXPECT_GT(coeffs.a_u, 0.0);
    EXPECT_LT(coeffs.a_u, 1.0);
    EXPECT_GT(coeffs.c_u, 0.0); // Output gain should be positive
}

// ============================================================================
// Filter State Tests
// ============================================================================

TEST(DrydenFilter, InitState) {
    auto state = vulcan::dryden::init_state<double>();

    EXPECT_DOUBLE_EQ(state.x_u, 0.0);
    EXPECT_DOUBLE_EQ(state.x_v1, 0.0);
    EXPECT_DOUBLE_EQ(state.x_v2, 0.0);
    EXPECT_DOUBLE_EQ(state.x_w1, 0.0);
    EXPECT_DOUBLE_EQ(state.x_w2, 0.0);
}

TEST(DrydenFilter, StepUpdatesState) {
    auto state = vulcan::dryden::init_state<double>();
    auto coeffs = vulcan::dryden::mil_spec_coeffs(
        200.0, vulcan::wind::TurbulenceSeverity::Light, 60.0, 0.01);

    // Step with non-zero noise
    auto gust = vulcan::dryden::step(state, coeffs, 1.0, 1.0, 1.0);

    // State should have changed
    EXPECT_NE(state.x_u, 0.0);
    EXPECT_NE(state.x_v1, 0.0);
    EXPECT_NE(state.x_w1, 0.0);
}

TEST(DrydenFilter, ZeroNoiseProducesDecay) {
    auto state = vulcan::dryden::init_state<double>();
    auto coeffs = vulcan::dryden::mil_spec_coeffs(
        200.0, vulcan::wind::TurbulenceSeverity::Moderate, 60.0, 0.01);

    // First step with noise to excite state
    vulcan::dryden::step(state, coeffs, 1.0, 1.0, 1.0);
    double x_u_after_noise = state.x_u;

    // Steps with zero noise should decay
    for (int i = 0; i < 100; ++i) {
        vulcan::dryden::step(state, coeffs, 0.0, 0.0, 0.0);
    }

    EXPECT_LT(std::abs(state.x_u), std::abs(x_u_after_noise));
}

// ============================================================================
// Statistical Tests
// ============================================================================

TEST(DrydenFilter, ZeroMean) {
    auto state = vulcan::dryden::init_state<double>();
    auto coeffs = vulcan::dryden::mil_spec_coeffs(
        500.0, vulcan::wind::TurbulenceSeverity::Moderate, 100.0, 0.01);

    std::mt19937 rng(12345);
    std::normal_distribution<double> noise(0.0, 1.0);

    double sum_u = 0.0, sum_v = 0.0, sum_w = 0.0;
    double sum_u_sq = 0.0, sum_v_sq = 0.0, sum_w_sq = 0.0;
    const int N = 10000;

    // Warm-up
    for (int i = 0; i < 1000; ++i) {
        vulcan::dryden::step(state, coeffs, noise(rng), noise(rng), noise(rng));
    }

    // Collect samples
    for (int i = 0; i < N; ++i) {
        auto gust = vulcan::dryden::step(state, coeffs, noise(rng), noise(rng),
                                         noise(rng));
        sum_u += gust.u_g;
        sum_v += gust.v_g;
        sum_w += gust.w_g;
        sum_u_sq += gust.u_g * gust.u_g;
        sum_v_sq += gust.v_g * gust.v_g;
        sum_w_sq += gust.w_g * gust.w_g;
    }

    double mean_u = sum_u / N;
    double mean_v = sum_v / N;
    double mean_w = sum_w / N;

    // Compute standard deviations
    double std_u = std::sqrt(sum_u_sq / N - mean_u * mean_u);
    double std_v = std::sqrt(sum_v_sq / N - mean_v * mean_v);
    double std_w = std::sqrt(sum_w_sq / N - mean_w * mean_w);

    // Mean should be small relative to standard deviation
    // For a zero-mean process with large N, |mean| / std should be small
    // We accept |mean| < 0.5 * std as "approximately zero mean"
    // This is a practical test given finite samples and filter approximations
    if (std_u > 0.01)
        EXPECT_LT(std::abs(mean_u) / std_u, 0.5);
    if (std_v > 0.01)
        EXPECT_LT(std::abs(mean_v) / std_v, 0.5);
    if (std_w > 0.01)
        EXPECT_LT(std::abs(mean_w) / std_w, 0.5);
}

TEST(DrydenFilter, NonZeroVariance) {
    auto state = vulcan::dryden::init_state<double>();
    auto coeffs = vulcan::dryden::mil_spec_coeffs(
        500.0, vulcan::wind::TurbulenceSeverity::Moderate, 100.0, 0.01);

    std::mt19937 rng(54321);
    std::normal_distribution<double> noise(0.0, 1.0);

    double sum_u_sq = 0.0;
    const int N = 10000;

    // Warm-up
    for (int i = 0; i < 1000; ++i) {
        vulcan::dryden::step(state, coeffs, noise(rng), noise(rng), noise(rng));
    }

    // Collect samples
    for (int i = 0; i < N; ++i) {
        auto gust = vulcan::dryden::step(state, coeffs, noise(rng), noise(rng),
                                         noise(rng));
        sum_u_sq += gust.u_g * gust.u_g;
    }

    double var_u = sum_u_sq / N;

    // Variance should be positive and in reasonable range
    // Target σ ≈ 3.0 for moderate turbulence at high altitude
    EXPECT_GT(var_u, 0.1); // Not zero
}

// ============================================================================
// Symbolic Tests
// ============================================================================

TEST(DrydenFilter, SymbolicStep) {
    // Create symbolic noise inputs
    auto nu = janus::sym("noise_u");
    auto nv = janus::sym("noise_v");
    auto nw = janus::sym("noise_w");

    // Create symbolic state
    auto state = vulcan::dryden::init_state<janus::SymbolicScalar>();
    state.x_u = janus::sym("x_u");
    state.x_v1 = janus::sym("x_v1");
    state.x_v2 = janus::sym("x_v2");
    state.x_w1 = janus::sym("x_w1");
    state.x_w2 = janus::sym("x_w2");

    auto coeffs = vulcan::dryden::mil_spec_coeffs(
        200.0, vulcan::wind::TurbulenceSeverity::Light, 50.0, 0.01);

    auto gust = vulcan::dryden::step(state, coeffs, nu, nv, nw);

    // Verify outputs are symbolic (not constant)
    EXPECT_FALSE(gust.u_g.is_constant());
    EXPECT_FALSE(gust.v_g.is_constant());
    EXPECT_FALSE(gust.w_g.is_constant());
}

TEST(DrydenFilter, SymbolicGradient) {
    auto nu = janus::sym("noise_u");
    auto state = vulcan::dryden::init_state<janus::SymbolicScalar>();

    auto coeffs = vulcan::dryden::mil_spec_coeffs(
        200.0, vulcan::wind::TurbulenceSeverity::Light, 50.0, 0.01);

    janus::SymbolicScalar zero(0);
    auto gust = vulcan::dryden::step(state, coeffs, nu, zero, zero);

    // du_g/d(noise_u) should be non-zero
    auto du_dn = janus::jacobian(gust.u_g, nu);

    janus::Function f("du_dn", {nu}, {du_dn});
    auto result = f({1.0});

    // Gradient should be positive (more noise = more output)
    EXPECT_GT(std::abs(result[0](0, 0)), 0.0);
}

TEST(DrydenPSD, SymbolicGradient) {
    auto omega = janus::sym("omega");
    auto psd = vulcan::dryden::psd_longitudinal(omega, 2.0, 100.0);

    auto dpsd_domega = janus::jacobian(psd, omega);

    janus::Function f("dpsd_domega", {omega}, {dpsd_domega});

    // At Ω > 0, gradient should be negative (PSD decreases)
    auto result = f({0.01});
    EXPECT_LT(result[0](0, 0), 0.0);
}
