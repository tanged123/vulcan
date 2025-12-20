// Tests for von Kármán Turbulence Model
// Tests PSD functions, filter coefficients, filter stepping, and comparison
// with Dryden
#include <gtest/gtest.h>
#include <janus/janus.hpp>
#include <vulcan/wind/DrydenTurbulence.hpp>
#include <vulcan/wind/VonKarmanTurbulence.hpp>

#include <cmath>
#include <random>

// ============================================================================
// PSD Function Tests
// ============================================================================

TEST(VonKarmanPSD, LongitudinalZeroFrequency) {
    // At Ω=0, PSD_u = σ² * 2L/π (same as Dryden at DC)
    double sigma = 2.0;
    double L = 100.0;

    double psd = vulcan::von_karman::psd_longitudinal(0.0, sigma, L);
    double expected = sigma * sigma * 2.0 * L / M_PI;
    EXPECT_NEAR(psd, expected, 1e-10);
}

TEST(VonKarmanPSD, LongitudinalRolloff) {
    double sigma = 2.0;
    double L = 100.0;

    // von Kármán has (5/6) exponent rolloff, different from Dryden
    double omega = 0.1;

    double psd_vk = vulcan::von_karman::psd_longitudinal(omega, sigma, L);
    double psd_dry = vulcan::dryden::psd_longitudinal(omega, sigma, L);

    // Both should be positive
    EXPECT_GT(psd_vk, 0.0);
    EXPECT_GT(psd_dry, 0.0);

    // At moderate frequencies, von Kármán typically has higher PSD
    // due to slower rolloff
}

TEST(VonKarmanPSD, LateralZeroFrequency) {
    // At Ω=0, numerator = 1, denominator = 1
    double sigma = 3.0;
    double L = 150.0;

    double psd = vulcan::von_karman::psd_lateral(0.0, sigma, L);
    double expected = sigma * sigma * L / M_PI;
    EXPECT_NEAR(psd, expected, 1e-10);
}

TEST(VonKarmanPSD, LateralHighFrequencyRolloff) {
    double sigma = 3.0;
    double L = 100.0;

    double psd_dc = vulcan::von_karman::psd_lateral(0.0, sigma, L);
    double psd_high = vulcan::von_karman::psd_lateral(1.0, sigma, L);

    // High frequency PSD should be much smaller
    EXPECT_LT(psd_high, psd_dc * 0.1);
}

TEST(VonKarmanPSD, SpectralConstant) {
    // Verify the spectral constant is used
    EXPECT_NEAR(vulcan::von_karman::SPECTRAL_CONSTANT, 1.339, 0.001);
}

// ============================================================================
// Filter Coefficient Tests
// ============================================================================

TEST(VonKarmanFilter, CoefficientComputation) {
    vulcan::wind::TurbulenceParams<double> params{.sigma_u = 2.0,
                                                  .sigma_v = 2.0,
                                                  .sigma_w = 1.5,
                                                  .L_u = 200.0,
                                                  .L_v = 200.0,
                                                  .L_w = 50.0};

    double airspeed = 100.0;
    double dt = 0.01;

    auto coeffs =
        vulcan::von_karman::compute_filter_coeffs(params, airspeed, dt);

    // State transition matrices should be stable (diagonal elements < 1)
    for (int i = 0; i < 3; ++i) {
        EXPECT_LT(coeffs.A_u[i][i], 1.0);
        EXPECT_GT(coeffs.A_u[i][i], 0.0);
    }

    for (int i = 0; i < 4; ++i) {
        EXPECT_LT(coeffs.A_v[i][i], 1.0);
        EXPECT_GT(coeffs.A_v[i][i], 0.0);
        EXPECT_LT(coeffs.A_w[i][i], 1.0);
        EXPECT_GT(coeffs.A_w[i][i], 0.0);
    }
}

TEST(VonKarmanFilter, HigherOrderThanDryden) {
    // von Kármán uses 3rd order for longitudinal (vs 1st for Dryden)
    // and 4th order for lateral/vertical (vs 2nd for Dryden)
    vulcan::wind::TurbulenceParams<double> params{.sigma_u = 2.0,
                                                  .sigma_v = 2.0,
                                                  .sigma_w = 2.0,
                                                  .L_u = 100.0,
                                                  .L_v = 100.0,
                                                  .L_w = 100.0};

    auto vk_coeffs =
        vulcan::von_karman::compute_filter_coeffs(params, 50.0, 0.01);
    auto dry_coeffs = vulcan::dryden::compute_filter_coeffs(params, 50.0, 0.01);

    // von Kármán has more parameters (3x3 vs scalar for longitudinal)
    // Just verify both compute without error
    EXPECT_GT(vk_coeffs.A_u[0][0], 0.0);
    EXPECT_GT(dry_coeffs.a_u, 0.0);
}

// ============================================================================
// Filter State Tests
// ============================================================================

TEST(VonKarmanFilter, InitState) {
    auto state = vulcan::von_karman::init_state<double>();

    for (const auto &s : state.x_u)
        EXPECT_DOUBLE_EQ(s, 0.0);
    for (const auto &s : state.x_v)
        EXPECT_DOUBLE_EQ(s, 0.0);
    for (const auto &s : state.x_w)
        EXPECT_DOUBLE_EQ(s, 0.0);
}

TEST(VonKarmanFilter, StepUpdatesState) {
    auto state = vulcan::von_karman::init_state<double>();
    auto coeffs = vulcan::von_karman::mil_spec_coeffs(
        200.0, vulcan::wind::TurbulenceSeverity::Light, 60.0, 0.01);

    auto gust = vulcan::von_karman::step(state, coeffs, 1.0, 1.0, 1.0);

    // States should have updated
    bool any_changed = false;
    for (const auto &s : state.x_u)
        if (s != 0.0)
            any_changed = true;
    EXPECT_TRUE(any_changed);
}

TEST(VonKarmanFilter, ZeroNoiseDecay) {
    auto state = vulcan::von_karman::init_state<double>();
    auto coeffs = vulcan::von_karman::mil_spec_coeffs(
        200.0, vulcan::wind::TurbulenceSeverity::Moderate, 60.0, 0.01);

    // Excite with noise
    vulcan::von_karman::step(state, coeffs, 1.0, 1.0, 1.0);
    double initial_x_u0 = state.x_u[0];

    // Decay with zero noise
    for (int i = 0; i < 100; ++i) {
        vulcan::von_karman::step(state, coeffs, 0.0, 0.0, 0.0);
    }

    EXPECT_LT(std::abs(state.x_u[0]), std::abs(initial_x_u0));
}

// ============================================================================
// Statistical Tests
// ============================================================================

TEST(VonKarmanFilter, ZeroMean) {
    auto state = vulcan::von_karman::init_state<double>();
    auto coeffs = vulcan::von_karman::mil_spec_coeffs(
        500.0, vulcan::wind::TurbulenceSeverity::Moderate, 100.0, 0.01);

    std::mt19937 rng(11111);
    std::normal_distribution<double> noise(0.0, 1.0);

    double sum_u = 0.0, sum_v = 0.0, sum_w = 0.0;
    const int N = 10000;

    // Warm-up
    for (int i = 0; i < 1000; ++i) {
        vulcan::von_karman::step(state, coeffs, noise(rng), noise(rng),
                                 noise(rng));
    }

    // Collect
    for (int i = 0; i < N; ++i) {
        auto gust = vulcan::von_karman::step(state, coeffs, noise(rng),
                                             noise(rng), noise(rng));
        sum_u += gust.u_g;
        sum_v += gust.v_g;
        sum_w += gust.w_g;
    }

    EXPECT_NEAR(sum_u / N, 0.0, 0.35);
    EXPECT_NEAR(sum_v / N, 0.0, 0.35);
    EXPECT_NEAR(sum_w / N, 0.0, 0.35);
}

TEST(VonKarmanFilter, NonZeroVariance) {
    auto state = vulcan::von_karman::init_state<double>();
    auto coeffs = vulcan::von_karman::mil_spec_coeffs(
        500.0, vulcan::wind::TurbulenceSeverity::Moderate, 100.0, 0.01);

    std::mt19937 rng(22222);
    std::normal_distribution<double> noise(0.0, 1.0);

    double sum_sq = 0.0;
    const int N = 10000;

    // Warm-up
    for (int i = 0; i < 1000; ++i) {
        vulcan::von_karman::step(state, coeffs, noise(rng), noise(rng),
                                 noise(rng));
    }

    for (int i = 0; i < N; ++i) {
        auto gust = vulcan::von_karman::step(state, coeffs, noise(rng),
                                             noise(rng), noise(rng));
        sum_sq += gust.u_g * gust.u_g;
    }

    EXPECT_GT(sum_sq / N, 0.1);
}

// ============================================================================
// Comparison with Dryden
// ============================================================================

TEST(VonKarmanVsDryden, BothProduceValidOutput) {
    vulcan::wind::TurbulenceParams<double> params{.sigma_u = 2.0,
                                                  .sigma_v = 2.0,
                                                  .sigma_w = 2.0,
                                                  .L_u = 100.0,
                                                  .L_v = 100.0,
                                                  .L_w = 100.0};

    auto dry_state = vulcan::dryden::init_state<double>();
    auto vk_state = vulcan::von_karman::init_state<double>();

    auto dry_coeffs = vulcan::dryden::compute_filter_coeffs(params, 50.0, 0.01);
    auto vk_coeffs =
        vulcan::von_karman::compute_filter_coeffs(params, 50.0, 0.01);

    std::mt19937 rng(33333);
    std::normal_distribution<double> noise(0.0, 1.0);

    // Both should produce finite, reasonable outputs
    for (int i = 0; i < 100; ++i) {
        double n_u = noise(rng), n_v = noise(rng), n_w = noise(rng);

        auto dry_gust =
            vulcan::dryden::step(dry_state, dry_coeffs, n_u, n_v, n_w);
        auto vk_gust =
            vulcan::von_karman::step(vk_state, vk_coeffs, n_u, n_v, n_w);

        EXPECT_TRUE(std::isfinite(dry_gust.u_g));
        EXPECT_TRUE(std::isfinite(vk_gust.u_g));
        EXPECT_TRUE(std::isfinite(dry_gust.v_g));
        EXPECT_TRUE(std::isfinite(vk_gust.v_g));
        EXPECT_TRUE(std::isfinite(dry_gust.w_g));
        EXPECT_TRUE(std::isfinite(vk_gust.w_g));
    }
}

// ============================================================================
// Symbolic Tests
// ============================================================================

TEST(VonKarmanPSD, SymbolicEvaluation) {
    auto omega = janus::sym("omega");
    auto psd = vulcan::von_karman::psd_longitudinal(omega, 2.0, 100.0);

    EXPECT_FALSE(psd.is_constant());

    janus::Function f("psd_vk", {omega}, {psd});

    // At Ω=0
    auto result = f({0.0});
    double expected = 4.0 * 200.0 / M_PI;
    EXPECT_NEAR(result[0](0, 0), expected, 1e-10);
}

TEST(VonKarmanFilter, SymbolicStep) {
    auto nu = janus::sym("noise_u");
    auto nv = janus::sym("noise_v");
    auto nw = janus::sym("noise_w");

    auto state = vulcan::von_karman::init_state<janus::SymbolicScalar>();
    for (auto &s : state.x_u)
        s = janus::sym("x");
    for (auto &s : state.x_v)
        s = janus::sym("x");
    for (auto &s : state.x_w)
        s = janus::sym("x");

    auto coeffs = vulcan::von_karman::mil_spec_coeffs(
        200.0, vulcan::wind::TurbulenceSeverity::Light, 50.0, 0.01);

    auto gust = vulcan::von_karman::step(state, coeffs, nu, nv, nw);

    EXPECT_FALSE(gust.u_g.is_constant());
    EXPECT_FALSE(gust.v_g.is_constant());
    EXPECT_FALSE(gust.w_g.is_constant());
}

TEST(VonKarmanPSD, SymbolicGradient) {
    auto omega = janus::sym("omega");
    auto psd = vulcan::von_karman::psd_longitudinal(omega, 2.0, 100.0);

    auto dpsd = janus::jacobian(psd, omega);

    janus::Function f("dpsd_vk", {omega}, {dpsd});

    // Gradient should be negative at positive frequencies
    auto result = f({0.01});
    EXPECT_LT(result[0](0, 0), 0.0);
}
