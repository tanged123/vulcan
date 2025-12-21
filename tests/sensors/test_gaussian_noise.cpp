// Gaussian White Noise Tests
#include <cmath>
#include <gtest/gtest.h>
#include <random>
#include <vulcan/sensors/GaussianNoise.hpp>

// =============================================================================
// Noise Scaling Tests
// =============================================================================

TEST(GaussianNoise, ApplyScaling) {
    // Simple scaling test
    double noise_input = 1.0; // Unit variance input
    double sigma = 0.5;

    double output = vulcan::gaussian::apply(noise_input, sigma);

    EXPECT_DOUBLE_EQ(output, 0.5);
}

TEST(GaussianNoise, ApplyNegativeInput) {
    double noise_input = -2.0;
    double sigma = 3.0;

    double output = vulcan::gaussian::apply(noise_input, sigma);

    EXPECT_DOUBLE_EQ(output, -6.0);
}

// =============================================================================
// PSD Scaling Tests
// =============================================================================

TEST(GaussianNoise, ApplyPSD) {
    // Test PSD-based scaling
    double noise_input = 1.0;
    double psd_density = 1e-4; // units²/Hz
    double dt = 0.01;          // 100 Hz

    double output = vulcan::gaussian::apply_psd(noise_input, psd_density, dt);

    // Expected sigma = sqrt(psd/dt) = sqrt(1e-4 / 0.01) = sqrt(0.01) = 0.1
    EXPECT_DOUBLE_EQ(output, 0.1);
}

TEST(GaussianNoise, VarianceFromPSD) {
    double psd = 1e-4;
    double dt = 0.01;

    double variance = vulcan::gaussian::variance_from_psd(psd, dt);

    // Expected: psd/dt = 1e-4 / 0.01 = 0.01
    EXPECT_DOUBLE_EQ(variance, 0.01);
}

TEST(GaussianNoise, PSDFromVariance) {
    double variance = 0.01;
    double dt = 0.01;

    double psd = vulcan::gaussian::psd_from_variance(variance, dt);

    // Expected: variance * dt = 0.01 * 0.01 = 1e-4
    EXPECT_DOUBLE_EQ(psd, 1e-4);
}

TEST(GaussianNoise, PSDVarianceRoundTrip) {
    // Should round-trip correctly
    double original_psd = 2.5e-5;
    double dt = 0.001;

    double variance = vulcan::gaussian::variance_from_psd(original_psd, dt);
    double recovered_psd = vulcan::gaussian::psd_from_variance(variance, dt);

    EXPECT_NEAR(recovered_psd, original_psd, 1e-15);
}

// =============================================================================
// Allan Variance Parameter Conversion
// =============================================================================

TEST(GaussianNoise, SigmaFromARW) {
    // Typical consumer gyro ARW: 8.7e-5 rad/s/√Hz
    double N = 8.7e-5;
    double dt = 0.01; // 100 Hz

    double sigma = vulcan::gaussian::sigma_from_arw(N, dt);

    // Expected: N / sqrt(dt) = 8.7e-5 / sqrt(0.01) = 8.7e-5 / 0.1 = 8.7e-4
    EXPECT_NEAR(sigma, 8.7e-4, 1e-10);
}

TEST(GaussianNoise, SigmaFromARWHighRate) {
    // Higher sample rate = lower per-sample noise
    double N = 1e-4;
    double dt_low = 0.01;   // 100 Hz
    double dt_high = 0.001; // 1000 Hz

    double sigma_low = vulcan::gaussian::sigma_from_arw(N, dt_low);
    double sigma_high = vulcan::gaussian::sigma_from_arw(N, dt_high);

    // Higher rate should have higher per-sample sigma (but lower integrated
    // noise)
    EXPECT_GT(sigma_high, sigma_low);
    // Ratio should be sqrt(dt_low/dt_high) = sqrt(10) ≈ 3.16
    EXPECT_NEAR(sigma_high / sigma_low, std::sqrt(10.0), 1e-10);
}

// =============================================================================
// Statistical Verification
// =============================================================================

TEST(GaussianNoise, StatisticalVariance) {
    // Verify that applying scaling produces correct variance
    std::mt19937 rng(42);
    std::normal_distribution<double> dist(0.0, 1.0);

    double sigma = 2.5;
    double sum = 0.0;
    double sum_sq = 0.0;
    int N = 100000;

    for (int i = 0; i < N; ++i) {
        double noise_input = dist(rng);
        double output = vulcan::gaussian::apply(noise_input, sigma);
        sum += output;
        sum_sq += output * output;
    }

    double mean = sum / N;
    double variance = sum_sq / N - mean * mean;

    // Mean should be near zero
    EXPECT_NEAR(mean, 0.0, 0.1);
    // Variance should be sigma²
    EXPECT_NEAR(variance, sigma * sigma, 0.1);
}

// =============================================================================
// Symbolic Mode Compatibility
// =============================================================================

TEST(GaussianNoise, SymbolicCompatibility) {
    // Test that the function compiles with CasADi types
    auto noise_sym = janus::sym("noise");
    double sigma = 0.5;

    auto output = vulcan::gaussian::apply(noise_sym, sigma);

    // Should create a symbolic expression
    EXPECT_FALSE(output.is_constant());

    // Create function and evaluate at specific value
    janus::Function f("gaussian_apply", {noise_sym}, {output});
    auto result = f({2.0});
    EXPECT_DOUBLE_EQ(result[0](0, 0), 1.0);
}
