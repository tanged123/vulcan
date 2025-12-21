// Distribution Tests
#include <cmath>
#include <gtest/gtest.h>
#include <vulcan/rng/Distributions.hpp>
#include <vulcan/rng/RNG.hpp>

using namespace vulcan;

// =============================================================================
// IMU Noise Generation
// =============================================================================

TEST(Distributions, GenerateIMUNoise) {
    rng::RNG gen(42);
    auto input = rng::generate_imu_noise(gen);

    // All channels should be populated
    EXPECT_NE(input.gyro_arw.norm(), 0.0);
    EXPECT_NE(input.gyro_bias.norm(), 0.0);
    EXPECT_NE(input.gyro_rrw.norm(), 0.0);
    EXPECT_NE(input.accel_arw.norm(), 0.0);
    EXPECT_NE(input.accel_bias.norm(), 0.0);
    EXPECT_NE(input.accel_rrw.norm(), 0.0);
}

TEST(Distributions, IMUNoiseReproducibility) {
    rng::RNG gen1(42);
    rng::RNG gen2(42);

    auto input1 = rng::generate_imu_noise(gen1);
    auto input2 = rng::generate_imu_noise(gen2);

    EXPECT_EQ(input1.gyro_arw, input2.gyro_arw);
    EXPECT_EQ(input1.gyro_bias, input2.gyro_bias);
    EXPECT_EQ(input1.accel_arw, input2.accel_arw);
}

TEST(Distributions, IMUNoiseIntegration) {
    // Test that generated noise works with Allan variance model
    rng::RNG gen(42);
    auto state = allan::init_state<double>();
    auto coeffs = allan::consumer_grade_coeffs(0.01);

    for (int i = 0; i < 100; ++i) {
        auto input = rng::generate_imu_noise(gen);
        auto noise = allan::step(state, coeffs, input);

        EXPECT_TRUE(std::isfinite(noise.gyro(0)));
        EXPECT_TRUE(std::isfinite(noise.accel(2)));
    }
}

// =============================================================================
// 3D Noise Generation
// =============================================================================

TEST(Distributions, GenerateNoise3) {
    rng::RNG gen(42);

    int N = 10000;
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();

    for (int i = 0; i < N; ++i) {
        sum += rng::generate_noise3(gen);
    }

    Eigen::Vector3d mean = sum / N;
    EXPECT_NEAR(mean.norm(), 0.0, 0.1);
}

TEST(Distributions, GenerateNoiseN) {
    rng::RNG gen(42);
    auto v = rng::generate_noiseN<6>(gen);

    EXPECT_EQ(v.size(), 6);
    for (int i = 0; i < 6; ++i) {
        EXPECT_TRUE(std::isfinite(v(i)));
    }
}

// =============================================================================
// Correlated Noise
// =============================================================================

TEST(Distributions, CorrelatedNoiseCovariance) {
    rng::RNG gen(42);

    // Create covariance matrix
    Eigen::Matrix3d P;
    P << 1.0, 0.5, 0.2, 0.5, 2.0, 0.3, 0.2, 0.3, 0.5;

    // Cholesky decomposition
    Eigen::LLT<Eigen::Matrix3d> llt(P);
    Eigen::Matrix3d L = llt.matrixL();

    // Generate samples and estimate covariance
    int N = 50000;
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Matrix3d sum_outer = Eigen::Matrix3d::Zero();

    for (int i = 0; i < N; ++i) {
        auto v = rng::generate_correlated_noise<3>(gen, L);
        sum += v;
        sum_outer += v * v.transpose();
    }

    Eigen::Vector3d mean = sum / N;
    Eigen::Matrix3d cov = sum_outer / N - mean * mean.transpose();

    // Check covariance matches target (10% tolerance)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(cov(i, j), P(i, j), 0.1 * std::abs(P(i, j)) + 0.05);
        }
    }
}

TEST(Distributions, CorrelatedNoiseDynamic) {
    rng::RNG gen(42);

    Eigen::MatrixXd P(4, 4);
    P.setIdentity();
    P *= 2.0;

    Eigen::LLT<Eigen::MatrixXd> llt(P);
    Eigen::MatrixXd L = llt.matrixL();

    auto v = rng::generate_correlated_noise(gen, L);
    EXPECT_EQ(v.size(), 4);
}

// =============================================================================
// Turbulence Noise
// =============================================================================

TEST(Distributions, TurbulenceNoise) {
    rng::RNG gen(42);
    double nu, nv, nw;

    rng::generate_turbulence_noise(gen, nu, nv, nw);

    EXPECT_TRUE(std::isfinite(nu));
    EXPECT_TRUE(std::isfinite(nv));
    EXPECT_TRUE(std::isfinite(nw));
}

TEST(Distributions, TurbulenceNoiseIndependence) {
    rng::RNG gen(42);
    int N = 10000;
    double sum_uv = 0.0;

    for (int i = 0; i < N; ++i) {
        double nu, nv, nw;
        rng::generate_turbulence_noise(gen, nu, nv, nw);
        sum_uv += nu * nv;
    }

    double cov_uv = sum_uv / N;
    EXPECT_NEAR(cov_uv, 0.0, 0.05);
}
