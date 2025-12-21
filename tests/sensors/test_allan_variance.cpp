// Allan Variance Parameterized Noise Model Tests
#include <cmath>
#include <gtest/gtest.h>
#include <random>
#include <vulcan/sensors/AllanVarianceNoise.hpp>

// =============================================================================
// IMU Grade Presets
// =============================================================================

TEST(AllanVariance, ConsumerGradeGyro) {
    auto params = vulcan::sensors::imu_grades::consumer_gyro();

    // Verify approximate values
    EXPECT_GT(params.N, 1e-5);        // ARW should be significant
    EXPECT_GT(params.B, 1e-6);        // Bias instability
    EXPECT_GT(params.K, 1e-9);        // Rate random walk
    EXPECT_GT(params.tau_bias, 10.0); // Correlation time
}

TEST(AllanVariance, NavigationGradeGyro) {
    auto params = vulcan::sensors::imu_grades::navigation_gyro();

    // Navigation grade should be much better than consumer
    auto consumer = vulcan::sensors::imu_grades::consumer_gyro();

    EXPECT_LT(params.N, consumer.N);
    EXPECT_LT(params.B, consumer.B);
    EXPECT_LT(params.K, consumer.K);
}

TEST(AllanVariance, GradeOrdering) {
    // Noise should decrease from consumer to navigation
    auto consumer = vulcan::sensors::imu_grades::consumer_gyro();
    auto industrial = vulcan::sensors::imu_grades::industrial_gyro();
    auto tactical = vulcan::sensors::imu_grades::tactical_gyro();
    auto navigation = vulcan::sensors::imu_grades::navigation_gyro();

    // ARW ordering
    EXPECT_GT(consumer.N, industrial.N);
    EXPECT_GT(industrial.N, tactical.N);
    EXPECT_GT(tactical.N, navigation.N);

    // Bias ordering
    EXPECT_GT(consumer.B, industrial.B);
    EXPECT_GT(industrial.B, tactical.B);
    EXPECT_GT(tactical.B, navigation.B);
}

// =============================================================================
// Axis State and Coefficients
// =============================================================================

TEST(AllanVariance, InitAxisState) {
    auto state = vulcan::allan::init_axis_state<double>();

    EXPECT_DOUBLE_EQ(state.bias_state.bias, 0.0);
    EXPECT_DOUBLE_EQ(state.rrw_state.value, 0.0);
}

TEST(AllanVariance, ComputeAxisCoeffs) {
    auto params = vulcan::sensors::imu_grades::industrial_gyro();
    double dt = 0.01;

    auto coeffs = vulcan::allan::compute_axis_coeffs(params, dt);

    // ARW sigma should be N / sqrt(dt)
    EXPECT_NEAR(coeffs.arw_sigma, params.N / std::sqrt(dt), 1e-15);

    // Bias coeffs should exist
    EXPECT_GT(coeffs.bias_coeffs.a, 0.0);
    EXPECT_GT(coeffs.bias_coeffs.b, 0.0);

    // RRW coeffs should exist
    EXPECT_GT(coeffs.rrw_coeffs.gain, 0.0);
}

// =============================================================================
// Single Axis Step
// =============================================================================

TEST(AllanVariance, StepAxis) {
    auto state = vulcan::allan::init_axis_state<double>();
    auto params = vulcan::sensors::imu_grades::industrial_gyro();
    auto coeffs = vulcan::allan::compute_axis_coeffs(params, 0.01);

    // Step with unit noise
    double out = vulcan::allan::step_axis(state, coeffs, 1.0, 1.0, 1.0);

    // Output should be non-zero
    EXPECT_NE(out, 0.0);
}

TEST(AllanVariance, StepAxisZeroNoise) {
    auto state = vulcan::allan::init_axis_state<double>();
    auto params = vulcan::sensors::imu_grades::consumer_gyro();
    auto coeffs = vulcan::allan::compute_axis_coeffs(params, 0.01);

    // Step with zero noise
    double out = vulcan::allan::step_axis(state, coeffs, 0.0, 0.0, 0.0);

    // Output should be zero at initial state with zero noise
    EXPECT_DOUBLE_EQ(out, 0.0);
}

// =============================================================================
// IMU State and Coefficients
// =============================================================================

TEST(AllanVariance, InitIMUState) {
    auto state = vulcan::allan::init_state<double>();

    for (int i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(state.gyro[i].bias_state.bias, 0.0);
        EXPECT_DOUBLE_EQ(state.accel[i].bias_state.bias, 0.0);
    }
}

TEST(AllanVariance, ComputeIMUCoeffs) {
    auto gyro_params = vulcan::sensors::imu_grades::tactical_gyro();
    auto accel_params = vulcan::sensors::imu_grades::tactical_accel();
    double dt = 0.01;

    auto coeffs = vulcan::allan::compute_coeffs(gyro_params, accel_params, dt);

    // Gyro and accel coefficients should be different
    EXPECT_NE(coeffs.gyro.arw_sigma, coeffs.accel.arw_sigma);
}

// =============================================================================
// IMU Step Function
// =============================================================================

TEST(AllanVariance, StepIMU) {
    auto state = vulcan::allan::init_state<double>();
    auto coeffs = vulcan::allan::consumer_grade_coeffs(0.01);

    vulcan::allan::IMUNoiseInput<double> input;
    input.gyro_arw = Eigen::Vector3d::Ones();
    input.gyro_bias = Eigen::Vector3d::Ones();
    input.gyro_rrw = Eigen::Vector3d::Ones();
    input.accel_arw = Eigen::Vector3d::Ones();
    input.accel_bias = Eigen::Vector3d::Ones();
    input.accel_rrw = Eigen::Vector3d::Ones();

    auto output = vulcan::allan::step(state, coeffs, input);

    // Should produce non-zero 3-axis output
    EXPECT_NE(output.gyro.norm(), 0.0);
    EXPECT_NE(output.accel.norm(), 0.0);

    // All three axes should be the same with same input
    EXPECT_DOUBLE_EQ(output.gyro(0), output.gyro(1));
    EXPECT_DOUBLE_EQ(output.gyro(1), output.gyro(2));
}

TEST(AllanVariance, StepIMUSimple) {
    auto state = vulcan::allan::init_state<double>();
    auto coeffs = vulcan::allan::industrial_grade_coeffs(0.01);

    Eigen::Vector3d gyro_noise = Eigen::Vector3d::Random();
    Eigen::Vector3d accel_noise = Eigen::Vector3d::Random();

    auto output =
        vulcan::allan::step_simple(state, coeffs, gyro_noise, accel_noise);

    EXPECT_EQ(output.gyro.size(), 3);
    EXPECT_EQ(output.accel.size(), 3);
}

// =============================================================================
// Grade Factory Functions
// =============================================================================

TEST(AllanVariance, GradeFactoryFunctions) {
    double dt = 0.01;

    auto consumer = vulcan::allan::consumer_grade_coeffs(dt);
    auto industrial = vulcan::allan::industrial_grade_coeffs(dt);
    auto tactical = vulcan::allan::tactical_grade_coeffs(dt);
    auto navigation = vulcan::allan::navigation_grade_coeffs(dt);

    // Consumer should have highest noise
    EXPECT_GT(consumer.gyro.arw_sigma, industrial.gyro.arw_sigma);
    EXPECT_GT(industrial.gyro.arw_sigma, tactical.gyro.arw_sigma);
    EXPECT_GT(tactical.gyro.arw_sigma, navigation.gyro.arw_sigma);
}

// =============================================================================
// Statistical Verification
// =============================================================================

TEST(AllanVariance, StatisticalNoiseLevel) {
    std::mt19937 rng(42);
    std::normal_distribution<double> dist(0.0, 1.0);

    auto state = vulcan::allan::init_state<double>();
    auto coeffs = vulcan::allan::consumer_grade_coeffs(0.01);

    int num_samples = 10000;
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d sum_sq = Eigen::Vector3d::Zero();

    for (int i = 0; i < num_samples; ++i) {
        vulcan::allan::IMUNoiseInput<double> input;
        for (int j = 0; j < 3; ++j) {
            input.gyro_arw(j) = dist(rng);
            input.gyro_bias(j) = dist(rng);
            input.gyro_rrw(j) = dist(rng);
            input.accel_arw(j) = dist(rng);
            input.accel_bias(j) = dist(rng);
            input.accel_rrw(j) = dist(rng);
        }

        auto output = vulcan::allan::step(state, coeffs, input);
        sum += output.gyro;
        sum_sq += output.gyro.cwiseProduct(output.gyro);
    }

    Eigen::Vector3d mean = sum / num_samples;
    Eigen::Vector3d variance = sum_sq / num_samples - mean.cwiseProduct(mean);

    // Variance should be non-zero and reasonable
    for (int i = 0; i < 3; ++i) {
        EXPECT_GT(variance(i), 0.0);
    }
}

// =============================================================================
// Symbolic Mode Compatibility
// =============================================================================

TEST(AllanVariance, SymbolicAxisStep) {
    auto state = vulcan::allan::init_axis_state<janus::SymbolicScalar>();
    auto params = vulcan::sensors::imu_grades::industrial_gyro();
    auto coeffs = vulcan::allan::compute_axis_coeffs(params, 0.01);

    auto noise1 = janus::sym("w1");
    auto noise2 = janus::sym("w2");
    auto noise3 = janus::sym("w3");

    auto output =
        vulcan::allan::step_axis(state, coeffs, noise1, noise2, noise3);

    // Should create symbolic expression
    EXPECT_FALSE(output.is_constant());

    // Create function and evaluate with all zero noise
    janus::Function f("allan_step", {noise1, noise2, noise3}, {output});
    auto result = f({0.0, 0.0, 0.0});
    EXPECT_DOUBLE_EQ(result[0](0, 0), 0.0);
}

// =============================================================================
// State Persistence
// =============================================================================

TEST(AllanVariance, BiasAccumulation) {
    std::mt19937 rng(42);
    std::normal_distribution<double> dist(0.0, 1.0);

    auto state = vulcan::allan::init_state<double>();
    // Use consumer grade with significant bias instability
    auto coeffs = vulcan::allan::consumer_grade_coeffs(0.01);

    // Run for a while with noise
    for (int i = 0; i < 1000; ++i) {
        vulcan::allan::IMUNoiseInput<double> input;
        for (int j = 0; j < 3; ++j) {
            input.gyro_arw(j) = dist(rng);
            input.gyro_bias(j) = dist(rng);
            input.gyro_rrw(j) = dist(rng);
            input.accel_arw(j) = dist(rng);
            input.accel_bias(j) = dist(rng);
            input.accel_rrw(j) = dist(rng);
        }
        vulcan::allan::step(state, coeffs, input);
    }

    // Bias state should have accumulated
    bool has_nonzero_bias = false;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(state.gyro[i].bias_state.bias) > 1e-10) {
            has_nonzero_bias = true;
        }
    }
    EXPECT_TRUE(has_nonzero_bias);

    // RRW should have accumulated
    bool has_nonzero_rrw = false;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(state.gyro[i].rrw_state.value) > 1e-10) {
            has_nonzero_rrw = true;
        }
    }
    EXPECT_TRUE(has_nonzero_rrw);
}
