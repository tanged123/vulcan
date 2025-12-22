// Test Kalman Filter - Linear Kalman filter tests
// Part of Vulcan - Aerospace Engineering Utilities

#include <gtest/gtest.h>
#include <vulcan/estimation/Estimation.hpp>

using namespace vulcan::estimation;

// =============================================================================
// Constant Velocity Tracking Model
// A classic test case for Kalman filters:
// State: [position, velocity]
// Dynamics: x' = x + v*dt, v' = v
// Measurement: position only
// =============================================================================

class KalmanFilterTest : public ::testing::Test {
  protected:
    static constexpr int N = 2; // State dim: [pos, vel]
    static constexpr int M = 1; // Measurement dim: pos only
    double dt = 0.1;            // Sample time

    // State transition matrix (constant velocity)
    Eigen::Matrix<double, N, N> F;
    // Measurement matrix (observe position only)
    Eigen::Matrix<double, M, N> H;
    // Process noise
    ProcessNoise<N> Q;
    // Measurement noise
    MeasurementNoise<M> R;

    void SetUp() override {
        // F = [1 dt; 0 1]
        F << 1.0, dt, 0.0, 1.0;

        // H = [1 0]
        H << 1.0, 0.0;

        // Small process noise
        Q = ProcessNoise<N>::diagonal(Eigen::Vector2d(0.01, 0.01));

        // Measurement noise variance
        R = MeasurementNoise<M>::scalar(1.0);
    }
};

TEST_F(KalmanFilterTest, PredictIncreasesUncertainty) {
    // Start with known state
    FilterState<double, N> state;
    state.x << 0.0, 1.0; // Position 0, velocity 1
    state.P = Eigen::Matrix2d::Identity() * 0.1;

    auto predicted = kf_predict<double, N>(state, F, Q);

    // State should advance
    EXPECT_NEAR(predicted.x(0), dt, 1e-10);  // pos = 0 + 1*dt
    EXPECT_NEAR(predicted.x(1), 1.0, 1e-10); // vel unchanged

    // Covariance should increase (uncertainty grows without measurement)
    EXPECT_GT(predicted.P(0, 0), state.P(0, 0));
}

TEST_F(KalmanFilterTest, UpdateDecreasesUncertainty) {
    // Start with uncertain state
    FilterState<double, N> state;
    state.x << 0.0, 0.0;
    state.P = Eigen::Matrix2d::Identity() * 10.0;

    // Measurement at position 1
    Eigen::Matrix<double, M, 1> z;
    z << 1.0;

    auto updated = kf_update<double, N, M>(state, z, H, R);

    // State should move toward measurement
    EXPECT_GT(updated.x(0), 0.0);
    EXPECT_LT(updated.x(0), 1.0); // Not all the way due to prior

    // Covariance should decrease
    EXPECT_LT(updated.P(0, 0), state.P(0, 0));
}

TEST_F(KalmanFilterTest, ConvergenceToTruth) {
    // Simulate tracking a constant velocity target
    double true_pos = 0.0;
    double true_vel = 2.0; // m/s

    FilterState<double, N> state;
    state.x << 5.0, 0.0; // Start with wrong guess
    state.P = Eigen::Matrix2d::Identity() * 100.0;

    std::default_random_engine rng(42);
    std::normal_distribution<double> noise(0.0, 1.0);

    // Run filter for several steps
    for (int i = 0; i < 100; ++i) {
        // Predict
        state = kf_predict<double, N>(state, F, Q);

        // Simulate true dynamics and measurement
        true_pos += true_vel * dt;
        double meas = true_pos + noise(rng);

        Eigen::Matrix<double, M, 1> z;
        z << meas;

        // Update
        state = kf_update<double, N, M>(state, z, H, R);
    }

    // Should have converged close to truth
    EXPECT_NEAR(state.x(0), true_pos, 2.0); // Within 2m
    EXPECT_NEAR(state.x(1), true_vel, 0.5); // Within 0.5 m/s

    // Uncertainty should be small
    EXPECT_LT(state.P(0, 0), 2.0);
    EXPECT_LT(state.P(1, 1), 1.0);
}

TEST_F(KalmanFilterTest, CovarianceStaysPositiveDefinite) {
    FilterState<double, N> state;
    state.x << 0.0, 1.0;
    state.P = Eigen::Matrix2d::Identity();

    // Run many iterations
    for (int i = 0; i < 1000; ++i) {
        state = kf_predict<double, N>(state, F, Q);

        Eigen::Matrix<double, M, 1> z;
        z << state.x(0) + 0.1 * std::sin(i);

        state = kf_update<double, N, M>(state, z, H, R);

        // Check positive definiteness
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(state.P);
        ASSERT_GT(solver.eigenvalues().minCoeff(), 0.0)
            << "Covariance became non-positive-definite at step " << i;
    }
}

TEST_F(KalmanFilterTest, KfStepCombinesPredictAndUpdate) {
    FilterState<double, N> state;
    state.x << 0.0, 1.0;
    state.P = Eigen::Matrix2d::Identity();

    Eigen::Matrix<double, M, 1> z;
    z << 0.2;

    // Run combined step
    auto result1 = kf_step<double, N, M>(state, z, F, H, Q, R);

    // Run separately
    auto predicted = kf_predict<double, N>(state, F, Q);
    auto result2 = kf_update<double, N, M>(predicted, z, H, R);

    // Should match
    EXPECT_NEAR((result1.x - result2.x).norm(), 0.0, 1e-10);
    EXPECT_NEAR((result1.P - result2.P).norm(), 0.0, 1e-10);
}

TEST_F(KalmanFilterTest, WithControlInput) {
    // State [pos, vel], control is acceleration
    constexpr int U = 1;
    // B matrix: [0.5*dt^2; dt]
    Eigen::Matrix<double, N, U> B;
    B << 0.5 * dt * dt, dt;

    FilterState<double, N> state;
    state.x << 0.0, 0.0;
    state.P = Eigen::Matrix2d::Identity();

    // Apply constant acceleration
    Eigen::Matrix<double, U, 1> u;
    u << 1.0; // 1 m/s^2

    state = kf_predict<double, N, U>(state, F, B, u, Q);

    // Position should increase by 0.5*a*t^2 = 0.005
    EXPECT_NEAR(state.x(0), 0.5 * 1.0 * dt * dt, 1e-10);
    // Velocity should increase by a*t = 0.1
    EXPECT_NEAR(state.x(1), 1.0 * dt, 1e-10);
}
