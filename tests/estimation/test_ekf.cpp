// Test Extended Kalman Filter - EKF tests with nonlinear systems
// Part of Vulcan - Aerospace Engineering Utilities

#include <gtest/gtest.h>
#include <vulcan/estimation/Estimation.hpp>

using namespace vulcan::estimation;

// =============================================================================
// Nonlinear Bearing-Range Tracking
// State: [x, y] position in 2D
// Measurement: range and bearing from origin
// =============================================================================

class EKFTest : public ::testing::Test {
  protected:
    static constexpr int N = 2; // State dim: [x, y]
    static constexpr int M = 2; // Measurement dim: [range, bearing]
    double dt = 0.1;

    ProcessNoise<N> Q;
    MeasurementNoise<M> R;

    void SetUp() override {
        Q = ProcessNoise<N>::scalar(0.01);
        R = MeasurementNoise<M>::diagonal(
            Eigen::Vector2d(0.1, 0.01)); // range, bearing
    }

    // Nonlinear measurement: [range, bearing] from position
    static Eigen::Vector2d h(const Eigen::Vector2d &x) {
        double range = std::sqrt(x(0) * x(0) + x(1) * x(1));
        double bearing = std::atan2(x(1), x(0));
        return Eigen::Vector2d(range, bearing);
    }

    // Jacobian of measurement
    static Eigen::Matrix<double, M, N> H_jacobian(const Eigen::Vector2d &x) {
        double r = std::sqrt(x(0) * x(0) + x(1) * x(1));
        double r2 = r * r;

        Eigen::Matrix<double, M, N> H;
        // d(range)/d(x,y)
        H(0, 0) = x(0) / r;
        H(0, 1) = x(1) / r;
        // d(bearing)/d(x,y)
        H(1, 0) = -x(1) / r2;
        H(1, 1) = x(0) / r2;

        return H;
    }
};

TEST_F(EKFTest, LinearSystemMatchesKF) {
    // For linear system, EKF should match standard KF
    Eigen::Matrix<double, N, N> F = Eigen::Matrix<double, N, N>::Identity();
    Eigen::Matrix<double, M, N> H_linear;
    H_linear << 1.0, 0.0, 0.0, 1.0; // Direct state observation

    FilterState<double, N> state;
    state.x << 1.0, 2.0;
    state.P = Eigen::Matrix2d::Identity();

    Eigen::Vector2d z(1.1, 2.1);

    // Linear dynamics and measurement
    auto f = [&F](const Eigen::Vector2d &x) { return F * x; };
    auto h = [&H_linear](const Eigen::Vector2d &x) { return H_linear * x; };

    // EKF update
    auto ekf_result = ekf_update<double, N, M>(state, z, h, H_linear, R);

    // KF update
    auto kf_result = kf_update<double, N, M>(state, z, H_linear, R);

    // Should match
    EXPECT_NEAR((ekf_result.x - kf_result.x).norm(), 0.0, 1e-10);
    EXPECT_NEAR((ekf_result.P - kf_result.P).norm(), 0.0, 1e-10);
}

TEST_F(EKFTest, NonlinearUpdateConverges) {
    // Target at (3, 4) -> range=5, bearing = atan2(4,3)
    Eigen::Vector2d true_pos(3.0, 4.0);
    Eigen::Vector2d true_meas = h(true_pos);

    // Start with uncertain estimate
    FilterState<double, N> state;
    state.x << 2.0, 3.0; // Wrong initial guess
    state.P = Eigen::Matrix2d::Identity() * 10.0;

    // Jacobian at current estimate
    auto H = H_jacobian(state.x);

    // Update with true measurement
    auto updated = ekf_update<double, N, M>(state, true_meas, h, H, R);

    // Should move toward truth
    double dist_before = (state.x - true_pos).norm();
    double dist_after = (updated.x - true_pos).norm();
    EXPECT_LT(dist_after, dist_before);
}

TEST_F(EKFTest, PredictWithNonlinearDynamics) {
    // Circular motion: x' = x*cos(w*dt) - y*sin(w*dt)
    //                  y' = x*sin(w*dt) + y*cos(w*dt)
    double omega = 0.5; // rad/s
    double c = std::cos(omega * dt);
    double s = std::sin(omega * dt);

    auto f = [c, s](const Eigen::Vector2d &x) {
        return Eigen::Vector2d(c * x(0) - s * x(1), s * x(0) + c * x(1));
    };

    // Jacobian of rotation dynamics
    Eigen::Matrix2d F;
    F << c, -s, s, c;

    FilterState<double, N> state;
    state.x << 1.0, 0.0; // Start on x-axis
    state.P = Eigen::Matrix2d::Identity() * 0.1;

    auto predicted = ekf_predict<double, N>(state, f, F, Q);

    // Should rotate slightly
    EXPECT_NEAR(predicted.x(0), c, 1e-10);
    EXPECT_NEAR(predicted.x(1), s, 1e-10);
}

TEST_F(EKFTest, CovarianceStaysPositiveDefinite) {
    FilterState<double, N> state;
    state.x << 3.0, 4.0;
    state.P = Eigen::Matrix2d::Identity();

    std::default_random_engine rng(42);
    std::normal_distribution<double> noise(0.0, 0.1);

    for (int i = 0; i < 100; ++i) {
        // Simulate noisy measurement
        auto true_meas = h(state.x);
        Eigen::Vector2d z(true_meas(0) + noise(rng), true_meas(1) + noise(rng));

        auto H = H_jacobian(state.x);

        state = ekf_update<double, N, M>(state, z, h, H, R);

        // Check positive definiteness
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(state.P);
        ASSERT_GT(solver.eigenvalues().minCoeff(), 0.0)
            << "Covariance became non-positive-definite at step " << i;
    }
}

TEST_F(EKFTest, EkfStepCombined) {
    // Simple identity dynamics
    auto f = [](const Eigen::Vector2d &x, const Eigen::Vector2d & /*u*/) {
        return x;
    };
    Eigen::Matrix2d F = Eigen::Matrix2d::Identity();

    FilterState<double, N> state;
    state.x << 3.0, 4.0;
    state.P = Eigen::Matrix2d::Identity();

    Eigen::Vector2d u = Eigen::Vector2d::Zero();
    Eigen::Vector2d z = h(state.x) + Eigen::Vector2d(0.1, 0.01);
    auto H = H_jacobian(state.x);

    auto result = ekf_step<double, N, N, M>(state, u, z, f, h, F, H, Q, R);

    // Just verify it runs without error and produces reasonable output
    EXPECT_TRUE(result.P.allFinite());
    EXPECT_TRUE(result.x.allFinite());
}
