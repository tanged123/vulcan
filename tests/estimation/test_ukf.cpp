// Test Unscented Kalman Filter - UKF tests
// Part of Vulcan - Aerospace Engineering Utilities

#include <gtest/gtest.h>
#include <vulcan/estimation/Estimation.hpp>

using namespace vulcan::estimation;

// =============================================================================
// UKF Tests using same nonlinear system as EKF for comparison
// =============================================================================

class UKFTest : public ::testing::Test {
  protected:
    static constexpr int N = 2;
    static constexpr int M = 2;
    double dt = 0.1;

    ProcessNoise<N> Q;
    MeasurementNoise<M> R;

    void SetUp() override {
        Q = ProcessNoise<N>::scalar(0.01);
        R = MeasurementNoise<M>::diagonal(Eigen::Vector2d(0.1, 0.01));
    }

    // Range-bearing measurement
    static Eigen::Vector2d h(const Eigen::Vector2d &x) {
        double range = std::sqrt(x(0) * x(0) + x(1) * x(1));
        double bearing = std::atan2(x(1), x(0));
        return Eigen::Vector2d(range, bearing);
    }
};

TEST_F(UKFTest, SigmaPointGeneration) {
    Eigen::Vector2d x(1.0, 2.0);
    Eigen::Matrix2d P = Eigen::Matrix2d::Identity();

    UKFParams params;
    auto sigma = sigma_points<N>(x, P, params);

    // Should have 2N+1 = 5 sigma points
    EXPECT_EQ(sigma.cols(), 5);

    // Mean sigma point should be x
    EXPECT_NEAR((sigma.col(0) - x).norm(), 0.0, 1e-10);

    // Points should be symmetric around mean
    for (int i = 0; i < N; ++i) {
        Eigen::Vector2d plus = sigma.col(1 + i) - x;
        Eigen::Vector2d minus = sigma.col(1 + N + i) - x;
        EXPECT_NEAR((plus + minus).norm(), 0.0, 1e-10);
    }
}

TEST_F(UKFTest, LinearSystemMatchesKF) {
    // For linear system, UKF should give very similar results to KF
    Eigen::Matrix<double, N, N> F = Eigen::Matrix<double, N, N>::Identity();
    Eigen::Matrix<double, M, N> H_linear;
    H_linear << 1.0, 0.0, 0.0, 1.0;

    FilterState<double, N> state;
    state.x << 1.0, 2.0;
    state.P = Eigen::Matrix2d::Identity();

    Eigen::Vector2d z(1.1, 2.1);

    auto h_linear = [](const Eigen::Vector2d &x) { return x; };

    // UKF update
    auto ukf_result = ukf_update<N, M>(state, z, h_linear, R);

    // KF update
    auto kf_result = kf_update<double, N, M>(state, z, H_linear, R);

    // Should be very close (not exact due to different implementations)
    EXPECT_NEAR((ukf_result.x - kf_result.x).norm(), 0.0, 0.01);
    EXPECT_NEAR((ukf_result.P - kf_result.P).norm(), 0.0, 0.1);
}

TEST_F(UKFTest, NonlinearUpdateConverges) {
    Eigen::Vector2d true_pos(3.0, 4.0);
    Eigen::Vector2d true_meas = h(true_pos);

    FilterState<double, N> state;
    state.x << 2.0, 3.0;
    state.P = Eigen::Matrix2d::Identity() * 10.0;

    auto updated = ukf_update<N, M>(state, true_meas, h, R);

    // Should move toward truth
    double dist_before = (state.x - true_pos).norm();
    double dist_after = (updated.x - true_pos).norm();
    EXPECT_LT(dist_after, dist_before);
}

TEST_F(UKFTest, PredictWithNonlinearDynamics) {
    double omega = 0.5;
    double c = std::cos(omega * dt);
    double s = std::sin(omega * dt);

    auto f = [c, s](const Eigen::Vector2d &x,
                    const Eigen::Matrix<double, 1, 1> & /*u*/) {
        return Eigen::Vector2d(c * x(0) - s * x(1), s * x(0) + c * x(1));
    };

    FilterState<double, N> state;
    state.x << 1.0, 0.0;
    state.P = Eigen::Matrix2d::Identity() * 0.01;

    Eigen::Matrix<double, 1, 1> u = Eigen::Matrix<double, 1, 1>::Zero();

    auto predicted = ukf_predict<N, 1>(state, u, f, Q);

    // Should rotate
    EXPECT_NEAR(predicted.x(0), c, 0.01);
    EXPECT_NEAR(predicted.x(1), s, 0.01);
}

TEST_F(UKFTest, CovarianceStaysPositiveDefinite) {
    FilterState<double, N> state;
    state.x << 3.0, 4.0;
    state.P = Eigen::Matrix2d::Identity();

    std::default_random_engine rng(42);
    std::normal_distribution<double> noise(0.0, 0.1);

    for (int i = 0; i < 100; ++i) {
        auto true_meas = h(state.x);
        Eigen::Vector2d z(true_meas(0) + noise(rng), true_meas(1) + noise(rng));

        state = ukf_update<N, M>(state, z, h, R);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(state.P);
        ASSERT_GT(solver.eigenvalues().minCoeff(), 0.0)
            << "Covariance became non-positive-definite at step " << i;
    }
}

TEST_F(UKFTest, UKFParamsAffectSpread) {
    Eigen::Vector2d x(0.0, 0.0);
    Eigen::Matrix2d P = Eigen::Matrix2d::Identity();

    UKFParams narrow;
    narrow.alpha = 0.001;

    UKFParams wide;
    wide.alpha = 1.0;

    auto sigma_narrow = sigma_points<N>(x, P, narrow);
    auto sigma_wide = sigma_points<N>(x, P, wide);

    // Wide alpha should produce more spread sigma points
    double spread_narrow = (sigma_narrow.col(1) - sigma_narrow.col(0)).norm();
    double spread_wide = (sigma_wide.col(1) - sigma_wide.col(0)).norm();

    EXPECT_GT(spread_wide, spread_narrow);
}

TEST_F(UKFTest, UkfStepCombined) {
    auto f = [](const Eigen::Vector2d &x, const Eigen::Vector2d & /*u*/) {
        return x;
    };

    FilterState<double, N> state;
    state.x << 3.0, 4.0;
    state.P = Eigen::Matrix2d::Identity();

    Eigen::Vector2d u = Eigen::Vector2d::Zero();
    Eigen::Vector2d z = h(state.x) + Eigen::Vector2d(0.1, 0.01);

    auto result = ukf_step<N, N, M>(state, u, z, f, h, Q, R);

    EXPECT_TRUE(result.P.allFinite());
    EXPECT_TRUE(result.x.allFinite());
}
