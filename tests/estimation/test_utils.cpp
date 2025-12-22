// Test Estimation Utilities - Gating, Joseph form, etc.
// Part of Vulcan - Aerospace Engineering Utilities

#include <gtest/gtest.h>
#include <vulcan/estimation/Estimation.hpp>

using namespace vulcan::estimation;

// =============================================================================
// Normalized Innovation Squared Tests
// =============================================================================

TEST(EstimationUtilsTest, NISScalarMeasurement) {
    // 1D case: NIS = y^2 / S
    Eigen::Matrix<double, 1, 1> innovation;
    innovation << 2.0;

    Eigen::Matrix<double, 1, 1> S;
    S << 4.0;

    double nis = normalized_innovation_squared<double, 1>(innovation, S);
    EXPECT_NEAR(nis, 1.0, 1e-10); // 4/4 = 1
}

TEST(EstimationUtilsTest, NISVectorMeasurement) {
    Eigen::Vector2d innovation(1.0, 2.0);
    Eigen::Matrix2d S = Eigen::Matrix2d::Identity();

    double nis = normalized_innovation_squared<double, 2>(innovation, S);
    // NIS = [1 2] * I^-1 * [1; 2] = 1 + 4 = 5
    EXPECT_NEAR(nis, 5.0, 1e-10);
}

// =============================================================================
// Gating Tests
// =============================================================================

TEST(EstimationUtilsTest, PassesGateTrue) {
    Eigen::Matrix<double, 1, 1> innovation;
    innovation << 1.0;

    Eigen::Matrix<double, 1, 1> S;
    S << 1.0;

    // NIS = 1, threshold = 3.84 (chi-squared 95% for dof=1)
    EXPECT_TRUE((passes_gate<double, 1>(innovation, S, 3.84)));
}

TEST(EstimationUtilsTest, PassesGateFalse) {
    Eigen::Matrix<double, 1, 1> innovation;
    innovation << 5.0;

    Eigen::Matrix<double, 1, 1> S;
    S << 1.0;

    // NIS = 25, threshold = 3.84
    EXPECT_FALSE((passes_gate<double, 1>(innovation, S, 3.84)));
}

TEST(EstimationUtilsTest, MeasurementPassesGate) {
    Eigen::Vector2d z(1.0, 2.0);
    Eigen::Vector2d z_pred(0.9, 2.1);
    Eigen::Matrix2d S = Eigen::Matrix2d::Identity() * 10.0;

    double threshold = chi_squared_threshold(2, 0.95); // ~5.99
    EXPECT_TRUE((measurement_passes_gate<double, 2>(z, z_pred, S, threshold)));
}

// =============================================================================
// Chi-Squared Threshold Tests
// =============================================================================

TEST(EstimationUtilsTest, ChiSquaredThresholds95) {
    EXPECT_NEAR(chi_squared_threshold(1, 0.95), 3.841, 0.01);
    EXPECT_NEAR(chi_squared_threshold(2, 0.95), 5.991, 0.01);
    EXPECT_NEAR(chi_squared_threshold(3, 0.95), 7.815, 0.01);
}

TEST(EstimationUtilsTest, ChiSquaredThresholds99) {
    EXPECT_NEAR(chi_squared_threshold(1, 0.99), 6.635, 0.01);
    EXPECT_NEAR(chi_squared_threshold(2, 0.99), 9.210, 0.01);
    EXPECT_NEAR(chi_squared_threshold(3, 0.99), 11.345, 0.01);
}

// =============================================================================
// Joseph Form Tests
// =============================================================================

TEST(EstimationUtilsTest, JosephFormBasic) {
    constexpr int N = 2;
    constexpr int M = 1;

    Eigen::Matrix2d P_prior = Eigen::Matrix2d::Identity();
    Eigen::Matrix<double, N, M> K;
    K << 0.5, 0.1;
    Eigen::Matrix<double, M, N> H;
    H << 1.0, 0.0;
    Eigen::Matrix<double, M, M> R;
    R << 0.1;

    auto P_joseph = joseph_update<N, M>(P_prior, K, H, R);

    // Should be symmetric
    EXPECT_NEAR((P_joseph - P_joseph.transpose()).norm(), 0.0, 1e-10);

    // Should be positive definite
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(P_joseph);
    EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
}

TEST(EstimationUtilsTest, JosephFormMatchesStandard) {
    // For well-conditioned case, Joseph form and standard should match
    constexpr int N = 2;
    constexpr int M = 1;

    Eigen::Matrix2d P_prior = Eigen::Matrix2d::Identity() * 10.0;
    Eigen::Matrix<double, M, N> H;
    H << 1.0, 0.0;
    Eigen::Matrix<double, M, M> R;
    R << 1.0;

    // Compute Kalman gain
    Eigen::Matrix<double, M, M> S = H * P_prior * H.transpose() + R;
    Eigen::Matrix<double, N, M> K = P_prior * H.transpose() * S.inverse();

    // Standard update
    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d P_standard = (I - K * H) * P_prior;

    // Joseph form
    Eigen::Matrix2d P_joseph = joseph_update<N, M>(P_prior, K, H, R);

    // Should be close
    EXPECT_NEAR((P_joseph - P_standard).norm(), 0.0, 0.1);
}

// =============================================================================
// Symmetrize Tests
// =============================================================================

TEST(EstimationUtilsTest, SymmetrizeAlreadySymmetric) {
    Eigen::Matrix2d P = Eigen::Matrix2d::Identity();
    auto P_sym = symmetrize<2>(P);
    EXPECT_NEAR((P - P_sym).norm(), 0.0, 1e-10);
}

TEST(EstimationUtilsTest, SymmetrizeAsymmetric) {
    Eigen::Matrix2d P;
    P << 1.0, 0.5, 0.4, 1.0; // Slightly asymmetric

    auto P_sym = symmetrize<2>(P);

    // Should be symmetric
    EXPECT_NEAR((P_sym - P_sym.transpose()).norm(), 0.0, 1e-10);

    // Off-diagonal should be average
    EXPECT_NEAR(P_sym(0, 1), 0.45, 1e-10);
    EXPECT_NEAR(P_sym(1, 0), 0.45, 1e-10);
}

// =============================================================================
// Noise Type Tests
// =============================================================================

TEST(EstimationTypesTest, ProcessNoiseDiagonal) {
    Eigen::Vector2d variances(1.0, 2.0);
    auto Q = ProcessNoise<2>::diagonal(variances);

    EXPECT_NEAR(Q.Q(0, 0), 1.0, 1e-10);
    EXPECT_NEAR(Q.Q(1, 1), 2.0, 1e-10);
    EXPECT_NEAR(Q.Q(0, 1), 0.0, 1e-10);
}

TEST(EstimationTypesTest, MeasurementNoiseScalar) {
    auto R = MeasurementNoise<3>::scalar(5.0);

    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(R.R(i, i), 5.0, 1e-10);
        for (int j = 0; j < 3; ++j) {
            if (i != j) {
                EXPECT_NEAR(R.R(i, j), 0.0, 1e-10);
            }
        }
    }
}

TEST(EstimationTypesTest, FilterStateDefaultConstructor) {
    FilterState<double, 3> state;

    EXPECT_EQ(state.x.size(), 3);
    EXPECT_NEAR(state.x.norm(), 0.0, 1e-10);                    // Zero state
    EXPECT_TRUE(state.P.isApprox(Eigen::Matrix3d::Identity())); // Identity cov
}
