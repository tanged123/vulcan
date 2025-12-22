#include <cmath>
#include <gtest/gtest.h>
#include <vulcan/transfer_functions/Discretize.hpp>

using namespace vulcan::tf;

TEST(DiscretizeTest, MatrixExpIdentity) {
    Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
    Eigen::Matrix2d expA = matrix_exp_pade(A);
    EXPECT_NEAR(expA(0, 0), 1.0, 1e-12);
    EXPECT_NEAR(expA(1, 1), 1.0, 1e-12);
}

TEST(DiscretizeTest, MatrixExpDiagonal) {
    Eigen::Matrix2d A;
    A << -1.0, 0.0, 0.0, -2.0;
    Eigen::Matrix2d expA = matrix_exp_pade(A);
    EXPECT_NEAR(expA(0, 0), std::exp(-1.0), 1e-12);
    EXPECT_NEAR(expA(1, 1), std::exp(-2.0), 1e-12);
}

TEST(DiscretizeTest, ZOHFirstOrder) {
    Eigen::Matrix<double, 1, 1> A_c, A_d, B_c, B_d;
    A_c << -1.0;
    B_c << 1.0;

    discretize_zoh<1>(A_c, B_c, 0.1, A_d, B_d);

    EXPECT_NEAR(A_d(0, 0), std::exp(-0.1), 1e-12);
    EXPECT_NEAR(B_d(0, 0), 1.0 - std::exp(-0.1), 1e-12);
}

TEST(DiscretizeTest, ZOHOscillator) {
    Eigen::Matrix2d A_c, A_d;
    Eigen::Vector2d B_c, B_d;
    A_c << 0.0, 1.0, -1.0, 0.0;
    B_c << 0.0, 1.0;

    discretize_zoh<2>(A_c, B_c, 0.1, A_d, B_d);

    EXPECT_NEAR(A_d(0, 0), std::cos(0.1), 1e-12);
    EXPECT_NEAR(A_d(0, 1), std::sin(0.1), 1e-12);
}

TEST(DiscretizeTest, TustinFirstOrder) {
    Eigen::Matrix<double, 1, 1> A_c, A_d, B_c, B_d;
    A_c << -1.0;
    B_c << 1.0;

    discretize_tustin<1>(A_c, B_c, 0.1, A_d, B_d);
    EXPECT_NEAR(A_d(0, 0), 0.95 / 1.05, 1e-12);
}

TEST(DiscretizeTest, EulerFirstOrder) {
    Eigen::Matrix<double, 1, 1> A_c, A_d, B_c, B_d;
    A_c << -1.0;
    B_c << 1.0;

    discretize_euler<1>(A_c, B_c, 0.1, A_d, B_d);
    EXPECT_NEAR(A_d(0, 0), 0.9, 1e-12);
    EXPECT_NEAR(B_d(0, 0), 0.1, 1e-12);
}

TEST(DiscretizeTest, BackwardEuler) {
    Eigen::Matrix<double, 1, 1> A_c, A_d, B_c, B_d;
    A_c << -1.0;
    B_c << 1.0;

    discretize_backward_euler<1>(A_c, B_c, 0.1, A_d, B_d);
    EXPECT_NEAR(A_d(0, 0), 1.0 / 1.1, 1e-12);
}
