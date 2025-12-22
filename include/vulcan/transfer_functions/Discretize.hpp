#pragma once

#include <janus/janus.hpp>

namespace vulcan::tf {

// =============================================================================
// Matrix Exponential (Padé approximation)
// =============================================================================

/**
 * @brief Compute matrix exponential using Padé(13,13) approximation.
 *
 * @tparam N Matrix dimension
 * @param A Input matrix
 * @return Eigen::Matrix<double, N, N> exp(A)
 */
template <int N>
Eigen::Matrix<double, N, N>
matrix_exp_pade(const Eigen::Matrix<double, N, N> &A) {
    static constexpr double b[] = {64764752532480000.0,
                                   32382376266240000.0,
                                   7771770303897600.0,
                                   1187353796428800.0,
                                   129060195264000.0,
                                   10559470521600.0,
                                   670442572800.0,
                                   33522128640.0,
                                   1323241920.0,
                                   40840800.0,
                                   960960.0,
                                   16380.0,
                                   182.0,
                                   1.0};

    double norm_A = A.cwiseAbs().colwise().sum().maxCoeff();
    int s = 0;
    if (norm_A > 0) {
        s = std::max(0, static_cast<int>(std::ceil(std::log2(norm_A / 5.37))));
    }
    Eigen::Matrix<double, N, N> As = A / std::pow(2.0, s);

    Eigen::Matrix<double, N, N> I = Eigen::Matrix<double, N, N>::Identity();
    Eigen::Matrix<double, N, N> A2 = As * As;
    Eigen::Matrix<double, N, N> A4 = A2 * A2;
    Eigen::Matrix<double, N, N> A6 = A4 * A2;

    Eigen::Matrix<double, N, N> U = b[13] * A6 + b[11] * A4 + b[9] * A2;
    U = A6 * U;
    U = U + b[7] * A6 + b[5] * A4 + b[3] * A2 + b[1] * I;
    U = As * U;

    Eigen::Matrix<double, N, N> V = b[12] * A6 + b[10] * A4 + b[8] * A2;
    V = A6 * V;
    V = V + b[6] * A6 + b[4] * A4 + b[2] * A2 + b[0] * I;

    Eigen::Matrix<double, N, N> P = (V - U).fullPivLu().solve(V + U);

    for (int i = 0; i < s; ++i) {
        P = P * P;
    }
    return P;
}

// =============================================================================
// Discretization Methods
// =============================================================================

/**
 * @brief Zero-order hold (ZOH) discretization.
 *
 * @tparam N State dimension
 * @param A_c Continuous state matrix
 * @param B_c Continuous input matrix
 * @param dt Sample time [s]
 * @param A_d Output discrete state matrix
 * @param B_d Output discrete input matrix
 */
template <int N>
void discretize_zoh(const Eigen::Matrix<double, N, N> &A_c,
                    const Eigen::Matrix<double, N, 1> &B_c, double dt,
                    Eigen::Matrix<double, N, N> &A_d,
                    Eigen::Matrix<double, N, 1> &B_d) {
    constexpr int M = N + 1;
    Eigen::Matrix<double, M, M> aug = Eigen::Matrix<double, M, M>::Zero();
    aug.template topLeftCorner<N, N>() = A_c * dt;
    aug.template block<N, 1>(0, N) = B_c * dt;

    Eigen::Matrix<double, M, M> exp_aug = matrix_exp_pade<M>(aug);
    A_d = exp_aug.template topLeftCorner<N, N>();
    B_d = exp_aug.template block<N, 1>(0, N);
}

/**
 * @brief Tustin (bilinear) discretization.
 *
 * @tparam N State dimension
 */
template <int N>
void discretize_tustin(const Eigen::Matrix<double, N, N> &A_c,
                       const Eigen::Matrix<double, N, 1> &B_c, double dt,
                       Eigen::Matrix<double, N, N> &A_d,
                       Eigen::Matrix<double, N, 1> &B_d) {
    Eigen::Matrix<double, N, N> I = Eigen::Matrix<double, N, N>::Identity();
    Eigen::Matrix<double, N, N> A_half = A_c * (dt / 2.0);
    Eigen::FullPivLU<Eigen::Matrix<double, N, N>> lu(I - A_half);
    A_d = lu.solve(I + A_half);
    B_d = lu.solve(B_c * dt);
}

/**
 * @brief Forward Euler discretization.
 *
 * @tparam N State dimension
 */
template <int N>
void discretize_euler(const Eigen::Matrix<double, N, N> &A_c,
                      const Eigen::Matrix<double, N, 1> &B_c, double dt,
                      Eigen::Matrix<double, N, N> &A_d,
                      Eigen::Matrix<double, N, 1> &B_d) {
    Eigen::Matrix<double, N, N> I = Eigen::Matrix<double, N, N>::Identity();
    A_d = I + A_c * dt;
    B_d = B_c * dt;
}

/**
 * @brief Backward Euler discretization.
 *
 * @tparam N State dimension
 */
template <int N>
void discretize_backward_euler(const Eigen::Matrix<double, N, N> &A_c,
                               const Eigen::Matrix<double, N, 1> &B_c,
                               double dt, Eigen::Matrix<double, N, N> &A_d,
                               Eigen::Matrix<double, N, 1> &B_d) {
    Eigen::Matrix<double, N, N> I = Eigen::Matrix<double, N, N>::Identity();
    Eigen::FullPivLU<Eigen::Matrix<double, N, N>> lu(I - A_c * dt);
    A_d = lu.solve(I);
    B_d = A_d * B_c * dt;
}

} // namespace vulcan::tf
