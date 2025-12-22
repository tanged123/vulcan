#pragma once

#include <janus/janus.hpp>

namespace vulcan::tf {

/**
 * @brief Second-order discrete-time system coefficients.
 *
 * Continuous form: ÿ + 2ζω_n*ẏ + ω_n²*y = K*ω_n²*u
 *
 * State-space representation with state x = [y, ẏ]ᵀ
 *
 * @tparam Scalar Variable type (e.g., double or casadi::MX)
 */
template <typename Scalar> struct SecondOrderCoeffs {
    Eigen::Matrix<Scalar, 2, 2> A; ///< Discrete state matrix
    Eigen::Matrix<Scalar, 2, 1> B; ///< Discrete input matrix
    Eigen::Matrix<Scalar, 1, 2> C; ///< Output matrix
    Scalar D;                      ///< Feedthrough (usually 0)
    double omega_n;                ///< Natural frequency [rad/s]
    double zeta;                   ///< Damping ratio
    double K;                      ///< DC gain
    double dt;                     ///< Sample time [s]
};

/**
 * @brief Compute second-order discrete-time system coefficients.
 *
 * Uses ZOH discretization via closed-form solution.
 *
 * @tparam Scalar Variable type
 * @param omega_n Natural frequency [rad/s] (must be positive)
 * @param zeta Damping ratio (0 < zeta for stability)
 * @param K DC gain
 * @param dt Sample time [s] (must be positive)
 * @return SecondOrderCoeffs<Scalar> The discretized coefficients
 */
template <typename Scalar>
SecondOrderCoeffs<Scalar> second_order(double omega_n, double zeta, double K,
                                       double dt) {
    SecondOrderCoeffs<Scalar> c;
    c.omega_n = omega_n;
    c.zeta = zeta;
    c.K = K;
    c.dt = dt;

    double wn = omega_n;
    double z = zeta;
    double wn2 = wn * wn;

    if (z < 1.0) {
        // Underdamped
        double wd = wn * std::sqrt(1.0 - z * z);
        double sigma = z * wn;
        double e_sigma = std::exp(-sigma * dt);
        double cos_wd = std::cos(wd * dt);
        double sin_wd = std::sin(wd * dt);

        c.A(0, 0) =
            static_cast<Scalar>(e_sigma * (cos_wd + (sigma / wd) * sin_wd));
        c.A(0, 1) = static_cast<Scalar>(e_sigma * sin_wd / wd);
        c.A(1, 0) = static_cast<Scalar>(-e_sigma * wn2 * sin_wd / wd);
        c.A(1, 1) =
            static_cast<Scalar>(e_sigma * (cos_wd - (sigma / wd) * sin_wd));

        double b0 = K * (1.0 - e_sigma * (cos_wd + (sigma / wd) * sin_wd));
        double b1 = K * e_sigma * wn2 * sin_wd / wd;
        c.B(0) = static_cast<Scalar>(b0);
        c.B(1) = static_cast<Scalar>(b1);

    } else if (z > 1.0) {
        // Overdamped
        double sqrtTerm = wn * std::sqrt(z * z - 1.0);
        double p1 = -z * wn + sqrtTerm;
        double p2 = -z * wn - sqrtTerm;
        double e1 = std::exp(p1 * dt);
        double e2 = std::exp(p2 * dt);
        double dp = p1 - p2;

        c.A(0, 0) = static_cast<Scalar>((p1 * e2 - p2 * e1) / dp);
        c.A(0, 1) = static_cast<Scalar>((e1 - e2) / dp);
        c.A(1, 0) = static_cast<Scalar>(p1 * p2 * (e2 - e1) / dp);
        c.A(1, 1) = static_cast<Scalar>((p1 * e1 - p2 * e2) / dp);

        double b0 = K * (1.0 - (p1 * e2 - p2 * e1) / dp);
        double b1 = K * p1 * p2 * (e1 - e2) / dp;
        c.B(0) = static_cast<Scalar>(b0);
        c.B(1) = static_cast<Scalar>(b1);

    } else {
        // Critically damped
        double e_wn = std::exp(-wn * dt);

        c.A(0, 0) = static_cast<Scalar>(e_wn * (1.0 + wn * dt));
        c.A(0, 1) = static_cast<Scalar>(e_wn * dt);
        c.A(1, 0) = static_cast<Scalar>(-e_wn * wn * wn * dt);
        c.A(1, 1) = static_cast<Scalar>(e_wn * (1.0 - wn * dt));

        double b0 = K * (1.0 - e_wn * (1.0 + wn * dt));
        double b1 = K * e_wn * wn * wn * dt;
        c.B(0) = static_cast<Scalar>(b0);
        c.B(1) = static_cast<Scalar>(b1);
    }

    c.C(0, 0) = static_cast<Scalar>(1.0);
    c.C(0, 1) = static_cast<Scalar>(0.0);
    c.D = static_cast<Scalar>(0.0);

    return c;
}

/**
 * @brief Compute next state for second-order system (stateless).
 *
 * Computes: x[k+1] = A*x[k] + B*u[k]
 *
 * @tparam Scalar Variable type
 * @param coeffs The second-order coefficients
 * @param state Current state [y, ẏ]ᵀ
 * @param input Current input u[k]
 * @return Eigen::Matrix<Scalar, 2, 1> Next state
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 2, 1>
second_order_step(const SecondOrderCoeffs<Scalar> &coeffs,
                  const Eigen::Matrix<Scalar, 2, 1> &state,
                  const Scalar &input) {
    return coeffs.A * state + coeffs.B * input;
}

/**
 * @brief Extract output from state (stateless).
 *
 * Computes: y = C*x + D*u
 *
 * @tparam Scalar Variable type
 * @param coeffs The second-order coefficients
 * @param state Current state [y, ẏ]ᵀ
 * @param input Current input u[k]
 * @return Scalar Output y
 */
template <typename Scalar>
Scalar second_order_output(const SecondOrderCoeffs<Scalar> &coeffs,
                           const Eigen::Matrix<Scalar, 2, 1> &state,
                           const Scalar &input) {
    return (coeffs.C * state)(0) + coeffs.D * input;
}

/**
 * @brief Compute theoretical system characteristics.
 *
 * @param omega_n Natural frequency [rad/s]
 * @param zeta Damping ratio
 * @return std::tuple<double, double, double> {rise_time, settling_time,
 * overshoot_percent}
 */
inline std::tuple<double, double, double>
second_order_characteristics(double omega_n, double zeta) {
    double tr = (1.8 - 0.6 * zeta) / omega_n;
    double ts = 4.0 / (zeta * omega_n);
    double Mp = 0.0;
    if (zeta < 1.0 && zeta > 0.0) {
        Mp = 100.0 * std::exp(-zeta * M_PI / std::sqrt(1.0 - zeta * zeta));
    }
    return {tr, ts, Mp};
}

} // namespace vulcan::tf
