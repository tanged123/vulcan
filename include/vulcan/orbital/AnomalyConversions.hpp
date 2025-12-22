// Vulcan Anomaly Conversions
// Mean, Eccentric, and True anomaly solvers
#pragma once

#include <cmath>
#include <janus/janus.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::orbital::anomaly {

/**
 * @brief Solve Kepler's equation: M = E - e*sin(E)
 *
 * Uses Newton-Raphson iteration. For symbolic types, uses fixed
 * iterations to enable automatic differentiation.
 *
 * @tparam Scalar double or casadi::MX
 * @param M Mean anomaly [rad]
 * @param e Eccentricity [-]
 * @param tol Convergence tolerance (numeric only)
 * @param max_iter Maximum iterations (numeric only)
 * @return Eccentric anomaly [rad]
 */
template <typename Scalar>
Scalar mean_to_eccentric(const Scalar &M, const Scalar &e, double tol = 1e-12,
                         int max_iter = 50) {
    // Initial guess
    Scalar E = M;

    if constexpr (std::is_same_v<Scalar, double>) {
        // Newton-Raphson for numeric
        for (int iter = 0; iter < max_iter; ++iter) {
            double f = E - e * std::sin(E) - M;
            double f_prime = 1.0 - e * std::cos(E);
            double dE = f / f_prime;
            E -= dE;
            if (std::abs(dE) < tol)
                break;
        }
    } else {
        // Fixed Newton steps for symbolic (enables autodiff)
        for (int iter = 0; iter < 10; ++iter) {
            Scalar f = E - e * janus::sin(E) - M;
            Scalar f_prime = 1.0 - e * janus::cos(E);
            E = E - f / f_prime;
        }
    }

    return E;
}

/**
 * @brief Convert eccentric anomaly to true anomaly
 *
 * @tparam Scalar double or casadi::MX
 * @param E Eccentric anomaly [rad]
 * @param e Eccentricity [-]
 * @return True anomaly [rad]
 */
template <typename Scalar>
Scalar eccentric_to_true(const Scalar &E, const Scalar &e) {
    const Scalar cos_E = janus::cos(E);
    const Scalar sin_E = janus::sin(E);
    const Scalar sqrt_term = janus::sqrt(1.0 - e * e);

    return janus::atan2(sqrt_term * sin_E, cos_E - e);
}

/**
 * @brief Convert true anomaly to eccentric anomaly
 *
 * @tparam Scalar double or casadi::MX
 * @param nu True anomaly [rad]
 * @param e Eccentricity [-]
 * @return Eccentric anomaly [rad]
 */
template <typename Scalar>
Scalar true_to_eccentric(const Scalar &nu, const Scalar &e) {
    const Scalar cos_nu = janus::cos(nu);
    const Scalar sin_nu = janus::sin(nu);
    const Scalar sqrt_term = janus::sqrt(1.0 - e * e);

    return janus::atan2(sqrt_term * sin_nu, e + cos_nu);
}

/**
 * @brief Convert eccentric anomaly to mean anomaly
 *
 * Direct application of Kepler's equation: M = E - e*sin(E)
 *
 * @tparam Scalar double or casadi::MX
 * @param E Eccentric anomaly [rad]
 * @param e Eccentricity [-]
 * @return Mean anomaly [rad]
 */
template <typename Scalar>
Scalar eccentric_to_mean(const Scalar &E, const Scalar &e) {
    return E - e * janus::sin(E);
}

/**
 * @brief Convert true anomaly to mean anomaly
 *
 * @tparam Scalar double or casadi::MX
 * @param nu True anomaly [rad]
 * @param e Eccentricity [-]
 * @return Mean anomaly [rad]
 */
template <typename Scalar>
Scalar true_to_mean(const Scalar &nu, const Scalar &e) {
    Scalar E = true_to_eccentric(nu, e);
    return eccentric_to_mean(E, e);
}

/**
 * @brief Convert mean anomaly to true anomaly
 *
 * @tparam Scalar double or casadi::MX
 * @param M Mean anomaly [rad]
 * @param e Eccentricity [-]
 * @return True anomaly [rad]
 */
template <typename Scalar>
Scalar mean_to_true(const Scalar &M, const Scalar &e) {
    Scalar E = mean_to_eccentric(M, e);
    return eccentric_to_true(E, e);
}

} // namespace vulcan::orbital::anomaly
