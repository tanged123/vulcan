#pragma once

#include <janus/janus.hpp>

namespace vulcan::tf {

/**
 * @brief First-order discrete-time system coefficients.
 *
 * Continuous form: τ*ẏ + y = K*u
 * Discretized (ZOH): y[k+1] = a*y[k] + b*u[k]
 *
 * Where:
 *   a = exp(-dt/τ)
 *   b = K*(1 - a)
 *
 * @tparam Scalar Variable type (e.g., double or casadi::MX)
 */
template <typename Scalar> struct FirstOrderCoeffs {
    Scalar a;   ///< State coefficient: exp(-dt/τ)
    Scalar b;   ///< Input coefficient: K*(1 - a)
    double tau; ///< Time constant [s]
    double K;   ///< DC gain
    double dt;  ///< Sample time [s]
};

/**
 * @brief Compute first-order discrete-time system coefficients.
 *
 * @tparam Scalar Variable type
 * @param tau Time constant [s] (must be positive)
 * @param K DC gain (default: 1.0)
 * @param dt Sample time [s] (must be positive)
 * @return FirstOrderCoeffs<Scalar> The discretized coefficients
 */
template <typename Scalar>
FirstOrderCoeffs<Scalar> first_order(double tau, double K, double dt) {
    double a = std::exp(-dt / tau);
    double b = K * (1.0 - a);
    return FirstOrderCoeffs<Scalar>{static_cast<Scalar>(a),
                                    static_cast<Scalar>(b), tau, K, dt};
}

/**
 * @brief Compute next state for first-order system (stateless).
 *
 * Computes: y[k+1] = a*y[k] + b*u[k]
 *
 * @tparam Scalar Variable type
 * @param coeffs The first-order coefficients
 * @param state Current state y[k]
 * @param input Current input u[k]
 * @return Scalar Next state y[k+1]
 */
template <typename Scalar>
Scalar first_order_step(const FirstOrderCoeffs<Scalar> &coeffs,
                        const Scalar &state, const Scalar &input) {
    return coeffs.a * state + coeffs.b * input;
}

/**
 * @brief Compute analytical step response at time t.
 *
 * y(t) = K * (1 - exp(-t/tau))
 *
 * @tparam Scalar Variable type
 * @param tau Time constant [s]
 * @param K DC gain
 * @param t Time [s]
 * @return Scalar Output at time t for unit step input
 */
template <typename Scalar>
Scalar first_order_response(double tau, double K, const Scalar &t) {
    return static_cast<Scalar>(K) * (1.0 - janus::exp(-t / tau));
}

} // namespace vulcan::tf
