// Allan Variance Parameterized Noise Model
// Composite IMU noise model using Allan variance parameters
#pragma once

#include <Eigen/Core>
#include <vulcan/sensors/BiasInstability.hpp>
#include <vulcan/sensors/GaussianNoise.hpp>
#include <vulcan/sensors/MarkovProcess.hpp>
#include <vulcan/sensors/NoiseTypes.hpp>
#include <vulcan/sensors/RandomWalk.hpp>

namespace vulcan::allan {

// =============================================================================
// Per-Axis Noise State
// =============================================================================

/**
 * @brief Combined noise state for a single sensor axis
 *
 * Tracks state for all noise processes:
 * - White noise (angle/velocity random walk) - stateless
 * - Bias instability (Markov process with long τ)
 * - Rate random walk (integrated white noise)
 *
 * @tparam Scalar double or casadi::MX
 */
template <typename Scalar> struct AxisState {
    bias_instability::State<Scalar> bias_state; ///< Bias instability state
    random_walk::State<Scalar> rrw_state;       ///< Rate random walk state
};

/**
 * @brief Initialize per-axis state to zero
 *
 * @tparam Scalar double or casadi::MX
 * @return AxisState with all zeros
 */
template <typename Scalar> AxisState<Scalar> init_axis_state() {
    return AxisState<Scalar>{
        .bias_state = bias_instability::init_state<Scalar>(),
        .rrw_state = random_walk::init_state<Scalar>(),
    };
}

// =============================================================================
// Per-Axis Coefficients
// =============================================================================

/**
 * @brief Pre-computed coefficients for single axis
 *
 * Contains all gains and filter coefficients needed for stepping
 * the noise model at runtime.
 */
struct AxisCoeffs {
    double arw_sigma;                     ///< White noise std dev: N/√(Δt)
    bias_instability::Coeffs bias_coeffs; ///< Bias instability coefficients
    random_walk::Coeffs rrw_coeffs;       ///< Rate random walk coefficients
};

/**
 * @brief Compute per-axis coefficients from Allan parameters
 *
 * @param params Allan variance parameters for this axis
 * @param dt Time step [s]
 * @return AxisCoeffs ready for simulation
 */
inline AxisCoeffs
compute_axis_coeffs(const sensors::AllanParams<double> &params, double dt) {
    return AxisCoeffs{
        .arw_sigma = gaussian::sigma_from_arw(params.N, dt),
        .bias_coeffs =
            bias_instability::compute_coeffs(params.B, params.tau_bias, dt),
        .rrw_coeffs = random_walk::compute_coeffs(params.K, dt),
    };
}

// =============================================================================
// Per-Axis Step Function
// =============================================================================

/**
 * @brief Step single-axis noise model
 *
 * Computes total noise as sum of:
 * - White noise (ARW): σ_arw * w1
 * - Bias instability: Markov process driven by w2
 * - Rate random walk: integrated w3
 *
 * Uses 3 independent white noise inputs for the 3 processes.
 *
 * @tparam Scalar double or casadi::MX
 * @param state Current axis state (updated in-place)
 * @param coeffs Pre-computed axis coefficients
 * @param noise_arw White noise sample for ARW
 * @param noise_bias White noise sample for bias instability
 * @param noise_rrw White noise sample for rate random walk
 * @return Total noise output for this axis
 */
template <typename Scalar>
Scalar step_axis(AxisState<Scalar> &state, const AxisCoeffs &coeffs,
                 const Scalar &noise_arw, const Scalar &noise_bias,
                 const Scalar &noise_rrw) {
    // White noise (angle/velocity random walk)
    Scalar arw = coeffs.arw_sigma * noise_arw;

    // Bias instability (Markov process)
    Scalar bias = bias_instability::step(state.bias_state, coeffs.bias_coeffs,
                                         noise_bias);

    // Rate random walk (integrated white noise)
    Scalar rrw =
        random_walk::step(state.rrw_state, coeffs.rrw_coeffs, noise_rrw);

    return arw + bias + rrw;
}

// =============================================================================
// 3-Axis IMU Noise State
// =============================================================================

/**
 * @brief Complete 3-axis IMU noise state
 *
 * @tparam Scalar double or casadi::MX
 */
template <typename Scalar> struct IMUNoiseState {
    AxisState<Scalar> gyro[3];  ///< Gyroscope noise states (x, y, z)
    AxisState<Scalar> accel[3]; ///< Accelerometer noise states (x, y, z)
};

/**
 * @brief Initialize IMU noise state
 *
 * @tparam Scalar double or casadi::MX
 * @return IMUNoiseState with all zeros
 */
template <typename Scalar> IMUNoiseState<Scalar> init_state() {
    IMUNoiseState<Scalar> state;
    for (int i = 0; i < 3; ++i) {
        state.gyro[i] = init_axis_state<Scalar>();
        state.accel[i] = init_axis_state<Scalar>();
    }
    return state;
}

// =============================================================================
// 3-Axis IMU Noise Coefficients
// =============================================================================

/**
 * @brief Complete IMU noise coefficients
 *
 * Assumes same noise parameters for all axes within gyro/accel.
 * For per-axis parameters, use step_axis directly.
 */
struct IMUNoiseCoeffs {
    AxisCoeffs gyro;  ///< Gyroscope coefficients (same for all axes)
    AxisCoeffs accel; ///< Accelerometer coefficients (same for all axes)
};

/**
 * @brief Compute IMU coefficients from Allan parameters
 *
 * @param gyro_params Allan variance parameters for gyroscope
 * @param accel_params Allan variance parameters for accelerometer
 * @param dt Time step [s]
 * @return IMUNoiseCoeffs ready for simulation
 */
inline IMUNoiseCoeffs
compute_coeffs(const sensors::AllanParams<double> &gyro_params,
               const sensors::AllanParams<double> &accel_params, double dt) {
    return IMUNoiseCoeffs{
        .gyro = compute_axis_coeffs(gyro_params, dt),
        .accel = compute_axis_coeffs(accel_params, dt),
    };
}

// =============================================================================
// 3-Axis IMU Step Function
// =============================================================================

/**
 * @brief Noise input structure for IMU step
 *
 * Contains all white noise samples needed for one time step.
 * Each axis needs 3 independent noise samples (ARW, bias, RRW).
 *
 * @tparam Scalar double or casadi::MX
 */
template <typename Scalar> struct IMUNoiseInput {
    Eigen::Vector<Scalar, 3> gyro_arw;   ///< Gyro ARW noise (3 axes)
    Eigen::Vector<Scalar, 3> gyro_bias;  ///< Gyro bias noise (3 axes)
    Eigen::Vector<Scalar, 3> gyro_rrw;   ///< Gyro RRW noise (3 axes)
    Eigen::Vector<Scalar, 3> accel_arw;  ///< Accel ARW noise (3 axes)
    Eigen::Vector<Scalar, 3> accel_bias; ///< Accel bias noise (3 axes)
    Eigen::Vector<Scalar, 3> accel_rrw;  ///< Accel RRW noise (3 axes)
};

/**
 * @brief Step the complete IMU noise model
 *
 * Updates all internal states and returns noise to add to ideal outputs.
 *
 * @tparam Scalar double or casadi::MX
 * @param state Current IMU noise state (updated in-place)
 * @param coeffs Pre-computed coefficients
 * @param input White noise samples for all processes
 * @return IMUNoiseSample with gyro and accel noise vectors
 */
template <typename Scalar>
sensors::IMUNoiseSample<Scalar> step(IMUNoiseState<Scalar> &state,
                                     const IMUNoiseCoeffs &coeffs,
                                     const IMUNoiseInput<Scalar> &input) {
    sensors::IMUNoiseSample<Scalar> output;

    // Step each gyroscope axis
    for (int i = 0; i < 3; ++i) {
        output.gyro(i) =
            step_axis(state.gyro[i], coeffs.gyro, input.gyro_arw(i),
                      input.gyro_bias(i), input.gyro_rrw(i));
    }

    // Step each accelerometer axis
    for (int i = 0; i < 3; ++i) {
        output.accel(i) =
            step_axis(state.accel[i], coeffs.accel, input.accel_arw(i),
                      input.accel_bias(i), input.accel_rrw(i));
    }

    return output;
}

/**
 * @brief Simplified step with single noise vector per sensor
 *
 * Convenience function that uses same noise for ARW and splits
 * noise between bias and RRW processes (for simpler usage when
 * full 18-sample input is not needed).
 *
 * @tparam Scalar double or casadi::MX
 * @param state Current IMU noise state (updated in-place)
 * @param coeffs Pre-computed coefficients
 * @param gyro_noise 3-element white noise for gyroscopes
 * @param accel_noise 3-element white noise for accelerometers
 * @return IMUNoiseSample with gyro and accel noise vectors
 */
template <typename Scalar>
sensors::IMUNoiseSample<Scalar>
step_simple(IMUNoiseState<Scalar> &state, const IMUNoiseCoeffs &coeffs,
            const Eigen::Vector<Scalar, 3> &gyro_noise,
            const Eigen::Vector<Scalar, 3> &accel_noise) {
    // Note: This simplified version uses correlated noise between processes
    // For proper simulation, use the full IMUNoiseInput version
    IMUNoiseInput<Scalar> input;
    input.gyro_arw = gyro_noise;
    input.gyro_bias = gyro_noise;
    input.gyro_rrw = gyro_noise;
    input.accel_arw = accel_noise;
    input.accel_bias = accel_noise;
    input.accel_rrw = accel_noise;

    return step(state, coeffs, input);
}

// =============================================================================
// Grade-Based Factory Functions
// =============================================================================

/**
 * @brief Compute coefficients for consumer-grade IMU
 *
 * @param dt Time step [s]
 * @return IMUNoiseCoeffs for consumer-grade sensors
 */
inline IMUNoiseCoeffs consumer_grade_coeffs(double dt) {
    return compute_coeffs(sensors::imu_grades::consumer_gyro(),
                          sensors::imu_grades::consumer_accel(), dt);
}

/**
 * @brief Compute coefficients for industrial-grade IMU
 *
 * @param dt Time step [s]
 * @return IMUNoiseCoeffs for industrial-grade sensors
 */
inline IMUNoiseCoeffs industrial_grade_coeffs(double dt) {
    return compute_coeffs(sensors::imu_grades::industrial_gyro(),
                          sensors::imu_grades::industrial_accel(), dt);
}

/**
 * @brief Compute coefficients for tactical-grade IMU
 *
 * @param dt Time step [s]
 * @return IMUNoiseCoeffs for tactical-grade sensors
 */
inline IMUNoiseCoeffs tactical_grade_coeffs(double dt) {
    return compute_coeffs(sensors::imu_grades::tactical_gyro(),
                          sensors::imu_grades::tactical_accel(), dt);
}

/**
 * @brief Compute coefficients for navigation-grade IMU
 *
 * @param dt Time step [s]
 * @return IMUNoiseCoeffs for navigation-grade sensors
 */
inline IMUNoiseCoeffs navigation_grade_coeffs(double dt) {
    return compute_coeffs(sensors::imu_grades::navigation_gyro(),
                          sensors::imu_grades::navigation_accel(), dt);
}

} // namespace vulcan::allan
