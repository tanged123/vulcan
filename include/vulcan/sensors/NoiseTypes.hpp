// Sensor Noise Model Types
// Common types and parameter structures for IMU noise modeling
#pragma once

#include <Eigen/Core>

namespace vulcan::sensors {

// =============================================================================
// Allan Variance Parameters
// =============================================================================

/**
 * @brief Allan variance noise parameters for IMU sensors
 *
 * These parameters characterize the noise floor of inertial sensors as
 * identified from Allan variance analysis:
 * - N: Angle/Velocity Random Walk (slope -1/2 on Allan deviation plot)
 * - B: Bias Instability (slope 0, the flat region minimum)
 * - K: Rate Random Walk (slope +1/2)
 * - tau_bias: Correlation time for bias instability process
 *
 * @tparam Scalar double or casadi::MX
 */
template <typename Scalar> struct AllanParams {
    Scalar N;        ///< Angle/velocity random walk [rad/s/√Hz] or [m/s²/√Hz]
    Scalar B;        ///< Bias instability [rad/s] or [m/s²]
    Scalar K;        ///< Rate random walk [rad/s²/√Hz] or [m/s³/√Hz]
    Scalar tau_bias; ///< Bias instability correlation time [s]
};

// =============================================================================
// First-Order Markov Process Parameters
// =============================================================================

/**
 * @brief First-order Gauss-Markov process parameters
 *
 * Characterizes exponentially correlated noise:
 *   dx/dt = -x/τ + w(t)
 * where w(t) is white noise with variance chosen to give steady-state σ².
 *
 * @tparam Scalar double or casadi::MX
 */
template <typename Scalar> struct MarkovParams {
    Scalar sigma; ///< Steady-state standard deviation
    Scalar tau;   ///< Correlation time [s]
};

// =============================================================================
// IMU Noise Sample Output
// =============================================================================

/**
 * @brief 3-axis IMU noise sample
 *
 * Represents noise contributions to add to ideal sensor outputs.
 *
 * @tparam Scalar double or casadi::MX
 */
template <typename Scalar> struct IMUNoiseSample {
    Eigen::Vector<Scalar, 3> gyro;  ///< Gyroscope noise [rad/s]
    Eigen::Vector<Scalar, 3> accel; ///< Accelerometer noise [m/s²]
};

// =============================================================================
// IMU Grade Presets
// =============================================================================

namespace imu_grades {

/**
 * @brief Consumer-grade MEMS gyroscope (smartphone-level)
 *
 * Typical values:
 * - ARW: 0.3 deg/√hr ≈ 8.7e-5 rad/s/√Hz
 * - Bias instability: 10 deg/hr ≈ 4.8e-5 rad/s
 * - Rate random walk: 0.5 deg/hr/√hr ≈ 4.0e-8 rad/s²/√Hz
 * - τ_bias: ~100s
 */
inline AllanParams<double> consumer_gyro() {
    return AllanParams<double>{
        .N = 8.7e-5,       // rad/s/√Hz
        .B = 4.8e-5,       // rad/s
        .K = 4.0e-8,       // rad/s²/√Hz
        .tau_bias = 100.0, // s
    };
}

/**
 * @brief Consumer-grade MEMS accelerometer (smartphone-level)
 */
inline AllanParams<double> consumer_accel() {
    return AllanParams<double>{
        .N = 3.0e-3,       // m/s²/√Hz (300 µg/√Hz)
        .B = 5.0e-4,       // m/s² (50 µg)
        .K = 1.0e-5,       // m/s³/√Hz
        .tau_bias = 100.0, // s
    };
}

/**
 * @brief Industrial-grade MEMS gyroscope
 *
 * Typical values:
 * - ARW: 0.1 deg/√hr ≈ 2.9e-5 rad/s/√Hz
 * - Bias instability: 1 deg/hr ≈ 4.8e-6 rad/s
 */
inline AllanParams<double> industrial_gyro() {
    return AllanParams<double>{
        .N = 2.9e-5,       // rad/s/√Hz
        .B = 4.8e-6,       // rad/s
        .K = 1.0e-8,       // rad/s²/√Hz
        .tau_bias = 300.0, // s
    };
}

/**
 * @brief Industrial-grade MEMS accelerometer
 */
inline AllanParams<double> industrial_accel() {
    return AllanParams<double>{
        .N = 1.0e-3,       // m/s²/√Hz (100 µg/√Hz)
        .B = 1.0e-4,       // m/s² (10 µg)
        .K = 3.0e-6,       // m/s³/√Hz
        .tau_bias = 300.0, // s
    };
}

/**
 * @brief Tactical-grade IMU gyroscope
 *
 * Typical values:
 * - ARW: 0.01 deg/√hr ≈ 2.9e-6 rad/s/√Hz
 * - Bias instability: 0.1 deg/hr ≈ 4.8e-7 rad/s
 */
inline AllanParams<double> tactical_gyro() {
    return AllanParams<double>{
        .N = 2.9e-6,        // rad/s/√Hz
        .B = 4.8e-7,        // rad/s
        .K = 1.0e-9,        // rad/s²/√Hz
        .tau_bias = 1000.0, // s
    };
}

/**
 * @brief Tactical-grade IMU accelerometer
 */
inline AllanParams<double> tactical_accel() {
    return AllanParams<double>{
        .N = 3.0e-4,        // m/s²/√Hz (30 µg/√Hz)
        .B = 3.0e-5,        // m/s² (3 µg)
        .K = 1.0e-6,        // m/s³/√Hz
        .tau_bias = 1000.0, // s
    };
}

/**
 * @brief Navigation-grade IMU gyroscope
 *
 * High-performance fiber optic or ring laser gyros:
 * - ARW: 0.001 deg/√hr ≈ 2.9e-7 rad/s/√Hz
 * - Bias instability: 0.01 deg/hr ≈ 4.8e-8 rad/s
 */
inline AllanParams<double> navigation_gyro() {
    return AllanParams<double>{
        .N = 2.9e-7,        // rad/s/√Hz
        .B = 4.8e-8,        // rad/s
        .K = 1.0e-10,       // rad/s²/√Hz
        .tau_bias = 3600.0, // s (1 hour)
    };
}

/**
 * @brief Navigation-grade IMU accelerometer
 */
inline AllanParams<double> navigation_accel() {
    return AllanParams<double>{
        .N = 5.0e-5,        // m/s²/√Hz (5 µg/√Hz)
        .B = 1.0e-5,        // m/s² (1 µg)
        .K = 1.0e-7,        // m/s³/√Hz
        .tau_bias = 3600.0, // s (1 hour)
    };
}

} // namespace imu_grades

} // namespace vulcan::sensors
