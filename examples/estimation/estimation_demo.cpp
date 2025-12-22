// Estimation Demo - State estimation using Kalman filters
// Demonstrates KF, EKF, and UKF for position/velocity tracking

#include <iomanip>
#include <iostream>
#include <random>
#include <vulcan/estimation/Estimation.hpp>

using namespace vulcan::estimation;

// =============================================================================
// Linear Kalman Filter Demo - Constant Velocity Tracking
// =============================================================================

void run_kf_demo() {
    std::cout << "=== Linear Kalman Filter Demo ===\n";
    std::cout << "Tracking a target moving at constant velocity\n\n";

    constexpr int N = 2; // State: [position, velocity]
    constexpr int M = 1; // Measurement: position only
    double dt = 0.1;

    // State transition matrix: x' = x + v*dt, v' = v
    Eigen::Matrix<double, N, N> F;
    F << 1.0, dt, 0.0, 1.0;

    // Measurement matrix: observe position
    Eigen::Matrix<double, M, N> H;
    H << 1.0, 0.0;

    // Noise specifications
    auto Q = ProcessNoise<N>::diagonal(Eigen::Vector2d(0.01, 0.01));
    auto R = MeasurementNoise<M>::scalar(1.0);

    // True target state
    double true_pos = 0.0;
    double true_vel = 5.0; // m/s

    // Initialize filter with wrong guess
    FilterState<double, N> state;
    state.x << 10.0, 0.0; // Guess: 10m, 0 m/s
    state.P = Eigen::Matrix2d::Identity() * 100.0;

    std::default_random_engine rng(42);
    std::normal_distribution<double> noise(0.0, 1.0);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Step | True Pos | Est Pos  | Est Vel  | Pos Error\n";
    std::cout << "-----|----------|----------|----------|----------\n";

    for (int i = 0; i < 20; ++i) {
        // Predict
        state = kf_predict<double, N>(state, F, Q);

        // Simulate truth and noisy measurement
        true_pos += true_vel * dt;
        double measurement = true_pos + noise(rng);

        Eigen::Matrix<double, M, 1> z;
        z << measurement;

        // Update
        state = kf_update<double, N, M>(state, z, H, R);

        double error = std::abs(state.x(0) - true_pos);
        std::cout << std::setw(4) << i << " | " << std::setw(8) << true_pos
                  << " | " << std::setw(8) << state.x(0) << " | "
                  << std::setw(8) << state.x(1) << " | " << std::setw(8)
                  << error << "\n";
    }

    std::cout << "\nFinal covariance diagonal: [" << state.P(0, 0) << ", "
              << state.P(1, 1) << "]\n\n";
}

// =============================================================================
// Extended Kalman Filter Demo - Nonlinear Bearing-Range Tracking
// =============================================================================

void run_ekf_demo() {
    std::cout << "=== Extended Kalman Filter Demo ===\n";
    std::cout << "Tracking with range-bearing measurements\n\n";

    constexpr int N = 2; // State: [x, y]
    constexpr int M = 2; // Measurement: [range, bearing]

    // Noise
    auto Q = ProcessNoise<N>::scalar(0.01);
    auto R = MeasurementNoise<M>::diagonal(Eigen::Vector2d(0.1, 0.01));

    // Nonlinear measurement model
    auto h = [](const Eigen::Vector2d &x) -> Eigen::Vector2d {
        double range = std::sqrt(x(0) * x(0) + x(1) * x(1));
        double bearing = std::atan2(x(1), x(0));
        return Eigen::Vector2d(range, bearing);
    };

    // Jacobian of measurement
    auto H_jacobian =
        [](const Eigen::Vector2d &x) -> Eigen::Matrix<double, M, N> {
        double r = std::sqrt(x(0) * x(0) + x(1) * x(1));
        double r2 = r * r;
        Eigen::Matrix<double, M, N> H;
        H(0, 0) = x(0) / r;
        H(0, 1) = x(1) / r;
        H(1, 0) = -x(1) / r2;
        H(1, 1) = x(0) / r2;
        return H;
    };

    // True target position
    Eigen::Vector2d true_pos(10.0, 10.0);
    Eigen::Vector2d true_meas = h(true_pos);

    // Initialize filter (uncertain)
    FilterState<double, N> state;
    state.x << 8.0, 12.0; // Initial guess
    state.P = Eigen::Matrix2d::Identity() * 5.0;

    std::default_random_engine rng(42);
    std::normal_distribution<double> range_noise(0.0, std::sqrt(0.1));
    std::normal_distribution<double> bearing_noise(0.0, std::sqrt(0.01));

    std::cout << "Step | True (x,y)       | Est (x,y)        | Error\n";
    std::cout << "-----|------------------|------------------|-------\n";

    for (int i = 0; i < 10; ++i) {
        // Noisy measurement
        Eigen::Vector2d z(true_meas(0) + range_noise(rng),
                          true_meas(1) + bearing_noise(rng));

        auto H = H_jacobian(state.x);
        state = ekf_update<double, N, M>(state, z, h, H, R);

        double error = (state.x - true_pos).norm();
        std::cout << std::setw(4) << i << " | (" << std::setw(6) << true_pos(0)
                  << ", " << std::setw(6) << true_pos(1) << ") | ("
                  << std::setw(6) << state.x(0) << ", " << std::setw(6)
                  << state.x(1) << ") | " << std::setw(5) << error << "\n";
    }
    std::cout << "\n";
}

// =============================================================================
// UKF Demo - Same nonlinear problem, no Jacobians needed
// =============================================================================

void run_ukf_demo() {
    std::cout << "=== Unscented Kalman Filter Demo ===\n";
    std::cout << "Same tracking problem, no Jacobians required\n\n";

    constexpr int N = 2;
    constexpr int M = 2;

    auto Q = ProcessNoise<N>::scalar(0.01);
    auto R = MeasurementNoise<M>::diagonal(Eigen::Vector2d(0.1, 0.01));

    auto h = [](const Eigen::Vector2d &x) -> Eigen::Vector2d {
        double range = std::sqrt(x(0) * x(0) + x(1) * x(1));
        double bearing = std::atan2(x(1), x(0));
        return Eigen::Vector2d(range, bearing);
    };

    Eigen::Vector2d true_pos(10.0, 10.0);
    Eigen::Vector2d true_meas = h(true_pos);

    FilterState<double, N> state;
    state.x << 8.0, 12.0;
    state.P = Eigen::Matrix2d::Identity() * 5.0;

    std::default_random_engine rng(42);
    std::normal_distribution<double> range_noise(0.0, std::sqrt(0.1));
    std::normal_distribution<double> bearing_noise(0.0, std::sqrt(0.01));

    UKFParams params;
    params.alpha = 0.1;

    std::cout << "Step | True (x,y)       | Est (x,y)        | Error\n";
    std::cout << "-----|------------------|------------------|-------\n";

    for (int i = 0; i < 10; ++i) {
        Eigen::Vector2d z(true_meas(0) + range_noise(rng),
                          true_meas(1) + bearing_noise(rng));

        state = ukf_update<N, M>(state, z, h, R, params);

        double error = (state.x - true_pos).norm();
        std::cout << std::setw(4) << i << " | (" << std::setw(6) << true_pos(0)
                  << ", " << std::setw(6) << true_pos(1) << ") | ("
                  << std::setw(6) << state.x(0) << ", " << std::setw(6)
                  << state.x(1) << ") | " << std::setw(5) << error << "\n";
    }
    std::cout << "\n";
}

// =============================================================================
// Measurement Gating Demo
// =============================================================================

void run_gating_demo() {
    std::cout << "=== Measurement Gating Demo ===\n";
    std::cout << "Rejecting outliers using chi-squared threshold\n\n";

    Eigen::Vector2d z_expected(10.0, 20.0);
    Eigen::Matrix2d S = Eigen::Matrix2d::Identity() * 1.0;

    double threshold_95 = chi_squared_threshold(2, 0.95);
    std::cout << "Chi-squared threshold (95%, dof=2): " << threshold_95
              << "\n\n";

    std::vector<Eigen::Vector2d> measurements = {
        {10.1, 20.1}, // Good
        {10.5, 19.5}, // Good
        {15.0, 25.0}, // Outlier
        {10.2, 20.3}, // Good
        {50.0, 50.0}  // Major outlier
    };

    for (size_t i = 0; i < measurements.size(); ++i) {
        const auto &z = measurements[i];
        Eigen::Vector2d innovation = z - z_expected;
        double nis = normalized_innovation_squared<double, 2>(innovation, S);
        bool passes = passes_gate<double, 2>(innovation, S, threshold_95);

        std::cout << "Measurement " << i << ": (" << z(0) << ", " << z(1)
                  << ") -> NIS=" << std::fixed << std::setprecision(2) << nis
                  << " -> " << (passes ? "ACCEPT" : "REJECT") << "\n";
    }
    std::cout << "\n";
}

int main() {
    run_kf_demo();
    run_ekf_demo();
    run_ukf_demo();
    run_gating_demo();
    return 0;
}
