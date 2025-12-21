// Sensor Noise Models Demo
// Demonstrates IMU noise simulation using Allan variance parameters
#include <iomanip>
#include <iostream>
#include <random>
#include <vulcan/vulcan.hpp>

int main() {
    std::cout << "============================================\n";
    std::cout << "  Vulcan Sensor Noise Models Demo\n";
    std::cout << "============================================\n\n";

    // ========================================
    // 1. IMU Grade Comparison
    // ========================================
    std::cout << "=== IMU Grade Parameters ===\n\n";

    auto print_params = [](const char *name,
                           const vulcan::sensors::AllanParams<double> &p) {
        std::cout << name << ":\n";
        std::cout << "  ARW (N):     " << std::scientific
                  << std::setprecision(2) << p.N << " rad/s/√Hz\n";
        std::cout << "  Bias (B):    " << p.B << " rad/s\n";
        std::cout << "  RRW (K):     " << p.K << " rad/s²/√Hz\n";
        std::cout << "  Corr. time:  " << std::fixed << std::setprecision(1)
                  << p.tau_bias << " s\n\n";
    };

    print_params("Consumer-grade gyro",
                 vulcan::sensors::imu_grades::consumer_gyro());
    print_params("Industrial-grade gyro",
                 vulcan::sensors::imu_grades::industrial_gyro());
    print_params("Tactical-grade gyro",
                 vulcan::sensors::imu_grades::tactical_gyro());
    print_params("Navigation-grade gyro",
                 vulcan::sensors::imu_grades::navigation_gyro());

    // ========================================
    // 2. Numeric IMU Noise Simulation
    // ========================================
    std::cout << "=== IMU Noise Time Series ===\n\n";

    double dt = 0.01; // 100 Hz
    int num_steps = 1000;

    // Initialize random number generator
    std::mt19937 rng(42);
    std::normal_distribution<double> dist(0.0, 1.0);

    // Create consumer-grade IMU noise model
    auto state = vulcan::allan::init_state<double>();
    auto coeffs = vulcan::allan::consumer_grade_coeffs(dt);

    std::cout << "Consumer-grade IMU at 100 Hz for 10s:\n";
    std::cout << "  Time[s]  Gyro_X[rad/s]  Gyro_Y[rad/s]  Gyro_Z[rad/s]\n";

    for (int i = 0; i < num_steps; ++i) {
        // Generate noise input
        vulcan::allan::IMUNoiseInput<double> input;
        for (int j = 0; j < 3; ++j) {
            input.gyro_arw(j) = dist(rng);
            input.gyro_bias(j) = dist(rng);
            input.gyro_rrw(j) = dist(rng);
            input.accel_arw(j) = dist(rng);
            input.accel_bias(j) = dist(rng);
            input.accel_rrw(j) = dist(rng);
        }

        // Step the noise model
        auto noise = vulcan::allan::step(state, coeffs, input);

        // Print every 100 steps
        if (i % 100 == 0) {
            double t = i * dt;
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "  " << std::setw(6) << t;
            std::cout << std::scientific << std::setprecision(4);
            std::cout << "  " << std::setw(13) << noise.gyro(0);
            std::cout << "  " << std::setw(13) << noise.gyro(1);
            std::cout << "  " << std::setw(13) << noise.gyro(2);
            std::cout << "\n";
        }
    }
    std::cout << "\n";

    // ========================================
    // 3. Bias Accumulation Demo
    // ========================================
    std::cout << "=== Bias Instability Accumulation ===\n\n";

    auto bias_state = vulcan::bias_instability::init_state<double>();
    double sigma_b = 1e-5; // rad/s
    double tau = 100.0;    // 100 second correlation time
    auto bias_coeffs =
        vulcan::bias_instability::compute_coeffs(sigma_b, tau, dt);

    std::cout << "Bias evolution (σ=" << sigma_b << " rad/s, τ=" << tau
              << "s):\n";
    std::cout << "  Time[s]    Bias[rad/s]\n";

    double max_bias = 0.0;
    for (int i = 0; i < 10000; ++i) {
        double bias =
            vulcan::bias_instability::step(bias_state, bias_coeffs, dist(rng));
        if (std::abs(bias) > std::abs(max_bias)) {
            max_bias = bias;
        }

        if (i % 1000 == 0) {
            double t = i * dt;
            std::cout << std::fixed << std::setprecision(1);
            std::cout << "  " << std::setw(7) << t;
            std::cout << std::scientific << std::setprecision(4);
            std::cout << "    " << std::setw(12) << bias << "\n";
        }
    }
    std::cout << "\n  Max bias reached: " << std::scientific << max_bias
              << " rad/s\n\n";

    // ========================================
    // 4. Random Walk Drift
    // ========================================
    std::cout << "=== Random Walk (Drift) Demo ===\n\n";

    auto rw_state = vulcan::random_walk::init_state<double>();
    double K = 1e-6; // rad/s²/√Hz
    auto rw_coeffs = vulcan::random_walk::compute_coeffs(K, dt);

    std::cout << "Position drift from rate random walk (K=" << K
              << " rad/s²/√Hz):\n";
    std::cout << "  Time[s]    Drift[rad/s]    Expected_σ[rad/s]\n";

    for (int i = 0; i <= 10000; i += 2000) {
        // Reset and run to this time
        auto temp_state = vulcan::random_walk::init_state<double>();
        for (int j = 0; j < i; ++j) {
            vulcan::random_walk::step(temp_state, rw_coeffs, dist(rng));
        }

        double t = i * dt;
        double expected_sigma = vulcan::random_walk::expected_stddev(K, t);

        std::cout << std::fixed << std::setprecision(1);
        std::cout << "  " << std::setw(7) << t;
        std::cout << std::scientific << std::setprecision(4);
        std::cout << "    " << std::setw(12) << temp_state.value;
        std::cout << "        " << std::setw(12) << expected_sigma << "\n";
    }
    std::cout << "\n";

    // ========================================
    // 5. Symbolic Mode: Graph Generation
    // ========================================
    std::cout << "=== Symbolic Mode Demo ===\n\n";

    using Scalar = janus::SymbolicScalar;

    // Create symbolic noise inputs
    auto w1 = janus::sym("w_arw");
    auto w2 = janus::sym("w_bias");
    auto w3 = janus::sym("w_rrw");

    // Initialize symbolic state
    auto sym_state = vulcan::allan::init_axis_state<Scalar>();
    auto sym_coeffs = vulcan::allan::compute_axis_coeffs(
        vulcan::sensors::imu_grades::industrial_gyro(), dt);

    // Step to get symbolic output
    auto noise_output =
        vulcan::allan::step_axis(sym_state, sym_coeffs, w1, w2, w3);

    std::cout << "Symbolic noise expression created.\n";
    std::cout << "Output depends on: w_arw, w_bias, w_rrw\n";

    // Create Janus function
    janus::Function f("imu_noise", {w1, w2, w3}, {noise_output});

    // Evaluate at specific values
    auto result = f({1.0, 0.5, 0.2});
    std::cout << "Evaluated at (1.0, 0.5, 0.2): " << result[0](0, 0) << "\n\n";

    // Export graph
    janus::export_graph_html(noise_output, "graph_imu_noise", "IMU_Noise");
    std::cout << "Generated: graph_imu_noise.html\n\n";

    // ========================================
    // 6. First-Order Markov Process
    // ========================================
    std::cout << "=== First-Order Markov Process ===\n\n";

    double tau_markov = 10.0;
    double sigma_markov = 0.1;
    auto markov_coeffs =
        vulcan::markov::discretize(tau_markov, sigma_markov, dt);

    std::cout << "Process parameters:\n";
    std::cout << "  τ = " << tau_markov << " s\n";
    std::cout << "  σ = " << sigma_markov << "\n";
    std::cout << "  φ = " << markov_coeffs.phi << " (state transition)\n";
    std::cout << "  q = " << markov_coeffs.q << " (noise gain)\n\n";

    std::cout << "Theoretical properties:\n";
    std::cout << "  PSD at DC:           "
              << vulcan::markov::psd_at_frequency(tau_markov, sigma_markov, 0.0)
              << "\n";
    std::cout << "  Autocorr at τ:       "
              << vulcan::markov::autocorrelation(tau_markov, sigma_markov,
                                                 tau_markov)
              << "\n";
    std::cout << "  Process noise PSD:   "
              << vulcan::markov::process_noise_psd(tau_markov, sigma_markov)
              << "\n\n";

    std::cout << "============================================\n";
    std::cout << "  Demo Complete!\n";
    std::cout << "============================================\n";

    return 0;
}
