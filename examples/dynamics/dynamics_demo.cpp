// Vulcan Dynamics Demo
// Showcases usage of 3-DOF, Pseudo-5DOF, and 6-DOF dynamics models
// Includes both numeric execution and symbolic graph generation

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/dynamics/Dynamics.hpp> // Aggregate header

#include <iomanip>
#include <iostream>
#include <vector>

using namespace vulcan::dynamics;
using namespace vulcan;

// =============================================================================
// 1. 3-DOF Point Mass Example (Numeric)
// =============================================================================
void run_3dof_numeric() {
    std::cout << "\n=== 3-DOF Point Mass (Numeric) ===\n";

    // Initial state (ECEF)
    Vec3<double> r_ecef{constants::earth::R_eq + 400000.0, 0.0,
                        0.0};              // 400km alt
    Vec3<double> v_ecef{0.0, 7670.0, 0.0}; // Orbital velocity
    double mass = 1000.0;

    // Forces (Body frame)
    Vec3<double> thrust_body{2000.0, 0.0, 0.0}; // Small maneuvers

    // Transform force to ECEF (assuming aligned with velocity for simplicity)
    Vec3<double> v_hat = velocity_direction(v_ecef);
    Vec3<double> f_ecef = thrust_body(0) * v_hat; // Thrust along velocity

    // Gravity
    double mu = constants::earth::mu;
    Vec3<double> a_grav = -mu * r_ecef / std::pow(r_ecef.norm(), 3);
    Vec3<double> f_grav = mass * a_grav;

    // Environment
    Vec3<double> omega_earth{0.0, 0.0, constants::earth::omega};

    // Compute acceleration
    Vec3<double> accel = point_mass_acceleration_ecef(
        r_ecef, v_ecef, Vec3<double>(f_ecef + f_grav), mass, omega_earth);

    std::cout << "Position: " << r_ecef.transpose() << " m\n";
    std::cout << "Velocity: " << v_ecef.transpose() << " m/s\n";
    std::cout << "Accel:    " << accel.transpose() << " m/s²\n";

    // Derived quantities
    std::cout << "Speed:    " << speed(v_ecef) << " m/s\n";
    std::cout << "Energy:   " << specific_energy(r_ecef, v_ecef, mu) / 1e6
              << " MJ/kg\n";
}

// =============================================================================
// 2. Pseudo-5DOF Guided Vehicle (Symbolic)
// =============================================================================
void run_5dof_symbolic() {
    std::cout << "\n=== Pseudo-5DOF Guided (Symbolic) ===\n";
    using MX = casadi::MX;

    // Symbolic inputs
    auto thrust = MX::sym("thrust");
    auto drag = MX::sym("drag");
    auto lift = MX::sym("lift");
    auto mass = MX::sym("mass");

    // Flight state
    auto gamma = MX::sym("gamma"); // Flight path angle
    auto chi = MX::sym("chi");     // Heading
    auto phi = MX::sym("phi");     // Bank angle
    auto velocity = MX::sym("v");

    // Compute dynamics (Bank-to-Turn)
    // 1. Velocity derivative
    auto v_dot =
        velocity_dot_btt(thrust, drag, lift, mass, MX(9.81), gamma, chi, phi);

    // 2. Flight path rate
    auto weight = mass * 9.81;
    auto g_dot = gamma_dot(lift, weight, mass, velocity, gamma, phi);

    // 3. Heading rate
    auto c_dot = chi_dot_btt(lift, mass, velocity, gamma, phi);

    // Create Janus Function
    janus::Function f_dyn("guided_5dof",
                          {thrust, drag, lift, mass, velocity, gamma, chi, phi},
                          {v_dot, g_dot, c_dot});

    std::cout << "Generated symbolic graph for 5-DOF dynamics.\n";

    // Evaluate with concrete values
    // 5000N thrust, 1000N drag, 10000N lift (turning), 1000kg, 200m/s, level
    // flight, 45 deg bank
    std::vector<double> args = {5000.0, 1000.0, 10000.0, 1000.0,
                                200.0,  0.0,    0.0,     M_PI / 4};
    auto res = f_dyn(args);

    std::cout << "Inputs: T=5kN, D=1kN, L=10kN, m=1t, V=200m/s, Bank=45°\n";
    std::cout << "V_dot (NED): " << res[0]
              << "\n"; // Expect forward accel + gravity
    std::cout << "Gamma_dot:   " << res[1] << " rad/s\n";
    std::cout << "Chi_dot:     " << res[2] << " rad/s\n";

    // Export graph visualization (if needed)
    // f_dyn.export_graph("guided_5dof.html");
}

// =============================================================================
// 3. Rigid Body 6-DOF (Numeric)
// =============================================================================
void run_6dof_numeric() {
    std::cout << "\n=== Rigid Body 6-DOF (Numeric) ===\n";

    // Mass Properties (Cylinder: L=5m, R=0.5m, m=1000kg)
    double m = 1000.0;
    double r = 0.5;
    double h = 5.0;
    double I_axial = 0.5 * m * r * r;                        // X-axis
    double I_trans = (1.0 / 12.0) * m * (3 * r * r + h * h); // Y/Z axes
    auto props = MassProperties<double>::diagonal(m, I_axial, I_trans, I_trans);

    // Initial state
    RigidBodyState<double> state;
    state.position = Vec3<double>{0, 0, -1000};    // 1km altitude
    state.velocity_body = Vec3<double>{100, 0, 0}; // 100 m/s forward
    state.attitude = janus::Quaternion<double>();
    state.omega_body = Vec3<double>{1.0, 0, 0}; // Spinning 1 rad/s roll

    // Forces/Moments
    Vec3<double> force_body{5000, 0, 0}; // 5kN Thrust
    Vec3<double> moment_body{0, 100, 0}; // Small pitch moment

    // Compute derivatives
    auto derivs =
        compute_6dof_derivatives(state, force_body, moment_body, props);

    std::cout << "Pos Dot (NED):  " << derivs.position_dot.transpose() << "\n";
    std::cout << "Vel Dot (Body): " << derivs.velocity_dot.transpose() << "\n";
    std::cout << "Omega Dot:      " << derivs.omega_dot.transpose() << "\n";
    std::cout << "Quat Dot:       " << derivs.attitude_dot.coeffs().transpose()
              << "\n";
}

int main() {
    try {
        std::cout << "Vulcan Dynamics Examples\n";
        std::cout << "========================\n";

        run_3dof_numeric();
        run_5dof_symbolic();
        run_6dof_numeric();

        std::cout << "\nDemo completed successfully.\n";
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
