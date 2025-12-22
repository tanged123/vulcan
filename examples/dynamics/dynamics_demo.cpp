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

// =============================================================================
// 4. Optimization: Max Sustained Turn Rate (Symbolic)
// =============================================================================
void run_5dof_optimization() {
    std::cout << "\n=== Optimization: Max Sustained Turn Rate (Symbolic) ===\n";
    using MX = casadi::MX;

    // Fixed Parameters
    double m = 1000.0;           // Mass [kg]
    double v_tgt = 300.0;        // Velocity [m/s]
    double alt = 5000.0;         // Altitude [m]
    double g = 9.80665;          // Gravity [m/s^2]
    double S = 20.0;             // Wing Area [m^2]
    double rho = 0.736;          // Density at 5km [kg/m^3]
    double max_thrust = 30000.0; // Max Thrust [N]

    // Aerodynamics (Drag Polar: CD = CD0 + k*CL^2)
    double CD0 = 0.02;
    double k = 0.05;

    // Initialize Optimizer
    janus::Opti opti;

    // Decision Variables
    auto lift = opti.variable(10000.0);  // Lift Force [N] (Guess approx Weight)
    auto bank = opti.variable(0.5);      // Bank Angle [rad] (Guess ~30 deg)
    auto thrust = opti.variable(5000.0); // Thrust Force [N] (Guess drag)

    // Derived States (Fixed for this point mass optimization)
    auto velocity = MX(v_tgt);
    auto gamma = MX(0.0); // Level flight
    auto chi = MX(0.0);   // Arbitrary heading
    auto mass = MX(m);
    auto gravity = MX(g);

    // Aerodynamic Limit Constraints
    auto q = 0.5 * rho * velocity * velocity; // Dynamic pressure
    auto CL = lift / (q * S);
    auto CD = CD0 + k * CL * CL;
    auto drag = q * S * CD;

    // Dynamics from Library
    auto weight = mass * gravity;

    // 1. Altitude constraint: gamma_dot = 0 (Sustained altitude)
    // gamma_dot = (L*cos(phi) - W) / (mV)
    // Constraint: L*cos(phi) == W  (Vertical equilibrium)
    opti.subject_to(lift * janus::cos(bank) == weight);

    // 2. Speed constraint: v_dot = 0 (Sustained speed)
    // Along velocity vector: T - D - W*sin(gamma) = m*v_dot
    // Since gamma=0, sin(gamma)=0. So T == D.
    opti.subject_to(thrust == drag);

    // 3. Physical Constraints
    opti.subject_to(thrust <= max_thrust);
    opti.subject_to(thrust >= 0.0);
    opti.subject_to(lift >= 0.0);
    opti.subject_to(bank >= 0.0);
    opti.subject_to(bank <= 1.5708); // Max 90 deg

    // Objective: Maximize Turn Rate (Chi_dot)
    // chi_dot = (L*sin(phi)) / (mV*cos(gamma))
    // We minimize negative chi_dot
    auto chi_dot_val = chi_dot_btt(lift, mass, velocity, gamma, bank);
    opti.minimize(-chi_dot_val);

    std::cout << "Problem Setup:\n";
    std::cout << "  Maximize: Turn Rate (Chi_dot)\n";
    std::cout
        << "  Subject to: Level Flight (Gamma_dot=0), Const Speed (V_dot=0)\n";
    std::cout << "  Variables: Lift, Bank, Thrust\n";

    // Solve
    try {
        auto sol = opti.solve({.verbose = false});

        double lift_opt = sol.value(lift);
        double bank_opt = sol.value(bank);
        double thrust_opt = sol.value(thrust);
        double turn_rate_opt = sol.value(chi_dot_val);
        double load_factor = lift_opt / (m * g);

        std::cout << "Results:\n";
        std::cout << "  Turn Rate:    " << (turn_rate_opt * 180.0 / 3.14159)
                  << " deg/s\n";
        std::cout << "  Bank Angle:   " << (bank_opt * 180.0 / 3.14159)
                  << " deg\n";
        std::cout << "  Load Factor:  " << load_factor << " g\n";
        std::cout << "  Thrust Req:   " << thrust_opt / 1000.0 << " kN (Max "
                  << max_thrust / 1000.0 << ")\n";
        std::cout << "  Lift Req:     " << lift_opt / 1000.0 << " kN\n";

    } catch (const std::exception &e) {
        std::cerr << "Optimization failed: " << e.what() << "\n";
    }
}

int main() {
    try {
        std::cout << "Vulcan Dynamics Examples\n";
        std::cout << "========================\n";

        run_3dof_numeric();
        run_5dof_symbolic();
        run_6dof_numeric();
        run_5dof_optimization();

        std::cout << "\nDemo completed successfully.\n";
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
