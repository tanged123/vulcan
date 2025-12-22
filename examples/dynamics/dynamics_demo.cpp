// Vulcan Dynamics Demo
// Demonstrates 6DOF rigid body dynamics for trajectory simulation
#include <vulcan/vulcan.hpp>

#include <iomanip>
#include <iostream>

using namespace vulcan;
using namespace vulcan::dynamics;

void print_header(const std::string &title) {
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << title << "\n";
    std::cout << std::string(60, '=') << "\n";
}

// Example 1: Free-falling rotating body simulation
void free_fall_rotation_demo() {
    print_header("Free-Falling Rotating Body");

    // Define mass properties (cylindrical body)
    double mass = 100.0;         // 100 kg
    double Ixx = 5.0, Iyy = 5.0; // Symmetric about Z
    double Izz = 10.0;           // Spin axis
    auto mass_props = MassProperties<double>::diagonal(mass, Ixx, Iyy, Izz);

    // Initial state: 1000m altitude, moving at 50 m/s in body X, spinning
    RigidBodyState<double> state{
        .position = Vec3<double>{0.0, 0.0, 1000.0},    // 1000m up
        .velocity_body = Vec3<double>{50.0, 0.0, 0.0}, // 50 m/s forward
        .attitude = janus::Quaternion<double>(),       // Identity
        .omega_body = Vec3<double>{0.0, 0.0, 1.0}      // 1 rad/s spin
    };

    // Gravity in reference frame (pointing down)
    Vec3<double> gravity_ref{0.0, 0.0, -9.81};

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Initial state:\n";
    std::cout << "  Position: [" << state.position.transpose() << "] m\n";
    std::cout << "  Velocity (body): [" << state.velocity_body.transpose()
              << "] m/s\n";
    std::cout << "  Omega (body): [" << state.omega_body.transpose()
              << "] rad/s\n\n";

    // Simple Euler integration for demo
    double dt = 0.01;
    for (int i = 0; i < 100; ++i) {
        // Transform gravity to body frame
        Vec3<double> gravity_body =
            velocity_to_body_frame(gravity_ref, state.attitude);
        Vec3<double> force_body = gravity_body * mass;
        Vec3<double> moment_body = Vec3<double>::Zero(); // No torque

        // Compute derivatives
        auto derivs = compute_6dof_derivatives(state, force_body, moment_body,
                                               mass_props);

        // Euler step
        state.position += derivs.position_dot * dt;
        state.velocity_body += derivs.velocity_dot * dt;
        state.attitude = janus::Quaternion<double>(
            state.attitude.w + derivs.attitude_dot.w * dt,
            state.attitude.x + derivs.attitude_dot.x * dt,
            state.attitude.y + derivs.attitude_dot.y * dt,
            state.attitude.z + derivs.attitude_dot.z * dt);
        state.attitude = state.attitude.normalized();
        state.omega_body += derivs.omega_dot * dt;
    }

    std::cout << "After 1 second (100 steps):\n";
    std::cout << "  Position: [" << state.position.transpose() << "] m\n";
    std::cout << "  Velocity (body): [" << state.velocity_body.transpose()
              << "] m/s\n";
    std::cout << "  Omega (body): [" << state.omega_body.transpose()
              << "] rad/s\n";
    std::cout << "  Altitude drop: " << (1000.0 - state.position(2))
              << " m (expected ~4.9m from 0.5*g*t^2)\n";
}

// Example 2: Euler's equations - torque-free motion of asymmetric body
void torque_free_motion_demo() {
    print_header("Torque-Free Asymmetric Body");

    // Asymmetric inertia tensor (tumbling motion)
    double Ixx = 2.0, Iyy = 3.0, Izz = 4.0;
    Mat3<double> I = Mat3<double>::Zero();
    I(0, 0) = Ixx;
    I(1, 1) = Iyy;
    I(2, 2) = Izz;

    // Initial spin
    Vec3<double> omega{1.0, 0.5, 0.1};
    Vec3<double> moment = Vec3<double>::Zero();

    // Compute angular momentum (conserved)
    Vec3<double> H_initial = I * omega;
    double L_initial = H_initial.norm();

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Inertia: Ixx=" << Ixx << ", Iyy=" << Iyy << ", Izz=" << Izz
              << "\n";
    std::cout << "Initial omega: [" << omega.transpose() << "]\n";
    std::cout << "Angular momentum magnitude: " << L_initial
              << " (conserved)\n\n";

    // Integrate for 10 seconds
    double dt = 0.001;
    for (int i = 0; i < 10000; ++i) {
        auto omega_dot = rotational_dynamics(omega, moment, I);
        omega += omega_dot * dt;
    }

    // Check conservation
    Vec3<double> H_final = I * omega;
    double L_final = H_final.norm();

    std::cout << "After 10 seconds:\n";
    std::cout << "  Omega: [" << omega.transpose() << "]\n";
    std::cout << "  Angular momentum: " << L_final << "\n";
    std::cout << "  Conservation error: " << std::abs(L_final - L_initial)
              << " (should be ~0)\n";
}

// Example 3: ECEF dynamics with Coriolis/centrifugal
void ecef_dynamics_demo() {
    print_header("ECEF Dynamics (Coriolis/Centrifugal)");

    // Earth parameters
    constexpr double omega_e = 7.2921159e-5; // rad/s
    constexpr double R_earth = 6.371e6;      // m

    Vec3<double> omega_earth{0.0, 0.0, omega_e};

    // Object at equator, moving eastward at 100 m/s
    Vec3<double> position{R_earth, 0.0, 0.0}; // On equator
    Vec3<double> velocity{0.0, 100.0, 0.0};   // Eastward
    Vec3<double> force = Vec3<double>::Zero();
    double mass = 1.0;

    auto a_ecef = translational_dynamics_ecef(position, velocity, force, mass,
                                              omega_earth);

    std::cout << std::scientific << std::setprecision(4);
    std::cout << "Position: equator, altitude 0\n";
    std::cout << "Velocity: 100 m/s eastward\n\n";

    std::cout << "Fictitious accelerations (ECEF frame):\n";
    std::cout << "  a_x (radial): " << a_ecef(0) << " m/s²\n";
    std::cout << "  a_y (eastward): " << a_ecef(1) << " m/s²\n";
    std::cout << "  a_z (polar): " << a_ecef(2) << " m/s²\n\n";

    // Break down components
    double coriolis_x = 2.0 * omega_e * velocity(1);
    double centrifugal_x = omega_e * omega_e * position(0);

    std::cout << "Component breakdown:\n";
    std::cout << "  Coriolis (2*omega*v): " << coriolis_x
              << " m/s² (outward)\n";
    std::cout << "  Centrifugal (omega²*r): " << centrifugal_x
              << " m/s² (outward)\n";
}

int main() {
    std::cout << "Vulcan Rigid Body Dynamics Demo\n";
    std::cout << "================================\n";
    std::cout << "Demonstrating 6DOF equations of motion for\n";
    std::cout << "trajectory simulation and optimization.\n";

    free_fall_rotation_demo();
    torque_free_motion_demo();
    ecef_dynamics_demo();

    print_header("Demo Complete");
    std::cout << "All dynamics functions support both numeric (double)\n";
    std::cout
        << "and symbolic (casadi::MX) types for trajectory optimization.\n";

    return 0;
}
