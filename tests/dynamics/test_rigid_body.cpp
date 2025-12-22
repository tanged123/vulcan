// Test suite for rigid body dynamics
#include <vulcan/dynamics/Dynamics.hpp>

#include <gtest/gtest.h>

namespace vulcan::dynamics {

// =============================================================================
// MassProperties Tests
// =============================================================================

TEST(MassPropertiesTest, FromMass) {
    auto props = MassProperties<double>::from_mass(100.0);
    EXPECT_DOUBLE_EQ(props.mass, 100.0);
    // Point mass inertia = m * I for a point at origin (unit sphere moments)
    EXPECT_DOUBLE_EQ(props.inertia(0, 0), 100.0);
    EXPECT_DOUBLE_EQ(props.inertia(1, 1), 100.0);
    EXPECT_DOUBLE_EQ(props.inertia(2, 2), 100.0);
    EXPECT_DOUBLE_EQ(props.cg_offset.norm(), 0.0);
}

TEST(MassPropertiesTest, Diagonal) {
    auto props = MassProperties<double>::diagonal(100.0, 10.0, 20.0, 30.0);
    EXPECT_DOUBLE_EQ(props.mass, 100.0);
    EXPECT_DOUBLE_EQ(props.inertia(0, 0), 10.0);
    EXPECT_DOUBLE_EQ(props.inertia(1, 1), 20.0);
    EXPECT_DOUBLE_EQ(props.inertia(2, 2), 30.0);
    // Off-diagonals should be zero
    EXPECT_DOUBLE_EQ(props.inertia(0, 1), 0.0);
    EXPECT_DOUBLE_EQ(props.inertia(0, 2), 0.0);
    EXPECT_DOUBLE_EQ(props.inertia(1, 2), 0.0);
}

TEST(MassPropertiesTest, FullInertia) {
    // Test that products of inertia are negated correctly
    auto props =
        MassProperties<double>::full(100.0, 10.0, 20.0, 30.0, 1.0, 2.0, 3.0);
    EXPECT_DOUBLE_EQ(props.mass, 100.0);
    EXPECT_DOUBLE_EQ(props.inertia(0, 0), 10.0);
    EXPECT_DOUBLE_EQ(props.inertia(1, 1), 20.0);
    EXPECT_DOUBLE_EQ(props.inertia(2, 2), 30.0);
    // Products of inertia negated
    EXPECT_DOUBLE_EQ(props.inertia(0, 1), -1.0);
    EXPECT_DOUBLE_EQ(props.inertia(1, 0), -1.0);
    EXPECT_DOUBLE_EQ(props.inertia(0, 2), -2.0);
    EXPECT_DOUBLE_EQ(props.inertia(2, 0), -2.0);
    EXPECT_DOUBLE_EQ(props.inertia(1, 2), -3.0);
    EXPECT_DOUBLE_EQ(props.inertia(2, 1), -3.0);
}

// =============================================================================
// Translational Dynamics Tests
// =============================================================================

TEST(TranslationalDynamicsTest, PureTranslation) {
    // Test case 3: Pure translational (no rotation)
    // v_body = (100, 0, 0), ω = (0, 0, 0), F = (500, 0, 0), m = 100
    // Expected: v_dot = F/m = (5, 0, 0)
    Vec3<double> v_body{100.0, 0.0, 0.0};
    Vec3<double> omega{0.0, 0.0, 0.0};
    Vec3<double> force{500.0, 0.0, 0.0};
    double mass = 100.0;

    auto v_dot = translational_dynamics(v_body, omega, force, mass);

    EXPECT_NEAR(v_dot(0), 5.0, 1e-12);
    EXPECT_NEAR(v_dot(1), 0.0, 1e-12);
    EXPECT_NEAR(v_dot(2), 0.0, 1e-12);
}

TEST(TranslationalDynamicsTest, TransportTerm) {
    // Test case 4: Transport term (spinning body)
    // v_body = (10, 0, 0), ω = (0, 0, 0.1), F = (0, 0, 0), m = 1
    // v_dot = F/m - ω × v = -ω × v = -(0, 0, 0.1) × (10, 0, 0) = -(0, 1, 0) =
    // (0, -1, 0)
    Vec3<double> v_body{10.0, 0.0, 0.0};
    Vec3<double> omega{0.0, 0.0, 0.1};
    Vec3<double> force{0.0, 0.0, 0.0};
    double mass = 1.0;

    auto v_dot = translational_dynamics(v_body, omega, force, mass);

    EXPECT_NEAR(v_dot(0), 0.0, 1e-12);
    EXPECT_NEAR(v_dot(1), -1.0, 1e-12); // Negative, since v_dot = -ω×v
    EXPECT_NEAR(v_dot(2), 0.0, 1e-12);
}

// =============================================================================
// Rotational Dynamics Tests (Euler's Equations)
// =============================================================================

TEST(RotationalDynamicsTest, EulerEquationsComponentCheck) {
    // Test case 2: Euler's equations component check
    // Ixx = 2, Iyy = 3, Izz = 4, ω = (1, 2, 3), M = (0, 0, 0)
    // Expected:
    //   ω_dot_x = -(Izz - Iyy) * ωy * ωz / Ixx = -(4-3)*2*3/2 = -3.0
    //   ω_dot_y = -(Ixx - Izz) * ωz * ωx / Iyy = -(2-4)*3*1/3 = +2.0
    //   ω_dot_z = -(Iyy - Ixx) * ωx * ωy / Izz = -(3-2)*1*2/4 = -0.5
    Mat3<double> I = Mat3<double>::Zero();
    I(0, 0) = 2.0;
    I(1, 1) = 3.0;
    I(2, 2) = 4.0;

    Vec3<double> omega{1.0, 2.0, 3.0};
    Vec3<double> moment{0.0, 0.0, 0.0};

    auto omega_dot = rotational_dynamics(omega, moment, I);

    EXPECT_NEAR(omega_dot(0), -3.0, 1e-12);
    EXPECT_NEAR(omega_dot(1), 2.0, 1e-12);
    EXPECT_NEAR(omega_dot(2), -0.5, 1e-12);
}

TEST(RotationalDynamicsTest, TorqueFreeSymmetricGyroscope) {
    // Test case 1: Torque-free symmetric gyroscope
    // Ixx = Iyy = 10, Izz = 5, ω = (0.1, 0.2, 0.5), M = (0, 0, 0)
    // For symmetric body (Ixx = Iyy), spin axis torque free: ω_dot_z = 0
    Mat3<double> I = Mat3<double>::Zero();
    I(0, 0) = 10.0;
    I(1, 1) = 10.0;
    I(2, 2) = 5.0;

    Vec3<double> omega{0.1, 0.2, 0.5};
    Vec3<double> moment{0.0, 0.0, 0.0};

    auto omega_dot = rotational_dynamics(omega, moment, I);

    // ω_dot_z = -(Iyy - Ixx) * ωx * ωy / Izz = -(10-10)*0.1*0.2/5 = 0
    EXPECT_NEAR(omega_dot(2), 0.0, 1e-10);
}

TEST(RotationalDynamicsTest, WithAppliedTorque) {
    // Simple case with applied torque
    Mat3<double> I = Mat3<double>::Identity() * 10.0;
    Vec3<double> omega{0.0, 0.0, 0.0};
    Vec3<double> moment{100.0, 0.0, 0.0};

    auto omega_dot = rotational_dynamics(omega, moment, I);

    // With no rotation, ω_dot = M / I = 100/10 = 10
    EXPECT_NEAR(omega_dot(0), 10.0, 1e-12);
    EXPECT_NEAR(omega_dot(1), 0.0, 1e-12);
    EXPECT_NEAR(omega_dot(2), 0.0, 1e-12);
}

// =============================================================================
// Frame Transformation Tests
// =============================================================================

TEST(FrameTransformTest, IdentityQuaternion) {
    Vec3<double> v_body{1.0, 2.0, 3.0};
    janus::Quaternion<double> q; // Default constructor is identity

    auto v_ref = velocity_to_reference_frame(v_body, q);
    auto v_back = velocity_to_body_frame(v_ref, q);

    EXPECT_NEAR(v_ref(0), v_body(0), 1e-12);
    EXPECT_NEAR(v_ref(1), v_body(1), 1e-12);
    EXPECT_NEAR(v_ref(2), v_body(2), 1e-12);
    EXPECT_NEAR(v_back(0), v_body(0), 1e-12);
    EXPECT_NEAR(v_back(1), v_body(1), 1e-12);
    EXPECT_NEAR(v_back(2), v_body(2), 1e-12);
}

TEST(FrameTransformTest, Rotation90DegreesZ) {
    // 90 degree rotation about Z axis
    // Body X becomes Reference Y
    Vec3<double> v_body{1.0, 0.0, 0.0};
    double angle = M_PI / 2.0;
    auto q = janus::Quaternion<double>(std::cos(angle / 2), 0.0, 0.0,
                                       std::sin(angle / 2));

    auto v_ref = velocity_to_reference_frame(v_body, q);

    EXPECT_NEAR(v_ref(0), 0.0, 1e-12);
    EXPECT_NEAR(v_ref(1), 1.0, 1e-12);
    EXPECT_NEAR(v_ref(2), 0.0, 1e-12);
}

TEST(FrameTransformTest, RoundTrip) {
    // Random rotation and velocity
    Vec3<double> v_body{1.0, 2.0, 3.0};
    auto q =
        janus::Quaternion<double>(0.5, 0.5, 0.5, 0.5); // 120 deg about (1,1,1)

    auto v_ref = velocity_to_reference_frame(v_body, q);
    auto v_back = velocity_to_body_frame(v_ref, q);

    EXPECT_NEAR(v_back(0), v_body(0), 1e-12);
    EXPECT_NEAR(v_back(1), v_body(1), 1e-12);
    EXPECT_NEAR(v_back(2), v_body(2), 1e-12);
}

// =============================================================================
// Full 6DOF Derivatives Tests
// =============================================================================

TEST(SixDofDerivativesTest, QuaternionKinematicsConsistency) {
    // Test case 6: Quaternion kinematics
    // Identity quaternion, ω = (0.1, 0, 0)
    // Expected: q_dot = 0.5 * q ⊗ (0, 0.1, 0, 0) = (0, 0.05, 0, 0)
    RigidBodyState<double> state{.position = Vec3<double>::Zero(),
                                 .velocity_body = Vec3<double>::Zero(),
                                 .attitude =
                                     janus::Quaternion<double>(), // identity
                                 .omega_body = Vec3<double>{0.1, 0.0, 0.0}};

    Vec3<double> force{0.0, 0.0, 0.0};
    Vec3<double> moment{0.0, 0.0, 0.0};
    auto mass_props = MassProperties<double>::diagonal(1.0, 1.0, 1.0, 1.0);

    auto derivs = compute_6dof_derivatives(state, force, moment, mass_props);

    EXPECT_NEAR(derivs.attitude_dot.w, 0.0, 1e-12);
    EXPECT_NEAR(derivs.attitude_dot.x, 0.05, 1e-12);
    EXPECT_NEAR(derivs.attitude_dot.y, 0.0, 1e-12);
    EXPECT_NEAR(derivs.attitude_dot.z, 0.0, 1e-12);
}

TEST(SixDofDerivativesTest, PositionDotFromVelocity) {
    // Position rate should be velocity transformed to reference frame
    RigidBodyState<double> state{.position = Vec3<double>::Zero(),
                                 .velocity_body = Vec3<double>{100.0, 0.0, 0.0},
                                 .attitude =
                                     janus::Quaternion<double>(), // identity
                                 .omega_body = Vec3<double>::Zero()};

    Vec3<double> force{0.0, 0.0, 0.0};
    Vec3<double> moment{0.0, 0.0, 0.0};
    auto mass_props = MassProperties<double>::diagonal(1.0, 1.0, 1.0, 1.0);

    auto derivs = compute_6dof_derivatives(state, force, moment, mass_props);

    EXPECT_NEAR(derivs.position_dot(0), 100.0, 1e-12);
    EXPECT_NEAR(derivs.position_dot(1), 0.0, 1e-12);
    EXPECT_NEAR(derivs.position_dot(2), 0.0, 1e-12);
}

// =============================================================================
// ECEF Dynamics Tests
// =============================================================================

TEST(EcefDynamicsTest, CoriolisAndCentrifugal) {
    // Test case 5: ECEF Coriolis + Centrifugal
    // Position at equator r = (6.371e6, 0, 0), v = (0, 100, 0)
    // ω_earth = (0, 0, 7.2921159e-5)
    constexpr double omega_e = 7.2921159e-5;
    constexpr double R_earth = 6.371e6;

    Vec3<double> position{R_earth, 0.0, 0.0};
    Vec3<double> velocity{0.0, 100.0, 0.0};
    Vec3<double> force{0.0, 0.0, 0.0};
    Vec3<double> omega_earth{0.0, 0.0, omega_e};
    double mass = 1.0;

    auto a = translational_dynamics_ecef(position, velocity, force, mass,
                                         omega_earth);

    // Coriolis: -2 * ω × v = -2 * (0, 0, ω_e) × (0, 100, 0)
    //         = -2 * (-ω_e * 100, 0, 0) = (2 * ω_e * 100, 0, 0)
    // Wait, cross product: (0, 0, ω) × (0, v, 0) = (-ω*v, 0, 0)
    // So -2 * (-ω*v) = 2*ω*v in x direction
    double expected_coriolis_x = 2.0 * omega_e * 100.0; // ~0.01458

    // Centrifugal: -ω × (ω × r)
    // ω × r = (0, 0, ω_e) × (R, 0, 0) = (0, ω_e * R, 0)
    // ω × (ω × r) = (0, 0, ω_e) × (0, ω_e * R, 0) = (-ω_e² * R, 0, 0)
    // -ω × (ω × r) = (ω_e² * R, 0, 0)
    double expected_centrifugal_x = omega_e * omega_e * R_earth; // ~0.0339

    // Total acceleration should be Coriolis + Centrifugal (both positive x)
    EXPECT_NEAR(a(0), expected_coriolis_x + expected_centrifugal_x, 1e-6);
    EXPECT_NEAR(a(1), 0.0, 1e-10);
    EXPECT_NEAR(a(2), 0.0, 1e-10);
}

// =============================================================================
// Symbolic Tests with janus::Function Evaluation
// =============================================================================

TEST(RigidBodySymbolicTest, TranslationalDynamicsFunction) {
    using MX = casadi::MX;

    auto vx = janus::sym("vx");
    auto vy = janus::sym("vy");
    auto vz = janus::sym("vz");
    auto wx = janus::sym("wx");
    auto wy = janus::sym("wy");
    auto wz = janus::sym("wz");
    auto Fx = janus::sym("Fx");
    auto Fy = janus::sym("Fy");
    auto Fz = janus::sym("Fz");
    auto m = janus::sym("m");

    Vec3<MX> v{vx, vy, vz};
    Vec3<MX> omega{wx, wy, wz};
    Vec3<MX> force{Fx, Fy, Fz};

    auto v_dot = translational_dynamics(v, omega, force, m);

    janus::Function f("trans_dyn", {vx, vy, vz, wx, wy, wz, Fx, Fy, Fz, m},
                      {v_dot(0), v_dot(1), v_dot(2)});

    // Test case: v=(10,0,0), ω=(0,0,0.1), F=(0,0,0), m=1
    auto result = f({10.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0});

    // Compare with numeric
    Vec3<double> v_num{10.0, 0.0, 0.0};
    Vec3<double> omega_num{0.0, 0.0, 0.1};
    Vec3<double> force_num{0.0, 0.0, 0.0};
    auto v_dot_num = translational_dynamics(v_num, omega_num, force_num, 1.0);

    EXPECT_NEAR(result[0](0, 0), v_dot_num(0), 1e-12);
    EXPECT_NEAR(result[1](0, 0), v_dot_num(1), 1e-12);
    EXPECT_NEAR(result[2](0, 0), v_dot_num(2), 1e-12);
}

TEST(RigidBodySymbolicTest, RotationalDynamicsFunction) {
    using MX = casadi::MX;

    auto wx = janus::sym("wx");
    auto wy = janus::sym("wy");
    auto wz = janus::sym("wz");
    auto Mx = janus::sym("Mx");
    auto My = janus::sym("My");
    auto Mz = janus::sym("Mz");
    auto Ixx = janus::sym("Ixx");
    auto Iyy = janus::sym("Iyy");
    auto Izz = janus::sym("Izz");

    Vec3<MX> omega{wx, wy, wz};
    Vec3<MX> moment{Mx, My, Mz};
    Mat3<MX> I = Mat3<MX>::Zero();
    I(0, 0) = Ixx;
    I(1, 1) = Iyy;
    I(2, 2) = Izz;

    auto omega_dot = rotational_dynamics(omega, moment, I);

    janus::Function f("rot_dyn", {wx, wy, wz, Mx, My, Mz, Ixx, Iyy, Izz},
                      {omega_dot(0), omega_dot(1), omega_dot(2)});

    // Test case: ω=(1,2,3), M=(0,0,0), I=(2,3,4)
    auto result = f({1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 2.0, 3.0, 4.0});

    // Compare with numeric calculation
    Mat3<double> I_num = Mat3<double>::Zero();
    I_num(0, 0) = 2.0;
    I_num(1, 1) = 3.0;
    I_num(2, 2) = 4.0;
    Vec3<double> omega_num{1.0, 2.0, 3.0};
    Vec3<double> moment_num{0.0, 0.0, 0.0};
    auto omega_dot_num = rotational_dynamics(omega_num, moment_num, I_num);

    EXPECT_NEAR(result[0](0, 0), omega_dot_num(0), 1e-12);
    EXPECT_NEAR(result[1](0, 0), omega_dot_num(1), 1e-12);
    EXPECT_NEAR(result[2](0, 0), omega_dot_num(2), 1e-12);
}

TEST(RigidBodySymbolicTest, EcefDynamicsFunction) {
    using MX = casadi::MX;

    auto rx = janus::sym("rx");
    auto ry = janus::sym("ry");
    auto rz = janus::sym("rz");
    auto vx = janus::sym("vx");
    auto vy = janus::sym("vy");
    auto vz = janus::sym("vz");
    auto Fx = janus::sym("Fx");
    auto Fy = janus::sym("Fy");
    auto Fz = janus::sym("Fz");
    auto m = janus::sym("m");
    auto we = janus::sym("we");

    Vec3<MX> pos{rx, ry, rz};
    Vec3<MX> vel{vx, vy, vz};
    Vec3<MX> force{Fx, Fy, Fz};
    Vec3<MX> omega_earth{MX(0.0), MX(0.0), we};

    auto a = translational_dynamics_ecef(pos, vel, force, m, omega_earth);

    janus::Function f("ecef_dyn", {rx, ry, rz, vx, vy, vz, Fx, Fy, Fz, m, we},
                      {a(0), a(1), a(2)});

    // Test case at equator
    constexpr double R = 6.371e6;
    constexpr double omega_e = 7.2921159e-5;
    auto result =
        f({R, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 1.0, omega_e});

    // Compare with numeric
    Vec3<double> pos_num{R, 0.0, 0.0};
    Vec3<double> vel_num{0.0, 100.0, 0.0};
    Vec3<double> force_num{0.0, 0.0, 0.0};
    Vec3<double> omega_e_num{0.0, 0.0, omega_e};
    auto a_num = translational_dynamics_ecef(pos_num, vel_num, force_num, 1.0,
                                             omega_e_num);

    EXPECT_NEAR(result[0](0, 0), a_num(0), 1e-6);
    EXPECT_NEAR(result[1](0, 0), a_num(1), 1e-10);
    EXPECT_NEAR(result[2](0, 0), a_num(2), 1e-10);
}

} // namespace vulcan::dynamics
