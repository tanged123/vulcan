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
// Symbolic Instantiation Test
// =============================================================================

TEST(RigidBodySymbolicTest, Instantiation) {
    using MX = casadi::MX;

    // Create symbolic mass properties
    auto m = MX::sym("m");
    auto Ixx = MX::sym("Ixx");
    auto Iyy = MX::sym("Iyy");
    auto Izz = MX::sym("Izz");
    auto mass_props = MassProperties<MX>::diagonal(m, Ixx, Iyy, Izz);

    // Create symbolic state
    RigidBodyState<MX> state{
        .position = Vec3<MX>::Zero(),
        .velocity_body = Vec3<MX>{MX::sym("vx"), MX::sym("vy"), MX::sym("vz")},
        .attitude = janus::Quaternion<MX>(), // identity
        .omega_body = Vec3<MX>{MX::sym("wx"), MX::sym("wy"), MX::sym("wz")}};

    // Create symbolic inputs
    Vec3<MX> force{MX::sym("Fx"), MX::sym("Fy"), MX::sym("Fz")};
    Vec3<MX> moment{MX::sym("Mx"), MX::sym("My"), MX::sym("Mz")};

    // This should NOT throw — graph builds successfully
    auto derivs =
        compute_6dof_derivatives<MX>(state, force, moment, mass_props);

    // Verify outputs are symbolic (non-empty graph)
    EXPECT_FALSE(derivs.velocity_dot(0).is_empty());
    EXPECT_FALSE(derivs.omega_dot(0).is_empty());
    EXPECT_FALSE(derivs.position_dot(0).is_empty());
    EXPECT_FALSE(derivs.attitude_dot.w.is_empty());
}

TEST(RigidBodySymbolicTest, TranslationalDynamics) {
    using MX = casadi::MX;

    Vec3<MX> v{MX::sym("vx"), MX::sym("vy"), MX::sym("vz")};
    Vec3<MX> omega{MX::sym("wx"), MX::sym("wy"), MX::sym("wz")};
    Vec3<MX> force{MX::sym("Fx"), MX::sym("Fy"), MX::sym("Fz")};
    auto mass = MX::sym("m");

    auto v_dot = translational_dynamics(v, omega, force, mass);

    EXPECT_FALSE(v_dot(0).is_empty());
    EXPECT_FALSE(v_dot(1).is_empty());
    EXPECT_FALSE(v_dot(2).is_empty());
}

TEST(RigidBodySymbolicTest, RotationalDynamics) {
    using MX = casadi::MX;

    Vec3<MX> omega{MX::sym("wx"), MX::sym("wy"), MX::sym("wz")};
    Vec3<MX> moment{MX::sym("Mx"), MX::sym("My"), MX::sym("Mz")};

    // Diagonal inertia for simplicity
    Mat3<MX> I = Mat3<MX>::Zero();
    I(0, 0) = MX::sym("Ixx");
    I(1, 1) = MX::sym("Iyy");
    I(2, 2) = MX::sym("Izz");

    auto omega_dot = rotational_dynamics(omega, moment, I);

    EXPECT_FALSE(omega_dot(0).is_empty());
    EXPECT_FALSE(omega_dot(1).is_empty());
    EXPECT_FALSE(omega_dot(2).is_empty());
}

TEST(RigidBodySymbolicTest, EcefDynamics) {
    using MX = casadi::MX;

    Vec3<MX> pos{MX::sym("rx"), MX::sym("ry"), MX::sym("rz")};
    Vec3<MX> vel{MX::sym("vx"), MX::sym("vy"), MX::sym("vz")};
    Vec3<MX> force{MX::sym("Fx"), MX::sym("Fy"), MX::sym("Fz")};
    Vec3<MX> omega_e{MX(0.0), MX(0.0), MX::sym("we")};
    auto mass = MX::sym("m");

    auto a = translational_dynamics_ecef(pos, vel, force, mass, omega_e);

    EXPECT_FALSE(a(0).is_empty());
    EXPECT_FALSE(a(1).is_empty());
    EXPECT_FALSE(a(2).is_empty());
}

} // namespace vulcan::dynamics
