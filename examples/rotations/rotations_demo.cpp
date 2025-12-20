// Vulcan Rotations Demo
// Demonstrates the unified rotations library with all 12 Euler sequences
#include <vulcan/rotations/Rotations.hpp>

#include <janus/janus.hpp>

#include <iomanip>
#include <iostream>
#include <numbers>

using namespace vulcan;

// Helper to print a DCM
template <typename Scalar>
void print_dcm(const std::string &name, const Mat3<Scalar> &R) {
    std::cout << name << ":\n";
    std::cout << std::fixed << std::setprecision(6);
    for (int i = 0; i < 3; ++i) {
        std::cout << "  [";
        for (int j = 0; j < 3; ++j) {
            std::cout << std::setw(10) << R(i, j);
            if (j < 2)
                std::cout << ", ";
        }
        std::cout << "]\n";
    }
}

// Helper to print Euler angles
template <typename Scalar>
void print_euler(const std::string &name, const Vec3<Scalar> &e,
                 const char *seq_name) {
    std::cout << name << " (" << seq_name << "): [" << std::fixed
              << std::setprecision(4) << e(0) * 180.0 / std::numbers::pi
              << "°, " << e(1) * 180.0 / std::numbers::pi << "°, "
              << e(2) * 180.0 / std::numbers::pi << "°]\n";
}

int main() {
    std::cout << "=== Vulcan Rotations Library Demo ===\n\n";

    // =========================================================================
    // 1. Aerospace Standard: ZYX (Yaw-Pitch-Roll)
    // =========================================================================
    std::cout << "--- 1. Aerospace Standard: ZYX (Yaw-Pitch-Roll) ---\n";
    {
        double yaw = 30.0 * std::numbers::pi / 180.0;   // 30°
        double pitch = 15.0 * std::numbers::pi / 180.0; // 15°
        double roll = 10.0 * std::numbers::pi / 180.0;  // 10°

        // Create DCM from Euler angles
        auto R = dcm_from_euler(yaw, pitch, roll, EulerSequence::ZYX);
        print_dcm("DCM from ZYX(30°, 15°, 10°)", R);

        // Create quaternion
        auto q = quaternion_from_euler(yaw, pitch, roll, EulerSequence::ZYX);
        std::cout << "Quaternion: [" << q.w << ", " << q.x << ", " << q.y
                  << ", " << q.z << "]\n";

        // Extract angles back
        auto euler_back = euler_from_dcm(R, EulerSequence::ZYX);
        print_euler("Extracted angles", euler_back, "ZYX");

        // Verify matches Janus
        auto R_janus =
            janus::rotation_matrix_from_euler_angles(roll, pitch, yaw);
        double max_diff = (R - R_janus).cwiseAbs().maxCoeff();
        std::cout << "Max diff from Janus: " << max_diff << "\n\n";
    }

    // =========================================================================
    // 2. Robotics Convention: XYZ (Roll-Pitch-Yaw)
    // =========================================================================
    std::cout << "--- 2. Robotics Convention: XYZ (Roll-Pitch-Yaw) ---\n";
    {
        double e1 = 20.0 * std::numbers::pi / 180.0; // Roll about X
        double e2 = 25.0 * std::numbers::pi / 180.0; // Pitch about Y
        double e3 = 35.0 * std::numbers::pi / 180.0; // Yaw about Z

        auto R = dcm_from_euler(e1, e2, e3, EulerSequence::XYZ);
        print_dcm("DCM from XYZ(20°, 25°, 35°)", R);

        auto euler_back = euler_from_dcm(R, EulerSequence::XYZ);
        print_euler("Extracted angles", euler_back, "XYZ");
        std::cout << "\n";
    }

    // =========================================================================
    // 3. Classical Mechanics: ZXZ (Precession-Nutation-Spin)
    // =========================================================================
    std::cout << "--- 3. Classical Mechanics: ZXZ (Proper Euler) ---\n";
    {
        double precession = 45.0 * std::numbers::pi / 180.0;
        double nutation = 30.0 * std::numbers::pi / 180.0;
        double spin = 60.0 * std::numbers::pi / 180.0;

        auto R = dcm_from_euler(precession, nutation, spin, EulerSequence::ZXZ);
        print_dcm("DCM from ZXZ(45°, 30°, 60°)", R);

        auto euler_back = euler_from_dcm(R, EulerSequence::ZXZ);
        print_euler("Extracted angles", euler_back, "ZXZ");
        std::cout << "\n";
    }

    // =========================================================================
    // 4. All 12 Sequences - Roundtrip Verification
    // =========================================================================
    std::cout << "--- 4. All 12 Euler Sequences Roundtrip ---\n";
    {
        std::vector<EulerSequence> all_sequences = {
            EulerSequence::XYZ, EulerSequence::XZY, EulerSequence::YXZ,
            EulerSequence::YZX, EulerSequence::ZXY, EulerSequence::ZYX,
            EulerSequence::XYX, EulerSequence::XZX, EulerSequence::YXY,
            EulerSequence::YZY, EulerSequence::ZXZ, EulerSequence::ZYZ};

        double e1 = 0.3, e2 = 0.4, e3 = 0.2;
        // For proper Euler sequences, use different e2 to avoid singularity
        double e2_proper = 0.8;

        for (auto seq : all_sequences) {
            double test_e2 = is_proper_euler(seq) ? e2_proper : e2;
            auto R = dcm_from_euler(e1, test_e2, e3, seq);
            auto euler_back = euler_from_dcm(R, seq);

            double err1 = std::abs(euler_back(0) - e1);
            double err2 = std::abs(euler_back(1) - test_e2);
            double err3 = std::abs(euler_back(2) - e3);
            double max_err = std::max({err1, err2, err3});

            std::cout << euler_sequence_name(seq)
                      << ": max error = " << std::scientific
                      << std::setprecision(2) << max_err
                      << (max_err < 1e-10 ? " ✓" : " ✗") << "\n";
        }
        std::cout << "\n";
    }

    // =========================================================================
    // 5. Axis-Angle and Rodrigues' Formula
    // =========================================================================
    std::cout << "--- 5. Axis-Angle and Rodrigues' Formula ---\n";
    {
        Vec3<double> axis;
        axis << 1.0, 1.0, 1.0;
        axis.normalize();
        double angle = 45.0 * std::numbers::pi / 180.0;

        std::cout << "Axis: [" << axis(0) << ", " << axis(1) << ", " << axis(2)
                  << "], Angle: 45°\n";

        auto R = dcm_from_axis_angle(axis, angle);
        print_dcm("DCM from Rodrigues", R);

        auto [axis_back, angle_back] = axis_angle_from_dcm(R);
        std::cout << "Extracted axis: [" << axis_back(0) << ", " << axis_back(1)
                  << ", " << axis_back(2) << "]\n";
        std::cout << "Extracted angle: "
                  << angle_back * 180.0 / std::numbers::pi << "°\n\n";
    }

    // =========================================================================
    // 6. Quaternion Interpolation (Slerp)
    // =========================================================================
    std::cout << "--- 6. Quaternion Interpolation (Slerp) ---\n";
    {
        // Identity to 90° about Z
        auto q0 = janus::Quaternion<double>(1, 0, 0, 0);
        auto q1 = quaternion_from_euler(0.0, 0.0, std::numbers::pi / 2.0,
                                        EulerSequence::ZYX);

        std::cout << "q0: identity\n";
        std::cout << "q1: 90° roll about X\n";
        std::cout << "\nInterpolation:\n";

        for (double t = 0.0; t <= 1.0; t += 0.25) {
            auto q = vulcan::slerp(q0, q1, t);
            auto euler = euler_from_quaternion(q, EulerSequence::ZYX);
            std::cout << "  t=" << t
                      << ": roll=" << euler(2) * 180.0 / std::numbers::pi
                      << "°\n";
        }
        std::cout << "\n";
    }

    // =========================================================================
    // 7. Symbolic Computation with Graph Visualization
    // =========================================================================
    std::cout << "--- 7. Symbolic Computation with Graph Visualization ---\n";
    {
        using Scalar = janus::SymbolicScalar;

        Scalar yaw = janus::sym("yaw");
        Scalar pitch = janus::sym("pitch");
        Scalar roll = janus::sym("roll");

        auto R = dcm_from_euler(yaw, pitch, roll, EulerSequence::ZYX);

        std::cout << "Created symbolic DCM for ZYX sequence.\n";
        std::cout << "R[0,0] = " << R(0, 0) << "\n";
        std::cout << "R[2,0] = " << R(2, 0) << " (should be -sin(pitch))\n";

        // Export computational graphs as HTML
        janus::export_graph_html(R(0, 0), "graph_dcm_00", "DCM_R00_ZYX");
        janus::export_graph_html(R(2, 0), "graph_dcm_20", "DCM_R20_ZYX");
        std::cout << "\nExported computational graphs:\n";
        std::cout << "   -> graph_dcm_00.html (R[0,0] = cos(yaw)*cos(pitch))\n";
        std::cout << "   -> graph_dcm_20.html (R[2,0] = -sin(pitch))\n";

        // Create CasADi function
        janus::Function f("dcm_zyx", {yaw, pitch, roll},
                          {R(0, 0), R(1, 0), R(2, 0)});

        // Evaluate at specific values
        double yaw_val = 0.5;
        double pitch_val = 0.3;
        double roll_val = 0.2;

        auto result = f({yaw_val, pitch_val, roll_val});
        auto R_numeric =
            dcm_from_euler(yaw_val, pitch_val, roll_val, EulerSequence::ZYX);

        std::cout << "\nEvaluating at yaw=0.5, pitch=0.3, roll=0.2:\n";
        std::cout << "  Symbolic R[0,0]: " << result[0](0, 0)
                  << ", Numeric: " << R_numeric(0, 0) << "\n";
        std::cout << "  Symbolic R[2,0]: " << result[2](0, 0)
                  << ", Numeric: " << R_numeric(2, 0)
                  << " (expected: " << -std::sin(pitch_val) << ")\n";
    }

    // =========================================================================
    // 8. Rodrigues Formula Graph
    // =========================================================================
    std::cout << "\n--- 8. Rodrigues Formula Symbolic Graph ---\n";
    {
        using Scalar = janus::SymbolicScalar;

        Scalar angle = janus::sym("angle");
        Vec3<Scalar> axis;
        axis << Scalar(0), Scalar(0), Scalar(1); // Z-axis (constant)

        auto R = dcm_from_axis_angle(axis, angle);

        std::cout << "Created symbolic Rodrigues DCM for Z-axis rotation.\n";
        std::cout << "R[0,0] = " << R(0, 0) << " (should be cos(angle))\n";
        std::cout << "R[0,1] = " << R(0, 1) << " (should be -sin(angle))\n";

        // Export computational graph
        janus::export_graph_html(R(0, 0), "graph_rodrigues",
                                 "RodriguesFormula");
        std::cout << "\nExported: graph_rodrigues.html\n";
    }

    std::cout << "\n=== Demo Complete ===\n";
    return 0;
}
