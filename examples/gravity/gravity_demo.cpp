// Vulcan Gravity Demo
// Demonstrates gravity models with symbolic optimization and graph
// visualization
#include <vulcan/gravity/Gravity.hpp>

#include <janus/janus.hpp>

#include <iomanip>
#include <iostream>
#include <numbers>

using namespace vulcan;
using namespace vulcan::gravity;

// Helper to print a Vec3
template <typename Scalar>
void print_vec3(const std::string &name, const Vec3<Scalar> &v,
                const std::string &units = "m/s²") {
    std::cout << name << ": [" << std::fixed << std::setprecision(6) << v(0)
              << ", " << v(1) << ", " << v(2) << "] " << units << "\n";
    std::cout << "  Magnitude: " << janus::norm(v) << " " << units << "\n";
}

int main() {
    std::cout << "=== Vulcan Gravity Models Demo ===\n\n";

    // =========================================================================
    // 1. Point Mass Gravity - The Simplest Model
    // =========================================================================
    std::cout << "--- 1. Point Mass Gravity ---\n";
    {
        // ISS orbit altitude (~400 km)
        Vec3<double> r_iss;
        r_iss << constants::earth::R_eq + 400000.0, 0.0, 0.0;

        auto g = point_mass::acceleration(r_iss);
        print_vec3("ISS gravity (point mass)", g);

        // Verify inverse-square law
        double g_surface =
            point_mass::acceleration_magnitude(constants::earth::R_eq);
        double g_iss = point_mass::acceleration_magnitude(janus::norm(r_iss));
        std::cout << "Surface gravity: " << g_surface << " m/s²\n";
        std::cout << "ISS gravity: " << g_iss << " m/s²\n";
        std::cout << "Ratio (r_iss/R_eq)²: "
                  << std::pow(janus::norm(r_iss) / constants::earth::R_eq, 2)
                  << "\n";
        std::cout << "Ratio g_surface/g_iss: " << g_surface / g_iss << "\n\n";
    }

    // =========================================================================
    // 2. J2 Gravity - Oblate Earth
    // =========================================================================
    std::cout << "--- 2. J2 Gravity (Oblate Earth) ---\n";
    {
        // Compare equator vs pole at same altitude
        double alt = 500000.0; // 500 km

        Vec3<double> r_equator, r_pole;
        r_equator << constants::earth::R_eq + alt, 0.0, 0.0;
        r_pole << 0.0, 0.0, constants::earth::R_pol + alt;

        auto g_eq = j2::acceleration(r_equator);
        auto g_pole = j2::acceleration(r_pole);

        print_vec3("Equator 500km (J2)", g_eq);
        print_vec3("Pole 500km (J2)", g_pole);

        std::cout << "\nJ2 effect: Polar gravity is "
                  << (janus::norm(g_pole) / janus::norm(g_eq) - 1.0) * 100.0
                  << "% stronger than equatorial\n\n";
    }

    // =========================================================================
    // 3. J2-J4 Gravity - Higher Fidelity
    // =========================================================================
    std::cout << "--- 3. J2-J4 Gravity (Higher Fidelity) ---\n";
    {
        Vec3<double> r;
        r << 7000000.0, 0.0, 1000000.0; // Off-equatorial position

        auto g_pm = point_mass::acceleration(r);
        auto g_j2 = j2::acceleration(r);
        auto g_j2j4 = j2j4::acceleration(r);

        std::cout << "Position: [7000, 0, 1000] km\n";
        print_vec3("Point Mass", g_pm);
        print_vec3("J2", g_j2);
        print_vec3("J2-J4", g_j2j4);

        // Show perturbation magnitudes
        Vec3<double> delta_j2 = g_j2 - g_pm;
        Vec3<double> delta_j2j4 = g_j2j4 - g_j2;
        std::cout << "\nJ2 perturbation: " << janus::norm(delta_j2) * 1000.0
                  << " mm/s²\n";
        std::cout << "J3+J4 perturbation: " << janus::norm(delta_j2j4) * 1000.0
                  << " mm/s²\n\n";
    }

    // =========================================================================
    // 4. Spherical Harmonics - General Expansion
    // =========================================================================
    std::cout << "--- 4. Spherical Harmonics ---\n";
    {
        Vec3<double> r;
        r << 7000000.0, 0.0, 1000000.0;

        // Default coefficients (J2, J3, J4)
        auto g_sh = spherical_harmonics::acceleration(r);
        auto g_j2j4 = j2j4::acceleration(r);

        print_vec3("Spherical Harmonics", g_sh);
        print_vec3("J2-J4 (for comparison)", g_j2j4);

        Vec3<double> diff = g_sh - g_j2j4;
        std::cout << "Difference: " << janus::norm(diff) * 1e6 << " μm/s²\n\n";
    }

    // =========================================================================
    // 5. Gravitational Potential
    // =========================================================================
    std::cout << "--- 5. Gravitational Potential ---\n";
    {
        Vec3<double> r_leo, r_geo;
        r_leo << constants::earth::R_eq + 400000.0, 0.0, 0.0;
        r_geo << constants::earth::R_eq + 35786000.0, 0.0, 0.0;

        auto U_leo = j2::potential(r_leo);
        auto U_geo = j2::potential(r_geo);

        std::cout << std::scientific << std::setprecision(4);
        std::cout << "LEO potential (400 km): " << U_leo << " J/kg\n";
        std::cout << "GEO potential (35786 km): " << U_geo << " J/kg\n";
        std::cout << "Delta-V to raise (sqrt(2*ΔU)): "
                  << std::sqrt(2 * (U_geo - U_leo)) / 1000.0 << " km/s\n\n";
        std::cout << std::fixed;
    }

    // =========================================================================
    // 6. Symbolic Computation - Optimization-Ready
    // =========================================================================
    std::cout << "--- 6. Symbolic Computation ---\n";
    {
        using Scalar = janus::SymbolicScalar;

        Scalar x = janus::sym("x");
        Scalar y = janus::sym("y");
        Scalar z = janus::sym("z");

        Vec3<Scalar> r;
        r << x, y, z;

        // Create symbolic gravity expressions
        auto g = j2::acceleration(r);

        std::cout << "Created symbolic J2 gravity expressions.\n";
        std::cout << "g_x has " << casadi::MX::n_nodes(g(0)) << " nodes\n";
        std::cout << "g_y has " << casadi::MX::n_nodes(g(1)) << " nodes\n";
        std::cout << "g_z has " << casadi::MX::n_nodes(g(2)) << " nodes\n";

        // Create CasADi function for numerical evaluation
        janus::Function f("j2_gravity", {x, y, z}, {g(0), g(1), g(2)});

        // Evaluate at specific position
        double test_x = 7000000.0, test_y = 500000.0, test_z = 1000000.0;
        auto result = f({test_x, test_y, test_z});

        Vec3<double> r_num;
        r_num << test_x, test_y, test_z;
        auto g_num = j2::acceleration(r_num);

        std::cout << "\nEvaluating at [7000, 500, 1000] km:\n";
        std::cout << "  Symbolic: [" << result[0](0, 0) << ", "
                  << result[1](0, 0) << ", " << result[2](0, 0) << "]\n";
        std::cout << "  Numeric:  [" << g_num(0) << ", " << g_num(1) << ", "
                  << g_num(2) << "]\n\n";
    }

    // =========================================================================
    // 7. Graph Visualization - Export to HTML
    // =========================================================================
    std::cout << "--- 7. Graph Visualization ---\n";
    {
        using Scalar = janus::SymbolicScalar;

        Scalar x = janus::sym("x");
        Scalar y = janus::sym("y");
        Scalar z = janus::sym("z");

        Vec3<Scalar> r;
        r << x, y, z;

        // Point mass - simple expression
        auto g_pm = point_mass::acceleration(r);
        janus::export_graph_html(g_pm(0), "graph_point_mass",
                                 "PointMassGravity_X");

        // J2 - more complex
        auto g_j2 = j2::acceleration(r);
        janus::export_graph_html(g_j2(0), "graph_j2_gravity", "J2Gravity_X");

        // Gravitational potential
        auto U = j2::potential(r);
        janus::export_graph_html(U, "graph_j2_potential", "J2Potential");

        std::cout << "Exported computational graphs:\n";
        std::cout << "   -> graph_point_mass.html (Point Mass g_x)\n";
        std::cout << "   -> graph_j2_gravity.html (J2 g_x)\n";
        std::cout << "   -> graph_j2_potential.html (J2 Potential)\n\n";
    }

    // =========================================================================
    // 8. Jacobian for Optimization - Gravity Gradients
    // =========================================================================
    std::cout << "--- 8. Gravity Gradient (Jacobian) ---\n";
    {
        using Scalar = janus::SymbolicScalar;

        Scalar x = janus::sym("x");
        Scalar y = janus::sym("y");
        Scalar z = janus::sym("z");

        Vec3<Scalar> r;
        r << x, y, z;

        auto g = j2::acceleration(r);

        // Compute Jacobian of gravity w.r.t. position
        auto J = janus::jacobian({g(0), g(1), g(2)}, {x, y, z});

        std::cout << "Computed 3x3 gravity gradient tensor (Jacobian).\n";

        // Create function for evaluation
        janus::Function f_jacobian("gravity_gradient", {x, y, z},
                                   {J(0, 0), J(0, 1), J(0, 2), J(1, 0), J(1, 1),
                                    J(1, 2), J(2, 0), J(2, 1), J(2, 2)});

        // Evaluate at a position
        auto result = f_jacobian({7000000.0, 0.0, 0.0});

        std::cout << "\nGravity gradient at [7000, 0, 0] km (μ/r³ units):\n";
        std::cout << std::scientific << std::setprecision(4);
        std::cout << "  [" << result[0](0, 0) << ", " << result[1](0, 0) << ", "
                  << result[2](0, 0) << "]\n";
        std::cout << "  [" << result[3](0, 0) << ", " << result[4](0, 0) << ", "
                  << result[5](0, 0) << "]\n";
        std::cout << "  [" << result[6](0, 0) << ", " << result[7](0, 0) << ", "
                  << result[8](0, 0) << "]\n";

        // Export Jacobian graph
        janus::export_graph_html(J(0, 0), "graph_gravity_gradient",
                                 "GravityGradient_xx");
        std::cout << "\nExported: graph_gravity_gradient.html\n";
    }

    std::cout << "\n=== Demo Complete ===\n";
    return 0;
}
