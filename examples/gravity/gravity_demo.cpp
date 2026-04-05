// Vulcan Gravity Demo
// Demonstrates gravity models with symbolic optimization and graph
// visualization
#include <vulcan/gravity/Gravity.hpp>
#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/QuantityEigen.hpp>
#include <vulcan/quantity/Units.hpp>

#include <janus/janus.hpp>

#include <iomanip>
#include <iostream>
#include <numbers>

using namespace vulcan;
using namespace vulcan::gravity;
using namespace vulcan::units;

// Helper: create a Vec3<Quantity<m, Scalar>> from three raw values
template <typename Scalar>
Vec3<Quantity<m, Scalar>> make_pos(Scalar x, Scalar y, Scalar z) {
    Vec3<Quantity<m, Scalar>> v;
    v(0) = Quantity<m, Scalar>{x};
    v(1) = Quantity<m, Scalar>{y};
    v(2) = Quantity<m, Scalar>{z};
    return v;
}

// Helper to print a Vec3<Quantity<Unit, double>>
template <auto U>
void print_qvec3(const std::string &name, const Vec3<Quantity<U, double>> &v,
                 const std::string &unit_str = "m/s^2") {
    double x = v(0).value(), y = v(1).value(), z = v(2).value();
    double mag = std::sqrt(x * x + y * y + z * z);
    std::cout << name << ": [" << std::fixed << std::setprecision(6) << x
              << ", " << y << ", " << z << "] " << unit_str << "\n";
    std::cout << "  Magnitude: " << mag << " " << unit_str << "\n";
}

int main() {
    std::cout << "=== Vulcan Gravity Models Demo ===\n\n";

    // =========================================================================
    // 1. Point Mass Gravity - The Simplest Model
    // =========================================================================
    std::cout << "--- 1. Point Mass Gravity ---\n";
    {
        // ISS orbit altitude (~400 km)
        auto r_iss =
            make_pos(constants::earth::R_eq.value() + 400000.0, 0.0, 0.0);

        auto g = point_mass::acceleration(r_iss);
        print_qvec3("ISS gravity (point mass)", g);

        // Verify inverse-square law
        Quantity<m, double> r_surface{constants::earth::R_eq.value()};
        double r_iss_mag = std::sqrt(r_iss(0).value() * r_iss(0).value() +
                                     r_iss(1).value() * r_iss(1).value() +
                                     r_iss(2).value() * r_iss(2).value());
        Quantity<m, double> r_iss_dist{r_iss_mag};

        double g_surface =
            point_mass::acceleration_magnitude(r_surface).value();
        double g_iss = point_mass::acceleration_magnitude(r_iss_dist).value();
        std::cout << "Surface gravity: " << g_surface << " m/s^2\n";
        std::cout << "ISS gravity: " << g_iss << " m/s^2\n";
        std::cout << "Ratio (r_iss/R_eq)^2: "
                  << std::pow(r_iss_mag / constants::earth::R_eq.value(), 2)
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

        auto r_equator =
            make_pos(constants::earth::R_eq.value() + alt, 0.0, 0.0);
        auto r_pole = make_pos(0.0, 0.0, constants::earth::R_pol.value() + alt);

        auto g_eq = j2::acceleration(r_equator);
        auto g_pole = j2::acceleration(r_pole);

        print_qvec3("Equator 500km (J2)", g_eq);
        print_qvec3("Pole 500km (J2)", g_pole);

        // Compute magnitudes for comparison
        auto eq_mag = std::sqrt(g_eq(0).value() * g_eq(0).value() +
                                g_eq(1).value() * g_eq(1).value() +
                                g_eq(2).value() * g_eq(2).value());
        auto pole_mag = std::sqrt(g_pole(0).value() * g_pole(0).value() +
                                  g_pole(1).value() * g_pole(1).value() +
                                  g_pole(2).value() * g_pole(2).value());

        std::cout << "\nJ2 effect: Polar gravity is "
                  << (pole_mag / eq_mag - 1.0) * 100.0
                  << "% stronger than equatorial\n\n";
    }

    // =========================================================================
    // 3. J2-J4 Gravity - Higher Fidelity
    // =========================================================================
    std::cout << "--- 3. J2-J4 Gravity (Higher Fidelity) ---\n";
    {
        auto r = make_pos(7000000.0, 0.0, 1000000.0); // Off-equatorial position

        auto g_pm = point_mass::acceleration(r);
        auto g_j2 = j2::acceleration(r);
        auto g_j2j4 = j2j4::acceleration(r);

        std::cout << "Position: [7000, 0, 1000] km\n";
        print_qvec3("Point Mass", g_pm);
        print_qvec3("J2", g_j2);
        print_qvec3("J2-J4", g_j2j4);

        // Show perturbation magnitudes
        Vec3<double> delta_j2;
        delta_j2(0) = g_j2(0).value() - g_pm(0).value();
        delta_j2(1) = g_j2(1).value() - g_pm(1).value();
        delta_j2(2) = g_j2(2).value() - g_pm(2).value();

        Vec3<double> delta_j2j4;
        delta_j2j4(0) = g_j2j4(0).value() - g_j2(0).value();
        delta_j2j4(1) = g_j2j4(1).value() - g_j2(1).value();
        delta_j2j4(2) = g_j2j4(2).value() - g_j2(2).value();

        std::cout << "\nJ2 perturbation: " << janus::norm(delta_j2) * 1000.0
                  << " mm/s^2\n";
        std::cout << "J3+J4 perturbation: " << janus::norm(delta_j2j4) * 1000.0
                  << " mm/s^2\n\n";
    }

    // =========================================================================
    // 4. Spherical Harmonics - General Expansion
    // =========================================================================
    std::cout << "--- 4. Spherical Harmonics ---\n";
    {
        auto r = make_pos(7000000.0, 0.0, 1000000.0);

        // Default coefficients (J2, J3, J4)
        auto g_sh = spherical_harmonics::acceleration(r);
        auto g_j2j4 = j2j4::acceleration(r);

        print_qvec3("Spherical Harmonics", g_sh);
        print_qvec3("J2-J4 (for comparison)", g_j2j4);

        Vec3<double> diff;
        diff(0) = g_sh(0).value() - g_j2j4(0).value();
        diff(1) = g_sh(1).value() - g_j2j4(1).value();
        diff(2) = g_sh(2).value() - g_j2j4(2).value();
        std::cout << "Difference: " << janus::norm(diff) * 1e6 << " um/s^2\n\n";
    }

    // =========================================================================
    // 5. Gravitational Potential
    // =========================================================================
    std::cout << "--- 5. Gravitational Potential ---\n";
    {
        auto r_leo =
            make_pos(constants::earth::R_eq.value() + 400000.0, 0.0, 0.0);
        auto r_geo =
            make_pos(constants::earth::R_eq.value() + 35786000.0, 0.0, 0.0);

        auto U_leo = j2::potential(r_leo);
        auto U_geo = j2::potential(r_geo);

        std::cout << std::scientific << std::setprecision(4);
        std::cout << "LEO potential (400 km): " << U_leo.value() << " J/kg\n";
        std::cout << "GEO potential (35786 km): " << U_geo.value() << " J/kg\n";
        std::cout << "Delta-V to raise (sqrt(2*DeltaU)): "
                  << std::sqrt(2 * (U_geo.value() - U_leo.value())) / 1000.0
                  << " km/s\n\n";
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

        auto r = make_pos(x, y, z);

        // Create symbolic gravity expressions
        auto g = j2::acceleration(r);

        std::cout << "Created symbolic J2 gravity expressions.\n";
        std::cout << "g_x has " << casadi::MX::n_nodes(g(0).value())
                  << " nodes\n";
        std::cout << "g_y has " << casadi::MX::n_nodes(g(1).value())
                  << " nodes\n";
        std::cout << "g_z has " << casadi::MX::n_nodes(g(2).value())
                  << " nodes\n";

        // Create CasADi function for numerical evaluation
        janus::Function f("j2_gravity", {x, y, z},
                          {g(0).value(), g(1).value(), g(2).value()});

        // Evaluate at specific position
        double test_x = 7000000.0, test_y = 500000.0, test_z = 1000000.0;
        auto result = f({test_x, test_y, test_z});

        auto r_num = make_pos(test_x, test_y, test_z);
        auto g_num = j2::acceleration(r_num);

        std::cout << "\nEvaluating at [7000, 500, 1000] km:\n";
        std::cout << "  Symbolic: [" << result[0](0, 0) << ", "
                  << result[1](0, 0) << ", " << result[2](0, 0) << "]\n";
        std::cout << "  Numeric:  [" << g_num(0).value() << ", "
                  << g_num(1).value() << ", " << g_num(2).value() << "]\n\n";
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

        auto r = make_pos(x, y, z);

        // Point mass - simple expression
        auto g_pm = point_mass::acceleration(r);
        janus::export_graph_html(g_pm(0).value(), "graph_point_mass",
                                 "PointMassGravity_X");

        // J2 - more complex
        auto g_j2 = j2::acceleration(r);
        janus::export_graph_html(g_j2(0).value(), "graph_j2_gravity",
                                 "J2Gravity_X");

        // Gravitational potential
        auto U = j2::potential(r);
        janus::export_graph_html(U.value(), "graph_j2_potential",
                                 "J2Potential");

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

        auto r = make_pos(x, y, z);

        auto g = j2::acceleration(r);

        // Compute Jacobian of gravity w.r.t. position
        auto J = janus::jacobian({g(0).value(), g(1).value(), g(2).value()},
                                 {x, y, z});

        std::cout << "Computed 3x3 gravity gradient tensor (Jacobian).\n";

        // Create function for evaluation
        janus::Function f_jacobian("gravity_gradient", {x, y, z},
                                   {J(0, 0), J(0, 1), J(0, 2), J(1, 0), J(1, 1),
                                    J(1, 2), J(2, 0), J(2, 1), J(2, 2)});

        // Evaluate at a position
        auto result = f_jacobian({7000000.0, 0.0, 0.0});

        std::cout << "\nGravity gradient at [7000, 0, 0] km (mu/r^3 units):\n";
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
