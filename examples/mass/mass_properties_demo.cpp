// Vulcan Mass Properties Demo
// Demonstrates mass property utilities for vehicle modeling and analysis
#include <vulcan/vulcan.hpp>

#include <iomanip>
#include <iostream>

using namespace vulcan;
using namespace vulcan::mass;

void print_header(const std::string &title) {
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << title << "\n";
    std::cout << std::string(60, '=') << "\n";
}

void print_mass_props(const std::string &name,
                      const MassProperties<double> &mp) {
    std::cout << name << ":\n";
    std::cout << "  Mass: " << mp.mass << " kg\n";
    std::cout << "  CG:   [" << mp.cg.transpose() << "] m\n";
    std::cout << "  Inertia (about CG, body axes):\n";
    std::cout << "    Ixx=" << mp.inertia(0, 0) << "  Iyy=" << mp.inertia(1, 1)
              << "  Izz=" << mp.inertia(2, 2) << " kg·m²\n";
}

// Example 1: Basic shape factories
void shape_factories_demo() {
    print_header("Shape Factory Constructors");

    // Solid sphere
    auto sphere =
        MassProperties<double>::solid_sphere(100.0, 0.5, {0.0, 0.0, 0.0});
    print_mass_props("Solid Sphere (100 kg, r=0.5 m)", sphere);
    // I = (2/5) * m * r² = 0.4 * 100 * 0.25 = 10 kg·m²
    std::cout << "  Expected I = (2/5)*m*r² = " << 0.4 * 100.0 * 0.25
              << " kg·m²\n\n";

    // Solid cylinder (axis along Z)
    auto cylinder = MassProperties<double>::solid_cylinder(50.0, 0.3, 1.0);
    print_mass_props("Solid Cylinder (50 kg, r=0.3 m, L=1.0 m)", cylinder);
    double Ixx_cyl = (1.0 / 12.0) * 50.0 * (3 * 0.09 + 1.0);
    double Izz_cyl = 0.5 * 50.0 * 0.09;
    std::cout << "  Expected Ixx = " << Ixx_cyl << ", Izz = " << Izz_cyl
              << " kg·m²\n\n";

    // Solid box
    auto box = MassProperties<double>::solid_box(80.0, 2.0, 1.0, 0.5);
    print_mass_props("Solid Box (80 kg, 2x1x0.5 m)", box);
}

// Example 2: Aggregation - building a simple vehicle
void aggregation_demo() {
    print_header("Mass Properties Aggregation");

    // Build a simple aircraft from components
    // Fuselage: cylinder at origin
    auto fuselage = MassProperties<double>::solid_cylinder(
        500.0,                      // mass [kg]
        0.5,                        // radius [m]
        6.0,                        // length [m]
        Vec3<double>{3.0, 0.0, 0.0} // CG at center of fuselage
    );
    print_mass_props("Fuselage", fuselage);

    // Left wing: box offset in Y
    auto left_wing =
        MassProperties<double>::solid_box(100.0,                     // mass
                                          0.5, 4.0, 0.1,             // dims
                                          Vec3<double>{2.5, -2.5, 0} // CG
        );
    print_mass_props("Left Wing", left_wing);

    // Right wing: mirror of left
    auto right_wing =
        MassProperties<double>::solid_box(100.0,                    // mass
                                          0.5, 4.0, 0.1,            // dims
                                          Vec3<double>{2.5, 2.5, 0} // CG
        );
    print_mass_props("Right Wing", right_wing);

    // Tail section
    auto tail =
        MassProperties<double>::point_mass(50.0, Vec3<double>{5.5, 0.0, 0.0});
    print_mass_props("Tail", tail);

    // Aggregate all components
    std::cout << "\n--- Aggregating Components ---\n";
    auto aircraft = fuselage + left_wing + right_wing + tail;
    print_mass_props("Complete Aircraft", aircraft);

    std::cout << "\n  Total mass expected: " << 500 + 100 + 100 + 50 << " kg\n";
    std::cout << "  Actual total mass:   " << aircraft.mass << " kg\n";
}

// Example 3: Using aggregate_mass_properties() with a vector
void aggregate_vector_demo() {
    print_header("Aggregate from Vector");

    // Create a vector of point masses representing fuel tanks
    std::vector<MassProperties<double>> tanks;
    tanks.push_back(
        MassProperties<double>::point_mass(100.0, {1.0, -1.0, 0.0}));
    tanks.push_back(MassProperties<double>::point_mass(100.0, {1.0, 1.0, 0.0}));
    tanks.push_back(MassProperties<double>::point_mass(50.0, {4.0, 0.0, 0.0}));

    auto total_fuel = aggregate_mass_properties(tanks);
    print_mass_props("Total Fuel (3 tanks)", total_fuel);

    // CG calculation: (100*1 + 100*1 + 50*4) / 250 = 400/250 = 1.6 for x
    std::cout << "  Expected CG x: " << (100 * 1.0 + 100 * 1.0 + 50 * 4.0) / 250
              << " m\n";
}

// Example 4: Parallel axis theorem
void parallel_axis_demo() {
    print_header("Parallel Axis Theorem");

    // Create a sphere at a position
    auto sphere = MassProperties<double>::solid_sphere(
        100.0, 0.5, Vec3<double>{2.0, 0.0, 0.0});
    print_mass_props("Sphere at (2, 0, 0)", sphere);

    // Get inertia about origin
    auto I_origin = sphere.inertia_about_point(Vec3<double>::Zero());
    std::cout << "\nInertia about origin:\n";
    std::cout << "  Ixx = " << I_origin(0, 0) << " kg·m²\n";
    std::cout << "  Iyy = " << I_origin(1, 1) << " kg·m²\n";
    std::cout << "  Izz = " << I_origin(2, 2) << " kg·m²\n";

    // Sphere I about CG: (2/5)*100*0.25 = 10 kg·m²
    // Parallel axis adds: m*d² = 100*4 = 400 kg·m² to Iyy, Izz
    double I_sphere = 0.4 * 100.0 * 0.25;
    std::cout << "\nExpected values:\n";
    std::cout << "  Ixx = " << I_sphere << " (no offset in y or z)\n";
    std::cout << "  Iyy = Izz = " << I_sphere + 100 * 4 << " (I_cg + m*d²)\n";
}

// Example 5: Variable mass (fuel burn simulation)
void variable_mass_demo() {
    print_header("Variable Mass Simulation");

    double dry_mass = 500.0;
    double fuel_mass = 200.0;
    double burn_rate = 10.0; // kg/s
    double dt = 1.0;         // timestep
    double duration = 15.0;  // seconds

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Simulating fuel burn at " << burn_rate << " kg/s\n\n";
    std::cout << "  Time   Mass     CG_x\n";
    std::cout << "  [s]    [kg]     [m]\n";
    std::cout << "  ----   ----     ----\n";

    for (double t = 0; t <= duration; t += dt * 3) {
        double current_fuel = std::max(0.0, fuel_mass - burn_rate * t);
        double total_mass = dry_mass + current_fuel;

        // Dry mass at x=2, fuel at x=4
        auto dry = MassProperties<double>::point_mass(dry_mass, {2.0, 0, 0});
        auto fuel =
            MassProperties<double>::point_mass(current_fuel, {4.0, 0, 0});
        auto vehicle = dry + fuel;

        std::cout << "  " << std::setw(4) << t << "   " << std::setw(5)
                  << vehicle.mass << "   " << std::setw(5) << vehicle.cg(0)
                  << "\n";
    }
    std::cout << "\nNote: CG shifts forward as fuel burns (mass moves toward "
                 "front)\n";
}

// Example 6: Physical validation
void validation_demo() {
    print_header("Physical Validation");

    auto valid_sphere = MassProperties<double>::solid_sphere(100.0, 1.0);
    std::cout << "Solid sphere: is_physically_valid = "
              << (is_physically_valid(valid_sphere) ? "true" : "false") << "\n";

    // Create an invalid inertia tensor (violates triangle inequality)
    auto invalid = MassProperties<double>{
        .mass = 100.0,
        .cg = Vec3<double>::Zero(),
        .inertia = (Mat3<double>() << 1, 0, 0, 0, 1, 0, 0, 0, 100).finished()};
    std::cout << "Invalid tensor (Izz >> Ixx + Iyy): is_physically_valid = "
              << (is_physically_valid(invalid) ? "true" : "false") << "\n";

    auto point = MassProperties<double>::point_mass(10.0, Vec3<double>::Zero());
    std::cout << "Point mass: is_point_mass = "
              << (is_point_mass(point) ? "true" : "false") << "\n";
}

int main() {
    std::cout << "Vulcan Mass Properties Demo\n";
    std::cout << "============================\n";
    std::cout << "Demonstrating mass property utilities for aerospace "
                 "vehicle modeling.\n";

    shape_factories_demo();
    aggregation_demo();
    aggregate_vector_demo();
    parallel_axis_demo();
    variable_mass_demo();
    validation_demo();

    print_header("Demo Complete");
    std::cout << "Mass properties support both numeric (double) and symbolic\n";
    std::cout << "(casadi::MX) types for trajectory optimization.\n";

    return 0;
}
