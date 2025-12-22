# Mass Properties

The `vulcan::mass` namespace provides a comprehensive `MassProperties<Scalar>` struct for modeling rigid body mass, center of gravity, and inertia tensors. It supports aggregation via operator overloading, making it easy to build up vehicle mass properties from components.

## Key Features

- **Factory constructors** — Point mass, shapes (sphere, cylinder, box), full tensor
- **Aggregation** — Combine components with `+`, `-`, `*` operators
- **Parallel axis theorem** — Query inertia about any reference point
- **Symbolic compatible** — Works with `casadi::MX` for optimization

## Frame Convention

All quantities are expressed in **body-fixed coordinates**:

| Field | Description |
|-------|-------------|
| `mass` | Total mass [kg] |
| `cg` | CG position in body frame, from body origin [m] |
| `inertia` | Inertia tensor about CG, in body-fixed axes [kg·m²] |

```
Body Frame (typical aerospace convention):
  X: Forward (out the nose)
  Y: Right (starboard wing)
  Z: Down
```

## Quick Start

```cpp
#include <vulcan/mass/MassProperties.hpp>

using namespace vulcan::mass;

// Create component mass properties
auto fuselage = MassProperties<double>::solid_cylinder(
    500.0,                       // mass [kg]
    0.5,                         // radius [m]
    6.0,                         // length [m]
    Vec3<double>{3.0, 0.0, 0.0}  // CG position
);

auto left_wing = MassProperties<double>::solid_box(
    100.0, 0.5, 4.0, 0.1,        // mass, dx, dy, dz
    Vec3<double>{2.5, -2.5, 0}   // CG position
);

auto right_wing = MassProperties<double>::solid_box(
    100.0, 0.5, 4.0, 0.1,
    Vec3<double>{2.5, 2.5, 0}
);

// Aggregate all components
auto aircraft = fuselage + left_wing + right_wing;

std::cout << "Total mass: " << aircraft.mass << " kg\n";
std::cout << "CG: [" << aircraft.cg.transpose() << "] m\n";
```

## Factory Constructors

### Point Mass
```cpp
auto mp = MassProperties<double>::point_mass(10.0, {1.0, 0.0, 0.0});
// Zero inertia about its own CG
```

### Solid Shapes
```cpp
// Sphere: I = (2/5) * m * r² on all axes
auto sphere = MassProperties<double>::solid_sphere(100.0, 0.5);

// Cylinder (axis along Z): Ixx = Iyy = (1/12)*m*(3r² + L²), Izz = (1/2)*m*r²
auto cylinder = MassProperties<double>::solid_cylinder(50.0, 0.3, 1.0);

// Box: Ixx = (1/12)*m*(dy² + dz²), etc.
auto box = MassProperties<double>::solid_box(80.0, 2.0, 1.0, 0.5);
```

### Full Tensor
```cpp
// Products of inertia use POSITIVE convention: Ixy = ∫xy dm
// Negation applied internally
auto mp = MassProperties<double>::from_components(
    100.0,                        // mass
    Vec3<double>{1.0, 0.0, 0.0},  // CG
    10.0, 20.0, 30.0,             // Ixx, Iyy, Izz
    1.0, 2.0, 3.0                 // Ixy, Ixz, Iyz
);
```

## Aggregation

Mass properties combine using the parallel axis theorem:

```cpp
auto wing = MassProperties<double>::solid_box(100.0, ...);
auto fuse = MassProperties<double>::solid_cylinder(500.0, ...);
auto tail = MassProperties<double>::point_mass(50.0, ...);

// Add components
auto total = wing + fuse + tail;

// Subtract (for hole modeling)
auto drilled = solid - hole;

// Scale
auto half = total * 0.5;
```

### Vector Aggregation

```cpp
std::vector<MassProperties<double>> tanks = {
    MassProperties<double>::point_mass(100.0, {1.0, -1.0, 0.0}),
    MassProperties<double>::point_mass(100.0, {1.0,  1.0, 0.0}),
    MassProperties<double>::point_mass(50.0,  {4.0,  0.0, 0.0})
};

auto total_fuel = aggregate_mass_properties(tanks);
```

## Parallel Axis Theorem

Query inertia about any point (not just CG):

```cpp
auto sphere = MassProperties<double>::solid_sphere(100.0, 0.5, {2.0, 0.0, 0.0});

// Inertia about body origin
auto I_origin = sphere.inertia_about_point(Vec3<double>::Zero());
// Adds m*d² contribution to off-axis moments
```

## Variable Mass

Mass properties are passed as input to dynamics functions, enabling variable mass:

```cpp
for (double t = 0; t < t_final; t += dt) {
    double current_mass = dry_mass + fuel_mass - burn_rate * t;
    
    auto dry = MassProperties<double>::point_mass(dry_mass, {2.0, 0, 0});
    auto fuel = MassProperties<double>::point_mass(fuel_mass - burn_rate * t, {4.0, 0, 0});
    auto vehicle = dry + fuel;
    
    auto derivs = compute_6dof_derivatives(state, F, M, vehicle);
    // CG shifts as fuel burns
}
```

## Symbolic Usage

All operations work with `casadi::MX` for trajectory optimization:

```cpp
using MX = casadi::MX;

auto m1 = janus::sym("m1");
auto x1 = janus::sym("x1");

auto mp1 = MassProperties<MX>::point_mass(m1, Vec3<MX>{x1, MX(0), MX(0)});
auto mp2 = MassProperties<MX>::point_mass(m2, Vec3<MX>{x2, MX(0), MX(0)});

auto combined = mp1 + mp2;  // Symbolic aggregation

janus::Function f("aggregate", {m1, m2, x1, x2}, {combined.mass, combined.cg(0)});
```

## Validation (Numeric Only)

```cpp
auto mp = MassProperties<double>::solid_sphere(100.0, 1.0);

// Check if physically realizable
bool valid = is_physically_valid(mp);  // true

// Check if point mass (zero inertia)
bool pm = is_point_mass(mp);  // false

// Get principal moments (eigenvalues)
Vec3<double> moments = principal_moments(mp);

// Get principal axes (eigenvectors)
Mat3<double> axes = principal_axes(mp);
```

## See Also

- [Rigid Body Dynamics](dynamics.md) — 6DOF equations of motion
- [Propulsion](propulsion.md) — Mass rate from engines
- [Coordinate Frames](coordinates.md) — Body frame definitions
