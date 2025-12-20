# Vulcan Atmosphere Models Walkthrough

This guide explains how to use the Atmospheric Models in Vulcan, specifically the US Standard Atmosphere 1976. It covers both numeric retrieval of atmospheric properties and symbolic usage for optimization with the Janus framework. The guide follows `examples/atmosphere/atmosphere_profile.cpp`.

## 1. US Standard Atmosphere 1976

Vulcan implements the US Standard Atmosphere 1976, providing properties like temperature, pressure, density, speed of sound, and gravity as a function of altitude.

### Basic Usage (Numeric Mode)

You can retrieve individual properties or the full state at a given altitude.

```cpp
#include <vulcan/vulcan.hpp>

// ...

double altitude_m = 10000.0; // 10 km

// Retrieve single property
double rho = vulcan::ussa1976::density(altitude_m);

// Retrieve full state struct
auto atm = vulcan::ussa1976::state(altitude_m);

std::cout << "Temperature: " << atm.temperature << " K\n";
std::cout << "Pressure:    " << atm.pressure << " Pa\n";
std::cout << "Density:     " << atm.density << " kg/m^3\n";
std::cout << "Speed of Sound: " << atm.speed_of_sound << " m/s\n";
std::cout << "Gravity:     " << atm.gravity << " m/s^2\n";
```

## 2. Symbolic Mode (Optimization)

A key feature of Vulcan is its compatibility with Janus for symbolic computation and optimization. The atmospheric models are templated on `Scalar` type, allowing them to accept `janus::SymbolicScalar` (usually `casadi::MX`) for optimization problems.

### Example: Finding Altitude for a Target Density

In this example, we use `janus::Opti` to find the altitude where the air density matches a specific target.

```cpp
#include <janus/optimization/Opti.hpp>
#include <vulcan/vulcan.hpp>

// Templated wrapper function
template <typename Scalar> Scalar air_density(const Scalar &altitude) {
    return vulcan::ussa1976::density(altitude);
}

int main() {
    janus::Opti opti;
    
    // Define optimization variable (altitude)
    auto h = opti.variable(5000.0); // Initial guess: 5 km
    
    // Symbolic calculation of density
    auto rho_sym = air_density(h);
    
    // Define objective: minimize squared difference from target
    double target_rho = 0.5; // kg/m^3
    auto objective = (rho_sym - target_rho) * (rho_sym - target_rho);
    
    opti.minimize(objective);
    
    // Constraints on altitude
    opti.subject_to(h >= 0.0);
    opti.subject_to(h <= 50000.0);
    
    // Solve the problem
    auto sol = opti.solve();
    
    double h_opt = static_cast<double>(sol.value(h));
    std::cout << "Optimal Altitude: " << h_opt << " m for density " << target_rho << "\n";
}
```

This demonstrates the "write once, run anywhere" philosophy of Vulcan/Janus: the same `ussa1976::density` function works for both standard `double` calculations and symbolic graph generation for optimization solvers.
