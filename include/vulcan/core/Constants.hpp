// Vulcan Physical Constants
// Consolidated library of physical constants for aerospace applications
//
// Constants are expressed as Quantity<unit> values where feasible.
// Complex compound units (kg/mol, J/(kg·K), etc.) are kept as raw double
// because mp-units constexpr type algebra cannot represent them directly.
#pragma once

#include <vulcan/quantity/Quantity.hpp>
#include <vulcan/quantity/Units.hpp>

namespace vulcan::constants {

// Helper alias to reduce verbosity
template <auto Unit> using Q = vulcan::Quantity<Unit>;

// =============================================================================
// =============================================================================
// Earth Constants
// =============================================================================
namespace earth {
/// Gravitational parameter (GM) [m^3/s^2]
inline constexpr Q<vulcan::units::m * vulcan::units::m * vulcan::units::m /
                   (vulcan::units::s * vulcan::units::s)>
    mu{3.986004418e14};

/// Equatorial radius [m] (WGS84)
inline constexpr Q<vulcan::units::m> R_eq{6378137.0};

/// Polar radius [m]
inline constexpr Q<vulcan::units::m> R_pol{6356752.3142};

/// Mean radius [m]
inline constexpr Q<vulcan::units::m> R_mean{6371008.8};

/// Flattening (WGS84)
inline constexpr Q<vulcan::units::dimensionless> f{1.0 / 298.257223563};

/// J2 zonal harmonic coefficient
inline constexpr Q<vulcan::units::dimensionless> J2{1.08263e-3};

/// J3 zonal harmonic coefficient
inline constexpr Q<vulcan::units::dimensionless> J3{-2.54e-6};

/// J4 zonal harmonic coefficient
inline constexpr Q<vulcan::units::dimensionless> J4{-1.61e-6};

/// Angular velocity [rad/s]
inline constexpr Q<vulcan::units::rad_s> omega{7.2921159e-5};
} // namespace earth

// =============================================================================
// WGS84 Ellipsoid Constants
// =============================================================================
namespace wgs84 {
/// Semi-major axis (equatorial radius) [m]
inline constexpr Q<vulcan::units::m> a{earth::R_eq.value()};

/// Flattening
inline constexpr Q<vulcan::units::dimensionless> f{earth::f.value()};

/// Semi-minor axis (polar radius) [m]
inline constexpr Q<vulcan::units::m> b{a.value() * (1.0 - f.value())};

/// First eccentricity squared
inline constexpr Q<vulcan::units::dimensionless> e2{2.0 * f.value() -
                                                    f.value() * f.value()};

/// Second eccentricity squared
inline constexpr Q<vulcan::units::dimensionless> e_prime2{e2.value() /
                                                          (1.0 - e2.value())};

/// Gravitational parameter [m^3/s^2]
inline constexpr Q<vulcan::units::m * vulcan::units::m * vulcan::units::m /
                   (vulcan::units::s * vulcan::units::s)>
    mu{earth::mu.value()};

/// Angular velocity [rad/s]
inline constexpr Q<vulcan::units::rad_s> omega{earth::omega.value()};
} // namespace wgs84

// =============================================================================
// Atmospheric Constants
// =============================================================================
namespace atmosphere {
/// Sea level temperature [K]
inline constexpr Q<vulcan::units::K> T0{288.15};

/// Sea level pressure [Pa]
inline constexpr Q<vulcan::units::Pa> P0{101325.0};

/// Sea level density [kg/m^3]
// complex unit: kg/(m^3) — kept as raw double
inline constexpr double rho0 = 1.225;

/// Temperature lapse rate in troposphere [K/m]
inline constexpr Q<vulcan::units::K / vulcan::units::m> L{0.0065};

/// Tropopause altitude [m]
inline constexpr Q<vulcan::units::m> h_tropopause{11000.0};

/// Molar mass of air [kg/mol]
// complex unit: kg/mol — kept as raw double
inline constexpr double M = 0.0289644;

/// Gas constant for air [J/(kg·K)]
// complex unit: J/(kg·K) — kept as raw double
inline constexpr double R_air = 287.05287;
// Alias for backward compatibility if needed, but R_air is more explicit
inline constexpr double R = R_air;

/// Universal gas constant [J/(mol·K)]
// complex unit: J/(mol·K) — kept as raw double
inline constexpr double R_universal = 8.31447;

/// Ratio of specific heats for air
inline constexpr Q<vulcan::units::dimensionless> gamma{1.4};
} // namespace atmosphere

// =============================================================================
// Physics Constants
// =============================================================================
namespace physics {
/// Speed of light in vacuum [m/s] (Exact)
inline constexpr Q<vulcan::units::mps> c{299792458.0};

/// Gravitational constant [m^3/(kg·s^2)] (CODATA 2018)
// complex unit: m^3/(kg·s^2) — kept as raw double
inline constexpr double G = 6.67430e-11;

/// Boltzmann constant [J/K] (Exact)
// complex unit: J/K — kept as raw double
inline constexpr double k_B = 1.380649e-23;

/// Stefan-Boltzmann constant [W/(m^2·K^4)] (CODATA 2018)
// complex unit: W/(m^2·K^4) — kept as raw double
inline constexpr double sigma = 5.670374e-8;

/// Standard gravity [m/s^2]
inline constexpr Q<vulcan::units::m / (vulcan::units::s * vulcan::units::s)> g0{
    9.80665};
} // namespace physics

// =============================================================================
// Sun Constants
// =============================================================================
namespace sun {
/// Astronomical Unit [m] - exact IAU 2012 definition
inline constexpr Q<vulcan::units::m> AU{149597870700.0};

/// Gravitational parameter (GM) [m³/s²]
inline constexpr Q<vulcan::units::m * vulcan::units::m * vulcan::units::m /
                   (vulcan::units::s * vulcan::units::s)>
    mu{1.32712440018e20};

/// Mean radius [m]
inline constexpr Q<vulcan::units::m> radius{6.96e8};
} // namespace sun

// =============================================================================
// Moon Constants
// =============================================================================
namespace moon {
/// Gravitational parameter (GM) [m³/s²]
inline constexpr Q<vulcan::units::m * vulcan::units::m * vulcan::units::m /
                   (vulcan::units::s * vulcan::units::s)>
    mu{4.9028695e12};

/// Mean radius [m]
inline constexpr Q<vulcan::units::m> radius{1.7374e6};

/// Mean Earth-Moon distance [m]
inline constexpr Q<vulcan::units::m> mean_distance{3.844e8};
} // namespace moon

// =============================================================================
// Angular Conversion Constants
// =============================================================================
namespace angle {
/// Pi [rad]
inline constexpr Q<vulcan::units::rad> pi{3.14159265358979323846};

/// Degrees to radians conversion factor (dimensionless ratio, kept as double)
inline constexpr double deg2rad = 3.14159265358979323846 / 180.0;

/// Radians to degrees conversion factor (dimensionless ratio, kept as double)
inline constexpr double rad2deg = 180.0 / 3.14159265358979323846;

/// Arcseconds to radians (dimensionless ratio, kept as double)
inline constexpr double arcsec2rad = deg2rad / 3600.0;
} // namespace angle

} // namespace vulcan::constants
