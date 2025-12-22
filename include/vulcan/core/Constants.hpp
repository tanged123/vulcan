// Vulcan Physical Constants
// Consolidated library of physical constants for aerospace applications
#pragma once

namespace vulcan::constants {

// =============================================================================
// =============================================================================
// Earth Constants
// =============================================================================
namespace earth {
/// Gravitational parameter (GM) [m^3/s^2]
inline constexpr double mu = 3.986004418e14;

/// Equatorial radius [m] (WGS84)
inline constexpr double R_eq = 6378137.0;

/// Polar radius [m]
inline constexpr double R_pol = 6356752.3142;

/// Mean radius [m]
inline constexpr double R_mean = 6371008.8;

/// Flattening (WGS84)
inline constexpr double f = 1.0 / 298.257223563;

/// J2 zonal harmonic coefficient
inline constexpr double J2 = 1.08263e-3;

/// J3 zonal harmonic coefficient
inline constexpr double J3 = -2.54e-6;

/// J4 zonal harmonic coefficient
inline constexpr double J4 = -1.61e-6;

/// Angular velocity [rad/s]
inline constexpr double omega = 7.2921159e-5;
} // namespace earth

// =============================================================================
// WGS84 Ellipsoid Constants
// =============================================================================
namespace wgs84 {
/// Semi-major axis (equatorial radius) [m]
inline constexpr double a = earth::R_eq;

/// Flattening
inline constexpr double f = earth::f;

/// Semi-minor axis (polar radius) [m]
inline constexpr double b = a * (1.0 - f);

/// First eccentricity squared
inline constexpr double e2 = 2.0 * f - f * f;

/// Second eccentricity squared
inline constexpr double e_prime2 = e2 / (1.0 - e2);

/// Gravitational parameter [m^3/s^2]
inline constexpr double mu = earth::mu;

/// Angular velocity [rad/s]
inline constexpr double omega = earth::omega;
} // namespace wgs84

// =============================================================================
// Atmospheric Constants
// =============================================================================
namespace atmosphere {
/// Sea level temperature [K]
inline constexpr double T0 = 288.15;

/// Sea level pressure [Pa]
inline constexpr double P0 = 101325.0;

/// Sea level density [kg/m^3]
inline constexpr double rho0 = 1.225;

/// Temperature lapse rate in troposphere [K/m]
inline constexpr double L = 0.0065;

/// Tropopause altitude [m]
inline constexpr double h_tropopause = 11000.0;

/// Molar mass of air [kg/mol]
inline constexpr double M = 0.0289644;

/// Gas constant for air [J/(kg·K)]
inline constexpr double R_air = 287.05287;
// Alias for backward compatibility if needed, but R_air is more explicit
inline constexpr double R = R_air;

/// Universal gas constant [J/(mol·K)]
inline constexpr double R_universal = 8.31447;

/// Ratio of specific heats for air
inline constexpr double gamma = 1.4;
} // namespace atmosphere

// =============================================================================
// Physics Constants
// =============================================================================
namespace physics {
/// Speed of light in vacuum [m/s] (Exact)
inline constexpr double c = 299792458.0;

/// Gravitational constant [m^3/(kg·s^2)] (CODATA 2018)
inline constexpr double G = 6.67430e-11;

/// Boltzmann constant [J/K] (Exact)
inline constexpr double k_B = 1.380649e-23;

/// Stefan-Boltzmann constant [W/(m^2·K^4)] (CODATA 2018)
inline constexpr double sigma = 5.670374e-8;

/// Standard gravity [m/s^2]
inline constexpr double g0 = 9.80665;
} // namespace physics

// =============================================================================
// Angular Conversion Constants
// =============================================================================
namespace angle {
/// Pi
inline constexpr double pi = 3.14159265358979323846;

/// Degrees to radians conversion factor
inline constexpr double deg2rad = pi / 180.0;

/// Radians to degrees conversion factor
inline constexpr double rad2deg = 180.0 / pi;

/// Arcseconds to radians
inline constexpr double arcsec2rad = deg2rad / 3600.0;
} // namespace angle

} // namespace vulcan::constants
