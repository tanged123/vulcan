// Vulcan Aerospace Unit Vocabulary
// Curated unit aliases wrapping mp-units v2.5.0 for ergonomic aerospace use
//
// Usage:  Quantity<m>, Quantity<ft>, Quantity<rad_s>
// Instead of: Quantity<mp_units::si::metre>,
// Quantity<mp_units::international::foot>, ...
#pragma once

#include <mp-units/systems/angular.h>
#include <mp-units/systems/international.h>
#include <mp-units/systems/si.h>

namespace vulcan::units {

// =============================================================================
// Base SI units
// =============================================================================
inline constexpr auto m = mp_units::si::metre;
inline constexpr auto kg = mp_units::si::kilogram;
inline constexpr auto s = mp_units::si::second;
inline constexpr auto K = mp_units::si::kelvin;
inline constexpr auto rad = mp_units::si::radian;

// =============================================================================
// Derived SI units
// =============================================================================
inline constexpr auto N = mp_units::si::newton;
inline constexpr auto Pa = mp_units::si::pascal;
inline constexpr auto J = mp_units::si::joule;
inline constexpr auto W = mp_units::si::watt;
inline constexpr auto Hz = mp_units::si::hertz;

// =============================================================================
// Aerospace / non-SI units
// =============================================================================
inline constexpr auto ft = mp_units::international::foot;
inline constexpr auto deg = mp_units::non_si::degree;

inline constexpr auto nmi = mp_units::international::nautical_mile;
inline constexpr auto kn = mp_units::international::knot;
inline constexpr auto lb = mp_units::international::pound;
inline constexpr auto lbf = mp_units::international::pound_force;

// =============================================================================
// SI prefixed units commonly used in aerospace
// =============================================================================
inline constexpr auto km = mp_units::si::kilo<mp_units::si::metre>;
inline constexpr auto mm = mp_units::si::milli<mp_units::si::metre>;

// =============================================================================
// Compound units
// =============================================================================
inline constexpr auto mps = m / s;                       // metres per second
inline constexpr auto fps = ft / s;                      // feet per second
inline constexpr auto rad_s = rad / s;                   // radians per second
inline constexpr auto deg_s = deg / s;                   // degrees per second
inline constexpr auto kph = km / mp_units::non_si::hour; // km/h

// =============================================================================
// Propulsion compound units
// =============================================================================
inline constexpr auto kg_per_s = kg / s;         // mass flow rate [kg/s]
inline constexpr auto m2 = m * m;                // area [m²]
inline constexpr auto per_s = mp_units::one / s; // inverse seconds [1/s]

// =============================================================================
// Atmosphere / fluid dynamics compound units
// =============================================================================
inline constexpr auto m_per_s2 = m / (s * s);       // acceleration [m/s²]
inline constexpr auto kg_per_m3 = kg / (m * m * m); // density [kg/m³]
inline constexpr auto Pa_s = Pa * s;                // dynamic viscosity [Pa·s]

// =============================================================================
// Dimensionless
// =============================================================================
inline constexpr auto dimensionless = mp_units::one;

// =============================================================================
// Long-form aliases (readability in domain-specific code)
// =============================================================================
inline constexpr auto metre = m;
inline constexpr auto kilogram = kg;
inline constexpr auto second = s;
inline constexpr auto kelvin = K;
inline constexpr auto radian = rad;
inline constexpr auto newton = N;
inline constexpr auto pascal = Pa;
inline constexpr auto joule = J;
inline constexpr auto watt = W;
inline constexpr auto hertz = Hz;
inline constexpr auto foot = ft;
inline constexpr auto degree = deg;
inline constexpr auto knot = kn;

} // namespace vulcan::units
