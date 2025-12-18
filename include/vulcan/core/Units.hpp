// Vulcan Unit Conversions
// Common aerospace unit conversion utilities
#pragma once

#include <vulcan/core/Constants.hpp>

namespace vulcan::units {

// =============================================================================
// Angular Conversions
// =============================================================================

/// Convert degrees to radians
template <typename Scalar> constexpr Scalar deg_to_rad(const Scalar &deg) {
    return deg * constants::angle::deg2rad;
}

/// Convert radians to degrees
template <typename Scalar> constexpr Scalar rad_to_deg(const Scalar &rad) {
    return rad * constants::angle::rad2deg;
}

// =============================================================================
// Length Conversions
// =============================================================================

/// Convert feet to meters
template <typename Scalar> constexpr Scalar ft_to_m(const Scalar &ft) {
    return ft * 0.3048;
}

/// Convert meters to feet
template <typename Scalar> constexpr Scalar m_to_ft(const Scalar &m) {
    return m / 0.3048;
}

/// Convert nautical miles to meters
template <typename Scalar> constexpr Scalar nm_to_m(const Scalar &nm) {
    return nm * 1852.0;
}

/// Convert meters to nautical miles
template <typename Scalar> constexpr Scalar m_to_nm(const Scalar &m) {
    return m / 1852.0;
}

/// Convert kilometers to meters
template <typename Scalar> constexpr Scalar km_to_m(const Scalar &km) {
    return km * 1000.0;
}

/// Convert meters to kilometers
template <typename Scalar> constexpr Scalar m_to_km(const Scalar &m) {
    return m / 1000.0;
}

// =============================================================================
// Speed Conversions
// =============================================================================

/// Convert knots to meters per second
template <typename Scalar> constexpr Scalar kts_to_mps(const Scalar &kts) {
    return kts * 0.514444;
}

/// Convert meters per second to knots
template <typename Scalar> constexpr Scalar mps_to_kts(const Scalar &mps) {
    return mps / 0.514444;
}

/// Convert feet per second to meters per second
template <typename Scalar> constexpr Scalar fps_to_mps(const Scalar &fps) {
    return fps * 0.3048;
}

/// Convert meters per second to feet per second
template <typename Scalar> constexpr Scalar mps_to_fps(const Scalar &mps) {
    return mps / 0.3048;
}

// =============================================================================
// Mass/Force Conversions
// =============================================================================

/// Convert pounds to kilograms
template <typename Scalar> constexpr Scalar lb_to_kg(const Scalar &lb) {
    return lb * 0.453592;
}

/// Convert kilograms to pounds
template <typename Scalar> constexpr Scalar kg_to_lb(const Scalar &kg) {
    return kg / 0.453592;
}

/// Convert pound-force to Newtons
template <typename Scalar> constexpr Scalar lbf_to_N(const Scalar &lbf) {
    return lbf * 4.44822;
}

/// Convert Newtons to pound-force
template <typename Scalar> constexpr Scalar N_to_lbf(const Scalar &N) {
    return N / 4.44822;
}

// =============================================================================
// Pressure Conversions
// =============================================================================

/// Convert psi to Pascals
template <typename Scalar> constexpr Scalar psi_to_Pa(const Scalar &psi) {
    return psi * 6894.76;
}

/// Convert Pascals to psi
template <typename Scalar> constexpr Scalar Pa_to_psi(const Scalar &Pa) {
    return Pa / 6894.76;
}

/// Convert atmospheres to Pascals
template <typename Scalar> constexpr Scalar atm_to_Pa(const Scalar &atm) {
    return atm * 101325.0;
}

/// Convert Pascals to atmospheres
template <typename Scalar> constexpr Scalar Pa_to_atm(const Scalar &Pa) {
    return Pa / 101325.0;
}

// =============================================================================
// Temperature Conversions
// =============================================================================

/// Convert Celsius to Kelvin
template <typename Scalar> constexpr Scalar C_to_K(const Scalar &C) {
    return C + 273.15;
}

/// Convert Kelvin to Celsius
template <typename Scalar> constexpr Scalar K_to_C(const Scalar &K) {
    return K - 273.15;
}

/// Convert Fahrenheit to Kelvin
template <typename Scalar> constexpr Scalar F_to_K(const Scalar &F) {
    return (F - 32.0) * 5.0 / 9.0 + 273.15;
}

/// Convert Kelvin to Fahrenheit
template <typename Scalar> constexpr Scalar K_to_F(const Scalar &K) {
    return (K - 273.15) * 9.0 / 5.0 + 32.0;
}

} // namespace vulcan::units
