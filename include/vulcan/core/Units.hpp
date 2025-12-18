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

/// Convert pounds-mass (lbm) to kilograms
template <typename Scalar> constexpr Scalar lbm_to_kg(const Scalar &lbm) {
    return lbm * 0.45359237;
}

/// Convert kilograms to pounds-mass (lbm)
template <typename Scalar> constexpr Scalar kg_to_lbm(const Scalar &kg) {
    return kg / 0.45359237;
}

/// Convert slugs to kilograms (1 slug = 1 lbf·s²/ft ≈ 14.5939 kg)
template <typename Scalar> constexpr Scalar slug_to_kg(const Scalar &slug) {
    return slug * 14.593903;
}

/// Convert kilograms to slugs
template <typename Scalar> constexpr Scalar kg_to_slug(const Scalar &kg) {
    return kg / 14.593903;
}

/// Convert slugs to pounds-mass (1 slug ≈ 32.174 lbm)
template <typename Scalar> constexpr Scalar slug_to_lbm(const Scalar &slug) {
    return slug * 32.17405;
}

/// Convert pounds-mass to slugs
template <typename Scalar> constexpr Scalar lbm_to_slug(const Scalar &lbm) {
    return lbm / 32.17405;
}

/// Convert pound-force to Newtons
template <typename Scalar> constexpr Scalar lbf_to_N(const Scalar &lbf) {
    return lbf * 4.4482216;
}

/// Convert Newtons to pound-force
template <typename Scalar> constexpr Scalar N_to_lbf(const Scalar &N) {
    return N / 4.4482216;
}

// =============================================================================
// Inertia (Moment of Inertia) Conversions
// =============================================================================

/// Convert slug-ft² to kg-m²
template <typename Scalar>
constexpr Scalar slugft2_to_kgm2(const Scalar &slugft2) {
    // 1 slug-ft² = 14.593903 kg * (0.3048 m)² = 1.35582 kg·m²
    return slugft2 * 1.3558179;
}

/// Convert kg-m² to slug-ft²
template <typename Scalar>
constexpr Scalar kgm2_to_slugft2(const Scalar &kgm2) {
    return kgm2 / 1.3558179;
}

/// Convert lbm-ft² to kg-m²
template <typename Scalar>
constexpr Scalar lbmft2_to_kgm2(const Scalar &lbmft2) {
    // 1 lbm-ft² = 0.45359237 kg * (0.3048 m)² = 0.042140 kg·m²
    return lbmft2 * 0.04214011;
}

/// Convert kg-m² to lbm-ft²
template <typename Scalar> constexpr Scalar kgm2_to_lbmft2(const Scalar &kgm2) {
    return kgm2 / 0.04214011;
}

/// Convert lbm-in² to kg-m²
template <typename Scalar>
constexpr Scalar lbmin2_to_kgm2(const Scalar &lbmin2) {
    // 1 lbm-in² = 0.45359237 kg * (0.0254 m)² = 2.9264e-4 kg·m²
    return lbmin2 * 2.9263966e-4;
}

/// Convert kg-m² to lbm-in²
template <typename Scalar> constexpr Scalar kgm2_to_lbmin2(const Scalar &kgm2) {
    return kgm2 / 2.9263966e-4;
}

/// Convert slug-in² to kg-m²
template <typename Scalar>
constexpr Scalar slugin2_to_kgm2(const Scalar &slugin2) {
    // 1 slug-in² = 14.593903 kg * (0.0254 m)² = 9.4154e-3 kg·m²
    return slugin2 * 9.4154069e-3;
}

/// Convert kg-m² to slug-in²
template <typename Scalar>
constexpr Scalar kgm2_to_slugin2(const Scalar &kgm2) {
    return kgm2 / 9.4154069e-3;
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

/// Convert Rankine to Kelvin
template <typename Scalar> constexpr Scalar R_to_K(const Scalar &R) {
    return R * 5.0 / 9.0;
}

/// Convert Kelvin to Rankine
template <typename Scalar> constexpr Scalar K_to_R(const Scalar &K) {
    return K * 9.0 / 5.0;
}

} // namespace vulcan::units
