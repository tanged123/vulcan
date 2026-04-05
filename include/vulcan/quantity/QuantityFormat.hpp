// Vulcan Quantity Formatting
// Provides the to_string() implementation for Quantity<Unit, Rep>.
//
// Include this header when you need to_string() on Quantity values.
// It is intentionally separate from Quantity.hpp to avoid pulling <sstream>
// into every translation unit that uses Quantity.
#pragma once

#include <vulcan/quantity/Quantity.hpp>

#include <mp-units/framework.h>

#include <sstream>
#include <string>

namespace vulcan {

// ============================================================================
// to_string() — numeric (mp-units-compatible) specialisation
// ============================================================================

template <auto Unit, typename Rep>
std::string Quantity<Unit, Rep>::to_string() const {
    std::ostringstream oss;
    oss << value();
    // unit_symbol_to writes the unit's textual symbol into an output iterator.
    // For dimensionless (mp_units::one) there is no meaningful symbol, so we
    // skip the separator and symbol entirely.
    if constexpr (Unit != mp_units::one) {
        oss << ' ';
        mp_units::unit_symbol_to<char>(std::ostream_iterator<char>(oss), Unit);
    }
    return oss.str();
}

// ============================================================================
// to_string() — symbolic (non-mp-units-compatible) specialisation
// ============================================================================

template <auto Unit, typename Rep>
    requires(!detail::MpUnitsCompatible<Rep, Unit>)
std::string Quantity<Unit, Rep>::to_string() const {
    std::ostringstream oss;
    oss << "[symbolic]";
    if constexpr (Unit != mp_units::one) {
        oss << ' ';
        mp_units::unit_symbol_to<char>(std::ostream_iterator<char>(oss), Unit);
    }
    return oss.str();
}

} // namespace vulcan
