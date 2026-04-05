// Vulcan Quantity Type
// A thin, ergonomic wrapper around mp_units::quantity for aerospace use.
//
// Template parameters:
//   Unit — an mp-units unit value (e.g. vulcan::units::m, vulcan::units::ft)
//   Rep  — the numeric representation (double, float, or casadi::MX)
//
// Design goals:
//   - value() returns the raw number in the DECLARED unit (no silent SI
//   conversion)
//   - Cross-unit arithmetic delegates to mp-units for correct unit algebra
//   - Dimensionless quantities implicitly convert to Rep
//   - Comparisons return Rep-native types (compatible with janus::where)
#pragma once

#include <mp-units/framework.h>
#include <mp-units/systems/si.h>
#include <vulcan/quantity/Units.hpp>

#include <concepts>
#include <type_traits>

namespace vulcan {

// ============================================================================
// Quantity<Unit, Rep>
// ============================================================================

template <auto Unit, typename Rep = double> class Quantity {
    static_assert(mp_units::Unit<std::remove_const_t<decltype(Unit)>>,
                  "First template argument must be an mp-units Unit");

    // The underlying mp-units quantity type.
    // We build a reference from the unit to let mp-units track quantity spec.
    using raw_type = mp_units::quantity<Unit, Rep>;

    raw_type q_;

  public:
    // -- Member types / constants -------------------------------------------
    using rep = Rep;
    static constexpr auto unit = Unit;

    // -- Construction -------------------------------------------------------

    /// Default: zero-initialised.
    constexpr Quantity() : q_{Rep{0}, Unit} {}

    /// Construct from a raw numeric value (explicit to prevent accidental unit
    /// loss).
    constexpr explicit Quantity(const Rep &val) : q_{val, Unit} {}

    /// Construct from an mp_units::quantity (explicit).
    template <auto R2, typename Rep2>
        requires std::constructible_from<raw_type, mp_units::quantity<R2, Rep2>>
    constexpr explicit Quantity(const mp_units::quantity<R2, Rep2> &mq)
        : q_{mq} {}

    // -- Observers ----------------------------------------------------------

    /// Return the numeric value in the DECLARED unit.
    /// Quantity<ft>(1000).value() == 1000, NOT 304.8.
    [[nodiscard]] constexpr Rep value() const {
        return q_.numerical_value_ref_in(Unit);
    }

    /// Return the underlying mp_units::quantity for interop.
    [[nodiscard]] constexpr const raw_type &raw() const { return q_; }

    // -- Unit conversion ----------------------------------------------------

    /// Convert to a different unit. Returns a new Quantity in ToUnit.
    /// Example: Quantity<ft>(1000).in<m>()
    template <auto ToUnit> [[nodiscard]] constexpr auto in() const {
        auto converted = q_.force_in(ToUnit);
        return Quantity<ToUnit, Rep>{converted};
    }

    // -- Same-unit arithmetic -----------------------------------------------

    friend constexpr Quantity operator+(const Quantity &lhs,
                                        const Quantity &rhs) {
        return Quantity{lhs.value() + rhs.value()};
    }

    friend constexpr Quantity operator-(const Quantity &lhs,
                                        const Quantity &rhs) {
        return Quantity{lhs.value() - rhs.value()};
    }

    constexpr Quantity operator-() const { return Quantity{-value()}; }

    constexpr Quantity &operator+=(const Quantity &rhs) {
        q_ += rhs.q_;
        return *this;
    }

    constexpr Quantity &operator-=(const Quantity &rhs) {
        q_ -= rhs.q_;
        return *this;
    }

    // -- Cross-unit multiplication / division --------------------------------

    template <auto U2, typename Rep2>
    [[nodiscard]] friend constexpr auto
    operator*(const Quantity &lhs, const Quantity<U2, Rep2> &rhs) {
        auto result = lhs.q_ * rhs.raw();
        using result_type = decltype(result);
        return Quantity<result_type::unit, typename result_type::rep>{result};
    }

    template <auto U2, typename Rep2>
    [[nodiscard]] friend constexpr auto
    operator/(const Quantity &lhs, const Quantity<U2, Rep2> &rhs) {
        auto result = lhs.q_ / rhs.raw();
        using result_type = decltype(result);
        return Quantity<result_type::unit, typename result_type::rep>{result};
    }

    // -- Scalar multiplication / division ------------------------------------

    friend constexpr Quantity operator*(const Quantity &q, const Rep &s) {
        return Quantity{q.value() * s};
    }
    friend constexpr Quantity operator*(const Rep &s, const Quantity &q) {
        return Quantity{s * q.value()};
    }
    friend constexpr Quantity operator/(const Quantity &q, const Rep &s) {
        return Quantity{q.value() / s};
    }

    // -- Comparisons (return bool for double, Rep-native for symbolic) ------

    friend constexpr auto operator==(const Quantity &a, const Quantity &b) {
        return a.value() == b.value();
    }
    friend constexpr auto operator!=(const Quantity &a, const Quantity &b) {
        return a.value() != b.value();
    }
    friend constexpr auto operator<(const Quantity &a, const Quantity &b) {
        return a.value() < b.value();
    }
    friend constexpr auto operator>(const Quantity &a, const Quantity &b) {
        return a.value() > b.value();
    }
    friend constexpr auto operator<=(const Quantity &a, const Quantity &b) {
        return a.value() <= b.value();
    }
    friend constexpr auto operator>=(const Quantity &a, const Quantity &b) {
        return a.value() >= b.value();
    }

    // -- Dimensionless implicit conversion -----------------------------------

    /// Implicit conversion to Rep when the unit is dimensionless
    /// (mp_units::one).
    constexpr operator Rep() const
        requires(Unit == mp_units::one)
    {
        return value();
    }
};

} // namespace vulcan
