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
//
// Dual-mode storage:
//   - When Rep satisfies mp-units Representation (double, float, ...):
//     wraps mp_units::quantity<Unit, Rep> for full mp-units interop.
//   - When Rep does NOT satisfy mp-units Representation (casadi::MX):
//     stores Rep directly; uses mp-units only at the TYPE level for unit
//     algebra and compile-time conversion factors.
#pragma once

#include <mp-units/framework.h>
#include <mp-units/systems/si.h>
#include <vulcan/quantity/Units.hpp>

#include <concepts>
#include <string>
#include <type_traits>

namespace vulcan {

namespace detail {

// Detect whether a type satisfies mp-units' Representation requirements.
// casadi::MX fails because operator== returns MX, not bool.
template <typename Rep, auto Unit>
concept MpUnitsCompatible = requires {
    typename mp_units::quantity<Unit, Rep>;
    requires requires(Rep v) {
        { mp_units::quantity<Unit, Rep>{v, Unit} };
    };
};

// Compile-time conversion factor from FromUnit to ToUnit as a double.
// We use mp-units with double as a surrogate: create 1.0 in FromUnit, convert
// to ToUnit via force_in, and read out the numeric factor.
template <auto FromUnit, auto ToUnit> consteval double conversion_factor() {
    if constexpr (FromUnit == ToUnit) {
        return 1.0;
    } else {
        return mp_units::quantity<FromUnit, double>{1.0, FromUnit}
            .force_numerical_value_in(ToUnit);
    }
}

} // namespace detail

// ============================================================================
// Quantity<Unit, Rep> — native mp-units storage
// ============================================================================
// Primary template: for mp-units-compatible Reps (double, float, etc.)

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

    // -- Formatting
    // ------------------------------------------------------------

    /// Human-readable string: "<value> <unit_symbol>"
    /// Definition provided in QuantityFormat.hpp.
    [[nodiscard]] std::string to_string() const;

    // -- Dimensionless implicit conversion -----------------------------------

    /// Implicit conversion to Rep when the unit is dimensionless
    /// (mp_units::one).
    constexpr operator Rep() const
        requires(Unit == mp_units::one)
    {
        return value();
    }
};

// ============================================================================
// Quantity<Unit, Rep> — symbolic storage (e.g. casadi::MX)
// ============================================================================
// Partial specialization: when Rep does NOT satisfy mp-units Representation.
// We cannot instantiate mp_units::quantity<Unit, MX> because MX::operator==
// returns MX (not bool), violating std::equality_comparable.
// Instead, we store the raw Rep directly and use mp-units only for compile-time
// unit algebra and conversion factors.

template <auto Unit, typename Rep>
    requires(!detail::MpUnitsCompatible<Rep, Unit>)
class Quantity<Unit, Rep> {
    static_assert(mp_units::Unit<std::remove_const_t<decltype(Unit)>>,
                  "First template argument must be an mp-units Unit");

    Rep val_;

  public:
    // -- Member types / constants -------------------------------------------
    using rep = Rep;
    static constexpr auto unit = Unit;

    // -- Construction -------------------------------------------------------

    /// Default: zero-initialised.
    Quantity() : val_{Rep{0}} {}

    /// Construct from a raw numeric value.
    explicit Quantity(const Rep &val) : val_{val} {}

    // -- Observers ----------------------------------------------------------

    /// Return the numeric value in the DECLARED unit.
    [[nodiscard]] Rep value() const { return val_; }

    // -- Unit conversion ----------------------------------------------------

    /// Convert to a different unit using compile-time conversion factor.
    /// Example: Quantity<ft, MX>(x).in<m>()  -->  x * 0.3048
    template <auto ToUnit> [[nodiscard]] auto in() const {
        constexpr double factor = detail::conversion_factor<Unit, ToUnit>();
        if constexpr (factor == 1.0) {
            return Quantity<ToUnit, Rep>{val_};
        } else {
            return Quantity<ToUnit, Rep>{Rep(val_ * Rep(factor))};
        }
    }

    // -- Same-unit arithmetic -----------------------------------------------

    friend Quantity operator+(const Quantity &lhs, const Quantity &rhs) {
        return Quantity{Rep(lhs.val_ + rhs.val_)};
    }

    friend Quantity operator-(const Quantity &lhs, const Quantity &rhs) {
        return Quantity{Rep(lhs.val_ - rhs.val_)};
    }

    Quantity operator-() const { return Quantity{Rep(-val_)}; }

    Quantity &operator+=(const Quantity &rhs) {
        val_ = Rep(val_ + rhs.val_);
        return *this;
    }

    Quantity &operator-=(const Quantity &rhs) {
        val_ = Rep(val_ - rhs.val_);
        return *this;
    }

    // -- Cross-unit multiplication / division --------------------------------
    // Result unit is determined by mp-units at compile time.
    // We compute the result unit via mp-units type algebra (using double as a
    // surrogate to query the result unit), then store the symbolic value.

    template <auto U2, typename Rep2>
    [[nodiscard]] friend auto operator*(const Quantity &lhs,
                                        const Quantity<U2, Rep2> &rhs) {
        // Determine result unit via mp-units type-level algebra (with double)
        using result_q = decltype(mp_units::quantity<Unit, double>{1.0, Unit} *
                                  mp_units::quantity<U2, double>{1.0, U2});
        static constexpr auto ResultUnit = result_q::unit;
        using CommonRep = std::common_type_t<Rep, Rep2>;
        return Quantity<ResultUnit, CommonRep>{
            CommonRep(lhs.val_ * rhs.value())};
    }

    template <auto U2, typename Rep2>
    [[nodiscard]] friend auto operator/(const Quantity &lhs,
                                        const Quantity<U2, Rep2> &rhs) {
        using result_q = decltype(mp_units::quantity<Unit, double>{1.0, Unit} /
                                  mp_units::quantity<U2, double>{1.0, U2});
        static constexpr auto ResultUnit = result_q::unit;
        using CommonRep = std::common_type_t<Rep, Rep2>;
        return Quantity<ResultUnit, CommonRep>{
            CommonRep(lhs.val_ / rhs.value())};
    }

    // -- Scalar multiplication / division ------------------------------------

    friend Quantity operator*(const Quantity &q, const Rep &s) {
        return Quantity{Rep(q.val_ * s)};
    }
    friend Quantity operator*(const Rep &s, const Quantity &q) {
        return Quantity{Rep(s * q.val_)};
    }
    friend Quantity operator/(const Quantity &q, const Rep &s) {
        return Quantity{Rep(q.val_ / s)};
    }

    // -- Comparisons (return Rep-native types for symbolic) -----------------

    friend auto operator==(const Quantity &a, const Quantity &b) {
        return a.val_ == b.val_;
    }
    friend auto operator!=(const Quantity &a, const Quantity &b) {
        return a.val_ != b.val_;
    }
    friend auto operator<(const Quantity &a, const Quantity &b) {
        return a.val_ < b.val_;
    }
    friend auto operator>(const Quantity &a, const Quantity &b) {
        return a.val_ > b.val_;
    }
    friend auto operator<=(const Quantity &a, const Quantity &b) {
        return a.val_ <= b.val_;
    }
    friend auto operator>=(const Quantity &a, const Quantity &b) {
        return a.val_ >= b.val_;
    }

    // -- Formatting
    // ------------------------------------------------------------

    /// Human-readable string: "[symbolic] <unit_symbol>"
    /// Definition provided in QuantityFormat.hpp.
    [[nodiscard]] std::string to_string() const;

    // -- Dimensionless implicit conversion -----------------------------------

    /// Implicit conversion to Rep when the unit is dimensionless.
    operator Rep() const
        requires(Unit == mp_units::one)
    {
        return val_;
    }
};

} // namespace vulcan
