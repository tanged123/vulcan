// Vulcan Quantity Math Overloads
// Janus-compatible math functions for Quantity<Unit, Rep> types.
//
// Tier 2: Dimension-preserving (abs, min, max, clamp, where, floor, ceil,
//         round, sign)
// Tier 3: Dimension-changing  (sin, cos, tan, asin, acos, atan2, sqrt,
//         wrap_to_pi, wrap_to_2pi)
#pragma once

#include <mp-units/math.h>
#include <vulcan/quantity/Quantity.hpp>

#include <janus/math/Arithmetic.hpp>
#include <janus/math/Logic.hpp>
#include <janus/math/Trig.hpp>

#include <vulcan/core/Units.hpp>

namespace vulcan {

// ============================================================================
// Tier 2 — Dimension-preserving (unit in = unit out)
// ============================================================================

template <auto U, typename Rep> auto abs(Quantity<U, Rep> x) {
    return Quantity<U, Rep>{janus::abs(x.value())};
}

template <auto U, typename Rep> auto floor(Quantity<U, Rep> x) {
    return Quantity<U, Rep>{janus::floor(x.value())};
}

template <auto U, typename Rep> auto ceil(Quantity<U, Rep> x) {
    return Quantity<U, Rep>{janus::ceil(x.value())};
}

template <auto U, typename Rep> auto round(Quantity<U, Rep> x) {
    return Quantity<U, Rep>{janus::round(x.value())};
}

template <auto U, typename Rep> auto sign(Quantity<U, Rep> x) {
    return Quantity<U, Rep>{janus::sign(x.value())};
}

template <auto U, typename Rep>
auto min(Quantity<U, Rep> a, Quantity<U, Rep> b) {
    return Quantity<U, Rep>{janus::min(a.value(), b.value())};
}

template <auto U, typename Rep>
auto max(Quantity<U, Rep> a, Quantity<U, Rep> b) {
    return Quantity<U, Rep>{janus::max(a.value(), b.value())};
}

template <auto U, typename Rep>
auto clamp(Quantity<U, Rep> val, Quantity<U, Rep> lo, Quantity<U, Rep> hi) {
    return Quantity<U, Rep>{janus::clamp(val.value(), lo.value(), hi.value())};
}

template <typename Cond, auto U, typename Rep>
auto where(const Cond &cond, Quantity<U, Rep> if_true,
           Quantity<U, Rep> if_false) {
    return Quantity<U, Rep>{
        janus::where(cond, if_true.value(), if_false.value())};
}

// ============================================================================
// Tier 3 — Dimension-changing (trig, sqrt, wrap)
// ============================================================================

// --- Trig: Quantity<rad> → Quantity<dimensionless> ---

template <auto U, typename Rep>
    requires(U == vulcan::units::rad)
auto sin(Quantity<U, Rep> x) {
    return Quantity<vulcan::units::dimensionless, Rep>{janus::sin(x.value())};
}

template <auto U, typename Rep>
    requires(U == vulcan::units::rad)
auto cos(Quantity<U, Rep> x) {
    return Quantity<vulcan::units::dimensionless, Rep>{janus::cos(x.value())};
}

template <auto U, typename Rep>
    requires(U == vulcan::units::rad)
auto tan(Quantity<U, Rep> x) {
    return Quantity<vulcan::units::dimensionless, Rep>{janus::tan(x.value())};
}

// --- Inverse trig: Quantity<dimensionless> → Quantity<rad> ---

template <auto U, typename Rep>
    requires(U == vulcan::units::dimensionless)
auto asin(Quantity<U, Rep> x) {
    return Quantity<vulcan::units::rad, Rep>{janus::asin(x.value())};
}

template <auto U, typename Rep>
    requires(U == vulcan::units::dimensionless)
auto acos(Quantity<U, Rep> x) {
    return Quantity<vulcan::units::rad, Rep>{janus::acos(x.value())};
}

// --- atan2: two dimensionless → Quantity<rad> ---

template <auto U, typename Rep>
    requires(U == vulcan::units::dimensionless)
auto atan2(Quantity<U, Rep> y, Quantity<U, Rep> x) {
    return Quantity<vulcan::units::rad, Rep>{
        janus::atan2(y.value(), x.value())};
}

// --- atan2: two same-unit quantities → Quantity<rad> (e.g., m/m) ---

template <auto U, typename Rep>
    requires(U != vulcan::units::dimensionless)
auto atan2(Quantity<U, Rep> y, Quantity<U, Rep> x) {
    return Quantity<vulcan::units::rad, Rep>{
        janus::atan2(y.value(), x.value())};
}

// --- sqrt: uses mp-units to compute result unit ---

template <auto U, typename Rep>
    requires detail::MpUnitsCompatible<Rep, U>
auto sqrt(Quantity<U, Rep> x) {
    auto result_q = mp_units::sqrt(x.raw());
    using ResultQ = decltype(result_q);
    constexpr auto result_unit = ResultQ::unit;
    return Quantity<result_unit, Rep>{janus::sqrt(x.value())};
}

// --- wrap_to_pi / wrap_to_2pi: Quantity<rad> → Quantity<rad> ---

template <auto U, typename Rep>
    requires(U == vulcan::units::rad)
auto wrap_to_pi(Quantity<U, Rep> x) {
    return Quantity<U, Rep>{vulcan::units::wrap_to_pi(x.value())};
}

template <auto U, typename Rep>
    requires(U == vulcan::units::rad)
auto wrap_to_2pi(Quantity<U, Rep> x) {
    return Quantity<U, Rep>{vulcan::units::wrap_to_2pi(x.value())};
}

} // namespace vulcan
