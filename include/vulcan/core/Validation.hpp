#pragma once

#include <janus/math/Arithmetic.hpp>
#include <janus/math/Logic.hpp>
#include <string>
#include <type_traits>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanError.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::validation {

// =============================================================================
// Checks (Boolean)
// =============================================================================

/// Check if value is finite (not NaN or Inf)
template <typename Scalar> bool is_finite(const Scalar &x) {
    if constexpr (std::is_same_v<Scalar, SymbolicScalar>) {
        return true; // Symbolics are assumed finite/checked by solver
    } else {
        using std::isfinite;
        return isfinite(x);
    }
}

/// Check if value is in range [min, max] inclusive
/// Returns boolean for numeric, symbolic logic for symbolic
template <typename Scalar>
auto is_in_range(const Scalar &x, const Scalar &min, const Scalar &max) {
    return x >= min && x <= max;
}

// =============================================================================
// Math Utilities
// =============================================================================

/// Clamp value to [min, max]
template <typename Scalar>
Scalar clamp(const Scalar &x, const Scalar &min, const Scalar &max) {
    // return min(max(x, min_val), max_val)
    return janus::min(janus::max(x, min), max);
}

// =============================================================================
// Assertions (Throws on failure)
// =============================================================================

/// Assert value is finite
template <typename Scalar>
void assert_finite(const Scalar &x, const std::string &name) {
    if constexpr (!std::is_same_v<Scalar, SymbolicScalar>) {
        if (!is_finite(x)) {
            throw ValidationError(name + " must be finite.");
        }
    }
}

/// Assert value is positive (> 0)
template <typename Scalar>
void assert_positive(const Scalar &x, const std::string &name) {
    if constexpr (!std::is_same_v<Scalar, SymbolicScalar>) {
        if (x <= Scalar(0)) {
            throw ValidationError(name + " must be positive.");
        }
    }
}

/// Assert value is non-negative (>= 0)
template <typename Scalar>
void assert_non_negative(const Scalar &x, const std::string &name) {
    if constexpr (!std::is_same_v<Scalar, SymbolicScalar>) {
        if (x < Scalar(0)) {
            throw ValidationError(name + " must be non-negative.");
        }
    }
}

/// Assert quaternion is unit magnitude
/// @param q Quaternion (w, x, y, z) or similar vector of size 4
/// @param tolerance Tolerance for magnitude check
template <typename VectorType>
void assert_unit_quaternion(const VectorType &q, double tolerance = 1e-6) {
    using Scalar = typename VectorType::Scalar;

    if constexpr (!std::is_same_v<Scalar, SymbolicScalar>) {
        // Compute norm squared
        Scalar norm_sq = q.squaredNorm();

        using std::abs;
        if (abs(norm_sq - Scalar(1.0)) > Scalar(tolerance)) {
            throw ValidationError("Quaternion is not unit magnitude.");
        }
    }
}

} // namespace vulcan::validation
