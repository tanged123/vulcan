#pragma once

#include <janus/math/Arithmetic.hpp>
#include <janus/math/Logic.hpp>
#include <sstream>
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
            std::ostringstream ss;
            ss << name << " must be finite (got " << x << ").";
            throw ValidationError(ss.str());
        }
    }
}

/// Assert value is positive (> 0)
template <typename Scalar>
void assert_positive(const Scalar &x, const std::string &name) {
    if constexpr (!std::is_same_v<Scalar, SymbolicScalar>) {
        if (x <= Scalar(0)) {
            std::ostringstream ss;
            ss << name << " must be positive (got " << x << ").";
            throw ValidationError(ss.str());
        }
    }
}

/// Assert value is non-negative (>= 0)
template <typename Scalar>
void assert_non_negative(const Scalar &x, const std::string &name) {
    if constexpr (!std::is_same_v<Scalar, SymbolicScalar>) {
        if (x < Scalar(0)) {
            std::ostringstream ss;
            ss << name << " must be non-negative (got " << x << ").";
            throw ValidationError(ss.str());
        }
    }
}

/// Assert value is in range [min, max]
template <typename Scalar>
void assert_in_range(const Scalar &x, const std::string &name,
                     const Scalar &min_val, const Scalar &max_val) {
    if constexpr (!std::is_same_v<Scalar, SymbolicScalar>) {
        if (x < min_val || x > max_val) {
            std::ostringstream ss;
            ss << name << " must be in range [" << min_val << ", " << max_val
               << "] (got " << x << ").";
            throw ValidationError(ss.str());
        }
    }
}

/// Assert value is less than or equal to max
template <typename Scalar>
void assert_at_most(const Scalar &x, const std::string &name,
                    const Scalar &max_val) {
    if constexpr (!std::is_same_v<Scalar, SymbolicScalar>) {
        if (x > max_val) {
            std::ostringstream ss;
            ss << name << " must be at most " << max_val << " (got " << x
               << ").";
            throw ValidationError(ss.str());
        }
    }
}

/// Assert value is greater than or equal to min
template <typename Scalar>
void assert_at_least(const Scalar &x, const std::string &name,
                     const Scalar &min_val) {
    if constexpr (!std::is_same_v<Scalar, SymbolicScalar>) {
        if (x < min_val) {
            std::ostringstream ss;
            ss << name << " must be at least " << min_val << " (got " << x
               << ").";
            throw ValidationError(ss.str());
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
        Scalar norm_sq = q.squaredNorm();

        using std::abs;
        if (abs(norm_sq - Scalar(1.0)) > Scalar(tolerance)) {
            std::ostringstream ss;
            ss << "Quaternion must be unit magnitude (squared norm = "
               << norm_sq << ", expected 1.0 ± " << tolerance << ").";
            throw ValidationError(ss.str());
        }
    }
}

} // namespace vulcan::validation
