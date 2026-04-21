// Vulcan Core Types
// Type aliases for Vulcan engineering utilities
// Re-exports commonly used Metis types for dual numeric/symbolic compatibility
#pragma once

#include <metis/core/MetisTypes.hpp>

namespace vulcan {

// =============================================================================
// Fixed-Size Matrix/Vector Templates (re-exported from Metis)
// =============================================================================
// These are templated on Scalar, so they work with both double and casadi::MX

using metis::Mat2;
using metis::Mat3;
using metis::Mat4;
using metis::Vec2;
using metis::Vec3;
using metis::Vec4;

// =============================================================================
// Dynamic-Size Matrix/Vector Templates
// =============================================================================

/// Dynamic-size matrix template (use with Scalar = double or SymbolicScalar)
template <typename Scalar> using Matrix = metis::MetisMatrix<Scalar>;

/// Dynamic-size vector template
template <typename Scalar> using Vector = metis::MetisVector<Scalar>;

// =============================================================================
// Concrete Numeric Types (for when you specifically need double)
// =============================================================================

using NumericScalar = metis::NumericScalar; // double
using NumericMatrix = metis::NumericMatrix; // Eigen::MatrixXd
using NumericVector = metis::NumericVector; // Eigen::VectorXd

// Convenience aliases for fixed-size numeric types
using Vec2d = Vec2<double>;
using Vec3d = Vec3<double>;
using Vec4d = Vec4<double>;
using Mat2d = Mat2<double>;
using Mat3d = Mat3<double>;
using Mat4d = Mat4<double>;

// =============================================================================
// Concrete Symbolic Types (for when you specifically need casadi::MX)
// =============================================================================

using SymbolicScalar = metis::SymbolicScalar; // casadi::MX
using SymbolicMatrix =
    metis::SymbolicMatrix; // Eigen::Matrix<casadi::MX, Dynamic, Dynamic>
using SymbolicVector =
    metis::SymbolicVector; // Eigen::Matrix<casadi::MX, Dynamic, 1>

// Convenience aliases for fixed-size symbolic types
using Vec2s = Vec2<SymbolicScalar>;
using Vec3s = Vec3<SymbolicScalar>;
using Vec4s = Vec4<SymbolicScalar>;
using Mat2s = Mat2<SymbolicScalar>;
using Mat3s = Mat3<SymbolicScalar>;
using Mat4s = Mat4<SymbolicScalar>;

// =============================================================================
// Symbolic Utilities (re-exported for convenience)
// =============================================================================

using metis::as_mx;      // Convert SymbolicVector to casadi::MX
using metis::as_vector;  // Convert casadi::MX to SymbolicVector
using metis::sym;        // Create scalar symbolic variable
using metis::sym_vec;    // Create symbolic vector (returns SymbolicVector)
using metis::sym_vector; // Alias for sym_vec
using metis::to_eigen;   // Convert casadi::MX to Eigen matrix
using metis::to_mx;      // Convert Eigen matrix to casadi::MX

} // namespace vulcan
