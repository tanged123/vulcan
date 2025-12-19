// Vulcan Core Types
// Type aliases for Vulcan engineering utilities
// Re-exports commonly used Janus types for dual numeric/symbolic compatibility
#pragma once

#include <janus/core/JanusTypes.hpp>

namespace vulcan {

// =============================================================================
// Fixed-Size Matrix/Vector Templates (re-exported from Janus)
// =============================================================================
// These are templated on Scalar, so they work with both double and casadi::MX

using janus::Mat2;
using janus::Mat3;
using janus::Mat4;
using janus::Vec2;
using janus::Vec3;
using janus::Vec4;

// =============================================================================
// Dynamic-Size Matrix/Vector Templates
// =============================================================================

/// Dynamic-size matrix template (use with Scalar = double or SymbolicScalar)
template <typename Scalar> using Matrix = janus::JanusMatrix<Scalar>;

/// Dynamic-size vector template
template <typename Scalar> using Vector = janus::JanusVector<Scalar>;

// =============================================================================
// Concrete Numeric Types (for when you specifically need double)
// =============================================================================

using NumericScalar = janus::NumericScalar; // double
using NumericMatrix = janus::NumericMatrix; // Eigen::MatrixXd
using NumericVector = janus::NumericVector; // Eigen::VectorXd

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

using SymbolicScalar = janus::SymbolicScalar; // casadi::MX
using SymbolicMatrix =
    janus::SymbolicMatrix; // Eigen::Matrix<casadi::MX, Dynamic, Dynamic>
using SymbolicVector =
    janus::SymbolicVector; // Eigen::Matrix<casadi::MX, Dynamic, 1>

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

using janus::as_mx;      // Convert SymbolicVector to casadi::MX
using janus::as_vector;  // Convert casadi::MX to SymbolicVector
using janus::sym;        // Create scalar symbolic variable
using janus::sym_vec;    // Create symbolic vector (returns SymbolicVector)
using janus::sym_vector; // Alias for sym_vec
using janus::to_eigen;   // Convert casadi::MX to Eigen matrix
using janus::to_mx;      // Convert Eigen matrix to casadi::MX

} // namespace vulcan
