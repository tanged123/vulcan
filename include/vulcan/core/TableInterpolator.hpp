// Vulcan Table Interpolation Wrapper
// Thin wrappers around janus::Interpolator for atmospheric/aero property
// lookups
#pragma once

#include <janus/math/Interpolate.hpp>
#include <vector>

namespace vulcan {

// ============================================================================
// Table1D - One-dimensional table interpolation
// ============================================================================

/**
 * @brief 1D table interpolation wrapper
 *
 * Wraps janus::Interpolator for single-variable lookups (e.g., altitude →
 * temperature). Supports both numeric (double) and symbolic (casadi::MX)
 * evaluation.
 *
 * @example
 * ```cpp
 * janus::NumericVector alt(5), temp(5);
 * alt << 0, 5, 10, 15, 20;       // km
 * temp << 288, 256, 223, 217, 217; // K
 *
 * vulcan::Table1D table(alt, temp);
 *
 * double T = table(7.5);  // Interpolate at 7.5 km
 * ```
 */
class Table1D {
  private:
    janus::Interpolator m_interp;

  public:
    /**
     * @brief Construct 1D interpolation table
     *
     * @param x Independent variable grid (must be sorted, ascending)
     * @param y Dependent variable values at grid points
     * @param method Interpolation method (default: Linear)
     * @throw janus::InterpolationError if x is not sorted or sizes don't match
     */
    Table1D(
        const janus::NumericVector &x, const janus::NumericVector &y,
        janus::InterpolationMethod method = janus::InterpolationMethod::Linear)
        : m_interp(x, y, method) {}

    /**
     * @brief Query table at a single point
     *
     * Values outside the grid are clamped to the boundary values.
     *
     * @tparam Scalar Query type (double or janus::SymbolicScalar)
     * @param x Query point
     * @return Interpolated value
     */
    template <janus::JanusScalar Scalar>
    Scalar operator()(const Scalar &x) const {
        return m_interp(x);
    }

    /**
     * @brief Query table at multiple points (batch evaluation)
     *
     * @tparam Derived Eigen expression type
     * @param x Vector of query points
     * @return Vector of interpolated values
     */
    template <typename Derived>
    auto operator()(const Eigen::MatrixBase<Derived> &x) const {
        return m_interp(x);
    }

    /**
     * @brief Get the interpolation method
     */
    janus::InterpolationMethod method() const { return m_interp.method(); }

    /**
     * @brief Check if table is valid (initialized)
     */
    bool valid() const { return m_interp.valid(); }
};

// ============================================================================
// TableND - N-dimensional table interpolation
// ============================================================================

/**
 * @brief N-dimensional table interpolation wrapper
 *
 * Wraps janus::Interpolator for multi-variate lookups
 * (e.g., Mach + altitude + angle → aerodynamic coefficient).
 *
 * Values must be provided in Fortran (column-major) order:
 * For a 2D table with x_grid (size m) and y_grid (size n),
 * values should be ordered as: z(0,0), z(1,0), ..., z(m-1,0), z(0,1), ...,
 * z(m-1,n-1)
 *
 * @example
 * ```cpp
 * janus::NumericVector x(2), y(3);
 * x << 0, 1;
 * y << 0, 1, 2;
 *
 * janus::NumericVector values(6);  // 2 x 3 = 6 values
 * values << 0, 1, 0, 1, 0, 1;      // Fortran order
 *
 * vulcan::TableND table({x, y}, values);
 *
 * janus::NumericVector query(2);
 * query << 0.5, 1.5;
 * double result = table(query);
 * ```
 */
class TableND {
  private:
    janus::Interpolator m_interp;
    int m_dims;

  public:
    /**
     * @brief Construct N-dimensional interpolation table
     *
     * @param grid_points Vector of 1D grids for each dimension
     * @param values Flattened values in Fortran (column-major) order
     * @param method Interpolation method (default: Linear)
     * @throw janus::InterpolationError if grids are not sorted or sizes don't
     * match
     */
    TableND(
        const std::vector<janus::NumericVector> &grid_points,
        const janus::NumericVector &values,
        janus::InterpolationMethod method = janus::InterpolationMethod::Linear)
        : m_interp(grid_points, values, method),
          m_dims(static_cast<int>(grid_points.size())) {}

    /**
     * @brief Query table at a single N-D point
     *
     * @tparam Scalar Scalar type (double or janus::SymbolicScalar)
     * @param x Query point (size must match dims())
     * @return Interpolated value
     */
    template <janus::JanusScalar Scalar>
    Scalar operator()(const janus::JanusVector<Scalar> &x) const {
        return m_interp(x);
    }

    /**
     * @brief Query table at multiple N-D points (batch evaluation)
     *
     * @tparam Derived Eigen expression type
     * @param x Matrix of query points (rows = points, cols = dims)
     * @return Vector of interpolated values
     */
    template <typename Derived>
    auto operator()(const Eigen::MatrixBase<Derived> &x) const {
        return m_interp(x);
    }

    /**
     * @brief Get number of dimensions
     */
    int dims() const { return m_dims; }

    /**
     * @brief Get the interpolation method
     */
    janus::InterpolationMethod method() const { return m_interp.method(); }

    /**
     * @brief Check if table is valid (initialized)
     */
    bool valid() const { return m_interp.valid(); }
};

} // namespace vulcan
