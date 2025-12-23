// Vulcan Table Interpolation Wrapper
// Thin wrappers around janus::Interpolator and janus::ScatteredInterpolator
// for atmospheric/aero property lookups
#pragma once

#include <janus/math/Interpolate.hpp>
#include <janus/math/ScatteredInterpolator.hpp>
#include <vector>

namespace vulcan {

// Re-export RBF kernel types for convenience
using janus::RBFKernel;

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

// ============================================================================
// ScatteredTable1D - One-dimensional scattered data interpolation
// ============================================================================

/**
 * @brief 1D scattered data interpolation using Radial Basis Functions
 *
 * Wraps janus::ScatteredInterpolator for single-variable lookups with
 * non-uniform spacing. Uses RBF fitting followed by grid resampling for
 * fast symbolic-compatible queries.
 *
 * @example
 * ```cpp
 * janus::NumericVector x(5), y(5);
 * x << 0.0, 0.3, 0.7, 1.5, 2.0;  // Non-uniform spacing
 * y << 0.0, 0.29, 0.64, 1.0, 0.91;
 *
 * vulcan::ScatteredTable1D table(x, y);
 *
 * double val = table(1.0);  // Interpolate at 1.0
 * std::cout << "RMS error: " << table.reconstruction_error() << "\n";
 * ```
 */
class ScatteredTable1D {
  private:
    janus::ScatteredInterpolator m_interp;

  public:
    /**
     * @brief Construct 1D scattered interpolation table
     *
     * @param x Independent variable values (can be non-uniform)
     * @param y Dependent variable values at x points
     * @param grid_resolution Grid points for resampling (default: 50)
     * @param kernel RBF kernel type (default: ThinPlateSpline)
     * @throw janus::InterpolationError if sizes don't match or < 2 points
     */
    ScatteredTable1D(const janus::NumericVector &x,
                     const janus::NumericVector &y, int grid_resolution = 50,
                     RBFKernel kernel = RBFKernel::ThinPlateSpline)
        : m_interp(x, y, grid_resolution, kernel) {}

    /**
     * @brief Query table at a single point
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
     * @brief Get RMS reconstruction error at original data points
     *
     * Lower values indicate better fit quality.
     */
    double reconstruction_error() const {
        return m_interp.reconstruction_error();
    }

    /**
     * @brief Get number of dimensions (always 1)
     */
    int dims() const { return m_interp.dims(); }

    /**
     * @brief Check if table is valid (initialized)
     */
    bool valid() const { return m_interp.valid(); }
};

// ============================================================================
// ScatteredTableND - N-dimensional scattered data interpolation
// ============================================================================

/**
 * @brief N-dimensional scattered data interpolation using Radial Basis
 * Functions
 *
 * Wraps janus::ScatteredInterpolator for multi-variate lookups with
 * unstructured (non-gridded) data points, such as wind tunnel measurements
 * or CFD samples.
 *
 * @example
 * ```cpp
 * // 2D scattered data: (Mach, alpha) -> CL
 * janus::NumericMatrix points(20, 2);  // 20 test points
 * janus::NumericVector values(20);      // CL measurements
 * // ... fill with wind tunnel data ...
 *
 * vulcan::ScatteredTableND table(points, values, 30);
 *
 * janus::NumericVector query(2);
 * query << 0.8, 5.0;  // Mach=0.8, alpha=5°
 * double cl = table(query);
 * ```
 */
class ScatteredTableND {
  private:
    janus::ScatteredInterpolator m_interp;

  public:
    /**
     * @brief Construct N-D scattered interpolation table
     *
     * @param points Data locations, shape (n_points, n_dims)
     * @param values Function values at each point (length n_points)
     * @param grid_resolution Points per dimension for resampling grid
     * @param kernel RBF kernel type (default: ThinPlateSpline)
     * @param epsilon Shape parameter for Multiquadric/Gaussian kernels
     * @param method Gridded interpolation method for final queries
     * @throw janus::InterpolationError if inputs are invalid
     */
    ScatteredTableND(
        const janus::NumericMatrix &points, const janus::NumericVector &values,
        int grid_resolution = 20, RBFKernel kernel = RBFKernel::ThinPlateSpline,
        double epsilon = 1.0,
        janus::InterpolationMethod method = janus::InterpolationMethod::Linear)
        : m_interp(points, values, grid_resolution, kernel, epsilon, method) {}

    /**
     * @brief Construct with explicit per-dimension grid specification
     *
     * @param points Data locations, shape (n_points, n_dims)
     * @param values Function values at each point
     * @param grid_points Explicit grid coordinates per dimension
     * @param kernel RBF kernel type
     * @param epsilon Shape parameter for Multiquadric/Gaussian kernels
     * @param method Gridded interpolation method for final queries
     */
    ScatteredTableND(
        const janus::NumericMatrix &points, const janus::NumericVector &values,
        const std::vector<janus::NumericVector> &grid_points,
        RBFKernel kernel = RBFKernel::ThinPlateSpline, double epsilon = 1.0,
        janus::InterpolationMethod method = janus::InterpolationMethod::Linear)
        : m_interp(points, values, grid_points, kernel, epsilon, method) {}

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
     * @brief Get RMS reconstruction error at original data points
     *
     * Lower values indicate better fit quality.
     */
    double reconstruction_error() const {
        return m_interp.reconstruction_error();
    }

    /**
     * @brief Get number of input dimensions
     */
    int dims() const { return m_interp.dims(); }

    /**
     * @brief Check if table is valid (initialized)
     */
    bool valid() const { return m_interp.valid(); }
};

} // namespace vulcan
