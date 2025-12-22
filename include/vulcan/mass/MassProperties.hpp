// Vulcan Mass Properties
// Full mass properties for rigid body components with aggregation support
#pragma once

#include <vulcan/core/VulcanTypes.hpp>

#include <janus/janus.hpp>

#include <type_traits>
#include <vector>

namespace vulcan::mass {

/// Full mass properties for a rigid body component
///
/// All quantities are expressed in **body-fixed coordinates**:
/// - `cg`: Position of the center of gravity in the body frame, measured from
///   the body frame origin (typically a reference point like nose, or
///   structural origin) [m]
/// - `inertia`: Inertia tensor about the CG, expressed in body-fixed axes
///   (i.e., the tensor components Ixx, Ixy, etc. are with respect to body
///   X, Y, Z axes) [kg·m²]
///
/// ## Frame Convention
///
/// ```
/// Body Frame (typical aerospace convention):
///   - X: Forward (out the nose)
///   - Y: Right (starboard wing)
///   - Z: Down
///
/// CG position is a 3-vector [x_cg, y_cg, z_cg] in body coordinates.
/// Inertia tensor is 3x3, about CG, with components in body axes.
/// ```
///
/// When passed to dynamics functions like `compute_6dof_derivatives()`, the
/// mass properties must be consistent with the body frame used for forces,
/// moments, and angular velocity.
///
/// ## Inertia Tensor Convention
///
/// Uses the STANDARD mathematical convention:
/// ```
///   | Ixx  Ixy  Ixz |
///   | Ixy  Iyy  Iyz |
///   | Ixz  Iyz  Izz |
/// ```
///
/// where Ixy = -∫xy dm (negative product of inertia).
///
/// This differs from some CAD tools (SolidWorks, NX) which use opposite signs.
///
/// Reference: https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
///
/// ## Aggregation
///
/// Mass properties can be combined using operator+:
/// ```cpp
/// auto total = wing + fuselage + tail;
/// ```
///
/// The combined CG is the mass-weighted average, and inertia is computed
/// using the parallel axis theorem. All components must be expressed in the
/// **same body frame** for aggregation to be meaningful.
template <typename Scalar> struct MassProperties {
    Scalar mass;     ///< Total mass [kg]
    Vec3<Scalar> cg; ///< CG position in body frame [m]
    Mat3<Scalar>
        inertia; ///< Inertia tensor about CG, in body-fixed axes [kg·m²]

    // =========================================================================
    // Factory Constructors
    // =========================================================================

    /// Create a point mass at a specified location
    ///
    /// A point mass has zero inertia about its own CG.
    ///
    /// @param m Mass [kg]
    /// @param position CG position [m]
    static MassProperties<Scalar> point_mass(const Scalar &m,
                                             const Vec3<Scalar> &position) {
        return MassProperties<Scalar>{
            .mass = m, .cg = position, .inertia = Mat3<Scalar>::Zero()};
    }

    /// Create mass properties with diagonal inertia tensor
    ///
    /// @param m Mass [kg]
    /// @param cg_pos CG position [m]
    /// @param Ixx Principal moment about X [kg·m²]
    /// @param Iyy Principal moment about Y [kg·m²]
    /// @param Izz Principal moment about Z [kg·m²]
    static MassProperties<Scalar> diagonal(const Scalar &m,
                                           const Vec3<Scalar> &cg_pos,
                                           const Scalar &Ixx, const Scalar &Iyy,
                                           const Scalar &Izz) {
        Mat3<Scalar> I = Mat3<Scalar>::Zero();
        I(0, 0) = Ixx;
        I(1, 1) = Iyy;
        I(2, 2) = Izz;
        return MassProperties<Scalar>{.mass = m, .cg = cg_pos, .inertia = I};
    }

    /// Create mass properties with full 6-component inertia tensor
    ///
    /// Products of inertia use POSITIVE convention: Ixy_input = ∫xy dm
    /// The function applies negation internally to form the standard tensor.
    ///
    /// @param m Mass [kg]
    /// @param cg_pos CG position [m]
    /// @param Ixx Moment about X [kg·m²]
    /// @param Iyy Moment about Y [kg·m²]
    /// @param Izz Moment about Z [kg·m²]
    /// @param Ixy Product of inertia (positive convention) [kg·m²]
    /// @param Ixz Product of inertia (positive convention) [kg·m²]
    /// @param Iyz Product of inertia (positive convention) [kg·m²]
    static MassProperties<Scalar>
    from_components(const Scalar &m, const Vec3<Scalar> &cg_pos,
                    const Scalar &Ixx, const Scalar &Iyy, const Scalar &Izz,
                    const Scalar &Ixy, const Scalar &Ixz, const Scalar &Iyz) {
        Mat3<Scalar> I;
        I(0, 0) = Ixx;
        I(0, 1) = -Ixy;
        I(0, 2) = -Ixz;
        I(1, 0) = -Ixy;
        I(1, 1) = Iyy;
        I(1, 2) = -Iyz;
        I(2, 0) = -Ixz;
        I(2, 1) = -Iyz;
        I(2, 2) = Izz;
        return MassProperties<Scalar>{.mass = m, .cg = cg_pos, .inertia = I};
    }

    // =========================================================================
    // Shape Factories
    // =========================================================================

    /// Create mass properties for a solid sphere
    ///
    /// I = (2/5) * m * r² for all axes
    ///
    /// @param m Mass [kg]
    /// @param radius Sphere radius [m]
    /// @param cg_pos CG position [m] (default: origin)
    static MassProperties<Scalar>
    solid_sphere(const Scalar &m, const Scalar &radius,
                 const Vec3<Scalar> &cg_pos = Vec3<Scalar>::Zero()) {
        Scalar I_diag = Scalar(0.4) * m * radius * radius; // 2/5 = 0.4
        return diagonal(m, cg_pos, I_diag, I_diag, I_diag);
    }

    /// Create mass properties for a solid cylinder (axis along Z)
    ///
    /// Ixx = Iyy = (1/12) * m * (3*r² + L²)
    /// Izz = (1/2) * m * r²
    ///
    /// @param m Mass [kg]
    /// @param radius Cylinder radius [m]
    /// @param length Cylinder length along Z [m]
    /// @param cg_pos CG position [m] (default: origin)
    static MassProperties<Scalar>
    solid_cylinder(const Scalar &m, const Scalar &radius, const Scalar &length,
                   const Vec3<Scalar> &cg_pos = Vec3<Scalar>::Zero()) {
        Scalar r2 = radius * radius;
        Scalar L2 = length * length;
        Scalar Ixx = (Scalar(1) / Scalar(12)) * m * (Scalar(3) * r2 + L2);
        Scalar Izz = Scalar(0.5) * m * r2;
        return diagonal(m, cg_pos, Ixx, Ixx, Izz);
    }

    /// Create mass properties for a solid rectangular box
    ///
    /// Ixx = (1/12) * m * (dy² + dz²)
    /// Iyy = (1/12) * m * (dx² + dz²)
    /// Izz = (1/12) * m * (dx² + dy²)
    ///
    /// @param m Mass [kg]
    /// @param dx Width in X [m]
    /// @param dy Width in Y [m]
    /// @param dz Width in Z [m]
    /// @param cg_pos CG position [m] (default: origin)
    static MassProperties<Scalar>
    solid_box(const Scalar &m, const Scalar &dx, const Scalar &dy,
              const Scalar &dz,
              const Vec3<Scalar> &cg_pos = Vec3<Scalar>::Zero()) {
        Scalar k = Scalar(1) / Scalar(12);
        Scalar Ixx = k * m * (dy * dy + dz * dz);
        Scalar Iyy = k * m * (dx * dx + dz * dz);
        Scalar Izz = k * m * (dx * dx + dy * dy);
        return diagonal(m, cg_pos, Ixx, Iyy, Izz);
    }

    // =========================================================================
    // Backward Compatibility Factories
    // =========================================================================

    /// Create from mass only (point mass at origin)
    ///
    /// @deprecated Use point_mass() instead
    static MassProperties<Scalar> from_mass(const Scalar &m) {
        // Return with identity-scaled inertia for legacy compatibility
        Mat3<Scalar> I = Mat3<Scalar>::Identity();
        I(0, 0) = m;
        I(1, 1) = m;
        I(2, 2) = m;
        return MassProperties<Scalar>{
            .mass = m, .cg = Vec3<Scalar>::Zero(), .inertia = I};
    }

    /// Create with diagonal inertia at origin (4-arg overload)
    ///
    /// @deprecated Use diagonal(m, cg, Ixx, Iyy, Izz) instead
    static MassProperties<Scalar> diagonal(const Scalar &m, const Scalar &Ixx,
                                           const Scalar &Iyy,
                                           const Scalar &Izz) {
        return diagonal(m, Vec3<Scalar>::Zero(), Ixx, Iyy, Izz);
    }

    /// Create with full inertia at origin (7-arg overload)
    ///
    /// @deprecated Use from_components() instead
    static MassProperties<Scalar> full(const Scalar &m, const Scalar &Ixx,
                                       const Scalar &Iyy, const Scalar &Izz,
                                       const Scalar &Ixy, const Scalar &Ixz,
                                       const Scalar &Iyz) {
        return from_components(m, Vec3<Scalar>::Zero(), Ixx, Iyy, Izz, Ixy, Ixz,
                               Iyz);
    }

    // =========================================================================
    // Parallel Axis Theorem
    // =========================================================================

    /// Get inertia tensor about an arbitrary point
    ///
    /// Uses the parallel axis theorem (tensor generalization):
    /// J = I + m * (|r|² * Identity - r ⊗ r)
    ///
    /// where r is the vector from the point to the CG.
    ///
    /// Reference:
    /// https://en.wikipedia.org/wiki/Parallel_axis_theorem#Tensor_generalization
    ///
    /// @param point The reference point [m]
    /// @return Inertia tensor about that point [kg·m²]
    Mat3<Scalar> inertia_about_point(const Vec3<Scalar> &point) const {
        Vec3<Scalar> r = cg - point;
        Scalar r_dot_r = janus::dot(r, r);

        // J = I + m * (|r|² * I_3 - r ⊗ r)
        Mat3<Scalar> J = inertia;
        J(0, 0) += mass * (r_dot_r - r(0) * r(0));
        J(1, 1) += mass * (r_dot_r - r(1) * r(1));
        J(2, 2) += mass * (r_dot_r - r(2) * r(2));
        J(0, 1) -= mass * r(0) * r(1);
        J(1, 0) -= mass * r(0) * r(1);
        J(0, 2) -= mass * r(0) * r(2);
        J(2, 0) -= mass * r(0) * r(2);
        J(1, 2) -= mass * r(1) * r(2);
        J(2, 1) -= mass * r(1) * r(2);

        return J;
    }

    // =========================================================================
    // Aggregation Operators
    // =========================================================================

    /// Add two mass properties (aggregation)
    ///
    /// Computes combined CG as mass-weighted average, then uses parallel axis
    /// theorem to express both inertias about the combined CG and sums them.
    MassProperties<Scalar>
    operator+(const MassProperties<Scalar> &other) const {
        Scalar total_mass = mass + other.mass;

        // Combined CG (mass-weighted average)
        Vec3<Scalar> combined_cg =
            (mass * cg + other.mass * other.cg) / total_mass;

        // Transform both inertias to combined CG and sum
        Mat3<Scalar> I1_at_cg = inertia_about_point(combined_cg);
        Mat3<Scalar> I2_at_cg = other.inertia_about_point(combined_cg);
        Mat3<Scalar> combined_inertia = I1_at_cg + I2_at_cg;

        return MassProperties<Scalar>{
            .mass = total_mass, .cg = combined_cg, .inertia = combined_inertia};
    }

    /// Subtract mass properties (hole removal)
    ///
    /// Used for modeling holes or removed material.
    MassProperties<Scalar>
    operator-(const MassProperties<Scalar> &other) const {
        return *this + (Scalar(-1) * other);
    }

    /// Scale mass properties by a factor
    ///
    /// Scales mass and inertia; CG remains unchanged.
    MassProperties<Scalar> operator*(const Scalar &factor) const {
        return MassProperties<Scalar>{
            .mass = mass * factor, .cg = cg, .inertia = inertia * factor};
    }

    /// Divide mass properties by a factor
    MassProperties<Scalar> operator/(const Scalar &factor) const {
        return *this * (Scalar(1) / factor);
    }

    /// In-place addition
    MassProperties<Scalar> &operator+=(const MassProperties<Scalar> &other) {
        *this = *this + other;
        return *this;
    }

    // =========================================================================
    // Accessors (for backward compatibility)
    // =========================================================================

    /// Get CG offset from origin (alias for cg)
    ///
    /// @deprecated Use cg directly
    Vec3<Scalar> cg_offset() const { return cg; }
};

/// Left-multiplication: factor * props
template <typename Scalar>
MassProperties<Scalar> operator*(const Scalar &factor,
                                 const MassProperties<Scalar> &props) {
    return props * factor;
}

// =============================================================================
// Free Functions
// =============================================================================

/// Aggregate a collection of mass properties
///
/// Computes combined CG and inertia tensor about combined CG.
/// Equivalent to sum() in Python.
///
/// @param components Vector of component mass properties
/// @return Combined mass properties
template <typename Scalar>
MassProperties<Scalar> aggregate_mass_properties(
    const std::vector<MassProperties<Scalar>> &components) {
    if (components.empty()) {
        return MassProperties<Scalar>{.mass = Scalar(0),
                                      .cg = Vec3<Scalar>::Zero(),
                                      .inertia = Mat3<Scalar>::Zero()};
    }

    MassProperties<Scalar> result = components[0];
    for (size_t i = 1; i < components.size(); ++i) {
        result = result + components[i];
    }
    return result;
}

/// Transform mass properties to a new coordinate frame
///
/// @param props Original mass properties
/// @param rotation Rotation matrix from old frame to new frame
/// @param translation Translation from old origin to new origin (in old frame)
/// @return Mass properties expressed in new frame
template <typename Scalar>
MassProperties<Scalar>
transform_mass_properties(const MassProperties<Scalar> &props,
                          const Mat3<Scalar> &rotation,
                          const Vec3<Scalar> &translation) {
    // Transform CG position
    Vec3<Scalar> new_cg = rotation * (props.cg - translation);

    // Transform inertia tensor: I' = R * I * R^T
    Mat3<Scalar> new_inertia = rotation * props.inertia * rotation.transpose();

    return MassProperties<Scalar>{
        .mass = props.mass, .cg = new_cg, .inertia = new_inertia};
}

// =============================================================================
// Numeric-Only Functions (require eigenvalue decomposition)
// =============================================================================

/// Check if mass properties are physically valid (numeric only)
///
/// Conditions checked:
/// 1. mass > 0
/// 2. Principal moments > 0
/// 3. Triangle inequality: I1 + I2 >= I3 (for all permutations)
///
/// @return true if physically possible
template <typename Scalar>
bool is_physically_valid(const MassProperties<Scalar> &props) {
    static_assert(std::is_same_v<Scalar, double>,
                  "is_physically_valid only works with double");

    if (props.mass < 0) {
        return false;
    }

    // Compute eigenvalues for principal moments
    Eigen::SelfAdjointEigenSolver<Mat3<double>> solver(props.inertia);
    Vec3<double> eigs = solver.eigenvalues();

    // All principal moments must be non-negative
    if (eigs(0) < 0 || eigs(1) < 0 || eigs(2) < 0) {
        return false;
    }

    // Triangle inequality
    if (eigs(0) + eigs(1) < eigs(2) || eigs(0) + eigs(2) < eigs(1) ||
        eigs(1) + eigs(2) < eigs(0)) {
        return false;
    }

    return true;
}

/// Check if effectively a point mass (zero inertia)
template <typename Scalar>
bool is_point_mass(const MassProperties<Scalar> &props) {
    static_assert(std::is_same_v<Scalar, double>,
                  "is_point_mass only works with double");
    return props.inertia.norm() < 1e-12;
}

/// Compute principal moments of inertia (eigenvalues, numeric only)
template <typename Scalar>
Vec3<Scalar> principal_moments(const MassProperties<Scalar> &props) {
    static_assert(std::is_same_v<Scalar, double>,
                  "principal_moments only works with double");
    Eigen::SelfAdjointEigenSolver<Mat3<double>> solver(props.inertia);
    return solver.eigenvalues();
}

/// Compute principal axes rotation matrix (eigenvectors, numeric only)
template <typename Scalar>
Mat3<Scalar> principal_axes(const MassProperties<Scalar> &props) {
    static_assert(std::is_same_v<Scalar, double>,
                  "principal_axes only works with double");
    Eigen::SelfAdjointEigenSolver<Mat3<double>> solver(props.inertia);
    return solver.eigenvectors();
}

} // namespace vulcan::mass
