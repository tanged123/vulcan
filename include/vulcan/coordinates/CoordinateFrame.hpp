// Vulcan Coordinate Frame
// Orthonormal basis vector representation for coordinate frames
#pragma once

#include <vulcan/coordinates/EarthModel.hpp>
#include <vulcan/core/VulcanTypes.hpp>

#include <janus/math/Linalg.hpp>
#include <janus/math/Quaternion.hpp>
#include <janus/math/Rotations.hpp>
#include <janus/math/Trig.hpp>

namespace vulcan {

// =============================================================================
// CoordinateFrame - Basis Vector Representation
// =============================================================================

/// Coordinate frame defined by orthonormal basis vectors expressed in ECEF
///
/// This is the central abstraction for coordinate transformations in Vulcan.
/// All frames are represented by three unit basis vectors (X, Y, Z axes)
/// expressed in the ECEF (Earth-Centered Earth-Fixed) frame, plus an origin
/// point. Transformations between frames are performed via dot products
/// (projections) through ECEF as the hub.
///
/// This representation follows the TAOS specification's "Unit Vector
/// Projection Method" which provides O(2N) transformations instead of O(N²)
/// direct matrix pairs.
///
/// Example:
/// @code
///   // Create an ECI frame at some time
///   double gmst = rotation.gmst(t);
///   auto eci = CoordinateFrame<double>::eci(gmst);
///
///   // Transform a vector from ECEF to ECI
///   Vec3<double> v_ecef = {1000.0, 2000.0, 3000.0};
///   Vec3<double> v_eci = eci.from_ecef(v_ecef);
/// @endcode
///
/// @tparam Scalar Scalar type (double for numeric, janus::SymbolicScalar for
/// symbolic)
template <typename Scalar> struct CoordinateFrame {
    Vec3<Scalar> x_axis; ///< Unit X basis vector in ECEF
    Vec3<Scalar> y_axis; ///< Unit Y basis vector in ECEF
    Vec3<Scalar> z_axis; ///< Unit Z basis vector in ECEF
    Vec3<Scalar> origin; ///< Frame origin in ECEF [m]

    // =========================================================================
    // Constructors
    // =========================================================================

    /// Default constructor - creates ECEF identity frame
    CoordinateFrame()
        : x_axis(Vec3<Scalar>::UnitX()), y_axis(Vec3<Scalar>::UnitY()),
          z_axis(Vec3<Scalar>::UnitZ()), origin(Vec3<Scalar>::Zero()) {}

    /// Construct from basis vectors and origin
    CoordinateFrame(const Vec3<Scalar> &x, const Vec3<Scalar> &y,
                    const Vec3<Scalar> &z, const Vec3<Scalar> &o)
        : x_axis(x), y_axis(y), z_axis(z), origin(o) {}

    // =========================================================================
    // Core Transformations
    // =========================================================================

    /// Project vector FROM ECEF TO this frame (rotation only)
    ///
    /// Uses dot product projections onto basis vectors.
    /// This is equivalent to multiplying by the DCM transpose.
    ///
    /// @param v Vector expressed in ECEF
    /// @return Vector expressed in this frame
    [[nodiscard]] Vec3<Scalar> from_ecef(const Vec3<Scalar> &v) const {
        Vec3<Scalar> result;
        result(0) = janus::dot(v, x_axis);
        result(1) = janus::dot(v, y_axis);
        result(2) = janus::dot(v, z_axis);
        return result;
    }

    /// Project vector FROM this frame TO ECEF (rotation only)
    ///
    /// Reconstructs ECEF vector from local components.
    /// This is equivalent to multiplying by the DCM.
    ///
    /// @param v Vector expressed in this frame
    /// @return Vector expressed in ECEF
    [[nodiscard]] Vec3<Scalar> to_ecef(const Vec3<Scalar> &v) const {
        return x_axis * v(0) + y_axis * v(1) + z_axis * v(2);
    }

    /// Transform position FROM ECEF TO this frame (includes origin offset)
    ///
    /// @param pos_ecef Position in ECEF [m]
    /// @return Position in this frame [m]
    [[nodiscard]] Vec3<Scalar>
    position_from_ecef(const Vec3<Scalar> &pos_ecef) const {
        Vec3<Scalar> relative = pos_ecef - origin;
        return from_ecef(relative);
    }

    /// Transform position FROM this frame TO ECEF (includes origin offset)
    ///
    /// @param pos_local Position in this frame [m]
    /// @return Position in ECEF [m]
    [[nodiscard]] Vec3<Scalar>
    position_to_ecef(const Vec3<Scalar> &pos_local) const {
        return to_ecef(pos_local) + origin;
    }

    // =========================================================================
    // Direction Cosine Matrix
    // =========================================================================

    /// Compose DCM as column vectors [x_axis | y_axis | z_axis]
    ///
    /// The DCM transforms vectors from this frame to ECEF:
    /// v_ecef = dcm() * v_local
    ///
    /// @return 3x3 Direction Cosine Matrix
    [[nodiscard]] Mat3<Scalar> dcm() const {
        Mat3<Scalar> R;
        R.col(0) = x_axis;
        R.col(1) = y_axis;
        R.col(2) = z_axis;
        return R;
    }

    /// Compose inverse DCM (ECEF to local)
    ///
    /// The inverse DCM transforms vectors from ECEF to this frame:
    /// v_local = dcm_inverse() * v_ecef
    ///
    /// For orthonormal frames, this is simply the transpose.
    ///
    /// @return 3x3 inverse (transpose) DCM
    [[nodiscard]] Mat3<Scalar> dcm_inverse() const { return dcm().transpose(); }

    // =========================================================================
    // Quaternion Representation
    // =========================================================================

    /// Get frame orientation as quaternion (rotation from ECEF to this frame)
    ///
    /// The quaternion represents the rotation that transforms vectors from
    /// ECEF coordinates to this frame's coordinates.
    ///
    /// @return Quaternion representing frame orientation
    [[nodiscard]] janus::Quaternion<Scalar> quaternion() const {
        // The DCM columns are our basis vectors in ECEF
        // DCM = [x_axis | y_axis | z_axis] transforms local -> ECEF
        // We want ECEF -> local, which is DCM transpose
        // Quaternion from rotation matrix expects ECEF -> local convention
        return janus::Quaternion<Scalar>::from_rotation_matrix(dcm_inverse());
    }

    /// Create frame from quaternion and origin
    ///
    /// The quaternion should represent the rotation from ECEF to the
    /// target frame (i.e., how to transform ECEF vectors to local coords).
    ///
    /// @param q Quaternion (ECEF to local rotation)
    /// @param o Origin in ECEF [m]
    /// @return CoordinateFrame
    static CoordinateFrame
    from_quaternion(const janus::Quaternion<Scalar> &q,
                    const Vec3<Scalar> &o = Vec3<Scalar>::Zero()) {
        // q.to_rotation_matrix() gives R such that v_local = R * v_ecef
        // We need basis vectors in ECEF, so we take R^T columns
        Mat3<Scalar> R = q.to_rotation_matrix();
        Mat3<Scalar> R_T = R.transpose();
        return CoordinateFrame(R_T.col(0), R_T.col(1), R_T.col(2), o);
    }

    // =========================================================================
    // Validation
    // =========================================================================

    /// Check if frame is orthonormal within tolerance
    ///
    /// Verifies:
    /// - All basis vectors are unit length
    /// - All basis vectors are mutually orthogonal
    ///
    /// @param tol Tolerance for numerical checks
    /// @return True if frame is valid
    [[nodiscard]] bool is_valid(double tol = 1e-9) const {
        if constexpr (std::is_floating_point_v<Scalar>) {
            // Check unit lengths
            double norm_x = x_axis.norm();
            double norm_y = y_axis.norm();
            double norm_z = z_axis.norm();
            if (std::abs(norm_x - 1.0) > tol)
                return false;
            if (std::abs(norm_y - 1.0) > tol)
                return false;
            if (std::abs(norm_z - 1.0) > tol)
                return false;

            // Check orthogonality
            double dot_xy = x_axis.dot(y_axis);
            double dot_yz = y_axis.dot(z_axis);
            double dot_xz = x_axis.dot(z_axis);
            if (std::abs(dot_xy) > tol)
                return false;
            if (std::abs(dot_yz) > tol)
                return false;
            if (std::abs(dot_xz) > tol)
                return false;

            return true;
        } else {
            // For symbolic types, validation is not meaningful
            return true;
        }
    }

    // =========================================================================
    // Static Factory Methods
    // =========================================================================

    /// Create ECEF identity frame
    ///
    /// X: Points to prime meridian at equator
    /// Y: Points to 90°E at equator
    /// Z: Points to North Pole
    ///
    /// @return ECEF frame
    static CoordinateFrame ecef() { return CoordinateFrame(); }

    /// Create ECI (Earth-Centered Inertial) frame at given GMST
    ///
    /// The ECI frame is rotated from ECEF about the Z-axis by the
    /// Greenwich Mean Sidereal Time (or Earth Rotation Angle).
    ///
    /// At GMST=0, ECI aligns with ECEF (X points to vernal equinox,
    /// which coincides with Greenwich at epoch).
    ///
    /// @param gmst Greenwich Mean Sidereal Time [rad]
    /// @return ECI frame expressed in ECEF
    static CoordinateFrame eci(Scalar gmst) {
        // ECI is ECEF rotated by -gmst about Z
        // ECI X-axis points to vernal equinox, which is at angle -gmst from
        // Greenwich
        Scalar c = janus::cos(gmst);
        Scalar s = janus::sin(gmst);

        Vec3<Scalar> x_eci, y_eci, z_eci;
        x_eci << c, -s, Scalar(0);
        y_eci << s, c, Scalar(0);
        z_eci << Scalar(0), Scalar(0), Scalar(1);

        return CoordinateFrame(x_eci, y_eci, z_eci, Vec3<Scalar>::Zero());
    }

    /// Create NED (North-East-Down) frame at given geodetic position
    ///
    /// X: Points North (tangent to meridian)
    /// Y: Points East (tangent to parallel)
    /// Z: Points Down (opposite to ellipsoid normal)
    ///
    /// This is the standard aerospace local horizon frame.
    ///
    /// @param lon Longitude [rad]
    /// @param lat Geodetic latitude [rad]
    /// @return NED frame expressed in ECEF
    static CoordinateFrame ned(Scalar lon, Scalar lat) {
        Scalar sin_lat = janus::sin(lat);
        Scalar cos_lat = janus::cos(lat);
        Scalar sin_lon = janus::sin(lon);
        Scalar cos_lon = janus::cos(lon);

        // North: -sin(lat)*cos(lon), -sin(lat)*sin(lon), cos(lat)
        Vec3<Scalar> north;
        north << -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat;

        // East: -sin(lon), cos(lon), 0
        Vec3<Scalar> east;
        east << -sin_lon, cos_lon, Scalar(0);

        // Down: -cos(lat)*cos(lon), -cos(lat)*sin(lon), -sin(lat)
        Vec3<Scalar> down;
        down << -cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat;

        return CoordinateFrame(north, east, down, Vec3<Scalar>::Zero());
    }

    /// Create ENU (East-North-Up) frame at given geodetic position
    ///
    /// X: Points East (tangent to parallel)
    /// Y: Points North (tangent to meridian)
    /// Z: Points Up (ellipsoid normal)
    ///
    /// Common in surveying and some navigation applications.
    ///
    /// @param lon Longitude [rad]
    /// @param lat Geodetic latitude [rad]
    /// @return ENU frame expressed in ECEF
    static CoordinateFrame enu(Scalar lon, Scalar lat) {
        Scalar sin_lat = janus::sin(lat);
        Scalar cos_lat = janus::cos(lat);
        Scalar sin_lon = janus::sin(lon);
        Scalar cos_lon = janus::cos(lon);

        // East: -sin(lon), cos(lon), 0
        Vec3<Scalar> east;
        east << -sin_lon, cos_lon, Scalar(0);

        // North: -sin(lat)*cos(lon), -sin(lat)*sin(lon), cos(lat)
        Vec3<Scalar> north;
        north << -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat;

        // Up: cos(lat)*cos(lon), cos(lat)*sin(lon), sin(lat)
        Vec3<Scalar> up;
        up << cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;

        return CoordinateFrame(east, north, up, Vec3<Scalar>::Zero());
    }
};

// =============================================================================
// Free Functions for Frame Operations
// =============================================================================

/// Transform vector between any two frames (via ECEF hub)
///
/// @param v Vector in source frame
/// @param from Source frame
/// @param to Destination frame
/// @return Vector in destination frame
template <typename Scalar>
Vec3<Scalar> transform_vector(const Vec3<Scalar> &v,
                              const CoordinateFrame<Scalar> &from,
                              const CoordinateFrame<Scalar> &to) {
    // from → ECEF → to
    Vec3<Scalar> v_ecef = from.to_ecef(v);
    return to.from_ecef(v_ecef);
}

/// Transform position between any two frames (via ECEF hub)
///
/// Accounts for origin offsets between frames.
///
/// @param pos Position in source frame
/// @param from Source frame
/// @param to Destination frame
/// @return Position in destination frame
template <typename Scalar>
Vec3<Scalar> transform_position(const Vec3<Scalar> &pos,
                                const CoordinateFrame<Scalar> &from,
                                const CoordinateFrame<Scalar> &to) {
    // from → ECEF → to (with origin handling)
    Vec3<Scalar> pos_ecef = from.position_to_ecef(pos);
    return to.position_from_ecef(pos_ecef);
}

} // namespace vulcan
