// Vulcan Rigid Body Dynamics Types
// State, derivative, and mass property structures for 6DOF simulation
#pragma once

#include <vulcan/core/VulcanTypes.hpp>

#include <janus/math/Quaternion.hpp>

namespace vulcan::dynamics {

// =============================================================================
// Mass Properties
// =============================================================================

/// Mass properties for a rigid body
///
/// This is passed AS INPUT to dynamics functions, not stored internally.
/// The caller provides the current mass properties at each timestep,
/// enabling variable mass vehicles (rockets, aircraft with fuel burn).
///
/// The inertia tensor is a full 3x3 SYMMETRIC matrix:
///   | Ixx  -Ixy  -Ixz |
///   |-Ixy   Iyy  -Iyz |
///   |-Ixz  -Iyz   Izz |
///
/// Sign convention: Products of inertia (Ixy, Ixz, Iyz) use the POSITIVE
/// convention, i.e., Ixy = ∫ xy dm. The negative signs appear in the tensor.
template <typename Scalar> struct MassProperties {
    Scalar mass;            ///< Current mass [kg]
    Mat3<Scalar> inertia;   ///< Full inertia tensor in body frame [kg·m²]
    Vec3<Scalar> cg_offset; ///< CG offset from geometric center [m]

    /// Construct from mass only (point mass at origin)
    static MassProperties<Scalar> from_mass(const Scalar &m) {
        Mat3<Scalar> I = Mat3<Scalar>::Identity();
        I(0, 0) = m;
        I(1, 1) = m;
        I(2, 2) = m;
        return MassProperties<Scalar>{
            .mass = m, .inertia = I, .cg_offset = Vec3<Scalar>::Zero()};
    }

    /// Construct for diagonal inertia tensor (principal axes aligned with body)
    static MassProperties<Scalar> diagonal(const Scalar &m, const Scalar &Ixx,
                                           const Scalar &Iyy,
                                           const Scalar &Izz) {
        Mat3<Scalar> I = Mat3<Scalar>::Zero();
        I(0, 0) = Ixx;
        I(1, 1) = Iyy;
        I(2, 2) = Izz;
        return MassProperties<Scalar>{
            .mass = m, .inertia = I, .cg_offset = Vec3<Scalar>::Zero()};
    }

    /// Construct with full inertia tensor (6 unique components)
    ///
    /// Products of inertia use POSITIVE convention: Ixy = ∫ xy dm
    /// The negative signs are applied internally to form the proper tensor.
    static MassProperties<Scalar> full(const Scalar &m, const Scalar &Ixx,
                                       const Scalar &Iyy, const Scalar &Izz,
                                       const Scalar &Ixy, const Scalar &Ixz,
                                       const Scalar &Iyz) {
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
        return MassProperties<Scalar>{
            .mass = m, .inertia = I, .cg_offset = Vec3<Scalar>::Zero()};
    }
};

// =============================================================================
// State Structures
// =============================================================================

/// Rigid body state in body-fixed frame
///
/// Position/velocity relative to an implicit reference frame (e.g., ECI/ECEF)
template <typename Scalar> struct RigidBodyState {
    Vec3<Scalar> position;              ///< Position in reference frame [m]
    Vec3<Scalar> velocity_body;         ///< Velocity in body frame [m/s]
    janus::Quaternion<Scalar> attitude; ///< Body-to-reference quaternion
    Vec3<Scalar> omega_body; ///< Angular velocity in body frame [rad/s]
};

/// Time derivatives of rigid body state
template <typename Scalar> struct RigidBodyDerivatives {
    Vec3<Scalar> position_dot; ///< Velocity in reference frame [m/s]
    Vec3<Scalar> velocity_dot; ///< Acceleration in body frame [m/s²]
    janus::Quaternion<Scalar> attitude_dot; ///< Quaternion rate
    Vec3<Scalar> omega_dot; ///< Angular accel in body frame [rad/s²]
};

} // namespace vulcan::dynamics
