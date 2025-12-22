// Vulcan Rigid Body Dynamics Types
// State, derivative, and mass property structures for 6DOF simulation
#pragma once

#include <vulcan/core/VulcanTypes.hpp>
#include <vulcan/mass/MassProperties.hpp>

#include <janus/math/Quaternion.hpp>

// =============================================================================
// State Structures
// =============================================================================

namespace vulcan::dynamics {

// Import MassProperties from mass namespace for backward compatibility
using vulcan::mass::aggregate_mass_properties;
using vulcan::mass::MassProperties;
using vulcan::mass::transform_mass_properties;

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
