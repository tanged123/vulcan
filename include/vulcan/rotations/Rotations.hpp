// Vulcan Rotations Library
// Comprehensive rotation utilities for aerospace applications
//
// This header includes all rotation-related functionality:
// - All 12 Euler angle sequences with DCM and quaternion conversions
// - DCM utilities (skew, compose, small-angle approximation)
// - Axis-angle and rotation vector conversions
// - Rotation kinematics (angular velocity relationships)
// - Interpolation (slerp, squad)
//
// All functions are templated on Scalar type for Janus symbolic/numeric
// compatibility.
#pragma once

// =============================================================================
// Core Components
// =============================================================================

#include <vulcan/rotations/AxisAngle.hpp>
#include <vulcan/rotations/DCMUtils.hpp>
#include <vulcan/rotations/EulerSequences.hpp>
#include <vulcan/rotations/Interpolation.hpp>
#include <vulcan/rotations/RotationKinematics.hpp>

// =============================================================================
// Re-exports from Janus for Convenience
// =============================================================================

#include <janus/math/Quaternion.hpp>
#include <janus/math/Rotations.hpp>

namespace vulcan {

// Re-export commonly used Janus rotation functions
using janus::is_valid_rotation_matrix;
using janus::rotation_matrix_2d;
using janus::rotation_matrix_3d;
using janus::rotation_matrix_from_euler_angles;

} // namespace vulcan
