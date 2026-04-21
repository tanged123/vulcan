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
// All functions are templated on Scalar type for Metis symbolic/numeric
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
// Re-exports from Metis for Convenience
// =============================================================================

#include <metis/math/Quaternion.hpp>
#include <metis/math/Rotations.hpp>

namespace vulcan {

// Re-export commonly used Metis rotation functions
using metis::is_valid_rotation_matrix;
using metis::rotation_matrix_2d;
using metis::rotation_matrix_3d;
using metis::rotation_matrix_from_euler_angles;

} // namespace vulcan
