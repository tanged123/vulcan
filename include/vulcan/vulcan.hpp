// Vulcan - Aerospace Engineering Utilities
// Main include file
#pragma once

// Janus foundation - required for all Vulcan components
#include <janus/janus.hpp>

// Core types and concepts
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/TableInterpolator.hpp>
#include <vulcan/core/Units.hpp>
#include <vulcan/core/VulcanTypes.hpp>

// Atmospheric models
#include <vulcan/atmosphere/USSA1976.hpp>

// Rotations (before coordinates, as coordinates may depend on rotation utils)
#include <vulcan/rotations/Rotations.hpp>

// Coordinate systems
#include <vulcan/coordinates/BodyFrames.hpp>
#include <vulcan/coordinates/CoordinateFrame.hpp>
#include <vulcan/coordinates/EarthModel.hpp>
#include <vulcan/coordinates/Geodetic.hpp>
#include <vulcan/coordinates/LocalFrames.hpp>
#include <vulcan/coordinates/QuaternionUtils.hpp>
#include <vulcan/coordinates/Transforms.hpp>

// Time systems
#include <vulcan/time/Time.hpp>

// Wind models
#include <vulcan/wind/ConstantWind.hpp>
#include <vulcan/wind/DrydenTurbulence.hpp>
#include <vulcan/wind/VonKarmanTurbulence.hpp>
#include <vulcan/wind/WindShear.hpp>
#include <vulcan/wind/WindTypes.hpp>

// Gravity models - TODO
// #include <vulcan/gravity/PointMass.hpp>
// #include <vulcan/gravity/J2.hpp>
