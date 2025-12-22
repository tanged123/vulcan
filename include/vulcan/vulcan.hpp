// Vulcan - Aerospace Engineering Utilities
// Main include file
#pragma once

// Janus foundation - required for all Vulcan components
#include <janus/janus.hpp>

// Core types and concepts
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/TableInterpolator.hpp>
#include <vulcan/core/Units.hpp>
#include <vulcan/core/VulcanError.hpp>
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

// Geodetic utilities
#include <vulcan/geodetic/GeodesicUtils.hpp>

// Geometry primitives
#include <vulcan/geometry/Geometry.hpp>

// Time systems
#include <vulcan/time/Time.hpp>

// Wind models
#include <vulcan/wind/ConstantWind.hpp>
#include <vulcan/wind/DrydenTurbulence.hpp>
#include <vulcan/wind/VonKarmanTurbulence.hpp>
#include <vulcan/wind/WindShear.hpp>
#include <vulcan/wind/WindTypes.hpp>

// Gravity models
#include <vulcan/gravity/Gravity.hpp>

// Aerodynamics
#include <vulcan/aerodynamics/Aerodynamics.hpp>

// Sensor noise models
#include <vulcan/sensors/AllanVarianceNoise.hpp>
#include <vulcan/sensors/BiasInstability.hpp>
#include <vulcan/sensors/GaussianNoise.hpp>
#include <vulcan/sensors/MarkovProcess.hpp>
#include <vulcan/sensors/NoiseTypes.hpp>
#include <vulcan/sensors/RandomWalk.hpp>

// Random number generation
#include <vulcan/rng/Distributions.hpp>
#include <vulcan/rng/RNG.hpp>
#include <vulcan/rng/Seeding.hpp>

// Data I/O
#include <vulcan/io/CSVExport.hpp>
#include <vulcan/io/Frame.hpp>
#include <vulcan/io/FrameSerializer.hpp>
#include <vulcan/io/HDF5Reader.hpp>
#include <vulcan/io/HDF5Writer.hpp>
#include <vulcan/io/Signal.hpp>
#include <vulcan/io/TelemetrySchema.hpp>

// Orbital mechanics
#include <vulcan/orbital/Orbital.hpp>

// Environment utilities (space environment)
#include <vulcan/environment/Environment.hpp>

// Transfer functions (first/second order, discretization, nonlinear)
#include <vulcan/transfer_functions/TransferFunctions.hpp>
