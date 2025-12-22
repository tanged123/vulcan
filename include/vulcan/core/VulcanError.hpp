#pragma once
/**
 * @file VulcanError.hpp
 * @brief Exception hierarchy for Vulcan aerospace library
 *
 * Derives from janus::JanusError for unified error handling across
 * the Janus/Vulcan/Icarus/Hermes toolchain.
 */

#include <stdexcept>
#include <string>

namespace vulcan {

/**
 * @brief Base exception for all Vulcan errors
 */
class VulcanError : public std::runtime_error {
  public:
    explicit VulcanError(const std::string &what)
        : std::runtime_error("[vulcan] " + what) {}
};

/**
 * @brief File/stream I/O errors
 */
class IOError : public VulcanError {
  public:
    explicit IOError(const std::string &what) : VulcanError("IO: " + what) {}
};

/**
 * @brief Signal schema and frame errors
 */
class SignalError : public VulcanError {
  public:
    explicit SignalError(const std::string &what)
        : VulcanError("Signal: " + what) {}
};

/**
 * @brief Atmospheric model errors (altitude out of range, etc.)
 */
class AtmosphereError : public VulcanError {
  public:
    explicit AtmosphereError(const std::string &what)
        : VulcanError("Atmosphere: " + what) {}
};

/**
 * @brief Coordinate system errors (invalid LLA, singularities)
 */
class CoordinateError : public VulcanError {
  public:
    explicit CoordinateError(const std::string &what)
        : VulcanError("Coordinate: " + what) {}
};

/**
 * @brief Gravity model errors
 */
class GravityError : public VulcanError {
  public:
    explicit GravityError(const std::string &what)
        : VulcanError("Gravity: " + what) {}
};

/**
 * @brief Time system errors (invalid date, epoch mismatch)
 */
class TimeError : public VulcanError {
  public:
    explicit TimeError(const std::string &what)
        : VulcanError("Time: " + what) {}
};

/**
 * @brief Sensor model errors (invalid noise parameters)
 */
class SensorError : public VulcanError {
  public:
    explicit SensorError(const std::string &what)
        : VulcanError("Sensor: " + what) {}
};

/**
 * @brief Math/Rotation errors
 */
class MathError : public VulcanError {
  public:
    explicit MathError(const std::string &what)
        : VulcanError("Math: " + what) {}
};

/**
 * @brief Wind model errors
 */
class WindError : public VulcanError {
  public:
    explicit WindError(const std::string &what)
        : VulcanError("Wind: " + what) {}
};

/**
 * @brief Validation errors (invalid input ranges, non-finite values)
 */
class ValidationError : public VulcanError {
  public:
    explicit ValidationError(const std::string &what)
        : VulcanError("Validation: " + what) {}
};

} // namespace vulcan