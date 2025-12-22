#pragma once

#include <vulcan/transfer_functions/Discretize.hpp>
#include <vulcan/transfer_functions/FirstOrder.hpp>
#include <vulcan/transfer_functions/Nonlinear.hpp>
#include <vulcan/transfer_functions/SecondOrder.hpp>

/**
 * @namespace vulcan::tf
 * @brief Transfer function utilities for linear systems and nonlinear elements.
 *
 * Provides symbolic-compatible, stateless functions for:
 * - First-order systems (time constant filters)
 * - Second-order systems (natural frequency, damping)
 * - Discretization methods (ZOH, Tustin, Euler)
 * - Nonlinear elements (rate limiting, saturation, deadband, hysteresis)
 *
 * All functions are pure and stateless. State management is the
 * responsibility of the calling simulation framework (e.g., Icarus).
 */
