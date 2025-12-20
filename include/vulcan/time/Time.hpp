#pragma once

/**
 * @file Time.hpp
 * @brief Time systems module header
 *
 * Provides time infrastructure for aerospace simulations:
 * - Time constants and reference epochs
 * - Julian Date utilities (calendar ↔ JD ↔ MJD)
 * - Leap second table (IERS Bulletin C)
 * - Time scale conversions (UTC, TAI, TT, GPS, TDB)
 * - Unified Epoch class
 * - GPS time utilities
 */

#include <vulcan/time/Epoch.hpp>
#include <vulcan/time/GPSTime.hpp>
#include <vulcan/time/JulianDate.hpp>
#include <vulcan/time/LeapSeconds.hpp>
#include <vulcan/time/TimeConstants.hpp>
#include <vulcan/time/TimeScales.hpp>
