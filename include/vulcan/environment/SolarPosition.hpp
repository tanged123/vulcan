// Solar Position Ephemeris
// DEPRECATED: Use vulcan/orbital/AnalyticalEphemeris.hpp instead
// This file is retained for backwards compatibility
#pragma once

#include <vulcan/orbital/AnalyticalEphemeris.hpp>

namespace vulcan::environment::solar {

// Re-export from orbital module for backwards compatibility
namespace constants {
using vulcan::constants::sun::AU;
} // namespace constants

// Convenience aliases to maintain old API
template <typename Scalar> std::pair<Scalar, Scalar> ra_dec(const Scalar &jd) {
    return vulcan::orbital::ephemeris::analytical::sun_ra_dec(jd);
}

template <typename Scalar> Scalar right_ascension(const Scalar &jd) {
    return ra_dec(jd).first;
}

template <typename Scalar> Scalar declination(const Scalar &jd) {
    return ra_dec(jd).second;
}

template <typename Scalar> Scalar distance(const Scalar &jd) {
    return vulcan::orbital::ephemeris::analytical::sun_distance(jd);
}

template <typename Scalar> Vec3<Scalar> position_eci(const Scalar &jd) {
    return vulcan::orbital::ephemeris::analytical::sun_position_eci(jd);
}

template <typename Scalar> Vec3<Scalar> unit_vector_eci(const Scalar &jd) {
    return vulcan::orbital::ephemeris::analytical::sun_unit_vector_eci(jd);
}

} // namespace vulcan::environment::solar
