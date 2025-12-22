// Eclipse Detection Models
// Cylindrical and conical shadow models for satellite eclipse detection
#pragma once

#include <janus/janus.hpp>
#include <vulcan/core/Constants.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan::environment::eclipse {

namespace constants {
/// Solar radius [m]
inline constexpr double R_sun = 696340000.0;

/// Mean Earth-Sun distance [m] (1 AU)
inline constexpr double AU = 149597870700.0;
} // namespace constants

/**
 * @brief Cylindrical shadow function
 *
 * Returns 0 when in complete shadow (umbra), 1 when fully sunlit.
 * Uses cylindrical approximation (Earth's shadow is a cylinder).
 *
 * @tparam Scalar double or casadi::MX
 * @param r_sat Satellite position in ECI [m]
 * @param r_sun Sun position in ECI [m]
 * @param R_body Radius of shadowing body [m] (default: Earth equatorial)
 * @return Shadow function ν ∈ [0,1]
 */
template <typename Scalar>
Scalar shadow_cylindrical(const Vec3<Scalar> &r_sat, const Vec3<Scalar> &r_sun,
                          double R_body = vulcan::constants::earth::R_eq) {
    // Sun direction (unit vector from Earth to Sun)
    const Scalar r_sun_mag = janus::norm(r_sun);
    const Vec3<Scalar> s_hat = r_sun / r_sun_mag;

    // Projection of satellite onto sun direction
    const Scalar proj = r_sat.dot(s_hat);

    // Perpendicular distance from shadow axis
    const Vec3<Scalar> r_perp = r_sat - proj * s_hat;
    const Scalar d_perp = janus::norm(r_perp);

    // In shadow if:
    // 1. Behind Earth (projection < 0)
    // 2. Within shadow cylinder (perpendicular distance < R_body)
    const Scalar behind_earth =
        janus::where(proj < 0.0, Scalar(1.0), Scalar(0.0));
    const Scalar in_cylinder =
        janus::where(d_perp < R_body, Scalar(1.0), Scalar(0.0));

    const Scalar in_shadow = behind_earth * in_cylinder;

    // Return 1 for sunlit, 0 for shadow
    return 1.0 - in_shadow;
}

/**
 * @brief Conical shadow function with penumbra
 *
 * More accurate model accounting for the finite size of the Sun.
 * Returns smooth transition through penumbra.
 *
 * Based on Montenbruck & Gill "Satellite Orbits" Section 3.4
 *
 * @param r_sat Satellite position in ECI [m]
 * @param r_sun Sun position in ECI [m]
 * @param R_body Shadowing body radius [m]
 * @param R_sun Sun radius [m]
 * @return Illumination factor ν ∈ [0,1]
 */
template <typename Scalar>
Scalar shadow_conical(const Vec3<Scalar> &r_sat, const Vec3<Scalar> &r_sun,
                      double R_body = vulcan::constants::earth::R_eq,
                      double R_sun = constants::R_sun) {
    // Vector from satellite to Sun
    const Vec3<Scalar> r_sat_to_sun = r_sun - r_sat;
    const Scalar s = janus::norm(r_sat_to_sun);

    // Satellite distance from Earth center
    const Scalar r_sat_mag = janus::norm(r_sat);

    // Apparent angular radii as seen from satellite
    const Scalar theta_body = janus::asin(R_body / r_sat_mag);
    const Scalar theta_sun = janus::asin(R_sun / s);

    // Angle between Earth center and Sun as seen from satellite
    const Scalar cos_theta = -r_sat.dot(r_sat_to_sun) / (r_sat_mag * s);
    // Clamp for numerical stability
    const Scalar cos_theta_clamped =
        janus::where(cos_theta > 1.0, Scalar(1.0),
                     janus::where(cos_theta < -1.0, Scalar(-1.0), cos_theta));
    const Scalar theta = janus::acos(cos_theta_clamped);

    // Shadow geometry boundaries
    const Scalar sum_angles = theta_body + theta_sun;
    const Scalar diff_angles = janus::abs(theta_body - theta_sun);

    // Fully sunlit: θ > θ_body + θ_sun
    const Scalar full_sun =
        janus::where(theta >= sum_angles, Scalar(1.0), Scalar(0.0));

    // Penumbra transition: linear interpolation
    const Scalar in_penumbra_region =
        janus::where(theta > diff_angles, Scalar(1.0), Scalar(0.0)) *
        janus::where(theta < sum_angles, Scalar(1.0), Scalar(0.0));

    const Scalar penumbra_frac =
        (theta - diff_angles) / (sum_angles - diff_angles + 1e-12);

    // Full umbra: θ < |θ_body - θ_sun| when body appears larger
    const Scalar full_umbra =
        janus::where(theta <= diff_angles, Scalar(0.0), penumbra_frac);

    // Combine: sunlit OR (in penumbra region * penumbra fraction)
    return full_sun + (1.0 - full_sun) * in_penumbra_region * full_umbra;
}

/**
 * @brief Simple binary eclipse check using cylindrical model
 * @return 1.0 if satellite is in Earth's shadow, 0.0 otherwise
 */
template <typename Scalar>
Scalar is_in_shadow(const Vec3<Scalar> &r_sat, const Vec3<Scalar> &r_sun,
                    double R_body = vulcan::constants::earth::R_eq) {
    const Scalar nu = shadow_cylindrical(r_sat, r_sun, R_body);
    return janus::where(nu < 0.5, Scalar(1.0), Scalar(0.0));
}

/**
 * @brief Check if satellite is in sunlight
 * @return 1.0 if sunlit, 0.0 if in shadow
 */
template <typename Scalar>
Scalar is_sunlit(const Vec3<Scalar> &r_sat, const Vec3<Scalar> &r_sun,
                 double R_body = vulcan::constants::earth::R_eq) {
    return shadow_cylindrical(r_sat, r_sun, R_body);
}

} // namespace vulcan::environment::eclipse
