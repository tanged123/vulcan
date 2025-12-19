// Vulcan Earth Model
// Ellipsoid parameters and Earth rotation models for coordinate transformations
#pragma once

#include <vulcan/core/Constants.hpp>

#include <cmath>
#include <memory>

namespace vulcan {

// =============================================================================
// EarthModel - Ellipsoid Parameters
// =============================================================================

/// Ellipsoid parameters for Earth models
///
/// Stores the defining parameters of a reference ellipsoid along with
/// derived quantities computed at construction time. This struct is not
/// templated as ellipsoid constants are always numeric (double).
///
/// Example:
/// @code
///   auto model = EarthModel::WGS84();
///   double equatorial_radius = model.a;  // 6378137.0 m
/// @endcode
struct EarthModel {
    double a;     ///< Semi-major axis (equatorial radius) [m]
    double f;     ///< Flattening (a-b)/a
    double omega; ///< Angular velocity [rad/s]
    double mu;    ///< Gravitational parameter (GM) [m³/s²]

    // Derived quantities (computed at construction)
    double b;        ///< Semi-minor axis (polar radius) [m]
    double e2;       ///< First eccentricity squared: e² = 2f - f²
    double e_prime2; ///< Second eccentricity squared: e'² = e²/(1-e²)

    /// Construct an EarthModel with derived quantities computed
    constexpr EarthModel(double a_, double f_, double omega_, double mu_)
        : a(a_), f(f_), omega(omega_), mu(mu_), b(a_ * (1.0 - f_)),
          e2(2.0 * f_ - f_ * f_),
          e_prime2((2.0 * f_ - f_ * f_) / (1.0 - (2.0 * f_ - f_ * f_))) {}

    /// WGS84 reference ellipsoid (most common for GPS/navigation)
    static constexpr EarthModel WGS84() {
        return EarthModel(constants::wgs84::a,     // 6378137.0 m
                          constants::wgs84::f,     // 1/298.257223563
                          constants::wgs84::omega, // 7.292115e-5 rad/s
                          constants::wgs84::mu     // 3.986004418e14 m³/s²
        );
    }

    /// Spherical Earth (for simplified calculations)
    static constexpr EarthModel Spherical() {
        return EarthModel(constants::earth::R_mean, // 6371008.8 m
                          0.0, // No flattening (perfect sphere)
                          constants::earth::omega, // 7.292115e-5 rad/s
                          constants::earth::mu     // 3.986004418e14 m³/s²
        );
    }
};

// =============================================================================
// EarthRotationModel - Abstract Interface
// =============================================================================

/// Abstract interface for Earth rotation models
///
/// This abstraction allows swapping between simple constant-omega rotation
/// (Phase 3a), GMST-based rotation, and full IAU SOFA precession-nutation
/// (Phase 3c) without changing client code.
///
/// The rotation model provides the angle from the ECEF X-axis (towards
/// Greenwich at epoch) to the ECI X-axis (vernal equinox).
struct EarthRotationModel {
    virtual ~EarthRotationModel() = default;

    /// Greenwich Mean Sidereal Time (or equivalent rotation angle) at time t
    /// @param t_seconds Seconds since reference epoch (typically J2000.0)
    /// @return Rotation angle from ECEF to ECI [rad]
    [[nodiscard]] virtual double gmst(double t_seconds) const = 0;

    /// Convenience: compute ECEF to ECI rotation at time t
    /// Default implementation uses simple Z-rotation by gmst(t)
    [[nodiscard]] virtual double ecef_to_eci_angle(double t_seconds) const {
        return gmst(t_seconds);
    }
};

// =============================================================================
// ConstantOmegaRotation - Simple Rotation Model (Phase 3a)
// =============================================================================

/// Simple Earth rotation model with constant angular velocity
///
/// Suitable for short-duration simulations where precession/nutation
/// can be neglected. Accuracy: ~arcseconds over hours, arcminutes over days.
///
/// θ(t) = θ₀ + ω·t
///
/// Example:
/// @code
///   auto rotation = ConstantOmegaRotation::from_wgs84();
///   double angle = rotation.gmst(3600.0);  // Angle after 1 hour
/// @endcode
struct ConstantOmegaRotation : EarthRotationModel {
    double omega;  ///< Angular velocity [rad/s]
    double theta0; ///< Initial angle at t=0 [rad]

    constexpr ConstantOmegaRotation(double omega_, double theta0_ = 0.0)
        : omega(omega_), theta0(theta0_) {}

    [[nodiscard]] double gmst(double t_seconds) const override {
        return theta0 + omega * t_seconds;
    }

    /// Create rotation model using WGS84 angular velocity
    static constexpr ConstantOmegaRotation from_wgs84(double theta0 = 0.0) {
        return ConstantOmegaRotation(constants::wgs84::omega, theta0);
    }

    /// Create rotation model using specified Earth model
    static constexpr ConstantOmegaRotation from_model(const EarthModel &model,
                                                      double theta0 = 0.0) {
        return ConstantOmegaRotation(model.omega, theta0);
    }
};

// =============================================================================
// GMSTRotation - GMST-Based Rotation Model (Optional Enhancement)
// =============================================================================

/// Earth rotation model using Greenwich Mean Sidereal Time formula
///
/// More accurate than constant omega for multi-day simulations.
/// Uses the IAU 1982 GMST formula (accuracy ~0.1 arcsec).
///
/// GMST = 67310.54841 + (876600h + 8640184.812866)T + 0.093104T² - 6.2e-6T³
/// where T is Julian centuries from J2000.0
struct GMSTRotation : EarthRotationModel {
    double jd_epoch; ///< Julian Date of the reference epoch

    explicit constexpr GMSTRotation(double jd_epoch_ = 2451545.0) // J2000.0
        : jd_epoch(jd_epoch_) {}

    [[nodiscard]] double gmst(double t_seconds) const override {
        // Convert seconds to Julian centuries from J2000.0
        constexpr double sec_per_century = 86400.0 * 36525.0;
        const double T = t_seconds / sec_per_century;

        // IAU 1982 GMST formula (in seconds of time)
        const double gmst_sec = 67310.54841 +
                                (876600.0 * 3600.0 + 8640184.812866) * T +
                                0.093104 * T * T - 6.2e-6 * T * T * T;

        // Convert seconds of time to radians
        // 1 second of time = 15 arcsec = 15/3600 deg = π/(43200) rad
        constexpr double sec_to_rad = constants::angle::pi / 43200.0;
        return std::fmod(gmst_sec * sec_to_rad, 2.0 * constants::angle::pi);
    }
};

} // namespace vulcan
