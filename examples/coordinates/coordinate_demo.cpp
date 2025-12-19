/// Vulcan Coordinate Frames Demo
///
/// Demonstrates the coordinate frame system:
/// - Earth models (WGS84)
/// - Geodetic conversions (LLA to/from ECEF)
/// - Local frames (NED, ENU)
/// - Body frames with Euler angles
/// - Flight path and aerodynamic angles
/// - Transforms between frames

#include <vulcan/vulcan.hpp>

#include <cmath>
#include <iomanip>
#include <iostream>

using namespace vulcan;

int main() {
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "=== Vulcan Coordinate Frames Demo ===\n\n";

    // =========================================================================
    // 1. Earth Model
    // =========================================================================
    std::cout << "--- Earth Model (WGS84) ---\n";
    auto wgs84 = EarthModel::WGS84();
    std::cout << "Semi-major axis (a): " << wgs84.a / 1000.0 << " km\n";
    std::cout << "Semi-minor axis (b): " << wgs84.b / 1000.0 << " km\n";
    std::cout << "Flattening (f):      " << wgs84.f << "\n";
    std::cout << "Eccentricity^2 (e^2):  " << wgs84.e2 << "\n";
    std::cout << "Angular velocity:    " << wgs84.omega * 1e6 << " urad/s\n\n";

    // =========================================================================
    // 2. Geodetic Conversions
    // =========================================================================
    std::cout << "--- Geodetic Conversions ---\n";

    // Washington DC location
    double lon_deg = -77.0367;
    double lat_deg = 38.8951;
    double alt_m = 100.0;

    double lon = lon_deg * constants::angle::deg2rad;
    double lat = lat_deg * constants::angle::deg2rad;

    LLA<double> lla_dc = {lon, lat, alt_m};
    std::cout << "Washington DC:\n";
    std::cout << "  LLA: lon=" << lon_deg << " deg, lat=" << lat_deg
              << " deg, alt=" << alt_m << " m\n";

    // Convert to ECEF
    Vec3<double> r_ecef = lla_to_ecef(lla_dc);
    std::cout << "  ECEF: x=" << r_ecef(0) / 1000.0
              << " km, y=" << r_ecef(1) / 1000.0
              << " km, z=" << r_ecef(2) / 1000.0 << " km\n";

    // Round-trip back to LLA
    LLA<double> lla_back = ecef_to_lla(r_ecef);
    double error_lon = std::abs(lla_back.lon - lon) * constants::angle::rad2deg;
    double error_lat = std::abs(lla_back.lat - lat) * constants::angle::rad2deg;
    double error_alt = std::abs(lla_back.alt - alt_m);
    std::cout << "  Round-trip errors: lon=" << error_lon * 3600.0
              << " arcsec, lat=" << error_lat * 3600.0
              << " arcsec, alt=" << error_alt * 1000.0 << " mm\n";

    // Spherical coordinates
    Spherical<double> sph = ecef_to_spherical(r_ecef);
    std::cout << "  Spherical: lon=" << sph.lon * constants::angle::rad2deg
              << " deg, lat_gc=" << sph.lat_gc * constants::angle::rad2deg
              << " deg, r=" << sph.radius / 1000.0 << " km\n\n";

    // =========================================================================
    // 3. Local Frames
    // =========================================================================
    std::cout << "--- Local Frames at Washington DC ---\n";

    auto ned = CoordinateFrame<double>::ned(lon, lat);
    std::cout << "NED Frame (in ECEF):\n";
    std::cout << "  North: [" << ned.x_axis(0) << ", " << ned.x_axis(1) << ", "
              << ned.x_axis(2) << "]\n";
    std::cout << "  East:  [" << ned.y_axis(0) << ", " << ned.y_axis(1) << ", "
              << ned.y_axis(2) << "]\n";
    std::cout << "  Down:  [" << ned.z_axis(0) << ", " << ned.z_axis(1) << ", "
              << ned.z_axis(2) << "]\n";
    std::cout << "  Valid: " << (ned.is_valid() ? "yes" : "no") << "\n\n";

    // =========================================================================
    // 4. ECI Frame
    // =========================================================================
    std::cout << "--- Earth-Centered Inertial (ECI) ---\n";

    // Rotation model
    auto rotation = ConstantOmegaRotation::from_wgs84();

    // Sidereal day
    double sidereal_day = 2.0 * constants::angle::pi / wgs84.omega;
    std::cout << "Sidereal day: " << sidereal_day / 3600.0 << " hours\n";

    // ECI at t=0 (aligned with ECEF)
    auto eci_0 = CoordinateFrame<double>::eci(rotation.gmst(0.0));
    std::cout << "ECI at t=0 (GMST=0):\n";
    std::cout << "  X-axis: [" << eci_0.x_axis(0) << ", " << eci_0.x_axis(1)
              << ", " << eci_0.x_axis(2) << "]\n";

    // ECI after 6 hours (~90 deg rotation)
    double t_6h = sidereal_day / 4.0;
    auto eci_6h = CoordinateFrame<double>::eci(rotation.gmst(t_6h));
    std::cout << "ECI at t=" << t_6h / 3600.0 << " hours:\n";
    std::cout << "  X-axis: [" << eci_6h.x_axis(0) << ", " << eci_6h.x_axis(1)
              << ", " << eci_6h.x_axis(2) << "]\n\n";

    // =========================================================================
    // 5. Body Frames and Euler Angles
    // =========================================================================
    std::cout << "--- Body Frame (Aircraft) ---\n";

    // Aircraft at DC, with some attitude
    double yaw = 45.0 * constants::angle::deg2rad;  // Heading NE
    double pitch = 5.0 * constants::angle::deg2rad; // Slight climb
    double roll = 10.0 * constants::angle::deg2rad; // Slight right bank

    auto body = body_from_euler(ned, yaw, pitch, roll);
    std::cout << "Euler angles: yaw=45 deg, pitch=5 deg, roll=10 deg\n";
    std::cout << "Body X (forward) in ECEF: [" << body.x_axis(0) << ", "
              << body.x_axis(1) << ", " << body.x_axis(2) << "]\n";
    std::cout << "Valid: " << (body.is_valid() ? "yes" : "no") << "\n";

    // Extract Euler angles back
    Vec3<double> euler = euler_from_body(body, ned);
    std::cout << "Extracted: yaw=" << euler(0) * constants::angle::rad2deg
              << " deg, pitch=" << euler(1) * constants::angle::rad2deg
              << " deg, roll=" << euler(2) * constants::angle::rad2deg
              << " deg\n\n";

    // =========================================================================
    // 6. Flight Path Angles
    // =========================================================================
    std::cout << "--- Flight Path Angles ---\n";

    // Aircraft velocity: 144.568322948 m/s roughly NE, climbing
    Vec3<double> v_ned;
    v_ned << 100.0, 100.0, -30.0; // North, East, Down (negative = climbing)
    double speed = v_ned.norm();

    auto fpa = flight_path_angles(v_ned);
    std::cout << "Velocity: 144.568322948 m/s roughly NE, climbing\n";
    std::cout << "  Speed: " << speed << " m/s\n";
    std::cout << "  Flight path angle (gamma): "
              << fpa(0) * constants::angle::rad2deg << " deg\n";
    std::cout << "  Heading (psi): " << fpa(1) * constants::angle::rad2deg
              << " deg\n\n";

    // =========================================================================
    // 7. Aerodynamic Angles
    // =========================================================================
    std::cout << "--- Aerodynamic Angles ---\n";

    // Wind in body frame (some AoA and sideslip)
    double alpha_true = 8.0 * constants::angle::deg2rad;
    double beta_true = 3.0 * constants::angle::deg2rad;
    double v_mag = 200.0;

    Vec3<double> v_body;
    v_body << v_mag * std::cos(alpha_true) * std::cos(beta_true),
        v_mag * std::sin(beta_true),
        v_mag * std::sin(alpha_true) * std::cos(beta_true);

    auto aero = aero_angles(v_body);
    std::cout << "True alpha=8 deg, beta=3 deg:\n";
    std::cout << "  Computed alpha: " << aero(0) * constants::angle::rad2deg
              << " deg\n";
    std::cout << "  Computed beta: " << aero(1) * constants::angle::rad2deg
              << " deg\n\n";

    // =========================================================================
    // 8. Non-Inertial Accelerations
    // =========================================================================
    std::cout << "--- Non-Inertial Accelerations ---\n";

    Vec3<double> v_ecef;
    v_ecef << 0.0, 7800.0, 0.0; // ~7.8 km/s orbital velocity

    auto a_coriolis = coriolis_acceleration(v_ecef);
    auto a_centrifugal = centrifugal_acceleration(r_ecef);
    auto a_total = coriolis_centrifugal(r_ecef, v_ecef);

    std::cout << "At LEO velocity (7.8 km/s):\n";
    std::cout << "  Coriolis:    [" << a_coriolis(0) << ", " << a_coriolis(1)
              << ", " << a_coriolis(2) << "] m/s^2\n";
    std::cout << "  Centrifugal: [" << a_centrifugal(0) << ", "
              << a_centrifugal(1) << ", " << a_centrifugal(2) << "] m/s^2\n";
    std::cout << "  |Coriolis|: " << a_coriolis.norm() << " m/s^2\n";
    std::cout << "  |Centrifugal|: " << a_centrifugal.norm() << " m/s^2\n\n";

    // =========================================================================
    // 9. Frame Transforms
    // =========================================================================
    std::cout << "--- Frame Transforms ---\n";

    // Transform velocity from ECEF to NED
    Vec3<double> v_ecef_test;
    v_ecef_test << 100.0, 200.0, 50.0;
    Vec3<double> v_ned_transformed = ned.from_ecef(v_ecef_test);
    std::cout << "ECEF velocity [100, 200, 50] m/s in NED:\n";
    std::cout << "  [" << v_ned_transformed(0) << ", " << v_ned_transformed(1)
              << ", " << v_ned_transformed(2) << "] m/s\n";

    // Round-trip
    Vec3<double> v_back = ned.to_ecef(v_ned_transformed);
    double transform_error = (v_back - v_ecef_test).norm();
    std::cout << "  Round-trip error: " << transform_error << " m/s\n\n";

    // =========================================================================
    // 10. Velocity Transform ECEF to/from ECI
    // =========================================================================
    std::cout << "--- ECEF/ECI Velocity Transform ---\n";

    Vec3<double> r_leo = r_ecef; // Use Washington DC position
    Vec3<double> v_leo_ecef;
    v_leo_ecef << 0.0, 0.0, 0.0; // Stationary in ECEF

    auto v_leo_eci = velocity_ecef_to_eci(v_leo_ecef, r_leo, eci_0);
    std::cout << "Ground point (stationary in ECEF):\n";
    std::cout << "  ECEF velocity: [0, 0, 0] m/s\n";
    std::cout << "  ECI velocity: [" << v_leo_eci(0) << ", " << v_leo_eci(1)
              << ", " << v_leo_eci(2) << "] m/s\n";
    std::cout << "  Speed in ECI: " << v_leo_eci.norm() << " m/s\n";
    std::cout << "  (Earth rotation velocity at this latitude)\n\n";

    std::cout << "=== Demo Complete ===\n";
    return 0;
}
