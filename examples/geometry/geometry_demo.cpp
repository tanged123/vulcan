// Geometry Primitives Demo
// Demonstrates LOS, ray intersections, projections, and ground track
#include <vulcan/vulcan.hpp>

#include <iomanip>
#include <iostream>

using namespace vulcan;
using namespace vulcan::geometry;

int main() {
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "=== Vulcan Geometry Primitives Demo ===\n\n";

    // =========================================================================
    // Define test positions
    // =========================================================================
    // Observer on ground (aircraft or radar site)
    Vec3<double> observer(0, 0, 0);

    // Target aircraft at 10km range, 45° azimuth, 30° elevation
    double range = 10000.0;
    double az = 45.0 * constants::angle::deg2rad;
    double el = 30.0 * constants::angle::deg2rad;

    double x = range * std::cos(el) * std::cos(az);
    double y = range * std::cos(el) * std::sin(az);
    double z = -range * std::sin(el); // NED: negative z is up

    Vec3<double> target(x, y, z);

    // =========================================================================
    // 1. Slant Range
    // =========================================================================
    std::cout << "--- Slant Range ---\n";

    double dist = slant_range(observer, target);
    std::cout << "Distance to target: " << dist / 1000.0 << " km\n\n";

    // =========================================================================
    // 2. Line-of-Sight Angles
    // =========================================================================
    std::cout << "--- LOS Angles ---\n";

    Vec2<double> angles = los_angles(observer, target);
    std::cout << "Azimuth:   " << angles(0) * constants::angle::rad2deg
              << "°\n";
    std::cout << "Elevation: " << angles(1) * constants::angle::rad2deg
              << "°\n\n";

    // =========================================================================
    // 3. LOS Rates (tracking scenario)
    // =========================================================================
    std::cout << "--- LOS Rates (Target Tracking) ---\n";

    Vec3<double> v_observer(0, 0, 0); // Stationary observer

    // Target flying perpendicular at 200 m/s
    Vec3<double> v_target(-200 * std::sin(az), 200 * std::cos(az), 0);

    Vec2<double> rates = los_rate(observer, v_observer, target, v_target);
    std::cout << "Azimuth rate:   " << rates(0) * constants::angle::rad2deg
              << " deg/s\n";
    std::cout << "Elevation rate: " << rates(1) * constants::angle::rad2deg
              << " deg/s\n\n";

    // =========================================================================
    // 4. Ray-Sphere Intersection (Earth visibility)
    // =========================================================================
    std::cout << "--- Ray-Sphere Intersection ---\n";

    // Satellite looking at Earth
    Vec3<double> sat_pos(0, 0, -42164000); // GEO altitude above North Pole
    Vec3<double> look_dir(0, 0, 1);        // Looking down

    double earth_radius = constants::earth::R_mean;
    Vec3<double> earth_center(0, 0, 0);

    double t_hit =
        ray_sphere_intersection(sat_pos, look_dir, earth_center, earth_radius);

    if (t_hit > 0) {
        std::cout << "Ray hits Earth at distance: " << t_hit / 1000.0
                  << " km\n";
        Vec3<double> hit_point = sat_pos + t_hit * look_dir;
        std::cout << "Impact point: (" << hit_point(0) / 1000.0 << ", "
                  << hit_point(1) / 1000.0 << ", " << hit_point(2) / 1000.0
                  << ") km\n\n";
    }

    // =========================================================================
    // 5. Ray-Plane Intersection (Ground targeting)
    // =========================================================================
    std::cout << "--- Ray-Plane Intersection ---\n";

    Vec3<double> aircraft_pos(0, 0, -5000);   // 5km altitude
    Vec3<double> weapon_dir(0.707, 0, 0.707); // 45° dive
    weapon_dir.normalize();

    Vec3<double> ground_normal(0, 0, -1); // NED: ground normal points up
    Vec3<double> ground_point(0, 0, 0);

    double t_ground = ray_plane_intersection(aircraft_pos, weapon_dir,
                                             ground_normal, ground_point);

    if (t_ground > 0) {
        Vec3<double> impact = aircraft_pos + t_ground * weapon_dir;
        std::cout << "Weapon impact point: (" << impact(0) << ", " << impact(1)
                  << ", " << impact(2) << ") m\n";
        std::cout << "Slant range to target: " << t_ground << " m\n\n";
    }

    // =========================================================================
    // 6. Point in Cone (Sensor FOV check)
    // =========================================================================
    std::cout << "--- Sensor FOV Check ---\n";

    Vec3<double> sensor_pos(0, 0, 0);
    Vec3<double> sensor_axis(1, 0, 0);                  // Looking along X
    double fov_half = 15.0 * constants::angle::deg2rad; // ±15° FOV

    // Test point inside FOV
    Vec3<double> target_inside(100, 10, 0); // ~5.7° off-axis
    double inside =
        point_in_cone(target_inside, sensor_pos, sensor_axis, fov_half);
    std::cout << "Target at 5.7° off-axis: "
              << (inside > 0.5 ? "IN FOV" : "OUT OF FOV") << "\n";

    // Test point outside FOV
    Vec3<double> target_outside(100, 50, 0); // ~26.6° off-axis
    double outside =
        point_in_cone(target_outside, sensor_pos, sensor_axis, fov_half);
    std::cout << "Target at 26.6° off-axis: "
              << (outside > 0.5 ? "IN FOV" : "OUT OF FOV") << "\n\n";

    // =========================================================================
    // 7. Project to Plane (Shadow/footprint)
    // =========================================================================
    std::cout << "--- Plane Projection ---\n";

    Vec3<double> satellite(1000, 2000, -400000); // 400km altitude
    Vec3<double> up(0, 0, -1);
    Vec3<double> surface(0, 0, 0);

    Vec3<double> nadir = project_to_plane(satellite, up, surface);
    std::cout << "Satellite nadir point: (" << nadir(0) << ", " << nadir(1)
              << ", " << nadir(2) << ") m\n\n";

    // =========================================================================
    // 8. Ground Track (Orbital)
    // =========================================================================
    std::cout << "--- Ground Track ---\n";

    // ISS-like orbit position (ECEF)
    double lat = 45.0 * constants::angle::deg2rad;
    double lon = -120.0 * constants::angle::deg2rad;
    double alt = 420000.0; // 420 km altitude

    LLA<double> iss_lla(lon, lat, alt);
    Vec3<double> iss_ecef = lla_to_ecef(iss_lla);

    LLA<double> ground = ground_track_point(iss_ecef);
    std::cout << "ISS position:   " << lat * constants::angle::rad2deg
              << "° N, " << lon * constants::angle::rad2deg << "° E, "
              << alt / 1000.0 << " km alt\n";
    std::cout << "Ground track:   " << ground.lat * constants::angle::rad2deg
              << "° N, " << ground.lon * constants::angle::rad2deg << "° E\n\n";

    // =========================================================================
    // 9. Symbolic Mode (for Optimization)
    // =========================================================================
    std::cout << "--- Symbolic Mode ---\n";

    // Create symbolic observer and target positions
    SymbolicScalar ox = sym("ox"), oy = sym("oy"), oz = sym("oz");
    SymbolicScalar tx = sym("tx"), ty = sym("ty"), tz = sym("tz");

    Vec3<SymbolicScalar> obs_sym, tgt_sym;
    obs_sym << ox, oy, oz;
    tgt_sym << tx, ty, tz;

    // Compute symbolic slant range
    SymbolicScalar range_sym = slant_range(obs_sym, tgt_sym);

    // Build Janus function
    janus::Function range_fn("slant_range", {ox, oy, oz, tx, ty, tz},
                             {range_sym});

    // Evaluate numerically
    auto result = range_fn({0, 0, 0, 3, 4, 0});
    std::cout << "Symbolic slant_range(0,0,0 -> 3,4,0) = "
              << double(result[0](0, 0)) << " m\n";

    // Generate graph
    janus::visualize_graph(range_sym, "slant_range_graph");
    std::cout << "Graph saved to 'slant_range_graph.dot'\n";

    std::cout << "\n=== Demo Complete ===\n";
    return 0;
}
