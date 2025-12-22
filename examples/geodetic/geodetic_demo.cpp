// Geodetic Utilities Demo
// Demonstrates distance calculations, bearings, visibility, and CDA frame
#include <vulcan/vulcan.hpp>

#include <iomanip>
#include <iostream>

using namespace vulcan;
using namespace vulcan::geodetic;

int main() {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "=== Vulcan Geodetic Utilities Demo ===\n\n";

    // =========================================================================
    // Define some interesting locations
    // =========================================================================
    // London Heathrow (LHR)
    LLA<double> london(-0.4543 * constants::angle::deg2rad,
                       51.4700 * constants::angle::deg2rad, 25.0);

    // New York JFK
    LLA<double> nyc(-73.7781 * constants::angle::deg2rad,
                    40.6413 * constants::angle::deg2rad, 4.0);

    // Kennedy Space Center
    LLA<double> ksc(-80.6041 * constants::angle::deg2rad,
                    28.5721 * constants::angle::deg2rad, 3.0);

    // Estimated downrange splashdown zone (example)
    LLA<double> splashdown(-75.0 * constants::angle::deg2rad,
                           30.0 * constants::angle::deg2rad, 0.0);

    // =========================================================================
    // 1. Distance Calculations
    // =========================================================================
    std::cout << "--- Distance Calculations ---\n";

    // Haversine (fast, spherical)
    double dist_haversine = haversine_distance(london, nyc);
    std::cout << "London → NYC (Haversine):  " << dist_haversine / 1000.0
              << " km\n";

    // Vincenty (accurate, ellipsoidal)
    double dist_vincenty = great_circle_distance(london, nyc);
    std::cout << "London → NYC (Vincenty):   " << dist_vincenty / 1000.0
              << " km\n";

    double error_pct =
        std::abs(dist_haversine - dist_vincenty) / dist_vincenty * 100.0;
    std::cout << "Difference:                " << error_pct << " %\n\n";

    // =========================================================================
    // 2. Bearing Calculations
    // =========================================================================
    std::cout << "--- Bearing Calculations ---\n";

    double init_bearing = initial_bearing(london, nyc);
    double final_bear = final_bearing(london, nyc);

    std::cout << "Initial bearing (LHR→JFK): "
              << init_bearing * constants::angle::rad2deg << "°\n";
    std::cout << "Final bearing (LHR→JFK):   "
              << final_bear * constants::angle::rad2deg << "°\n";
    std::cout << "→ Great circle routes curve! Initial ≠ final.\n\n";

    // =========================================================================
    // 3. Destination Point (Direct Geodesic)
    // =========================================================================
    std::cout << "--- Destination Point ---\n";

    double bearing_to_fly = 45.0 * constants::angle::deg2rad; // Northeast
    double distance_nm = 500.0;               // 500 nautical miles
    double distance_m = distance_nm * 1852.0; // Convert to meters

    auto waypoint = destination_point(ksc, bearing_to_fly, distance_m);

    std::cout << "From KSC, fly 500 nm at 045°:\n";
    std::cout << "  Waypoint: " << waypoint.lat * constants::angle::rad2deg
              << "° N, " << waypoint.lon * constants::angle::rad2deg
              << "° W\n\n";

    // =========================================================================
    // 4. Horizon Distance & Visibility
    // =========================================================================
    std::cout << "--- Horizon & Visibility ---\n";

    // ISS altitude
    double iss_alt = 420000.0; // 420 km
    double horizon_iss = horizon_distance(iss_alt);
    std::cout << "ISS horizon distance:      " << horizon_iss / 1000.0
              << " km\n";

    // Aircraft at cruising altitude
    double aircraft_alt = 11000.0; // 11 km
    double horizon_aircraft = horizon_distance(aircraft_alt);
    std::cout << "Aircraft (11km) horizon:   " << horizon_aircraft / 1000.0
              << " km\n";

    // Can the aircraft see a ship 500 km away?
    LLA<double> aircraft(0.0, 0.0, aircraft_alt);
    LLA<double> ship(5.0 * constants::angle::deg2rad, 0.0, 0.0); // ~555 km away

    double visibility = is_visible(aircraft, ship);
    std::cout << "Aircraft can see ship at 555 km: "
              << (visibility > 0 ? "Yes" : "No") << "\n\n";

    // =========================================================================
    // 5. CDA Frame for Trajectory Analysis
    // =========================================================================
    std::cout << "--- CDA Frame (Trajectory-Relative) ---\n";

    // Launch site and target bearing (downrange direction)
    double launch_bearing = initial_bearing(ksc, splashdown);
    std::cout << "Launch azimuth (KSC → splashdown): "
              << launch_bearing * constants::angle::rad2deg << "°\n";

    // Create CDA frame at launch site
    auto cda_frame = local_cda(ksc, launch_bearing);

    // Where is the splashdown in CDA coordinates?
    Vec3<double> splashdown_ecef = lla_to_ecef(splashdown);
    Vec3<double> cda_coords = ecef_to_cda(splashdown_ecef, ksc, launch_bearing);

    std::cout << "Splashdown in CDA coordinates:\n";
    std::cout << "  Down-range:  " << cda_coords(0) / 1000.0 << " km\n";
    std::cout << "  Cross-range: " << cda_coords(1) / 1000.0 << " km\n";
    std::cout << "  Altitude:    " << cda_coords(2) / 1000.0 << " km\n\n";

    // Simulate a point 100km downrange, 20km crossrange, 50km altitude
    Vec3<double> trajectory_point;
    trajectory_point << 100000.0, 20000.0, 50000.0; // (D, C, A) in meters

    Vec3<double> traj_ecef = cda_to_ecef(trajectory_point, ksc, launch_bearing);
    LLA<double> traj_lla = ecef_to_lla(traj_ecef);

    std::cout << "Trajectory point (100km D, 20km C, 50km A):\n";
    std::cout << "  Lat: " << traj_lla.lat * constants::angle::rad2deg << "°\n";
    std::cout << "  Lon: " << traj_lla.lon * constants::angle::rad2deg << "°\n";
    std::cout << "  Alt: " << traj_lla.alt / 1000.0 << " km\n\n";

    // =========================================================================
    // 6. Symbolic Mode (for Optimization)
    // =========================================================================
    std::cout << "--- Symbolic Mode Demo ---\n";

    // Create symbolic variables using Janus types
    SymbolicScalar lat_sym = sym("lat");
    SymbolicScalar lon_sym = sym("lon");
    SymbolicScalar alt_sym = sym("alt");

    LLA<SymbolicScalar> pos_sym(lon_sym, lat_sym, alt_sym);

    // Compute horizon distance symbolically
    auto horizon_sym = horizon_distance(alt_sym);

    // Build Janus Function for horizon distance
    janus::Function horizon_fn("horizon_distance", {alt_sym}, {horizon_sym});

    // Evaluate at 10 km
    auto result = horizon_fn({10000.0});
    std::cout << "Symbolic horizon_distance(10km) = " << double(result[0](0, 0))
              << " m\n";

    // Build symbolic initial_bearing function
    SymbolicScalar lat2_sym = sym("lat2");
    SymbolicScalar lon2_sym = sym("lon2");
    LLA<SymbolicScalar> pos2_sym(lon2_sym, lat2_sym, SymbolicScalar(0.0));

    auto bearing_result = initial_bearing(pos_sym, pos2_sym);
    janus::Function bearing_fn("initial_bearing",
                               {lon_sym, lat_sym, lon2_sym, lat2_sym},
                               {bearing_result});

    // Evaluate: London to NYC bearing
    auto bearing_eval = bearing_fn({-0.4543 * constants::angle::deg2rad,
                                    51.4700 * constants::angle::deg2rad,
                                    -73.7781 * constants::angle::deg2rad,
                                    40.6413 * constants::angle::deg2rad});
    std::cout << "Symbolic initial_bearing(LHR→JFK) = "
              << double(bearing_eval[0](0, 0)) * constants::angle::rad2deg
              << "°\n";

    // Generate graph visualization
    std::cout << "  Generating computation graph...\n";
    janus::visualize_graph(horizon_sym, "horizon_distance_graph");
    std::cout << "  Graph saved to 'horizon_distance_graph.dot'\n";

    std::cout << "\n=== Demo Complete ===\n";
    return 0;
}
