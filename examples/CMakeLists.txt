# =============================================================================
# Intro Examples
# =============================================================================
add_executable(getting_started intro/getting_started.cpp)
target_link_libraries(getting_started PRIVATE vulcan)
# Create PCH for examples
target_precompile_headers(getting_started PRIVATE
    <Eigen/Dense>
    <vector>
    <iostream>
    <cmath>
)

# =============================================================================
# Coordinate Examples
# =============================================================================
add_executable(coordinate_demo coordinates/coordinate_demo.cpp)
target_link_libraries(coordinate_demo PRIVATE vulcan)
target_precompile_headers(coordinate_demo REUSE_FROM getting_started)

# =============================================================================
# Atmosphere Examples
# =============================================================================
add_executable(atmosphere_profile atmosphere/atmosphere_profile.cpp)
target_link_libraries(atmosphere_profile PRIVATE vulcan)
target_precompile_headers(atmosphere_profile REUSE_FROM getting_started)

# =============================================================================
# Time Examples
# =============================================================================
add_executable(time_optimization time/time_optimization.cpp)
target_link_libraries(time_optimization PRIVATE vulcan)
target_precompile_headers(time_optimization REUSE_FROM getting_started)

add_executable(mission_planning time/mission_planning.cpp)
target_link_libraries(mission_planning PRIVATE vulcan)
target_precompile_headers(mission_planning REUSE_FROM getting_started)

# =============================================================================
# Rotations Examples
# =============================================================================
add_executable(rotations_demo rotations/rotations_demo.cpp)
target_link_libraries(rotations_demo PRIVATE vulcan)
target_precompile_headers(rotations_demo REUSE_FROM getting_started)

# =============================================================================
# Gravity Examples
# =============================================================================
add_executable(gravity_demo gravity/gravity_demo.cpp)
target_link_libraries(gravity_demo PRIVATE vulcan)
target_precompile_headers(gravity_demo REUSE_FROM getting_started)

# =============================================================================
# Wind Examples
# =============================================================================
add_executable(wind_optimization wind/wind_optimization.cpp)
target_link_libraries(wind_optimization PRIVATE vulcan)
target_precompile_headers(wind_optimization REUSE_FROM getting_started)

# =============================================================================
# Aerodynamics Examples
# =============================================================================
add_executable(aerodynamics_demo aerodynamics/aerodynamics_demo.cpp)
target_link_libraries(aerodynamics_demo PRIVATE vulcan)
target_precompile_headers(aerodynamics_demo REUSE_FROM getting_started)

# =============================================================================
# Sensor Noise Examples
# =============================================================================
add_executable(sensor_noise_demo sensors/sensor_noise_demo.cpp)
target_link_libraries(sensor_noise_demo PRIVATE vulcan)
target_precompile_headers(sensor_noise_demo REUSE_FROM getting_started)

# =============================================================================
# Data I/O Examples
# =============================================================================
add_executable(telemetry_demo io/telemetry_demo.cpp)
target_link_libraries(telemetry_demo PRIVATE vulcan)
target_precompile_headers(telemetry_demo REUSE_FROM getting_started)

# =============================================================================
# Environment Examples
# =============================================================================
add_executable(environment_demo environment/environment_demo.cpp)
target_link_libraries(environment_demo PRIVATE vulcan)

# =============================================================================
# Geodetic Examples
# =============================================================================
add_executable(geodetic_demo geodetic/geodetic_demo.cpp)
target_link_libraries(geodetic_demo PRIVATE vulcan)
target_precompile_headers(geodetic_demo REUSE_FROM getting_started)

# =============================================================================
# Geometry Examples
# =============================================================================
add_executable(geometry_demo geometry/geometry_demo.cpp)
target_link_libraries(geometry_demo PRIVATE vulcan)
target_precompile_headers(geometry_demo REUSE_FROM getting_started)

# =============================================================================
# Orbital Mechanics Examples
# =============================================================================
add_executable(orbital_demo orbital/orbital_demo.cpp)
target_link_libraries(orbital_demo PRIVATE vulcan)
target_precompile_headers(orbital_demo REUSE_FROM getting_started)

add_executable(orbital_optimization orbital/orbital_optimization.cpp)
target_link_libraries(orbital_optimization PRIVATE vulcan)
target_precompile_headers(orbital_optimization REUSE_FROM getting_started)

# =============================================================================
# Propulsion Examples
# =============================================================================
add_executable(propulsion_demo propulsion/propulsion_demo.cpp)
target_link_libraries(propulsion_demo PRIVATE vulcan)
target_precompile_headers(propulsion_demo REUSE_FROM getting_started)

# =============================================================================
# Transfer Functions Examples
# =============================================================================
add_executable(transfer_functions_demo transfer_functions/transfer_functions_demo.cpp)
target_link_libraries(transfer_functions_demo PRIVATE vulcan)
target_precompile_headers(transfer_functions_demo REUSE_FROM getting_started)

# =============================================================================
# State Estimation Examples
# =============================================================================
add_executable(estimation_demo estimation/estimation_demo.cpp)
target_link_libraries(estimation_demo PRIVATE vulcan)
target_precompile_headers(estimation_demo REUSE_FROM getting_started)

# =============================================================================
# Rigid Body Dynamics Examples
# =============================================================================
add_executable(dynamics_demo dynamics/dynamics_demo.cpp)
target_link_libraries(dynamics_demo PRIVATE vulcan)
target_precompile_headers(dynamics_demo REUSE_FROM getting_started)

# =============================================================================
# Integration with Janus Optimization
# =============================================================================
# add_executable(trajectory_opt integration/trajectory_optimization.cpp)
# target_link_libraries(trajectory_opt PRIVATE vulcan)

