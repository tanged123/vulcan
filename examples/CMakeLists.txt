# =============================================================================
# Intro Examples
# =============================================================================
add_executable(getting_started intro/getting_started.cpp)
target_link_libraries(getting_started PRIVATE vulcan)

# =============================================================================
# Coordinate Examples
# =============================================================================
add_executable(coordinate_demo coordinates/coordinate_demo.cpp)
target_link_libraries(coordinate_demo PRIVATE vulcan)

# =============================================================================
# Atmosphere Examples
# =============================================================================
add_executable(atmosphere_profile atmosphere/atmosphere_profile.cpp)
target_link_libraries(atmosphere_profile PRIVATE vulcan)

# =============================================================================
# Time Examples
# =============================================================================
add_executable(time_optimization time/time_optimization.cpp)
target_link_libraries(time_optimization PRIVATE vulcan)

add_executable(mission_planning time/mission_planning.cpp)
target_link_libraries(mission_planning PRIVATE vulcan)

# =============================================================================
# Rotations Examples
# =============================================================================
add_executable(rotations_demo rotations/rotations_demo.cpp)
target_link_libraries(rotations_demo PRIVATE vulcan)

# =============================================================================
# Gravity Examples
# =============================================================================
add_executable(gravity_demo gravity/gravity_demo.cpp)
target_link_libraries(gravity_demo PRIVATE vulcan)

# =============================================================================
# Wind Examples
# =============================================================================
add_executable(wind_optimization wind/wind_optimization.cpp)
target_link_libraries(wind_optimization PRIVATE vulcan)

# =============================================================================
# Aerodynamics Examples
# =============================================================================
add_executable(aerodynamics_demo aerodynamics/aerodynamics_demo.cpp)
target_link_libraries(aerodynamics_demo PRIVATE vulcan)

# =============================================================================
# Integration with Janus Optimization
# =============================================================================
# add_executable(trajectory_opt integration/trajectory_optimization.cpp)
# target_link_libraries(trajectory_opt PRIVATE vulcan)
