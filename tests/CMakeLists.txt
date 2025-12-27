find_package(GTest REQUIRED)

# --- Precompiled Headers ---
# Create a PCH target that other test targets can reuse
# This dramatically speeds up compilation by pre-parsing heavy template headers

# Core tests
add_executable(test_core
    core/test_table_interpolator.cpp
    core/test_vulcan_error.cpp
    core/test_constants.cpp
    core/test_units.cpp
    core/test_validation.cpp
    core/test_symbolic_validation.cpp
)
target_link_libraries(test_core PRIVATE vulcan GTest::gtest_main)
# Create the PCH on the first test target
target_precompile_headers(test_core PRIVATE
    <gtest/gtest.h>
    <Eigen/Dense>
    <vector>
    <string>
    <cmath>
)
gtest_discover_tests(test_core)

# Core tests - coordinates
add_executable(test_coordinates
    coordinates/test_frames.cpp
    coordinates/test_geodetic.cpp
    coordinates/test_transforms.cpp
    coordinates/test_benchmarks.cpp
)
target_link_libraries(test_coordinates PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_coordinates REUSE_FROM test_core)

include(GoogleTest)
gtest_discover_tests(test_coordinates)

# Atmosphere tests
add_executable(test_atmosphere
    atmosphere/test_standard.cpp
    atmosphere/test_exponential.cpp
)
target_link_libraries(test_atmosphere PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_atmosphere REUSE_FROM test_core)
gtest_discover_tests(test_atmosphere)

# Gravity tests
add_executable(test_gravity
    gravity/test_point_mass.cpp
    gravity/test_j2.cpp
    gravity/test_j2j4.cpp
    gravity/test_spherical_harmonics.cpp
)
target_link_libraries(test_gravity PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_gravity REUSE_FROM test_core)
gtest_discover_tests(test_gravity)

# Time tests
add_executable(test_time
    time/test_time.cpp
)
target_link_libraries(test_time PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_time REUSE_FROM test_core)
gtest_discover_tests(test_time)

# Rotations tests
add_executable(test_rotations
    rotations/test_euler_sequences.cpp
    rotations/test_dcm_utils.cpp
    rotations/test_axis_angle.cpp
)
target_link_libraries(test_rotations PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_rotations REUSE_FROM test_core)
gtest_discover_tests(test_rotations)

# Wind tests
add_executable(test_wind
    wind/test_constant_wind.cpp
    wind/test_wind_shear.cpp
    wind/test_dryden.cpp
    wind/test_von_karman.cpp
)
target_link_libraries(test_wind PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_wind REUSE_FROM test_core)
gtest_discover_tests(test_wind)

# Aerodynamics tests
add_executable(test_aerodynamics
    aerodynamics/test_aerodynamics.cpp
)
target_link_libraries(test_aerodynamics PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_aerodynamics REUSE_FROM test_core)
gtest_discover_tests(test_aerodynamics)

# Sensor noise tests
add_executable(test_sensors
    sensors/test_gaussian_noise.cpp
    sensors/test_random_walk.cpp
    sensors/test_bias_instability.cpp
    sensors/test_markov_process.cpp
    sensors/test_allan_variance.cpp
)
target_link_libraries(test_sensors PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_sensors REUSE_FROM test_core)
gtest_discover_tests(test_sensors)

# RNG tests
add_executable(test_rng
    rng/test_rng.cpp
    rng/test_distributions.cpp
)
target_link_libraries(test_rng PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_rng REUSE_FROM test_core)
gtest_discover_tests(test_rng)

# I/O tests
add_executable(test_io
    io/test_telemetry_schema.cpp
    io/test_frame.cpp
    io/test_frame_serializer.cpp
    io/test_hdf5_roundtrip.cpp
    io/test_csv_export.cpp
    io/test_yaml_node.cpp
    io/test_yaml_convert.cpp
    io/test_yaml_file.cpp
    io/test_yaml_env.cpp
)
target_link_libraries(test_io PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_io REUSE_FROM test_core)
gtest_discover_tests(test_io)

# Environment tests
add_executable(test_environment
    environment/test_solar_position.cpp
    environment/test_eclipse.cpp
    environment/test_magnetic_field.cpp
)
target_link_libraries(test_environment PRIVATE vulcan GTest::gtest_main)
gtest_discover_tests(test_environment)

# Geodetic utilities tests
add_executable(test_geodetic_utils
    geodetic/test_geodetic_utils.cpp
)
target_link_libraries(test_geodetic_utils PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_geodetic_utils REUSE_FROM test_core)
gtest_discover_tests(test_geodetic_utils)

# Geometry primitives tests
add_executable(test_geometry
    geometry/test_geometry.cpp
)
target_link_libraries(test_geometry PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_geometry REUSE_FROM test_core)
gtest_discover_tests(test_geometry)

# Orbital mechanics tests
add_executable(test_orbital
    orbital/test_orbital_quantities.cpp
    orbital/test_anomaly_conversions.cpp
    orbital/test_state_conversions.cpp
    orbital/test_analytical_ephemeris.cpp
    orbital/test_transfer_mechanics.cpp
)
target_link_libraries(test_orbital PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_orbital REUSE_FROM test_core)
gtest_discover_tests(test_orbital)

# Propulsion tests
add_executable(test_propulsion
    propulsion/test_rocket.cpp
    propulsion/test_altitude_thrust.cpp
    propulsion/test_air_breathing.cpp
    propulsion/test_electric.cpp
)
target_link_libraries(test_propulsion PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_propulsion REUSE_FROM test_core)
gtest_discover_tests(test_propulsion)

# Transfer functions tests
add_executable(test_transfer_functions
    transfer_functions/test_first_order.cpp
    transfer_functions/test_second_order.cpp
    transfer_functions/test_nonlinear.cpp
    transfer_functions/test_discretize.cpp
)
target_link_libraries(test_transfer_functions PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_transfer_functions REUSE_FROM test_core)
gtest_discover_tests(test_transfer_functions)

# Estimation tests
add_executable(test_estimation
    estimation/test_kalman_filter.cpp
    estimation/test_ekf.cpp
    estimation/test_ukf.cpp
    estimation/test_utils.cpp
)
target_link_libraries(test_estimation PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_estimation REUSE_FROM test_core)
gtest_discover_tests(test_estimation)

# Rigid body dynamics tests
add_executable(test_dynamics
    dynamics/test_mass_properties.cpp
    dynamics/test_rigid_body.cpp
    dynamics/test_point_mass.cpp
    dynamics/test_guided_5dof.cpp
    dynamics/test_oscillator_1dof.cpp
    dynamics/test_rail_launch.cpp
    dynamics/test_planar_2dof.cpp
    dynamics/test_slosh.cpp
)
target_link_libraries(test_dynamics PRIVATE vulcan GTest::gtest_main)
target_precompile_headers(test_dynamics REUSE_FROM test_core)
gtest_discover_tests(test_dynamics)

