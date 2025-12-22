find_package(GTest REQUIRED)

# Core tests
add_executable(test_core
    core/test_table_interpolator.cpp
    core/test_vulcan_error.cpp
)
target_link_libraries(test_core PRIVATE vulcan GTest::gtest_main)
gtest_discover_tests(test_core)

# Core tests - coordinates
add_executable(test_coordinates
    coordinates/test_frames.cpp
    coordinates/test_geodetic.cpp
    coordinates/test_transforms.cpp
    coordinates/test_benchmarks.cpp
)
target_link_libraries(test_coordinates PRIVATE vulcan GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(test_coordinates)

# Atmosphere tests
add_executable(test_atmosphere
    atmosphere/test_standard.cpp
    atmosphere/test_exponential.cpp
)
target_link_libraries(test_atmosphere PRIVATE vulcan GTest::gtest_main)
gtest_discover_tests(test_atmosphere)

# Gravity tests
add_executable(test_gravity
    gravity/test_point_mass.cpp
    gravity/test_j2.cpp
    gravity/test_j2j4.cpp
    gravity/test_spherical_harmonics.cpp
)
target_link_libraries(test_gravity PRIVATE vulcan GTest::gtest_main)
gtest_discover_tests(test_gravity)

# Time tests
add_executable(test_time
    time/test_time.cpp
)
target_link_libraries(test_time PRIVATE vulcan GTest::gtest_main)
gtest_discover_tests(test_time)

# Rotations tests
add_executable(test_rotations
    rotations/test_euler_sequences.cpp
    rotations/test_dcm_utils.cpp
    rotations/test_axis_angle.cpp
)
target_link_libraries(test_rotations PRIVATE vulcan GTest::gtest_main)
gtest_discover_tests(test_rotations)

# Wind tests
add_executable(test_wind
    wind/test_constant_wind.cpp
    wind/test_wind_shear.cpp
    wind/test_dryden.cpp
    wind/test_von_karman.cpp
)
target_link_libraries(test_wind PRIVATE vulcan GTest::gtest_main)
gtest_discover_tests(test_wind)

# Aerodynamics tests
add_executable(test_aerodynamics
    aerodynamics/test_aerodynamics.cpp
)
target_link_libraries(test_aerodynamics PRIVATE vulcan GTest::gtest_main)
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
gtest_discover_tests(test_sensors)

# RNG tests
add_executable(test_rng
    rng/test_rng.cpp
    rng/test_distributions.cpp
)
target_link_libraries(test_rng PRIVATE vulcan GTest::gtest_main)
gtest_discover_tests(test_rng)

# I/O tests
add_executable(test_io
    io/test_telemetry_schema.cpp
    io/test_frame.cpp
    io/test_frame_serializer.cpp
    io/test_hdf5_roundtrip.cpp
    io/test_csv_export.cpp
)
target_link_libraries(test_io PRIVATE vulcan GTest::gtest_main)
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
gtest_discover_tests(test_geodetic_utils)

