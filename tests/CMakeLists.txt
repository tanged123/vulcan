find_package(GTest REQUIRED)

# Core tests
add_executable(test_core
    core/test_table_interpolator.cpp
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
