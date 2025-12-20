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
