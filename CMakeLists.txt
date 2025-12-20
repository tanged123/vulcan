cmake_minimum_required(VERSION 3.20)
project(
  vulcan
  VERSION 0.4.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Generate compile_commands.json for
                                      # clangd

# --- Dependencies ---
find_package(Eigen3 3.4 REQUIRED)
find_package(casadi REQUIRED)
find_package(janus REQUIRED) # Janus as external dependency
find_package(HDF5 REQUIRED COMPONENTS C CXX)
find_package(HighFive REQUIRED) # C++ HDF5 wrapper

# --- Main Library ---
add_library(vulcan INTERFACE)
target_include_directories(
  vulcan INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                   $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# Link Dependencies
target_link_libraries(vulcan INTERFACE Eigen3::Eigen casadi janus::janus
                                       HighFive)

# --- Coverage ---
option(ENABLE_COVERAGE "Enable coverage reporting" OFF)
if(ENABLE_COVERAGE)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
    add_link_options(--coverage)
  else()
    message(WARNING "Coverage not supported for this compiler")
  endif()
endif()

# --- Install ---
include(GNUInstallDirs)

install(
  TARGETS vulcan
  EXPORT vulcanTargets
  INCLUDES
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
  EXPORT vulcanTargets
  FILE vulcanConfig.cmake
  NAMESPACE vulcan::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vulcan)

# --- Testing ---
enable_testing()
add_subdirectory(tests)

# --- Examples ---
add_subdirectory(examples)
