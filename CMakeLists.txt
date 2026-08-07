cmake_minimum_required(VERSION 3.28)

project(
  livo_slam
  VERSION 0.1.0
  DESCRIPTION "LiDAR-visual-inertial fusion localization and mapping"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(LIVO_SLAM_BUILD_TESTS "Build the test suite" ON)

include(cmake/dependencies.cmake)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(livo_slam_warnings INTERFACE)
target_compile_options(
  livo_slam_warnings
  INTERFACE
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Wsign-conversion
    -Wnon-virtual-dtor
    -Wold-style-cast)

add_subdirectory(src)

add_executable(livo_slam src/main.cpp)
target_link_libraries(livo_slam PRIVATE livo_slam_ros_lib livo_slam_warnings)

install(TARGETS livo_slam RUNTIME DESTINATION bin)

if(LIVO_SLAM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
