cmake_minimum_required(VERSION 3.8)
project(adaptive_lio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set Release Mode as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# Setup include directories
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/thirdparty/sophus
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Add include directories from found packages
get_filename_component(pcl_conversions_INCLUDE_DIRS "${pcl_conversions_DIR}/../../../include" ABSOLUTE)
get_filename_component(message_filters_INCLUDE_DIRS "${message_filters_DIR}/../../../include" ABSOLUTE)
include_directories(${pcl_conversions_INCLUDE_DIRS})
include_directories(${message_filters_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_definitions(${PCL_DEFINITIONS})

# Include additional configurations
include(cmake/packages.cmake)

# Add subdirectories
add_subdirectory(src)

# Export dependencies
ament_export_dependencies(
  livox_ros_driver2
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  PCL
  Eigen3
  Ceres
  glog
  gflags
  yaml-cpp
  pcl_conversions
  message_filters
)


ament_export_include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
)

ament_export_libraries(
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# Install launch files.
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(adaptive_lio_core_test test/test_core.cpp)
  target_include_directories(adaptive_lio_core_test PRIVATE src thirdparty)
  add_test(NAME adaptive_lio_core COMMAND adaptive_lio_core_test)
  add_executable(adaptive_lio_geometry_test test/test_geometry.cpp)
  target_include_directories(adaptive_lio_geometry_test PRIVATE src thirdparty)
  add_test(NAME adaptive_lio_geometry COMMAND adaptive_lio_geometry_test)
  add_executable(adaptive_lio_registration_test test/test_registration.cpp)
  target_include_directories(adaptive_lio_registration_test PRIVATE src thirdparty)
  target_link_libraries(adaptive_lio_registration_test Ceres::ceres)
  add_test(NAME adaptive_lio_registration COMMAND adaptive_lio_registration_test)
  add_executable(adaptive_lio_pipeline_test test/test_pipeline.cpp)
  target_include_directories(adaptive_lio_pipeline_test PRIVATE src thirdparty)
  target_link_libraries(adaptive_lio_pipeline_test ${PROJECT_NAME}.lio)
  add_test(NAME adaptive_lio_pipeline COMMAND adaptive_lio_pipeline_test ${CMAKE_CURRENT_SOURCE_DIR}/config/mapping_m.yaml)
  add_executable(adaptive_lio_imu_quality_test test/test_imu_quality.cpp)
  target_include_directories(adaptive_lio_imu_quality_test PRIVATE src thirdparty)
  add_test(NAME adaptive_lio_imu_quality COMMAND adaptive_lio_imu_quality_test)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
