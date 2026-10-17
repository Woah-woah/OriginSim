cmake_minimum_required(VERSION 3.5)
project(mppi_controller)

add_definitions(-DXTENSOR_ENABLE_XSIMD)
add_definitions(-DXTENSOR_USE_XSIMD)

set(XTENSOR_USE_TBB 0)
set(XTENSOR_USE_OPENMP 0)
set(XTENSOR_USE_XSIMD 1)

# set(XTENSOR_DEFAULT_LAYOUT column_major)  # row_major, column_major
# set(XTENSOR_DEFAULT_TRAVERSAL row_major)  # row_major, column_major

find_package(ament_cmake REQUIRED)
find_package(xsimd REQUIRED)
find_package(xtensor REQUIRED)
find_package(nav_executor REQUIRED)

include_directories(
  include
)

set(dependencies_pkgs
  rclcpp
  pluginlib
  tf2
  geometry_msgs
  visualization_msgs
  nav_msgs
  std_msgs
  angles
  map_server
  tf2_geometry_msgs
  tf2_eigen
  tf2_ros
)

foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-mno-avx512f" COMPILER_SUPPORTS_AVX512)
check_cxx_compiler_flag("-msse4.2" COMPILER_SUPPORTS_SSE4)
check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)

if(COMPILER_SUPPORTS_AVX512)
  add_compile_options(-mno-avx512f)
endif()

if(COMPILER_SUPPORTS_SSE4)
  add_compile_options(-msse4.2)
endif()

if(COMPILER_SUPPORTS_AVX2)
  add_compile_options(-mavx2)
endif()

if(COMPILER_SUPPORTS_FMA)
  add_compile_options(-mfma)
endif()

# If building one the same hardware to be deployed on, try `-march=native`!
add_compile_options(-O3 -finline-limit=10000000 -ffp-contract=fast -ffast-math -mtune=generic)

add_library(mppi_controller SHARED
  src/controller.cpp
  src/optimizer.cpp
  src/critic_manager.cpp
  src/trajectory_visualizer.cpp
  src/path_handler.cpp
  src/parameters_handler.cpp
  src/noise_generator.cpp
)

add_library(mppi_critics SHARED
  src/critics/obstacles_critic.cpp
  src/critics/cost_critic.cpp
  src/critics/goal_critic.cpp
  src/critics/goal_angle_critic.cpp
  src/critics/path_align_critic.cpp
  src/critics/path_follow_critic.cpp
  src/critics/path_angle_critic.cpp
  src/critics/prefer_forward_critic.cpp
  src/critics/twirling_critic.cpp
  src/critics/constraint_critic.cpp
  src/critics/velocity_deadband_critic.cpp
)

set(libraries mppi_controller mppi_critics)

foreach(lib IN LISTS libraries)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    # Clang: use C++20 and optimization, omit -fconcepts
    target_compile_features(${lib} PUBLIC cxx_std_20)
  else()
    target_compile_options(${lib} PUBLIC -fconcepts)
  endif()
  target_include_directories(${lib} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> ${xsimd_INCLUDE_DIRS})  #  ${OpenMP_INCLUDE_DIRS}
  target_link_libraries(${lib} xtensor xtensor::optimize xtensor::use_xsimd)
  ament_target_dependencies(${lib} ${dependencies_pkgs})
endforeach()

# Only the adapter depends on the executor's header-only controller contract.
add_library(mppi_controller_plugin SHARED src/controller_plugin.cpp)
target_link_libraries(mppi_controller_plugin mppi_controller)
ament_target_dependencies(mppi_controller_plugin nav_executor pluginlib)
install(TARGETS mppi_controller_plugin LIBRARY DESTINATION lib)
pluginlib_export_plugin_description_file(nav_executor controller_plugins.xml)

install(TARGETS mppi_controller mppi_critics
  EXPORT mppi_controller_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_targets(mppi_controller_targets HAS_LIBRARY_TARGET)
ament_export_libraries(${libraries})
ament_export_dependencies(${dependencies_pkgs} xtensor xsimd nav_executor)
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(mppi_controller critics.xml)

ament_package()
