cmake_minimum_required(VERSION 3.16)
project(minco_controller)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
set(dependencies nav_executor rclcpp map_server geometry_msgs nav_msgs pluginlib
  tf2 tf2_ros tf2_geometry_msgs)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

# Build the same qpOASES 3.2.2 sources supplied by upstream, unchanged.
file(GLOB qpoases_sources CONFIGURE_DEPENDS third_party/qpOASES/src/*.cpp)
add_library(minco_qpoases STATIC ${qpoases_sources})
set_target_properties(minco_qpoases PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(minco_qpoases SYSTEM PUBLIC third_party/qpOASES/include)
target_compile_definitions(minco_qpoases PRIVATE __NO_COPYRIGHT__ LINUX)

add_library(minco_mpc_solver SHARED src/mpc_solver.cpp src/path_trajectory.cpp)
target_compile_features(minco_mpc_solver PUBLIC cxx_std_17)
target_include_directories(minco_mpc_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(minco_mpc_solver PUBLIC Eigen3::Eigen PRIVATE minco_qpoases)

add_library(minco_controller SHARED src/minco_mpc_controller.cpp)
target_link_libraries(minco_controller minco_mpc_solver)
ament_target_dependencies(minco_controller ${dependencies})
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(minco_qpoases PRIVATE -O2)
  target_compile_options(minco_mpc_solver PRIVATE -O2 -Wall -Wextra -Wpedantic)
  target_compile_options(minco_controller PRIVATE -O2 -Wall -Wextra -Wpedantic)
endif()

pluginlib_export_plugin_description_file(nav_executor controller_plugins.xml)
install(TARGETS minco_mpc_solver minco_controller EXPORT minco_controller_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/minco_controller DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
install(FILES third_party/qpOASES/LICENSE third_party/qpOASES/LICENSE.txt
  DESTINATION share/${PROJECT_NAME}/third_party/qpOASES)

ament_export_targets(minco_controller_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Eigen3 ${dependencies})
ament_package()
