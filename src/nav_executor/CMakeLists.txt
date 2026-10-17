cmake_minimum_required(VERSION 3.8)
project(nav_executor)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
set(dependencies rclcpp map_server topology_global_planner
  geometry_msgs nav_msgs std_msgs std_srvs tf2 tf2_geometry_msgs pluginlib)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}_core src/task_manager.cpp src/path_planner.cpp src/path_executor.cpp
  src/route_monitor.cpp src/path_collision.cpp)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_core ${dependencies})
target_link_libraries(${PROJECT_NAME}_core Threads::Threads)

add_executable(nav_executor_node src/nav_executor_node.cpp)
target_link_libraries(nav_executor_node ${PROJECT_NAME}_core)
target_link_libraries(nav_executor_node yaml-cpp)
ament_target_dependencies(nav_executor_node ${dependencies})
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(nav_executor_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS ${PROJECT_NAME}_core nav_executor_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Export the header-only controller contract, without linking plugin providers
# against the executor core or its planner implementation.
ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs nav_msgs map_server)

ament_package()
