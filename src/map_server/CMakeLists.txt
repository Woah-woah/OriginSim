cmake_minimum_required(VERSION 3.8)
project(map_server)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Costmap.msg"
  DEPENDENCIES std_msgs nav_msgs geometry_msgs)
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(origin_map_server_core SHARED src/map_geometry.cpp src/costmap_client.cpp
  src/static_layer.cpp src/inflation_layer.cpp src/footprint_layer.cpp src/static_map_loader.cpp)
target_compile_features(origin_map_server_core PUBLIC cxx_std_17)
target_include_directories(origin_map_server_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(origin_map_server_core PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(origin_map_server_core PUBLIC rclcpp nav_msgs std_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(origin_map_server_core PRIVATE yaml-cpp ${OpenCV_LIBS})
target_link_libraries(origin_map_server_core PUBLIC ${typesupport_target})

add_executable(map_server_node src/static_map_node.cpp)
add_executable(costmap_server_node src/costmap_node.cpp)
foreach(target map_server_node costmap_server_node)
  target_link_libraries(${target} origin_map_server_core)
  ament_target_dependencies(${target} rclcpp nav_msgs tf2 tf2_ros tf2_geometry_msgs)
endforeach()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  foreach(target origin_map_server_core map_server_node costmap_server_node)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endforeach()
endif()

install(TARGETS origin_map_server_core EXPORT origin_map_server_core_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS map_server_node costmap_server_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_export_targets(origin_map_server_core_targets HAS_LIBRARY_TARGET)
ament_export_libraries(origin_map_server_core)
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime rclcpp nav_msgs std_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs)
ament_package()
