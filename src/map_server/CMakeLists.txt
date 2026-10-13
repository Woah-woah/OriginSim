cmake_minimum_required(VERSION 3.8)
project(map_server)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree io)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Costmap.msg" "msg/DynamicObstacles.msg"
  DEPENDENCIES std_msgs nav_msgs geometry_msgs builtin_interfaces)
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(origin_map_server_core SHARED src/map_geometry.cpp src/costmap_client.cpp
  src/static_layer.cpp src/inflation_layer.cpp src/footprint_layer.cpp src/static_map_loader.cpp
  src/dynamic_memory.cpp src/dynamic_layer.cpp)
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
add_library(origin_dynamic_map_core SHARED src/dynamic_cloud_filter.cpp)
target_compile_features(origin_dynamic_map_core PUBLIC cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(origin_dynamic_map_core PRIVATE "$<$<NOT:$<CONFIG:Debug>>:-O3>")
endif()
target_include_directories(origin_dynamic_map_core SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_include_directories(origin_dynamic_map_core PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(origin_dynamic_map_core PUBLIC origin_map_server_core ${PCL_LIBRARIES})
target_link_libraries(origin_dynamic_map_core PRIVATE ${OpenCV_LIBS})
add_executable(dynamic_map_node src/dynamic_map_node.cpp)
target_link_libraries(dynamic_map_node origin_dynamic_map_core)
ament_target_dependencies(dynamic_map_node rclcpp sensor_msgs nav_msgs pcl_conversions
  ament_index_cpp tf2_ros tf2_geometry_msgs)
foreach(target map_server_node costmap_server_node)
  target_link_libraries(${target} origin_map_server_core)
  ament_target_dependencies(${target} rclcpp nav_msgs tf2 tf2_ros tf2_geometry_msgs)
endforeach()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  foreach(target origin_map_server_core origin_dynamic_map_core map_server_node
    costmap_server_node dynamic_map_node)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endforeach()
endif()

install(TARGETS origin_map_server_core origin_dynamic_map_core EXPORT origin_map_server_core_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS map_server_node costmap_server_node dynamic_map_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_export_targets(origin_map_server_core_targets HAS_LIBRARY_TARGET)
ament_export_libraries(origin_map_server_core)
ament_export_include_directories(include)

ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  tf2
  tf2_ros
  std_msgs
  geometry_msgs
  tf2_geometry_msgs
  nav_msgs
  builtin_interfaces
  PCL
)

ament_package()
